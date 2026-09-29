//! `--net bridge:PATH`: the station's Ethernet frames go to the real LAN through a socket_vmnet
//! daemon (lima-vm/socket_vmnet v1.2.2, `--vmnet-mode=bridged`), and the LAN's come back. The
//! home router, not `VirtualNet`, then serves DHCP, ARP and everything else.
//!
//! The wire format both ways is a 4-byte big-endian length followed by the Ethernet frame without
//! its FCS. What the daemon does with it (socket_vmnet v1.2.2 `main.c`, read at the tag):
//! - it reads a client's header with one `read()` of 4 bytes and the body with one `read()` of
//!   that length, and `assert`s that the body came whole (main.c:553-575; the Makefile sets no
//!   `NDEBUG`), so each frame goes out in a single `write()`, header and body together;
//! - its buffer is 64 KiB and a longer header trips an `assert` (main.c:544, :564);
//! - it sends every frame to every client with a blocking `writev()` (main.c:169-194, :603-627),
//!   so one client that stops reading stalls all the others (socket_vmnet issue #173, open). A reader
//!   thread therefore drains the socket continuously into a bounded queue and drops what does not
//!   fit; a writer thread with its own bounded queue keeps a stalled daemon from freezing the
//!   emulation;
//! - it floods: every client sees every frame, and the emulated MAC has no address filter
//!   (esp32s3 `bus.rs`, `wifi_rx_deliver`). The reader passes only frames addressed to the
//!   station, broadcast and multicast.
//! A restarted daemon drops every connection, and restarting it is what people do when the
//! network stops working after the Mac wakes from sleep (issue #130), so a lost connection is
//! retried with a growing delay.

use std::io::{Read, Write};
use std::os::unix::net::UnixStream;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering::Relaxed};
use std::sync::mpsc::{sync_channel, Receiver, SyncSender, TryRecvError, TrySendError};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use super::BridgeStats;

/// The daemon's receive buffer (socket_vmnet v1.2.2 main.c:544): nothing longer may be sent,
/// and nothing longer can honestly come back.
pub const MAX_FRAME: usize = 64 * 1024;
/// The longest frame vmnet takes: its `vmnet_max_packet_size`, which the daemon logs at start
/// ("vmnet_max_packet_size: 1514" on the owner's Mac, /var/log/socket_vmnet/bridged.en0.stderr,
/// 2026-09-29). A longer one fails `vmnet_write` and the daemon then drops the whole connection
/// (main.c:590-593), so it is not sent.
pub const MAX_OUT: usize = 1514;
/// Frames the reader may hold for the emulation, and frames the station may have in flight to the
/// daemon. A burst of LAN broadcast past this is dropped and counted.
pub const RX_QUEUE: usize = 256;
pub const TX_QUEUE: usize = 256;
const BACKOFF_FIRST: Duration = Duration::from_millis(200);
const BACKOFF_MAX: Duration = Duration::from_secs(5);

#[derive(Default)]
struct Counters { rx_ok: AtomicU64, rx_filtered: AtomicU64, rx_dropped: AtomicU64, tx: AtomicU64, tx_dropped: AtomicU64, reconnects: AtomicU64 }

/// The connection the writer uses; `generation` changes with every reconnect.
struct Link { generation: u64, stream: Option<UnixStream> }

struct Shared {
    path: String,
    /// The station's MAC, packed into the low 48 bits: `--mac` at first, then whatever source
    /// address the station's own frames carry.
    station: AtomicU64,
    counters: Counters,
    stop: AtomicBool,
    link: Mutex<Link>,
    log: bool,
}

pub struct Bridge {
    shared: Arc<Shared>,
    rx: Receiver<Vec<u8>>,
    tx: Option<SyncSender<Vec<u8>>>,
}

fn pack(mac: &[u8]) -> u64 { mac.iter().take(6).fold(0, |a, &b| (a << 8) | b as u64) }
fn unpack(v: u64) -> [u8; 6] { let b = v.to_be_bytes(); [b[2], b[3], b[4], b[5], b[6], b[7]] }

/// One frame as the daemon expects it: length and body in one buffer, for one `write()`.
pub fn encode(frame: &[u8]) -> Vec<u8> {
    let mut b = Vec::with_capacity(4 + frame.len());
    b.extend_from_slice(&(frame.len() as u32).to_be_bytes());
    b.extend_from_slice(frame);
    b
}

/// Send one frame: header and body in one buffer, handed to a single `write()`.
pub fn write_frame(w: &mut impl Write, frame: &[u8]) -> std::io::Result<()> { w.write_all(&encode(frame)) }

/// Read one frame. `Ok(None)` is a clean end of stream; a length past `MAX_FRAME` means the
/// stream is out of step and is reported as an error so the caller reconnects.
pub fn read_frame(r: &mut impl Read) -> std::io::Result<Option<Vec<u8>>> {
    let mut hdr = [0u8; 4];
    match r.read_exact(&mut hdr) {
        Ok(()) => {}
        Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => return Ok(None),
        Err(e) => return Err(e),
    }
    let len = u32::from_be_bytes(hdr) as usize;
    if len > MAX_FRAME { return Err(std::io::Error::new(std::io::ErrorKind::InvalidData, format!("frame length {len} past {MAX_FRAME}"))); }
    let mut body = vec![0u8; len];
    match r.read_exact(&mut body) {
        Ok(()) => Ok(Some(body)),
        Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => Ok(None),
        Err(e) => Err(e),
    }
}

/// Does a frame from the LAN belong to the station? Its own MAC, broadcast and multicast (the
/// group bit, which broadcast also has). The station's own frames never come back to it.
pub fn for_station(frame: &[u8], station: &[u8; 6]) -> bool {
    if frame.len() < 14 { return false; }
    if frame[6..12] == station[..] { return false; }
    frame[0] & 1 != 0 || frame[0..6] == station[..]
}

impl Bridge {
    /// Connect to the daemon's socket, or say why not. Later losses are retried in the background.
    pub fn connect(path: &str, mac: [u8; 6], log: bool) -> Result<Self, String> {
        let stream = UnixStream::connect(path).map_err(|e| format!("cannot connect to {path}: {e}"))?;
        Self::start(stream, path, mac, log)
    }

    /// Run the bridge over a connected stream; `path` is where to reconnect when it ends.
    pub fn start(stream: UnixStream, path: &str, mac: [u8; 6], log: bool) -> Result<Self, String> {
        let writer_half = stream.try_clone().map_err(|e| format!("{path}: {e}"))?;
        let shared = Arc::new(Shared {
            path: path.to_string(), station: AtomicU64::new(pack(&mac)), counters: Counters::default(),
            stop: AtomicBool::new(false), link: Mutex::new(Link { generation: 1, stream: Some(writer_half) }), log,
        });
        let (rx_send, rx) = sync_channel(RX_QUEUE);
        let (tx, tx_recv) = sync_channel::<Vec<u8>>(TX_QUEUE);
        let s = shared.clone();
        std::thread::Builder::new().name("vmnet-reader".into()).spawn(move || reader(s, stream, rx_send)).map_err(|e| e.to_string())?;
        let s = shared.clone();
        std::thread::Builder::new().name("vmnet-writer".into()).spawn(move || writer(s, tx_recv)).map_err(|e| e.to_string())?;
        Ok(Bridge { shared, rx, tx: Some(tx) })
    }

    pub fn path(&self) -> &str { &self.shared.path }
    pub fn station(&self) -> [u8; 6] { unpack(self.shared.station.load(Relaxed)) }

    /// Queue one frame from the station for the LAN. Never blocks: a full queue drops the frame.
    pub fn send(&self, eth: &[u8]) {
        let c = &self.shared.counters;
        if eth.len() < 14 || eth.len() > MAX_OUT { c.tx_dropped.fetch_add(1, Relaxed); return; }
        if eth[6] & 1 == 0 { self.shared.station.store(pack(&eth[6..12]), Relaxed); }   // learn the station's MAC
        let Some(tx) = &self.tx else { c.tx_dropped.fetch_add(1, Relaxed); return };
        match tx.try_send(eth.to_vec()) {
            Ok(()) => {}
            Err(TrySendError::Full(_)) | Err(TrySendError::Disconnected(_)) => { c.tx_dropped.fetch_add(1, Relaxed); }
        }
    }

    /// Up to `max` frames the LAN sent the station since the last call.
    pub fn recv(&self, max: usize) -> Vec<Vec<u8>> {
        let mut v = Vec::new();
        while v.len() < max {
            match self.rx.try_recv() { Ok(f) => v.push(f), Err(TryRecvError::Empty) | Err(TryRecvError::Disconnected) => break }
        }
        v
    }

    pub fn stats(&self) -> BridgeStats {
        let c = &self.shared.counters;
        BridgeStats { rx_ok: c.rx_ok.load(Relaxed), rx_filtered: c.rx_filtered.load(Relaxed), rx_dropped: c.rx_dropped.load(Relaxed),
                      tx: c.tx.load(Relaxed), tx_dropped: c.tx_dropped.load(Relaxed), reconnects: c.reconnects.load(Relaxed) }
    }
}

impl Drop for Bridge {
    fn drop(&mut self) {
        self.shared.stop.store(true, Relaxed);
        self.tx = None;                                                     // the writer's queue closes
        if let Some(s) = self.shared.link.lock().unwrap().stream.take() { let _ = s.shutdown(std::net::Shutdown::Both); }
    }
}

/// Wait `d`, but give up at once when the bridge is dropped. Returns false then.
fn pause(shared: &Shared, d: Duration) -> bool {
    let step = Duration::from_millis(20);
    let mut left = d;
    while !left.is_zero() {
        if shared.stop.load(Relaxed) { return false; }
        let s = left.min(step); std::thread::sleep(s); left -= s;
    }
    !shared.stop.load(Relaxed)
}

fn reader(shared: Arc<Shared>, mut stream: UnixStream, out: SyncSender<Vec<u8>>) {
    let c = &shared.counters;
    loop {
        // drain this connection until it ends
        let why = loop {
            match read_frame(&mut stream) {
                Ok(Some(f)) => {
                    if !for_station(&f, &unpack(shared.station.load(Relaxed))) { c.rx_filtered.fetch_add(1, Relaxed); continue; }
                    match out.try_send(f) {
                        Ok(()) => { c.rx_ok.fetch_add(1, Relaxed); }
                        Err(TrySendError::Full(_)) => { c.rx_dropped.fetch_add(1, Relaxed); }
                        Err(TrySendError::Disconnected(_)) => return,     // the bridge is gone
                    }
                }
                Ok(None) => break "the daemon closed the connection".to_string(),
                Err(e) => break e.to_string(),
            }
        };
        if shared.stop.load(Relaxed) { return; }
        { let mut l = shared.link.lock().unwrap(); if let Some(s) = l.stream.take() { let _ = s.shutdown(std::net::Shutdown::Both); } }
        eprintln!("[bridge] {}: {}; reconnecting", shared.path, why);
        // come back with a growing delay: launchd restarts the daemon, which takes a moment
        let mut delay = BACKOFF_FIRST;
        stream = loop {
            if !pause(&shared, delay) { return; }
            match UnixStream::connect(&shared.path).and_then(|s| s.try_clone().map(|w| (s, w))) {
                Ok((s, w)) => {
                    let mut l = shared.link.lock().unwrap();
                    l.generation += 1; l.stream = Some(w);
                    c.reconnects.fetch_add(1, Relaxed);
                    eprintln!("[bridge] {}: connected again", shared.path);
                    break s;
                }
                Err(e) => { if shared.log { eprintln!("[bridge] {}: {} (next try in {:?})", shared.path, e, delay); } delay = (delay * 2).min(BACKOFF_MAX); }
            }
        };
    }
}

fn writer(shared: Arc<Shared>, frames: Receiver<Vec<u8>>) {
    let c = &shared.counters;
    let mut current: (u64, Option<UnixStream>) = (0, None);
    for f in frames {
        {
            let l = shared.link.lock().unwrap();
            if l.generation != current.0 { current = (l.generation, l.stream.as_ref().and_then(|s| s.try_clone().ok())); }
        }
        let Some(s) = &mut current.1 else { c.tx_dropped.fetch_add(1, Relaxed); continue };
        // one write for header and body: the daemon reads each with a single read()
        match write_frame(s, &f) {
            Ok(()) => { c.tx.fetch_add(1, Relaxed); }
            Err(e) => {
                c.tx_dropped.fetch_add(1, Relaxed);
                if shared.log { eprintln!("[bridge] {}: write: {}", shared.path, e); }
                // EPIPE and friends: end the connection, and the reader will make a new one
                let _ = s.shutdown(std::net::Shutdown::Both);
                current.1 = None;
            }
        }
    }
}

#[cfg(test)]
mod tests;
