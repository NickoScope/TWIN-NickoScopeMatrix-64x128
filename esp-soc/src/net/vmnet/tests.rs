//! The bridge against a stand-in for the socket_vmnet daemon that reads and writes the way
//! v1.2.2's main.c does: one read() for the 4-byte header, one for the body, and one write
//! (writev) per frame going the other way.

use super::*;
use std::os::unix::net::UnixListener;
use std::time::Instant;

const STATION: [u8; 6] = [0x02, 0x54, 0x57, 0x49, 0x4e, 0x01];
const OTHER: [u8; 6] = [0x02, 0x11, 0x22, 0x33, 0x44, 0x55];

fn frame(da: [u8; 6], sa: [u8; 6], len: usize) -> Vec<u8> {
    let mut f = Vec::with_capacity(len);
    f.extend_from_slice(&da); f.extend_from_slice(&sa); f.extend_from_slice(&[0x08, 0x00]);
    while f.len() < len { f.push(f.len() as u8); }
    f
}

/// What main.c:553-575 does with a client's frame: exactly one read() each, and an assert.
fn daemon_read(s: &mut UnixStream) -> Vec<u8> {
    let mut hdr = [0u8; 4];
    assert_eq!(s.read(&mut hdr).unwrap(), 4, "the header came in pieces");
    let len = u32::from_be_bytes(hdr) as usize;
    assert!(len <= MAX_FRAME, "main.c:564 would abort on {len}");
    let mut body = vec![0u8; MAX_FRAME];
    let n = s.read(&mut body[..len]).unwrap();
    assert_eq!(n, len, "main.c:575 would abort: the body came in pieces");
    body.truncate(len);
    body
}

/// What main.c:175-186 does towards a client: header and frame in one writev().
fn daemon_write(s: &mut UnixStream, f: &[u8]) { s.write_all(&encode(f)).unwrap(); }

fn wait_for(what: &str, cond: impl Fn() -> bool) {
    let t = Instant::now();
    while !cond() {
        assert!(t.elapsed() < Duration::from_secs(10), "timed out waiting for {what}");
        std::thread::sleep(Duration::from_millis(5));
    }
}

fn pair() -> (Bridge, UnixStream) {
    let (ours, daemon) = UnixStream::pair().unwrap();
    (Bridge::start(ours, "/nonexistent/socket_vmnet", STATION, false).unwrap(), daemon)
}

#[test]
fn a_frame_is_one_write_length_first() {
    struct Writes(Vec<Vec<u8>>);
    impl Write for Writes {
        fn write(&mut self, b: &[u8]) -> std::io::Result<usize> { self.0.push(b.to_vec()); Ok(b.len()) }
        fn flush(&mut self) -> std::io::Result<()> { Ok(()) }
    }
    let f = frame(OTHER, STATION, 1514);
    let mut w = Writes(Vec::new());
    write_frame(&mut w, &f).unwrap();
    assert_eq!(w.0.len(), 1, "header and body must leave in one write()");
    assert_eq!(&w.0[0][..4], &[0, 0, 0x05, 0xea]);
    assert_eq!(&w.0[0][4..], &f[..]);
}

#[test]
fn frames_cross_in_the_daemons_framing_both_ways() {
    let (b, mut d) = pair();
    for len in [60, 342, 1514] {
        let f = frame([0xff; 6], STATION, len);
        b.send(&f);
        assert_eq!(daemon_read(&mut d), f);
    }
    let back = frame(STATION, OTHER, 590);
    daemon_write(&mut d, &back);
    wait_for("the frame from the LAN", || b.stats().rx_ok == 1);
    assert_eq!(b.recv(8), vec![back]);
    assert_eq!(b.stats(), BridgeStats { rx_ok: 1, tx: 3, ..Default::default() });
}

#[test]
fn nothing_past_64_kib_goes_out_or_is_believed_coming_in() {
    let mut ok = encode(&vec![0u8; MAX_FRAME]);
    assert_eq!(read_frame(&mut &ok[..]).unwrap().unwrap().len(), MAX_FRAME);
    ok[..4].copy_from_slice(&(MAX_FRAME as u32 + 1).to_be_bytes());
    assert_eq!(read_frame(&mut &ok[..]).unwrap_err().kind(), std::io::ErrorKind::InvalidData);
    assert!(read_frame(&mut &[0u8, 0][..]).unwrap().is_none(), "a short header is the end of the stream");

    // vmnet itself takes no more than 1514 bytes, and a failed vmnet_write costs the connection
    let (b, mut d) = pair();
    b.send(&frame(OTHER, STATION, MAX_FRAME + 1));
    b.send(&frame(OTHER, STATION, MAX_OUT + 1));
    b.send(&frame(OTHER, STATION, 13)[..13]);
    b.send(&frame(OTHER, STATION, MAX_OUT));
    assert_eq!(daemon_read(&mut d).len(), MAX_OUT);
    assert_eq!((b.stats().tx, b.stats().tx_dropped), (1, 3));
}

#[test]
fn only_the_station_broadcast_and_multicast_come_through() {
    assert!(for_station(&frame(STATION, OTHER, 60), &STATION));
    assert!(for_station(&frame([0xff; 6], OTHER, 60), &STATION));
    assert!(for_station(&frame([0x01, 0x00, 0x5e, 0x00, 0x00, 0xfb], OTHER, 60), &STATION));   // mDNS
    assert!(for_station(&frame([0x33, 0x33, 0x00, 0x00, 0x00, 0x01], OTHER, 60), &STATION));   // IPv6 all-nodes
    assert!(!for_station(&frame(OTHER, [0x02, 9, 9, 9, 9, 9], 60), &STATION));                 // someone else's
    assert!(!for_station(&frame([0xff; 6], STATION, 60), &STATION));                           // our own, echoed
    assert!(!for_station(&frame(STATION, OTHER, 60)[..13], &STATION));                          // runt

    // The MAC is learned from what the station sends; --mac only covers the time before that.
    let (b, mut d) = pair();
    let learned = [0x02, 0x54, 0x57, 0x49, 0x4e, 0x02];
    b.send(&frame([0xff; 6], learned, 342));
    daemon_read(&mut d);
    assert_eq!(b.station(), learned);
    for f in [frame(learned, OTHER, 60), frame(STATION, OTHER, 60), frame(OTHER, [0x02, 9, 9, 9, 9, 9], 60), frame([0xff; 6], OTHER, 60)] {
        daemon_write(&mut d, &f);
    }
    wait_for("four frames", || { let s = b.stats(); s.rx_ok + s.rx_filtered == 4 });
    assert_eq!((b.stats().rx_ok, b.stats().rx_filtered), (2, 2));
    let got = b.recv(8);
    assert_eq!((got[0][..6].to_vec(), got[1][..6].to_vec()), (learned.to_vec(), vec![0xff; 6]));
}

#[test]
fn the_daemon_never_waits_on_a_slow_emulation() {
    // socket_vmnet #173: a client that stops reading blocks the daemon's writev for every client.
    // The emulation takes nothing here, yet every write completes; the excess is dropped and counted.
    let (b, mut d) = pair();
    let total = RX_QUEUE + 300;       // well past the 8 KiB a Unix socket buffers on macOS
    let (done, finished) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        for i in 0..total { let mut f = frame([0xff; 6], OTHER, 100); f[20] = i as u8; daemon_write(&mut d, &f); }
        done.send(d).unwrap();
    });
    let _d = finished.recv_timeout(Duration::from_secs(10)).expect("the daemon blocked writing to the bridge");
    wait_for("the reader to take every frame", || { let s = b.stats(); s.rx_ok + s.rx_dropped == total as u64 });
    assert_eq!(b.stats().rx_ok, RX_QUEUE as u64);
    assert_eq!(b.stats().rx_dropped, 300);
    assert_eq!(b.recv(usize::MAX).len(), RX_QUEUE);
    assert!(b.recv(usize::MAX).is_empty());
}

#[test]
fn a_poll_takes_a_bounded_batch() {
    let (b, mut d) = pair();
    for _ in 0..10 { daemon_write(&mut d, &frame([0xff; 6], OTHER, 60)); }
    wait_for("ten frames", || b.stats().rx_ok == 10);
    assert_eq!(b.recv(4).len(), 4);
    assert_eq!(b.recv(100).len(), 6);
}

#[test]
fn it_reconnects_when_the_daemon_goes_away() {
    let path = std::env::temp_dir().join(format!("esp32sim-vmnet-{}.sock", std::process::id()));
    let _ = std::fs::remove_file(&path);
    let listener = UnixListener::bind(&path).unwrap();
    let b = Bridge::connect(path.to_str().unwrap(), STATION, false).unwrap();
    let (first, _) = listener.accept().unwrap();
    drop(first);                                   // the daemon restarts (sleep, Wi-Fi reconnect: #130)
    let (mut second, _) = listener.accept().unwrap();
    wait_for("the reconnect to count", || b.stats().reconnects == 1);
    let out = frame([0xff; 6], STATION, 342);
    b.send(&out);
    assert_eq!(daemon_read(&mut second), out, "the writer moved to the new connection");
    daemon_write(&mut second, &frame(STATION, OTHER, 60));
    wait_for("a frame on the new connection", || b.stats().rx_ok == 1);
    drop(b);
    let _ = std::fs::remove_file(&path);
}

#[test]
fn no_daemon_is_a_clear_error() {
    let e = Bridge::connect("/nonexistent/socket_vmnet.bridged.en0", STATION, false).err().unwrap();
    assert!(e.contains("/nonexistent/socket_vmnet.bridged.en0"), "{e}");
    let e = crate::net::vmnet_stub::Bridge::connect("/x", STATION, false).err().unwrap();
    assert!(e.contains("Unix host"), "{e}");
}
