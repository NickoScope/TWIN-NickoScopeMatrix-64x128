//! User-mode NAT: guest TCP and UDP flows are relayed through ordinary host sockets.
//!
//! TCP uses bounded buffers in both directions. SYN, data and FIN share one retransmission queue,
//! so a dropped virtual-air frame cannot discard acknowledged guest bytes or strand a handshake.
//! This is a small relay, without window scaling, SACK or congestion control.
//!
//! Host ports can also be forwarded inward (`--hostfwd`): an accepted host connection becomes a
//! flow the relay opens toward the leased station, from the gateway address, and from then on it
//! is the same `Tcp` flow as an outbound one. A forwarded UDP port delivers datagrams from the
//! gateway and sends the guest's answers back to the last host peer.

use std::collections::VecDeque;
use std::io::{self, ErrorKind, Read, Write};
use std::net::{IpAddr, Ipv4Addr, SocketAddr, TcpListener, TcpStream, UdpSocket};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::mpsc::{channel, Receiver, TryRecvError};
use crate::net::packet::{ethernet, ip_packet, transport_checksum, udp_packet, GATEWAY_MAC};

const MSS: usize = 1400;
const WINDOW: u16 = 5840;
const RETRANSMIT_US: u64 = 300_000;
const IDLE_CLOSE_US: u64 = 120_000_000;
const TIME_WAIT_US: u64 = 30_000_000;
const MAX_FLOWS: usize = 64;
const MAX_CONNECTS: usize = 8;
const UDP_BATCH: usize = 16;
/// Host connections taken from one forwarded port per poll; the rest wait in the host's backlog.
const MAX_ACCEPTS: usize = 8;
/// How long a forwarded SYN may go unanswered. lwIP drops a SYN silently while the listener's
/// backlog is full (tcp_in.c, tcp_listen_input), so a busy single-client server is waited for.
const FORWARD_SYN_TIMEOUT_US: u64 = 30_000_000;
/// Gateway-side ports of forwarded TCP flows: the dynamic range of RFC 6335 section 6.
const EPHEMERAL: u16 = 49152;
/// The largest UDP payload one unfragmented IPv4 packet carries over a 1500-byte MTU.
const UDP_MAX_PAYLOAD: usize = 1500 - 20 - 8;
const FIN: u8 = 0x01;
const SYN: u8 = 0x02;
const RST: u8 = 0x04;
const ACK: u8 = 0x10;
const PSH: u8 = 0x08;

fn ip(a: &[u8; 4]) -> Ipv4Addr { Ipv4Addr::from(*a) }
fn address(a: &[u8; 4], port: u16) -> SocketAddr { SocketAddr::new(IpAddr::V4(ip(a)), port) }

/// A host port forwarded into the guest: `--hostfwd tcp:8080-80`. The host side is always bound
/// on 127.0.0.1, so the guest is reachable from this machine only.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct HostFwd { pub udp: bool, pub host_port: u16, pub guest_port: u16 }

impl HostFwd {
    /// `tcp:HOSTPORT-GUESTPORT` or `udp:HOSTPORT-GUESTPORT`.
    pub fn parse(spec: &str) -> Result<Self, String> {
        let form = || format!("{spec:?}: expected tcp:HOSTPORT-GUESTPORT or udp:HOSTPORT-GUESTPORT");
        let (proto, ports) = spec.split_once(':').ok_or_else(form)?;
        let udp = match proto { "tcp" => false, "udp" => true, _ => return Err(form()) };
        let (host, guest) = ports.split_once('-').ok_or_else(form)?;
        let port = |p: &str| p.parse::<u16>().ok().filter(|&n| n != 0).ok_or_else(|| format!("{spec:?}: port {p:?} is not 1-65535"));
        Ok(Self { udp, host_port: port(host)?, guest_port: port(guest)? })
    }
}

impl std::fmt::Display for HostFwd {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}:{}-{}", if self.udp { "udp" } else { "tcp" }, self.host_port, self.guest_port)
    }
}

/// Where forwarded traffic enters the guest: the station holding the DHCP lease, and the gateway
/// address that forwarded connections and datagrams come from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Station { pub mac: [u8; 6], pub ip: [u8; 4], pub gateway: [u8; 4] }

enum Listener { Tcp(TcpListener), Udp { sock: UdpSocket, peer: Option<SocketAddr> } }
struct Forward { rule: HostFwd, listener: Listener }

// A permit belongs to the blocking connect worker, not its guest flow. Removing a flow with RST
// must not free a slot while connect_timeout is still running on the host.
static CONNECTING: AtomicUsize = AtomicUsize::new(0);
struct ConnectPermit<'a>(&'a AtomicUsize);
impl<'a> ConnectPermit<'a> {
    fn acquire(counter: &'a AtomicUsize) -> Option<Self> {
        counter.fetch_update(Ordering::Relaxed, Ordering::Relaxed, |n| (n < MAX_CONNECTS).then_some(n + 1)).ok().map(|_| Self(counter))
    }
}
impl Drop for ConnectPermit<'_> {
    fn drop(&mut self) { self.0.fetch_sub(1, Ordering::Relaxed); }
}

fn connect(addr: SocketAddr) -> Option<Receiver<io::Result<TcpStream>>> {
    let permit = ConnectPermit::acquire(&CONNECTING)?;
    let (tx, rx) = channel();
    std::thread::Builder::new().name("nat-connect".into()).spawn(move || {
        let _permit = permit;
        let result = TcpStream::connect_timeout(&addr, std::time::Duration::from_secs(4))
            .and_then(|sock| { sock.set_nonblocking(true)?; sock.set_nodelay(true)?; Ok(sock) });
        let _ = tx.send(result);
    }).ok()?;
    Some(rx)
}

enum Transport { Connecting(Receiver<io::Result<TcpStream>>), Connected(TcpStream), TimeWait, Closed }
#[derive(Clone, Copy, PartialEq, Eq)]
enum GuestWrite { Open, Draining, Closed }

struct Sent {
    seq: u32,
    flags: u8,
    data: Vec<u8>,
    at_us: u64,
}
impl Sent {
    fn len(&self) -> usize { self.data.len() + usize::from(self.flags & (SYN | FIN) != 0) }
}

struct Tcp {
    guest_mac: [u8; 6], guest_ip: [u8; 4], guest_port: u16, dst_ip: [u8; 4], dst_port: u16,
    transport: Transport,
    /// opened by the relay toward the guest (a forwarded host connection), not by a guest SYN
    inbound: bool,
    guest_write: GuestWrite,
    host_closed: bool,
    our_seq: u32,
    guest_seq: u32,
    guest_window: u16,
    to_host: VecDeque<u8>,
    unacked: VecDeque<Sent>,
    last_activity_us: u64,
}

impl Tcp {
    fn handshake_pending(&self) -> bool { self.unacked.front().is_some_and(|s| s.flags & SYN != 0) }
    fn in_flight(&self) -> usize { self.unacked.iter().map(Sent::len).sum() }

    fn acknowledge(&mut self, ack: u32) {
        let Some(first) = self.unacked.front() else { return };
        let mut count = ack.wrapping_sub(first.seq) as usize;
        if count > self.in_flight() { return; } // old or beyond anything sent
        while count > 0 {
            let sent = self.unacked.front_mut().unwrap();
            let n = sent.len();
            if count < n {
                sent.data.drain(..count);
                sent.seq = ack;
                break;
            }
            count -= n;
            self.unacked.pop_front();
        }
    }

    fn accept(&mut self, seq: u32, data: &[u8], fin: bool) {
        if self.guest_write != GuestWrite::Open || seq != self.guest_seq { return; }
        if data.len() > WINDOW as usize - self.to_host.len() { return; }
        self.to_host.extend(data);
        self.guest_seq = self.guest_seq.wrapping_add(data.len() as u32);
        if fin {
            self.guest_seq = self.guest_seq.wrapping_add(1);
            self.guest_write = GuestWrite::Draining;
        }
    }

    fn segment(&self, flags: u8, data: &[u8], seq: u32) -> Vec<u8> {
        let mut seg = Vec::with_capacity(20 + data.len());
        seg.extend_from_slice(&self.dst_port.to_be_bytes());
        seg.extend_from_slice(&self.guest_port.to_be_bytes());
        seg.extend_from_slice(&seq.to_be_bytes());
        seg.extend_from_slice(&self.guest_seq.to_be_bytes());
        seg.extend_from_slice(&[0x50, flags]);
        seg.extend_from_slice(&(WINDOW - self.to_host.len() as u16).to_be_bytes());
        seg.extend_from_slice(&[0; 4]);
        seg.extend_from_slice(data);
        let check = transport_checksum(&self.dst_ip, &self.guest_ip, 6, &seg);
        seg[16..18].copy_from_slice(&check.to_be_bytes());
        ethernet(&self.guest_mac, &GATEWAY_MAC, 0x0800, &ip_packet(6, &self.dst_ip, &self.guest_ip, &seg))
    }

    fn send(&mut self, flags: u8, data: Vec<u8>, now_us: u64) -> Vec<u8> {
        let frame = self.segment(flags, &data, self.our_seq);
        let sent = Sent { seq: self.our_seq, flags, data, at_us: now_us };
        self.our_seq = self.our_seq.wrapping_add(sent.len() as u32);
        self.unacked.push_back(sent);
        frame
    }

    fn retransmit(&mut self, now_us: u64) -> Option<Vec<u8>> {
        let sent = self.unacked.front_mut()?;
        if now_us.wrapping_sub(sent.at_us) < RETRANSMIT_US { return None; }
        sent.at_us = now_us;
        let sent = self.unacked.front().unwrap();
        Some(self.segment(sent.flags, &sent.data, sent.seq))
    }

    fn closed(&self) -> bool {
        matches!(self.transport, Transport::Closed)
    }

    /// The guest's SYN-ACK to a forwarded connection (RFC 9293 section 3.10.7.3, SYN-SENT): one
    /// that acknowledges our SYN fixes the guest's sequence space; one that does not is answered
    /// with a reset carrying its ACK as the sequence number. A repeated SYN-ACK means our ACK was
    /// lost, so it is acknowledged again.
    fn syn_ack(&mut self, seq: u32, ack: u32, flags: u8, window: u16) -> Vec<Vec<u8>> {
        if flags & ACK == 0 { return Vec::new(); } // a simultaneous open is not what a server does
        if self.handshake_pending() {
            if ack != self.our_seq { return vec![self.segment(RST, &[], ack)]; }
            self.guest_seq = seq.wrapping_add(1);
            self.guest_window = window;
            self.acknowledge(ack);
        }
        vec![self.segment(ACK, &[], self.our_seq)]
    }
}

/// Preserve every unwritten byte across short writes and WouldBlock. Returns bytes actually sent.
fn flush_pending(writer: &mut impl Write, pending: &mut VecDeque<u8>) -> io::Result<usize> {
    let mut written = 0;
    while !pending.is_empty() {
        match writer.write(pending.as_slices().0) {
            Ok(0) => return Err(ErrorKind::WriteZero.into()),
            Ok(n) => { pending.drain(..n); written += n; }
            Err(e) if e.kind() == ErrorKind::Interrupted => continue,
            Err(e) if e.kind() == ErrorKind::WouldBlock => break,
            Err(e) => return Err(e),
        }
    }
    Ok(written)
}

struct Udp {
    guest_mac: [u8; 6], guest_ip: [u8; 4], guest_port: u16, dst_ip: [u8; 4], dst_port: u16,
    reply_src: [u8; 4], // DNS's guest-visible address, before host resolver redirection
    sock: UdpSocket,
    last_activity_us: u64,
}

pub struct Nat {
    tcp: Vec<Tcp>,
    udp: Vec<Udp>,
    forwards: Vec<Forward>,
    isn: u32,
    next_port: u16,
    pub resolver: [u8; 4],
    /// the leased station; forwarded traffic waits on the host side until there is one
    pub station: Option<Station>,
    pub log: bool,
    pub tcp_opened: u64, pub tcp_refused: u64, pub udp_flows: u64,
    pub udp_evicted: u64, pub udp_send_errors: u64,
    pub bytes_to_host: u64, pub bytes_to_guest: u64,
    /// forwarded host connections opened toward the guest, and those it refused or never answered
    pub fwd_tcp: u64, pub fwd_refused: u64,
    /// forwarded datagrams delivered to the guest, and guest answers sent back to a host peer
    pub fwd_udp_in: u64, pub fwd_udp_out: u64,
}

impl Nat {
    pub fn new(log: bool) -> Self {
        Self { tcp: Vec::new(), udp: Vec::new(), forwards: Vec::new(), isn: 0x1000, next_port: EPHEMERAL,
            resolver: host_resolver(), station: None, log,
            tcp_opened: 0, tcp_refused: 0, udp_flows: 0, udp_evicted: 0, udp_send_errors: 0, bytes_to_host: 0, bytes_to_guest: 0,
            fwd_tcp: 0, fwd_refused: 0, fwd_udp_in: 0, fwd_udp_out: 0 }
    }

    /// Listen on 127.0.0.1:`host_port` for the guest's `guest_port`. Returns the bound address
    /// (port 0 picks one, which the rule then carries as its host port).
    pub fn forward(&mut self, mut rule: HostFwd) -> io::Result<SocketAddr> {
        let at = (Ipv4Addr::LOCALHOST, rule.host_port);
        let (listener, local) = if rule.udp {
            let sock = UdpSocket::bind(at)?;
            sock.set_nonblocking(true)?;
            let local = sock.local_addr()?;
            (Listener::Udp { sock, peer: None }, local)
        } else {
            let listener = TcpListener::bind(at)?;
            listener.set_nonblocking(true)?;
            let local = listener.local_addr()?;
            (Listener::Tcp(listener), local)
        };
        rule.host_port = local.port();
        self.forwards.push(Forward { rule, listener });
        Ok(local)
    }

    /// A gateway-side port no live flow uses, walking the dynamic range.
    fn ephemeral_port(&mut self, gateway: &[u8; 4]) -> u16 {
        loop {
            let port = self.next_port;
            self.next_port = port.checked_add(1).unwrap_or(EPHEMERAL);
            if !self.tcp.iter().any(|c| c.dst_ip == *gateway && c.dst_port == port) { return port; }
        }
    }

    /// Take waiting host connections and datagrams on forwarded ports into the guest. Without a
    /// lease, or with the flow table full of live flows, they stay queued on the host side.
    fn accept_forwards(&mut self, now_us: u64, out: &mut Vec<Vec<u8>>) {
        let Some(st) = self.station else { return };
        for f in 0..self.forwards.len() {
            let rule = self.forwards[f].rule;
            if let Listener::Udp { sock, peer } = &mut self.forwards[f].listener {
                let mut buf = [0; 2048];
                for _ in 0..UDP_BATCH {
                    let Ok((n, from)) = sock.recv_from(&mut buf) else { break };
                    if n > UDP_MAX_PAYLOAD {
                        if self.log { eprintln!("[nat] hostfwd {} dropped a {}-byte datagram (larger than one unfragmented packet)", rule, n); }
                        continue;
                    }
                    *peer = Some(from);
                    if self.log { eprintln!("[nat] hostfwd {} {} bytes from {} -> guest", rule, n, from); }
                    self.fwd_udp_in += 1;
                    self.bytes_to_guest += n as u64;
                    let udp = udp_packet(&st.gateway, &st.ip, rule.host_port, rule.guest_port, &buf[..n]);
                    out.push(ethernet(&st.mac, &GATEWAY_MAC, 0x0800, &ip_packet(17, &st.gateway, &st.ip, &udp)));
                }
                continue;
            }
            for _ in 0..MAX_ACCEPTS {
                let evict = if self.tcp.len() < MAX_FLOWS { None } else {
                    match self.tcp.iter().position(|c| matches!(c.transport, Transport::TimeWait | Transport::Closed)) {
                        Some(i) => Some(i),
                        None => break,
                    }
                };
                let Listener::Tcp(listener) = &self.forwards[f].listener else { break };
                let (stream, peer) = match listener.accept() {
                    Ok(accepted) => accepted,
                    Err(e) if e.kind() == ErrorKind::Interrupted => continue,
                    Err(e) => {
                        if e.kind() != ErrorKind::WouldBlock && self.log { eprintln!("[nat] hostfwd {} accept failed: {}", rule, e); }
                        break;
                    }
                };
                if stream.set_nonblocking(true).is_err() { continue; }
                let _ = stream.set_nodelay(true);
                if let Some(i) = evict { self.tcp.remove(i); }
                let port = self.ephemeral_port(&st.gateway);
                self.isn = self.isn.wrapping_add(0x10000);
                let mut c = Tcp { guest_mac: st.mac, guest_ip: st.ip, guest_port: rule.guest_port, dst_ip: st.gateway, dst_port: port,
                    transport: Transport::Connected(stream), inbound: true, guest_write: GuestWrite::Open, host_closed: false,
                    our_seq: self.isn, guest_seq: 0, guest_window: 0,
                    to_host: VecDeque::new(), unacked: VecDeque::new(), last_activity_us: now_us };
                out.push(c.send(SYN, Vec::new(), now_us));
                self.tcp.push(c);
                self.fwd_tcp += 1;
                if self.log { eprintln!("[nat] hostfwd {} {} -> guest {}:{} from {}:{}", rule, peer, ip(&st.ip), rule.guest_port, ip(&st.gateway), port); }
            }
        }
    }

    /// Forward a UDP datagram through a connected socket, which accepts replies only from its peer.
    #[allow(clippy::too_many_arguments, reason = "packet fields stay explicit at the protocol boundary")]
    pub fn udp_out(&mut self, gmac: &[u8; 6], gip: &[u8; 4], sport: u16, dip: &[u8; 4], reply_src: &[u8; 4],
                   dport: u16, payload: &[u8], now_us: u64) {
        // An answer to a forwarded datagram goes back to the host peer that sent the last one.
        if self.station.is_some_and(|st| st.gateway == *reply_src) {
            if let Some(f) = self.forwards.iter().find(|f| f.rule.udp && f.rule.host_port == dport && f.rule.guest_port == sport) {
                if let Listener::Udp { sock, peer: Some(peer) } = &f.listener {
                    match sock.send_to(payload, peer) {
                        Ok(n) => { self.fwd_udp_out += 1; self.bytes_to_host += n as u64; }
                        Err(e) => { self.udp_send_errors += 1; if self.log { eprintln!("[nat] hostfwd {} reply to {} failed: {}", f.rule, peer, e); } }
                    }
                }
                return;
            }
        }
        let idx = self.udp.iter().position(|f| f.guest_ip == *gip && f.guest_port == sport && f.dst_ip == *dip && f.dst_port == dport);
        let idx = match idx {
            Some(i) => i,
            None => {
                let Ok(sock) = UdpSocket::bind("0.0.0.0:0") else { return };
                if sock.connect(address(dip, dport)).is_err() || sock.set_nonblocking(true).is_err() { return; }
                if self.udp.len() >= MAX_FLOWS {
                    let oldest = self.udp.iter().enumerate().max_by_key(|(_, f)| now_us.wrapping_sub(f.last_activity_us)).unwrap().0;
                    self.udp.remove(oldest);
                    self.udp_evicted += 1;
                    if self.log { eprintln!("[nat] UDP evicted least recently active flow (table full)"); }
                }
                self.udp.push(Udp { guest_mac: *gmac, guest_ip: *gip, guest_port: sport, dst_ip: *dip, dst_port: dport,
                    reply_src: *reply_src, sock, last_activity_us: now_us });
                self.udp_flows += 1;
                if self.log { eprintln!("[nat] UDP {}:{} -> {}:{} ({} bytes)", ip(gip), sport, ip(dip), dport, payload.len()); }
                self.udp.len() - 1
            }
        };
        let flow = &mut self.udp[idx];
        flow.last_activity_us = now_us;
        match flow.sock.send(payload) {
            Ok(n) => self.bytes_to_host += n as u64,
            Err(e) => {
                self.udp_send_errors += 1;
                if self.log { eprintln!("[nat] UDP {}:{} send failed: {}", ip(dip), dport, e); }
            }
        }
    }

    pub fn tcp_in(&mut self, gmac: &[u8; 6], gip: &[u8; 4], dip: &[u8; 4], seg: &[u8], now_us: u64) -> Vec<Vec<u8>> {
        self.tcp_in_with_connect(gmac, gip, dip, seg, now_us, connect)
    }

    #[allow(clippy::too_many_arguments, reason = "inject the connector without process-global state in admission tests")]
    fn tcp_in_with_connect(&mut self, gmac: &[u8; 6], gip: &[u8; 4], dip: &[u8; 4], seg: &[u8], now_us: u64,
                           connector: impl FnOnce(SocketAddr) -> Option<Receiver<io::Result<TcpStream>>>) -> Vec<Vec<u8>> {
        if seg.len() < 20 { return Vec::new(); }
        let off = ((seg[12] >> 4) as usize) * 4;
        if off < 20 || off > seg.len() { return Vec::new(); }
        let sport = u16::from_be_bytes([seg[0], seg[1]]);
        let dport = u16::from_be_bytes([seg[2], seg[3]]);
        let seq = u32::from_be_bytes(seg[4..8].try_into().unwrap());
        let ack = u32::from_be_bytes(seg[8..12].try_into().unwrap());
        let flags = seg[13];
        let data = &seg[off..];
        let window = u16::from_be_bytes([seg[14], seg[15]]);
        let idx = self.tcp.iter().position(|c| c.guest_ip == *gip && c.guest_port == sport && c.dst_port == dport && c.dst_ip == *dip);
        if flags & (SYN | ACK | RST) == SYN && idx.is_none() {
            let evict = if self.tcp.len() >= MAX_FLOWS {
                // Finished flows remember final ACKs only while their bounded slots are spare.
                let Some(i) = self.tcp.iter().position(|c| matches!(c.transport, Transport::TimeWait | Transport::Closed)) else { return Vec::new() };
                Some(i)
            } else { None };
            let Some(pending) = connector(address(dip, dport)) else { return Vec::new() };
            if let Some(i) = evict { self.tcp.remove(i); }
            self.isn = self.isn.wrapping_add(0x10000);
            self.tcp.push(Tcp { guest_mac: *gmac, guest_ip: *gip, guest_port: sport, dst_ip: *dip, dst_port: dport,
                transport: Transport::Connecting(pending), inbound: false, guest_write: GuestWrite::Open, host_closed: false,
                our_seq: self.isn, guest_seq: seq.wrapping_add(1), guest_window: window,
                to_host: VecDeque::new(), unacked: VecDeque::new(), last_activity_us: now_us });
            if self.log { eprintln!("[nat] TCP {}:{} -> {}:{} connecting", ip(gip), sport, ip(dip), dport); }
            return Vec::new();
        }
        let Some(i) = idx else { return Vec::new() };
        let c = &mut self.tcp[i];
        c.last_activity_us = now_us;
        if flags & RST != 0 {
            let refused = c.inbound && c.handshake_pending();
            c.transport = Transport::Closed;
            if refused {
                self.fwd_refused += 1;
                if self.log { eprintln!("[nat] hostfwd: guest port {} refused the connection", sport); }
            }
            return Vec::new();
        }
        if matches!(c.transport, Transport::TimeWait) {
            return if flags & FIN != 0 { vec![c.segment(ACK, &[], c.our_seq)] } else { Vec::new() };
        }
        if flags & SYN != 0 {
            if c.inbound { return c.syn_ack(seq, ack, flags, window); }
            return if c.handshake_pending() { vec![c.segment(SYN | ACK, &[], c.unacked[0].seq)] } else { Vec::new() };
        }
        // SYN-SENT accepts only a SYN-ACK or a reset (RFC 9293 section 3.10.7.3).
        if c.inbound && c.handshake_pending() { return Vec::new(); }
        if !matches!(c.transport, Transport::Connected(_)) { return Vec::new(); }
        if flags & ACK != 0 { c.acknowledge(ack); c.guest_window = window; }
        if c.handshake_pending() { return Vec::new(); }
        if !data.is_empty() || flags & FIN != 0 {
            c.accept(seq, data, flags & FIN != 0);
            return vec![c.segment(ACK, &[], c.our_seq)];
        }
        Vec::new()
    }

    /// Pump a bounded amount of host traffic, then retransmit and expire flows.
    pub fn poll(&mut self, now_us: u64) -> Vec<Vec<u8>> {
        let mut out = Vec::new();
        self.accept_forwards(now_us, &mut out);
        for c in &mut self.tcp {
            if c.closed() { continue; }
            // A forwarded SYN's retransmissions do not refresh the flow, so this counts from the accept.
            if c.inbound && c.handshake_pending() && now_us.wrapping_sub(c.last_activity_us) >= FORWARD_SYN_TIMEOUT_US {
                if self.log { eprintln!("[nat] hostfwd: guest port {} never answered, closing the host connection", c.guest_port); }
                self.fwd_refused += 1;
                c.transport = Transport::Closed;
                continue;
            }
            if c.host_closed && c.guest_write == GuestWrite::Closed && c.unacked.is_empty()
                && matches!(c.transport, Transport::Connected(_)) {
                // Release the host socket but keep enough state to re-ACK a lost final ACK.
                c.transport = Transport::TimeWait;
                c.last_activity_us = now_us;
            }
            if let Transport::Connecting(rx) = &c.transport {
                let result = match rx.try_recv() {
                    Ok(result) => Some(result),
                    Err(TryRecvError::Disconnected) => Some(Err(ErrorKind::ConnectionAborted.into())),
                    Err(TryRecvError::Empty) => None,
                };
                match result {
                    Some(Ok(sock)) => {
                        c.transport = Transport::Connected(sock);
                        self.tcp_opened += 1;
                        if self.log { eprintln!("[nat] TCP {}:{} connected", ip(&c.dst_ip), c.dst_port); }
                        out.push(c.send(SYN | ACK, Vec::new(), now_us));
                    }
                    Some(Err(e)) => {
                        if self.log { eprintln!("[nat] TCP {}:{} failed: {}", ip(&c.dst_ip), c.dst_port, e); }
                        self.tcp_refused += 1;
                        out.push(c.segment(RST | ACK, &[], c.our_seq));
                        c.transport = Transport::Closed;
                    }
                    None => {}
                }
            }
            if matches!(c.transport, Transport::Connected(_)) && !c.handshake_pending() {
                let Transport::Connected(sock) = &mut c.transport else { unreachable!() };
                let queued = c.to_host.len();
                match flush_pending(sock, &mut c.to_host) {
                    Ok(n) => self.bytes_to_host += n as u64,
                    Err(_) => { out.push(c.segment(RST | ACK, &[], c.our_seq)); c.transport = Transport::Closed; continue; }
                }
                if c.guest_write == GuestWrite::Draining && c.to_host.is_empty() {
                    let _ = sock.shutdown(std::net::Shutdown::Write);
                    c.guest_write = GuestWrite::Closed;
                }
                if c.to_host.len() < queued { out.push(c.segment(ACK, &[], c.our_seq)); } // reopen the receive window
                // One byte at a zero window becomes a persist probe. The same retransmission
                // queue retries it, recovering even when the guest's window-update ACK is lost.
                let send_window = usize::from(c.guest_window.clamp(1, WINDOW));
                while !c.host_closed && c.in_flight() < send_window {
                    let room = send_window - c.in_flight();
                    let mut buf = [0; MSS];
                    let Transport::Connected(sock) = &mut c.transport else { break };
                    match sock.read(&mut buf[..room.min(MSS)]) {
                        Ok(0) => { c.host_closed = true; out.push(c.send(FIN | ACK, Vec::new(), now_us)); }
                        Ok(n) => {
                            if self.log { eprintln!("[nat] TCP {}:{} -> guest {} bytes (seq {})", ip(&c.dst_ip), c.dst_port, n, c.our_seq); }
                            self.bytes_to_guest += n as u64;
                            c.last_activity_us = now_us;
                            out.push(c.send(PSH | ACK, buf[..n].to_vec(), now_us));
                        }
                        Err(e) if e.kind() == ErrorKind::WouldBlock => break,
                        Err(e) if e.kind() == ErrorKind::Interrupted => continue,
                        Err(_) => { out.push(c.segment(RST | ACK, &[], c.our_seq)); c.transport = Transport::Closed; break; }
                    }
                }
            }
            if let Some(frame) = c.retransmit(now_us) { out.push(frame); }
        }
        for flow in &mut self.udp {
            let mut buf = [0; 2048];
            for _ in 0..UDP_BATCH {
                let Ok(n) = flow.sock.recv(&mut buf) else { break };
                if self.log { eprintln!("[nat] UDP reply {} bytes -> guest port {}", n, flow.guest_port); }
                self.bytes_to_guest += n as u64;
                flow.last_activity_us = now_us;
                let udp = udp_packet(&flow.reply_src, &flow.guest_ip, flow.dst_port, flow.guest_port, &buf[..n]);
                out.push(ethernet(&flow.guest_mac, &GATEWAY_MAC, 0x0800, &ip_packet(17, &flow.reply_src, &flow.guest_ip, &udp)));
            }
        }
        self.tcp.retain(|c| {
            let timeout = if matches!(c.transport, Transport::TimeWait) { TIME_WAIT_US } else { IDLE_CLOSE_US };
            !c.closed() && now_us.wrapping_sub(c.last_activity_us) < timeout
        });
        self.udp.retain(|f| now_us.wrapping_sub(f.last_activity_us) < IDLE_CLOSE_US);
        out
    }
}

fn host_resolver() -> [u8; 4] {
    if let Ok(conf) = std::fs::read_to_string("/etc/resolv.conf") {
        for line in conf.lines() {
            if let Some(rest) = line.trim().strip_prefix("nameserver ") {
                if let Ok(a) = rest.trim().parse::<Ipv4Addr>() { return a.octets(); }
            }
        }
    }
    [1, 1, 1, 1]
}

#[cfg(test)]
mod tests;
