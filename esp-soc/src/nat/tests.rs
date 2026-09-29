use super::*;
use std::net::TcpListener;

fn flow() -> Tcp {
    Tcp { guest_mac: [2; 6], guest_ip: [10, 0, 2, 15], guest_port: 1234,
        dst_ip: [127, 0, 0, 1], dst_port: 80, transport: Transport::Closed, inbound: false,
        guest_write: GuestWrite::Open, host_closed: false,
        our_seq: 100, guest_seq: 200, guest_window: WINDOW,
        to_host: VecDeque::new(), unacked: VecDeque::new(), last_activity_us: 0 }
}

fn segment(seq: u32, ack: u32, flags: u8, data: &[u8]) -> Vec<u8> {
    let mut seg = vec![0; 20];
    seg[..2].copy_from_slice(&1234u16.to_be_bytes());
    seg[2..4].copy_from_slice(&80u16.to_be_bytes());
    seg[4..8].copy_from_slice(&seq.to_be_bytes());
    seg[8..12].copy_from_slice(&ack.to_be_bytes());
    seg[12] = 0x50; seg[13] = flags;
    seg[14..16].copy_from_slice(&WINDOW.to_be_bytes());
    seg.extend_from_slice(data);
    seg
}

fn socket_pair() -> (TcpStream, TcpStream) {
    let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
    let a = TcpStream::connect(listener.local_addr().unwrap()).unwrap();
    let b = listener.accept().unwrap().0;
    a.set_nonblocking(true).unwrap();
    b.set_read_timeout(Some(std::time::Duration::from_secs(1))).unwrap();
    (a, b)
}

// The relay is nonblocking; wait for kernel loopback delivery without advancing emulated time.
fn poll_ready(nat: &mut Nat, now_us: u64) -> Vec<Vec<u8>> {
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(1);
    loop {
        let frames = nat.poll(now_us);
        if !frames.is_empty() { return frames; }
        assert!(std::time::Instant::now() < deadline, "loopback delivery timed out");
        std::thread::yield_now();
    }
}

#[test]
fn syn_data_and_fin_share_retransmission_and_cumulative_ack() {
    let mut c = flow();
    for (flags, data) in [(SYN | ACK, Vec::new()), (PSH | ACK, b"hello".to_vec()), (FIN | ACK, Vec::new())] {
        let seq = c.our_seq;
        let frame = c.send(flags, data.clone(), 10);
        assert!(c.retransmit(RETRANSMIT_US).is_none());
        assert_eq!(c.retransmit(RETRANSMIT_US + 10).unwrap(), frame);
        assert_eq!(u32::from_be_bytes(frame[38..42].try_into().unwrap()), seq);
        assert_eq!(transport_checksum(&c.dst_ip, &c.guest_ip, 6, &frame[34..]), 0);
        c.acknowledge(c.our_seq);
        assert!(c.unacked.is_empty());
    }
    assert_eq!(c.our_seq, 107);
}

#[test]
fn partial_and_wrapping_ack_preserve_unsent_sequence_space() {
    let mut c = flow();
    c.our_seq = u32::MAX - 2;
    c.send(PSH | ACK, b"abcdef".to_vec(), 0);
    c.send(FIN | ACK, Vec::new(), 0);
    c.acknowledge(1000); // ACK beyond what we sent is ignored
    assert_eq!(c.in_flight(), 7);
    c.acknowledge(0);
    assert_eq!(c.unacked[0].data, b"def");
    assert_eq!(c.unacked[0].seq, 0);
    c.acknowledge(4);
    assert!(c.unacked.is_empty());
}

#[test]
fn guest_fin_is_in_order_and_idempotent_including_payload() {
    let mut c = flow();
    c.accept(201, b"", true);
    assert_eq!(c.guest_seq, 200);
    c.accept(200, b"abc", true);
    assert_eq!(c.guest_seq, 204);
    c.accept(200, b"abc", true);
    c.accept(203, b"", true);
    assert_eq!(c.guest_seq, 204);
    assert_eq!(c.to_host.make_contiguous(), b"abc");
    assert!(c.guest_write == GuestWrite::Draining);
}

#[test]
fn full_guest_buffer_does_not_ack_unstored_bytes_or_fin() {
    let mut c = flow();
    c.accept(200, &vec![7; WINDOW as usize], false);
    let next = c.guest_seq;
    c.accept(next, &[8], true);
    assert_eq!(c.guest_seq, next);
    assert_eq!(c.to_host.len(), WINDOW as usize);
    assert!(c.guest_write == GuestWrite::Open);
    let ack = c.segment(ACK, &[], c.our_seq);
    assert_eq!(&ack[48..50], &[0, 0]); // advertised window
}

struct ShortWriter { bytes: Vec<u8>, allowance: usize }
impl Write for ShortWriter {
    fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
        if self.allowance == 0 { return Err(ErrorKind::WouldBlock.into()); }
        let n = bytes.len().min(self.allowance).min(2);
        self.bytes.extend_from_slice(&bytes[..n]); self.allowance -= n;
        Ok(n)
    }
    fn flush(&mut self) -> io::Result<()> { Ok(()) }
}

#[test]
fn short_write_and_would_block_preserve_exact_remaining_bytes() {
    let mut writer = ShortWriter { bytes: Vec::new(), allowance: 3 };
    let mut queue = VecDeque::from(b"abcdefgh".to_vec());
    assert_eq!(flush_pending(&mut writer, &mut queue).unwrap(), 3);
    assert_eq!(queue.make_contiguous(), b"defgh");
    assert_eq!(flush_pending(&mut writer, &mut queue).unwrap(), 0);
    writer.allowance = 100;
    assert_eq!(flush_pending(&mut writer, &mut queue).unwrap(), 5);
    assert!(queue.is_empty());
    assert_eq!(writer.bytes, b"abcdefgh");
}

#[test]
fn fin_waits_for_host_write_drain_and_remains_until_acknowledged() {
    let (socket, mut host) = socket_pair();
    let mut nat = Nat::new(false);
    let mut c = flow(); c.transport = Transport::Connected(socket);
    nat.tcp.push(c);
    let reply = nat.tcp_in(&[2; 6], &[10, 0, 2, 15], &[127, 0, 0, 1], &segment(200, 100, ACK | FIN, b"hello"), 0);
    assert_eq!(reply.len(), 1);
    nat.poll(1);
    let mut received = Vec::new(); host.read_to_end(&mut received).unwrap();
    assert_eq!(received, b"hello");
    host.shutdown(std::net::Shutdown::Write).unwrap();
    let sent = poll_ready(&mut nat, 2);
    assert_eq!(sent.len(), 1);
    assert_eq!(sent[0][47], ACK | FIN);
    assert_eq!(nat.poll(RETRANSMIT_US + 2), sent);
    assert_eq!(nat.tcp.len(), 1);
    nat.tcp_in(&[2; 6], &[10, 0, 2, 15], &[127, 0, 0, 1], &segment(206, 101, ACK, &[]), RETRANSMIT_US + 3);
    nat.poll(RETRANSMIT_US + 4);
    assert!(matches!(nat.tcp[0].transport, Transport::TimeWait));
    let reply = nat.tcp_in(&[2; 6], &[10, 0, 2, 15], &[127, 0, 0, 1], &segment(205, 101, ACK | FIN, &[]), RETRANSMIT_US + 5);
    assert_eq!(reply.len(), 1);
    assert_eq!(u32::from_be_bytes(reply[0][42..46].try_into().unwrap()), 206);
    nat.poll(RETRANSMIT_US + 5 + TIME_WAIT_US);
    assert!(nat.tcp.is_empty());
}

#[test]
fn dropped_syn_ack_is_resent_on_timer_and_duplicate_syn() {
    let (socket, _host) = socket_pair();
    let (tx, rx) = channel(); tx.send(Ok(socket)).unwrap();
    let mut c = flow(); c.transport = Transport::Connecting(rx);
    let mut nat = Nat::new(false); nat.tcp.push(c);
    let syn_ack = nat.poll(0);
    assert_eq!(syn_ack.len(), 1);
    assert_eq!(nat.poll(RETRANSMIT_US), syn_ack);
    assert_eq!(nat.tcp_in(&[2; 6], &[10, 0, 2, 15], &[127, 0, 0, 1], &segment(199, 0, SYN, &[]), RETRANSMIT_US + 1), syn_ack);
    nat.tcp_in(&[2; 6], &[10, 0, 2, 15], &[127, 0, 0, 1], &segment(200, 101, ACK, &[]), RETRANSMIT_US + 2);
    assert!(nat.poll(RETRANSMIT_US * 2).is_empty());
}

#[test]
fn host_reads_stop_at_the_unacknowledged_window() {
    let (socket, mut host) = socket_pair();
    host.write_all(&vec![7; WINDOW as usize * 2]).unwrap();
    let mut c = flow(); c.transport = Transport::Connected(socket);
    let mut nat = Nat::new(false); nat.tcp.push(c);
    let frames = poll_ready(&mut nat, 0);
    assert_eq!(frames.iter().map(|frame| frame.len() - 54).sum::<usize>(), WINDOW as usize);
    assert_eq!(nat.tcp[0].in_flight(), WINDOW as usize);
    assert!(nat.poll(1).is_empty());
    assert_eq!(nat.tcp[0].in_flight(), WINDOW as usize);
}

#[test]
fn zero_window_probe_recovers_a_lost_window_update() {
    let (socket, mut host) = socket_pair();
    host.write_all(b"hello").unwrap();
    let mut c = flow(); c.transport = Transport::Connected(socket); c.guest_window = 0;
    let mut nat = Nat::new(false); nat.tcp.push(c);
    let probe = poll_ready(&mut nat, 0);
    assert_eq!(probe.len(), 1);
    assert_eq!(&probe[0][54..], b"h");
    assert_eq!(nat.poll(RETRANSMIT_US), probe);
    // The repeated byte reaches the reopened window even though its window update was lost.
    nat.tcp_in(&[2; 6], &[10, 0, 2, 15], &[127, 0, 0, 1], &segment(200, 101, ACK, &[]), RETRANSMIT_US + 1);
    let rest = poll_ready(&mut nat, RETRANSMIT_US + 2);
    assert_eq!(rest.len(), 1);
    assert_eq!(&rest[0][54..], b"ello");
}

#[test]
fn connector_permits_are_bounded_until_the_worker_drops_them() {
    let counter = AtomicUsize::new(0);
    let mut permits: Vec<_> = (0..MAX_CONNECTS).map(|_| ConnectPermit::acquire(&counter).unwrap()).collect();
    assert!(ConnectPermit::acquire(&counter).is_none());
    permits.pop();
    assert!(ConnectPermit::acquire(&counter).is_some());
}

#[test]
fn time_wait_is_evicted_only_after_a_connect_worker_is_admitted() {
    // Final-ACK retry records cannot monopolize live-flow slots during connection churn.
    // Inject admission results so concurrent real workers cannot affect this test.
    let mut nat = Nat::new(false);
    for port in 0..MAX_FLOWS {
        let mut c = flow(); c.guest_port = port as u16; c.transport = Transport::TimeWait;
        nat.tcp.push(c);
    }
    nat.tcp_in_with_connect(&[2; 6], &[10, 0, 2, 15], &[127, 0, 0, 1], &segment(199, 0, SYN, &[]), 0, |_| None);
    assert_eq!(nat.tcp.len(), MAX_FLOWS);
    assert!(nat.tcp.iter().all(|c| matches!(c.transport, Transport::TimeWait)));
    let (_tx, rx) = channel();
    nat.tcp_in_with_connect(&[2; 6], &[10, 0, 2, 15], &[127, 0, 0, 1], &segment(199, 0, SYN, &[]), 0, |_| Some(rx));
    assert_eq!(nat.tcp.len(), MAX_FLOWS);
    assert!(matches!(nat.tcp.last().unwrap().transport, Transport::Connecting(_)));
}

#[test]
fn udp_at_capacity_reuses_existing_flows_and_evicts_the_least_recent() {
    let receiver = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
    receiver.set_read_timeout(Some(std::time::Duration::from_secs(1))).unwrap();
    let port = receiver.local_addr().unwrap().port();
    let mut nat = Nat::new(false);
    let mut bytes = [0; 16];
    for sport in 0..MAX_FLOWS as u16 {
        nat.udp_out(&[2; 6], &[10, 0, 2, 15], sport, &[127, 0, 0, 1], &[127, 0, 0, 1], port, b"one", sport as u64);
        assert_eq!(receiver.recv_from(&mut bytes).unwrap().0, 3);
    }
    nat.udp_out(&[2; 6], &[10, 0, 2, 15], 0, &[127, 0, 0, 1], &[127, 0, 0, 1], port, b"reuse", 100);
    assert_eq!(receiver.recv_from(&mut bytes).unwrap().0, 5);
    assert_eq!(nat.udp_flows, MAX_FLOWS as u64);
    nat.udp_out(&[2; 6], &[10, 0, 2, 15], 1234, &[127, 0, 0, 1], &[127, 0, 0, 1], port, b"new", 101);
    assert_eq!(receiver.recv_from(&mut bytes).unwrap().0, 3);
    assert_eq!(nat.udp.len(), MAX_FLOWS);
    assert_eq!(nat.udp_evicted, 1);
    assert!(nat.udp.iter().any(|f| f.guest_port == 0));
    assert!(!nat.udp.iter().any(|f| f.guest_port == 1));
    assert!(nat.udp.iter().any(|f| f.guest_port == 1234));
    assert_eq!(nat.udp_send_errors, 0);
}

#[test]
fn short_and_invalid_tcp_headers_are_rejected() {
    let mut nat = Nat::new(false);
    for len in 0..64 {
        for offset in 0..16 {
            let mut seg = vec![0; len];
            if len > 12 { seg[12] = offset << 4; }
            assert!(nat.tcp_in(&[2; 6], &[10, 0, 2, 15], &[127, 0, 0, 1], &seg, 0).is_empty());
        }
    }
    assert!(nat.tcp.is_empty());
}

#[test]
fn udp_ignores_other_senders_and_preserves_dns_reply_address() {
    let resolver = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
    resolver.set_read_timeout(Some(std::time::Duration::from_secs(1))).unwrap();
    let intruder = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
    let mut nat = Nat::new(false);
    nat.udp_out(&[2; 6], &[10, 0, 2, 15], 1234, &[127, 0, 0, 1], &[10, 0, 2, 3], resolver.local_addr().unwrap().port(), b"query", 0);
    let mut bytes = [0; 32];
    let (_, from) = resolver.recv_from(&mut bytes).unwrap();
    intruder.send_to(b"forged", from).unwrap();
    resolver.send_to(b"answer", from).unwrap();
    let frames = poll_ready(&mut nat, 1);
    assert_eq!(frames.len(), 1);
    assert_eq!(&frames[0][26..30], &[10, 0, 2, 3]);
    assert_eq!(&frames[0][42..], b"answer");
}

// ---- Host ports forwarded into the guest ----

const STATION: Station = Station { mac: [2; 6], ip: [10, 0, 2, 15], gateway: [10, 0, 2, 2] };

/// A segment from the guest's `sport` to the gateway's `dport`, as the guest's stack sends it.
fn from_guest(sport: u16, dport: u16, seq: u32, ack: u32, flags: u8, data: &[u8]) -> Vec<u8> {
    let mut seg = segment(seq, ack, flags, data);
    seg[..2].copy_from_slice(&sport.to_be_bytes());
    seg[2..4].copy_from_slice(&dport.to_be_bytes());
    seg
}

struct Seen { src: [u8; 4], dst: [u8; 4], sport: u16, dport: u16, seq: u32, ack: u32, flags: u8, data: Vec<u8> }

/// Decode a frame the relay sent to the guest, checking the IP and TCP checksums on the way.
fn seen(frame: &[u8]) -> Seen {
    assert_eq!(&frame[..6], &STATION.mac);
    assert_eq!(&frame[12..14], &[8, 0]);
    let ip_packet = &frame[14..];
    assert_eq!(checksum(&ip_packet[..20], 0), 0);
    let (src, dst): ([u8; 4], [u8; 4]) = (ip_packet[12..16].try_into().unwrap(), ip_packet[16..20].try_into().unwrap());
    let t = &ip_packet[20..];
    assert_eq!(transport_checksum(&src, &dst, 6, t), 0);
    let off = (t[12] >> 4) as usize * 4;
    Seen { src, dst, sport: u16::from_be_bytes([t[0], t[1]]), dport: u16::from_be_bytes([t[2], t[3]]),
        seq: u32::from_be_bytes(t[4..8].try_into().unwrap()), ack: u32::from_be_bytes(t[8..12].try_into().unwrap()),
        flags: t[13], data: t[off..].to_vec() }
}

fn tcp_forward(nat: &mut Nat) -> SocketAddr {
    nat.forward(HostFwd { udp: false, host_port: 0, guest_port: 80 }).unwrap()
}

fn client(at: SocketAddr) -> TcpStream {
    let c = TcpStream::connect(at).unwrap();
    c.set_read_timeout(Some(std::time::Duration::from_secs(1))).unwrap();
    c
}

use crate::net::packet::checksum;

#[test]
fn hostfwd_rules_parse_and_refuse_nonsense() {
    assert_eq!(HostFwd::parse("tcp:8080-80"), Ok(HostFwd { udp: false, host_port: 8080, guest_port: 80 }));
    assert_eq!(HostFwd::parse("udp:4210-4210"), Ok(HostFwd { udp: true, host_port: 4210, guest_port: 4210 }));
    assert_eq!(HostFwd::parse("tcp:8081-81").unwrap().to_string(), "tcp:8081-81");
    for bad in ["", "tcp", "tcp:8080", "sctp:1-2", "tcp:0-80", "tcp:8080-0", "tcp:70000-80", "tcp:a-80", "tcp:8080:80"] {
        assert!(HostFwd::parse(bad).is_err(), "{bad:?}");
    }
}

#[test]
fn forwarded_connection_waits_for_a_lease_then_opens_toward_the_guest() {
    let mut nat = Nat::new(false);
    let at = tcp_forward(&mut nat);
    assert_eq!(at.ip(), IpAddr::V4(Ipv4Addr::LOCALHOST));
    let _host = client(at);
    // No lease yet: the connection stays in the host backlog.
    std::thread::sleep(std::time::Duration::from_millis(20));
    assert!(nat.poll(0).is_empty());
    assert!(nat.tcp.is_empty());
    nat.station = Some(STATION);
    let syn = poll_ready(&mut nat, 1);
    assert_eq!(syn.len(), 1);
    let s = seen(&syn[0]);
    assert_eq!((s.src, s.dst, s.dport, s.flags), (STATION.gateway, STATION.ip, 80, SYN));
    assert_eq!(s.sport, EPHEMERAL);
    assert!(s.data.is_empty());
    assert_eq!(nat.fwd_tcp, 1);
    // An unanswered SYN is retransmitted on the shared timer.
    assert!(nat.poll(RETRANSMIT_US).is_empty());
    assert_eq!(nat.poll(RETRANSMIT_US + 1), syn);
}

/// Accept a forwarded connection and complete the handshake; returns the relay's ISS + 1 and port.
fn establish(nat: &mut Nat, guest_iss: u32) -> (u32, u16) {
    let syn = seen(&poll_ready(nat, 1)[0]);
    let our = syn.seq.wrapping_add(1);
    let ack = nat.tcp_in(&STATION.mac, &STATION.ip, &STATION.gateway, &from_guest(80, syn.sport, guest_iss, our, SYN | ACK, &[]), 2);
    assert_eq!(ack.len(), 1);
    let a = seen(&ack[0]);
    assert_eq!((a.flags, a.seq, a.ack), (ACK, our, guest_iss + 1));
    (our, syn.sport)
}

#[test]
fn forwarded_connection_relays_both_ways_and_closes_cleanly() {
    let mut nat = Nat::new(false);
    let at = tcp_forward(&mut nat);
    let mut host = client(at);
    host.write_all(b"GET / HTTP/1.0\r\n\r\n").unwrap();
    nat.station = Some(STATION);
    let (our, port) = establish(&mut nat, 5000);
    // A SYN-ACK repeated because our ACK was lost is acknowledged again, not taken as new.
    let again = nat.tcp_in(&STATION.mac, &STATION.ip, &STATION.gateway, &from_guest(80, port, 5000, our, SYN | ACK, &[]), 3);
    assert_eq!(seen(&again[0]).ack, 5001);
    // Host -> guest: the request the client wrote before the guest answered.
    let req = poll_ready(&mut nat, 4);
    let r = seen(&req[0]);
    assert_eq!((r.flags, r.seq, r.ack, r.data.as_slice()), (PSH | ACK, our, 5001, &b"GET / HTTP/1.0\r\n\r\n"[..]));
    // Guest -> host: the answer, acknowledged and written to the host socket.
    let reply = b"HTTP/1.0 200 OK\r\n\r\nhi";
    let acked = nat.tcp_in(&STATION.mac, &STATION.ip, &STATION.gateway,
        &from_guest(80, port, 5001, our + 18, PSH | ACK, reply), 5);
    assert_eq!(seen(&acked[0]).ack, 5001 + reply.len() as u32);
    nat.poll(6);
    let mut got = vec![0; reply.len()];
    host.read_exact(&mut got).unwrap();
    assert_eq!(got, reply);
    // The guest closes (an HTTP/1.0 server does): the host sees EOF after the data.
    let fin_seq = 5001 + reply.len() as u32;
    nat.tcp_in(&STATION.mac, &STATION.ip, &STATION.gateway, &from_guest(80, port, fin_seq, our + 18, FIN | ACK, &[]), 7);
    nat.poll(8);
    assert_eq!(host.read(&mut [0; 8]).unwrap(), 0);
    // The host closes too: FIN to the guest, and once acknowledged the host socket is released.
    drop(host);
    let fin = poll_ready(&mut nat, 9);
    let f = seen(&fin[0]);
    assert_eq!((f.flags, f.seq, f.ack), (FIN | ACK, our + 18, fin_seq + 1));
    nat.tcp_in(&STATION.mac, &STATION.ip, &STATION.gateway, &from_guest(80, port, fin_seq + 1, our + 19, ACK, &[]), 10);
    nat.poll(11);
    assert!(matches!(nat.tcp[0].transport, Transport::TimeWait));
    assert_eq!(nat.fwd_refused, 0);
}

#[test]
fn a_closed_guest_port_resets_and_the_host_connection_closes() {
    let mut nat = Nat::new(false);
    let at = tcp_forward(&mut nat);
    let mut host = client(at);
    nat.station = Some(STATION);
    let syn = seen(&poll_ready(&mut nat, 1)[0]);
    // lwIP answers a SYN to a port nobody listens on with RST|ACK, ACK = SYN + 1 (tcp_in.c).
    let rst = from_guest(80, syn.sport, 0, syn.seq + 1, RST | ACK, &[]);
    assert!(nat.tcp_in(&STATION.mac, &STATION.ip, &STATION.gateway, &rst, 2).is_empty());
    nat.poll(3);
    assert!(nat.tcp.is_empty());
    assert_eq!(nat.fwd_refused, 1);
    assert_eq!(host.read(&mut [0; 8]).unwrap(), 0);
}

#[test]
fn syn_sent_resets_a_wrong_ack_and_ignores_data_before_the_handshake() {
    let mut nat = Nat::new(false);
    let at = tcp_forward(&mut nat);
    let _host = client(at);
    nat.station = Some(STATION);
    let syn = seen(&poll_ready(&mut nat, 1)[0]);
    let stray = nat.tcp_in(&STATION.mac, &STATION.ip, &STATION.gateway, &from_guest(80, syn.sport, 7, syn.seq + 1, ACK, b"x"), 2);
    assert!(stray.is_empty());
    let wrong = nat.tcp_in(&STATION.mac, &STATION.ip, &STATION.gateway, &from_guest(80, syn.sport, 7, syn.seq + 9, SYN | ACK, &[]), 3);
    let r = seen(&wrong[0]);
    assert_eq!((r.flags, r.seq), (RST, syn.seq + 9));
    assert!(nat.tcp[0].handshake_pending());
}

#[test]
fn an_unanswered_forwarded_syn_gives_up() {
    let mut nat = Nat::new(false);
    let at = tcp_forward(&mut nat);
    let mut host = client(at);
    nat.station = Some(STATION);
    poll_ready(&mut nat, 1);
    nat.poll(FORWARD_SYN_TIMEOUT_US);
    assert_eq!(nat.tcp.len(), 1);
    nat.poll(FORWARD_SYN_TIMEOUT_US + 1);
    assert!(nat.tcp.is_empty());
    assert_eq!(nat.fwd_refused, 1);
    assert_eq!(host.read(&mut [0; 8]).unwrap(), 0);
}

#[test]
fn concurrent_forwarded_connections_get_their_own_gateway_ports() {
    let mut nat = Nat::new(false);
    let at = tcp_forward(&mut nat);
    let hosts: Vec<_> = (0..5).map(|_| client(at)).collect();
    nat.station = Some(STATION);
    let mut ports = Vec::new();
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(1);
    while ports.len() < hosts.len() {
        assert!(std::time::Instant::now() < deadline, "not every connection was accepted");
        ports.extend(nat.poll(1).iter().map(|f| seen(f).sport));
    }
    ports.sort();
    assert_eq!(ports, (EPHEMERAL..EPHEMERAL + 5).collect::<Vec<_>>());
    assert_eq!(nat.fwd_tcp, 5);
}

#[test]
fn a_full_flow_table_leaves_host_connections_queued() {
    let mut nat = Nat::new(false);
    let at = tcp_forward(&mut nat);
    let mut senders = Vec::new();
    for port in 0..MAX_FLOWS {
        let (tx, rx) = channel(); senders.push(tx);
        let mut c = flow(); c.guest_port = port as u16; c.transport = Transport::Connecting(rx);
        nat.tcp.push(c);
    }
    let _host = client(at);
    nat.station = Some(STATION);
    std::thread::sleep(std::time::Duration::from_millis(20));
    assert!(nat.poll(1).is_empty());
    assert_eq!(nat.tcp.len(), MAX_FLOWS);
    assert_eq!(nat.fwd_tcp, 0);
    // A finished flow frees its slot, and the queued connection is taken.
    nat.tcp[0].transport = Transport::TimeWait;
    let syn = poll_ready(&mut nat, 2);
    assert_eq!(seen(&syn[0]).flags, SYN);
    assert_eq!(nat.tcp.len(), MAX_FLOWS);
}

#[test]
fn forwarded_udp_arrives_from_the_gateway_and_answers_reach_the_last_peer() {
    let mut nat = Nat::new(false);
    let at = nat.forward(HostFwd { udp: true, host_port: 0, guest_port: 4210 }).unwrap();
    let peer = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
    peer.set_read_timeout(Some(std::time::Duration::from_secs(1))).unwrap();
    peer.send_to(b"stats", at).unwrap();
    std::thread::sleep(std::time::Duration::from_millis(20));
    assert!(nat.poll(0).is_empty()); // no lease: the datagram waits in the host socket
    nat.station = Some(STATION);
    let frames = poll_ready(&mut nat, 1);
    assert_eq!(frames.len(), 1);
    let f = &frames[0];
    assert_eq!(&f[..6], &STATION.mac);
    assert_eq!((&f[26..30], &f[30..34]), (&STATION.gateway[..], &STATION.ip[..]));
    assert_eq!(u16::from_be_bytes([f[34], f[35]]), at.port());
    assert_eq!(u16::from_be_bytes([f[36], f[37]]), 4210);
    assert_eq!(transport_checksum(&STATION.gateway, &STATION.ip, 17, &f[34..]), 0);
    assert_eq!(&f[42..], b"stats");
    // The guest answers the gateway address and port it saw.
    nat.udp_out(&STATION.mac, &STATION.ip, 4210, &STATION.gateway, &STATION.gateway, at.port(), b"ok", 2);
    let mut buf = [0; 8];
    let (n, from) = peer.recv_from(&mut buf).unwrap();
    assert_eq!((&buf[..n], from), (&b"ok"[..], at));
    assert!(nat.udp.is_empty(), "an answer to a forwarded datagram is not a new outbound flow");
    assert_eq!((nat.fwd_udp_in, nat.fwd_udp_out), (1, 1));
}
