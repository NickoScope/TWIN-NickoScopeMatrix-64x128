use super::*;
use std::net::Shutdown;
use std::sync::mpsc;
use std::time::Duration;

struct Connection {
    socket: TcpStream,
    handler: Option<std::thread::JoinHandle<()>>,
}

impl Connection {
    fn new(web: &WebServer) -> Self {
        let listener = TcpListener::bind(("127.0.0.1", 0)).unwrap();
        let socket = TcpStream::connect(listener.local_addr().unwrap()).unwrap();
        socket.set_read_timeout(Some(Duration::from_secs(3))).unwrap();
        let shared = web.shared.clone();
        let handler = std::thread::spawn(move || {
            let (stream, _) = listener.accept().unwrap();
            handle_client(stream, shared);
        });
        Self { socket, handler: Some(handler) }
    }

    fn handshake(&mut self, origin: Option<&str>) {
        let origin = origin.map(|o| format!("Origin: {o}\r\n")).unwrap_or_default();
        write!(self.socket, "GET /ws HTTP/1.1\r\nHost: localhost\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\nSec-WebSocket-Version: 13\r\n{origin}\r\n").unwrap();
    }

    fn head(&mut self) -> String {
        let mut bytes = Vec::new();
        while !bytes.ends_with(b"\r\n\r\n") {
            let mut byte = [0];
            self.socket.read_exact(&mut byte).unwrap();
            bytes.push(byte[0]);
            assert!(bytes.len() < 4096);
        }
        String::from_utf8(bytes).unwrap()
    }

    fn expect_frame(&mut self, opcode: u8, payload: &[u8]) {
        let expected = frame(opcode, payload);
        let mut actual = vec![0; expected.len()];
        self.socket.read_exact(&mut actual).unwrap();
        assert_eq!(actual, expected);
    }
}

impl Drop for Connection {
    fn drop(&mut self) {
        let _ = self.socket.shutdown(Shutdown::Both);
        self.handler.take().unwrap().join().unwrap();
    }
}

fn socket_web() -> WebServer {
    let web = WebServer::queued();
    web.shared.lock().unwrap().queue = false;
    web
}

fn masked_frame(opcode: u8, payload: &[u8]) -> Vec<u8> {
    assert!(payload.len() < 126);
    let mask = [0x31, 0x42, 0x53, 0x64];
    let mut bytes = vec![0x80 | opcode, 0x80 | payload.len() as u8];
    bytes.extend_from_slice(&mask);
    bytes.extend(payload.iter().enumerate().map(|(i, b)| b ^ mask[i & 3]));
    bytes
}

#[test]
fn websocket_key_header_matches_any_case() {
    for line in ["Sec-WebSocket-Key: abc==", "sec-websocket-key: abc==", "SEC-WEBSOCKET-KEY:abc==", "Sec-Websocket-Key :  abc==  "] {
        let head = format!("GET /ws HTTP/1.1\r\nHost: x\r\n{line}\r\nUpgrade: websocket\r\n\r\n");
        assert_eq!(header(&head, "Sec-WebSocket-Key"), Some("abc=="), "{line}");
    }
    assert_eq!(header("GET / HTTP/1.1\r\nHost: x\r\n\r\n", "Sec-WebSocket-Key"), None);
    assert_eq!(header("GET /Sec-WebSocket-Key: HTTP/1.1\r\n\r\n", "GET /Sec-WebSocket-Key"), None);
    assert_eq!(header("GET / HTTP/1.1\r\n\r\nOrigin: fake", "Origin"), None);
}

#[test]
fn websocket_accept_matches_rfc_6455_example() {
    let key = "dGhlIHNhbXBsZSBub25jZQ==";
    assert_eq!(b64(&sha1(format!("{}258EAFA5-E914-47DA-95CA-C5AB0DC85B11", key).as_bytes())), "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=");
}

#[test]
fn json_fields_are_top_level_and_strings_decode_unicode() {
    assert_eq!(json_str(r#"{"nested":{"t":"knob"},"t":"serial"}"#, "t").as_deref(), Some("serial"));
    assert_eq!(json_str(r#"{"t":"serial","line":"\uD83D\uDE80"}"#, "line").as_deref(), Some("🚀"));
    assert_eq!(json_str(r#"{"line":"unfinished}"#, "line"), None);
    assert_eq!(json_str(r#"{"d":1e2}"#, "d").as_deref(), Some("100"));
}

#[test]
fn websocket_origin_requires_this_loopback_port_when_present() {
    for origin in ["http://127.0.0.1:8766", "http://localhost:8766"] {
        assert!(local_origin(&format!("GET / HTTP/1.1\r\nOrigin: {origin}\r\n\r\n"), 8766));
    }
    for origin in ["null", "https://localhost:8766", "http://localhost:8765", "http://localhost.evil:8766", "http://127.0.0.1:8766/", "http://127.0.0.1:8766@evil"] {
        assert!(!local_origin(&format!("GET / HTTP/1.1\r\nOrigin: {origin}\r\n\r\n"), 8766), "{origin}");
    }
    assert!(local_origin("GET / HTTP/1.1\r\n\r\n", 8766), "native tools omit Origin");
    assert!(local_origin("GET / HTTP/1.1\r\nOrigin: http://localhost\r\n\r\n", 80));
}

#[test]
fn websocket_rejects_foreign_origin_before_registering_client() {
    let web = socket_web();
    let mut connection = Connection::new(&web);
    connection.handshake(Some("https://example.com"));
    assert!(connection.head().starts_with("HTTP/1.1 403 Forbidden\r\n"));
    assert_eq!(web.clients(), 0);
}

#[test]
fn websocket_snapshot_precedes_live_frames_and_ping_gets_pong() {
    let web = socket_web();
    web.set_hello(vec![frame(1, b"board"), frame(2, b"snapshot")]);
    let mut connection = Connection::new(&web);
    let origin = format!("http://localhost:{}", connection.socket.peer_addr().unwrap().port());
    connection.handshake(Some(&origin));
    assert!(connection.head().starts_with("HTTP/1.1 101 Switching Protocols\r\n"));
    connection.expect_frame(1, b"board");
    web.send_text("live");
    connection.socket.write_all(&masked_frame(9, b"ping payload")).unwrap();
    connection.socket.write_all(&masked_frame(9, b"")).unwrap();
    connection.expect_frame(2, b"snapshot");
    connection.expect_frame(1, b"live");
    connection.expect_frame(10, b"ping payload");
    connection.expect_frame(10, b"");
    assert!(web.poll_incoming().is_empty());
}

#[test]
fn websocket_preserves_frame_received_with_http_head() {
    let web = socket_web();
    let mut connection = Connection::new(&web);
    let mut request = b"GET / HTTP/1.1\r\nSec-WebSocket-Key: abc==\r\n\r\n".to_vec();
    request.extend(masked_frame(9, b"already buffered"));
    connection.socket.write_all(&request).unwrap();
    assert!(connection.head().starts_with("HTTP/1.1 101 Switching Protocols\r\n"));
    connection.expect_frame(10, b"already buffered");
}

#[test]
fn slow_snapshot_client_does_not_hold_emulator_mutex() {
    let web = socket_web();
    web.set_hello(vec![frame(2, &vec![0x5a; 16 << 20])]);
    let mut connection = Connection::new(&web);
    connection.handshake(None);
    assert!(connection.head().starts_with("HTTP/1.1 101 Switching Protocols\r\n"));
    let mut frame_header = [0; 10];
    connection.socket.read_exact(&mut frame_header).unwrap();
    assert_eq!(frame_header[0], 0x82);
    // Stop reading mid-snapshot. A socket write now blocks independently of the
    // emulator; its state and bounded live queue must remain usable.
    let (tx, rx) = mpsc::channel();
    let emulator = std::thread::spawn(move || {
        for _ in 0..300 { web.send_text("live"); }
        web.push_incoming("input".into());
        tx.send(web.poll_incoming()).unwrap();
    });
    let result = rx.recv_timeout(Duration::from_secs(2));
    drop(connection);
    emulator.join().unwrap();
    assert_eq!(result.unwrap(), vec!["input"]);
}

struct StaticRoot(std::path::PathBuf);

impl StaticRoot {
    fn new() -> Self {
        static NEXT: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let id = NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let path = std::env::temp_dir().join(format!("esp-soc-web-{}-{id}", std::process::id()));
        std::fs::create_dir_all(path.join("public")).unwrap();
        Self(path)
    }
    fn public(&self) -> String { self.0.join("public").to_string_lossy().into_owned() }
}

impl Drop for StaticRoot {
    fn drop(&mut self) { std::fs::remove_dir_all(&self.0).unwrap(); }
}

#[test]
fn static_files_stay_in_root_and_unknown_content_is_not_html() {
    let root = StaticRoot::new();
    std::fs::write(root.0.join("secret"), b"private").unwrap();
    std::fs::write(root.0.join("public/blob.bin"), b"<script>bad()</script>").unwrap();
    assert_eq!(static_file(&root.public(), "/../secret"), None);
    assert_eq!(static_file(&root.public(), "//secret"), None);
    assert_eq!(static_file(&root.public(), "secret"), None);
    let web = socket_web();
    web.shared.lock().unwrap().web_dir = root.public();
    let mut connection = Connection::new(&web);
    connection.socket.write_all(b"GET /blob.bin HTTP/1.1\r\n\r\n").unwrap();
    let head = connection.head();
    assert!(head.starts_with("HTTP/1.1 200 OK\r\n"));
    assert!(head.contains("Content-Type: application/octet-stream\r\n"));
    assert!(head.contains("X-Content-Type-Options: nosniff\r\n"));
    let mut body = Vec::new();
    connection.socket.read_to_end(&mut body).unwrap();
    assert_eq!(body, b"<script>bad()</script>");
    assert_eq!(content_type("/run.html"), "text/html; charset=utf-8");
}

#[cfg(unix)]
#[test]
fn static_symlinks_cannot_escape_root() {
    let root = StaticRoot::new();
    std::fs::write(root.0.join("secret"), b"private").unwrap();
    std::fs::write(root.0.join("public/data.txt"), b"public").unwrap();
    std::os::unix::fs::symlink("../secret", root.0.join("public/leak.txt")).unwrap();
    std::os::unix::fs::symlink("data.txt", root.0.join("public/alias.txt")).unwrap();
    assert_eq!(static_file(&root.public(), "/leak.txt"), None);
    assert_eq!(static_file(&root.public(), "/alias.txt"), Some(b"public".to_vec()));
}

#[test]
fn websocket_origin_accepts_matching_forwarded_loopback_host() {
    for host in ["localhost:9000", "127.0.0.1:9000", "[::1]:9000", "localhost"] {
        let request = format!("GET /ws HTTP/1.1\r\nHost: {host}\r\nOrigin: http://{host}\r\n\r\n");
        assert!(local_origin(&request, 8080), "{host}");
    }
    for host in ["localhost.evil:9000", "evil:9000", "127.0.0.1:9000@evil", "localhost:65536", "localhost:", "localhost:+80"] {
        let request = format!("GET /ws HTTP/1.1\r\nHost: {host}\r\nOrigin: http://{host}\r\n\r\n");
        assert!(!local_origin(&request, 8080), "{host}");
    }
    assert!(!local_origin("GET /ws HTTP/1.1\r\nHost: localhost:9000\r\nOrigin: http://localhost:9001\r\n\r\n", 8080));
}

// ---------------------------------------------------------------- /usj

use crate::usj_port::{PortIn, PortOut, UsjPort};

impl Connection {
    fn upgrade(&mut self, path: &str) -> String {
        write!(self.socket, "GET {path} HTTP/1.1\r\nHost: localhost\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\nSec-WebSocket-Version: 13\r\n\r\n").unwrap();
        self.head()
    }
    /// One server frame: (opcode, payload); the server never masks.
    fn read_frame(&mut self) -> (u8, Vec<u8>) {
        let mut h = [0u8; 2];
        self.socket.read_exact(&mut h).unwrap();
        let mut len = (h[1] & 0x7f) as usize;
        if len == 126 { let mut e = [0u8; 2]; self.socket.read_exact(&mut e).unwrap(); len = u16::from_be_bytes(e) as usize; }
        else if len == 127 { let mut e = [0u8; 8]; self.socket.read_exact(&mut e).unwrap(); len = u64::from_be_bytes(e) as usize; }
        let mut d = vec![0u8; len];
        self.socket.read_exact(&mut d).unwrap();
        (h[0] & 0x0f, d)
    }
}

/// A client frame of any length, masked (RFC 6455 §5.3); `fin` false for a fragment.
fn client_frame(opcode: u8, payload: &[u8], fin: bool) -> Vec<u8> {
    let mask = [0x9a, 0x0b, 0xfe, 0x71];
    let mut b = vec![(if fin { 0x80 } else { 0 }) | opcode];
    let n = payload.len();
    if n < 126 { b.push(0x80 | n as u8); } else if n < 65536 { b.push(0x80 | 126); b.extend_from_slice(&(n as u16).to_be_bytes()); } else { b.push(0x80 | 127); b.extend_from_slice(&(n as u64).to_be_bytes()); }
    b.extend_from_slice(&mask);
    b.extend(payload.iter().enumerate().map(|(i, x)| x ^ mask[i & 3]));
    b
}

fn wait_for(port: &UsjPort, n: usize) -> Vec<PortIn> {
    let mut got = Vec::new();
    for _ in 0..500 { got.extend(port.poll()); if got.len() >= n { break; } std::thread::sleep(Duration::from_millis(10)); }
    got
}

fn usj_web() -> (WebServer, UsjPort) {
    let web = socket_web();
    let port = UsjPort::new();
    web.attach_usj(port.clone());
    (web, port)
}

#[test]
fn usj_is_lossless_both_ways_for_every_byte_value() {
    let (web, port) = usj_web();
    let mut c = Connection::new(&web);
    assert!(c.upgrade("/usj").starts_with("HTTP/1.1 101"));
    c.socket.write_all(&client_frame(2, &[0x02], true)).unwrap();
    let open = wait_for(&port, 1);
    let PortIn::Open(id) = open[0] else { panic!("{open:?}") };
    // Host -> chip: 10,000 frames, every byte value, including 0x80..0xff that UTF-8 would mangle.
    let mut want = Vec::new();
    let mut wire = Vec::new();
    for i in 0..10_000u32 {
        let chunk: Vec<u8> = (0..(i % 70) as u8 + 1).map(|k| (i as u8).wrapping_mul(31).wrapping_add(k.wrapping_mul(97))).collect();
        want.extend_from_slice(&chunk);
        let mut f = vec![0x00];
        f.extend_from_slice(&chunk);
        wire.extend(client_frame(2, &f, true));
    }
    c.socket.write_all(&wire).unwrap();
    let mut got = Vec::new();
    for _ in 0..500 {
        for e in port.poll() { match e { PortIn::Data(d) => got.extend(d), other => panic!("{other:?}") } }
        if got.len() >= want.len() { break; }
        std::thread::sleep(Duration::from_millis(10));
    }
    assert_eq!(got.len(), want.len(), "nothing dropped");
    assert_eq!(got, want);
    // Chip -> host: 10,000 outputs, raw.
    for i in 0..10_000u32 { assert!(port.deliver(id, PortOut::Data(vec![0xff, i as u8, 0x80, (i >> 8) as u8]))); }
    for i in 0..10_000u32 {
        let (op, d) = c.read_frame();
        assert_eq!((op, d), (2, vec![0x00, 0xff, i as u8, 0x80, (i >> 8) as u8]), "frame {i}");
    }
    assert!(port.deliver(id, PortOut::Event("{\"t\":\"hello\"}".into())));
    assert_eq!(c.read_frame(), (2, b"\x10{\"t\":\"hello\"}".to_vec()));
}

#[test]
fn usj_lines_keep_their_place_among_the_bytes_and_close_releases() {
    let (web, port) = usj_web();
    let mut c = Connection::new(&web);
    assert!(c.upgrade("/usj?x=1").starts_with("HTTP/1.1 101"));
    c.socket.write_all(&client_frame(2, &[0x00, 0x41], true)).unwrap();    // data before any open
    assert_eq!(c.read_frame(), (2, b"\x10{\"t\":\"error\",\"error\":\"not open\"}".to_vec()));
    let mut wire = client_frame(2, &[0x02], true);
    wire.extend(client_frame(2, &[0x00, 0xc0], true));
    wire.extend(client_frame(2, &[0x01, 0x01], true));                     // DTR
    wire.extend(client_frame(2, &[0x00, 0xc1], true));
    wire.extend(client_frame(2, &[0x01, 0x02], true));                     // RTS
    // A fragmented data message arrives once, whole, with a ping between its fragments.
    wire.extend(client_frame(2, &[0x00, 0x01], false));
    wire.extend(client_frame(9, b"p", true));
    wire.extend(client_frame(0, &[0x02, 0x03], true));
    wire.extend(client_frame(1, b"text is ignored here", true));
    wire.extend(client_frame(2, &[0x7e], true));                            // unknown type: logged, ignored
    wire.extend(client_frame(2, &[0x03], true));
    c.socket.write_all(&wire).unwrap();
    let ev = wait_for(&port, 7);
    let PortIn::Open(id) = ev[0] else { panic!("{ev:?}") };
    assert_eq!(ev[1..], [PortIn::Data(vec![0xc0]), PortIn::Lines { dtr: true, rts: false }, PortIn::Data(vec![0xc1]),
                         PortIn::Lines { dtr: false, rts: true }, PortIn::Data(vec![0x01, 0x02, 0x03]), PortIn::Close(id)]);
    assert_eq!(c.read_frame(), (10, b"p".to_vec()));
    assert_eq!(c.read_frame(), (2, b"\x10{\"t\":\"closed\"}".to_vec()));
    assert!(!port.held());
    // A close frame is answered with 1000.
    c.socket.write_all(&client_frame(8, &[0x03, 0xe8], true)).unwrap();
    assert_eq!(c.read_frame(), (8, vec![0x03, 0xe8]));
}

#[test]
fn usj_second_client_is_refused_and_takes_over_after_the_first_leaves() {
    let (web, port) = usj_web();
    let mut a = Connection::new(&web);
    assert!(a.upgrade("/usj").starts_with("HTTP/1.1 101"));
    a.socket.write_all(&client_frame(2, &[0x02], true)).unwrap();
    let ev = wait_for(&port, 1);
    let PortIn::Open(first) = ev[0] else { panic!("{ev:?}") };
    let mut b = Connection::new(&web);
    assert!(b.upgrade("/usj").starts_with("HTTP/1.1 101"));
    b.socket.write_all(&client_frame(2, &[0x02], true)).unwrap();
    assert_eq!(b.read_frame(), (2, b"\x10{\"t\":\"error\",\"error\":\"busy\"}".to_vec()));
    drop(a);                                                                 // the first client goes away
    assert_eq!(wait_for(&port, 1), [PortIn::Close(first)]);
    b.socket.write_all(&client_frame(2, &[0x02], true)).unwrap();
    let ev = wait_for(&port, 1);
    assert!(matches!(ev[0], PortIn::Open(id) if id != first), "{ev:?}");
}

#[test]
fn usj_needs_a_port_and_a_local_origin() {
    let web = socket_web();
    let mut c = Connection::new(&web);
    assert!(c.upgrade("/usj").starts_with("HTTP/1.1 404 Not Found\r\n"), "no port in this run");
    let (web, _port) = usj_web();
    let mut c = Connection::new(&web);
    write!(c.socket, "GET /usj HTTP/1.1\r\nHost: localhost\r\nOrigin: https://nickoscope.github.io\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: abc==\r\n\r\n").unwrap();
    assert!(c.head().starts_with("HTTP/1.1 403 Forbidden\r\n"), "a public page may not reach the port");
}
