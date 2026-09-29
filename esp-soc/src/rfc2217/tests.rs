use super::*;
use crate::usj_port::PortIn;
use std::time::Duration;

fn replies(actions: &[Action]) -> Vec<u8> {
    actions.iter().flat_map(|a| match a { Action::Reply(r) => r.clone(), _ => Vec::new() }).collect()
}

#[test]
fn data_byte_255_is_doubled_both_ways() {
    let mut p = ComPort::new();
    assert_eq!(p.feed(&[0x01, IAC, IAC, 0x80, IAC, IAC, IAC, IAC]), [Action::Data(vec![0x01, 0xff, 0x80, 0xff, 0xff])]);
    assert_eq!(escape(&[0xff, 0x00, 0xff, 0xff, 0xc0]), [0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0xff, 0xc0]);
    // An IAC split across two reads is still one data byte.
    assert!(p.feed(&[0x10, IAC]) == [Action::Data(vec![0x10])]);
    assert_eq!(p.feed(&[IAC, 0x11]), [Action::Data(vec![0xff, 0x11])]);
    // Every byte value survives escape -> feed.
    let all: Vec<u8> = (0..=255u8).chain((0..=255u8).rev()).collect();
    let got: Vec<u8> = p.feed(&escape(&all)).into_iter().flat_map(|a| match a { Action::Data(d) => d, _ => panic!("{a:?}") }).collect();
    assert_eq!(got, all);
}

#[test]
fn pyserial_open_negotiates_without_loops() {
    // pyserial 3.5 Serial.open (rfc2217.py:429-469) sends these requests; the server answers each
    // once, and the answers to its own greeting need no answer.
    let mut p = ComPort::new();
    assert_eq!(p.greeting(), [IAC, WILL, BINARY, IAC, DO, BINARY]);
    let client = [IAC, DO, 1, IAC, WILL, SGA, IAC, DO, SGA, IAC, DO, COM_PORT_OPTION, IAC, WILL, COM_PORT_OPTION];
    assert_eq!(replies(&p.feed(&client)), [IAC, WONT, 1, IAC, DO, SGA, IAC, WILL, SGA, IAC, WILL, COM_PORT_OPTION, IAC, DO, COM_PORT_OPTION]);
    // The client's acceptance of BINARY (answers to the greeting): acknowledged, not answered.
    assert!(p.feed(&[IAC, DO, BINARY, IAC, WILL, BINARY]).is_empty());
    // Repeats are not answered again; an unknown option is refused.
    assert!(p.feed(&[IAC, WILL, COM_PORT_OPTION, IAC, DO, SGA]).is_empty());
    assert_eq!(replies(&p.feed(&[IAC, WILL, 24])), [IAC, DONT, 24]);
    // Turning an agreed option off is confirmed once.
    assert_eq!(replies(&p.feed(&[IAC, DONT, SGA])), [IAC, WONT, SGA]);
    assert!(p.feed(&[IAC, DONT, SGA]).is_empty());
}

#[test]
fn set_control_moves_the_lines_in_order_with_the_data_and_is_acknowledged() {
    let mut p = ComPort::new();
    let mut input = vec![0x41];
    input.extend(sb(SET_CONTROL, &[8]));        // DTR on
    input.push(0x42);
    input.extend(sb(SET_CONTROL, &[11]));       // RTS on
    input.extend(sb(SET_CONTROL, &[9]));        // DTR off: (RTS=1, DTR=0)
    input.extend(sb(SET_CONTROL, &[7]));        // what is DTR?
    input.extend(sb(SET_CONTROL, &[12]));       // RTS off
    input.push(0x43);
    let a = p.feed(&input);
    assert_eq!(a, [
        Action::Data(vec![0x41]),
        Action::Lines { dtr: true, rts: false }, Action::Reply(sb(105, &[8])),
        Action::Data(vec![0x42]),
        Action::Lines { dtr: true, rts: true }, Action::Reply(sb(105, &[11])),
        Action::Lines { dtr: false, rts: true }, Action::Reply(sb(105, &[9])),
        Action::Reply(sb(105, &[9])),
        Action::Lines { dtr: false, rts: false }, Action::Reply(sb(105, &[12])),
        Action::Data(vec![0x43]),
    ]);
    // pyserial's _reconfigure_port (rfc2217.py:499-541): flow control "none" is echoed; a
    // request for hardware flow control is answered with the setting in use, which pyserial
    // reads as a refusal (check_answer compares the value).
    assert_eq!(replies(&p.feed(&sb(SET_CONTROL, &[1]))), sb(105, &[1]));
    assert_eq!(replies(&p.feed(&sb(SET_CONTROL, &[3]))), sb(105, &[1]));
    assert_eq!(replies(&p.feed(&sb(SET_CONTROL, &[5]))), sb(105, &[5]), "BREAK recorded");
    assert_eq!(replies(&p.feed(&sb(SET_CONTROL, &[4]))), sb(105, &[5]));
}

#[test]
fn port_settings_echo_and_iac_in_a_value_is_escaped() {
    let mut p = ComPort::new();
    // 115200 = 0x0001C200, then a rate whose value holds a 0xFF byte.
    assert_eq!(replies(&p.feed(&sb(SET_BAUDRATE, &115_200u32.to_be_bytes()))), sb(101, &115_200u32.to_be_bytes()));
    let odd = 0x0001_ffffu32;
    let req = sb(SET_BAUDRATE, &odd.to_be_bytes());
    assert_eq!(req, [IAC, SB, COM_PORT_OPTION, 1, 0, 1, IAC, IAC, IAC, IAC, IAC, SE]);
    // Split anywhere, the answer is the same.
    let (a, b) = req.split_at(7);
    assert!(p.feed(a).is_empty());
    assert_eq!(replies(&p.feed(b)), sb(101, &odd.to_be_bytes()));
    assert_eq!(p.baud, odd);
    assert_eq!(replies(&p.feed(&sb(SET_BAUDRATE, &[0, 0, 0, 0]))), sb(101, &odd.to_be_bytes()), "0 asks for the current rate");
    assert_eq!(replies(&p.feed(&sb(SET_DATASIZE, &[8]))), sb(102, &[8]));
    assert_eq!(replies(&p.feed(&sb(SET_PARITY, &[1]))), sb(103, &[1]));
    assert_eq!(replies(&p.feed(&sb(SET_STOPSIZE, &[1]))), sb(104, &[1]));
    assert_eq!(replies(&p.feed(&sb(PURGE_DATA, &[1]))), sb(112, &[1]));
    assert_eq!(replies(&p.feed(&sb(NOTIFY_MODEMSTATE, &[]))), sb(107, &[0]));
    assert_eq!(replies(&p.feed(&sb(SET_MODEMSTATE_MASK, &[0]))), sb(111, &[0]));
    assert_eq!(replies(&p.feed(&sb(SIGNATURE, &[]))), sb(SIGNATURE, b"esp32sim USB-Serial/JTAG"));
}

#[test]
fn a_socket_client_reaches_the_port_and_a_second_is_refused() {
    let usj = UsjPort::new();
    let port = start(0, usj.clone()).unwrap();
    let mut c = TcpStream::connect(("127.0.0.1", port)).unwrap();
    c.set_read_timeout(Some(Duration::from_secs(3))).unwrap();
    let mut greeting = [0u8; 6];
    c.read_exact(&mut greeting).unwrap();
    assert_eq!(greeting, [IAC, WILL, BINARY, IAC, DO, BINARY]);
    // Wait for the claim, then take the Open.
    let mut ev = Vec::new();
    for _ in 0..300 { ev.extend(usj.poll()); if !ev.is_empty() { break; } std::thread::sleep(Duration::from_millis(10)); }
    let PortIn::Open(id) = ev[0] else { panic!("{ev:?}") };
    let mut second = TcpStream::connect(("127.0.0.1", port)).unwrap();
    second.set_read_timeout(Some(Duration::from_secs(3))).unwrap();
    let mut refusal = String::new();
    second.read_to_string(&mut refusal).unwrap();
    assert!(refusal.contains("held by another client"), "{refusal}");
    // Client -> chip: 0xff doubled on the wire arrives once; lines follow in order.
    let mut out = vec![0xc0, IAC, IAC, 0x00];
    out.extend(sb(SET_CONTROL, &[11]));
    c.write_all(&out).unwrap();
    let mut got = Vec::new();
    for _ in 0..300 { got.extend(usj.poll()); if got.len() >= 2 { break; } std::thread::sleep(Duration::from_millis(10)); }
    assert_eq!(got, [PortIn::Data(vec![0xc0, 0xff, 0x00]), PortIn::Lines { dtr: false, rts: true }]);
    let mut ack = vec![0u8; sb(105, &[11]).len()];
    c.read_exact(&mut ack).unwrap();
    assert_eq!(ack, sb(105, &[11]));
    // Chip -> client: every byte value, 0xff doubled on the wire.
    let all: Vec<u8> = (0..=255u8).collect();
    assert!(usj.deliver(id, PortOut::Data(all.clone())));
    let mut wire = vec![0u8; 257];
    c.read_exact(&mut wire).unwrap();
    assert_eq!(wire, escape(&all));
    drop(c);
    let mut closed = Vec::new();
    for _ in 0..300 { closed.extend(usj.poll()); if !closed.is_empty() { break; } std::thread::sleep(Duration::from_millis(10)); }
    assert_eq!(closed, [PortIn::Close(id)]);
    assert!(!usj.held());
}
