//! The USB-Serial/JTAG as an RFC 2217 serial port on a TCP port of 127.0.0.1 (`--serial-tcp`),
//! for tools that speak pyserial's `rfc2217://host:port`: esptool, idf.py monitor, miniterm.
//!
//! Only as much of RFC 2217 ("Telnet Com Port Control Option", October 1997) as a serial port
//! whose baud rate means nothing needs:
//!
//! - Telnet: the server offers and asks for BINARY both ways (RFC 856) and accepts SGA and
//!   COM-PORT-OPTION (44) from either side (RFC 2217 §1, p.5: "Typically a client will use WILL
//!   and WONT, while an access server will use DO and DONT"); every other option is refused. The
//!   stream is 8-bit whatever was agreed: pyserial's client escapes only IAC (rfc2217.py:628-641,
//!   pyserial 3.5), and data byte 255 is doubled both ways (RFC 856 §5, p.2).
//! - Com port commands (RFC 2217 §2-3, pp.6-11): each is answered with its code + 100 and the
//!   value in use, which is what pyserial waits for before it goes on (rfc2217.py:350-372
//!   `wait`/`check_answer`: the answer must repeat the requested value). SET-BAUDRATE,
//!   SET-DATASIZE, SET-PARITY and SET-STOPSIZE are stored and echoed: the USB-Serial/JTAG accepts
//!   and ignores SET_LINE_CODING (ESP32-S3 TRM v1.8 Table 33.3-1, p.1247). SET-CONTROL 8/9 and
//!   11/12 set DTR and RTS, which go to the chip in order with the data (`UsjLines`, TRM Table
//!   33.3-2); 7 and 10 report them; flow control is always "none" (1 outbound, 14 inbound);
//!   BREAK is recorded and ignored, as the controller ignores SEND_BREAK (Table 33.3-1).
//!   SET-LINESTATE-MASK and SET-MODEMSTATE-MASK are stored and echoed. PURGE-DATA is echoed:
//!   the server keeps no buffer of its own to clear (device output goes straight to the socket,
//!   and pyserial empties its own queue after the answer, rfc2217.py:643-649).
//! - A client poll of NOTIFY-LINESTATE / NOTIFY-MODEMSTATE is answered with 0, as pyserial's own
//!   server does (rfc2217.py:1287-1296); the modem inputs of the USB-Serial/JTAG are always 0 (its
//!   interrupt endpoint never reports, TRM §33.3.1 p.1247). SIGNATURE with no text gets the name.
//! - Not done: FLOWCONTROL-SUSPEND/RESUME (§5, p.13) are ignored — pyserial's client never sends
//!   them (its `rfc2217_flow_server_ready` is a stub, rfc2217.py:890-897).
//!
//! One TCP connection is one `usj_port::Session`; a second client while one holds the port gets a
//! line of text and the connection closed.
use crate::usj_port::{PortOut, Session, UsjPort};
use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};

pub const IAC: u8 = 255;
pub const DONT: u8 = 254;
pub const DO: u8 = 253;
pub const WONT: u8 = 252;
pub const WILL: u8 = 251;
pub const SB: u8 = 250;
pub const SE: u8 = 240;
pub const BINARY: u8 = 0;
pub const SGA: u8 = 3;
pub const COM_PORT_OPTION: u8 = 44;
/// Client-to-server command codes (RFC 2217 p.5); the server answers with code + 100.
pub const SIGNATURE: u8 = 0;
pub const SET_BAUDRATE: u8 = 1;
pub const SET_DATASIZE: u8 = 2;
pub const SET_PARITY: u8 = 3;
pub const SET_STOPSIZE: u8 = 4;
pub const SET_CONTROL: u8 = 5;
pub const NOTIFY_LINESTATE: u8 = 6;
pub const NOTIFY_MODEMSTATE: u8 = 7;
pub const SET_LINESTATE_MASK: u8 = 10;
pub const SET_MODEMSTATE_MASK: u8 = 11;
pub const PURGE_DATA: u8 = 12;

/// What the connection has to do with a piece of client input, in order.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Action {
    /// bytes for the socket (negotiation, answers)
    Reply(Vec<u8>),
    /// bytes for the chip
    Data(Vec<u8>),
    /// the host's full line state after a SET-CONTROL
    Lines { dtr: bool, rts: bool },
}

#[derive(Clone, Copy, Default)]
struct Opt { us: bool, us_pending: bool, him: bool, him_pending: bool }

#[derive(Clone, Copy, PartialEq, Eq)]
enum Mode { Data, Iac, Neg(u8), Sub, SubIac }

/// One connection's Telnet and com port state.
pub struct ComPort {
    mode: Mode,
    sub: Vec<u8>,
    opts: [Opt; 256],
    pub dtr: bool,
    pub rts: bool,
    pub baud: u32,
    datasize: u8,
    parity: u8,
    stopsize: u8,
    brk: bool,
    linestate_mask: u8,
    modemstate_mask: u8,
}

impl Default for ComPort { fn default() -> Self { Self::new() } }

/// `IAC SB COM-PORT-OPTION code value IAC SE`, with IAC in the value doubled.
pub fn sb(code: u8, value: &[u8]) -> Vec<u8> {
    let mut v = vec![IAC, SB, COM_PORT_OPTION, code];
    v.extend_from_slice(&escape(value));
    v.extend_from_slice(&[IAC, SE]);
    v
}

/// Data byte 255 is sent as IAC IAC (RFC 856 §5).
pub fn escape(data: &[u8]) -> Vec<u8> {
    let mut v = Vec::with_capacity(data.len() + data.len() / 64);
    for &b in data { v.push(b); if b == IAC { v.push(IAC); } }
    v
}

impl ComPort {
    /// Lines start deasserted: the port the machine hands out is at DTR = RTS = 0.
    pub fn new() -> Self {
        ComPort { mode: Mode::Data, sub: Vec::new(), opts: [Opt::default(); 256], dtr: false, rts: false, baud: 115_200, datasize: 8, parity: 1, stopsize: 1, brk: false, linestate_mask: 0, modemstate_mask: 255 }
    }

    /// The server's opening: WILL BINARY, DO BINARY (RFC 856).
    pub fn greeting(&mut self) -> Vec<u8> {
        self.opts[BINARY as usize].us_pending = true;
        self.opts[BINARY as usize].him_pending = true;
        vec![IAC, WILL, BINARY, IAC, DO, BINARY]
    }

    fn accepts(opt: u8) -> bool { matches!(opt, BINARY | SGA | COM_PORT_OPTION) }

    /// Client bytes in; what to do, in order.
    pub fn feed(&mut self, input: &[u8]) -> Vec<Action> {
        let mut out: Vec<Action> = Vec::new();
        let mut data: Vec<u8> = Vec::new();
        let flush = |data: &mut Vec<u8>, out: &mut Vec<Action>| if !data.is_empty() { out.push(Action::Data(std::mem::take(data))); };
        for &b in input {
            match self.mode {
                Mode::Data => if b == IAC { self.mode = Mode::Iac } else { data.push(b) },
                Mode::Iac => match b {
                    IAC => { data.push(IAC); self.mode = Mode::Data; }
                    SB => { self.sub.clear(); self.mode = Mode::Sub; }
                    WILL | WONT | DO | DONT => self.mode = Mode::Neg(b),
                    _ => self.mode = Mode::Data,     // NOP, GA, a stray SE ...: nothing to do (RFC 856 §5)
                },
                Mode::Neg(cmd) => {
                    self.mode = Mode::Data;
                    if let Some(r) = self.negotiate(cmd, b) { flush(&mut data, &mut out); out.push(Action::Reply(r)); }
                }
                Mode::Sub => if b == IAC { self.mode = Mode::SubIac } else { self.sub.push(b) },
                Mode::SubIac => match b {
                    IAC => { self.sub.push(IAC); self.mode = Mode::Sub; }
                    _ => {                           // IAC SE ends it; anything else ends a malformed one
                        self.mode = Mode::Data;
                        let sub = std::mem::take(&mut self.sub);
                        flush(&mut data, &mut out);
                        self.subnegotiation(&sub, &mut out);
                    }
                },
            }
        }
        flush(&mut data, &mut out);
        out
    }

    /// A DO/DONT/WILL/WONT from the client; the answer, if one is due. A request we made is
    /// acknowledged by the matching answer and not answered again (no negotiation loops, RFC 854).
    fn negotiate(&mut self, cmd: u8, opt: u8) -> Option<Vec<u8>> {
        let o = &mut self.opts[opt as usize];
        let reply = |c: u8| Some(vec![IAC, c, opt]);
        match cmd {
            WILL => {
                if o.him { None }
                else if o.him_pending { o.him = true; o.him_pending = false; None }
                else if Self::accepts(opt) { o.him = true; reply(DO) }
                else { reply(DONT) }
            }
            WONT => {
                let was_on = o.him && !o.him_pending;
                o.him = false; o.him_pending = false;
                if was_on { reply(DONT) } else { None }
            }
            DO => {
                if o.us { None }
                else if o.us_pending { o.us = true; o.us_pending = false; None }
                else if Self::accepts(opt) { o.us = true; reply(WILL) }
                else { reply(WONT) }
            }
            _ => {                                   // DONT
                let was_on = o.us && !o.us_pending;
                o.us = false; o.us_pending = false;
                if was_on { reply(WONT) } else { None }
            }
        }
    }

    fn subnegotiation(&mut self, sub: &[u8], out: &mut Vec<Action>) {
        let [COM_PORT_OPTION, code, value @ ..] = sub else { return };
        let answer = |out: &mut Vec<Action>, v: &[u8]| out.push(Action::Reply(sb(code + 100, v)));
        match *code {
            SIGNATURE => if value.is_empty() { out.push(Action::Reply(sb(SIGNATURE, b"esp32sim USB-Serial/JTAG"))); },
            SET_BAUDRATE => {
                if let Ok(v) = <[u8; 4]>::try_from(value.get(..4).unwrap_or(&[])) { let b = u32::from_be_bytes(v); if b != 0 { self.baud = b; } }
                answer(out, &self.baud.to_be_bytes());
            }
            SET_DATASIZE => { if let Some(&v) = value.first() { if (5..=8).contains(&v) { self.datasize = v; } } answer(out, &[self.datasize]); }
            SET_PARITY => { if let Some(&v) = value.first() { if (1..=5).contains(&v) { self.parity = v; } } answer(out, &[self.parity]); }
            SET_STOPSIZE => { if let Some(&v) = value.first() { if (1..=3).contains(&v) { self.stopsize = v; } } answer(out, &[self.stopsize]); }
            SET_CONTROL => {
                let Some(&v) = value.first() else { return };
                let lines = |s: &mut Self, out: &mut Vec<Action>| out.push(Action::Lines { dtr: s.dtr, rts: s.rts });
                let now = match v {
                    0..=3 | 17..=19 => 1,                               // flow control: none, whatever was asked
                    4 => if self.brk { 5 } else { 6 },
                    5 | 6 => { self.brk = v == 5; v }
                    7 => if self.dtr { 8 } else { 9 },
                    8 | 9 => { self.dtr = v == 8; lines(self, out); v }
                    10 => if self.rts { 11 } else { 12 },
                    11 | 12 => { self.rts = v == 11; lines(self, out); v }
                    13..=16 => 14,                                      // inbound flow control: none
                    _ => return,
                };
                answer(out, &[now]);
            }
            NOTIFY_LINESTATE | NOTIFY_MODEMSTATE => answer(out, &[0]),
            SET_LINESTATE_MASK => { if let Some(&v) = value.first() { self.linestate_mask = v; } answer(out, &[self.linestate_mask]); }
            SET_MODEMSTATE_MASK => { if let Some(&v) = value.first() { self.modemstate_mask = v; } answer(out, &[self.modemstate_mask]); }
            PURGE_DATA => if let Some(&v) = value.first() { if (1..=3).contains(&v) { answer(out, &[v]); } },
            _ => {}
        }
    }
}

/// Listen on 127.0.0.1:`port` (0 picks one); returns the port. Each connection is served on its own threads.
pub fn start(port: u16, usj: UsjPort) -> std::io::Result<u16> {
    let listener = TcpListener::bind(("127.0.0.1", port))?;
    let port = listener.local_addr()?.port();
    std::thread::spawn(move || {
        for stream in listener.incoming().flatten() {
            let usj = usj.clone();
            std::thread::spawn(move || serve(stream, usj));
        }
    });
    Ok(port)
}

/// One client: claim the port, negotiate, then carry bytes both ways until either side closes.
pub fn serve(mut stream: TcpStream, usj: UsjPort) {
    let peer = stream.peer_addr().map(|a| a.to_string()).unwrap_or_default();
    let _ = stream.set_nodelay(true);
    let Ok(mut out) = stream.try_clone() else { return };
    let (tx, rx) = std::sync::mpsc::channel::<Vec<u8>>();
    let sink_tx = tx.clone();
    let session: Session = match usj.open(Box::new(move |o| match o {
        PortOut::Data(d) => sink_tx.send(escape(&d)).is_ok(),
        PortOut::Event(_) => true,     // the page's JSON events; the engine's log line says the same
    })) {
        Ok(s) => s,
        Err(_) => {
            eprintln!("[emu] rfc2217: {peer} refused: the USB-Serial/JTAG port is held by another client");
            let _ = stream.write_all(b"esp32sim: the USB-Serial/JTAG port is held by another client\r\n");
            return;
        }
    };
    eprintln!("[emu] rfc2217: {peer} holds the USB-Serial/JTAG port (session {})", session.id());
    let writer = std::thread::spawn(move || { for b in rx { if out.write_all(&b).is_err() { break; } } });
    let mut proto = ComPort::new();
    let _ = tx.send(proto.greeting());
    let (mut n_in, mut buf) = (0u64, vec![0u8; 65536]);
    loop {
        let n = match stream.read(&mut buf) { Ok(0) | Err(_) => break, Ok(n) => n };
        for a in proto.feed(&buf[..n]) {
            match a {
                Action::Reply(r) => { let _ = tx.send(r); }
                Action::Data(d) => { n_in += d.len() as u64; session.data(d); }
                Action::Lines { dtr, rts } => session.lines(dtr, rts),
            }
        }
    }
    eprintln!("[emu] rfc2217: {peer} closed the port after {n_in} bytes to the chip");
    session.close();
    drop(tx);
    let _ = stream.shutdown(std::net::Shutdown::Both);
    let _ = writer.join();
}

#[cfg(test)]
mod tests;
