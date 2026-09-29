//! The host end of the USB-Serial/JTAG: one virtual serial port that at most one client holds
//! open at a time, carried without loss between the machine and its front ends — the board UI's
//! `/usj` WebSocket (`web.rs`, for the browser flasher) and an RFC 2217 server on a TCP port
//! (`rfc2217.rs`, for esptool and pyserial).
//!
//! A front end claims the port with `UsjPort::open`; while another client holds it the claim fails
//! with `Busy`, as a second open of an exclusively opened serial device does. The claim is a
//! `Session`. Its bytes and DTR/RTS changes queue for the machine in the order the client made them,
//! with no bound and no drop; the machine takes them at its next service point
//! (`Machine::usj_service`), feeds the bytes to the controller's receive side in 64-byte USB
//! packets (`UsbSerialJtag::host_input`) and hands the lines to the chip
//! (`SocBus::usj_lines`). Closing or dropping the session queues `Close`, which the machine
//! applies as DTR = RTS = 0 — so a chip held in reset (RTS=1, DTR=0) is let go when its host goes
//! away — before it forgets the session.
//!
//! The controller's transmit side goes to the session's sink only between the machine's handling of
//! that session's `Open` and its `Close`, matched by session id: output of one session never reaches
//! the next, and while nobody holds the port the output stays on the console as before. A client
//! that closes the port (`Session::close`) keeps its sink until the machine has handled that `Close`
//! (`UsjPort::finish`), so what the chip says in answer to the bytes and lines sent before the close
//! - a `release` event, the last output - still reaches it. A client that is gone (`abandon`, or a
//! dropped session) loses its sink at once: nobody is left to read it.
use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

/// From the client, in the order it acted.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PortIn {
    /// Session `id` holds the port from here on.
    Open(u64),
    /// Bytes for the controller's receive side (USB OUT).
    Data(Vec<u8>),
    /// The full line state after one change.
    Lines { dtr: bool, rts: bool },
    /// Session `id` let the port go.
    Close(u64),
}

/// To the client.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PortOut {
    /// The controller's transmit side (USB IN), byte for byte.
    Data(Vec<u8>),
    /// A JSON event for the page (`hello`, `reset`, `release`); a raw-byte front end may drop it.
    Event(String),
}

/// Where a session's output goes. False once the client is gone.
pub type Sink = Box<dyn FnMut(PortOut) -> bool + Send>;

/// The port is held by another client.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Busy;

struct Inner {
    /// the session holding the port, 0 for none
    owner: u64,
    next_id: u64,
    inbox: VecDeque<PortIn>,
    /// each session's output, from its open until the machine is done with its close (`finish`) or
    /// its client is gone; two only while a closed session's `Close` waits behind the next `Open`
    sinks: Vec<(u64, Sink)>,
}

/// Cloneable handle; every clone is the same port.
#[derive(Clone)]
pub struct UsjPort(Arc<Mutex<Inner>>);

impl Default for UsjPort { fn default() -> Self { Self::new() } }

impl UsjPort {
    pub fn new() -> Self { UsjPort(Arc::new(Mutex::new(Inner { owner: 0, next_id: 1, inbox: VecDeque::new(), sinks: Vec::new() }))) }

    /// Claim the port; the machine's output for the new session goes to `sink`.
    pub fn open(&self, sink: Sink) -> Result<Session, Busy> {
        let mut i = self.0.lock().unwrap();
        if i.owner != 0 { return Err(Busy); }
        let id = i.next_id;
        i.next_id += 1;
        i.owner = id;
        i.sinks.push((id, sink));
        i.inbox.push_back(PortIn::Open(id));
        Ok(Session { id, port: self.clone(), closed: false })
    }

    /// Is some client holding the port?
    pub fn held(&self) -> bool { self.0.lock().unwrap().owner != 0 }

    /// Machine side: everything the clients did since the last call, in order.
    pub fn poll(&self) -> VecDeque<PortIn> { std::mem::take(&mut self.0.lock().unwrap().inbox) }

    /// Machine side: output for session `id`. False (and dropped) when that session has no sink any
    /// more: the machine finished its close, or its client is gone.
    pub fn deliver(&self, id: u64, out: PortOut) -> bool {
        let mut i = self.0.lock().unwrap();
        let Some(k) = i.sinks.iter().position(|(sid, _)| *sid == id) else { return false };
        if (i.sinks[k].1)(out) { true } else { drop(i.sinks.remove(k)); false }
    }

    /// Machine side: session `id`'s `Close` is handled; nothing more goes to its client.
    pub fn finish(&self, id: u64) { self.0.lock().unwrap().sinks.retain(|(sid, _)| *sid != id); }

    fn push(&self, id: u64, what: PortIn) {
        let mut i = self.0.lock().unwrap();
        if i.owner == id { i.inbox.push_back(what); }
    }

    /// Free the port for the next client; `keep`: the sink stays until the machine's `finish`.
    fn release(&self, id: u64, keep: bool) {
        let mut i = self.0.lock().unwrap();
        if i.owner != id { return; }
        i.owner = 0;
        if !keep { i.sinks.retain(|(sid, _)| *sid != id); }
        i.inbox.push_back(PortIn::Close(id));
    }
}

/// One client's hold on the port. Dropping it closes the port.
pub struct Session { id: u64, port: UsjPort, closed: bool }

impl Session {
    pub fn id(&self) -> u64 { self.id }
    /// Bytes for the controller's receive side.
    pub fn data(&self, bytes: Vec<u8>) { if !bytes.is_empty() { self.port.push(self.id, PortIn::Data(bytes)); } }
    /// The line state after one change (DTR, RTS as the host asserts them: true = active).
    pub fn lines(&self, dtr: bool, rts: bool) { self.port.push(self.id, PortIn::Lines { dtr, rts }); }
    /// Let the port go, in order: the port is free for another client at once, and the output the
    /// machine makes up to and while handling this close still reaches this session.
    pub fn close(mut self) { self.closed = true; self.port.release(self.id, true); }
    /// The client is gone: let the port go, and no output reaches this session afterwards.
    pub fn abandon(mut self) { self.closed = true; self.port.release(self.id, false); }
}

impl Drop for Session {
    fn drop(&mut self) { if !self.closed { self.port.release(self.id, false); } }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::mpsc;

    fn sink() -> (Sink, mpsc::Receiver<PortOut>) {
        let (tx, rx) = mpsc::channel();
        (Box::new(move |o| tx.send(o).is_ok()), rx)
    }

    #[test]
    fn one_holder_at_a_time_and_order_is_kept() {
        let port = UsjPort::new();
        let (s1, rx1) = sink();
        let a = port.open(s1).unwrap();
        let (s2, _rx2) = sink();
        assert!(matches!(port.open(s2), Err(Busy)), "a second client is refused while the first holds the port");
        a.data(vec![0xc0, 0xff, 0x00]);
        a.lines(true, false);
        a.data(vec![0x80]);
        let id = a.id();
        assert_eq!(Vec::from(port.poll()), [PortIn::Open(id), PortIn::Data(vec![0xc0, 0xff, 0x00]), PortIn::Lines { dtr: true, rts: false }, PortIn::Data(vec![0x80])]);
        assert!(port.deliver(id, PortOut::Data(vec![0xfe, 0xff])));
        assert_eq!(rx1.try_recv().unwrap(), PortOut::Data(vec![0xfe, 0xff]));
        drop(a);
        assert_eq!(Vec::from(port.poll()), [PortIn::Close(id)]);
        assert!(!port.deliver(id, PortOut::Data(vec![1])), "a closed session gets nothing");
        let (s3, rx3) = sink();
        let b = port.open(s3).unwrap();
        assert_ne!(b.id(), id);
        assert!(!port.deliver(id, PortOut::Data(vec![2])), "the old session's output never reaches the new one");
        assert!(rx3.try_recv().is_err());
        b.close();
        assert!(!port.held());
    }

    #[test]
    fn a_close_keeps_what_answers_the_frames_before_it() {
        let port = UsjPort::new();
        let (s, rx) = sink();
        let a = port.open(s).unwrap();
        let id = a.id();
        a.lines(false, false);
        a.close();
        assert!(!port.held(), "the next client may claim the port at once");
        let (s2, rx2) = sink();
        let b = port.open(s2).unwrap();
        assert_eq!(Vec::from(port.poll()), [PortIn::Open(id), PortIn::Lines { dtr: false, rts: false }, PortIn::Close(id), PortIn::Open(b.id())]);
        // The machine applies the lines, then the close: both answers still reach the first client.
        assert!(port.deliver(id, PortOut::Event("{\"t\":\"release\",\"strap\":15}".into())));
        assert!(port.deliver(id, PortOut::Event("{\"t\":\"closed\"}".into())));
        port.finish(id);
        assert!(!port.deliver(id, PortOut::Data(vec![1])), "nothing after the machine is done with the close");
        assert_eq!(rx.try_iter().collect::<Vec<_>>(), [PortOut::Event("{\"t\":\"release\",\"strap\":15}".into()), PortOut::Event("{\"t\":\"closed\"}".into())]);
        assert!(port.deliver(b.id(), PortOut::Data(vec![2])));
        assert_eq!(rx2.try_iter().collect::<Vec<_>>(), [PortOut::Data(vec![2])]);
        let bid = b.id();
        b.abandon();
        assert!(!port.deliver(bid, PortOut::Data(vec![3])), "a client that is gone loses its sink at once");
    }

    #[test]
    fn nothing_is_dropped_under_load() {
        let port = UsjPort::new();
        let (s, rx) = sink();
        let a = port.open(s).unwrap();
        let id = a.id();
        let _ = port.poll();
        for i in 0..10_000u32 { a.data(i.to_le_bytes().to_vec()); assert!(port.deliver(id, PortOut::Data(i.to_be_bytes().to_vec()))); }
        let got: Vec<u8> = port.poll().into_iter().flat_map(|p| match p { PortIn::Data(d) => d, _ => panic!() }).collect();
        assert_eq!(got, (0..10_000u32).flat_map(|i| i.to_le_bytes()).collect::<Vec<_>>());
        let out: Vec<u8> = rx.try_iter().flat_map(|o| match o { PortOut::Data(d) => d, _ => panic!() }).collect();
        assert_eq!(out, (0..10_000u32).flat_map(|i| i.to_be_bytes()).collect::<Vec<_>>());
    }
}
