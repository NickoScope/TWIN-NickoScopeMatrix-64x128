//! The USB host at the USB-Serial/JTAG: the client holding `Machine::usj` (the `/usj` WebSocket,
//! the RFC 2217 port) and script lines drive the controller's receive side and its DTR/RTS; the
//! chip's reaction to the lines is the bus's (`SocBus::usj_lines`, TRM v1.8 Table 33.3-2).
//!
//! A line change that resets the chip stops the service: what the client sent after it waits
//! until the reset has happened, so bytes sent after the reset reach the new controller and not
//! the old one the reset throws away. While the lines hold the chip in reset (RTS=1, DTR=0)
//! `hold_in_reset` lets time pass with the cores stopped.
use super::*;
use crate::usj_port::{PortIn, PortOut};
use std::collections::VecDeque;

/// How often the machine looks at the port: every millisecond of device time while a client
/// holds it — the USB frame period the controller model already keeps (SOF, `usb_serial_jtag.rs`)
/// and far inside the 100 ms esptool gives a SYNC answer (esptool 4.9.0 loader.py:86
/// SYNC_TIMEOUT; esptool-js 0.6.0 esploader.ts:503) — and every 10 ms while nobody does. A design choice,
/// not a figure from a source.
const OPEN_POLL_HZ: u64 = 1000;
const IDLE_POLL_HZ: u64 = 100;

pub(super) struct UsjState {
    /// the session the machine has seen open and not yet closed
    pub(super) session: Option<u64>,
    /// client actions not applied yet (after a reset request, until the reboot)
    pending: VecDeque<PortIn>,
    /// device cycle of the next service; `u64::MAX` without a port
    pub(super) next: u64,
    /// a line reset was requested and the reboot has not happened yet
    pub(super) reset_pending: bool,
    pub(super) bytes_in: u64,
    pub(super) bytes_out: u64,
}

impl Default for UsjState {
    fn default() -> Self { UsjState { session: None, pending: VecDeque::new(), next: u64::MAX, reset_pending: false, bytes_in: 0, bytes_out: 0 } }
}

impl<S: Soc> Machine<S> {
    /// Serve `port` as the USB-Serial/JTAG's host end.
    pub fn attach_usj(&mut self, port: crate::usj_port::UsjPort) {
        self.usj = Some(port);
        self.usj_state.next = 0;
    }

    /// Is a client holding the port, as far as the machine has seen?
    pub fn usj_session(&self) -> Option<u64> { self.usj_state.session }

    /// The host lines hold the chip in reset now.
    pub fn usj_held(&self) -> bool { self.usj_reset && self.bus.usj_line_state().is_some_and(|l| l.held) }

    fn usj_event(&self, json: &str) {
        if let (Some(id), Some(port)) = (self.usj_state.session, &self.usj) { port.deliver(id, PortOut::Event(json.to_string())); }
    }

    /// The USB host set DTR and RTS. True if that requested a chip reset: the caller must let the
    /// reset happen before it applies anything the host did later.
    pub fn usj_lines(&mut self, dtr: bool, rts: bool) -> bool {
        use esp_periph::LineEffect;
        match self.bus.usj_lines(dtr, rts) {
            LineEffect::None => false,
            LineEffect::AssertReset { download } => {
                if !self.usj_reset {
                    eprintln!("[emu] usj: t={:.3}s RTS=1 DTR=0 ignored: this run does not come back up through the ROM (--boot app or --no-reboot)", self.seconds());
                    return false;
                }
                eprintln!("[emu] usj: t={:.3}s RTS=1 DTR=0 -> chip reset {:#x} ({}), download mode flag {}, held until the lines change",
                          self.seconds(), esp_periph::RST_USB_UART_CHIP, esp_periph::reset_cause_name(esp_periph::RST_USB_UART_CHIP), download as u8);
                self.usj_event(&format!("{{\"t\":\"reset\",\"cause\":{},\"download\":{}}}", esp_periph::RST_USB_UART_CHIP, download));
                self.bus.request_reset(esp_periph::RST_USB_UART_CHIP);
                *self.bus.irq_dirty() = true;
                self.usj_state.reset_pending = true;
                true
            }
            LineEffect::ReleaseReset => {
                if !self.usj_reset { return false; }
                let strap = self.bus.strap();
                eprintln!("[emu] usj: t={:.3}s RTS={} DTR={} -> out of reset{}", self.seconds(), rts as u8, dtr as u8,
                          strap.map_or(String::new(), |v| format!(", GPIO_STRAPPING {v:#x} ({})", if v & 0x0c == 0 { "download" } else if v & 0x08 != 0 { "SPI boot" } else { "other boot" })));
                self.usj_event(&format!("{{\"t\":\"release\",\"strap\":{}}}", strap.map_or("null".to_string(), |v| v.to_string())));
                false
            }
        }
    }

    /// Take what clients did since the last call and apply it in order; send the device's
    /// output to the client holding the port. Called at run entry, with the page's input, and
    /// from the scheduling rounds (`after_round_rest`) at the poll rate. Out of line: the round's
    /// own cost stays one compare.
    #[inline(never)]
    pub(super) fn usj_service(&mut self) {
        let Some(port) = self.usj.clone() else { return };
        self.usj_state.pending.extend(port.poll());
        let hz = if self.usj_state.session.is_some() || !self.usj_state.pending.is_empty() { OPEN_POLL_HZ } else { IDLE_POLL_HZ };
        self.usj_state.next = self.bus.cycles() + S::CPU_HZ / hz;
        if self.usj_state.reset_pending { return; }
        while let Some(ev) = self.usj_state.pending.pop_front() {
            match ev {
                PortIn::Open(id) => {
                    self.drain_console();                       // what came before belongs to the console
                    self.usj_state.session = Some(id);
                    self.usj_state.bytes_in = 0; self.usj_state.bytes_out = 0;
                    let l = self.bus.usj_line_state().unwrap_or_default();
                    self.usj_event(&format!("{{\"t\":\"hello\",\"proto\":1,\"chip\":\"{}\",\"dtr\":{},\"rts\":{},\"held\":{},\"reset\":{}}}",
                                            S::NAME, l.dtr as u8, l.rts as u8, self.usj_held(), self.usj_reset));
                    if let Some(w) = &self.web { w.send_text("{\"t\":\"usj\",\"claimed\":true}"); }
                }
                PortIn::Data(d) => { self.usj_state.bytes_in += d.len() as u64; self.bus.serial_input(&d); }
                PortIn::Lines { dtr, rts } => if self.usj_lines(dtr, rts) { break; },
                PortIn::Close(id) => {
                    self.drain_console();                       // the session's last output
                    // A host that goes away drops its lines: a chip held in reset runs again, and
                    // the session still hears its `release`. (0, 0) never asks for a reset.
                    self.usj_lines(false, false);
                    if self.usj_state.session == Some(id) {
                        eprintln!("[emu] usj: t={:.3}s port closed ({} bytes to the chip, {} from it)", self.seconds(), self.usj_state.bytes_in, self.usj_state.bytes_out);
                        self.usj_event("{\"t\":\"closed\"}");
                        self.usj_state.session = None;
                        if let Some(w) = &self.web { w.send_text("{\"t\":\"usj\",\"claimed\":false}"); }
                    }
                    port.finish(id);
                }
            }
        }
        if self.usj_state.session.is_some() { self.drain_console(); }
    }

    /// The chip went through reset: what waited for it may be applied now.
    pub(super) fn usj_after_reboot(&mut self) { self.usj_state.reset_pending = false; }

    /// While the host holds the chip in reset (RTS=1, DTR=0) the cores do not run and device time
    /// goes on in 1 ms steps: devices tick, scripts, the page and the host channels are served,
    /// real-time pacing holds (without it, each step also sleeps 1 ms of wall time, so waiting for
    /// a host does not spin). Returns when the lines let go, the session closes, another reset is
    /// requested (the page's RESET), a script stops the run or the run's time is up.
    pub(super) fn hold_in_reset(&mut self) -> Option<Stop> {
        let step = S::CPU_HZ / 1000;
        while self.usj_held() {
            if self.bus.sw_reset() { self.drain_console(); return Some(Stop::SwReset); }
            if self.bus.cycles() >= self.max_cycles { self.drain_console(); return Some(Stop::Halted); }
            let chunk = step.min(self.max_cycles - self.bus.cycles()).max(1);
            if self.after_round(chunk) { self.drain_console(); return Some(Stop::Halted); }
            self.usj_service();
            if !self.rt.enabled { std::thread::sleep(std::time::Duration::from_millis(1)); }
        }
        None
    }
}
