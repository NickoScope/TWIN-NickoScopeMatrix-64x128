//! USB Serial/JTAG controller: the console on every module with a USB port.
use crate::device::{Device, WriteEffect};
use crate::regram::RegRam;
use emu_core::ClockDomain;

// ------------------------------------------------------------------ USB Serial/JTAG
/// the USB full-speed bulk packet size the Serial/JTAG device presents to the host
pub const USB_PACKET: usize = 64;

pub struct UsbSerialJtag {
    pub connected: bool,          // emulate a host: SOF every 1 ms
    sof_period: u64,
    pub dbg: bool,
    pub sof_count: u64,
    sof_acc: u64,
    pub tx_fifo: Vec<u8>,         // bytes written since last WR_DONE
    pub tx_out: Vec<u8>,          // flushed bytes for the host
    pub rx: std::collections::VecDeque<u8>,
    pub int_raw: u32, pub int_ena: u32, pub conf0: u32,
    /// host bytes not yet in the FIFO, one USB packet each
    pub rx_packets: std::collections::VecDeque<Vec<u8>>,
    ram: RegRam,
}
impl UsbSerialJtag {
    pub fn new(cpu_hz: u64) -> Self { UsbSerialJtag { sof_period: cpu_hz / 4000, connected: true, dbg: false, sof_count: 0, sof_acc: 0, tx_fifo: Vec::new(), tx_out: Vec::new(), rx_packets: Default::default(), rx: Default::default(), int_raw: 0, int_ena: 0, conf0: 0, ram: RegRam::new() } }
    /// advance by CPU cycles; raise SOF interrupt every 1 ms of emulated time
    pub fn tick(&mut self, cycles: u64) { if !self.connected { return; } self.sof_acc += cycles; if self.sof_acc >= self.sof_period { self.sof_acc -= self.sof_period; self.int_raw |= 1 << 1; if self.dbg { self.sof_count += 1; } } /* 4x per tick: HWCDC's tick hook clears it each tick */ }
    pub fn read(&mut self, off: u32) -> u32 {
        match off {
            0x0 => { let b = self.rx.pop_front().map(|b| b as u32).unwrap_or(0); self.present_packet(); b }
            0x4 => (1 << 1) | if self.rx.is_empty() { 0 } else { 1 << 2 },
            0x8 => { if self.dbg { eprintln!("[usb] int_raw read -> {:#x}", self.raw()); } self.raw() }
            0xc => { if self.dbg { eprintln!("[usb] int_st read -> {:#x} (ena {:#x})", self.raw() & self.int_ena, self.int_ena); } self.raw() & self.int_ena }
            0x10 => self.int_ena,
            0x18 => self.conf0,
            _ => self.ram.read(off),
        }
    }
    pub fn write(&mut self, off: u32, v: u32) {
        match off {
            0x0 => { self.tx_fifo.push(v as u8); if self.tx_fifo.len() >= 64 { self.flush(); } }
            0x4 => if v & 1 != 0 { self.flush(); },
            0x10 => { if self.dbg && v != self.int_ena { eprintln!("[usb] int_ena {:#x} -> {:#x} (raw {:#x}, fifo {} bytes)", self.int_ena, v, self.raw(), self.tx_fifo.len()); } self.int_ena = v }
            0x14 => { if self.dbg && v & !2 != 0 { eprintln!("[usb] int_clr {:#x} (raw before {:#x})", v, self.raw()); } self.int_raw &= !v; }
            0x18 => self.conf0 = v,
            _ => self.ram.write(off, v),
        }
    }
    fn flush(&mut self) { if self.dbg { eprintln!("[usb] flush {} bytes: {:?}", self.tx_fifo.len(), String::from_utf8_lossy(&self.tx_fifo)); } self.tx_out.extend_from_slice(&self.tx_fifo); self.tx_fifo.clear(); self.int_raw |= 1 << 3; }
    fn raw(&self) -> u32 { self.int_raw }
    /// Bytes from the host. The interface is USB: a transfer arrives as packets of at most 64
    /// bytes, and `SERIAL_OUT_RECV_PKT` is raised once per packet. Drivers rely on that —
    /// Arduino's HWCDC drains up to 64 bytes per interrupt and never polls the FIFO afterwards,
    /// so a single interrupt for a longer line left its tail stranded and the line's newline
    /// never arrived (found with a 66-byte JSON action; every committed script action is
    /// shorter). The bytes queue as packets and the next packet's interrupt follows the
    /// previous one being read out.
    pub fn host_input(&mut self, data: &[u8]) {
        for chunk in data.chunks(USB_PACKET) { self.rx_packets.push_back(chunk.to_vec()); }
        self.present_packet();
    }
    /// Move the next packet into the FIFO and raise RECV_PKT for it, once the FIFO is empty.
    fn present_packet(&mut self) {
        if self.rx.is_empty() { if let Some(p) = self.rx_packets.pop_front() { self.rx.extend(p); self.int_raw |= 1 << 2; } }
    }
    pub fn irq(&self) -> bool { self.raw() & self.int_ena != 0 }
}

// ------------------------------------------------------------------ CDC-ACM line state
/// What the USB host's DTR and RTS lines do to the chip. The host sets both with the CDC-ACM
/// request SET_CONTROL_LINE_STATE (ESP32-S3 TRM v1.8 Table 33.3-1, p.1247), and the controller acts
/// on the pair (Table 33.3-2, p.1248):
///
/// | RTS | DTR | action                    |
/// |-----|-----|---------------------------|
/// |  0  |  0  | clear download mode flag  |
/// |  0  |  1  | set download mode flag    |
/// |  1  |  0  | reset the chip            |
/// |  1  |  1  | no action                 |
///
/// "If the download mode flag is set when the ESP32-S3 is reset, the ESP32-S3 will reboot into
/// download mode", otherwise it boots from flash (same page). The S3 has no guest register that
/// shows these lines or the flag, so this state lives on the host side of the model and survives
/// the peripheral re-creation of a chip reset.
///
/// Two readings of the TRM are this model's, not the TRM's words [inferred, not verified on
/// silicon]: the chip is held in reset for as long as the pair stays (1,0) — Table 33.4-4 (p.1254)
/// names the step into (1,0) "Reset SoC" and the step out "Exit reset" — and the flag is taken
/// at the moment the pair enters (1,0), because Table 33.4-3 (p.1254) leaves (1,0) through (0,0),
/// "Clear download flag", and still reaches download mode. The logic is level-based: every
/// message is one new pair, a repeated pair does nothing, and a host that sets DTR and RTS in two
/// requests (esptool, esptool-js; Windows only propagates DTR on an RTS change, §33.4.2 p.1253)
/// is handled the same as one that sets both at once.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct UsjLines {
    pub dtr: bool,
    pub rts: bool,
    /// the download mode flag, as the last (0,0) / (0,1) left it
    pub flag: bool,
    /// the pair is (1,0): the chip is held in reset
    pub held: bool,
    /// the flag as it was when the pair entered (1,0): the reset boots into download mode
    pub latched_download: bool,
}

/// What one line change does to the chip.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LineEffect {
    None,
    /// The pair entered (1,0): reset the chip and hold it. `download` is the latched flag.
    AssertReset { download: bool },
    /// The pair left (1,0): let the chip run.
    ReleaseReset,
}

impl UsjLines {
    /// The host set the lines to (`dtr`, `rts`).
    pub fn set(&mut self, dtr: bool, rts: bool) -> LineEffect {
        let was_reset = self.held;
        self.dtr = dtr;
        self.rts = rts;
        match (rts, dtr) {
            (false, false) => self.flag = false,
            (false, true) => self.flag = true,
            _ => {}
        }
        let now_reset = rts && !dtr;
        if now_reset && !was_reset {
            self.held = true;
            self.latched_download = self.flag;
            return LineEffect::AssertReset { download: self.flag };
        }
        if was_reset && !now_reset {
            self.held = false;
            return LineEffect::ReleaseReset;
        }
        LineEffect::None
    }
    /// `bit0` DTR, `bit1` RTS: the form the host channels carry.
    pub fn bits(&self) -> u8 { self.dtr as u8 | (self.rts as u8) << 1 }
}

impl Device for UsbSerialJtag {
    fn read(&mut self, off: u32) -> u32 { UsbSerialJtag::read(self, off) }
    fn write(&mut self, off: u32, v: u32) -> WriteEffect { UsbSerialJtag::write(self, off, v); WriteEffect::NONE }
    fn irq_sources(&self) -> u64 { self.irq() as u64 }
    fn clock(&self) -> Option<ClockDomain> { Some(ClockDomain::Cpu) }
    fn tick(&mut self, cycles: u64) { UsbSerialJtag::tick(self, cycles) }
    fn debug(&mut self, on: bool) { self.dbg = on; }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Apply (dtr, rts) pairs in order; collect every effect that is not `None`.
    fn run(lines: &mut UsjLines, seq: &[(bool, bool)]) -> Vec<LineEffect> {
        seq.iter().map(|&(dtr, rts)| lines.set(dtr, rts)).filter(|e| *e != LineEffect::None).collect()
    }

    /// A host that sets one line per request: `D`/`R` are (line, level) steps from `start`.
    fn steps(start: (bool, bool), ops: &[(char, bool)]) -> Vec<(bool, bool)> {
        let (mut dtr, mut rts) = start;
        ops.iter().map(|&(line, v)| { if line == 'D' { dtr = v } else { rts = v } (dtr, rts) }).collect()
    }

    /// esptool's and esptool-js's `setRTS(x)`: RTS, then DTR again at its current level
    /// (esptool 4.9.0 reset.py:70-75, esptool-js 0.6.0 webserial.ts:440-452).
    fn set_rts(dtr: bool, v: bool) -> [(char, bool); 2] { [('R', v), ('D', dtr)] }

    #[test]
    fn table_33_4_3_resets_into_download_mode_once() {
        // TRM v1.8 Table 33.4-3 (p.1254) from RTS=0, with DTR and the flag either way. (From RTS=1
        // its first step, Clear DTR, is itself (1,0): one more reset, a flash boot, comes first.)
        for dtr0 in [false, true] {
            let mut l = UsjLines::default();
            l.set(dtr0, false);
            let seq = steps((dtr0, false), &[('D', false), ('R', false), ('D', true), ('R', false), ('R', true), ('D', false), ('R', true), ('R', false)]);
            assert_eq!(run(&mut l, &seq), [LineEffect::AssertReset { download: true }, LineEffect::ReleaseReset]);
            assert!(!l.flag && !l.held && l.latched_download, "left through (0,0): flag cleared, the reset kept it");
        }
    }

    #[test]
    fn table_33_4_4_resets_into_flash_boot_once() {
        let mut l = UsjLines { dtr: true, flag: true, ..Default::default() };
        let seq = steps((true, false), &[('D', false), ('R', false), ('R', true), ('R', false)]);
        assert_eq!(run(&mut l, &seq), [LineEffect::AssertReset { download: false }, LineEffect::ReleaseReset]);
    }

    #[test]
    fn the_pair_one_one_does_nothing_and_repeats_are_idempotent() {
        let mut l = UsjLines::default();
        assert_eq!(l.set(true, true), LineEffect::None);
        assert_eq!(l.set(true, true), LineEffect::None);
        assert_eq!(l.set(false, true), LineEffect::AssertReset { download: false });
        assert_eq!(l.set(false, true), LineEffect::None, "still (1,0): held, no second reset");
        assert!(l.held);
        assert_eq!(l.set(true, true), LineEffect::ReleaseReset, "(1,1) also leaves reset");
    }

    #[test]
    fn esptool_usb_jtag_serial_reset_enters_download_mode() {
        // esptool 4.9.0 reset.py:130-142 after pyserial 3.5 opened the port with both lines up (serialutil.py:216-217
        // _rts_state = _dtr_state = True; rfc2217.py:489-492 sends DTR, then RTS).
        let mut ops = vec![('D', true), ('R', true)];
        ops.extend(set_rts(true, false)); ops.push(('D', false));          // _setRTS(False) _setDTR(False)  idle
        ops.push(('D', true)); ops.extend(set_rts(true, false));           // _setDTR(True) _setRTS(False)   set IO0
        ops.extend(set_rts(true, true)); ops.push(('D', false));           // _setRTS(True) _setDTR(False)   reset
        ops.extend(set_rts(false, true));                                  // _setRTS(True)
        ops.push(('D', false)); ops.extend(set_rts(false, false));         // _setDTR(False) _setRTS(False)  out of reset
        let mut l = UsjLines::default();
        assert_eq!(run(&mut l, &steps((false, false), &ops)), [LineEffect::AssertReset { download: true }, LineEffect::ReleaseReset]);
        // esptool-js 0.6.0 reset.ts:113-129, its setRTS repeating DTR from `_DTR_state` (false at first).
        let mut ops = Vec::new();
        ops.extend(set_rts(false, false)); ops.push(('D', false));
        ops.push(('D', true)); ops.extend(set_rts(true, false));
        ops.extend(set_rts(true, true)); ops.push(('D', false)); ops.extend(set_rts(false, true));
        ops.extend(set_rts(false, false)); ops.push(('D', false));
        let mut l = UsjLines::default();
        assert_eq!(run(&mut l, &steps((false, false), &ops)), [LineEffect::AssertReset { download: true }, LineEffect::ReleaseReset]);
    }

    #[test]
    fn hard_and_classic_resets_boot_from_flash() {
        // esptool 4.9.0 HardReset (reset.py:154-164, not USB-OTG): setRTS(True), 100 ms, setRTS(False).
        let mut l = UsjLines::default();
        let ops: Vec<_> = set_rts(false, true).into_iter().chain(set_rts(false, false)).collect();
        assert_eq!(run(&mut l, &steps((false, false), &ops)), [LineEffect::AssertReset { download: false }, LineEffect::ReleaseReset]);
        // ClassicReset (reset.py:97-104, esptool-js reset.ts:81-89): a flash boot on the USJ.
        let mut l = UsjLines::default();
        let mut ops = vec![('D', false)];
        ops.extend(set_rts(false, true)); ops.push(('D', true)); ops.extend(set_rts(true, false)); ops.push(('D', false));
        assert_eq!(run(&mut l, &steps((false, false), &ops)), [LineEffect::AssertReset { download: false }, LineEffect::ReleaseReset]);
        assert!(!l.flag);
    }

    #[test]
    fn both_lines_in_one_request_work_the_same() {
        // A host that sets both lines at once (Web Serial setSignals with both members): 0,1 -> 1,0 -> 0,0.
        let mut l = UsjLines::default();
        assert_eq!(run(&mut l, &[(true, false), (false, true), (false, false)]), [LineEffect::AssertReset { download: true }, LineEffect::ReleaseReset]);
        assert_eq!(l.bits(), 0);
        l.set(true, true);
        assert_eq!(l.bits(), 3);
    }
}
