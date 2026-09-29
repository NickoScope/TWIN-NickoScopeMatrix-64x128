//! The physical inputs of the NickoScope LED panel (Waveshare ESP32-S3 RGB matrix board running
//! AnimatedPixelClock), as pin-level models. Nothing here decodes anything: the firmware's own
//! GPIO interrupt, TIMG timeout, NEC decoder and 1 kHz knob sampler do that, exactly as on the
//! hardware. This file only makes the levels the three devices put on the pins:
//!
//! - **GPIO0** is a wired-AND of three open-drain sources (low if any pulls low), idle HIGH through
//!   the board's R8 10 kOhm pull-up (FW `src/ir/ir.h:32-39`):
//!   - a TSOP IR receiver: the demodulated envelope, LOW during a carrier burst (Vishay 82460
//!     rev 2.2, p.3 Fig. 1 "Output Active Low"; the owner's TSOP2138 is in its parts table, p.2);
//!   - the EC11 knob's push switch, LOW while pressed (FW `src/control/control.cpp:29-36,280`);
//!   - the BOOT button, LOW while pressed (same line, FW `control.cpp:36,280`).
//! - **IO45 / IO46** are the knob's A / B contacts: the knob's common goes to 3V3, the board pulls
//!   both lines down with 10 kOhm (R59, R60), so a closed contact reads HIGH and idle is LOW
//!   (FW `control.cpp:33-43`, `pinA()`/`pinB()` invert at `control.cpp:174-176`).
//!
//! Sources. FW = the AnimatedPixelClock tree the twin runs (`src/` identical in
//! AnimatedPixelClock-twin 545a0d3 and AnimatedPixelClock-netbroker); IRR = IRremoteESP8266 2.9.0,
//! the copy the firmware is built with (`.pio/libdeps/matrix-waveshare-rgb/IRremoteESP8266/src`).
//! Every number below names its line; anything that is our own choice says so.
//!
//! Time is `VirtualCycle`, CPU cycles at `periph::CPU_HZ` (240 MHz, 240 cycles per microsecond).
//! Every input is observed at a bus cycle and scheduled strictly after the board's current cycle;
//! commands to a device that is still busy queue behind it, as one hand on one remote would.
use crate::periph::CPU_HZ;
use esp_soc::board::{BoardEdge, VirtualCycle};
use std::cmp::Reverse;
use std::collections::{BinaryHeap, VecDeque};

pub const PIN_GPIO0: u8 = 0;
pub const PIN_ENC_A: u8 = 45;
pub const PIN_ENC_B: u8 = 46;

const CYCLES_PER_US: u64 = CPU_HZ / 1_000_000;
const CYCLES_PER_MS: u64 = CPU_HZ / 1_000;

// ---------------------------------------------------------------- NEC, as the firmware's library knows it
/// IRR `ir_NEC.h:27` kNecTick.
pub const NEC_TICK_US: u32 = 560;
/// IRR `ir_NEC.h:28-29` kNecHdrMark = 16 ticks.
pub const NEC_HDR_MARK_US: u32 = 16 * NEC_TICK_US;
/// IRR `ir_NEC.h:30-31` kNecHdrSpace = 8 ticks.
pub const NEC_HDR_SPACE_US: u32 = 8 * NEC_TICK_US;
/// IRR `ir_NEC.h:32-33` kNecBitMark = 1 tick (also the footer mark, `ir_NEC.cpp:29-30`).
pub const NEC_BIT_MARK_US: u32 = NEC_TICK_US;
/// IRR `ir_NEC.h:34-35` kNecOneSpace = 3 ticks.
pub const NEC_ONE_SPACE_US: u32 = 3 * NEC_TICK_US;
/// IRR `ir_NEC.h:36-37` kNecZeroSpace = 1 tick.
pub const NEC_ZERO_SPACE_US: u32 = NEC_TICK_US;
/// IRR `ir_NEC.h:38-39` kNecRptSpace = 4 ticks.
pub const NEC_RPT_SPACE_US: u32 = 4 * NEC_TICK_US;
/// IRR `ir_NEC.h:41-42` kNecMinCommandLength = 193 ticks = 108,080 us: every NEC message (data or
/// repeat) is padded to it (`IRsend.cpp:376-381`), so a held key repeats on this period.
pub const NEC_MESSAGE_US: u32 = 193 * NEC_TICK_US;
/// IRR `IRremoteESP8266.h:1362` kNECBits.
pub const NEC_BITS: u32 = 32;
/// IRR `ir_NEC.h:43-46` kNecMinGap: the least space after the footer mark.
pub const NEC_MIN_GAP_US: u32 = NEC_MESSAGE_US - (NEC_HDR_MARK_US + NEC_HDR_SPACE_US + NEC_BITS * (NEC_BIT_MARK_US + NEC_ONE_SPACE_US) + NEC_BIT_MARK_US);

/// A data message as the remote sends it, `(mark, space)` in microseconds, the way
/// `IRsend::sendNEC` builds it (IRR `ir_NEC.cpp:28-32`): header, 32 bits MSB first (`sendGeneric`
/// with MSBfirst = true, `IRsend.cpp:253-268`: mark, then a one- or zero-space), the footer mark,
/// then the space that pads the message to `NEC_MESSAGE_US` (`IRsend.cpp:376-381`).
/// The 32-bit value is IRremoteESP8266's `results.value` form (the owner's codes below are in it).
pub fn nec_message(code: u32) -> Vec<(u32, u32)> {
    let mut m = vec![(NEC_HDR_MARK_US, NEC_HDR_SPACE_US)];
    for bit in (0..NEC_BITS).rev() {
        m.push((NEC_BIT_MARK_US, if code >> bit & 1 != 0 { NEC_ONE_SPACE_US } else { NEC_ZERO_SPACE_US }));
    }
    m.push((NEC_BIT_MARK_US, 0));
    pad(&mut m);
    m
}

/// A repeat message: header mark, a 4-tick space, the footer mark, padded (IRR `ir_NEC.cpp:34-39`).
pub fn nec_repeat() -> Vec<(u32, u32)> {
    let mut m = vec![(NEC_HDR_MARK_US, NEC_RPT_SPACE_US), (NEC_BIT_MARK_US, 0)];
    pad(&mut m);
    m
}

/// `space(max(gap, mesgtime - elapsed))` after the footer mark, IRR `IRsend.cpp:376-381`.
fn pad(m: &mut [(u32, u32)]) {
    let elapsed: u32 = m.iter().map(|(mark, space)| mark + space).sum();
    let last = m.last_mut().expect("a message has a footer");
    last.1 = NEC_MIN_GAP_US.max(NEC_MESSAGE_US.saturating_sub(elapsed));
}

// ---------------------------------------------------------------- the owner's remote
/// One button of the owner's remote: the number people see (1..10, FW `src/ir/ir_map.h:36-37,108`),
/// the function the live panel has it set to (the machine names of FW `ir_map.h:59-80`), and its
/// NEC code in IRremoteESP8266's value form.
#[derive(Clone, Copy, Debug)]
pub struct RemoteButton { pub number: u8, pub name: &'static str, pub code: u32 }

/// The ten codes the owner's panel had learned, read live with a read-only `GET /api/ir` by the
/// twin research on 2026-09-29 (`research/vp_study_io.json`, finding "The owner's remote is
/// standard NEC with address 0x00"). The research verifier did not re-read the live panel
/// (`vp_checks.json` #64, "unclear"), but re-decoded all ten as NEC address 0 with a valid
/// inverted command (#126). The names are the functions the panel had set, which live in its NVS
/// and can be changed in the portal; the codes are what the remote sends.
pub const OWNER_REMOTE: [RemoteButton; 10] = [
    RemoteButton { number: 1, name: "ccw", code: 0x00FF_708F },
    RemoteButton { number: 2, name: "cw", code: 0x00FF_58A7 },
    RemoteButton { number: 3, name: "ok", code: 0x00FF_E01F },
    RemoteButton { number: 4, name: "long", code: 0x00FF_41BE },
    RemoteButton { number: 5, name: "power", code: 0x00FF_28D7 },
    RemoteButton { number: 6, name: "bright_up", code: 0x00FF_C03F },
    RemoteButton { number: 7, name: "carousel", code: 0x00FF_19E6 },
    RemoteButton { number: 8, name: "home", code: 0x00FF_12ED },
    RemoteButton { number: 9, name: "bright_down", code: 0x00FF_40BF },
    RemoteButton { number: 10, name: "media_toggle", code: 0x00FF_C936 },
];

/// A remote key by function name (any case), by number `1`..`10`, or a raw 32-bit code `0x...`.
pub fn remote_code(key: &str) -> Result<u32, String> {
    if let Some(hex) = key.strip_prefix("0x").or_else(|| key.strip_prefix("0X")) {
        return u32::from_str_radix(hex, 16).map_err(|_| format!("bad NEC code {key} (32 bits, hex)"));
    }
    if let Ok(n) = key.parse::<u8>() {
        return OWNER_REMOTE.iter().find(|b| b.number == n).map(|b| b.code).ok_or_else(|| format!("no remote button {n} (1..10)"));
    }
    OWNER_REMOTE.iter().find(|b| b.name.eq_ignore_ascii_case(key)).map(|b| b.code)
        .ok_or_else(|| format!("no remote button {key}: {} | 1..10 | 0xCODE", OWNER_REMOTE.iter().map(|b| b.name).collect::<Vec<_>>().join(" ")))
}

// ---------------------------------------------------------------- the TSOP
/// The receiver's own timing, applied to every burst: the output falls `delay_us` after the burst
/// starts and stays low for the burst's length plus `mark_stretch_us`. Both default to 0, an ideal
/// receiver. Vishay 82460 rev 2.2 p.3 Fig. 1 bounds them at the 38 kHz carrier:
/// 4/f0 < td < 10/f0 and tpi - 3.0/f0 < tpo < tpi + 3.5/f0 (`TSOP_DELAY_US`, `TSOP_STRETCH_US`).
/// What the owner's receiver really does next to the HUB75 panel is unmeasured (ADR-TWIN-01 §6 #16).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Tsop { pub delay_us: u32, pub mark_stretch_us: i32 }
/// Vishay 82460 p.3 Fig. 1 at f0 = 38 kHz: 4/f0 = 105.3 us, 10/f0 = 263.2 us.
pub const TSOP_DELAY_US: (u32, u32) = (105, 263);
/// Vishay 82460 p.3 Fig. 1 at f0 = 38 kHz: -3.0/f0 = -78.9 us, +3.5/f0 = +92.1 us.
pub const TSOP_STRETCH_US: (i32, i32) = (-79, 92);

impl Tsop {
    /// Anything that keeps every mark and space of a NEC message longer than zero and every frame
    /// inside its message slot; the datasheet's bounds are above, a test may go past them.
    pub fn new(delay_us: u32, mark_stretch_us: i32) -> Result<Self, String> {
        let shortest = NEC_BIT_MARK_US.min(NEC_ZERO_SPACE_US) as i32;
        if mark_stretch_us <= -shortest || mark_stretch_us >= shortest {
            return Err(format!("mark stretch {mark_stretch_us} us: must stay inside +-{shortest} us, the shortest NEC mark and space"));
        }
        if delay_us > 1000 { return Err(format!("TSOP delay {delay_us} us: the model takes up to 1000 us (datasheet 105..263)")); }
        Ok(Tsop { delay_us, mark_stretch_us })
    }
}

// ---------------------------------------------------------------- the knob
/// Where the EC11 rests. The owner's knob type is unknown (ADR-TWIN-01 §6 #15; the bench note at
/// FW `control.cpp:81-83` hints at half-cycle, not verified), and the firmware handles both: it
/// accepts a detent at logical 11 always, and at 00 too once it has seen 00 rest for 250 ms
/// (FW `control.cpp:77,221,239`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Detent {
    /// One full quadrature cycle per click; rests with both contacts open.
    FullCycle,
    /// Half a cycle per click; rests with both contacts open, or both closed.
    HalfCycle,
}

/// Default time each quadrature state is held: FW `control.cpp:90-102` needs
/// CTRL_ENC_STABLE_MS = 2 agreeing 1 kHz samples (`kSampleUs`, `control.cpp:74`) before a state
/// counts; 3 ms gives a sample of margin (ADR-TWIN-01 §4.2 B5, and FW `tools/control` counts
/// "20 fast clicks at 3 ms a state").
pub const KNOB_STATE_MS: u32 = 3;
/// Default time from the last edge of one click to the first edge of the next. After a step the
/// sampler ignores the knob for CTRL_ENC_LOCKOUT_MS = 10 ms (FW `control.h:28-30`,
/// `control.cpp:223-225`); the 2 ms above it are our margin for sampler jitter (not from a source).
pub const KNOB_GAP_MS: u32 = 12;

// ---------------------------------------------------------------- the model
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Line { Tsop, Sw, Boot, A, B }

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Ev {
    /// A NEC message slot starts: the next message of the key being held, or the next key.
    IrSlot,
    /// A source pulls its line (`true`: TSOP/SW/BOOT pull GPIO0 low; A/B contact closed).
    Level(Line, bool),
}
impl Ev {
    /// Order at one cycle: a slot first (its first edge may fall on the same cycle), then levels
    /// in the order they were scheduled.
    fn rank(self) -> u8 { match self { Ev::IrSlot => 0, Ev::Level(..) => 1 } }
}

/// A key on the remote: sent from `slot0`, repeated while held.
#[derive(Clone, Copy, Debug)]
struct Key { code: u32, slot0: VirtualCycle, release_at: Option<VirtualCycle> }
/// A key waiting for the one before it; `hold` is None while its button is still down.
#[derive(Clone, Copy, Debug)]
struct Pending { code: u32, down_at: VirtualCycle, hold: Option<u64> }

/// Running totals for the end-of-run report.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct InputCounts { pub ir_messages: u64, pub ir_repeats: u64, pub detents: u64, pub sw_presses: u64, pub boot_presses: u64, pub gpio0_edges: u64 }

pub struct PanelInputs {
    cycle: VirtualCycle,
    seq: u64,
    queue: BinaryHeap<Reverse<(VirtualCycle, u8, u64, Ev)>>,
    /// source states: TSOP / SW / BOOT pulling GPIO0 low, A / B contact closed
    tsop_low: bool, sw_low: bool, boot_low: bool, a_closed: bool, b_closed: bool,
    /// pin levels as last reported: GPIO0, IO45, IO46
    levels: [bool; 3],
    edges: Vec<BoardEdge>,
    detent: Detent,
    state_cycles: u64,
    gap_cycles: u64,
    tsop: Tsop,
    /// half-cycle knob only: resting with both contacts closed (logical 00)
    knob_closed_rest: bool,
    knob_free_at: VirtualCycle,
    sw_free_at: VirtualCycle,
    boot_free_at: VirtualCycle,
    key: Option<Key>,
    pending: VecDeque<Pending>,
    pub counts: InputCounts,
}

impl Default for PanelInputs { fn default() -> Self { Self::new() } }

impl PanelInputs {
    pub fn new() -> Self {
        PanelInputs {
            cycle: 0, seq: 0, queue: BinaryHeap::new(),
            tsop_low: false, sw_low: false, boot_low: false, a_closed: false, b_closed: false,
            levels: [true, false, false], edges: Vec::new(),
            detent: Detent::FullCycle, state_cycles: KNOB_STATE_MS as u64 * CYCLES_PER_MS, gap_cycles: KNOB_GAP_MS as u64 * CYCLES_PER_MS,
            tsop: Tsop::default(), knob_closed_rest: false, knob_free_at: 0, sw_free_at: 0, boot_free_at: 0,
            key: None, pending: VecDeque::new(), counts: InputCounts::default(),
        }
    }

    // ------------------------------------------------------------ configuration
    /// The knob's detent type; takes effect for turns scheduled after it. A half-cycle knob left
    /// resting with both contacts closed turns its next full cycles from there.
    pub fn set_detent(&mut self, d: Detent) { self.detent = d; }
    pub fn detent(&self) -> Detent { self.detent }
    /// Milliseconds per quadrature state and between clicks, for turns scheduled after it.
    /// Values the firmware filters out are allowed on purpose: tests use them.
    pub fn set_knob_timing(&mut self, state_ms: u32, gap_ms: u32) -> Result<(), String> {
        if state_ms == 0 || state_ms > 1000 || gap_ms > 10_000 { return Err(format!("knob timing {state_ms}/{gap_ms} ms out of range")); }
        self.state_cycles = state_ms as u64 * CYCLES_PER_MS;
        self.gap_cycles = gap_ms as u64 * CYCLES_PER_MS;
        Ok(())
    }
    pub fn set_tsop(&mut self, tsop: Tsop) { self.tsop = tsop; }
    pub fn tsop(&self) -> Tsop { self.tsop }

    // ------------------------------------------------------------ inputs
    /// The first cycle a new input may change anything: strictly after the board's present.
    fn first(&self, at: VirtualCycle) -> VirtualCycle { at.max(self.cycle).saturating_add(1) }

    fn push(&mut self, cycle: VirtualCycle, ev: Ev) {
        self.seq += 1;
        self.queue.push(Reverse((cycle, ev.rank(), self.seq, ev)));
    }

    /// Press a remote key observed at bus cycle `at`, held `hold_ms` (0: one data message). While
    /// held, a repeat message starts at every `NEC_MESSAGE_US` slot boundary the key is still down
    /// at (Vishay 80071, NEC: repeats "in each 108 ms time slot as long as the key is pressed";
    /// that a repeat is sent only if the key is down when its slot starts is our reading, not
    /// checked against the remote's encoder IC).
    pub fn ir_press(&mut self, at: VirtualCycle, code: u32, hold_ms: u32) {
        let down_at = self.first(at);
        self.key_down(Pending { code, down_at, hold: Some(hold_ms as u64 * CYCLES_PER_MS) });
    }
    /// A remote key goes down and stays down until `ir_up` (the UI's press-and-hold).
    pub fn ir_down(&mut self, at: VirtualCycle, code: u32) {
        let down_at = self.first(at);
        self.key_down(Pending { code, down_at, hold: None });
    }
    /// The earliest key held open by `ir_down` is let go. Nothing held: nothing happens.
    pub fn ir_up(&mut self, at: VirtualCycle) {
        let up = self.first(at);
        if let Some(k) = self.key.as_mut().filter(|k| k.release_at.is_none()) {
            k.release_at = Some(up.max(k.slot0));   // the next slot boundary looks at it
        } else if let Some(p) = self.pending.iter_mut().find(|p| p.hold.is_none()) {
            p.hold = Some(up.saturating_sub(p.down_at));   // it will be sent for as long as it was held
        }
    }
    fn key_down(&mut self, p: Pending) {
        if self.key.is_none() && self.pending.is_empty() {
            self.start_key(p, p.down_at);
        } else {
            self.pending.push_back(p);   // one remote, one key: it goes after the one being sent
        }
    }
    fn start_key(&mut self, p: Pending, slot0: VirtualCycle) {
        self.key = Some(Key { code: p.code, slot0, release_at: p.hold.map(|h| slot0.saturating_add(h)) });
        self.push(slot0, Ev::IrSlot);
    }

    /// Turn the knob `detents` clicks, observed at bus cycle `at`. `cw` is the firmware's
    /// clockwise: with `reverse` off it is the IO45 (A) contact changing first (FW
    /// `control.cpp:79,176,227-245`). Which physical rotation that is on the owner's knob is
    /// not known (ADR-TWIN-01 §6 #15).
    pub fn knob_turn(&mut self, at: VirtualCycle, cw: bool, detents: u32) {
        let mut t = self.first(at).max(self.knob_free_at);
        let (lead, lag) = if cw { (Line::A, Line::B) } else { (Line::B, Line::A) };
        for _ in 0..detents {
            // From the rest (both contacts open, or both closed on a half-cycle knob): the leading
            // contact changes, then the lagging one; a full cycle then changes both back.
            // Clockwise from open: A closes, B closes, A opens, B opens - logical 11, 10, 00, 01,
            // 11, the table's two +1 transitions (FW `control.cpp:79`, index = old | new << 2).
            let c = self.knob_closed_rest;
            let mut steps = vec![(lead, !c), (lag, !c)];
            match self.detent {
                Detent::FullCycle => steps.extend([(lead, c), (lag, c)]),
                Detent::HalfCycle => self.knob_closed_rest = !c,
            }
            for (i, (line, closed)) in steps.into_iter().enumerate() {
                if i > 0 { t += self.state_cycles; }
                self.push(t, Ev::Level(line, closed));
            }
            t += self.gap_cycles;
            self.counts.detents += 1;
        }
        self.knob_free_at = t;
    }

    /// Press the knob's switch for `ms`, observed at `at`. A press still under way is finished first.
    pub fn sw_press(&mut self, at: VirtualCycle, ms: u32) {
        let t = self.first(at).max(self.sw_free_at);
        let end = t + ms as u64 * CYCLES_PER_MS;
        self.push(t, Ev::Level(Line::Sw, true));
        self.push(end, Ev::Level(Line::Sw, false));
        self.sw_free_at = end;
        self.counts.sw_presses += 1;
    }
    /// The knob's switch goes down or up and stays (the UI's press-and-hold).
    pub fn sw(&mut self, at: VirtualCycle, down: bool) {
        let t = self.first(at).max(self.sw_free_at);
        self.push(t, Ev::Level(Line::Sw, down));
        self.sw_free_at = t;
        if down { self.counts.sw_presses += 1; }
    }
    /// Press BOOT for `ms`, observed at `at`.
    pub fn boot_press(&mut self, at: VirtualCycle, ms: u32) {
        let t = self.first(at).max(self.boot_free_at);
        let end = t + ms as u64 * CYCLES_PER_MS;
        self.push(t, Ev::Level(Line::Boot, true));
        self.push(end, Ev::Level(Line::Boot, false));
        self.boot_free_at = end;
        self.counts.boot_presses += 1;
    }
    /// BOOT goes down or up and stays.
    pub fn boot(&mut self, at: VirtualCycle, down: bool) {
        let t = self.first(at).max(self.boot_free_at);
        self.push(t, Ev::Level(Line::Boot, down));
        self.boot_free_at = t;
        if down { self.counts.boot_presses += 1; }
    }

    // ------------------------------------------------------------ the board plumbing
    /// The levels the devices hold the pins at now (GPIO0 wired-AND, IO45, IO46).
    pub fn input_levels(&self) -> Vec<(u8, bool)> {
        vec![(PIN_GPIO0, self.levels[0]), (PIN_ENC_A, self.levels[1]), (PIN_ENC_B, self.levels[2])]
    }
    /// The level of one of the three pins as of the board's present.
    pub fn level(&self, pin: u8) -> Option<bool> {
        match pin { PIN_GPIO0 => Some(self.levels[0]), PIN_ENC_A => Some(self.levels[1]), PIN_ENC_B => Some(self.levels[2]), _ => None }
    }
    pub fn cycle(&self) -> VirtualCycle { self.cycle }
    /// Is anything still to happen: queued edges, a key being sent or held, keys waiting?
    pub fn busy(&self) -> bool { !self.queue.is_empty() || self.key.is_some() || !self.pending.is_empty() }

    pub fn next_deadline(&self) -> Option<VirtualCycle> { self.queue.peek().map(|Reverse((c, ..))| *c) }

    pub fn advance_to(&mut self, cycle: VirtualCycle) {
        assert!(cycle >= self.cycle, "board time moved backwards from {} to {}", self.cycle, cycle);
        while let Some(t) = self.next_deadline().filter(|t| *t <= cycle) {
            // Everything due at this cycle, then the pins once: sources that change together
            // (a release meeting a press) never make a zero-width glitch.
            while let Some(Reverse((_, _, _, ev))) = self.queue.peek().copied().filter(|Reverse((c, ..))| *c == t) {
                self.queue.pop();
                self.apply(t, ev);
            }
            self.emit(t);
        }
        self.cycle = cycle;
    }

    pub fn take_edges(&mut self) -> Vec<BoardEdge> { std::mem::take(&mut self.edges) }

    fn apply(&mut self, t: VirtualCycle, ev: Ev) {
        match ev {
            Ev::Level(line, on) => match line {
                Line::Tsop => self.tsop_low = on,
                Line::Sw => self.sw_low = on,
                Line::Boot => self.boot_low = on,
                Line::A => self.a_closed = on,
                Line::B => self.b_closed = on,
            },
            Ev::IrSlot => {
                let Some(key) = self.key else { return };
                let held = t == key.slot0 || key.release_at.is_none_or(|r| t < r);
                if held {
                    let message = if t == key.slot0 { nec_message(key.code) } else { nec_repeat() };
                    if t == key.slot0 { self.counts.ir_messages += 1; } else { self.counts.ir_repeats += 1; }
                    self.send(t, &message);
                    self.push(t + NEC_MESSAGE_US as u64 * CYCLES_PER_US, Ev::IrSlot);
                } else {
                    // The key's last message slot has ended: the next key may start now.
                    self.key = None;
                    if let Some(p) = self.pending.pop_front() { self.start_key(p, t.max(p.down_at)); }
                }
            }
        }
    }

    /// The TSOP's output for one message starting at `t`: low for each burst.
    fn send(&mut self, t: VirtualCycle, message: &[(u32, u32)]) {
        let (delay, stretch) = (self.tsop.delay_us as i64, self.tsop.mark_stretch_us as i64);
        let mut at_us: i64 = 0;
        for &(mark, space) in message {
            let fall = at_us + delay;
            let rise = fall + mark as i64 + stretch;
            self.push(t + fall as u64 * CYCLES_PER_US, Ev::Level(Line::Tsop, true));
            self.push(t + rise as u64 * CYCLES_PER_US, Ev::Level(Line::Tsop, false));
            at_us += (mark + space) as i64;
        }
    }

    fn emit(&mut self, t: VirtualCycle) {
        let now = [!(self.tsop_low || self.sw_low || self.boot_low), self.a_closed, self.b_closed];
        for (i, pin) in [PIN_GPIO0, PIN_ENC_A, PIN_ENC_B].into_iter().enumerate() {
            if now[i] != self.levels[i] {
                self.edges.push(BoardEdge { cycle: t, pin, level: now[i] });
                if i == 0 { self.counts.gpio0_edges += 1; }
            }
        }
        self.levels = now;
    }

    // ------------------------------------------------------------ script and UI verbs
    /// Is `cmd args` one of this board's verbs? None: not ours (the machine's generic verbs
    /// apply). Some(Err): ours, but the arguments are wrong.
    pub fn check_input(&self, cmd: &str, args: &str) -> Option<Result<(), String>> {
        parse(cmd, args).map(|r| r.map(|_| ()))
    }

    /// Apply a verb (see `parse`) observed at bus cycle `at`.
    pub fn input_at(&mut self, at: VirtualCycle, cmd: &str, args: &str) -> Result<(), String> {
        let c = parse(cmd, args).ok_or_else(|| format!("hub75-panel: no input `{cmd}`"))??;
        match c {
            Command::IrPress(code, hold) => self.ir_press(at, code, hold),
            Command::IrDown(code) => self.ir_down(at, code),
            Command::IrUp => self.ir_up(at),
            Command::IrTsop(t) => self.set_tsop(t),
            Command::Knob(cw, n) => self.knob_turn(at, cw, n),
            Command::KnobDetent(d) => self.set_detent(d),
            Command::KnobTiming(s, g) => self.set_knob_timing(s, g)?,
            Command::Sw(Press::For(ms)) => self.sw_press(at, ms),
            Command::Sw(Press::Down) => self.sw(at, true),
            Command::Sw(Press::Up) => self.sw(at, false),
            Command::Boot(Press::For(ms)) => self.boot_press(at, ms),
            Command::Boot(Press::Down) => self.boot(at, true),
            Command::Boot(Press::Up) => self.boot(at, false),
        }
        Ok(())
    }

    pub fn report(&self) -> String {
        let c = &self.counts;
        format!("[emu] panel inputs: IR {} messages + {} repeats (TSOP delay {} us, stretch {:+} us), knob {} clicks ({:?}), SW {} presses, BOOT {} presses, GPIO0 {} edges; levels GPIO0={} IO45={} IO46={}",
                c.ir_messages, c.ir_repeats, self.tsop.delay_us, self.tsop.mark_stretch_us, c.detents, self.detent, c.sw_presses, c.boot_presses, c.gpio0_edges,
                self.levels[0] as u8, self.levels[1] as u8, self.levels[2] as u8)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Press { For(u32), Down, Up }

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Command {
    IrPress(u32, u32), IrDown(u32), IrUp, IrTsop(Tsop),
    Knob(bool, u32), KnobDetent(Detent), KnobTiming(u32, u32),
    Sw(Press), Boot(Press),
}

/// The longest hold or press a script or the UI may ask for in one command, and the most clicks
/// in one turn: bounds on host input (our choice, not a physical limit).
pub const MAX_HOLD_MS: u32 = 60_000;
pub const MAX_DETENTS: u32 = 256;
/// A press with no duration lasts as long as the machine's generic `press` (esp-soc `machine.rs`).
const DEFAULT_PRESS_MS: u32 = 100;

/// The board's verbs (`<seconds> <verb> <args>` in a script, the same text from the UI):
///
/// ```text
/// ir <key> [hold <ms>]      a remote key, NEC; <key> = ccw cw ok long power bright_up carousel
///                           home bright_down media_toggle | 1..10 | 0xCODE (32-bit)
/// ir down <key> | ir up     hold a key until released (repeats while held)
/// ir tsop <delay_us> <stretch_us>   the receiver's timing for messages sent after it
/// knob cw|ccw [n]           n clicks (default 1)
/// knob detent full|half     the knob's detent type for turns after it
/// knob timing <state_ms> <gap_ms>   quadrature timing for turns after it
/// sw [ms] | sw down | sw up          the knob's switch on GPIO0 (default 100 ms)
/// boot [ms] | boot down | boot up    the BOOT button on GPIO0
/// press sw|knob|boot [ms], release sw|knob|boot   the generic verbs, routed here
/// ```
///
/// `press 0` / `release 0` are refused: GPIO0 is shared, and a raw level would fight the
/// wired-AND. (`gpio 0 <level>` still writes the pin raw, and the next device edge overrides it.)
pub fn parse(cmd: &str, args: &str) -> Option<Result<Command, String>> {
    let a: Vec<&str> = args.split_whitespace().collect();
    let ms = |s: &str| -> Result<u32, String> {
        s.parse::<u32>().ok().filter(|v| *v <= MAX_HOLD_MS).ok_or_else(|| format!("bad duration {s} (0..{MAX_HOLD_MS} ms)"))
    };
    let press = |a: &[&str]| -> Result<Press, String> {
        match a {
            [] => Ok(Press::For(DEFAULT_PRESS_MS)),
            ["down"] => Ok(Press::Down),
            ["up"] => Ok(Press::Up),
            [v] => ms(v).map(Press::For),
            _ => Err(format!("expected [ms] | down | up, got `{}`", a.join(" "))),
        }
    };
    Some(match cmd {
        "ir" => (|| match a.as_slice() {
            ["up"] => Ok(Command::IrUp),
            ["down", key] => remote_code(key).map(Command::IrDown),
            ["tsop", d, s] => {
                let d: u32 = d.parse().map_err(|_| format!("bad TSOP delay {d}"))?;
                let s: i32 = s.parse().map_err(|_| format!("bad mark stretch {s}"))?;
                Tsop::new(d, s).map(Command::IrTsop)
            }
            [key] => remote_code(key).map(|c| Command::IrPress(c, 0)),
            [key, "hold", h] => Ok(Command::IrPress(remote_code(key)?, ms(h)?)),
            _ => Err("ir <key> [hold <ms>] | ir down <key> | ir up | ir tsop <delay_us> <stretch_us>".into()),
        })(),
        "knob" => (|| match a.as_slice() {
            ["detent", "full"] => Ok(Command::KnobDetent(Detent::FullCycle)),
            ["detent", "half"] => Ok(Command::KnobDetent(Detent::HalfCycle)),
            ["timing", s, g] => match (s.parse(), g.parse()) {
                (Ok(s), Ok(g)) if s > 0 && s <= 1000 && g <= 10_000 => Ok(Command::KnobTiming(s, g)),
                _ => Err(format!("bad knob timing {s} {g}")),
            },
            [dir @ ("cw" | "ccw"), rest @ ..] => {
                let n = match rest { [] => 1, [n] => n.parse::<u32>().ok().filter(|n| (1..=MAX_DETENTS).contains(n)).ok_or_else(|| format!("bad click count {n} (1..{MAX_DETENTS})"))?, _ => return Err("knob cw|ccw [n]".into()) };
                Ok(Command::Knob(*dir == "cw", n))
            }
            _ => Err("knob cw|ccw [n] | knob detent full|half | knob timing <state_ms> <gap_ms>".into()),
        })(),
        "sw" => press(&a).map(Command::Sw),
        "boot" => press(&a).map(Command::Boot),
        "press" | "release" => {
            let (target, rest) = match a.split_first() { Some((t, r)) => (*t, r), None => return None };
            let p = if cmd == "release" { if rest.is_empty() { Ok(Press::Up) } else { Err("release <name>".to_string()) } } else { press(rest).and_then(|p| if p == Press::Up || p == Press::Down { Err("press <name> [ms]".into()) } else { Ok(p) }) };
            match target {
                "sw" | "knob" => p.map(Command::Sw),
                "boot" => p.map(Command::Boot),
                "0" => Err("GPIO0 is the wired-AND of the IR receiver, the knob's switch and BOOT on this board: use sw, boot or ir".into()),
                _ => return None,
            }
        }
        _ => return None,
    })
}

#[cfg(test)]
#[path = "panel_inputs_tests.rs"]
mod tests;
