//! Checks of the panel's input waveforms: the NEC timing against IRremoteESP8266's `ir_NEC.h`,
//! and what the firmware's knob sampler makes of the knob, the switch and BOOT. The sampler below
//! is a line-by-line port of FW `src/control/control.cpp` `sampleTick()` (lines 195-270) with its
//! constants, the same logic `tools/twin/research/ctrl_model.py` transcribes; it is a test oracle,
//! not the firmware.
use super::*;

const US: u64 = CYCLES_PER_US;
const MS: u64 = CYCLES_PER_MS;

// ---------------------------------------------------------------- helpers
/// GPIO0's (level, duration in us) runs from a list of edges, starting from idle HIGH at `from`.
fn runs(edges: &[BoardEdge], pin: u8) -> Vec<(bool, u64)> {
    let e: Vec<&BoardEdge> = edges.iter().filter(|e| e.pin == pin).collect();
    e.windows(2).map(|w| {
        assert_ne!(w[0].level, w[1].level, "a pin only ever toggles");
        let d = w[1].cycle - w[0].cycle;
        assert_eq!(d % US, 0, "whole microseconds");
        (w[0].level, d / US)
    }).collect()
}

/// Decode a NEC data message from GPIO0 runs the way the protocol defines it (not the library's
/// tolerant matcher: exact lengths), MSB first like `IRsend::sendData`.
fn exact_nec(r: &[(bool, u64)]) -> u32 {
    assert_eq!(r[0], (false, NEC_HDR_MARK_US as u64));
    assert_eq!(r[1], (true, NEC_HDR_SPACE_US as u64));
    let mut v = 0u32;
    for bit in 0..32 {
        assert_eq!(r[2 + 2 * bit], (false, NEC_BIT_MARK_US as u64));
        let (lvl, space) = r[3 + 2 * bit];
        assert!(lvl);
        v = v << 1 | match space { s if s == NEC_ONE_SPACE_US as u64 => 1, s if s == NEC_ZERO_SPACE_US as u64 => 0, s => panic!("space {s}") };
    }
    assert_eq!(r[66], (false, NEC_BIT_MARK_US as u64), "footer mark");
    v
}

fn run_all(p: &mut PanelInputs, until: VirtualCycle) -> Vec<BoardEdge> { p.advance_to(until); p.take_edges() }

// ---------------------------------------------------------------- the knob sampler, ported
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CtrlEvent { Cw, Ccw, Press, Long }

/// FW `control.cpp:79` kEncTab.
const ENC_TAB: [i32; 16] = [0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0];

struct Ctrl {
    ab_shown: i32, ab_cand: i32, ab_run: u8,
    prev_ab: i32, ab_since: u32, last_enc: i32, enc_dir: i32, last_step: u32, half: i8,
    sw_raw: bool, sw_raw_since: u32, sw_down: bool, sw_down_at: u32, long_sent: bool,
    events: Vec<(u32, CtrlEvent)>,
}
/// FW `control.cpp:90-92` CTRL_ENC_STABLE_MS.
const STABLE: u8 = 2;
/// FW `control.h:28-30` CTRL_ENC_LOCKOUT_MS.
const LOCKOUT_MS: u32 = 10;
/// FW `control.cpp:141-143`: CTRL_SW_DEBOUNCE_MS 20 raised to CTRL_SW_DEBOUNCE_MIN_MS 40, because
/// IR_RX_ENABLED is set (platformio.ini) and IR_PIN 0 == CTRL_PIN_SW 0.
const DEBOUNCE_MS: u32 = 40;
/// FW `control.cpp:75-77`.
const SHORT_MAX_MS: u32 = 500;
const LONG_MS: u32 = 1000;
const REST_SEEN_MS: u32 = 250;

impl Ctrl {
    /// `controlBegin()`, FW `control.cpp:272-286`, with the pins as they are at `now`.
    fn begin(now: u32, a_high: bool, b_high: bool) -> Self {
        let ab = Self::enc_ab(a_high, b_high);
        Ctrl { ab_shown: ab, ab_cand: ab, ab_run: 0, prev_ab: ab, ab_since: now, last_enc: ab * 5, enc_dir: 0, last_step: 0,
               half: -1,   // CTRL_ENC_HALF_DETENT, FW control.h:34-37
               sw_raw: false, sw_raw_since: 0, sw_down: false, sw_down_at: 0, long_sent: false, events: Vec::new() }
    }
    /// FW `control.cpp:174-176`: CTRL_AB_ACTIVE_HIGH, so a HIGH pin is a logical 0.
    fn enc_ab(a_high: bool, b_high: bool) -> i32 { ((!b_high as i32) << 1) | !a_high as i32 }
    /// FW `control.cpp:96-102` filteredAB.
    fn filtered(&mut self, raw: i32) -> i32 {
        if raw == self.ab_shown { self.ab_cand = raw; self.ab_run = 0; return self.ab_shown; }
        if raw != self.ab_cand { self.ab_cand = raw; self.ab_run = 1; } else if self.ab_run < 255 { self.ab_run += 1; }
        if self.ab_run >= STABLE { self.ab_shown = raw; self.ab_run = 0; }
        self.ab_shown
    }
    /// FW `control.cpp:195-270` sampleTick, IR seam off (the remote's seam is exercised by the
    /// host oracle with the real sources, not here).
    fn tick(&mut self, now: u32, a_high: bool, b_high: bool, gpio0_high: bool) {
        let ab = { let raw = Self::enc_ab(a_high, b_high); self.filtered(raw) };
        if ab != self.prev_ab { self.prev_ab = ab; self.ab_since = now; }
        if self.half < 0 && ab == 0 && now.wrapping_sub(self.ab_since) >= REST_SEEN_MS { self.half = 1; }
        if now.wrapping_sub(self.last_step) < LOCKOUT_MS {
            self.enc_dir = 0;
            self.last_enc = ab * 5;
        } else {
            self.last_enc = (self.last_enc >> 2) | (ab << 2);
            self.enc_dir += ENC_TAB[(self.last_enc & 0x0F) as usize];
        }
        let at_detent = ab == 0b11 || (self.half == 1 && ab == 0b00);
        if self.enc_dir != 0 && at_detent {
            self.last_step = now;
            let forward = self.enc_dir > 0;   // s_cfgReverse false
            self.enc_dir = 0;
            self.last_enc = ab * 5;
            self.events.push((now, if forward { CtrlEvent::Cw } else { CtrlEvent::Ccw }));
        }
        let raw = !gpio0_high;
        if raw != self.sw_raw { self.sw_raw = raw; self.sw_raw_since = now; }
        if raw != self.sw_down && now.wrapping_sub(self.sw_raw_since) >= DEBOUNCE_MS {
            if raw { self.sw_down_at = now; self.long_sent = false; }
            else if !self.long_sent && now.wrapping_sub(self.sw_down_at) < SHORT_MAX_MS { self.events.push((now, CtrlEvent::Press)); }
            self.sw_down = raw;
        }
        if self.sw_down && !self.long_sent && now.wrapping_sub(self.sw_down_at) >= LONG_MS {
            self.long_sent = true;
            self.events.push((now, CtrlEvent::Long));
        }
    }
}

/// Run the scheduled inputs through the sampler: one sample per millisecond (`kSampleUs`, FW
/// `control.cpp:74`) at `phase_us` into each millisecond, for `ms` milliseconds. `millis()` at a
/// sample is the whole milliseconds, as `esp_timer_get_time() / 1000`.
fn sample(p: &mut PanelInputs, ms: u32, phase_us: u64) -> Ctrl {
    let mut c = Ctrl::begin(0, p.level(PIN_ENC_A).unwrap(), p.level(PIN_ENC_B).unwrap());
    for k in 1..=ms {
        p.advance_to((k as u64 * 1000 + phase_us) * US);
        c.tick(k, p.level(PIN_ENC_A).unwrap(), p.level(PIN_ENC_B).unwrap(), p.level(PIN_GPIO0).unwrap());
    }
    c
}
fn kinds(c: &Ctrl) -> Vec<CtrlEvent> { c.events.iter().map(|e| e.1).collect() }
const PHASES: [u64; 10] = [0, 100, 200, 300, 400, 500, 600, 700, 800, 999];

// ---------------------------------------------------------------- NEC against ir_NEC.h
#[test]
fn nec_constants_are_ir_nec_h() {
    // IRR ir_NEC.h:27-46, restated as numbers.
    assert_eq!((NEC_HDR_MARK_US, NEC_HDR_SPACE_US, NEC_BIT_MARK_US), (8960, 4480, 560));
    assert_eq!((NEC_ONE_SPACE_US, NEC_ZERO_SPACE_US, NEC_RPT_SPACE_US), (1680, 560, 2240));
    assert_eq!((NEC_MESSAGE_US, NEC_MIN_GAP_US, NEC_BITS), (108_080, 22_400, 32));
    assert_eq!(CYCLES_PER_US, 240, "esp32s3 periph::CPU_HZ is 240 MHz");
}

#[test]
fn every_message_fills_one_108_ms_slot() {
    for b in OWNER_REMOTE {
        let m = nec_message(b.code);
        assert_eq!(m.len(), 34, "header, 32 bits, footer");
        assert_eq!(m.iter().map(|(a, s)| a + s).sum::<u32>(), NEC_MESSAGE_US);
        // The owner's codes have 16 ones: 8960+4480+16*2240+16*1120+560 = 67,760 us of message,
        // leaving 40,320 us, more than the 15 ms that ends a capture (FW src/ir/ir.cpp:69).
        assert_eq!(b.code.count_ones(), 16);
        assert_eq!(m[33], (560, 40_320));
    }
    assert_eq!(nec_repeat(), vec![(8960, 2240), (560, 96_320)]);
    // A code of all ones is the longest message: the pad is kNecMinGap exactly.
    assert_eq!(nec_message(u32::MAX)[33], (560, NEC_MIN_GAP_US));
}

#[test]
fn the_owner_codes_are_nec_address_0_with_a_valid_command() {
    // IRR ir_NEC.cpp:115-136: command byte inverted in the low byte; address 0x00, inverted 0xFF.
    for b in OWNER_REMOTE {
        assert_eq!(b.code >> 16, 0x00FF, "button {}", b.number);
        assert_eq!((b.code >> 8 & 0xFF) ^ 0xFF, b.code & 0xFF, "button {}", b.number);
        assert_eq!(remote_code(b.name), Ok(b.code));
        assert_eq!(remote_code(&b.number.to_string()), Ok(b.code));
    }
    assert_eq!(remote_code("OK"), Ok(0x00FF_E01F));
    assert_eq!(remote_code("0x20DF10EF"), Ok(0x20DF_10EF));
    assert!(remote_code("11").is_err() && remote_code("menu").is_err() && remote_code("0x1FFFFFFFF").is_err());
}

#[test]
fn one_press_is_one_data_message_on_gpio0() {
    for b in OWNER_REMOTE {
        let mut p = PanelInputs::new();
        p.ir_press(0, b.code, 0);
        let edges = run_all(&mut p, 300 * MS);
        assert!(edges.iter().all(|e| e.pin == PIN_GPIO0));
        assert_eq!(edges.len(), 68, "34 bursts, a fall and a rise each");
        assert_eq!(edges[0], BoardEdge { cycle: 1, pin: PIN_GPIO0, level: false }, "active low, from the first cycle after the command");
        assert_eq!(exact_nec(&runs(&edges, PIN_GPIO0)), b.code, "button {}", b.number);
        assert!(!p.busy(), "the message slot has ended");
        assert_eq!(p.level(PIN_GPIO0), Some(true), "idle high");
        assert_eq!((p.counts.ir_messages, p.counts.ir_repeats), (1, 0));
    }
}

#[test]
fn a_held_key_repeats_every_108_08_ms_while_down() {
    // hold 800 ms: repeats at the slot boundaries k * 108.08 ms still before the release,
    // k = 1..7 (7 * 108.08 = 756.56 < 800 <= 864.64).
    let mut p = PanelInputs::new();
    p.ir_press(0, remote_code("ok").unwrap(), 800);
    let edges = run_all(&mut p, 2000 * MS);
    let falls: Vec<u64> = edges.iter().filter(|e| !e.level).map(|e| e.cycle).collect();
    assert_eq!(edges.len(), 68 + 7 * 4);
    assert_eq!(p.counts.ir_repeats, 7);
    let r = runs(&edges, PIN_GPIO0);
    assert_eq!(exact_nec(&r), 0x00FF_E01F);
    for k in 1..=7u64 {
        let start = 1 + k * NEC_MESSAGE_US as u64 * US;
        let i = 34 + (k as usize - 1) * 2;   // falls: 34 of the data message, then 2 per repeat
        assert_eq!(falls[i], start, "repeat {k} starts on its slot");
        assert_eq!(falls[i + 1] - falls[i], (NEC_HDR_MARK_US + NEC_RPT_SPACE_US) as u64 * US, "8960 mark + 2240 space");
        let rise = edges.iter().find(|e| e.level && e.cycle > falls[i + 1]).unwrap().cycle;
        assert_eq!(rise - falls[i + 1], NEC_BIT_MARK_US as u64 * US, "560 us footer");
    }
    // The boundary: a hold shorter than one slot gives no repeat, one just past it gives one.
    for (hold, want) in [(108, 0), (109, 1), (217, 2)] {
        let mut p = PanelInputs::new();
        p.ir_press(0, 0x00FF_708F, hold);
        run_all(&mut p, 1000 * MS);
        assert_eq!(p.counts.ir_repeats, want, "hold {hold} ms");
    }
}

#[test]
fn ir_down_holds_until_ir_up_and_keys_queue_behind_each_other() {
    let mut p = PanelInputs::new();
    p.ir_down(0, remote_code("cw").unwrap());
    p.advance_to(500 * MS);
    p.ir_up(500 * MS);          // down for 500 ms: repeats at 108, 216, 324, 432
    p.ir_press(500 * MS, remote_code("ccw").unwrap(), 0);   // waits for the slot to end
    let edges = { let mut e = p.take_edges(); e.extend(run_all(&mut p, 2000 * MS)); e };
    assert_eq!(p.counts.ir_repeats, 4);
    let slot = NEC_MESSAGE_US as u64 * US;
    // the second key starts where the first key's last slot (the 4th repeat) ends
    let second = 1 + 5 * slot;
    let tail: Vec<BoardEdge> = edges.iter().copied().filter(|e| e.cycle >= second).collect();
    assert_eq!(tail[0], BoardEdge { cycle: second, pin: PIN_GPIO0, level: false });
    assert_eq!(exact_nec(&runs(&tail, PIN_GPIO0)), 0x00FF_708F);
    assert!(!p.busy());
    // `ir up` with nothing held changes nothing
    p.ir_up(2000 * MS);
    assert!(!p.busy());

    // A key pressed and released while the one before it is still in its slot keeps its own
    // hold: down 60..400 ms is 340 ms, so 3 repeats once it gets the air.
    let mut p = PanelInputs::new();
    p.ir_down(0, 0x00FF_58A7);
    p.ir_up(50 * MS);
    p.ir_down(60 * MS, 0x00FF_708F);
    p.ir_up(400 * MS);
    run_all(&mut p, 2000 * MS);
    assert_eq!((p.counts.ir_messages, p.counts.ir_repeats), (2, 3));
}

#[test]
fn tsop_delay_and_mark_stretch_move_the_edges() {
    let mut p = PanelInputs::new();
    p.set_tsop(Tsop::new(150, 92).unwrap());
    p.ir_press(0, 0x00FF_E01F, 0);
    let edges = run_all(&mut p, 300 * MS);
    assert_eq!(edges[0].cycle, 1 + 150 * US, "td");
    let r = runs(&edges, PIN_GPIO0);
    assert_eq!(r[0], (false, 8960 + 92));
    assert_eq!(r[1], (true, 4480 - 92));
    assert_eq!(r[2], (false, 560 + 92));
    assert!(r.iter().all(|(l, d)| if *l { *d == 560 - 92 || *d == 1680 - 92 || *d == 4480 - 92 } else { *d == 652 || *d == 9052 }));
    // Vishay 82460 p.3 Fig. 1 at 38 kHz, as constants; the model refuses what breaks a message.
    assert_eq!((TSOP_DELAY_US, TSOP_STRETCH_US), ((105, 263), (-79, 92)));
    assert!(Tsop::new(0, 560).is_err() && Tsop::new(0, -560).is_err() && Tsop::new(1001, 0).is_err());
    assert!(Tsop::new(263, -79).is_ok() && Tsop::new(0, 200).is_ok(), "the host decoder test fails at +200: allowed on purpose");
}

// ---------------------------------------------------------------- GPIO0 wired-AND
#[test]
fn gpio0_is_low_while_any_source_pulls_it() {
    let mut p = PanelInputs::new();
    assert_eq!(p.input_levels(), vec![(0, true), (45, false), (46, false)], "idle: GPIO0 pulled up, IO45/IO46 pulled down");
    p.boot(0, true);
    p.ir_press(0, 0x00FF_E01F, 0);       // sent while BOOT holds the line low: invisible
    p.advance_to(50 * MS);
    p.boot(50 * MS, false);              // released mid-message: the rest of the message shows
    let edges = run_all(&mut p, 300 * MS);
    assert_eq!(edges[0], BoardEdge { cycle: 1, pin: 0, level: false });
    let after: Vec<&BoardEdge> = edges.iter().filter(|e| e.cycle > 50 * MS).collect();
    assert!(!after.is_empty() && after.iter().all(|e| e.cycle > 50 * MS + 1 || e.level));
    // Every edge really is a change, and the level at the end is idle.
    for w in edges.windows(2) { assert_ne!(w[0].level, w[1].level); }
    assert_eq!(p.level(0), Some(true));

    // Two switch presses queued back to back meet at one cycle: one long low, no glitch.
    let mut p = PanelInputs::new();
    p.sw_press(0, 100);
    p.sw_press(0, 100);
    let edges = run_all(&mut p, 500 * MS);
    assert_eq!(edges, vec![BoardEdge { cycle: 1, pin: 0, level: false }, BoardEdge { cycle: 1 + 200 * MS, pin: 0, level: true }]);

    // The switch held across an IR message: GPIO0 stays low throughout, no edge from the TSOP.
    let mut p = PanelInputs::new();
    p.sw_press(0, 150);
    p.ir_press(0, 0x00FF_E01F, 0);
    let edges = run_all(&mut p, 500 * MS);
    let inside = edges.iter().filter(|e| e.cycle > 1 && e.cycle < 1 + 150 * MS).count();
    assert_eq!(inside, 0);
}

// ---------------------------------------------------------------- the knob
#[test]
fn a_full_cycle_click_is_four_edges_3_ms_apart() {
    let mut p = PanelInputs::new();
    p.knob_turn(0, true, 2);
    p.knob_turn(0, false, 1);
    let e = run_all(&mut p, 200 * MS);
    let s = KNOB_STATE_MS as u64 * MS;
    let g = KNOB_GAP_MS as u64 * MS;
    let t1 = 1 + 3 * s + g;
    let t2 = t1 + 3 * s + g;
    assert_eq!(e, vec![
        BoardEdge { cycle: 1, pin: 45, level: true }, BoardEdge { cycle: 1 + s, pin: 46, level: true },
        BoardEdge { cycle: 1 + 2 * s, pin: 45, level: false }, BoardEdge { cycle: 1 + 3 * s, pin: 46, level: false },
        BoardEdge { cycle: t1, pin: 45, level: true }, BoardEdge { cycle: t1 + s, pin: 46, level: true },
        BoardEdge { cycle: t1 + 2 * s, pin: 45, level: false }, BoardEdge { cycle: t1 + 3 * s, pin: 46, level: false },
        BoardEdge { cycle: t2, pin: 46, level: true }, BoardEdge { cycle: t2 + s, pin: 45, level: true },
        BoardEdge { cycle: t2 + 2 * s, pin: 46, level: false }, BoardEdge { cycle: t2 + 3 * s, pin: 45, level: false },
    ]);
}

#[test]
fn the_sampler_counts_full_cycle_clicks_at_any_phase() {
    for phase in PHASES {
        let mut p = PanelInputs::new();
        p.knob_turn(50 * MS, true, 3);
        p.knob_turn(50 * MS, false, 2);
        let c = sample(&mut p, 400, phase);
        assert_eq!(kinds(&c), [CtrlEvent::Cw, CtrlEvent::Cw, CtrlEvent::Cw, CtrlEvent::Ccw, CtrlEvent::Ccw], "phase {phase} us");
        assert_eq!(c.half, -1, "a full-cycle knob never rests at 00");
    }
}

#[test]
fn one_ms_states_are_filtered_out_and_short_gaps_lose_clicks() {
    // FW control.cpp:81-102: a state must hold 2 samples.
    let mut p = PanelInputs::new();
    p.set_knob_timing(1, 12).unwrap();
    p.knob_turn(50 * MS, true, 5);
    assert_eq!(kinds(&sample(&mut p, 300, 500)), [], "1 ms a state");
    // The research's model case: a second full click 3 ms after the first ends is swallowed by
    // the 10 ms lockout (FW control.h:28-30).
    let mut p = PanelInputs::new();
    p.set_knob_timing(3, 3).unwrap();
    p.knob_turn(50 * MS, true, 2);
    assert_eq!(kinds(&sample(&mut p, 300, 500)), [CtrlEvent::Cw]);
}

#[test]
fn a_half_cycle_knob_is_learned_from_a_250_ms_rest_then_every_click_counts() {
    for phase in PHASES {
        let mut p = PanelInputs::new();
        p.set_detent(Detent::HalfCycle);
        p.knob_turn(100 * MS, true, 1);          // 11 -> 00, then rests
        let c = sample(&mut p, 450, phase);
        // FW control.cpp:221,239: the step waits until 00 has rested 250 ms.
        assert_eq!(kinds(&c), [CtrlEvent::Cw], "phase {phase}");
        let at = c.events[0].0;
        assert!((354..=355).contains(&at), "first half click delivered at {at} ms, 250 ms after the 00 rest began");
        assert_eq!(c.half, 1);
    }
    // Once learned, ten quick half clicks each way all count.
    let mut p = PanelInputs::new();
    p.set_detent(Detent::HalfCycle);
    p.knob_turn(100 * MS, true, 1);
    p.knob_turn(500 * MS, true, 10);
    p.knob_turn(500 * MS, false, 10);
    let c = sample(&mut p, 1500, 300);
    assert_eq!(kinds(&c).iter().filter(|e| **e == CtrlEvent::Cw).count(), 11);
    assert_eq!(kinds(&c).iter().filter(|e| **e == CtrlEvent::Ccw).count(), 10);
}

#[test]
fn a_half_click_needs_the_full_10_ms_lockout_after_the_step() {
    // A half click scores once (one +-1 transition), so it is lost if that transition lands inside
    // the 10 ms lockout after the previous step (FW control.h:28-30, control.cpp:223-225). From
    // the last edge of one click to the first of the next, 10 ms is the least that works at every
    // phase; at 9 ms every other click is lost. A full click scores twice and survives down to
    // ~4 ms (probed: 6 ms still counts all 20), so the half-cycle knob sets the bound.
    for (gap, want) in [(9, 1 + 10), (10, 1 + 20), (KNOB_GAP_MS, 1 + 20)] {
        for phase in PHASES {
            let mut p = PanelInputs::new();
            p.set_detent(Detent::HalfCycle);
            p.knob_turn(100 * MS, true, 1);          // learn the 00 rest first
            p.set_knob_timing(KNOB_STATE_MS, gap).unwrap();
            p.knob_turn(500 * MS, true, 20);
            assert_eq!(kinds(&sample(&mut p, 1200, phase)).len(), want, "gap {gap} ms, phase {phase}");
        }
    }
}

// ---------------------------------------------------------------- the switch and BOOT
#[test]
fn switch_and_boot_presses_as_the_sampler_sees_them() {
    let press = |f: &dyn Fn(&mut PanelInputs), ms: u32| { let mut p = PanelInputs::new(); f(&mut p); sample(&mut p, ms, 500).events };
    // FW control.cpp:260-269: pressed after 40 ms of LOW, a click if released within 500 ms.
    assert_eq!(press(&|p| p.sw_press(100 * MS, 150), 900), [(290, CtrlEvent::Press)]);
    assert_eq!(press(&|p| p.boot_press(100 * MS, 150), 900), [(290, CtrlEvent::Press)], "BOOT is the same line");
    assert_eq!(press(&|p| p.sw_press(100 * MS, 30), 900), [], "shorter than the debounce");
    // A long press fires once, 1000 ms after the debounced press (~1040 ms after the edge).
    assert_eq!(press(&|p| p.sw_press(100 * MS, 1100), 1800), [(1140, CtrlEvent::Long)]);
    // Held 500..1000 ms: neither a click nor a long press.
    assert_eq!(press(&|p| p.sw_press(100 * MS, 700), 1800), []);
    // UI-style down/up.
    assert_eq!(press(&|p| { p.sw(100 * MS, true); p.sw(250 * MS, false); }, 900), [(290, CtrlEvent::Press)]);
}

#[test]
fn nec_on_gpio0_never_clicks_the_switch() {
    // ADR-TWIN-01 B6 / research model case (c): the 40 ms debounce rejects IR marks (<= 8.96 ms).
    for phase in PHASES {
        let mut p = PanelInputs::new();
        p.ir_press(100 * MS, remote_code("ok").unwrap(), 1100);   // one message + 10 repeats
        p.set_tsop(Tsop::new(TSOP_DELAY_US.1, TSOP_STRETCH_US.1).unwrap());
        let c = sample(&mut p, 2000, phase);
        assert_eq!(c.events, [], "phase {phase}");
        assert_eq!(p.counts.ir_repeats, 10);
    }
}

// ---------------------------------------------------------------- verbs
#[test]
fn script_verbs_parse() {
    use Command::*;
    let ok = |c: &str, a: &str| parse(c, a).expect("ours").expect("valid");
    assert_eq!(ok("ir", "ok"), IrPress(0x00FF_E01F, 0));
    assert_eq!(ok("ir", "ok hold 800"), IrPress(0x00FF_E01F, 800));
    assert_eq!(ok("ir", "3"), IrPress(0x00FF_E01F, 0));
    assert_eq!(ok("ir", "0x00FF708F hold 50"), IrPress(0x00FF_708F, 50));
    assert_eq!(ok("ir", "down media_toggle"), IrDown(0x00FF_C936));
    assert_eq!(ok("ir", "up"), IrUp);
    assert_eq!(ok("ir", "tsop 150 92"), IrTsop(Tsop { delay_us: 150, mark_stretch_us: 92 }));
    assert_eq!(ok("knob", "cw 2"), Knob(true, 2));
    assert_eq!(ok("knob", "ccw"), Knob(false, 1));
    assert_eq!(ok("knob", "detent half"), KnobDetent(Detent::HalfCycle));
    assert_eq!(ok("knob", "timing 3 12"), KnobTiming(3, 12));
    assert_eq!(ok("sw", "150"), Sw(Press::For(150)));
    assert_eq!(ok("sw", ""), Sw(Press::For(100)));
    assert_eq!(ok("sw", "down"), Sw(Press::Down));
    assert_eq!(ok("boot", "up"), Boot(Press::Up));
    assert_eq!(ok("press", "sw 150"), Sw(Press::For(150)));
    assert_eq!(ok("press", "knob"), Sw(Press::For(100)));
    assert_eq!(ok("release", "boot"), Boot(Press::Up));
    for (c, a) in [("ir", ""), ("ir", "menu"), ("ir", "ok hold"), ("ir", "ok hold 99999999"), ("ir", "tsop 0 600"), ("knob", "cw 0"), ("knob", "cw 257"),
                   ("knob", "left"), ("knob", "detent quarter"), ("sw", "x"), ("press", "0"), ("release", "0"), ("press", "sw down"), ("release", "sw 5")] {
        assert!(matches!(parse(c, a), Some(Err(_))), "{c} {a}");
    }
    for (c, a) in [("press", "btn1"), ("gpio", "0 1"), ("serial", "ir"), ("touch", "1 2 1"), ("press", "")] {
        assert!(parse(c, a).is_none(), "{c} {a} is the machine's");
    }
}

#[test]
fn input_at_schedules_from_the_observed_cycle() {
    let mut p = PanelInputs::new();
    p.advance_to(10 * MS);
    p.input_at(5 * MS, "sw", "20").unwrap();           // observed in the board's past: starts now
    p.input_at(10 * MS, "knob", "cw").unwrap();
    assert!(p.input_at(10 * MS, "frob", "").is_err());
    assert!(p.input_at(10 * MS, "ir", "nope").is_err());
    let e = run_all(&mut p, 100 * MS);
    assert_eq!(e[0], BoardEdge { cycle: 10 * MS + 1, pin: 0, level: false });
    assert_eq!(e.iter().filter(|e| e.pin == 45 || e.pin == 46).count(), 4);
    assert!(p.report().contains("knob 1 clicks"));
}
