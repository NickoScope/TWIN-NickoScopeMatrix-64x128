//! The `hub75-panel` board's inputs through a whole machine: script verbs, the board's deadline
//! bounding the device tick (each edge reaches the GPIO model on the cycle it was scheduled for,
//! `SocBus::board_edge_lag`), the edges latching the GPIO edge interrupt.
use esp32s3::board::panel_inputs::{remote_code, NEC_MESSAGE_US, PIN_ENC_A, PIN_ENC_B, PIN_GPIO0};
use esp_soc::{ScriptAction, SocBus, Stop};

const IRAM: u32 = 0x4037_0000;
const WAITI_LOOP: [u8; 6] = [0x00, 0x70, 0x00, 0x06, 0xff, 0xff];   // waiti 0 ; j .  (the core sleeps; time jumps between deadlines)
const US: u64 = 240;

fn machine() -> esp32s3::Machine {
    let mut m = esp32s3::machine([1, 2, 3, 4, 5, 6]);
    m.console.capture = true;
    m.quantum = 64;
    SocBus::load_bytes(&mut m.bus, IRAM, &WAITI_LOOP).unwrap();
    m.cores[0].pc = IRAM;
    m.cores[0].ps = 0;
    m.bus.board = esp32s3::board::make_board("hub75-panel").unwrap();
    m.bus.attach_board_devices();
    m
}

fn bit(m: &esp32s3::Machine, pin: u8) -> bool { m.bus.periph.gpio.input >> pin & 1 != 0 }

#[test]
fn the_board_holds_its_idle_levels_from_attach() {
    let m = machine();
    assert_eq!(m.bus.board.name(), "hub75-panel");
    assert!(bit(&m, PIN_GPIO0), "GPIO0 pulled up");
    assert!(!bit(&m, PIN_ENC_A) && !bit(&m, PIN_ENC_B), "IO45/IO46 pulled down");
    assert_eq!(m.bus.board.named_pin("sw"), None, "no raw pin: GPIO0 belongs to the wired-AND");
    assert_eq!(m.bus.board.encoder(), None, "no raw quadrature: the board models its knob");
}

#[test]
fn script_verbs_become_board_actions_and_generic_ones_stay() {
    let mut m = machine();
    m.load_script("0.001 ir ok\n0.002 ir ok hold 800\n0.1 knob cw 2\n0.2 sw 150\n0.3 press boot 50\n0.4 gpio 17 0\n0.5 stop\n").unwrap();
    let ev = &m.script.events;
    assert!(matches!(&ev[0].1, ScriptAction::Board(c, a) if c == "ir" && a == "ok"));
    assert!(matches!(&ev[1].1, ScriptAction::Board(c, a) if c == "ir" && a == "ok hold 800"));
    assert!(matches!(&ev[2].1, ScriptAction::Board(c, a) if c == "knob" && a == "cw 2"));
    assert!(matches!(&ev[3].1, ScriptAction::Board(c, a) if c == "sw" && a == "150"));
    assert!(matches!(&ev[4].1, ScriptAction::Board(c, a) if c == "press" && a == "boot 50"));
    assert!(matches!(ev[5].1, ScriptAction::Gpio(17, false)));
    for bad in ["1.0 ir menu", "1.0 knob cw 0", "1.0 press 0", "1.0 sw forever"] {
        let e = m.load_script(bad).unwrap_err();
        assert!(e.starts_with("line 1:"), "{bad}: {e}");
    }
}

#[test]
fn a_remote_press_reaches_gpio0_on_its_cycles_and_latches_the_change_interrupt() {
    let mut m = machine();
    // Arduino attachInterrupt(pin, isr, CHANGE) writes int_type 3 (IRR IRrecv.cpp:413; arduino-esp32
    // 2.0.17 CHANGE = 0x03); no INT_ENA, so the status latches without taking the core away.
    m.bus.periph.gpio.pin[PIN_GPIO0 as usize] = 3 << 7;
    m.load_script("0.001 ir ok\n0.2 knob ccw 1\n0.3 sw 50\n0.4 stop\n").unwrap();
    SocBus::observe_gpio(&mut m.bus, true);
    assert!(matches!(m.run(u64::MAX), Stop::Halted));
    let ev = SocBus::take_gpio_events(&mut m.bus);
    let g0: Vec<(u64, bool)> = ev.iter().filter(|e| e.1 == PIN_GPIO0).map(|e| (e.0, e.2)).collect();
    assert_eq!(g0.len(), 68 + 2, "a data message, then the switch press");
    let start = g0[0].0;
    assert!((240_001..=240_001 + 64 * 4).contains(&start), "first edge {start} just after the script time");
    // Each edge reached the GPIO model on the cycle the board scheduled it for: the board's
    // deadline bounds the device tick. Without that bound the edges arrive up to a tick backstop
    // late and batched (review 4 measured 32,639 cycles), which IRrecv's ISR would timestamp.
    assert_eq!(m.bus.board_edge_lag, 0, "a board edge reached GPIO_IN after its cycle");
    // The NEC message's runs, exact to the cycle, as the board scheduled them (and, by the lag
    // above, as the GPIO model saw them).
    let widths: Vec<u64> = g0[..68].windows(2).map(|w| (w[1].0 - w[0].0) / US).collect();
    assert_eq!(&widths[..3], [8960, 4480, 560]);
    let mut code = 0u32;
    for b in 0..32 { code = code << 1 | (widths[3 + 2 * b] == 1680) as u32; }
    assert_eq!(code, remote_code("ok").unwrap());
    assert!(g0[68].0 >= 300 * 240_000 && !g0[68].1 && g0[69].1 && (g0[69].0 - g0[68].0) == 50 * 240_000);
    // The knob: B leads anticlockwise.
    let ab: Vec<(u8, bool)> = ev.iter().filter(|e| e.1 == PIN_ENC_A || e.1 == PIN_ENC_B).map(|e| (e.1, e.2)).collect();
    assert_eq!(ab, [(46, true), (45, true), (46, false), (45, false)]);
    // The pins really moved, and ended idle; GPIO0's CHANGE status latched.
    assert!(bit(&m, PIN_GPIO0) && !bit(&m, PIN_ENC_A) && !bit(&m, PIN_ENC_B));
    assert!(m.bus.periph.gpio.status & 1 != 0);
    assert!(m.bus.board.report().contains("IR 1 messages + 0 repeats"), "{}", m.bus.board.report());
    assert!(NEC_MESSAGE_US as u64 * US < 200 * 240_000, "the message was over before the knob turned");
}

#[test]
fn the_page_drives_the_board_verbs_and_bad_lines_are_dropped() {
    let mut m = machine();
    let web = esp_soc::web::WebServer::queued();
    m.web = Some(web.clone());
    for msg in [r#"{"t":"input","line":"ir menu"}"#, r#"{"t":"input","line":"frobnicate 1"}"#, r#"{"t":"input","line":""}"#] { web.push_incoming(msg.to_string()); }
    assert!(matches!(m.run(0), Stop::MaxInsns));
    assert_eq!(m.bus.board.next_deadline(), None, "nothing scheduled from lines the board refuses");
    web.push_incoming(r#"{"t":"input","line":"ir down ok"}"#.to_string());
    web.push_incoming(r#"{"t":"knobpress","v":"1"}"#.to_string());
    web.push_incoming(r#"{"t":"knob","d":"-2"}"#.to_string());
    assert!(matches!(m.run(0), Stop::MaxInsns));
    assert_eq!(m.bus.board.next_deadline(), Some(1), "the key, the switch and the knob start on the next cycle");
    assert!(m.script.events.is_empty(), "no raw GPIO actions queued for this board");
    m.load_script("0.3 stop\n").unwrap();
    SocBus::observe_gpio(&mut m.bus, true);
    web.push_incoming(r#"{"t":"input","line":"ir up"}"#.to_string());
    assert!(matches!(m.run(u64::MAX), Stop::Halted));
    let ev = SocBus::take_gpio_events(&mut m.bus);
    assert!(ev.iter().any(|e| e.1 == PIN_ENC_B && e.2), "anticlockwise: B leads");
    assert!(!bit(&m, PIN_GPIO0), "the switch is still held down");
}

/// Review 5: BOOT (or the knob's switch) held through the board's RESET, a power-on Chip Reset,
/// latches GPIO0 = 0 with IO46 = 0 (R60 pull-down): joint download boot, as on the hardware and
/// as the firmware's `control.cpp` expects. Released, the next reset boots from SPI flash. A
/// software reset does not sample the pins. Only GPIO_STRAPPING[3:2] follow the board.
#[test]
fn boot_held_through_a_power_on_reset_latches_download_mode() {
    let mut m = machine();
    let hold = |m: &mut esp32s3::Machine, verb: &str, line: &str| {
        m.bus.board_input(verb, line).unwrap();
        m.max_cycles = m.bus.cycles + 10 * 240_000;           // 10 ms: the edge lands, the core sleeps
        assert!(matches!(m.run(u64::MAX), Stop::Halted));
    };
    let reset = |m: &mut esp32s3::Machine, cause: u32| -> u32 {
        m.bus.request_reset(cause);
        assert_eq!(m.reboot(), cause);
        m.cores[0].pc = IRAM; m.cores[0].ps = 0;             // back to the sleeping loop (SRAM survives)
        m.bus.periph.gpio.strap
    };
    m.bus.set_strap(0x2f);                                  // --strap: bit 5 is not the board's
    hold(&mut m, "boot", "down");
    assert!(!bit(&m, PIN_GPIO0));
    let strap = reset(&mut m, esp_periph::RST_SW_SYS);
    assert_eq!(strap, 0x2f, "a software reset keeps the latch");
    let strap = reset(&mut m, esp_periph::RST_RTCWDT_RTC);
    assert_eq!(strap, 0x2f, "an RTC watchdog System Reset keeps the latch (TRM Table 7.1-1)");
    let strap = reset(&mut m, esp_periph::RST_POWERON);
    assert_eq!(strap & 0x0c, 0, "GPIO0 = 0, GPIO46 = 0: joint download (boot_mode.h IS_00XX)");
    assert_eq!(strap, 0x23, "the other bits stay as --strap set them");
    assert!(!bit(&m, PIN_GPIO0), "BOOT is still held after the reset");
    hold(&mut m, "boot", "up");
    let strap = reset(&mut m, esp_periph::RST_POWERON);
    assert_eq!(strap, 0x2b, "GPIO0 = 1: SPI boot (IS_1XXX); IO46 rests low");
    // The knob's switch is on the same wired-AND.
    hold(&mut m, "sw", "down");
    assert_eq!(reset(&mut m, esp_periph::RST_POWERON) & 0x0c, 0);
}
