//! The USB host at the USB-Serial/JTAG, through the whole machine and without a ROM: the DTR/RTS
//! sequences of ESP32-S3 TRM v1.8 Tables 33.4-3 and 33.4-4 (p.1254) reset the chip with cause 0x15
//! and the strap the ROM will read, the chip is held while RTS=1/DTR=0, the strap latch of a
//! power-on reset (BOOT held) and the USB reset agree, and the reserved data window esptool-js
//! reads answers 0. The core sleeps in `waiti` in IRAM; after each reset it is put back there
//! (SRAM survives a reset) in place of the ROM.
use emu_core::{Bus, Core};
use esp_soc::usj_port::{PortOut, Session, UsjPort};
use esp_soc::{SocBus, Stop};
use std::sync::mpsc::Receiver;

const IRAM: u32 = 0x4037_0000;
const WAITI_LOOP: [u8; 6] = [0x00, 0x70, 0x00, 0x06, 0xff, 0xff];   // waiti 0 ; j .
const MS: u64 = 240_000;

fn machine(board: &str) -> esp32s3::Machine {
    let mut m = esp32s3::machine([1, 2, 3, 4, 5, 6]);
    m.console.capture = true;
    m.bus.board = esp32s3::board::make_board(board).unwrap();
    m.bus.attach_board_devices();
    SocBus::load_bytes(&mut m.bus, IRAM, &WAITI_LOOP).unwrap();
    park(&mut m);
    m.usj_reset = true;
    m
}

fn park(m: &mut esp32s3::Machine) { m.cores[0].pc = IRAM; m.cores[0].ps = 0; }

fn run_ms(m: &mut esp32s3::Machine, ms: u64) -> Stop {
    m.max_cycles = m.bus.cycles + ms * MS;
    m.run(u64::MAX)
}

/// Run until the chip resets, reboot it as the front end does, and put the core back to sleep.
fn expect_reset(m: &mut esp32s3::Machine) -> (u32, u32) {
    let stop = run_ms(m, 5);
    assert!(matches!(stop, Stop::SwReset), "no reset: {stop:?}");
    let cause = m.reboot();
    park(m);
    (cause, m.bus.periph.gpio.strap)
}

fn open(m: &mut esp32s3::Machine) -> (UsjPort, Session, Receiver<PortOut>) {
    let port = UsjPort::new();
    m.attach_usj(port.clone());
    let (tx, rx) = std::sync::mpsc::channel();
    let s = port.open(Box::new(move |o| tx.send(o).is_ok())).unwrap();
    (port, s, rx)
}

fn events(rx: &Receiver<PortOut>) -> Vec<String> {
    rx.try_iter().filter_map(|o| match o { PortOut::Event(e) => Some(e), PortOut::Data(_) => None }).collect()
}

/// (DTR, RTS) steps as the host sets them, one request each.
fn lines(s: &Session, seq: &[(u8, u8)]) { for &(dtr, rts) in seq { s.lines(dtr != 0, rts != 0); } }

#[test]
fn trm_sequences_reset_into_download_then_into_flash_boot() {
    let mut m = machine("none");
    let (_port, s, rx) = open(&mut m);
    assert!(matches!(run_ms(&mut m, 1), Stop::Halted));
    assert_eq!(events(&rx), [r#"{"t":"hello","proto":1,"chip":"esp32s3","dtr":0,"rts":0,"held":false,"reset":true}"#]);
    assert_eq!(m.bus.periph.gpio.strap, 0x0f, "--strap default: GPIO0 = 1, SPI boot");

    // Table 33.4-3 up to "Clear DTR  RTS=1, DTR=0  Reset SoC".
    lines(&s, &[(0, 0), (0, 0), (1, 0), (1, 0), (1, 1), (0, 1)]);
    let (cause, strap) = expect_reset(&mut m);
    assert_eq!(cause, esp_periph::RST_USB_UART_CHIP);
    assert_eq!(esp_periph::reset_cause_name(cause), "USB_UART_CHIP_RESET");
    assert_eq!(strap & 0x0c, 0, "GPIO_STRAPPING[3:2] = 00: joint download (boot_mode.h IS_00XX)");
    assert_eq!(strap, 0x03, "the other bits stay as latched");
    assert_eq!(m.bus.periph.rtc.ram.read(0x38) & 0x3f, 0x15, "RESET_STATE carries the cause for the ROM");
    assert_eq!(events(&rx), [r#"{"t":"reset","cause":21,"download":true}"#]);

    // Held while the lines stay (1,0): time passes, core 0 runs nothing.
    let (insns, t0) = (m.cores[0].insn_count(), m.bus.cycles);
    assert!(matches!(run_ms(&mut m, 20), Stop::Halted));
    assert_eq!(m.cores[0].insn_count(), insns, "no instruction while held in reset");
    assert!(m.bus.cycles >= t0 + 20 * MS);
    assert!(m.usj_held());

    // "Set RTS" (still 1,0) and "Clear RTS": out of reset, download flag cleared.
    lines(&s, &[(0, 1), (0, 0)]);
    assert!(matches!(run_ms(&mut m, 2), Stop::Halted));
    assert!(!m.usj_held());
    assert!(m.cores[0].insn_count() > insns, "the core runs again");
    assert_eq!(events(&rx), [r#"{"t":"release","strap":3}"#]);
    assert_eq!(m.bus.periph.gpio.strap, 0x03, "the release does not touch the latch");

    // Table 33.4-4: Clear DTR, Clear RTS, Set RTS (reset, flag clear), Clear RTS.
    lines(&s, &[(0, 0), (0, 0), (0, 1), (0, 0)]);
    let (cause, strap) = expect_reset(&mut m);
    assert_eq!(cause, 0x15);
    assert_eq!(strap, 0x0f, "no flag: the latched pins, SPI boot (IS_1XXX)");
    assert!(matches!(run_ms(&mut m, 2), Stop::Halted));
    assert_eq!(events(&rx), [r#"{"t":"reset","cause":21,"download":false}"#, r#"{"t":"release","strap":15}"#]);
}

#[test]
fn the_host_going_away_or_the_page_reset_releases_a_held_chip() {
    let mut m = machine("none");
    let (_port, s, _rx) = open(&mut m);
    lines(&s, &[(0, 1)]);
    expect_reset(&mut m);
    assert!(matches!(run_ms(&mut m, 5), Stop::Halted));
    assert!(m.usj_held());
    drop(s);                                                 // the client disconnects in (1,0)
    assert!(matches!(run_ms(&mut m, 2), Stop::Halted));
    assert!(!m.usj_held(), "closing the port drops the lines");
    assert_eq!(m.usj_session(), None);

    let (_port, s, _rx) = open(&mut m);
    lines(&s, &[(0, 1)]);
    expect_reset(&mut m);
    assert!(matches!(run_ms(&mut m, 2), Stop::Halted));
    assert!(m.usj_held());
    m.bus.request_reset(esp_periph::RST_POWERON);          // the page's RESET
    assert!(matches!(run_ms(&mut m, 2), Stop::SwReset));
    assert_eq!(m.reboot(), esp_periph::RST_POWERON);
    park(&mut m);
    assert!(!m.usj_held(), "a power-on reset resets the USB device and its lines");
    assert!(matches!(run_ms(&mut m, 2), Stop::Halted));
}

#[test]
fn boot_strap_and_usb_reset_agree() {
    let mut m = machine("hub75-panel");
    let (_port, s, _rx) = open(&mut m);
    let hold = |m: &mut esp32s3::Machine, verb: &str| { m.bus.board_input("boot", verb).unwrap(); assert!(matches!(run_ms(m, 10), Stop::Halted)); };
    let reset = |m: &mut esp32s3::Machine, cause: u32| -> u32 {
        m.bus.request_reset(cause);
        assert!(matches!(run_ms(m, 1), Stop::SwReset));
        assert_eq!(m.reboot(), cause);
        park(m);
        m.bus.periph.gpio.strap
    };
    SocBus::set_strap(&mut m.bus, 0x2f);
    hold(&mut m, "down");
    assert_eq!(reset(&mut m, esp_periph::RST_POWERON), 0x23, "BOOT held through power-on: joint download");
    // A USB reset without the flag keeps a strapped download mode (esptool docs, "Leaving
    // Download Mode in USB-Serial/JTAG Mode": a USB reset does not re-read the pins).
    lines(&s, &[(0, 0), (0, 1), (0, 0)]);
    assert_eq!(expect_reset(&mut m).1, 0x23);
    hold(&mut m, "up");
    assert_eq!(reset(&mut m, esp_periph::RST_POWERON), 0x2b, "released: SPI boot; IO46 rests low");
    // The download flag clears [3:2] for one boot ...
    lines(&s, &[(1, 0), (0, 1), (0, 0)]);
    assert_eq!(expect_reset(&mut m).1, 0x23);
    // ... a software reset keeps the register ...
    assert_eq!(reset(&mut m, esp_periph::RST_SW_SYS), 0x23);
    // ... and a USB reset without the flag goes back to the latched pins.
    lines(&s, &[(0, 1), (0, 0)]);
    assert_eq!(expect_reset(&mut m).1, 0x2b);
}

#[test]
fn a_run_that_cannot_reboot_only_logs_line_resets() {
    let mut m = machine("none");
    m.usj_reset = false;
    let (_port, s, rx) = open(&mut m);
    lines(&s, &[(1, 0), (0, 1)]);
    assert!(matches!(run_ms(&mut m, 5), Stop::Halted));
    assert!(!m.usj_held());
    assert_eq!(events(&rx), [r#"{"t":"hello","proto":1,"chip":"esp32s3","dtr":0,"rts":0,"held":false,"reset":false}"#]);
}

#[test]
fn port_data_reaches_the_controller_and_its_output_the_client_only() {
    let mut m = machine("none");
    m.bus.periph.usb.tx_out = b"before".to_vec();            // console output made before the open
    let (_port, s, rx) = open(&mut m);
    let all: Vec<u8> = (0..=255u8).collect();
    s.data(all.clone());
    assert!(matches!(run_ms(&mut m, 1), Stop::Halted));
    let rx_fifo: Vec<u8> = std::iter::from_fn(|| (!m.bus.periph.usb.rx.is_empty()).then(|| m.bus.periph.usb.read(0) as u8)).collect();
    assert_eq!(rx_fifo, all, "all 256 byte values, in 64-byte packets, in order");
    assert_eq!(m.console.usb, b"before", "the earlier output stayed on the console");
    m.bus.periph.usb.tx_out = vec![0xc0, 0xff, 0x80, 0x00];
    assert!(matches!(run_ms(&mut m, 2), Stop::Halted));
    let out: Vec<u8> = rx.try_iter().flat_map(|o| match o { PortOut::Data(d) => d, PortOut::Event(_) => Vec::new() }).collect();
    assert_eq!(out, [0xc0, 0xff, 0x80, 0x00], "raw bytes to the client");
    assert_eq!(m.console.usb, b"before", "and not to the page's console backlog");
}

#[test]
fn script_lines_do_the_same() {
    let mut m = machine("none");
    m.load_script("0.001 usj 0 0\n0.002 usj 1 0\n0.003 usj 0 1\n0.010 usj 0 0\n0.011 usjhex c0 00 ff c0\n").unwrap();
    assert!(matches!(m.script.events[1].1, esp_soc::ScriptAction::UsjLines(true, false)));
    assert!(matches!(&m.script.events[4].1, esp_soc::ScriptAction::UsjData(d) if d == &[0xc0, 0x00, 0xff, 0xc0]));
    assert!(m.load_script("0.1 usj 2 0").is_err());
    assert!(m.load_script("0.1 usjhex c").is_err());
    let (cause, strap) = expect_reset(&mut m);
    assert_eq!((cause, strap), (0x15, 0x03));
    let insns = m.cores[0].insn_count();
    m.max_cycles = 8 * MS;                                    // until 8 ms: still held
    assert!(matches!(m.run(u64::MAX), Stop::Halted));
    assert_eq!(m.cores[0].insn_count(), insns);
    assert!(matches!(run_ms(&mut m, 5), Stop::Halted));
    assert!(!m.usj_held());
    assert_eq!(m.bus.periph.usb.rx.len(), 4);
}

#[test]
fn the_reserved_data_window_reads_zero() {
    let mut m = machine("none");
    // esptool-js 0.6.0 targets/esp32.ts:79-99: ESP32 eFuse words 3 and 5, APB_CTL_DATE.
    for a in [0x3FF5_A00Cu32, 0x3FF5_A014, 0x3FF6_607C, 0x3FF2_0000, 0x3FFF_FFFC] {
        assert_eq!(m.bus.read32(a), Ok(0), "{a:#x}");
    }
    assert_eq!(m.bus.read8(0x3FF5_A00F), Ok(0));
    assert_eq!(m.bus.read16(0x3FF5_A00E), Ok(0));
    assert!(m.bus.write32(0x3FF5_A00C, 1).is_err(), "writes still fault");
    assert!(m.bus.read32(0x3FF1_FFFC).is_ok(), "internal ROM 1 is still ROM");
    assert!(m.bus.read32(0x3FE0_0000).is_err(), "outside the window reads still fault");
}
