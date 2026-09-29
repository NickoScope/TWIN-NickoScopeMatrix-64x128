//! The MWDTs through the whole machine: code in IRAM arms a timer group's watchdog the way
//! wdt_hal does (unlock with the write key, prescaler, stage 0 hold, WDTCONFIG0) and then spins.
//! Unfed, the watchdog resets the chip with the group's cause when stage 0 expires; fed, it
//! never does. The programs were assembled with xtensa-esp32s3-elf-as (esp-13.2.0) and linked at
//! 0x40370000; the bytes and their objdump are below.
use esp_soc::{SocBus, Stop};

const IRAM: u32 = 0x4037_0000;
const ENTRY: u32 = IRAM + 0x14;
const MS: u64 = 240_000;

/// lit_base: .word 0x6001F000 (TIMG0; patched for TIMG1)  lit_key: .word 0x50D83AA1
/// lit_pre: .word 80 << 16 (one MWDT cycle per us)  lit_hold: .word 1000  lit_cfg: .word 0xE0000000
/// (WDT_EN | STG0 = 3, reset system), then at ENTRY:
/// ```text
/// 40370014: fffb21  l32r a2, lit_base      4037002c: 126232  s32i a3, a2, 72    (WDTCONFIG0)
/// 40370017: fffb31  l32r a3, lit_key       4037002f: 00a032  movi a3, 0
/// 4037001a: 196232  s32i a3, a2, 100 (key) 40370032: 196232  s32i a3, a2, 100   (lock)
/// 4037001d: fffa31  l32r a3, lit_pre       40370035: 186232  s32i a3, a2, 96    (a locked feed)
/// 40370020: 136232  s32i a3, a2, 76        40370038: ffff06  j 40370038         (spin)
/// 40370023: fffa31  l32r a3, lit_hold
/// 40370026: 146232  s32i a3, a2, 80
/// 40370029: fff931  l32r a3, lit_cfg
/// ```
const NO_FEED: [u8; 59] = [
    0x00, 0xf0, 0x01, 0x60, 0xa1, 0x3a, 0xd8, 0x50, 0x00, 0x00, 0x50, 0x00,
    0xe8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x21, 0xfb, 0xff, 0x31,
    0xfb, 0xff, 0x32, 0x62, 0x19, 0x31, 0xfa, 0xff, 0x32, 0x62, 0x13, 0x31,
    0xfa, 0xff, 0x32, 0x62, 0x14, 0x31, 0xf9, 0xff, 0x32, 0x62, 0x12, 0x32,
    0xa0, 0x00, 0x32, 0x62, 0x19, 0x32, 0x62, 0x18, 0x06, 0xff, 0xff,
];
/// The same literals and set-up up to WDTCONFIG0, then a loop that feeds after 2000 turns of a
/// delay loop:
/// ```text
/// 4037002f: d0a742  movi a4, 0x7d0         4037003b: 196232  s32i a3, a2, 100   (unlock)
/// 40370032: ffc442  addi a4, a4, -1        4037003e: 186232  s32i a3, a2, 96    (feed)
/// 40370035: ff9456  bnez a4, 40370032      40370041: 00a032  movi a3, 0
/// 40370038: fff331  l32r a3, lit_key       40370044: 196232  s32i a3, a2, 100   (lock)
///                                          40370047: fff906  j 4037002f
/// ```
const FEED: [u8; 74] = [
    0x00, 0xf0, 0x01, 0x60, 0xa1, 0x3a, 0xd8, 0x50, 0x00, 0x00, 0x50, 0x00,
    0xe8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x21, 0xfb, 0xff, 0x31,
    0xfb, 0xff, 0x32, 0x62, 0x19, 0x31, 0xfa, 0xff, 0x32, 0x62, 0x13, 0x31,
    0xfa, 0xff, 0x32, 0x62, 0x14, 0x31, 0xf9, 0xff, 0x32, 0x62, 0x12, 0x42,
    0xa7, 0xd0, 0x42, 0xc4, 0xff, 0x56, 0x94, 0xff, 0x31, 0xf3, 0xff, 0x32,
    0x62, 0x19, 0x32, 0x62, 0x18, 0x32, 0xa0, 0x00, 0x32, 0x62, 0x19, 0x06,
    0xf9, 0xff,
];

fn machine(prog: &[u8], timg_base: u32) -> esp32s3::Machine {
    let mut m = esp32s3::machine([1, 2, 3, 4, 5, 6]);
    m.console.capture = true;
    let mut prog = prog.to_vec();
    prog[..4].copy_from_slice(&timg_base.to_le_bytes());
    SocBus::load_bytes(&mut m.bus, IRAM, &prog).unwrap();
    m.cores[0].pc = ENTRY;
    m.cores[0].ps = 0;
    m
}

fn run_ms(m: &mut esp32s3::Machine, ms: u64) -> Stop {
    m.max_cycles = m.bus.cycles + ms * MS;
    m.run(u64::MAX)
}

#[test]
fn an_unfed_watchdog_resets_the_chip_with_its_groups_cause() {
    for (base, cause) in [(0x6001_f000, esp_periph::RST_TG0WDT_SYS), (0x6002_0000, esp_periph::RST_TG1WDT_SYS)] {
        let mut m = machine(&NO_FEED, base);
        let stop = run_ms(&mut m, 5);
        assert!(matches!(stop, Stop::SwReset), "{base:#x}: no reset: {stop:?}");
        assert_eq!(m.bus.reset_cause(), cause);
        // armed a few instructions in; stage 0 is 1000 us of 80 APB ticks = 240,000 CPU cycles
        let at = m.bus.cycles;
        assert!((MS..MS + MS / 100).contains(&at), "{base:#x}: reset at cycle {at}, want 1 ms after arming");
        // the chip comes back up with the cause the ROM reports
        assert_eq!(m.reboot(), cause);
        assert_eq!(m.bus.periph.rtc.read(0x38) & 0x3f, cause, "RTC_CNTL_RESET_STATE: PRO CPU reset cause");
    }
}

#[test]
fn a_fed_watchdog_never_resets() {
    for base in [0x6001_f000, 0x6002_0000] {
        let mut m = machine(&FEED, base);
        let stop = run_ms(&mut m, 20);
        assert!(!matches!(stop, Stop::SwReset), "{base:#x}: reset while fed at t={}", m.seconds());
        assert!(m.bus.cycles >= 20 * MS, "{base:#x}: ran {} cycles", m.bus.cycles);
        let wdt = m.bus.periph.timg[if base == 0x6001_f000 { 0 } else { 1 }].wdt.as_ref().unwrap();
        assert!(wdt.count < 1000 && wdt.stage == 0, "{base:#x}: count {} stage {}", wdt.count, wdt.stage);
    }
}

/// `waiti 0 ; j .`: the core sleeps, and time jumps from deadline to deadline (as in
/// panel_inputs.rs), so the flash boot expiry must bound the jump.
const WAITI_LOOP: [u8; 6] = [0x00, 0x70, 0x00, 0x06, 0xff, 0xff];

/// Flash boot protection needs a boot (TRM v1.8 §13.2.2.4, "During flash booting process"): a
/// start from the ROM's reset vector is one, and so is every chip reset; code the emulator enters
/// itself is not, whatever the strap (the default latch, 0x0f, is SPI boot).
#[test]
fn only_a_boot_through_the_rom_arms_flash_boot_protection() {
    let entered = || {
        let mut m = esp32s3::machine([1, 2, 3, 4, 5, 6]);
        SocBus::load_bytes(&mut m.bus, IRAM, &WAITI_LOOP).unwrap();
        m.cores[0].pc = IRAM;
        m.cores[0].ps = 0;
        m
    };
    let mut m = entered();
    assert!(!m.bus.periph.rom_boot);
    let stop = run_ms(&mut m, 500);
    assert!(!matches!(stop, Stop::SwReset), "entered code was reset: {stop:?}");

    let mut m = entered();
    SocBus::rom_boot(&mut m.bus);
    let stop = run_ms(&mut m, 500);
    assert!(matches!(stop, Stop::SwReset), "no flash boot reset: {stop:?}");
    assert_eq!(m.bus.reset_cause(), esp_periph::RST_TG0WDT_SYS);
    // STG0_HOLD's reset value, 26,000,000 cycles of the undivided APB (CPU / 3)
    let at = m.bus.cycles;
    assert!((78_000_000..78_000_000 + 64).contains(&at), "reset at cycle {at}");
    assert_eq!(m.reboot(), esp_periph::RST_TG0WDT_SYS);
    assert!(m.bus.periph.rom_boot, "a chip reset boots through the ROM again");

    let mut m = esp32s3::machine([1, 2, 3, 4, 5, 6]);
    m.boot_rom();
    assert!(m.bus.periph.rom_boot);
}
