use esp32s3::periph::{Peripherals, SRC_TG0_T1, SRC_TG1_T1};

#[test]
fn both_timer_one_alarms_reach_the_s3_interrupt_matrix() {
    let mut peripherals = Peripherals::new([0; 6]);
    for (base, source) in [(0x6001_f000, SRC_TG0_T1), (0x6002_0000, SRC_TG1_T1)] {
        peripherals.write32(base + 0x24, (1 << 31) | (1 << 30) | (2 << 13) | (1 << 10));
        peripherals.write32(base + 0x34, 5);
        peripherals.write32(base + 0x70, 2);
        peripherals.tick(30); // 10 APB ticks at the S3's 240 MHz CPU clock
        let bit = 1 << (source % 32);
        assert_ne!(peripherals.source_status()[source / 32] & bit, 0);
        // CORE0_INTR_STATUS_REG_1 is refreshed from the device source table.
        assert_ne!(peripherals.read32(0x600c_2190) & bit, 0);
        peripherals.write32(base + 0x7c, 2);
        assert_eq!(peripherals.source_status()[source / 32] & bit, 0);
    }
}

/// q256: a busy round is one device tick, so a TIMG alarm inside it must not drop the steps after it.
/// TIMG0 T0 at divider 2 on the 80 MHz APB (CPU/3): 256 cycles are 85 APB ticks, 42 steps. The
/// register read must match at quantum 64 (the alarm lands on a round boundary) and 256.
#[test]
fn timg_steps_after_an_alarm_survive_a_long_round() {
    use emu_core::{Bus, Core};
    use esp_soc::{SocBus, Stop};
    // (increase, autoreload, start, load, alarm, count after 42 steps)
    for (inc, auto, start, load, alarm, want) in [
        (true, true, 0, 0, 32, 10),       // one crossing: 42 - 32
        (true, true, 0, 0, 5, 2),         // eight crossings: 42 % 5
        (false, true, 40, 40, 30, 38),    // counting down, four crossings: 40 - 42 % 10
        (true, false, 0, 0, 32, 42),      // one-shot: the alarm disarms, counting goes on
    ] {
        for q in [64, 256] {
            let mut m = esp32s3::machine([0; 6]);
            m.quantum = q;
            m.vq_max = 1;
            m.bb_max = 1;
            for c in &mut m.cores { c.set_jit(false); }
            SocBus::load_bytes(&mut m.bus, 0x4037_0000, &[0x06, 0xff, 0xff]).unwrap();
            m.cores[0].pc = 0x4037_0000;
            m.cores[0].ps = 0;
            let t = 0x6001_f000;
            m.bus.write32(t + 0x18, start).unwrap();
            m.bus.write32(t + 0x20, 1).unwrap();
            m.bus.write32(t + 0x18, load).unwrap();
            m.bus.write32(t + 0x10, alarm).unwrap();
            m.bus.write32(t, (1 << 31) | u32::from(inc) << 30 | u32::from(auto) << 29 | (2 << 13) | (1 << 10)).unwrap();
            assert!(matches!(m.run(256), Stop::MaxInsns));
            assert_eq!(m.bus.cycles, 256);
            m.bus.write32(t + 0xc, 1).unwrap();
            let label = format!("q={q} inc={inc} auto={auto} load={load} alarm={alarm}");
            assert_eq!(m.bus.read32(t + 4).unwrap(), want, "{label}");
            assert_eq!(m.bus.read32(t + 0x74).unwrap() & 1, 1, "{label}: alarm raised");
        }
    }
}

/// The MWDTs' interrupt: TIMG_WDT_INT (bit 2 of each group's INT registers) is source 52 for
/// TIMG0 and 55 for TIMG1 (ETS_TG0/TG1_WDT_LEVEL_INTR_SOURCE, IDF 4.4.7
/// soc/esp32s3/include/soc/periph_defs.h:108, 111), the one the Interrupt WDT maps to CPU int 24.
#[test]
fn both_watchdog_interrupts_reach_the_s3_interrupt_matrix() {
    use esp32s3::periph::{SRC_TG0_WDT, SRC_TG1_WDT};
    let mut peripherals = Peripherals::new([0; 6]);
    for (base, source) in [(0x6001_f000, SRC_TG0_WDT), (0x6002_0000, SRC_TG1_WDT)] {
        peripherals.write32(base + 0x4c, 1 << 16);                // prescale 1
        peripherals.write32(base + 0x50, 10);                     // stage 0: 10 APB ticks
        peripherals.write32(base + 0x48, (1 << 31) | (1 << 29));  // enabled, stage 0 interrupts
        peripherals.write32(base + 0x70, 1 << 2);
        peripherals.tick(27);                                     // 9 APB ticks
        let bit = 1 << (source % 32);
        assert_eq!(peripherals.source_status()[source / 32] & bit, 0);
        peripherals.tick(3);
        assert_ne!(peripherals.source_status()[source / 32] & bit, 0);
        assert_ne!(peripherals.read32(0x600c_2190) & bit, 0, "CORE0_INTR_STATUS_REG_1");
        peripherals.write32(base + 0x7c, 1 << 2);
        assert_eq!(peripherals.source_status()[source / 32] & bit, 0);
        peripherals.write32(base + 0x48, 0);
    }
}

/// A stage that resets goes through RTC_CNTL's reset request, with the group's cause
/// (TG0WDT_SYS_RESET 7, TG1WDT_SYS_RESET 8: IDF 4.4.7 esp_rom/include/esp32s3/rom/rtc.h:76-77).
#[test]
fn a_watchdog_reset_takes_the_chip_reset_path() {
    for (base, cause) in [(0x6001_f000, esp_periph::RST_TG0WDT_SYS), (0x6002_0000, esp_periph::RST_TG1WDT_SYS)] {
        let mut peripherals = Peripherals::new([0; 6]);
        peripherals.write32(base + 0x50, 100);
        peripherals.write32(base + 0x48, (1 << 31) | (3 << 29));  // stage 0 resets the system
        peripherals.tick(297);
        assert!(!peripherals.rtc.sw_reset);
        peripherals.tick(3);
        assert!(peripherals.rtc.sw_reset);
        assert_eq!(peripherals.rtc.reset_cause, cause);
    }
}

/// Flash boot protection: in a boot through the ROM with the strap latch on SPI boot, TIMG0's
/// MWDT resets the core once STG0_HOLD's reset value, 26,000,000 cycles of the undivided 80 MHz
/// APB, has passed, unless the bootloader clears TIMG_WDT_FLASHBOOT_MOD_EN; in download mode, or
/// for code the emulator enters itself (no boot), it stays still.
#[test]
fn flash_boot_protection_follows_the_strap() {
    const HOLD0_CPU_CYCLES: u64 = 26_000_000 * 3;
    for (rom_boot, strap, resets) in [(true, 0x0f, true), (true, 0x08, true), (true, 0x04, true), (true, 0x03, false), (true, 0x00, false), (false, 0x0f, false)] {
        let mut peripherals = Peripherals::new([0; 6]);
        peripherals.rom_boot = rom_boot;
        peripherals.gpio.strap = strap;
        peripherals.tick(HOLD0_CPU_CYCLES - 3);
        assert!(!peripherals.rtc.sw_reset, "strap {strap:#x}");
        peripherals.tick(3);
        assert_eq!(peripherals.rtc.sw_reset, resets, "rom boot {rom_boot}, strap {strap:#x}");
        if resets { assert_eq!(peripherals.rtc.reset_cause, esp_periph::RST_TG0WDT_SYS); }
    }
    let mut peripherals = Peripherals::new([0; 6]);
    peripherals.rom_boot = true;
    peripherals.tick(HOLD0_CPU_CYCLES / 2);
    let config0 = peripherals.read32(0x6001_f048);
    peripherals.write32(0x6001_f048, config0 & !(1 << 14));
    peripherals.tick(10 * HOLD0_CPU_CYCLES);
    assert!(!peripherals.rtc.sw_reset, "cleared by bootloader_config_wdt()");
}
