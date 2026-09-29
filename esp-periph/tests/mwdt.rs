//! The ESP32-S3 main system watchdogs (MWDT) of the two timer groups: register reset values, the
//! four stages and their actions, feeding, write protection, flash boot protection and the
//! deadline the SoC defers device time by (ESP32-S3 TRM v1.8 §13.2.2; IDF 4.4.7
//! soc/esp32s3/include/soc/timer_group_reg.h:325-483).
use esp_periph::timg::{STG_INT, STG_OFF, STG_RESET_CPU, STG_RESET_SYSTEM, WDT_EN, WDT_FLASHBOOT_MOD_EN, WDT_INT, WDT_PROCPU_RESET_EN, WDT_WKEY};
use esp_periph::{Device, TimerGroup, RST_TG0WDT_CPU, RST_TG0WDT_SYS, RST_TG1WDT_CPU, RST_TG1WDT_SYS};

const CONFIG0: u32 = 0x48;
const CONFIG1: u32 = 0x4c;
const HOLD0: u32 = 0x50;
const FEED: u32 = 0x60;
const WPROTECT: u32 = 0x64;
const INT_ENA: u32 = 0x70;
const INT_RAW: u32 = 0x74;
const INT_CLR: u32 = 0x7c;

fn stages(actions: [u32; 4]) -> u32 { actions.iter().enumerate().fold(0, |v, (i, &a)| v | a << (29 - 2 * i)) }

/// Group `n` with its MWDT set to `prescale`, the four `holds` and `actions`, enabled.
fn armed(n: usize, prescale: u32, holds: [u32; 4], actions: [u32; 4]) -> TimerGroup {
    let mut g = TimerGroup::new_s3(n);
    g.write(CONFIG1, prescale << 16);
    for (i, h) in holds.iter().enumerate() { g.write(HOLD0 + 4 * i as u32, *h); }
    g.write(CONFIG0, WDT_EN | stages(actions));
    g
}

#[test]
fn registers_start_at_their_documented_reset_values() {
    let mut g = TimerGroup::new_s3(0);
    // FLASHBOOT_MOD_EN 1, SYS/CPU_RESET_LENGTH 1 (:343, 350, 358); prescale 1 (:406); holds (:419-455)
    assert_eq!(g.read(CONFIG0), (1 << 14) | (1 << 15) | (1 << 18));
    assert_eq!(g.read(CONFIG1), 1 << 16);
    assert_eq!([g.read(0x50), g.read(0x54), g.read(0x58), g.read(0x5c)], [26_000_000, 134_217_727, 1_048_575, 1_048_575]);
    assert_eq!(g.read(WPROTECT), 0x50D8_3AA1, "the write key's reset value: unlocked");
    assert_eq!(g.read(FEED), 0, "WDTFEED is write-only");
    g.write(CONFIG0, 0xffff_ffff);
    assert_eq!(g.read(CONFIG0), 0xff9f_f000, "reserved bits 0-11 and 21-22 read 0");
}

#[test]
fn stages_expire_in_order_with_their_actions() {
    for (n, sys) in [(0, RST_TG0WDT_SYS), (1, RST_TG1WDT_SYS)] {
        // prescale 2: one MWDT cycle per 2 APB ticks; stage 0 interrupts after 10, stage 1 resets 20 later
        let mut g = armed(n, 2, [10, 20, 5, 5], [STG_INT, STG_RESET_SYSTEM, STG_OFF, STG_OFF]);
        g.tick(19);
        assert_eq!(g.read(INT_RAW) & WDT_INT, 0, "19 APB ticks: 9 cycles, stage 0 still running");
        g.tick(1);
        assert_eq!(g.read(INT_RAW) & WDT_INT, WDT_INT, "stage 0 expired: TIMG_WDT_INT_RAW");
        assert_eq!(Device::irq_sources(&g), 0, "masked until TIMG_WDT_INT_ENA");
        g.write(INT_ENA, WDT_INT);
        assert_eq!(Device::irq_sources(&g), 1 << 2, "the group's third source");
        g.write(INT_CLR, WDT_INT);
        assert_eq!(Device::irq_sources(&g), 0);
        g.tick(39);
        assert_eq!(g.take_reset(), None, "stage 1 needs 20 cycles");
        g.tick(1);
        assert_eq!(g.take_reset(), Some(sys), "group {n}: stage 1 resets the system");
        assert_eq!(g.take_reset(), None, "a reset is handed over once");
    }
}

#[test]
fn feeding_returns_to_stage_zero_with_a_cleared_counter() {
    let mut g = armed(0, 1, [10, 20, 5, 5], [STG_INT, STG_RESET_SYSTEM, STG_OFF, STG_OFF]);
    g.tick(10);
    assert_eq!(g.read(INT_RAW) & WDT_INT, WDT_INT);
    g.write(INT_CLR, WDT_INT);
    g.tick(19);                                 // one cycle short of the reset
    g.write(FEED, 0);                           // "Write any value to feed the MWDT"
    g.tick(9);
    assert_eq!(g.read(INT_RAW) & WDT_INT, 0, "back in stage 0, counting from zero");
    g.tick(1);
    assert_eq!(g.read(INT_RAW) & WDT_INT, WDT_INT, "stage 0 expires again after its full timeout");
    g.tick(19);
    assert_eq!(g.take_reset(), None);
    g.tick(1);
    assert_eq!(g.take_reset(), Some(RST_TG0WDT_SYS));
}

#[test]
fn a_locked_watchdog_ignores_configuration_and_feeds() {
    let mut g = armed(1, 1, [10, 5, 5, 5], [STG_RESET_SYSTEM, STG_OFF, STG_OFF, STG_OFF]);
    g.write(WPROTECT, 0);                       // any other value than the key locks
    assert_eq!(g.read(WPROTECT), 0);
    g.write(CONFIG0, 0);                        // disable: ignored
    g.write(CONFIG1, 7 << 16);
    g.write(HOLD0, 1000);
    g.tick(9);
    g.write(FEED, 1);                           // feed: ignored
    assert_eq!((g.read(CONFIG0) & WDT_EN, g.read(CONFIG1), g.read(HOLD0)), (WDT_EN, 1 << 16, 10));
    g.tick(1);
    assert_eq!(g.take_reset(), Some(RST_TG1WDT_SYS), "the locked writes changed nothing");

    let mut g = armed(1, 1, [10, 5, 5, 5], [STG_RESET_SYSTEM, STG_OFF, STG_OFF, STG_OFF]);
    g.write(WPROTECT, 0);
    g.tick(9);
    g.write(WPROTECT, WDT_WKEY);
    g.write(FEED, 1);                           // unlocked again: the feed counts
    g.tick(9);
    assert_eq!(g.take_reset(), None);
    g.write(WPROTECT, WDT_WKEY);
    g.write(CONFIG0, 0);                        // and so does disabling
    g.tick(1000);
    assert_eq!(g.take_reset(), None);
}

#[test]
fn a_stage_set_off_still_lasts_its_timeout() {
    let mut g = armed(0, 1, [5, 3, 100, 100], [STG_OFF, STG_INT, STG_OFF, STG_OFF]);
    g.tick(7);
    assert_eq!(g.read(INT_RAW) & WDT_INT, 0, "stage 0 (off) took 5 cycles, stage 1 is 2 in");
    g.tick(1);
    assert_eq!(g.read(INT_RAW) & WDT_INT, WDT_INT);
}

#[test]
fn a_cpu_reset_stage_resets_only_with_procpu_reset_en() {
    let mut g = armed(0, 1, [4, 4, 4, 4], [STG_RESET_CPU; 4]);
    g.tick(1_000);
    assert_eq!(g.take_reset(), None, "TIMG_WDT_PROCPU_RESET_EN clear");
    for (n, cpu) in [(0, RST_TG0WDT_CPU), (1, RST_TG1WDT_CPU)] {
        let mut g = armed(n, 1, [4, 4, 4, 4], [STG_OFF, STG_RESET_CPU, STG_OFF, STG_OFF]);
        let c0 = g.read(CONFIG0);
        g.write(CONFIG0, c0 | WDT_PROCPU_RESET_EN);
        g.tick(7);
        assert_eq!(g.take_reset(), None);
        g.tick(1);
        assert_eq!(g.take_reset(), Some(cpu));
    }
}

/// TRM v1.8 §13.2.2.4: in flash boot the MWDT of TIMG0 runs with TIMG_WDT_EN clear, its stage 0
/// resetting the core, until software clears TIMG_WDT_FLASHBOOT_MOD_EN.
#[test]
fn flash_boot_protection_runs_timg0_only() {
    let hold0 = 26_000_000;                     // STG0_HOLD reset value, prescale 1
    let mut g = TimerGroup::new_s3(0);
    g.set_flash_boot(true);
    g.tick(hold0 - 1);
    assert_eq!(g.take_reset(), None);
    assert_eq!(Device::next_deadline(&g), Some(1), "the flash boot expiry is a deadline");
    g.tick(1);
    assert_eq!(g.take_reset(), Some(RST_TG0WDT_SYS), "stage 0 forced to a core reset");

    let mut g = TimerGroup::new_s3(1);          // the bit's reset value is 1 in TIMG1 too
    g.set_flash_boot(true);
    assert_ne!(g.read(CONFIG0) & WDT_FLASHBOOT_MOD_EN, 0);
    g.tick(10 * hold0);
    assert_eq!(g.take_reset(), None, "TIMG1's MWDT is not the flash boot one");

    let mut g = TimerGroup::new_s3(0);
    g.set_flash_boot(false);                    // a boot in download mode
    g.tick(10 * hold0);
    assert_eq!(g.take_reset(), None);

    let mut g = TimerGroup::new_s3(0);
    g.set_flash_boot(true);
    g.tick(hold0 / 2);
    let c0 = g.read(CONFIG0);
    g.write(CONFIG0, c0 & !WDT_FLASHBOOT_MOD_EN);                // bootloader_config_wdt()
    g.tick(10 * hold0);
    assert_eq!(g.take_reset(), None);

    let mut g = TimerGroup::new_s3(0);
    g.set_flash_boot(true);
    g.preset_after_bootloader();
    g.tick(10 * hold0);
    assert_eq!(g.take_reset(), None, "an app booted without the ROM finds it cleared");
}

/// The deadline the SoC bounds device time by lands exactly on the expiry, whatever the
/// prescaler phase, stage or divider.
#[test]
fn the_deadline_is_the_expiry() {
    for prescale in [1u32, 3, 40_000] {
        for already in [0u64, 1, 2, 7] {
            let mut g = armed(1, prescale, [5, 9, 2, 4], [STG_INT, STG_INT, STG_INT, STG_RESET_SYSTEM]);
            g.write(INT_ENA, WDT_INT);
            g.tick(already);
            g.write(INT_CLR, WDT_INT);
            let mut expiries = 0;
            loop {
                let label = format!("prescale {prescale}, {already} ticks in, expiry {expiries}");
                let d = Device::next_deadline(&g).expect("running");
                g.tick(d - 1);
                assert_eq!((g.read(INT_RAW) & WDT_INT, g.wdt.as_ref().unwrap().reset), (0, None), "{label}: early");
                g.tick(1);
                expiries += 1;
                if let Some(cause) = g.take_reset() { assert_eq!(cause, RST_TG1WDT_SYS); break; }
                assert_eq!(g.read(INT_RAW) & WDT_INT, WDT_INT, "{label}: late");
                g.write(INT_CLR, WDT_INT);
            }
            assert!(expiries >= 1 && expiries <= 4);
        }
    }
    let g = TimerGroup::new_s3(1);
    assert_eq!(Device::next_deadline(&g), None, "a disabled watchdog has no deadline");
}

/// One long tick leaves the watchdog as the same APB ticks one at a time do, across stage
/// boundaries, laps and a reset.
#[test]
fn a_long_tick_equals_single_ticks() {
    let configs: [([u32; 4], [u32; 4]); 4] = [
        ([3, 5, 2, 4], [STG_INT, STG_OFF, STG_INT, STG_OFF]),            // laps, never a reset
        ([3, 5, 2, 4], [STG_INT, STG_INT, STG_RESET_SYSTEM, STG_OFF]),
        ([0, 1, 0, 2], [STG_OFF, STG_INT, STG_OFF, STG_INT]),            // zero holds
        ([7, 7, 7, 7], [STG_OFF, STG_OFF, STG_OFF, STG_RESET_SYSTEM]),
    ];
    for (holds, actions) in configs {
        for prescale in [1u32, 2, 3] {
            for n in [1u64, 4, 17, 50, 333] {
                let mut a = armed(0, prescale, holds, actions);
                let mut b = armed(0, prescale, holds, actions);
                a.tick(n);
                for _ in 0..n { b.tick(1); }
                let (wa, wb) = (a.wdt.as_ref().unwrap(), b.wdt.as_ref().unwrap());
                let label = format!("holds {holds:?} actions {actions:?} prescale {prescale} n {n}");
                // once a stage resets, the chip goes: the prescaler phase after it means nothing
                let phase = |w: &esp_periph::Mwdt| if w.reset.is_some() { 0 } else { w.prescale_acc };
                assert_eq!((wa.count, wa.stage, phase(wa), a.int_raw, wa.reset), (wb.count, wb.stage, phase(wb), b.int_raw, wb.reset), "{label}");
            }
        }
    }
}

/// C3 and C6 keep `TimerGroup::new()`: their watchdog registers stay plain storage.
#[test]
fn a_group_without_a_watchdog_model_is_unchanged() {
    let mut g = TimerGroup::new();
    g.write(CONFIG0, WDT_EN | stages([STG_RESET_SYSTEM; 4]));
    g.write(HOLD0, 1);
    g.write(INT_ENA, 7);
    g.tick(1_000_000);
    assert_eq!(g.take_reset(), None);
    assert_eq!(g.read(CONFIG0), WDT_EN | stages([STG_RESET_SYSTEM; 4]));
    assert_eq!(g.read(WPROTECT), 0);
    assert_eq!(Device::next_deadline(&g), None);
}
