//! Timer group: two 54-bit timers on APB with prescaler and alarm, the main system watchdog
//! (MWDT, where the chip model has it) and the RTC calibration register.
use crate::device::{Device, WriteEffect};
use crate::regram::RegRam;
use crate::rtc_cntl::{RST_TG0WDT_CPU, RST_TG0WDT_SYS, RST_TG1WDT_CPU, RST_TG1WDT_SYS};
use crate::{RTC_SLOW_HZ, XTAL_HZ};
use emu_core::ClockDomain;

// ------------------------------------------------------------------ Timer group (T0/T1 + WDT + RTC calibration)
pub struct TimerGroup {
    ram: RegRam,
    pub t: [Timer; 2],
    pub int_raw: u32, pub int_ena: u32,
    /// The group's MWDT. `None` (chips whose watchdog layout is not modelled: C3, C6) leaves
    /// WDTCONFIG0..WDTWPROTECT plain registers, as before.
    pub wdt: Option<Mwdt>,
}
#[derive(Default, Clone, Copy)]
pub struct Timer { pub config: u32, pub count: u64, pub latch: u64, pub alarm: u64, pub load: u64, pub prescale_acc: u64 }
impl TimerGroup {
    pub fn new() -> Self { TimerGroup { ram: RegRam::new(), t: [Timer::default(); 2], int_raw: 0, int_ena: 0, wdt: None } }
    /// ESP32-S3 timer group `group` (0 or 1) with its MWDT: TIMG0's is the Task WDT and TIMG1's
    /// the Interrupt WDT under ESP-IDF 4.4 (esp_system/task_wdt.c:48, int_wdt.c:41).
    pub fn new_s3(group: usize) -> Self { TimerGroup { wdt: Some(Mwdt::new(group)), ..Self::new() } }
    /// ESP32-S3 TRM v1.8 §13.2.2.4: the MWDT of TIMG0 is the one flash boot protection starts.
    /// The chip says whether it booted from flash; TIMG1 ignores it.
    pub fn set_flash_boot(&mut self, flash_boot: bool) { if let Some(w) = &mut self.wdt { w.flash_boot = flash_boot && w.group == 0; } }
    /// The reset a watchdog stage asked for, once: its cause code, for the chip to carry out.
    pub fn take_reset(&mut self) -> Option<u32> { self.wdt.as_mut().and_then(|w| w.reset.take()) }
    /// What the 2nd-stage bootloader leaves behind when the emulator boots the app itself:
    /// bootloader_config_wdt() clears TIMG0's flash boot protection (IDF 4.4.7
    /// bootloader_support/src/bootloader_init.c:80-84).
    pub fn preset_after_bootloader(&mut self) { if let Some(w) = &mut self.wdt { w.config[0] &= !WDT_FLASHBOOT_MOD_EN; } }
    pub fn tick(&mut self, apb_ticks: u64) {
        for i in 0..2 {
            let t = &mut self.t[i];
            if t.config & (1 << 31) == 0 { continue; }   // TIMG_T0_EN
            let div = ((t.config >> 13) & 0xffff) as u64;
            let div = if div == 0 { 65536 } else { div };
            t.prescale_acc += apb_ticks;
            let mut steps = t.prescale_acc / div;
            t.prescale_acc %= div;
            if steps == 0 { continue; }
            let inc = t.config & (1 << 30) != 0;   // TIMG_T0_INCREASE
            // Steps to the alarm from `c`, if counting reaches it (not already at or past it).
            let alarm = t.alarm;
            let gap = |c: u64| if inc { alarm.checked_sub(c) } else { c.checked_sub(alarm) }.filter(|&d| d > 0);
            if t.config & (1 << 10) != 0 {   // TIMG_T0_ALARM_EN
                if let Some(d) = gap(t.count).filter(|&d| d <= steps) {
                    self.int_raw |= 1 << i;
                    steps -= d;
                    if t.config & (1 << 29) != 0 {   // autoreload
                        // q256: a tick can be longer than the reload period, so the steps after the
                        // alarm count from `load` and cross again every `gap(load)` steps.
                        t.count = t.load;
                        if let Some(p) = gap(t.load) { steps %= p; }
                    } else { t.count = t.alarm; t.config &= !(1 << 10); }
                }
            }
            t.count = if inc { (t.count + steps) & ((1 << 54) - 1) } else { t.count.wrapping_sub(steps) & ((1 << 54) - 1) };
        }
        if let Some(w) = &mut self.wdt {
            if w.tick(apb_ticks) { self.int_raw |= WDT_INT; }
        }
    }
    pub fn read(&mut self, off: u32) -> u32 {
        let (i, o) = if off < 0x24 { (0usize, off) } else if off < 0x48 { (1usize, off - 0x24) } else { (2, off) };
        if i < 2 {
            let t = &self.t[i];
            return match o { 0x0 => t.config, 0x4 => t.latch as u32, 0x8 => (t.latch >> 32) as u32, 0x10 => t.alarm as u32, 0x14 => (t.alarm >> 32) as u32, 0x18 => t.load as u32, 0x1c => (t.load >> 32) as u32, _ => 0 };
        }
        if let Some(w) = &self.wdt {
            match off {
                0x48..=0x5c => return w.config[((off - 0x48) >> 2) as usize],
                0x60 => return 0,                  // WDTFEED is write-only (timer_group_reg.h:467 "WT")
                0x64 => return w.wkey,
                _ => {}
            }
        }
        match off {
            0x68 => (self.ram.read(off) & !(1 << 15)) | (1 << 15),                       // RTCCALICFG: always RDY
            0x6c => { let n = (self.ram.read(0x68) >> 16) & 0x7fff; ((n as u64 * XTAL_HZ / RTC_SLOW_HZ) as u32) << 7 }   // RTCCALICFG1 value
            0x70 => self.int_ena, 0x74 => self.int_raw, 0x78 => self.int_raw & self.int_ena,
            0xf8 => 0x2006191,
            _ => self.ram.read(off),
        }
    }
    pub fn write(&mut self, off: u32, v: u32) {
        let (i, o) = if off < 0x24 { (0usize, off) } else if off < 0x48 { (1usize, off - 0x24) } else { (2, off) };
        if i < 2 {
            let t = &mut self.t[i];
            match o {
                0x0 => t.config = v,
                0xc => t.latch = t.count,
                0x10 => t.alarm = (t.alarm & !0xffff_ffff) | v as u64, 0x14 => t.alarm = (t.alarm & 0xffff_ffff) | ((v as u64 & 0x3fffff) << 32),
                0x18 => t.load = (t.load & !0xffff_ffff) | v as u64, 0x1c => t.load = (t.load & 0xffff_ffff) | ((v as u64 & 0x3fffff) << 32),
                0x20 => t.count = t.load,
                _ => {}
            }
            return;
        }
        if let Some(w) = &mut self.wdt {
            if (0x48..=0x64).contains(&off) { w.write(off, v); return; }
        }
        match off {
            0x70 => self.int_ena = v, 0x7c => self.int_raw &= !v,
            _ => self.ram.write(off, v),
        }
    }
}
impl Default for TimerGroup { fn default() -> Self { Self::new() } }

impl Device for TimerGroup {
    fn read(&mut self, off: u32) -> u32 { TimerGroup::read(self, off) }
    fn write(&mut self, off: u32, v: u32) -> WriteEffect { TimerGroup::write(self, off, v); WriteEffect::NONE }
    /// Bits 0, 1: T0, T1; bit 2: the MWDT (TIMG_INT_ST_TIMERS_REG, timer_group_reg.h:599-617).
    fn irq_sources(&self) -> u64 { (self.int_raw & self.int_ena & if self.wdt.is_some() { 7 } else { 3 }) as u64 }
    fn clock(&self) -> Option<ClockDomain> { Some(ClockDomain::Apb) }
    fn tick(&mut self, apb_ticks: u64) { TimerGroup::tick(self, apb_ticks) }
    /// APB ticks until the earliest armed alarm or watchdog stage expiry.
    fn has_deadline(&self) -> bool { true }
    fn next_deadline(&self) -> Option<u64> {
        let mut best: Option<u64> = self.wdt.as_ref().and_then(Mwdt::apb_until_expiry);
        for t in &self.t {
            if t.config & (1 << 31) == 0 || t.config & (1 << 10) == 0 { continue; }
            let div = ((t.config >> 13) & 0xffff) as u64;
            let div = if div == 0 { 65536 } else { div };
            let steps = if t.config & (1 << 30) != 0 {
                if t.count >= t.alarm { continue } else { t.alarm - t.count }
            } else if t.count <= t.alarm { continue } else { t.count - t.alarm };
            let apb = (steps * div).saturating_sub(t.prescale_acc);
            best = Some(best.map_or(apb, |b| b.min(apb)));
        }
        best
    }
    fn debug(&mut self, on: bool) { if let Some(w) = &mut self.wdt { w.log = on; } }
}

// ------------------------------------------------------------------ Main system watchdog (MWDT)
// ESP32-S3 register layout: IDF 4.4.7 soc/esp32s3/include/soc/timer_group_reg.h:325-483 (TRM v1.8
// Registers 12.10-12.17, p.665-667); behaviour: TRM v1.8 §13.2.2 (p.675-677).
/// TIMG_WDT_EN (timer_group_reg.h:394).
pub const WDT_EN: u32 = 1 << 31;
/// TIMG_WDT_FLASHBOOT_MOD_EN, reset value 1 (timer_group_reg.h:343).
pub const WDT_FLASHBOOT_MOD_EN: u32 = 1 << 14;
/// TIMG_WDT_PROCPU_RESET_EN, "WDT reset CPU enable" (timer_group_reg.h:336).
pub const WDT_PROCPU_RESET_EN: u32 = 1 << 13;
/// TIMG_WDT_WKEY: writes to the other watchdog registers are ignored unless WDTWPROTECT holds
/// this value (TRM v1.8 §13.2.2.3; MWDT_LL_WKEY_VALUE, hal/esp32s3/include/hal/mwdt_ll.h:34), and
/// it is also the register's reset value (timer_group_reg.h:479: 1356348065), so the watchdog
/// starts unlocked.
pub const WDT_WKEY: u32 = 0x50D8_3AA1;
/// TIMG_WDT_INT_ENA/RAW/ST/CLR bit (timer_group_reg.h:565, 591, 617, 643).
pub const WDT_INT: u32 = 1 << 2;
/// Stage actions, TIMG_WDT_STGn: "0: off, 1: interrupt, 2: reset CPU, 3: reset system"
/// (timer_group_reg.h:366-390; MWDT_LL_STG_SEL_*, mwdt_ll.h:37-40).
pub const STG_OFF: u32 = 0;
pub const STG_INT: u32 = 1;
pub const STG_RESET_CPU: u32 = 2;
pub const STG_RESET_SYSTEM: u32 = 3;

/// One MWDT: a 32-bit counter clocked by APB_CLK through a 16-bit prescaler, stepping through four
/// stages, each with a timeout (TIMG_WDT_STGn_HOLD) and an action at expiry (TRM v1.8 §13.2.2.1-2).
/// The model ticks from the same APB ticks as the group's T0/T1, so a watchdog that software feeds
/// in time on silicon is fed in time here, as long as the emulated code keeps silicon's pace.
#[derive(Clone, Debug)]
pub struct Mwdt {
    /// WDTCONFIG0..WDTCONFIG5 (0x48..0x5c).
    pub config: [u32; 6],
    /// WDTWPROTECT (0x64).
    pub wkey: u32,
    /// The expiry counter, in MWDT clock cycles, and the stage it counts for.
    pub count: u64, pub stage: usize,
    /// APB ticks toward the next MWDT clock cycle.
    pub prescale_acc: u64,
    /// The chip booted from flash and this is TIMG0: TIMG_WDT_FLASHBOOT_MOD_EN runs the watchdog.
    pub flash_boot: bool,
    /// A stage's reset, waiting for the chip: the reset cause.
    pub reset: Option<u32>,
    group: usize,
    /// `--debug timg`: print configuration changes, expiries and ignored locked writes.
    pub log: bool,
}

impl Mwdt {
    /// Register reset values: timer_group_reg.h:329-458 (TRM v1.8 Registers 12.10-12.15 agree).
    pub fn new(group: usize) -> Self {
        Mwdt {
            // FLASHBOOT_MOD_EN = 1, SYS_RESET_LENGTH = 1, CPU_RESET_LENGTH = 1 (timer_group_reg.h:343, 350, 358)
            config: [WDT_FLASHBOOT_MOD_EN | (1 << 15) | (1 << 18),
                     1 << 16,          // WDT_CLK_PRESCALE = 1 (:406)
                     26_000_000,       // STG0_HOLD (:419)
                     134_217_727,      // STG1_HOLD (:431)
                     1_048_575,        // STG2_HOLD (:443)
                     1_048_575],       // STG3_HOLD (:455)
            wkey: WDT_WKEY, count: 0, stage: 0, prescale_acc: 0, flash_boot: false, reset: None, group, log: false,
        }
    }
    fn flashboot_active(&self) -> bool { self.flash_boot && self.config[0] & WDT_FLASHBOOT_MOD_EN != 0 }
    /// TRM v1.8 §13.2.2.1: the counter runs while TIMG_WDT_EN is set; flash boot protection
    /// (§13.2.2.4) also runs it with TIMG_WDT_EN clear (mwdt_ll.h:81-83, 208-210). A disabled
    /// watchdog keeps its counter and stage; what silicon does with them is not documented (IDF
    /// always feeds before enabling: hal/wdt_hal_iram.c:151-152).
    pub fn running(&self) -> bool { self.reset.is_none() && (self.config[0] & WDT_EN != 0 || self.flashboot_active()) }
    /// APB ticks per MWDT clock cycle: "MWDT clock period = 12.5 ns * TIMG_WDT_CLK_PRESCALE"
    /// (timer_group_reg.h:406-408; TRM §13.2.2.1: "derived from the APB clock via a pre-MWDT 16-bit
    /// configurable prescaler"). 0 is outside the documented 1..65535 (mwdt_ll.h:221); it is taken
    /// as 65536, the slowest rate, as TIMG_Tx_DIVIDER 0 is above - not verified on silicon.
    fn divider(&self) -> u64 { match (self.config[1] >> 16) as u64 { 0 => 65536, d => d } }
    /// The action at the expiry of `stage`. During flash boot, "Stage 0 for the enabled MWDT is
    /// automatically configured to reset the system upon expiry, known as core reset" (§13.2.2.4).
    pub fn action(&self, stage: usize) -> u32 {
        if stage == 0 && self.flashboot_active() { return STG_RESET_SYSTEM; }
        (self.config[0] >> (29 - 2 * stage as u32)) & 3
    }
    /// MWDT clock cycles a stage lasts. "the 32-bit counters ... increment on each source clock
    /// cycle until the timeout value of the current stage is reached"; a hold of 0 is taken to
    /// expire on the first cycle.
    fn stage_len(&self, stage: usize) -> u64 { (self.config[2 + stage] as u64).max(1) }
    /// MWDT clock cycles until the current stage expires (1 if a lowered hold is already passed).
    fn left(&self) -> u64 { self.stage_len(self.stage).saturating_sub(self.count).max(1) }
    /// APB ticks until the current stage expires, if the watchdog runs.
    pub fn apb_until_expiry(&self) -> Option<u64> {
        if !self.running() { return None; }
        Some((self.left() * self.divider()).saturating_sub(self.prescale_acc))
    }
    /// TRM §13.2.2.1: "If a watchdog timer is fed by software, the timer will return to stage 0 and
    /// reset its counter value to zero." The prescaler's phase is kept (not documented).
    pub fn feed(&mut self) { self.count = 0; self.stage = 0; }
    /// Advance by `apb_ticks`. Returns true when an interrupt stage expired.
    pub fn tick(&mut self, apb_ticks: u64) -> bool {
        if !self.running() { return false; }
        let div = self.divider();
        self.prescale_acc += apb_ticks;
        let mut steps = self.prescale_acc / div;
        self.prescale_acc %= div;
        let mut irq = false;
        while steps > 0 {
            let left = self.left();
            if steps < left { self.count += steps; break; }
            steps -= left;
            // §13.2.2.2: "When one stage expires, the expiry action is triggered, the counter value
            // is reset to zero, and the next stage becomes active ... from stage 0 to 3, then back
            // to stage 0." A stage set to off still lasts its timeout: it "will have no effects on
            // the system".
            let (stage, action) = (self.stage, self.action(self.stage));
            self.count = 0;
            self.stage = (stage + 1) % 4;
            if action == STG_INT { irq = true; }
            if self.resets(stage) {
                self.reset = Some(match (action, self.group) {
                    (STG_RESET_CPU, 0) => RST_TG0WDT_CPU, (STG_RESET_CPU, _) => RST_TG1WDT_CPU,
                    (_, 0) => RST_TG0WDT_SYS, _ => RST_TG1WDT_SYS,
                });
            }
            if self.log { eprintln!("[timg{}] MWDT stage {} expired: {}", self.group, stage, ["no action", "interrupt", "CPU reset", "core reset"][action as usize]); }
            if let Some(cause) = self.reset {
                if self.log { eprintln!("[timg{}] MWDT resets the chip, cause {:#x}", self.group, cause); }
                break;
            }
            // Back at stage 0 with no stage that resets: every further lap repeats this one, so a
            // long tick skips the whole laps it covers.
            if self.stage == 0 && !(0..4).any(|s| self.resets(s)) {
                steps %= (0..4).map(|s| self.stage_len(s)).sum::<u64>();
            }
        }
        irq
    }
    /// Whether the expiry of `stage` resets. A CPU reset needs TIMG_WDT_PROCPU_RESET_EN, "WDT
    /// reset CPU enable" (timer_group_reg.h:336; APPCPU_RESET_EN is "Reserved", :329): read as
    /// that reset's enable, not verified on silicon. IDF 4.4 never gives an MWDT stage a CPU reset.
    fn resets(&self, stage: usize) -> bool {
        match self.action(stage) {
            STG_RESET_SYSTEM => true,
            STG_RESET_CPU => self.config[0] & WDT_PROCPU_RESET_EN != 0,
            _ => false,
        }
    }
    /// A register write at `off` (0x48..=0x64). TRM §13.2.2.3: "Any attempts to write to a
    /// watchdog timer's registers (other than the write-key field itself) whilst the write-key
    /// field's value is not 0x50D83AA1 will be ignored"; feeding is one of those writes (the
    /// procedure's step 2, "such as feeding").
    pub fn write(&mut self, off: u32, v: u32) {
        if off == 0x64 { self.wkey = v; return; }
        if self.wkey != WDT_WKEY {
            if self.log { eprintln!("[timg{}] MWDT locked: write {:#x} to {:#x} ignored", self.group, v, off); }
            return;
        }
        match off {
            // bits 21-22 and 0-11 are reserved (timer_group_reg.h:329-397)
            0x48 => {
                let v = v & 0xff9f_f000;
                if self.log && v != self.config[0] { eprintln!("[timg{}] MWDT config0 {:#010x} -> {:#010x} (stage {}, count {})", self.group, self.config[0], v, self.stage, self.count); }
                self.config[0] = v;
            }
            0x4c => self.config[1] = v & 0xffff_0000,     // WDT_CLK_PRESCALE [31:16]
            0x50..=0x5c => self.config[((off - 0x48) >> 2) as usize] = v,
            0x60 => self.feed(),                          // "Write any value to feed the MWDT" (:467)
            _ => {}
        }
    }
}
