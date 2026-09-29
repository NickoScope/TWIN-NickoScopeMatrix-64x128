//! The ESP32-S3 as a `Soc`: two LX7 cores on `bus::SocBus`, and the questions the machine asks
//! its bus (console, reset, app boot, board, audio, interrupt routing).
use crate::bus::{SocBus, DBUS_HIGH, DBUS_LOW, IBUS_HIGH, IBUS_LOW, MMU_ENTRIES, MMU_INVALID, SRC_FLASH};
use crate::periph::{self, NUM_SOURCES};
use esp_periph::Misc;
use esp_soc::{BoardModel, CoreState, Soc};
use xtensa_lx7::state::ps;
use xtensa_lx7::Cpu;

pub struct S3;
pub type Machine = esp_soc::Machine<S3>;

/// A machine with the default 8 MB flash / 2 MB PSRAM and no board (`bus.board` selects one).
pub fn machine(mac: [u8; 6]) -> Machine { let mut m = Machine::new(mac, SocBus::new(8 << 20, 2 << 20, mac)); m.set_debug(&esp_soc::DebugFlags::from_env()); m }

impl Soc for S3 {
    type Core = Cpu;
    type Bus = SocBus;
    const NAME: &'static str = "esp32s3";
    const ROM_ELF: &'static str = "esp32s3_rev0_rom.elf";
    const CPU_HZ: u64 = periph::CPU_HZ;
    const CORES: usize = 2;
    const IDLE_CHUNK: u64 = 64 * 8;
    const ROM_DATA_TABLE: &'static [&'static str] = &["_data_start"];
    fn new_core(i: usize) -> Cpu { Cpu::new(if i == 0 { 0xCDCD } else { 0xABAB }) }
    fn new_core_with_bus(i: usize, bus: &SocBus) -> Cpu {
        let mut core = Self::new_core(i);
        core.fetch_cache = bus.fetch_cache.clone();
        core
    }
    fn reset_core(c: &mut Cpu, i: usize) { Cpu::reset(c); if i == 1 { c.prid = 0xABAB; } }
    fn boot_core(c: &mut Cpu, entry: u32) {
        Cpu::reset(c);
        c.pc = entry;
        c.ps = ps::WOE | ps::UM;      // windows enabled, user vector; INTLEVEL 0
        c.vecbase = 0x4000_0000;
        c.set_ar(1, 0x3FCE_B000);     // bootloader stack (in DRAM, app treats as free)
        c.set_ar(0, 0);
    }
    fn irqs(bus: &SocBus, out: &mut [u32]) { let (l0, l1) = bus.periph.cpu_lines_both(); out[0] = l0; out[1] = l1; }
    /// Core 0 is always running. SYSTEM_CORE_1_CONTROL_0 controls core 1's clock gate, reset,
    /// and run-stall state.
    fn core_state(bus: &SocBus, core: usize) -> CoreState {
        if core == 0 { return CoreState::Running; }
        let (clk, reset, stall) = bus.periph.core1_control();
        if reset { CoreState::Reset } else if clk && !stall { CoreState::Running } else { CoreState::Held }
    }
}

impl esp_soc::SocBus for SocBus {
    fn cycles(&self) -> u64 { self.cycles }
    fn next_deadline(&self) -> Option<u64> { Some(SocBus::next_deadline(self)) }
    fn irq_dirty(&mut self) -> &mut bool { &mut self.irq_dirty }
    // Only the WASM JIT helpers currently stop before deferred device accesses.
    // `ESP32SIM_VQ_NATIVE` opts a native run in; it is only sound with `--no-jit`.
    fn can_defer(&self) -> bool { cfg!(target_arch = "wasm32") || std::env::var_os("ESP32SIM_VQ_NATIVE").is_some() }
    fn set_defer(&mut self, on: bool) { self.defer_mmio = on; self.mmio_deferred = false; }
    fn take_deferred(&mut self) -> bool { std::mem::take(&mut self.mmio_deferred) }
    fn refresh_irq(&mut self) -> bool {
        // Inputs are sampled at refresh boundaries. Clearing and reasserting a source
        // between samples cannot create a second edge at the CPU.
        let dirty = self.periph.lines_dirty() || self.periph.intmatrix_dirty;
        self.periph.intmatrix_dirty = false;
        dirty
    }
    fn flush_ticks(&mut self) { SocBus::flush_ticks(self) }
    fn touch_input(&mut self, x: u16, y: u16, down: bool) {
        self.board.touch_at(self.cycles, x, y, down);
        if self.board.next_deadline().is_some() { self.refresh_tick_budget(); }
    }
    fn board_input(&mut self, cmd: &str, args: &str) -> Result<(), String> {
        let r = self.board.input_at(self.cycles, cmd, args);
        // A new deadline bounds the device tick, so the first edge lands on its own cycle.
        if self.board.next_deadline().is_some() { self.refresh_tick_budget(); }
        r
    }
    fn misc(&mut self) -> &mut Misc { &mut self.periph.misc }
    fn load_bytes(&mut self, addr: u32, data: &[u8]) -> Result<(), String> { SocBus::load_bytes(self, addr, data) }
    fn write_flash(&mut self, offset: usize, data: &[u8]) -> Result<(), String> {
        let target = self.flash.get_mut(offset..).and_then(|tail| tail.get_mut(..data.len()))
            .ok_or("flash image too large")?;
        target.copy_from_slice(data);
        self.note_written(SRC_FLASH, offset, data.len());
        Ok(())
    }
    fn flash_contents(&self) -> Option<&[u8]> { Some(&self.flash) }
    fn set_flash_file(&mut self, f: std::fs::File) -> Result<(), String> { self.flash_file = Some(f); Ok(()) }
    /// Copy IRAM/DRAM segments, map IROM/DROM through the MMU, as the 2nd-stage bootloader would.
    fn boot_app(&mut self, app_off: usize) -> Result<u32, String> {
        let image = self.flash.get(app_off..).ok_or("app offset beyond flash")?;
        let img = esp_soc::image::parse(image)?;
        self.periph.system.preset_after_bootloader();
        self.periph.rtc.preset_after_bootloader();
        for s in &img.segments {
            let start = app_off + s.file_off as usize;
            let end = start + s.len as usize;
            if end > self.flash.len() { return Err("segment beyond flash".into()); }
            let flash_mapped = (DBUS_LOW..DBUS_HIGH).contains(&s.load_addr) || (IBUS_LOW..IBUS_HIGH).contains(&s.load_addr);
            if flash_mapped {
                // esptool aligns segments so vaddr and flash offset agree modulo 64 KiB
                if (s.load_addr & 0xffff) != (start as u32 & 0xffff) { return Err(format!("segment {:#x} not page-aligned with flash offset {:#x}", s.load_addr, start)); }
                let window_end = if s.load_addr < DBUS_HIGH { DBUS_HIGH } else { IBUS_HIGH };
                if !s.load_addr.checked_add(s.len).is_some_and(|end| end <= window_end) {
                    return Err("segment beyond the flash window".into());
                }
                let first_page = (start as u32) >> 16;
                let npages = ((s.load_addr & 0xffff) + s.len + 0xffff) >> 16;
                for i in 0..npages {
                    let vpage = (((s.load_addr & 0x1FF_FFFF) >> 16) + i) as usize;
                    self.mmu[vpage] = first_page + i;
                }
                self.invalidate_tlb();
            } else {
                let data = self.flash[start..end].to_vec();
                SocBus::load_bytes(self, s.load_addr, &data)?;
            }
        }
        Ok(img.entry)
    }
    /// Digital peripherals re-initialised, cache MMU invalid; SRAM, RTC memories, efuses and the
    /// RTC-domain registers survive, as on silicon. Returns the cause the ROM will report.
    fn reboot(&mut self, mac: [u8; 6]) -> u32 {
        self.flush_ticks();
        self.cancel_spi2_timing();
        let cause = self.periph.rtc.reset_cause;
        let old = std::mem::replace(&mut self.periph, periph::Peripherals::new(mac));
        let p = &mut self.periph;
        p.efuse = old.efuse;
        p.misc.log_unknown = old.misc.log_unknown; p.spi1.log = old.spi1.log;
        p.spi0.jedec = old.spi0.jedec; p.spi1.jedec = old.spi1.jedec;   // the flash chip is not reset: its ID keeps the --flash-mb capacity
        p.spi0.psram_mr[2] = old.spi0.psram_mr[2]; p.spi1.psram_mr[2] = old.spi1.psram_mr[2];   // nor the PSRAM: MR2 is read-only (APS12808L-OBMx Rev 3.0a §7.7) and keeps the --psram-mb density
        p.rtc.ram = old.rtc.ram; p.rtc.slow_ticks = old.rtc.slow_ticks;
        p.rtc.ram.write(0x38, cause | (cause << 6));
        p.rtc.ram.write(0x98, 0);                       // watchdog disarmed by the reset; the ROM re-arms it
        p.i2s0.pcm = old.i2s0.pcm; p.i2s0.frames_out = old.i2s0.frames_out; p.i2s1.pcm = old.i2s1.pcm; p.i2s1.frames_out = old.i2s1.frames_out;   // keep the captured audio continuous
        // The virtual AP and the network behind it are the world outside the chip: a reset leaves
        // them there. The AP forgets the station (it must associate again), the network keeps its
        // forwarded host ports, lease and UDP flows, as a router would for a rebooting client. The
        // station's TCP connections died with its stack: the NAT forgets them, or the rebooted
        // guest, whose fixed-seed RNG draws the same local ports, meets its old flows (review 6).
        p.wifi.ap = old.wifi.ap.map(|a| { let stats = a.stats; let mut n = crate::wifi::VirtualAp::new(a.cfg, a.log); n.stats = stats; n });
        p.wifi.net = old.wifi.net;
        if let Some(nat) = p.wifi.net.as_mut().and_then(|n| n.nat.as_mut()) { nat.station_reset(); }
        // A Chip Reset re-latches the strapping pins from the levels on them (ESP32-S3 TRM §8.1
        // "During Chip Reset ... hardware captures samples"; datasheet §3 "At Chip Reset, the
        // latches sample"). Its code is 0x01 (TRM Table 7.1-1; IDF soc/esp32s3/reset_reasons.h
        // RESET_REASON_CHIP_POWER_ON, _CHIP_BROWN_OUT and _CHIP_SUPER_WDT are all 0x01). 0x0F is
        // the System Reset form of brown-out (RESET_REASON_SYS_BROWN_OUT), 0x10/0x12/0x13 are
        // System Resets in that table, the rest core and CPU resets: none of them re-latch. Only
        // the boot-mode bits the board drives change: GPIO_STRAPPING[3] = GPIO0, [2] = GPIO46 (IDF
        // 5.5.4 soc/esp32s3/include/soc/boot_mode.h: IS_1XXX SPI boot, IS_00XX joint download).
        // The other bits stay `--strap`, and a board without input levels keeps them all.
        //
        // A USB-Serial/JTAG reset (0x15) is a Core Reset (Table 7.1-1), so it samples nothing; the
        // host's download mode flag, taken when RTS=1/DTR=0 began (`esp_periph::UsjLines`), picks
        // the boot instead (Table 33.3-2, p.1248: "if the download mode flag is set when the
        // ESP32-S3 is reset, the ESP32-S3 will reboot into download mode"). The model clears
        // GPIO_STRAPPING[3:2], joint download (boot_mode.h IS_00XX), for that one boot, the effect
        // TRM §8.2 (p.536) documents for RTC_CNTL_FORCE_DOWNLOAD_BOOT; which register the USB
        // controller drives on silicon, and what GPIO_STRAPPING reads then, is not verified. Without
        // the flag the reset boots with the latched pins: after BOOT held through power-on it stays
        // in download mode, as esptool's docs say a USB reset cannot leave a strapped download mode
        // ("Leaving Download Mode in USB-Serial/JTAG Mode", esp32s3). Any other reset keeps the
        // register as it was. A power-on reset also resets the USB device: the lines start over.
        let strap = match cause {
            esp_periph::RST_POWERON => {
                let mut v = self.strap_latched;
                for (pin, level) in self.board.input_levels() {
                    let bit = match pin { 0 => 3, 46 => 2, _ => continue };
                    if level { v |= 1 << bit } else { v &= !(1 << bit) }
                }
                self.strap_latched = v;
                self.usj = Default::default();
                v
            }
            esp_periph::RST_USB_UART_CHIP if self.usj.latched_download => self.strap_latched & !0x0c,
            esp_periph::RST_USB_UART_CHIP => self.strap_latched,
            _ => old.gpio.strap,
        };
        self.periph.gpio.strap = strap;
        self.mmu = [MMU_INVALID; MMU_ENTRIES];
        self.invalidate_tlb();
        self.reset_approximate_cache();
        self.attach_board_devices();
        self.refresh_tick_budget();
        self.irq_dirty = true;
        cause
    }
    fn sw_reset(&self) -> bool { self.periph.rtc.sw_reset }
    fn request_reset(&mut self, cause: u32) { self.periph.rtc.sw_reset = true; self.periph.rtc.reset_cause = cause; }
    fn reset_cause(&self) -> u32 { self.periph.rtc.reset_cause }
    fn last_fault(&self) -> Option<(u32, bool)> { self.last_fault }
    fn console_take(&mut self) -> [Vec<u8>; 4] {
        [std::mem::take(&mut self.periph.usb.tx_out), std::mem::take(&mut self.periph.uart[0].tx_out), std::mem::take(&mut self.periph.uart[1].tx_out), std::mem::take(&mut self.periph.uart[2].tx_out)]
    }
    fn serial_input(&mut self, data: &[u8]) {
        let before = self.periph.usb.irq();
        self.periph.usb.host_input(data);
        self.irq_dirty |= before != self.periph.usb.irq();
    }
    fn uart_input(&mut self, n: usize, data: &[u8]) {
        let Some(u) = self.periph.uart.get_mut(n) else { return };
        let before = u.irq();
        u.host_input(data);
        self.irq_dirty |= before != u.irq();
    }
    fn gpio_set_input(&mut self, pin: u8, level: bool) {
        let old_input = self.periph.gpio.input;
        self.periph.gpio.set_input(pin, level);
        // Host edges queue PCNT work without going through the MMIO refresh hook.
        // Recompute the threshold without dropping cycles already pending.
        self.refresh_tick_budget();
        self.irq_dirty |= old_input != self.periph.gpio.input;
        if let Some(ev) = &mut self.gpio_events { ev.push((self.cycles, pin, level)); }
    }
    fn set_flash_size(&mut self, bytes: usize) {
        self.flash = vec![0xff; bytes];
        let cap = bytes.trailing_zeros() as u8; self.periph.spi1.jedec[2] = cap; self.periph.spi0.jedec[2] = cap;
        self.rebuild_page_table();
    }
    /// The array, and MR2's density with it: IDF sizes the PSRAM from MR2 alone (opiram_psram.c:264).
    fn set_psram_size(&mut self, bytes: usize) -> Result<(), String> {
        self.psram = vec![0; bytes];
        self.periph.spi0.set_psram_size(bytes); self.periph.spi1.set_psram_size(bytes);
        self.rebuild_page_table();
        Ok(())
    }
    /// `--strap`: the pins as the next boot latches them (the run's first boot takes all of it).
    fn set_strap(&mut self, v: u32) { self.periph.gpio.strap = v; self.strap_latched = v; }
    fn strap(&self) -> Option<u32> { Some(self.periph.gpio.strap) }
    fn usj_lines(&mut self, dtr: bool, rts: bool) -> esp_periph::LineEffect { self.usj.set(dtr, rts) }
    fn usj_line_state(&self) -> Option<esp_periph::UsjLines> { Some(self.usj) }
    fn set_reset_cause(&mut self, c: u32) { self.periph.rtc.ram.write(0x38, c | (c << 6)); }
    fn report(&self) -> String {
        let p = &self.periph;
        let mut s = format!("[emu] i2s frames out: {} (i2s0 @ {} Hz) {} (i2s1 @ {} Hz)\n", p.i2s0.frames_out, p.i2s0.sample_rate, p.i2s1.frames_out, p.i2s1.sample_rate);
        { let r = self.board.report(); if !r.is_empty() { s += &r; s += "\n"; } }
        { let w = &p.wifi;
          if w.tx_frames + w.rx_frames > 0 { s += &format!("[emu] wifi: {} frames sent by the station, {} received ({} dropped: no descriptor){}\n", w.tx_frames, w.rx_frames, w.rx_dropped, w.ap.as_ref().map_or(String::new(), |ap| format!("; AP: {} beacons, {} probe responses, {} data frames from the station, {} dropped from a full queue, state {:?}", ap.stats.0, ap.stats.1, ap.stats.2, ap.queue_dropped, ap.state))); }
          if let Some(r) = w.net.as_ref().and_then(|n| n.bridge_report()) { s += &r; s += "\n"; }
          if let Some(n) = w.net.as_ref().filter(|n| n.bridge.is_none()) { s += &format!("[emu] net: {} DHCP leases, {} ARP replies, {} DNS answers, {} NTP answers, {} TCP refused, {} pings, {} frames ignored\n", n.dhcp_acks, n.arp_replies, n.dns_answers, n.ntp_answers, n.tcp_rejects, n.pings, n.unhandled);
            if let Some(t) = &n.nat { s += &format!("[emu] nat: {} TCP connections ({} failed), {} UDP flows ({} evicted, {} send errors), {} bytes out, {} bytes in\n", t.tcp_opened, t.tcp_refused, t.udp_flows, t.udp_evicted, t.udp_send_errors, t.bytes_to_host, t.bytes_to_guest);
                if t.fwd_tcp + t.fwd_udp_in > 0 { s += &format!("[emu] hostfwd: {} TCP connections into the guest ({} refused or unanswered), {} UDP datagrams in, {} answers out\n", t.fwd_tcp, t.fwd_refused, t.fwd_udp_in, t.fwd_udp_out); } } } }
        { let (a, sh, r) = (&p.aes, &p.sha, &p.rsa);
          if a.blocks + sh.blocks + r.ops > 0 { s += &format!("[emu] crypto: {} AES blocks, {} SHA blocks, {} RSA/MPI operations\n", a.blocks, sh.blocks, r.ops); } }
        if p.lcd_cam.lcd_frames > 0 { s += &format!("[emu] lcd: {} RGB frames\n", p.lcd_cam.lcd_frames); }
        if p.lcd_cam.i80_words > 0 { s += &format!("[emu] lcd: {} i8080 bus words at {} Hz PCLK\n", p.lcd_cam.i80_words, p.lcd_cam.lcd_pclk_hz()); }
        if p.lcd_cam.frames + p.lcd_cam.dropped > 0 { s += &format!("[emu] camera: {} frames delivered, {} dropped (no DMA/no picture)\n", p.lcd_cam.frames, p.lcd_cam.dropped); }
        if p.rmt.tx_count > 0 { s += &format!("[emu] rmt tx {}\n", p.rmt.tx_count); }
        s.trim_end().to_string()
    }
    fn set_debug(&mut self, f: &esp_soc::DebugFlags) {
        self.debug = f.clone();
        for area in f.iter() { esp_periph::Dispatch::debug(&mut self.periph, area, true); }
        self.periph.misc.log_all = f.has("mmio");
    }
    fn observe_gpio(&mut self, on: bool) { self.gpio_events = if on { Some(Vec::new()) } else { None }; }
    fn take_gpio_events(&mut self) -> Vec<(u64, u8, bool)> { self.gpio_events.as_mut().map(std::mem::take).unwrap_or_default() }
    fn gpio_input(&self) -> u64 { self.periph.gpio.input }
    fn board(&mut self) -> &mut dyn BoardModel { &mut *self.board }
    fn board_ref(&self) -> &dyn BoardModel { &*self.board }
    fn audio(&self) -> (&[i16], u32) { let a = self.periph.audio(); (&a.pcm, a.sample_rate) }
    fn camera_frames(&self) -> u64 { self.periph.lcd_cam.frames }
    fn irq_sources_of(&self, core: usize, line: u32) -> Vec<usize> { (0..NUM_SOURCES).filter(|&s| self.periph.intmatrix.map[core][s] == line).collect() }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matrix_routes_sampled_edges_and_nmi_to_machine_cores() {
        for irq in [10, 14, 22, 28, 30] {
            let mut m = Machine::new([0; 6], SocBus::new(8 << 20, 2 << 20, [0; 6]));
            m.bus.periph.intmatrix.map[0][periph::SRC_FROM_CPU0] = irq;
            m.bus.periph.intmatrix_dirty = true;
            m.cores[0].ps = if irq == 14 { ps::EXCM | 15 } else { 0 };
            m.cores[0].intenable = if irq == 14 { 0 } else { 1 << irq };
            m.sync_irq();
            m.bus.periph.system.write(0x30, 1);
            m.sync_irq();
            assert_ne!(m.cores[0].check_interrupts_pending() & (1 << irq), 0);
            assert!(m.cores[0].check_interrupts().is_some());
            // Acknowledgment while the source remains high cannot manufacture another edge.
            m.cores[0].interrupt &= !(1 << irq);
            m.sync_irq();
            assert_eq!(m.cores[0].interrupt & (1 << irq), 0);
            m.bus.periph.system.write(0x30, 0);
            m.sync_irq();
            m.bus.periph.system.write(0x30, 1);
            m.sync_irq();
            assert_ne!(m.cores[0].interrupt & (1 << irq), 0);
        }
    }

    #[test]
    fn primary_core_is_not_controlled_by_core_one_reset_registers() {
        let bus = SocBus::new(8 << 20, 2 << 20, [0; 6]);
        assert_eq!(S3::core_state(&bus, 0), CoreState::Running);
        assert_eq!(S3::core_state(&bus, 1), CoreState::Held);
    }
}
