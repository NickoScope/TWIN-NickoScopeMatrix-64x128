//! The NickoScope LED panel: a Waveshare ESP32-S3-RGB-Matrix board driving a 128x64 HUB75 panel
//! (two 64x64, 1/32 scan) from the LCD_CAM i8080 bus through a looping GDMA ring, the way
//! ESP32-HUB75-MatrixPanel-DMA 3.0.14 does it on the S3.
//!
//! The words go to the `hub75` crate's decoder, a generic HUB75 state machine (shift, latch,
//! OE-gated light in rows a and a+32), one GDMA descriptor at a time; the descriptor the driver
//! marks SUC_EOF closes a refresh. Refreshes are summed into windows of at least 1/60 s, an eye's
//! worth, and published as light: an LED's share of the most a 1/32-scan LED can emit.
//!
//! The board's I2C bus (SDA GPIO47, SCL GPIO48) carries the Sensirion SHTC3 at 0x70 (`i2c/shtc3.rs`).
//! The air it senses starts as a plausible room and is set at run time with the `climate` verb.
use std::cell::RefCell;
use std::sync::atomic::{AtomicU64, Ordering::Relaxed};
use std::sync::{Arc, Mutex};
use esp_soc::board::{BoardEdge, BoardModel, VirtualCycle};
use super::panel_inputs::PanelInputs;
use crate::i2c::{check_air, I2cDevice, Shtc3, Shtc3Air, SHTC3_ADDR};
use hub75::{Decoder, DecoderConfig, Refresh};
use hub75::render::{Optics, Renderer};

pub const W: usize = 128;
pub const H: usize = 64;
const SCAN: u64 = (H / 2) as u64;
const PANEL_SCALE: usize = 8;   // px per LED in PNGs and plain UIs

/// The controller the firmware's `Wire` drives: I2C0. AnimatedPixelClock begins `Wire` on 47/48
/// (`src/board/board_i2c.cpp:17`), and arduino-esp32 2.0.17 defines `TwoWire Wire = TwoWire(0)`
/// (`libraries/Wire/src/Wire.cpp:697`), port 0 of the IDF driver: the block at 0x6001_3000 that
/// the SoC model routes to `i2c[0]` (`periph.rs`, "I2C0").
pub const SHTC3_BUS: u8 = 0;

pub struct Panel {
    dec: Decoder,
    words16: Vec<u16>,
    acc: Vec<[u64; 3]>,          // on-clocks per LED in the open window
    acc_clocks: u64,
    acc_refreshes: u64,
    window: u64,                 // PCLKs per window, at least
    pub last: Refresh,           // the last closed window, as one refresh
    pub light: Vec<u16>,         // its R G B linear light per LED, 65535 = lit all its row slot
    pub frames: u64,             // closed windows
    pub refreshes: u64,
    pub words: u64,
    pub pclk: u64,
    renderer: RefCell<Renderer>,
    gpio_events: u64,
    /// The IR receiver on GPIO0, the EC11 knob on IO45/IO46 and BOOT (panel_inputs.rs)
    pub inputs: PanelInputs,
    /// The air at the SHTC3 (the `climate` verb) and its counters, shared with the device
    pub climate: Arc<Mutex<Shtc3Air>>,
    /// The board's cycle, for the SHTC3's wake-up, reset and measurement times
    climate_clock: Arc<AtomicU64>,
}

/// `climate <°C> <%RH>`: the air at the SHTC3 from the next measurement on.
fn parse_climate(args: &str) -> Result<(f64, f64), String> {
    let usage = || format!("climate <temperature °C> <humidity %RH>, got `{args}`");
    let v: Vec<f64> = args.split_whitespace().map(str::parse).collect::<Result<_, _>>().map_err(|_| usage())?;
    let [t, rh] = v[..] else { return Err(usage()) };
    check_air(t, rh)?;
    Ok((t, rh))
}

impl Default for Panel { fn default() -> Self { Self::new() } }

impl Panel {
    pub fn new() -> Self {
        Panel { dec: Decoder::new(DecoderConfig::default()), words16: Vec::new(), acc: vec![[0; 3]; W * H], acc_clocks: 0, acc_refreshes: 0,
                window: 10_000_000 / 60, last: Refresh::blank(W, H), light: vec![0; W * H * 3], frames: 0, refreshes: 0, words: 0, pclk: 0,
                renderer: RefCell::new(Renderer::new(Optics::default(), PANEL_SCALE)), gpio_events: 0, inputs: PanelInputs::new(),
                climate: Arc::default(), climate_clock: Arc::default() }
    }

    fn add_refresh(&mut self, r: Refresh) {
        self.refreshes += 1;
        for (a, c) in self.acc.iter_mut().zip(r.on_clk.iter()) { a[0] += c[0] as u64; a[1] += c[1] as u64; a[2] += c[2] as u64; }
        self.acc_clocks += r.total_clocks;
        self.acc_refreshes += 1;
        if self.acc_clocks >= self.window { self.close_window(); }
    }

    fn close_window(&mut self) {
        let full = (self.acc_clocks as f64 / SCAN as f64).max(1.0);
        for (i, a) in self.acc.iter_mut().enumerate() {
            for (l, v) in self.light[i * 3..i * 3 + 3].iter_mut().zip(a.iter()) { *l = ((*v as f64 / full).min(1.0) * 65535.0 + 0.5) as u16; }
            self.last.on_clk[i] = [a[0].min(u32::MAX as u64) as u32, a[1].min(u32::MAX as u64) as u32, a[2].min(u32::MAX as u64) as u32];
            *a = [0; 3];
        }
        self.last.total_clocks = self.acc_clocks;
        self.last.index = self.frames;
        self.acc_clocks = 0;
        self.acc_refreshes = 0;
        self.frames += 1;
    }
}

impl BoardModel for Panel {
    fn name(&self) -> &'static str { "hub75-panel" }
    fn gpio_changes(&mut self, changes: &[(u8, bool)]) { self.gpio_events += changes.len() as u64; }
    fn gpio_events(&self) -> u64 { self.gpio_events }
    fn lcd_i8080(&mut self, pclk_hz: u64, bus_bytes: u8, data: &[u8], eof: bool) {
        if bus_bytes != 2 { return; }
        if pclk_hz != self.pclk { self.pclk = pclk_hz; self.window = (pclk_hz / 60).max(1); }   // an eye's 1/60 s
        let mut words = std::mem::take(&mut self.words16);
        words.clear();
        words.extend(data.as_chunks::<2>().0.iter().map(|p| u16::from_le_bytes(*p)));
        self.words += words.len() as u64;
        if let Some(r) = self.dec.feed_segment(&words, eof) { self.add_refresh(r); }
        self.words16 = words;
        // A stream that never marks EOF still shows: close on clocks alone after four windows.
        if !eof && self.dec.pending_clocks() >= self.window.saturating_mul(4) { let r = self.dec.end_refresh(); self.add_refresh(r); }
    }
    fn display(&self) -> Option<(u32, u32, Vec<u16>, u64)> {
        let img = self.renderer.borrow_mut().render_panel(&self.last);
        Some(((W * PANEL_SCALE) as u32, (H * PANEL_SCALE) as u32, img.to_rgb565(), self.frames))
    }
    fn display_light(&self) -> Option<(u32, u32, Vec<u16>, u64)> { Some((W as u32, H as u32, self.light.clone(), self.frames)) }
    fn display_version(&self) -> u64 { self.frames }
    fn display_frames(&self) -> u64 { self.frames }
    /// A fresh SHTC3 each attach (boot, chip reset): idle, as after power-up (DS §5.2). A real one on
    /// the board's 3V3 keeps its state across an ESP32 reset; the firmware's cycle starts with the
    /// wake-up, which both states take, so it cannot tell.
    fn i2c_devices(&mut self) -> Vec<(u8, u8, Box<dyn I2cDevice>)> {
        vec![(SHTC3_BUS, SHTC3_ADDR, Box::new(Shtc3::new(self.climate.clone(), self.climate_clock.clone())))]
    }
    fn input_levels(&self) -> Vec<(u8, bool)> { self.inputs.input_levels() }
    fn next_deadline(&self) -> Option<VirtualCycle> { self.inputs.next_deadline() }
    fn advance_to(&mut self, cycle: VirtualCycle) { self.climate_clock.store(cycle, Relaxed); self.inputs.advance_to(cycle) }
    fn take_edges(&mut self) -> Vec<BoardEdge> { self.inputs.take_edges() }
    fn check_input(&self, cmd: &str, args: &str) -> Option<Result<(), String>> {
        if cmd == "climate" { return Some(parse_climate(args).map(|_| ())); }
        self.inputs.check_input(cmd, args)
    }
    fn input_at(&mut self, cycle: VirtualCycle, cmd: &str, args: &str) -> Result<(), String> {
        if cmd == "climate" {
            let (t, rh) = parse_climate(args)?;
            let mut air = self.climate.lock().expect("SHTC3 air mutex poisoned");
            (air.temp_c, air.humidity) = (t, rh);
            return Ok(());
        }
        self.inputs.input_at(cycle, cmd, args)
    }
    fn report(&self) -> String {
        let mut r = self.inputs.report(); if !r.is_empty() && !r.ends_with('\n') { r.push('\n'); }
        let a = self.climate.lock().expect("SHTC3 air mutex poisoned").clone();
        r += &format!("[panel] SHTC3 at I2C{} {:#04x}: air {:.2} °C {:.2} %RH, {} measurements, {} ID reads, {} NACKs\n",
                      SHTC3_BUS, SHTC3_ADDR, a.temp_c, a.humidity, a.measurements, a.id_reads, a.nacks);
        r + &format!("[panel] HUB75: {} bus words at {:.2} MHz PCLK, {} refreshes ({:.1} Hz), {} light windows\n",
                self.words, self.pclk as f64 / 1e6, self.refreshes, if self.words > 0 { self.refreshes as f64 * self.pclk as f64 / self.words as f64 } else { 0.0 }, self.frames)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::i2c::{crc8, raw_humidity, raw_temperature, I2c, INT_NACK, INT_TRANS_COMPLETE};

    const RSTART: u32 = 6 << 11;
    const STOP: u32 = 2 << 11;
    const fn write(n: u32) -> u32 { 1 << 11 | 1 << 8 | n }   // WRITE n bytes, ACK checked
    const fn read(n: u32) -> u32 { 3 << 11 | n }

    /// One transaction through the controller model, as the IDF driver lays it out. NACK?
    fn transact(bus: &mut I2c, tx: &[u8], cmds: &[u32]) -> bool {
        bus.write(0x18, 3 << 12);                                      // FIFO reset
        bus.int_raw = 0;
        for &b in tx { bus.write(0x1c, u32::from(b)); }
        for (i, &c) in cmds.iter().enumerate() { bus.write(0x58 + 4 * i as u32, c); }
        bus.write(0x04, 1 << 5);                                       // TRANS_START
        assert!(bus.int_raw & (INT_NACK | INT_TRANS_COMPLETE) != 0);
        bus.int_raw & INT_NACK != 0
    }

    #[test]
    fn the_shtc3_answers_on_i2c0_and_follows_the_climate_verb() {
        let mut panel = Panel::new();
        let devices = panel.i2c_devices();
        assert_eq!(devices.iter().map(|(b, a, _)| (*b, *a)).collect::<Vec<_>>(), [(0, 0x70)]);
        let mut bus = I2c::new();
        for (_, addr, dev) in devices { bus.attach(addr, dev); }
        let cmd = |bus: &mut I2c, c: u16| { let [h, l] = c.to_be_bytes(); transact(bus, &[0xe0, h, l], &[RSTART, write(3), STOP]) };
        let get6 = |bus: &mut I2c| {
            let nack = transact(bus, &[0xe1], &[RSTART, write(1), read(6), STOP]);
            (nack, (0..6).map(|_| bus.read(0x1c) as u8).collect::<Vec<_>>())
        };

        assert!(panel.check_input("climate", "19.5 60").is_some_and(|r| r.is_ok()));
        for bad in ["", "19.5", "19.5 60 1", "warm 60", "130 50", "20 -1"] {
            assert!(panel.check_input("climate", bad).is_some_and(|r| r.is_err()), "{bad}");
        }
        assert!(panel.check_input("knob", "cw 2").is_some_and(|r| r.is_ok()), "the inputs' verbs still reach them");
        assert!(panel.check_input("warp", "9").is_none());

        let us = crate::periph::CPU_HZ / 1_000_000;
        panel.advance_to(1_000 * us);
        assert!(!cmd(&mut bus, 0x3517), "wake-up");
        panel.advance_to(2_000 * us);
        assert!(!cmd(&mut bus, 0x7866), "measure: normal mode, T first, no stretching");
        panel.advance_to(12_000 * us);
        assert!(get6(&mut bus).0, "a read 10 ms in: NACK, still measuring");
        panel.advance_to(17_000 * us);
        let (nack, f) = get6(&mut bus);
        assert!(!nack);
        let t = raw_temperature(22.5).to_be_bytes();
        assert_eq!(f, [t[0], t[1], crc8(&t), f[3], f[4], f[5]]);
        assert_eq!(u16::from_be_bytes([f[3], f[4]]), raw_humidity(45.0));

        panel.input_at(18_000 * us, "climate", "21.8 52").unwrap();
        assert!(!cmd(&mut bus, 0x7866));
        panel.advance_to(31_000 * us);
        let (_, f) = get6(&mut bus);
        assert_eq!((u16::from_be_bytes([f[0], f[1]]), u16::from_be_bytes([f[3], f[4]])), (raw_temperature(21.8), raw_humidity(52.0)));
        assert!(!cmd(&mut bus, 0xb098), "sleep");
        assert!(cmd(&mut bus, 0x7866), "asleep: refused");
        assert!(panel.report().contains("SHTC3 at I2C0 0x70: air 21.80 °C 52.00 %RH, 2 measurements"), "{}", panel.report());
    }
}
