//! The NickoScope LED panel: a Waveshare ESP32-S3-RGB-Matrix board driving a 128x64 HUB75 panel
//! (two 64x64, 1/32 scan) from the LCD_CAM i8080 bus through a looping GDMA ring, the way
//! ESP32-HUB75-MatrixPanel-DMA 3.0.14 does it on the S3.
//!
//! The words go to the `hub75` crate's decoder, a generic HUB75 state machine (shift, latch,
//! OE-gated light in rows a and a+32), one GDMA descriptor at a time; the descriptor the driver
//! marks SUC_EOF closes a refresh. Refreshes are summed into windows of at least 1/60 s, an eye's
//! worth, and published as light: an LED's share of the most a 1/32-scan LED can emit.
use std::cell::RefCell;
use esp_soc::board::{BoardEdge, BoardModel, VirtualCycle};
use super::panel_inputs::PanelInputs;
use hub75::{Decoder, DecoderConfig, Refresh};
use hub75::render::{Optics, Renderer};

pub const W: usize = 128;
pub const H: usize = 64;
const SCAN: u64 = (H / 2) as u64;
const PANEL_SCALE: usize = 8;   // px per LED in PNGs and plain UIs

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
    /// A camera instead of an eye: None integrates whole refreshes (an eye, 1/60 s); Some(s)
    /// shows what an exposure of s seconds catches of the scan, the bands a phone films.
    pub shutter_s: Option<f64>,
    ring: Vec<u16>,              // the last RING words the bus clocked out, for the shutter
    ring_at: usize,
    ring_len: usize,
    shutter_dec: Decoder,
    shutter_phase: f64,          // where in the ring the next exposure starts, 0..1 of a refresh
}

/// Words kept for the shutter: two refreshes of the library's 128x64 chain (94,208 words each).
const RING: usize = 2 * 94_208 + 4096;

impl Default for Panel { fn default() -> Self { Self::new() } }

impl Panel {
    pub fn new() -> Self {
        Panel { dec: Decoder::new(DecoderConfig::default()), words16: Vec::new(), acc: vec![[0; 3]; W * H], acc_clocks: 0, acc_refreshes: 0,
                window: 10_000_000 / 60, last: Refresh::blank(W, H), light: vec![0; W * H * 3], frames: 0, refreshes: 0, words: 0, pclk: 0,
                renderer: RefCell::new(Renderer::new(Optics::default(), PANEL_SCALE)), gpio_events: 0, inputs: PanelInputs::new(),
                shutter_s: None, ring: vec![0; RING], ring_at: 0, ring_len: 0, shutter_dec: Decoder::new(DecoderConfig::default()), shutter_phase: 0.0 }
    }

    fn add_refresh(&mut self, r: Refresh) {
        self.refreshes += 1;
        for (a, c) in self.acc.iter_mut().zip(r.on_clk.iter()) { a[0] += c[0] as u64; a[1] += c[1] as u64; a[2] += c[2] as u64; }
        self.acc_clocks += r.total_clocks;
        self.acc_refreshes += 1;
        if self.acc_clocks >= self.window { self.close_window(); }
    }

    /// One camera exposure of `s` seconds out of the ring: decode a row's worth of words before it
    /// (to settle the shift register and latch, not counted), then the exposure itself. An LED is
    /// 1.0 when it was lit for all of the exposure or of its row slot, whichever is shorter.
    fn shutter_frame(&mut self, s: f64) {
        let pclk = self.pclk.max(1) as f64;
        let refresh = (self.last.total_clocks.max(1) / 2).max(1) as usize;   // `last` holds two refreshes, near enough
        let expo = ((s * pclk) as usize).clamp(1, refresh);
        let pre = 2 * W;
        if self.ring_len < expo + pre + refresh { return; }
        // Not locked to the scan, like a phone: each exposure starts a golden-ratio step further on.
        self.shutter_phase = (self.shutter_phase + 0.618_034) % 1.0;
        let back = expo + pre + (self.shutter_phase * refresh as f64) as usize;
        let start = (self.ring_at + RING - back) % RING;
        let words: Vec<u16> = (0..expo + pre).map(|k| self.ring[(start + k) % RING]).collect();
        self.shutter_dec.feed(&words[..pre]);
        let _ = self.shutter_dec.end_refresh();
        self.shutter_dec.feed(&words[pre..]);
        let r = self.shutter_dec.end_refresh();
        let slot = (refresh as f64 / SCAN as f64).max(1.0);
        let full = (expo as f64).min(slot);
        for (i, c) in r.on_clk.iter().enumerate() {
            for (l, v) in self.light[i * 3..i * 3 + 3].iter_mut().zip(c.iter()) { *l = ((*v as f64 / full).min(1.0) * 65535.0 + 0.5) as u16; }
        }
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
        if let Some(sh) = self.shutter_s { self.shutter_frame(sh); }
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
        if self.shutter_s.is_some() {
            let mut src = &words[words.len().saturating_sub(RING)..];
            while !src.is_empty() {
                let n = src.len().min(RING - self.ring_at);
                self.ring[self.ring_at..self.ring_at + n].copy_from_slice(&src[..n]);
                self.ring_at = (self.ring_at + n) % RING;
                src = &src[n..];
            }
            self.ring_len = (self.ring_len + words.len()).min(RING);
        }
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
    fn input_levels(&self) -> Vec<(u8, bool)> { self.inputs.input_levels() }
    fn next_deadline(&self) -> Option<VirtualCycle> { self.inputs.next_deadline() }
    fn advance_to(&mut self, cycle: VirtualCycle) { self.inputs.advance_to(cycle) }
    fn take_edges(&mut self) -> Vec<BoardEdge> { self.inputs.take_edges() }
    fn check_input(&self, cmd: &str, args: &str) -> Option<Result<(), String>> {
        if cmd == "shutter" { return Some(parse_shutter(args).map(|_| ())); }
        self.inputs.check_input(cmd, args)
    }
    fn input_at(&mut self, cycle: VirtualCycle, cmd: &str, args: &str) -> Result<(), String> {
        if cmd == "shutter" {
            self.shutter_s = parse_shutter(args)?;
            if self.shutter_s.is_none() { self.ring_len = 0; }
            return Ok(());
        }
        self.inputs.input_at(cycle, cmd, args)
    }
    fn report(&self) -> String {
        let mut r = self.inputs.report(); if !r.is_empty() && !r.ends_with('\n') { r.push('\n'); }
        r + &format!("[panel] HUB75: {} bus words at {:.2} MHz PCLK, {} refreshes ({:.1} Hz), {} light windows\n",
                self.words, self.pclk as f64 / 1e6, self.refreshes, if self.words > 0 { self.refreshes as f64 * self.pclk as f64 / self.words as f64 } else { 0.0 }, self.frames)
    }
}

/// `shutter eye` (whole refreshes) or `shutter N` (an exposure of 1/N s, N = 30..100000).
fn parse_shutter(args: &str) -> Result<Option<f64>, String> {
    let a = args.trim();
    if a.is_empty() || a == "eye" || a == "off" { return Ok(None); }
    let n: f64 = a.trim_start_matches("1/").parse().map_err(|_| "shutter: `eye` or N for 1/N s".to_string())?;
    if !(30.0..=100_000.0).contains(&n) { return Err("shutter: N from 30 to 100000 (1/N s)".into()); }
    Ok(Some(1.0 / n))
}
