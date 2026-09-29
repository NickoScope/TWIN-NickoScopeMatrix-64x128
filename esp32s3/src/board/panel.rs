//! The NickoScope LED panel: a Waveshare ESP32-S3-RGB-Matrix board driving a 128x64 HUB75 panel
//! (two 64x64, 1/32 scan) from the LCD_CAM i8080 bus through a looping GDMA ring, the way
//! ESP32-HUB75-MatrixPanel-DMA 3.0.14 does it on the S3.
//!
//! The words go to the `hub75` crate's decoder, a generic HUB75 state machine (shift, latch,
//! OE-gated light in rows a and a+32), one GDMA descriptor at a time; the descriptor the driver
//! marks SUC_EOF closes a refresh. Refreshes are summed into windows of at least 1/60 s, an eye's
//! worth, and published as light: an LED's share of the most a 1/32-scan LED can emit.
use std::cell::RefCell;
use esp_soc::board::BoardModel;
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
}

impl Default for Panel { fn default() -> Self { Self::new() } }

impl Panel {
    pub fn new() -> Self {
        Panel { dec: Decoder::new(DecoderConfig::default()), words16: Vec::new(), acc: vec![[0; 3]; W * H], acc_clocks: 0, acc_refreshes: 0,
                window: 10_000_000 / 60, last: Refresh::blank(W, H), light: vec![0; W * H * 3], frames: 0, refreshes: 0, words: 0, pclk: 0,
                renderer: RefCell::new(Renderer::new(Optics::default(), PANEL_SCALE)), gpio_events: 0 }
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
            for k in 0..3 { self.light[i * 3 + k] = ((a[k] as f64 / full).min(1.0) * 65535.0 + 0.5) as u16; }
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
    fn name(&self) -> &'static str { "panel" }
    fn gpio_changes(&mut self, changes: &[(u8, bool)]) { self.gpio_events += changes.len() as u64; }
    fn gpio_events(&self) -> u64 { self.gpio_events }
    fn lcd_i8080(&mut self, pclk_hz: u64, bus_bytes: u8, data: &[u8], eof: bool) {
        if bus_bytes != 2 { return; }
        if pclk_hz != self.pclk { self.pclk = pclk_hz; self.window = (pclk_hz / 60).max(1); }   // an eye's 1/60 s
        let mut words = std::mem::take(&mut self.words16);
        words.clear();
        words.extend(data.chunks_exact(2).map(|p| u16::from_le_bytes([p[0], p[1]])));
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
    fn report(&self) -> String {
        format!("[panel] HUB75: {} bus words at {:.2} MHz PCLK, {} refreshes ({:.1} Hz), {} light windows\n",
                self.words, self.pclk as f64 / 1e6, self.refreshes, if self.words > 0 { self.refreshes as f64 * self.pclk as f64 / self.words as f64 } else { 0.0 }, self.frames)
    }
}
