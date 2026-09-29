//! The NickoScope LED panel: a Waveshare ESP32-S3-RGB-Matrix board driving a 128x64 HUB75 panel
//! (two 64x64, 1/32 scan) from the LCD_CAM i8080 bus through a looping GDMA ring, the way
//! ESP32-HUB75-MatrixPanel-DMA 3.0.14 does it on the S3.
//!
//! The bus word, per the library's pin order (ESP32-HUB75-MatrixPanel-I2S-DMA.h, the 16 data pins
//! handed to Bus_Parallel16): bits 0-5 R1 G1 B1 R2 G2 B2, bit 6 LAT, bit 7 OE (1 = dark),
//! bits 8-12 row address A-E. CLK is the bus's PCLK and is not in the word.
//!
//! This first decoder is the generic HUB75 state machine: every word shifts one column in, LAT
//! copies the shift register to the output latch, and while OE is low every LED whose latched
//! bit is set is lit, in rows `a` and `a + 32`. Light is counted in PCLKs over a refresh window;
//! the picture is each LED's share of the most a 1/32-scan LED can be lit. Two assumptions, not
//! verified against the FM6124 datasheet: shifted word k lands on column x = k, and a latch shows
//! the data latched in it until the next latch.
use esp_soc::board::BoardModel;

pub const W: usize = 128;
pub const H: usize = 64;
const SCAN: usize = H / 2;

pub struct Panel {
    sr: [u8; W],            // shift register, 6 colour bits per column
    col: usize,
    latch: [u8; W],
    addr: usize,
    on_run: u64,            // PCLKs lit with the current latch and address, not yet added
    on: Vec<[u64; 3]>,      // lit PCLKs per LED and channel in this refresh window
    clocks: u64,            // PCLKs in this refresh window
    window: u64,            // PCLKs per refresh window, at least
    wraps: u64,             // refreshes begun in this window (the row address wrapping to a lower one)
    pub light: Vec<u16>,    // last finished window: R G B linear light per LED, 65535 = lit all its row slot
    pub frame: Vec<u16>,    // the same, sRGB-encoded RGB565 for PNGs and plain UIs
    pub frames: u64,
    pub words: u64,
    pub pclk: u64,
    gpio_events: u64,
}

impl Default for Panel { fn default() -> Self { Self::new() } }

impl Panel {
    pub fn new() -> Self {
        Panel { sr: [0; W], col: 0, latch: [0; W], addr: 0, on_run: 0, on: vec![[0; 3]; W * H], clocks: 0, window: 0, wraps: 0,
                light: vec![0; W * H * 3], frame: vec![0; W * H], frames: 0, words: 0, pclk: 0, gpio_events: 0 }
    }

    fn flush_run(&mut self) {
        if self.on_run == 0 { return; }
        let (top, bot) = (self.addr * W, (self.addr + SCAN) * W);
        for x in 0..W {
            let b = self.latch[x];
            if b == 0 { continue; }
            let t = &mut self.on[top + x];
            if b & 1 != 0 { t[0] += self.on_run; } if b & 2 != 0 { t[1] += self.on_run; } if b & 4 != 0 { t[2] += self.on_run; }
            let u = &mut self.on[bot + x];
            if b & 8 != 0 { u[0] += self.on_run; } if b & 16 != 0 { u[1] += self.on_run; } if b & 32 != 0 { u[2] += self.on_run; }
        }
        self.on_run = 0;
    }

    fn finish_window(&mut self) {
        self.flush_run();
        // The most light a 1/32-scan LED can give: lit through its whole row slot.
        let full = (self.clocks as f64 / SCAN as f64).max(1.0);
        // sRGB OETF (CSS Color 4, conversions.js, lin_sRGB_to_sRGB)
        let enc = |l: f64| -> f64 { if l <= 0.0031308 { 12.92 * l } else { 1.055 * l.powf(1.0 / 2.4) - 0.055 } };
        for (i, c) in self.on.iter_mut().enumerate() {
            let l = [0, 1, 2].map(|k| (c[k] as f64 / full).min(1.0));
            for k in 0..3 { self.light[i * 3 + k] = (l[k] * 65535.0 + 0.5) as u16; }
            let (r, g, b) = (enc(l[0]), enc(l[1]), enc(l[2]));
            self.frame[i] = (((r * 31.0 + 0.5) as u16) << 11) | (((g * 63.0 + 0.5) as u16) << 5) | ((b * 31.0 + 0.5) as u16);
            *c = [0; 3];
        }
        self.clocks = 0;
        self.wraps = 0;
        self.frames += 1;
    }

    /// One bus word per PCLK, as the HUB75 lines see it.
    pub fn feed(&mut self, words: &[u16]) {
        for &w in words {
            self.sr[self.col & (W - 1)] = (w & 0x3f) as u8;
            self.col += 1;
            let addr = ((w >> 8) & 0x1f) as usize;
            if w & 0x40 != 0 { self.flush_run(); self.latch = self.sr; self.col = 0; }
            if addr != self.addr {
                self.flush_run();
                // A refresh begins where the scan wraps to a lower row. The window closes on one,
                // so it holds whole refreshes and every row the same number of times.
                if addr < self.addr { self.wraps += 1; if self.clocks >= self.window && self.wraps >= 2 { self.finish_window(); self.wraps = 1; } }
                self.addr = addr;
            }
            if w & 0x80 == 0 { self.on_run += 1; }
            self.clocks += 1;
            // A stream with no scan (no address change for 4 windows) still shows, as darkness.
            if self.clocks >= self.window.saturating_mul(4) { self.finish_window(); }
        }
    }
}

impl BoardModel for Panel {
    fn name(&self) -> &'static str { "panel" }
    fn gpio_changes(&mut self, changes: &[(u8, bool)]) { self.gpio_events += changes.len() as u64; }
    fn gpio_events(&self) -> u64 { self.gpio_events }
    fn lcd_i8080(&mut self, pclk_hz: u64, bus_bytes: u8, data: &[u8]) {
        if bus_bytes != 2 { return; }
        if pclk_hz != self.pclk { self.pclk = pclk_hz; self.window = (pclk_hz / 60).max(1); }   // a 60 Hz eye window
        let words: Vec<u16> = data.chunks_exact(2).map(|p| u16::from_le_bytes([p[0], p[1]])).collect();
        self.words += words.len() as u64;
        self.feed(&words);
    }
    fn display(&self) -> Option<(u32, u32, Vec<u16>, u64)> { Some((W as u32, H as u32, self.frame.clone(), self.frames)) }
    fn display_light(&self) -> Option<(u32, u32, Vec<u16>, u64)> { Some((W as u32, H as u32, self.light.clone(), self.frames)) }
    fn display_version(&self) -> u64 { self.frames }
    fn display_frames(&self) -> u64 { self.frames }
    fn report(&self) -> String {
        format!("[panel] HUB75: {} bus words at {:.2} MHz PCLK, {} refresh windows\n", self.words, self.pclk as f64 / 1e6, self.frames)
    }
}
