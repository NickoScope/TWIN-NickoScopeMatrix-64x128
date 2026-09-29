//! A generic HUB75 state machine over the 16-bit words that LCD_CAM clocks out.
//!
//! One word is one PCLK. Per word the decoder does what the panel's column drivers and row
//! decoder do, and nothing that depends on how a particular library laid out its buffers:
//!
//! 1. the six colour bits (R1 G1 B1 R2 G2 B2) shift into a `width`-column shift register;
//! 2. LAT copies the shift register into the output latch (when exactly: [`LatchMode`]);
//! 3. while OE is at its lit level, every LED whose latched bit is 1 gains one on-clock, in rows
//!    `a` and `a + height/2`, where `a` is the address on A..E.
//!
//! Sources for the default word layout and geometry (all read for this crate):
//! - bit defines: `ESP32-HUB75-MatrixPanel-I2S-DMA.h:91-112` (R1=bit0 .. B2=bit5, LAT=bit6,
//!   OE=bit7, A..E=bits 8..12) in the library copy the firmware compiles (tag 3.0.14);
//! - the LCD data-line wiring that makes those the bus bits: `ESP32-HUB75-MatrixPanel-I2S-DMA.cpp:344-359`
//!   (`pin_d0 = r1` .. `pin_d12 = e`, `pin_d13..15 = -1`), and `platforms/esp32s3/gdma_lcd_parallel16.cpp:200-203`
//!   (`LCD_DATA_OUT0_IDX + i`, no byte or bit swap: `lcd_8bits_order = 0`, `lcd_bit_order = 0`,
//!   `gdma_lcd_parallel16.cpp:178-179`). CLK is the LCD PCLK (`gdma_lcd_parallel16.cpp:215`), not a data bit;
//! - OE = 1 blanks: `setBrightnessOE` clears `BIT_OE` inside the lit window and sets it outside
//!   with the comment "Disable output" (`ESP32-HUB75-MatrixPanel-I2S-DMA.cpp:731-738`);
//! - two rows in parallel: `MATRIX_ROWS_IN_PARALLEL 2` (`.h:67`), rows `y >= ROWS_PER_FRAME` use the
//!   RGB2 bits (`.cpp:425-430`);
//! - 128x64: `HUB75_PANEL_W 64`, `HUB75_PANEL_H 64`, `HUB75_CHAIN 2` (firmware `src/display/matrix_display.h:26-28`).
//!
//! Two behaviours are NOT verified against the column-driver datasheet (FM6124HJ, only a Chinese
//! datasheet exists and nobody on the project has read it; ADR-TWIN-01 §4.2 B2 and §6 #11, #13).
//! They are parameters, with the library-consistent choice as the default:
//! - [`LatchMode`]: which shift-register content a LAT pulse puts on the LEDs;
//! - [`ColumnOrder`]: which physical column `x` the k-th clocked word of a line ends up in.

/// Panel geometry as the shift chain sees it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PanelGeom {
    /// Columns in the whole chain (panels side by side): 2 x 64 = 128.
    pub width: usize,
    /// LED rows. Row `a` and row `a + height/2` light together (two rows in parallel).
    pub height: usize,
}

impl PanelGeom {
    /// The owner's panel: two 64x64 modules chained (firmware `matrix_display.h:26-28`), 1/32 scan
    /// (KB `docs/01-panel.md`, "Scan rate 1/32").
    pub const TWO_64X64: PanelGeom = PanelGeom { width: 128, height: 64 };

    /// Rows per scan slot group = the number of distinct addresses (32 for 1/32 scan).
    pub fn scan_rows(&self) -> usize {
        self.height / 2
    }
}

/// Where the HUB75 signals sit in the 16-bit word.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WordLayout {
    /// R1 is this bit; G1, B1, R2, G2, B2 follow in the next five bits.
    pub rgb_shift: u8,
    pub lat_bit: u8,
    pub oe_bit: u8,
    /// Level of the OE bit that blanks the LEDs (`true`: 1 = blanked, HUB75 OE is active low).
    pub oe_blank_level: bool,
    /// A is this bit; B, C, D, E follow.
    pub addr_shift: u8,
    /// Number of address lines wired (5 = A..E).
    pub addr_bits: u8,
}

impl WordLayout {
    /// ESP32-HUB75-MatrixPanel-DMA 3.0.14 on the ESP32-S3 (`.h:91-112`, `.cpp:344-359`).
    pub const LIB_3_0_14: WordLayout =
        WordLayout { rgb_shift: 0, lat_bit: 6, oe_bit: 7, oe_blank_level: true, addr_shift: 8, addr_bits: 5 };
}

/// Which shift-register content a LAT pulse puts on the LEDs.
///
/// The library sets LAT on exactly one word per line, the last one (word `width-1`,
/// `.cpp:645`), and keeps OE high (blanked) on that word and on word 0 at every brightness
/// (`.cpp:704-724`: the lit window is centred, at most `width - latch_blanking - 1` words wide,
/// so it never reaches word 0 or word `width-1`).
/// For that stream the three modes give the same picture except [`LatchMode::RisingEdge`], which
/// loses the last word of every line (the image moves one column). None of the three is checked
/// against the FM6124HJ datasheet; ADR-TWIN-01 §6 #10/#11 name the real-panel ramp test that
/// would tell.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum LatchMode {
    /// Transparent latch: while LAT is 1 the output follows the shift register, including the
    /// word clocked in on that same PCLK. Library-consistent: the data word that carries LAT is
    /// the line's last pixel (`x = width-1`), and it is shown. This is the ADR's assumption
    /// "data latched on word 127 lights in the next segment's OE window". Default.
    #[default]
    Level,
    /// Captures on the 0->1 edge of LAT, before that word's own data is shifted in (LAT rises at
    /// the start of the word, PCLK rises mid-word with `lcd_ck_out_edge = 0`,
    /// `gdma_lcd_parallel16.cpp:102`). Here for the ramp test; not library-consistent.
    RisingEdge,
    /// Captures on the 1->0 edge of LAT: the shift register as it stood after the last LAT-high
    /// word. Shows new data one word later than `Level`.
    FallingEdge,
}

/// Which physical column `x` the k-th word of a line (k = 0 is the first clocked) lands in.
///
/// After `width` clocks the first word sits at the far end of the chain. The library writes
/// pixel `x` to word `x` (`.cpp:453-457`), so `Identity` is what the library means. Whether the
/// real cabling shows `x = 0..63` on the left panel is ADR-TWIN-01 §6 #13, not verified.
#[derive(Clone, Debug, PartialEq, Eq, Default)]
pub enum ColumnOrder {
    /// x = k. Library-consistent default.
    #[default]
    Identity,
    /// x = width-1-k (the whole chain mirrored).
    Reversed,
    /// Panels of `panel_width` columns in reverse order, each panel's columns unchanged.
    ReversePanelOrder { panel_width: usize },
    /// Explicit table: `x = table[k]`, a permutation of `0..width`.
    Custom(Vec<usize>),
}

impl ColumnOrder {
    fn table(&self, width: usize) -> Vec<usize> {
        let t: Vec<usize> = match self {
            ColumnOrder::Identity => (0..width).collect(),
            ColumnOrder::Reversed => (0..width).rev().collect(),
            ColumnOrder::ReversePanelOrder { panel_width } => {
                let pw = *panel_width;
                assert!(pw > 0 && width.is_multiple_of(pw), "panel_width must divide the chain width");
                let n = width / pw;
                (0..width).map(|k| (n - 1 - k / pw) * pw + k % pw).collect()
            }
            ColumnOrder::Custom(t) => t.clone(),
        };
        let mut seen = vec![false; width];
        assert_eq!(t.len(), width, "column table must have one entry per column");
        for &x in &t {
            assert!(x < width && !seen[x], "column table must be a permutation of 0..width");
            seen[x] = true;
        }
        t
    }
}

/// Everything the decoder needs to know about the panel and its wiring.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DecoderConfig {
    pub geom: PanelGeom,
    pub layout: WordLayout,
    pub latch: LatchMode,
    pub columns: ColumnOrder,
}

impl Default for DecoderConfig {
    /// The owner's panel with the library 3.0.14 word layout and the library-consistent
    /// (unverified) latch and column rules.
    fn default() -> Self {
        DecoderConfig {
            geom: PanelGeom::TWO_64X64,
            layout: WordLayout::LIB_3_0_14,
            latch: LatchMode::Level,
            columns: ColumnOrder::Identity,
        }
    }
}

/// What every LED emitted over one refresh (the words between two `end_refresh` calls).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Refresh {
    pub width: usize,
    pub height: usize,
    /// On-clocks per LED and channel, row-major: `on_clk[y * width + x] = [r, g, b]`.
    pub on_clk: Vec<[u32; 3]>,
    /// PCLK cycles (= words) in this refresh. Duty of an LED channel = on_clk / total_clocks.
    pub total_clocks: u64,
    /// Words with OE at its lit level.
    pub lit_clocks: u64,
    /// Latch transfers seen.
    pub latches: u64,
    /// Sequence number of this refresh since the decoder was created.
    pub index: u64,
}

impl Refresh {
    pub fn blank(width: usize, height: usize) -> Refresh {
        Refresh { width, height, on_clk: vec![[0; 3]; width * height], total_clocks: 0, lit_clocks: 0, latches: 0, index: 0 }
    }

    pub fn get(&self, x: usize, y: usize) -> [u32; 3] {
        self.on_clk[y * self.width + x]
    }

    /// Fraction of the refresh each channel of LED (x, y) was lit.
    pub fn duty(&self, x: usize, y: usize) -> [f64; 3] {
        let t = self.total_clocks.max(1) as f64;
        self.get(x, y).map(|c| c as f64 / t)
    }

    /// Wall time of this refresh at a given PCLK (see [`crate::timing::lcd_pclk_hz`]).
    pub fn duration_s(&self, pclk_hz: f64) -> f64 {
        self.total_clocks as f64 / pclk_hz
    }
}

/// A GPIO level change on a HUB75 line while the pin is under GPIO (not LCD_CAM) control,
/// e.g. the FM6126A register writes bit-banged before DMA starts.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GpioEvent {
    pub t_ns: u64,
    pub pin: crate::gpio::HubPin,
    pub level: bool,
}

/// Keep at most this many GPIO events (the FM6126A init is about 3,500).
pub const GPIO_LOG_CAP: usize = 1 << 16;

const MODE_LEVEL: u8 = 0;
const MODE_RISING: u8 = 1;
const MODE_FALLING: u8 = 2;
const NO_ADDR: u16 = u16::MAX;

/// Streaming HUB75 decoder. Feed words as the GDMA delivers them, call [`Decoder::end_refresh`]
/// at every `suc_eof` (end of one chain pass). No allocation per word; one flush of
/// `width` columns per latch or address change.
pub struct Decoder {
    cfg: DecoderConfig,
    width: usize,
    scan: usize,
    rgb_shift: u32,
    lat_mask: u16,
    oe_mask: u16,
    oe_lit: u16,
    addr_shift: u32,
    addr_mask: u16,
    /// Shift register as a ring: the oldest entry sits at `head`.
    sr: Vec<u8>,
    head: usize,
    /// Output latch, in clock order: `latch[k]` = k-th clocked word of the latched line.
    latch: Vec<u8>,
    prev_lat: bool,
    run_addr: u16,
    run_len: u32,
    /// On-clocks in clock order: `acc[(row * width + k) * 3 + c]`.
    acc: Vec<u32>,
    col_x: Vec<usize>,
    total: u64,
    lit: u64,
    latches: u64,
    index: u64,
    gpio: Vec<GpioEvent>,
    gpio_dropped: u64,
}

impl Decoder {
    pub fn new(cfg: DecoderConfig) -> Decoder {
        let g = cfg.geom;
        let scan = g.scan_rows();
        assert!(g.width > 0 && scan > 0 && g.height == 2 * scan, "height must be two rows per address");
        assert!(scan.is_power_of_two() && scan <= 1 << 12, "rows per address must be a power of two");
        let l = cfg.layout;
        assert!(l.rgb_shift <= 10 && l.lat_bit < 16 && l.oe_bit < 16 && l.addr_bits <= 12);
        assert!(l.addr_shift as u32 + l.addr_bits as u32 <= 16);
        let oe_mask = 1u16 << l.oe_bit;
        // Address lines beyond the panel's scan are not wired: a 1/16 panel ignores E.
        let wired = ((1u32 << l.addr_bits) - 1) as u16;
        let addr_mask = wired & (scan as u16 - 1);
        let col_x = cfg.columns.table(g.width);
        Decoder {
            width: g.width,
            scan,
            rgb_shift: l.rgb_shift as u32,
            lat_mask: 1 << l.lat_bit,
            oe_mask,
            oe_lit: if l.oe_blank_level { 0 } else { oe_mask },
            addr_shift: l.addr_shift as u32,
            addr_mask,
            sr: vec![0; g.width],
            head: 0,
            latch: vec![0; g.width],
            prev_lat: false,
            run_addr: NO_ADDR,
            run_len: 0,
            acc: vec![0; g.height * g.width * 3],
            col_x,
            total: 0,
            lit: 0,
            latches: 0,
            index: 0,
            gpio: Vec::new(),
            gpio_dropped: 0,
            cfg,
        }
    }

    pub fn config(&self) -> &DecoderConfig {
        &self.cfg
    }

    /// Clocks fed since the last `end_refresh`.
    pub fn pending_clocks(&self) -> u64 {
        self.total
    }

    /// Power-on state: empty shift register and latch, nothing accumulated.
    pub fn reset(&mut self) {
        self.sr.fill(0);
        self.head = 0;
        self.latch.fill(0);
        self.prev_lat = false;
        self.run_addr = NO_ADDR;
        self.run_len = 0;
        self.acc.fill(0);
        self.total = 0;
        self.lit = 0;
        self.latches = 0;
    }

    /// Clock `words` into the panel, one word per PCLK.
    pub fn feed(&mut self, words: &[u16]) {
        match self.cfg.latch {
            LatchMode::Level => self.feed_mode::<MODE_LEVEL>(words),
            LatchMode::RisingEdge => self.feed_mode::<MODE_RISING>(words),
            LatchMode::FallingEdge => self.feed_mode::<MODE_FALLING>(words),
        }
    }

    /// One GDMA descriptor's worth of words; returns the finished refresh when the descriptor
    /// carried `suc_eof` (interface I1 of ADR-TWIN-01 §4.3).
    pub fn feed_segment(&mut self, words: &[u16], suc_eof: bool) -> Option<Refresh> {
        self.feed(words);
        if suc_eof {
            Some(self.end_refresh())
        } else {
            None
        }
    }

    #[inline(always)]
    fn feed_mode<const MODE: u8>(&mut self, words: &[u16]) {
        let Decoder {
            width, scan, rgb_shift, lat_mask, oe_mask, oe_lit, addr_shift, addr_mask, sr, head, latch, prev_lat,
            run_addr, run_len, acc, total, lit, latches, ..
        } = self;
        let (width, scan, rgb_shift, lat_mask, oe_mask, oe_lit, addr_shift, addr_mask) =
            (*width, *scan, *rgb_shift, *lat_mask, *oe_mask, *oe_lit, *addr_shift, *addr_mask);
        let mut h = *head;
        let mut prev = *prev_lat;
        let mut ra = *run_addr;
        let mut rl = *run_len;
        let mut n_lit = 0u64;
        let mut n_lat = 0u64;
        for &w in words {
            let lat = w & lat_mask != 0;
            if (MODE == MODE_RISING && lat && !prev) || (MODE == MODE_FALLING && !lat && prev) {
                flush(acc, latch, width, scan, ra, rl);
                rl = 0;
                capture(latch, sr, h);
                n_lat += 1;
            }
            sr[h] = ((w >> rgb_shift) & 0x3f) as u8;
            h += 1;
            if h == width {
                h = 0;
            }
            if MODE == MODE_LEVEL && lat {
                flush(acc, latch, width, scan, ra, rl);
                rl = 0;
                capture(latch, sr, h);
                n_lat += 1;
            }
            prev = lat;
            if w & oe_mask == oe_lit {
                let a = (w >> addr_shift) & addr_mask;
                if a != ra {
                    flush(acc, latch, width, scan, ra, rl);
                    rl = 0;
                    ra = a;
                }
                rl += 1;
                n_lit += 1;
            }
        }
        *head = h;
        *prev_lat = prev;
        *run_addr = ra;
        *run_len = rl;
        *total += words.len() as u64;
        *lit += n_lit;
        *latches += n_lat;
    }

    /// Close the current refresh (call at every `suc_eof`) and start the next one. The shift
    /// register and the latch keep their contents, as on the panel.
    pub fn end_refresh(&mut self) -> Refresh {
        let mut r = Refresh::blank(self.width, self.cfg.geom.height);
        self.end_refresh_into(&mut r);
        r
    }

    /// Like [`Decoder::end_refresh`] but reuses `out`'s allocation.
    pub fn end_refresh_into(&mut self, out: &mut Refresh) {
        flush(&mut self.acc, &self.latch, self.width, self.scan, self.run_addr, self.run_len);
        self.run_len = 0;
        let (w, h) = (self.width, self.cfg.geom.height);
        out.width = w;
        out.height = h;
        out.on_clk.clear();
        out.on_clk.resize(w * h, [0; 3]);
        for y in 0..h {
            let row = &self.acc[y * w * 3..(y + 1) * w * 3];
            let dst = &mut out.on_clk[y * w..(y + 1) * w];
            for (k, c) in row.as_chunks::<3>().0.iter().enumerate() {
                dst[self.col_x[k]] = *c;
            }
        }
        self.acc.fill(0);
        out.total_clocks = self.total;
        out.lit_clocks = self.lit;
        out.latches = self.latches;
        out.index = self.index;
        self.index += 1;
        self.total = 0;
        self.lit = 0;
        self.latches = 0;
    }

    /// Store a GPIO-driven level change on a HUB75 line (FM6126A init). Logged only: it has no
    /// electrical effect on the decoded picture. REG1/REG2's effect on LED current is unknown
    /// (ADR-TWIN-01 §4.2 B2, §6 #11), so the gain stays 1.
    pub fn record_gpio(&mut self, ev: GpioEvent) {
        if self.gpio.len() < GPIO_LOG_CAP {
            self.gpio.push(ev);
        } else {
            self.gpio_dropped += 1;
        }
    }

    pub fn gpio_log(&self) -> &[GpioEvent] {
        &self.gpio
    }

    /// Events that did not fit in [`GPIO_LOG_CAP`].
    pub fn gpio_dropped(&self) -> u64 {
        self.gpio_dropped
    }

    pub fn take_gpio_log(&mut self) -> Vec<GpioEvent> {
        self.gpio_dropped = 0;
        std::mem::take(&mut self.gpio)
    }
}

/// Copy the shift register (oldest first) into the output latch.
#[inline(always)]
fn capture(latch: &mut [u8], sr: &[u8], head: usize) {
    let n = sr.len() - head;
    latch[..n].copy_from_slice(&sr[head..]);
    latch[n..].copy_from_slice(&sr[..head]);
}

/// Credit `len` lit clocks at address `addr` to every LED whose latched bit is set.
#[inline(always)]
fn flush(acc: &mut [u32], latch: &[u8], width: usize, scan: usize, addr: u16, len: u32) {
    if len == 0 || addr == NO_ADDR {
        return;
    }
    let a = addr as usize;
    let top = &mut acc[a * width * 3..(a + 1) * width * 3];
    for (k, &v) in latch.iter().enumerate() {
        if v & 0x07 != 0 {
            let c = &mut top[k * 3..k * 3 + 3];
            c[0] += len * (v & 1) as u32;
            c[1] += len * ((v >> 1) & 1) as u32;
            c[2] += len * ((v >> 2) & 1) as u32;
        }
    }
    let b = a + scan;
    let bot = &mut acc[b * width * 3..(b + 1) * width * 3];
    for (k, &v) in latch.iter().enumerate() {
        if v & 0x38 != 0 {
            let c = &mut bot[k * 3..k * 3 + 3];
            c[0] += len * ((v >> 3) & 1) as u32;
            c[1] += len * ((v >> 4) & 1) as u32;
            c[2] += len * ((v >> 5) & 1) as u32;
        }
    }
}

/// Decode one pass of a looping chain in steady state: the segments are fed twice (the first
/// pass only loads the shift register and latch, as the previous pass would have on the panel)
/// and the second pass is returned.
pub fn decode_chain(cfg: &DecoderConfig, segments: &[&[u16]]) -> Refresh {
    let mut d = Decoder::new(cfg.clone());
    for s in segments {
        d.feed(s);
    }
    d.end_refresh();
    for s in segments {
        d.feed(s);
    }
    d.end_refresh()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A 4-column, 4-row panel (2 addresses) driven by hand, independent of any library.
    fn tiny() -> DecoderConfig {
        DecoderConfig { geom: PanelGeom { width: 4, height: 4 }, ..Default::default() }
    }

    const LAT: u16 = 1 << 6;
    const OE: u16 = 1 << 7; // 1 = blanked

    #[test]
    fn shift_latch_and_oe_by_hand() {
        let mut d = Decoder::new(tiny());
        // line: k=0 R1, k=1 G2, k=2 nothing, k=3 B1+R2 (carries LAT), all blanked
        d.feed(&[OE | 0b000001, OE | 0b010000, OE, OE | LAT | 0b001100]);
        // 5 lit clocks at address 1, then 2 at address 0, then blanked
        d.feed(&[1 << 8, 1 << 8, 1 << 8, 1 << 8, 1 << 8, 0, 0, OE]);
        let r = d.end_refresh();
        assert_eq!(r.total_clocks, 12);
        assert_eq!(r.lit_clocks, 7);
        assert_eq!(r.latches, 1);
        // address 1 -> rows 1 and 3; address 0 -> rows 0 and 2
        assert_eq!(r.get(0, 1), [5, 0, 0]);
        assert_eq!(r.get(1, 3), [0, 5, 0]);
        assert_eq!(r.get(3, 1), [0, 0, 5]);
        assert_eq!(r.get(3, 3), [5, 0, 0]);
        assert_eq!(r.get(0, 0), [2, 0, 0]);
        assert_eq!(r.get(3, 2), [2, 0, 0]);
        assert_eq!(r.on_clk.iter().filter(|c| **c != [0; 3]).count(), 8);
        // state survives the refresh boundary: the latch still shows the same line
        d.feed(&[0]);
        assert_eq!(d.end_refresh().get(0, 0), [1, 0, 0]);
    }

    #[test]
    fn unwired_address_lines_are_ignored() {
        // 2 addresses: only A is decoded, B..E are not wired on this geometry
        let mut d = Decoder::new(tiny());
        d.feed(&[OE | 1, OE, OE, OE | LAT]);
        d.feed(&[0b11110 << 8]); // A=0 with B..E set -> address 0
        let r = d.end_refresh();
        assert_eq!(r.get(0, 0), [1, 0, 0]);
    }

    #[test]
    fn oe_polarity_parameter() {
        let cfg = DecoderConfig { layout: WordLayout { oe_blank_level: false, ..WordLayout::LIB_3_0_14 }, ..tiny() };
        let mut d = Decoder::new(cfg);
        d.feed(&[1, 0, 0, LAT]); // OE bit 0 = blanked in this polarity
        d.feed(&[OE, OE, OE]);
        assert_eq!(d.end_refresh().get(0, 0), [3, 0, 0]);
    }

    #[test]
    fn gpio_events_are_logged_without_effect() {
        let mut d = Decoder::new(DecoderConfig::default());
        d.record_gpio(GpioEvent { t_ns: 5, pin: crate::gpio::HubPin::Oe, level: true });
        assert_eq!(d.gpio_log().len(), 1);
        let r = d.end_refresh();
        assert_eq!(r.total_clocks, 0);
        assert!(r.on_clk.iter().all(|c| *c == [0; 3]));
        assert_eq!(d.take_gpio_log().len(), 1);
        assert!(d.gpio_log().is_empty());
    }

    #[test]
    #[should_panic(expected = "permutation")]
    fn custom_column_table_must_be_a_permutation() {
        Decoder::new(DecoderConfig { columns: ColumnOrder::Custom(vec![0, 0, 1, 2]), ..tiny() });
    }
}
