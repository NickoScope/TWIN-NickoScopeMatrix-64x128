//! Reference model of what ESP32-HUB75-MatrixPanel-DMA 3.0.14 puts in its DMA buffers on the
//! ESP32-S3, for tests and demos. The decoder does not use it: it only turns a picture into the
//! word stream the real library would emit, so the decoder can be checked against the Python
//! oracle (`AnimatedPixelClock-twin/tools/twin/research/model.py`) and against the library source.
//!
//! Every function names the library lines it mirrors (paths relative to the library `src/`
//! directory that the firmware compiles, `.pio/libdeps/matrix-waveshare-rgb/...`). Only the
//! default line decoder (TYPE138) and driver-independent latch (one LAT word) are modelled.

/// `lumConvTab_8bit` (`cie_luts.h:98-115`, the table the firmware links: `cie_luts.h:168-183`
/// selects it when `PIXEL_COLOR_DEPTH_BITS` is 8, the default `.h:80`).
pub const LUM_CONV_TAB_8BIT: [u8; 256] = [
    0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4, //
    4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8, 9, 9, 9, 10, 10, 10, 10, 11, 11, //
    11, 12, 12, 12, 13, 13, 13, 14, 14, 15, 15, 15, 16, 16, 17, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 23, 24, 24, 25, //
    25, 26, 26, 27, 28, 28, 29, 29, 30, 31, 31, 32, 32, 33, 34, 34, 35, 36, 37, 37, 38, 39, 39, 40, 41, 42, 43, 43, 44, 45, 46, 47, //
    47, 48, 49, 50, 51, 52, 53, 54, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 70, 71, 72, 73, 74, 75, 76, 77, 79, //
    80, 81, 82, 83, 85, 86, 87, 88, 90, 91, 92, 94, 95, 96, 98, 99, 100, 102, 103, 105, 106, 108, 109, 110, 112, 113, 115, 116, 118, 120, 121, 123, //
    124, 126, 128, 129, 131, 132, 134, 136, 138, 139, 141, 143, 145, 146, 148, 150, 152, 154, 155, 157, 159, 161, 163, 165, 167, 169, 171, 173, 175, 177, 179, 181, //
    183, 185, 187, 189, 191, 193, 196, 198, 200, 202, 204, 207, 209, 211, 214, 216, 218, 220, 223, 225, 228, 230, 232, 235, 237, 240, 242, 245, 247, 250, 252, 255,
];

const BIT_LAT: u16 = 1 << 6; // .h:103
const BIT_OE: u16 = 1 << 7; // .h:104
const BITS_ADDR_OFFSET: u32 = 8; // .h:107
const BITS_RGB2_OFFSET: u32 = 3; // .h:97
/// `DMA_MAX (4096-4)` bytes per descriptor (`platforms/esp32s3/gdma_lcd_parallel16.hpp:84`).
const DMA_MAX_BYTES: usize = 4092;

/// One DMA descriptor of a chain pass: `len` words starting at word `offset` of a frame buffer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Desc {
    pub offset: usize,
    pub len: usize,
    pub suc_eof: bool,
}

/// The library's two frame buffers and its settings.
#[derive(Clone, Debug)]
pub struct LibModel {
    /// PIXELS_PER_ROW = mx_width * chain_length (`.h:887`).
    pub width: usize,
    /// ROWS_PER_FRAME = mx_height / 2.
    pub rows_per_frame: usize,
    pub depth: usize,
    pub latch_blanking: usize,
    pub lsb_msb_transition_bit: usize,
    pub calculated_refresh_rate: i64,
    pub brightness: u8,
    /// Buffer being drawn into (`fb`); the other one is on screen.
    pub back_buffer_id: usize,
    /// Each buffer: `[row][plane][x]` words, row blocks of `depth * width` (`.h:165-178`).
    bufs: [Vec<u16>; 2],
}

impl LibModel {
    /// A library instance with the given config, after `setupDMA` and `resetbuffers()`.
    pub fn new(width: usize, height: usize, depth: usize, i2sspeed_hz: u32, latch_blanking: usize, min_refresh: u32) -> LibModel {
        let rows = height / 2;
        let (t, rate) = crate::timing::library_refresh_estimate(i2sspeed_hz, width as u32, depth as u32, rows as u32, min_refresh);
        let mut m = LibModel {
            width,
            rows_per_frame: rows,
            depth,
            latch_blanking,
            lsb_msb_transition_bit: t as usize,
            calculated_refresh_rate: rate,
            brightness: 128, // `int brightness = 128;` (.h:879)
            back_buffer_id: 0,
            bufs: [vec![0; rows * depth * width], vec![0; rows * depth * width]],
        };
        m.reset_buffers();
        m
    }

    /// The firmware's config: 64x64 x 2, depth 8 (`.h:80`), `HZ_8M` (`.h:259,330`),
    /// `DEFAULT_LAT_BLANKING 2` (`.h:122`), `min_refresh_rate 60` (`.h:333`); `matrix_display.h:53-57`
    /// changes none of these. State after `begin()` (`.h:475-485`): buffers reset at brightness
    /// 128, then `flipDMABuffer()` so chain A shows buffer 0 and drawing goes to buffer 1.
    pub fn firmware() -> LibModel {
        let mut m = LibModel::new(128, 64, 8, 8_000_000, 2, 60);
        m.back_buffer_id = 1;
        m
    }

    pub fn buffer(&self, buf: usize) -> &[u16] {
        &self.bufs[buf]
    }

    /// Words of plane `plane` of row block `row` (`getDataPtr`, `.h:178`).
    pub fn plane_mut(&mut self, buf: usize, row: usize, plane: usize) -> &mut [u16] {
        let (w, d) = (self.width, self.depth);
        &mut self.bufs[buf][(row * d + plane) * w..(row * d + plane + 1) * w]
    }

    /// `resetbuffers()` (`.h:736-745`): clearFrameBuffer then setBrightnessOE, both buffers.
    pub fn reset_buffers(&mut self) {
        for b in 0..2 {
            self.clear_frame_buffer(b);
            self.set_brightness_oe(self.brightness, b);
        }
    }

    /// `clearFrameBuffer` for TYPE138 (`.cpp:532-669`).
    pub fn clear_frame_buffer(&mut self, buf: usize) {
        let (w, d, rpf, blank) = (self.width, self.depth, self.rows_per_frame, self.latch_blanking);
        for row in 0..rpf {
            let block = &mut self.bufs[buf][row * d * w..(row + 1) * d * w];
            // planes 1..depth-1 carry this row's address (.cpp:546-570)
            for v in block[w..].iter_mut() {
                *v = (row as u16) << BITS_ADDR_OFFSET;
            }
            // plane 0 carries the previous row's address (.cpp:572-600)
            let prev = if row == 0 { rpf - 1 } else { row - 1 } as u16;
            for v in block[..w].iter_mut() {
                *v = prev << BITS_ADDR_OFFSET;
            }
            for p in 0..d {
                let pl = &mut block[p * w..(p + 1) * w];
                pl[w - 1] |= BIT_LAT; // .cpp:645
                let mut b = blank; // .cpp:651-660
                while b > 0 {
                    b -= 1;
                    pl[b] |= BIT_OE;
                    pl[w - 1] |= BIT_OE;
                    pl[w - b - 1] |= BIT_OE;
                }
            }
        }
    }

    /// `setBrightnessOE(brt, buf)` (`.cpp:671-751`).
    pub fn set_brightness_oe(&mut self, brt: u8, buf: usize) {
        let (w, d, rpf, blank, t) = (self.width, self.depth, self.rows_per_frame, self.latch_blanking, self.lsb_msb_transition_bit);
        for row in 0..rpf {
            for c in 0..d {
                let bitplane = ((2 * d - c) % d) as i32; // .cpp:695
                let bitshift = ((d - t - 1) >> 1) as i32; // .cpp:696
                let rightshift = (bitplane - bitshift - 2).max(0); // .cpp:698
                let max_px = ((w - blank) >> rightshift) as i32; // .cpp:704
                let mut n = (max_px * brt as i32) >> 8; // .cpp:705
                if brt > 0 && n == 0 {
                    n = 1; // .cpp:708-710
                }
                if n > max_px - 1 {
                    n = max_px - 1; // .cpp:715-717
                }
                let xmax = (w as i32 + n + 1) >> 1; // .cpp:723
                let xmin = (w as i32 - n) >> 1; // .cpp:724
                let pl = &mut self.bufs[buf][(row * d + c) * w..(row * d + c + 1) * w];
                for (x, v) in pl.iter_mut().enumerate() {
                    let x = x as i32;
                    if x >= xmin && x < xmax {
                        *v &= !BIT_OE;
                    } else {
                        *v |= BIT_OE;
                    }
                }
            }
        }
    }

    /// `setBrightness(b)` (`.h:639-654`): rewrites OE in both buffers.
    pub fn set_brightness(&mut self, brt: u8) {
        self.brightness = brt;
        self.set_brightness_oe(brt, 0);
        self.set_brightness_oe(brt, 1);
    }

    /// `updateMatrixDMABuffer(x, y, r, g, b)` (`.cpp:390-464`) into buffer `buf`.
    pub fn draw_pixel(&mut self, buf: usize, x: usize, y: usize, r: u8, g: u8, b: u8) {
        if x >= self.width || y >= 2 * self.rows_per_frame {
            return;
        }
        let (rv, gv, bv) = (LUM_CONV_TAB_8BIT[r as usize], LUM_CONV_TAB_8BIT[g as usize], LUM_CONV_TAB_8BIT[b as usize]);
        let (mut y, mut off, mut clear) = (y, 0u32, 0b1111_1111_1111_1000u16);
        if y >= self.rows_per_frame {
            off = BITS_RGB2_OFFSET;
            clear = 0b1111_1111_1100_0111;
            y -= self.rows_per_frame;
        }
        for p in 0..self.depth {
            let m = 1u8 << p;
            let bits = (((bv & m) != 0) as u16) << 2 | (((gv & m) != 0) as u16) << 1 | ((rv & m) != 0) as u16;
            let pl = self.plane_mut(buf, y, p);
            pl[x] = (pl[x] & clear) | (bits << off);
        }
    }

    /// `updateMatrixDMABuffer(r, g, b)` (`.cpp:468-522`): the whole buffer one colour.
    pub fn fill(&mut self, buf: usize, r: u8, g: u8, b: u8) {
        let (rv, gv, bv) = (LUM_CONV_TAB_8BIT[r as usize], LUM_CONV_TAB_8BIT[g as usize], LUM_CONV_TAB_8BIT[b as usize]);
        for p in 0..self.depth {
            let m = 1u8 << p;
            let mut bits = (((bv & m) != 0) as u16) << 2 | (((gv & m) != 0) as u16) << 1 | ((rv & m) != 0) as u16;
            bits |= bits << BITS_RGB2_OFFSET;
            for row in 0..self.rows_per_frame {
                for v in self.plane_mut(buf, row, p).iter_mut() {
                    *v = (*v & 0b1111_1111_1100_0000) | bits;
                }
            }
        }
    }

    /// Plane of each 128-word segment within one row: all planes once, then plane i repeated
    /// `2^(i-t-1)` times for i > t (`.cpp:252-291`).
    pub fn segment_planes(&self) -> Vec<usize> {
        let t = self.lsb_msb_transition_bit;
        let mut s: Vec<usize> = (0..self.depth).collect();
        for i in t + 1..self.depth {
            s.extend(std::iter::repeat_n(i, 1 << (i - t - 1)));
        }
        s
    }

    /// The descriptors of one chain pass, in order (`setupDMA` Step 2 and 4, `.cpp:194-298`;
    /// `create_dma_desc_link`, `gdma_lcd_parallel16.cpp:353-410`: `suc_eof` on the last one only).
    pub fn descriptors(&self) -> Vec<Desc> {
        let (w, d, t) = (self.width, self.depth, self.lsb_msb_transition_bit);
        let one = w * 2; // getColorDepthSize(true) bytes
        let all = w * d * 2; // getColorDepthSize(false) bytes
        let n_one = one.div_ceil(DMA_MAX_BYTES);
        let last_one = one % DMA_MAX_BYTES;
        let n_all = all.div_ceil(DMA_MAX_BYTES);
        let last_all = all % DMA_MAX_BYTES;
        let step = DMA_MAX_BYTES / 2; // words
        let mut v = Vec::new();
        for row in 0..self.rows_per_frame {
            let base = row * d * w;
            for k in 0..n_all {
                let bytes = if k == n_all - 1 { last_all } else { DMA_MAX_BYTES };
                v.push(Desc { offset: base + k * step, len: bytes / 2, suc_eof: false });
            }
            for i in t + 1..d {
                for _ in 0..(1usize << (i - t - 1)) {
                    for k in 0..n_one {
                        let bytes = if k == n_one - 1 { last_one } else { DMA_MAX_BYTES };
                        v.push(Desc { offset: base + i * w + k * step, len: bytes / 2, suc_eof: false });
                    }
                }
            }
        }
        if let Some(l) = v.last_mut() {
            l.suc_eof = true;
        }
        v
    }

    /// Words of each descriptor of one pass over buffer `buf`, in chain order.
    pub fn segments(&self, buf: usize) -> Vec<&[u16]> {
        self.descriptors().iter().map(|d| &self.bufs[buf][d.offset..d.offset + d.len]).collect()
    }

    /// One chain pass over buffer `buf` as a flat word stream.
    pub fn stream(&self, buf: usize) -> Vec<u16> {
        self.segments(buf).concat()
    }
}

/// The two descriptor chains in "guest memory", walked the way GDMA walks them: follow `next`
/// after every descriptor, re-reading it each time, so a flip lands at the chain wrap
/// (`flip_dma_output_buffer`, `gdma_lcd_parallel16.cpp:431-449`; ADR-TWIN-01 §4.2 B1).
#[derive(Clone, Debug)]
pub struct DmaChains {
    /// Descriptors of chain A (`0..count`, buffer 0) then chain B (`count..2*count`, buffer 1).
    pub descs: Vec<(usize, Desc)>,
    pub next: Vec<usize>,
    pub count: usize,
    /// Descriptor the GDMA reads next.
    pub cur: usize,
}

impl DmaChains {
    /// Chains as `setupDMA` links them (each loops to its own head), then `begin()`'s
    /// `flipDMABuffer()` with back_buffer_id 0, and GDMA started at `_dmadesc_a[0]`
    /// (`.h:475-485`, `gdma_lcd_parallel16.cpp:412-418`).
    pub fn new(m: &LibModel) -> DmaChains {
        let d = m.descriptors();
        let count = d.len();
        let mut descs = Vec::with_capacity(2 * count);
        let mut next = Vec::with_capacity(2 * count);
        for (buf, base) in [(0usize, 0usize), (1, count)] {
            for (i, x) in d.iter().enumerate() {
                descs.push((buf, *x));
                next.push(if i == count - 1 { base } else { base + i + 1 });
            }
        }
        let mut c = DmaChains { descs, next, count, cur: 0 };
        c.flip(0);
        c
    }

    /// `flip_dma_output_buffer(back_buffer_id)`: re-point only the last descriptor of each chain.
    pub fn flip(&mut self, back_buffer_id: usize) {
        let (a_last, b_last, a0, b0) = (self.count - 1, 2 * self.count - 1, 0, self.count);
        if back_buffer_id == 1 {
            self.next[b_last] = b0;
            self.next[a_last] = b0;
        } else {
            self.next[a_last] = a0;
            self.next[b_last] = a0;
        }
    }

    /// Emit the current descriptor (buffer, words range, suc_eof) and advance along `next`.
    pub fn step<'a>(&mut self, m: &'a LibModel) -> (&'a [u16], bool) {
        let (buf, d) = self.descs[self.cur];
        self.cur = self.next[self.cur];
        (&m.buffer(buf)[d.offset..d.offset + d.len], d.suc_eof)
    }
}
