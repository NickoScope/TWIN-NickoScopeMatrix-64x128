//! From on-clocks to an image that looks like the panel.
//!
//! Photometry (sourced):
//! - An LED channel's light is proportional to the fraction of the refresh it was driven:
//!   `duty = on_clk / total_clocks`. The firmware already applied its perceptual curve before the
//!   bits reached the DMA buffer (`lumConvTab_8bit`, `cie_luts.h:98-115`, applied in
//!   `updateMatrixDMABuffer`, `ESP32-HUB75-MatrixPanel-I2S-DMA.cpp:408, 433-463`), so no second
//!   gamma is applied on the LED side.
//! - Linear light is encoded for the monitor with the sRGB OETF, CSS Color 4 sample code
//!   `gam_sRGB` (https://drafts.csswg.org/css-color-4/conversions.js lines 31-48, fetched
//!   2026-09-29): `v > 0.0031308 ? 1.055 * v^(1/2.4) - 0.055 : 12.92 * v`.
//! - Pitch 2 mm (KB `docs/01-panel.md`, spec table "Pixel pitch 2 mm").
//!
//! Everything optical beyond that (dot size, GOB diffusion PSF, LED primaries, white balance,
//! black level) has NO source: no measurement exists (ADR-TWIN-01 §4.2 B3, §6 #12). The defaults
//! in [`Optics::default`] are REFERENCE ONLY, UNCALIBRATED, picked to look plausible next to the
//! bring-up photo `photos/2026-09-14-bringup/page-clock.jpg`. By the owner's rule they may become
//! normative only after at least 5 independent measurements.

use crate::decode::Refresh;

/// Optical model of the panel face. All fields except `pitch_mm` are reference-only guesses.
#[derive(Clone, Debug, PartialEq)]
pub struct Optics {
    /// LED pitch (sourced: 2 mm, KB `docs/01-panel.md`).
    pub pitch_mm: f32,
    /// Monitor linear value for an LED that is lit for its entire row slot (duty = 1/scan rows).
    /// A display choice, not a physical claim. At 1.0, full white at brightness 255
    /// (2626 of 94,208 clocks) shows as 0.89 linear.
    pub exposure: f32,
    /// Per-channel emission gain (white balance). REFERENCE ONLY.
    pub channel_gain: [f32; 3],
    /// `primaries[c]` = linear-sRGB colour of LED channel c at unit emission. REFERENCE ONLY
    /// (the default is the sRGB primaries themselves; real LED primaries are unmeasured).
    pub primaries: [[f32; 3]; 3],
    /// Linear sRGB of the dark panel between LEDs under room light. REFERENCE ONLY.
    pub black_background: [f32; 3],
    /// Linear sRGB of an unlit LED package face. REFERENCE ONLY.
    pub black_led: [f32; 3],
    /// Diameter of the visible emitting dot. REFERENCE ONLY.
    pub dot_diameter_mm: f32,
    /// Standard deviation of the gaussian GOB diffusion glow. REFERENCE ONLY.
    pub glow_sigma_mm: f32,
    /// Share of each LED's light that goes into the glow instead of the dot (0..1). REFERENCE ONLY.
    pub glow_fraction: f32,
}

impl Default for Optics {
    fn default() -> Self {
        Optics {
            pitch_mm: 2.0,
            exposure: 1.0,
            channel_gain: [1.0, 1.0, 1.0],
            primaries: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            black_background: [0.0025, 0.0025, 0.0028],
            black_led: [0.018, 0.018, 0.02],
            dot_diameter_mm: 1.1,
            glow_sigma_mm: 0.8,
            glow_fraction: 0.2,
        }
    }
}

/// sRGB OETF, linear -> encoded, CSS Color 4 `gam_sRGB` (conversions.js:31-48).
pub fn srgb_encode(v: f32) -> f32 {
    let a = v.abs();
    if a > 0.003_130_8 {
        v.signum() * (1.055 * a.powf(1.0 / 2.4) - 0.055)
    } else {
        12.92 * v
    }
}

/// RGB888 -> RGB565 with rounding.
pub fn rgb888_to_rgb565(r: u8, g: u8, b: u8) -> u16 {
    let r5 = (r as u32 * 31 + 127) / 255;
    let g6 = (g as u32 * 63 + 127) / 255;
    let b5 = (b as u32 * 31 + 127) / 255;
    ((r5 << 11) | (g6 << 5) | b5) as u16
}

/// An RGB888 image, row-major.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Image {
    pub width: usize,
    pub height: usize,
    pub rgb: Vec<u8>,
}

impl Image {
    pub fn pixel(&self, x: usize, y: usize) -> [u8; 3] {
        let i = (y * self.width + x) * 3;
        [self.rgb[i], self.rgb[i + 1], self.rgb[i + 2]]
    }

    /// For UIs that take RGB565 (the esp32sim board UI: `BoardModel::display`, `esp-soc/src/board.rs:43`).
    pub fn to_rgb565(&self) -> Vec<u16> {
        self.rgb.as_chunks::<3>().0.iter().map(|p| rgb888_to_rgb565(p[0], p[1], p[2])).collect()
    }

    pub fn write_png(&self, path: impl AsRef<std::path::Path>) -> std::io::Result<()> {
        crate::png::write_rgb8(path, &self.rgb, self.width, self.height)
    }
}

const LUT_BITS: usize = 14;
const LUT_MAX: f32 = ((1 << LUT_BITS) - 1) as f32;

/// Renders refreshes with precomputed tables; reuse one per window size.
pub struct Renderer {
    optics: Optics,
    scale: usize,
    /// Coverage of each sub-pixel of a cell by the dot, `scale * scale`.
    disc: Vec<f32>,
    radius: usize,
    /// 1-D glow weights per sub-position: `weights[s * taps + (i + radius)]`, normalized so a
    /// uniform field glows at exactly its own level.
    weights: Vec<f32>,
    lut: Vec<u8>,
    led: Vec<[f32; 3]>,
    hpass: Vec<[f32; 3]>,
}

impl Renderer {
    /// `scale` output pixels per LED (8 -> 1024x512 for the 128x64 panel).
    pub fn new(optics: Optics, scale: usize) -> Renderer {
        assert!(scale >= 1);
        let s = scale as f32;
        let r_dot = 0.5 * optics.dot_diameter_mm / optics.pitch_mm; // in pitch units
        const SS: usize = 8;
        let mut disc = vec![0.0; scale * scale];
        for sy in 0..scale {
            for sx in 0..scale {
                let mut hit = 0;
                for j in 0..SS {
                    for i in 0..SS {
                        let u = (sx as f32 + (i as f32 + 0.5) / SS as f32) / s - 0.5;
                        let v = (sy as f32 + (j as f32 + 0.5) / SS as f32) / s - 0.5;
                        if u * u + v * v <= r_dot * r_dot {
                            hit += 1;
                        }
                    }
                }
                disc[sy * scale + sx] = hit as f32 / (SS * SS) as f32;
            }
        }
        let sigma = optics.glow_sigma_mm / optics.pitch_mm;
        let radius = if sigma > 0.0 { (3.0 * sigma).ceil() as usize } else { 0 };
        let taps = 2 * radius + 1;
        let mut weights = vec![0.0; scale * taps];
        for sx in 0..scale {
            let u = (sx as f32 + 0.5) / s - 0.5; // offset from own LED centre, pitch units
            let row = &mut weights[sx * taps..(sx + 1) * taps];
            for (t, w) in row.iter_mut().enumerate() {
                let d = u - (t as f32 - radius as f32);
                *w = if sigma > 0.0 { (-d * d / (2.0 * sigma * sigma)).exp() } else { 1.0 };
            }
            let sum: f32 = row.iter().sum();
            row.iter_mut().for_each(|w| *w /= sum);
        }
        let lut = (0..=LUT_MAX as usize)
            .map(|i| (srgb_encode(i as f32 / LUT_MAX) * 255.0 + 0.5).clamp(0.0, 255.0) as u8)
            .collect();
        Renderer { optics, scale, disc, radius, weights, lut, led: Vec::new(), hpass: Vec::new() }
    }

    pub fn optics(&self) -> &Optics {
        &self.optics
    }

    pub fn scale(&self) -> usize {
        self.scale
    }

    #[inline]
    fn encode(&self, v: f32) -> u8 {
        self.lut[(v.clamp(0.0, 1.0) * LUT_MAX + 0.5) as usize]
    }

    /// Linear-sRGB light of every LED, row-major. 1.0 = [`Optics::exposure`] reached.
    pub fn led_linear(&mut self, r: &Refresh) -> &[[f32; 3]] {
        let o = &self.optics;
        let scan = (r.height / 2) as f32;
        let k = if r.total_clocks == 0 { 0.0 } else { scan * o.exposure / r.total_clocks as f32 };
        let g = [k * o.channel_gain[0], k * o.channel_gain[1], k * o.channel_gain[2]];
        let p = o.primaries;
        self.led.clear();
        self.led.extend(r.on_clk.iter().map(|c| {
            let e = [c[0] as f32 * g[0], c[1] as f32 * g[1], c[2] as f32 * g[2]];
            [
                e[0] * p[0][0] + e[1] * p[1][0] + e[2] * p[2][0],
                e[0] * p[0][1] + e[1] * p[1][1] + e[2] * p[2][1],
                e[0] * p[0][2] + e[1] * p[1][2] + e[2] * p[2][2],
            ]
        }));
        &self.led
    }

    /// One pixel per LED: the LED's own light, sRGB-encoded (no dot, glow or black level).
    pub fn render_logical(&mut self, r: &Refresh) -> Image {
        self.led_linear(r);
        let mut rgb = Vec::with_capacity(self.led.len() * 3);
        for l in &self.led {
            rgb.extend_from_slice(&[self.encode(l[0]), self.encode(l[1]), self.encode(l[2])]);
        }
        Image { width: r.width, height: r.height, rgb }
    }

    /// `scale` x `scale` pixels per LED: round dots, GOB glow, black levels, sRGB-encoded.
    pub fn render_panel(&mut self, r: &Refresh) -> Image {
        let mut img = Image::default();
        self.render_panel_into(r, &mut img);
        img
    }

    pub fn render_panel_into(&mut self, r: &Refresh, img: &mut Image) {
        let (w, h, s) = (r.width, r.height, self.scale);
        let (ow, oh) = (w * s, h * s);
        self.led_linear(r);
        let gf = self.optics.glow_fraction.clamp(0.0, 1.0);
        let rad = self.radius as isize;
        let taps = 2 * self.radius + 1;
        // horizontal glow pass on the LED grid: hpass[y][px]
        self.hpass.clear();
        self.hpass.resize(h * ow, [0.0; 3]);
        if gf > 0.0 {
            for y in 0..h {
                let src = &self.led[y * w..(y + 1) * w];
                let dst = &mut self.hpass[y * ow..(y + 1) * ow];
                for (px, d) in dst.iter_mut().enumerate() {
                    let (cx, sx) = ((px / s) as isize, px % s);
                    let wt = &self.weights[sx * taps..(sx + 1) * taps];
                    let mut acc = [0.0f32; 3];
                    for (t, &k) in wt.iter().enumerate() {
                        let x = cx + t as isize - rad;
                        if x >= 0 && (x as usize) < w {
                            let l = src[x as usize];
                            acc[0] += l[0] * k;
                            acc[1] += l[1] * k;
                            acc[2] += l[2] * k;
                        }
                    }
                    *d = acc;
                }
            }
        }
        img.width = ow;
        img.height = oh;
        img.rgb.clear();
        img.rgb.resize(ow * oh * 3, 0);
        let (bb, bl) = (self.optics.black_background, self.optics.black_led);
        for py in 0..oh {
            let (cy, sy) = ((py / s) as isize, py % s);
            let wy = &self.weights[sy * taps..(sy + 1) * taps];
            let row_out = &mut img.rgb[py * ow * 3..(py + 1) * ow * 3];
            for px in 0..ow {
                let (cx, sx) = (px / s, px % s);
                let cov = self.disc[sy * s + sx];
                let l = self.led[cy as usize * w + cx];
                let core = (1.0 - gf) * cov;
                let mut v = [
                    l[0] * core + bl[0] * cov + bb[0] * (1.0 - cov),
                    l[1] * core + bl[1] * cov + bb[1] * (1.0 - cov),
                    l[2] * core + bl[2] * cov + bb[2] * (1.0 - cov),
                ];
                if gf > 0.0 {
                    for (t, &wt) in wy.iter().enumerate() {
                        let y = cy + t as isize - rad;
                        if y >= 0 && (y as usize) < h {
                            let g = self.hpass[y as usize * ow + px];
                            let k = gf * wt;
                            v[0] += g[0] * k;
                            v[1] += g[1] * k;
                            v[2] += g[2] * k;
                        }
                    }
                }
                let o = &mut row_out[px * 3..px * 3 + 3];
                o[0] = self.lut[(v[0].clamp(0.0, 1.0) * LUT_MAX + 0.5) as usize];
                o[1] = self.lut[(v[1].clamp(0.0, 1.0) * LUT_MAX + 0.5) as usize];
                o[2] = self.lut[(v[2].clamp(0.0, 1.0) * LUT_MAX + 0.5) as usize];
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn srgb_oetf_matches_css_color_4() {
        assert_eq!(srgb_encode(0.0), 0.0);
        assert!((srgb_encode(1.0) - 1.0).abs() < 1e-6);
        // linear segment and the knee (conversions.js:42-46)
        assert!((srgb_encode(0.002) - 12.92 * 0.002).abs() < 1e-7);
        let k = 0.003_130_8f32;
        assert!((srgb_encode(k) - 12.92 * k).abs() < 1e-6);
        assert!((srgb_encode(0.5) - (1.055 * 0.5f32.powf(1.0 / 2.4) - 0.055)).abs() < 1e-6);
        // 18% grey -> 0.4613 (formula value, computed by hand: 1.055*0.18^(1/2.4)-0.055)
        assert!((srgb_encode(0.18) - 0.461_356).abs() < 1e-4);
    }

    #[test]
    fn rgb565_rounds_and_keeps_extremes() {
        assert_eq!(rgb888_to_rgb565(255, 255, 255), 0xFFFF);
        assert_eq!(rgb888_to_rgb565(0, 0, 0), 0);
        assert_eq!(rgb888_to_rgb565(255, 0, 0), 0xF800);
        assert_eq!(rgb888_to_rgb565(0, 255, 0), 0x07E0);
        assert_eq!(rgb888_to_rgb565(0, 0, 255), 0x001F);
        assert_eq!(rgb888_to_rgb565(128, 128, 128) >> 11, 16); // 128*31/255 = 15.56 -> 16
    }
}
