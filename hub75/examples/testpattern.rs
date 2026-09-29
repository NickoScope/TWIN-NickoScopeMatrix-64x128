//! Render a test pattern the way the twin would: library buffers -> GDMA chain -> decoder ->
//! renderer, and write PNGs to `hub75/out/` for a visual check. Also prints decode and render
//! speed.
//!
//!     cargo run --release -p hub75 --example testpattern
use hub75::refmodel::{DmaChains, LibModel};
use hub75::render::{Optics, Renderer};
use hub75::{Decoder, DecoderConfig, Refresh};
use std::time::Instant;

fn draw(m: &mut LibModel, buf: usize) {
    let mut px = |x: i32, y: i32, c: [u8; 3]| {
        if (0..128).contains(&x) && (0..64).contains(&y) {
            m.draw_pixel(buf, x as usize, y as usize, c[0], c[1], c[2]);
        }
    };
    // colour bars, rows 0-9
    let bars = [[255, 0, 0], [0, 255, 0], [0, 0, 255], [0, 255, 255], [255, 0, 255], [255, 255, 0], [255, 255, 255], [255, 128, 0]];
    for (i, c) in bars.iter().enumerate() {
        for x in 0..16 {
            for y in 0..10 {
                px(i as i32 * 16 + x, y, *c);
            }
        }
    }
    // gradients 0..254 in steps of 2 across the width, rows 12-27 (R, G, B, W)
    for x in 0..128 {
        let v = (x * 2) as u8;
        for (k, c) in [[v, 0, 0], [0, v, 0], [0, 0, v], [v, v, v]].iter().enumerate() {
            for y in 0..4 {
                px(x, 12 + k as i32 * 4 + y, *c);
            }
        }
    }
    // input codes 186..200, 8 columns each, rows 29-34: 193 -> 194 gets darker
    for (i, v) in (186u8..=200).enumerate() {
        for x in 0..8 {
            for y in 29..35 {
                px(4 + i as i32 * 8 + x, y, [v, v, v]);
            }
        }
    }
    // circle and diagonals in the lower half, seam markers, corner dots
    for a in 0..360 {
        let t = (a as f32).to_radians();
        px(32 + (13.0 * t.cos()).round() as i32, 50 + (12.0 * t.sin()).round() as i32, [255, 200, 40]);
    }
    for i in 0..26 {
        px(60 + i, 38 + i, [40, 160, 255]);
        px(100 + i / 2, 38 + i, [255, 60, 120]);
    }
    for y in 36..64 {
        px(63, y, [80, 80, 80]);
        px(64, y, [80, 80, 80]);
    }
    for (x, y) in [(0, 0), (127, 0), (0, 63), (127, 63)] {
        px(x, y, [255, 255, 255]);
    }
}

/// Walk the chains like GDMA until `n` refreshes complete; return the last one.
fn run(m: &LibModel, chains: &mut DmaChains, dec: &mut Decoder, n: usize) -> Refresh {
    let mut last = None;
    let mut got = 0;
    while got < n {
        let (words, eof) = chains.step(m);
        if let Some(r) = dec.feed_segment(words, eof) {
            last = Some(r);
            got += 1;
        }
    }
    last.unwrap()
}

fn upscale(img: &hub75::render::Image, s: usize) -> hub75::render::Image {
    let (w, h) = (img.width * s, img.height * s);
    let mut rgb = Vec::with_capacity(w * h * 3);
    for y in 0..h {
        for x in 0..w {
            rgb.extend_from_slice(&img.pixel(x / s, y / s));
        }
    }
    hub75::render::Image { width: w, height: h, rgb }
}

fn main() -> std::io::Result<()> {
    let out = concat!(env!("CARGO_MANIFEST_DIR"), "/out");
    std::fs::create_dir_all(out)?;
    let mut ren = Renderer::new(Optics::default(), 8);
    for brt in [255u8, 50] {
        let mut m = LibModel::firmware(); // after begin(): chain A shows buffer 0, drawing goes to 1
        m.set_brightness(brt); // initDisplay(): setBrightness8(settings)
        let back = m.back_buffer_id;
        draw(&mut m, back);
        let mut chains = DmaChains::new(&m);
        chains.flip(back); // display() -> flipDMABuffer()
        let mut dec = Decoder::new(DecoderConfig::default());
        let r = run(&m, &mut chains, &mut dec, 3); // blank A, then B steady
        let white = r.get(0, 0);
        println!("brightness {brt}: refresh {} clocks, corner white {:?} on-clocks", r.total_clocks, white);
        let logical = ren.render_logical(&r);
        upscale(&logical, 8).write_png(format!("{out}/testpattern_b{brt}_logical_x8.png"))?;
        logical.write_png(format!("{out}/testpattern_b{brt}_logical.png"))?;
        let t = Instant::now();
        let panel = ren.render_panel(&r);
        let dt = t.elapsed();
        panel.write_png(format!("{out}/testpattern_b{brt}_panel.png"))?;
        println!("  panel {}x{} rendered in {:.2} ms", panel.width, panel.height, dt.as_secs_f64() * 1e3);
        if brt == 255 {
            let n = 600;
            let t = Instant::now();
            let _ = run(&m, &mut chains, &mut dec, n);
            let dt = t.elapsed().as_secs_f64();
            println!(
                "  decode: {n} refreshes of {} words in {:.1} ms = {:.1} us/refresh, {:.0} Mwords/s (need 94,208 x 106/s = 10 Mwords/s)",
                r.total_clocks,
                dt * 1e3,
                dt / n as f64 * 1e6,
                (n as f64 * r.total_clocks as f64) / dt / 1e6
            );
        }
    }
    println!("wrote PNGs to {out}");
    Ok(())
}
