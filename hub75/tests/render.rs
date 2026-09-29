//! Renderer: photometry (sourced) and the look (reference-only parameters).

use hub75::refmodel::LibModel;
use hub75::render::{srgb_encode, Optics, Renderer};
use hub75::{decode_chain, DecoderConfig, Refresh};

fn refresh_of(brt: u8, paint: impl Fn(&mut LibModel)) -> Refresh {
    let mut m = LibModel::firmware();
    m.set_brightness(brt);
    paint(&mut m);
    decode_chain(&DecoderConfig::default(), &m.segments(0))
}

fn enc(v: f32) -> i32 {
    (srgb_encode(v) * 255.0).round() as i32
}

fn close(a: u8, b: i32, tol: i32) -> bool {
    (a as i32 - b).abs() <= tol
}

#[test]
fn logical_image_is_duty_times_scan_rows_then_srgb() {
    let r = refresh_of(255, |m| m.fill(0, 255, 255, 255));
    let mut ren = Renderer::new(Optics::default(), 8);
    let img = ren.render_logical(&r);
    assert_eq!((img.width, img.height), (128, 64));
    // 2626 of 94,208 clocks, x 32 rows per scan = 0.892 linear
    let want = enc(2626.0 / 94208.0 * 32.0);
    assert_eq!(want, 242);
    for p in [img.pixel(0, 0), img.pixel(127, 63), img.pixel(64, 31)] {
        assert!(p.iter().all(|c| close(*c, want, 1)), "{p:?} vs {want}");
    }
    // brightness 50: 500 clocks
    let r = refresh_of(50, |m| m.fill(0, 255, 255, 255));
    let p = ren.render_logical(&r).pixel(3, 3);
    assert!(close(p[0], enc(500.0 / 94208.0 * 32.0), 1), "{p:?}");
}

#[test]
fn the_193_194_dip_reaches_the_image() {
    let r = refresh_of(255, |m| {
        m.draw_pixel(0, 0, 0, 193, 193, 193);
        m.draw_pixel(0, 1, 0, 194, 194, 194);
    });
    let mut ren = Renderer::new(Optics::default(), 8);
    let img = ren.render_logical(&r);
    assert!(img.pixel(0, 0)[0] > img.pixel(1, 0)[0] + 15, "{:?} {:?}", img.pixel(0, 0), img.pixel(1, 0));
}

#[test]
fn panel_image_size_dot_and_glow_reach() {
    let r = refresh_of(255, |m| m.draw_pixel(0, 40, 20, 255, 255, 255));
    let o = Optics::default();
    let mut ren = Renderer::new(o.clone(), 8);
    let logical = ren.render_logical(&r).pixel(40, 20);
    let img = ren.render_panel(&r);
    assert_eq!((img.width, img.height), (1024, 512));
    // dot centre of the lit LED is close to its logical value (core + own glow)
    let c = img.pixel(40 * 8 + 4, 20 * 8 + 4);
    assert!((c[0] as i32 - logical[0] as i32).abs() <= 12, "{c:?} vs {logical:?}");
    // glow reaches ceil(3 sigma) = 2 LEDs; 3 LEDs away there is only the unlit panel
    let bg = enc(o.black_background[0]);
    let off_led = enc(o.black_led[0]);
    let far_gap = img.pixel(43 * 8, 20 * 8); // corner of a cell: background only
    let far_dot = img.pixel(43 * 8 + 4, 20 * 8 + 4);
    assert!(close(far_gap[0], bg, 1), "{far_gap:?} vs {bg}");
    assert!(close(far_dot[0], off_led, 1), "{far_dot:?} vs {off_led}");
    // the neighbour's dot picks up some glow, but far less than the lit dot
    let near = img.pixel(41 * 8 + 4, 20 * 8 + 4);
    assert!(near[0] as i32 > off_led + 2 && (near[0] as i32) < c[0] as i32 / 2, "{near:?} vs unlit {off_led}");
}

#[test]
fn uniform_field_glow_floor_is_glow_fraction() {
    let r = refresh_of(255, |m| m.fill(0, 255, 255, 255));
    let o = Optics::default();
    let mut ren = Renderer::new(o.clone(), 8);
    let img = ren.render_panel(&r);
    let l = 2626.0 / 94208.0 * 32.0;
    // between dots, away from the edge: glow of a uniform field = glow_fraction * L
    let gap = img.pixel(60 * 8, 30 * 8);
    assert!(close(gap[0], enc(o.glow_fraction * l + o.black_background[0]), 2), "{gap:?}");
    // dot centre: (1 - g) * L + g * L (+ unlit-face level inside the dot) -> clipped near 1
    let dot = img.pixel(60 * 8 + 4, 30 * 8 + 4);
    assert!(close(dot[0], enc((l + o.black_led[0]).min(1.0)), 2), "{dot:?}");
}

#[test]
fn dark_and_empty_refreshes_render_black_levels() {
    let mut ren = Renderer::new(Optics::default(), 4);
    let empty = Refresh::blank(128, 64); // total_clocks 0
    let img = ren.render_panel(&empty);
    assert_eq!(img.rgb.len(), 512 * 256 * 3);
    assert!(img.rgb.iter().all(|&c| c < 40));
    assert_eq!(img.to_rgb565().len(), 512 * 256);
}

#[test]
fn white_balance_and_primaries_are_parameters() {
    let r = refresh_of(255, |m| m.draw_pixel(0, 5, 5, 255, 0, 0));
    let o = Optics {
        channel_gain: [0.5, 1.0, 1.0],
        primaries: [[1.0, 0.1, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        ..Optics::default()
    };
    let mut ren = Renderer::new(o, 8);
    let p = ren.render_logical(&r).pixel(5, 5);
    let l = 0.5 * 2626.0 / 94208.0 * 32.0;
    assert!(close(p[0], enc(l), 1) && close(p[1], enc(0.1 * l), 1) && p[2] == 0, "{p:?}");
}
