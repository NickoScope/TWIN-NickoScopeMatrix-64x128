//! HUB75 panel twin for esp32sim: what the LEDs of a 128x64 HUB75 panel show when the real
//! ESP32-S3 firmware drives it with ESP32-HUB75-MatrixPanel-DMA 3.0.14 (ADR-TWIN-01 §4.2 B2, B3).
//!
//! - [`decode`]: a generic HUB75 state machine over the 16-bit words LCD_CAM clocks out
//!   (shift, latch, OE-gated accumulation). It does not know the library's buffer layout, so it
//!   reproduces the library's quirks (non-binary BCM weights, previous-row addressing, tearing
//!   at a buffer flip) and survives a library change.
//! - [`render`]: on-clocks -> linear light -> sRGB image, as a plain 128x64 image or a scaled
//!   picture of the panel face (dots, GOB glow; optical parameters are reference-only).
//! - [`gpio`]: the FM6126A init that the library bit-bangs before DMA starts (logged only).
//! - [`timing`]: PCLK from the LCD_CAM clock register (10 MHz for this firmware).
//! - [`refmodel`]: the library's buffer build and descriptor chains, for tests and demos only.
//!
//! How an emulator drives it (interfaces I1/I2/I4 of ADR-TWIN-01 §4.3):
//! ```
//! use hub75::{Decoder, DecoderConfig, render::{Optics, Renderer}};
//! let mut dec = Decoder::new(DecoderConfig::default());
//! let mut ren = Renderer::new(Optics::default(), 8);
//! // for each GDMA OUT descriptor the LCD_CAM finishes clocking (words = its buffer as u16 LE):
//! let words: Vec<u16> = vec![0x0080; 128]; // OE high: dark
//! if let Some(refresh) = dec.feed_segment(&words, /* suc_eof */ true) {
//!     let _seconds = refresh.duration_s(hub75::timing::lcd_pclk_hz(160e6, 16, 1, 0, true, 1));
//!     let img = ren.render_panel(&refresh); // 1024x512 RGB888
//!     let _for_ui = img.to_rgb565();
//! }
//! ```

pub mod decode;
pub mod gpio;
pub mod png;
pub mod refmodel;
pub mod render;
pub mod timing;

pub use decode::{decode_chain, ColumnOrder, Decoder, DecoderConfig, GpioEvent, LatchMode, PanelGeom, Refresh, WordLayout};
