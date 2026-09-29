//! Boards around the ESP32-S3. The SoC model emits generic events (GPIO edges, RMT symbol
//! streams, SPI bytes, LCD_CAM frames, I2S samples); a `BoardModel` interprets them as the
//! devices wired to the pins. One file per board; the device models they share (DCS panels,
//! WS2812 chains, bit-banged SPI) live in `esp_soc::devices`.
//!
//! - `Atech14` — the Atech 14-port motherboard with the Knob, Light Grid and TFT modules
//! - `WaveshareCam` — ESP32-S3-CAM-OV5640
//! - `WaveshareLcd4b` — ESP32-S3-Touch-LCD-4B
//! - `WaveshareAmoled18V2` — ESP32-S3-Touch-AMOLED-1.8 V2
//! - `Panel` — the NickoScope LED panel (`hub75-panel`): Waveshare ESP32-S3-RGB-Matrix + 128x64
//!   HUB75, with its IR receiver, EC11 knob and BOOT (`PanelInputs`)
//! - `NoBoard` — a bare module: nothing on the pins (any ESP32-S3 firmware, console only)
pub use esp_soc::board::{Board, BoardEdge, BoardModel, NoBoard, VirtualCycle};

pub mod atech14;
pub mod panel;
pub mod panel_inputs;
pub mod waveshare_amoled18;
pub mod waveshare_cam;
pub mod waveshare_lcd4b;

pub use atech14::*;
pub use panel::Panel;
pub use waveshare_amoled18::*;
pub use waveshare_cam::*;
pub use waveshare_lcd4b::*;

/// Board by name: `atech14` (default), `none`, or a supported Waveshare board.
pub fn make_board(name: &str) -> Option<Board> {
    match name {
        "atech14" | "atech" => Some(Box::new(Atech14::new())),
        "none" | "bare" => Some(Box::new(NoBoard)),
        "waveshare-cam" | "waveshare" => Some(Box::new(WaveshareCam::new())),
        "waveshare-lcd4b" | "lcd4b" => Some(Box::new(WaveshareLcd4b::new())),
        "waveshare-amoled18-v2" | "amoled18-v2" => Some(Box::new(WaveshareAmoled18V2::new())),
        "hub75-panel" | "panel" | "nickoscope-panel" => Some(Box::new(Panel::new())),
        _ => None,
    }
}
