//! HUB75 lines while they are plain GPIOs: the FM6126A/FM6124 register writes that the library
//! bit-bangs before it hands the pins to LCD_CAM.
//!
//! Source: `ESP32-HUB75-MatrixPanel-leddrivers.cpp:48-105` (`fm6124init`, used for FM6124,
//! FM6126A and ICN2038S, `:24-30`). The firmware selects `FM6126A` (`matrix_display.h:54`).
//! The sequence, with `PIXELS_PER_ROW` = 128:
//! 1. R1..B2, CLK, LAT, OE become outputs driven low, then OE high (`:55-61`);
//! 2. REG1: 128 clocks of `{0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,0}[l % 16]` on all six colour lines,
//!    LAT high for `l > 116` (11 clocks), then LAT low (`:51, :65-76`);
//! 3. REG2: `{0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0}`, LAT high for `l > 115` (12 clocks) (`:52, :79-90`);
//! 4. 128 zero clocks, one clock with LAT high, LAT low, OE low, one more clock (`:93-104`).
//!
//! What REG1/REG2 do to the LED current is not known (no datasheet read; ADR-TWIN-01 §6 #11),
//! so these events are only logged; [`replay_shift_writes`] turns them into readable register
//! loads for that log.

use crate::decode::GpioEvent;

/// A HUB75 signal line.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum HubPin {
    R1,
    G1,
    B1,
    R2,
    G2,
    B2,
    A,
    B,
    C,
    D,
    E,
    Lat,
    Oe,
    Clk,
}

impl HubPin {
    /// Bit of this colour line in the decoder's 6-bit column value (R1 = bit 0 .. B2 = bit 5,
    /// the order of the word bits, `ESP32-HUB75-MatrixPanel-I2S-DMA.h:91-100`).
    pub fn rgb_bit(self) -> Option<u8> {
        match self {
            HubPin::R1 => Some(0),
            HubPin::G1 => Some(1),
            HubPin::B1 => Some(2),
            HubPin::R2 => Some(3),
            HubPin::G2 => Some(4),
            HubPin::B2 => Some(5),
            _ => None,
        }
    }
}

/// Which ESP32-S3 GPIO carries which HUB75 line.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PinMap {
    pub pins: [(u8, HubPin); 14],
}

impl PinMap {
    /// Waveshare ESP32-S3-RGB-Matrix board, as the firmware configures it
    /// (`AnimatedPixelClock-twin/src/display/matrix_display.h:41-45`):
    /// R1=4 G1=5 B1=6 R2=7 G2=15 B2=16 A=18 B=8 C=3 D=42 E=9 LAT=40 OE=2 CLK=41.
    pub const WAVESHARE_S3_RGB_MATRIX: PinMap = PinMap {
        pins: [
            (4, HubPin::R1),
            (5, HubPin::G1),
            (6, HubPin::B1),
            (7, HubPin::R2),
            (15, HubPin::G2),
            (16, HubPin::B2),
            (18, HubPin::A),
            (8, HubPin::B),
            (3, HubPin::C),
            (42, HubPin::D),
            (9, HubPin::E),
            (40, HubPin::Lat),
            (2, HubPin::Oe),
            (41, HubPin::Clk),
        ],
    };

    pub fn pin_for_gpio(&self, gpio: u8) -> Option<HubPin> {
        self.pins.iter().find(|p| p.0 == gpio).map(|p| p.1)
    }

    pub fn gpio_for(&self, pin: HubPin) -> u8 {
        self.pins.iter().find(|p| p.1 == pin).map(|p| p.0).expect("every HUB75 line is mapped")
    }
}

/// One register load reconstructed from bit-banged GPIO: the data clocked in and how many
/// clocks LAT was high when it fell.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ShiftWrite {
    /// Time LAT went low.
    pub t_ns: u64,
    /// CLK rising edges seen while LAT was high (11 = REG1, 12 = REG2 in `fm6124init`).
    pub lat_clocks: u32,
    /// CLK rising edges since the previous write.
    pub clocks: u32,
    /// The last `width` clocked 6-bit column values (R1 = bit 0 .. B2 = bit 5), first clocked first.
    pub columns: Vec<u8>,
    /// OE level when LAT fell (true = blanked).
    pub oe: bool,
}

/// Replay bit-banged events through a shift register clocked on CLK rising edges and report
/// one [`ShiftWrite`] per LAT falling edge. Levels start low, as `fm6124init` drives them.
pub fn replay_shift_writes(events: &[GpioEvent], width: usize) -> Vec<ShiftWrite> {
    let mut rgb = 0u8;
    let (mut clk, mut lat, mut oe) = (false, false, false);
    let mut sr = std::collections::VecDeque::from(vec![0u8; width]);
    let (mut lat_clocks, mut clocks) = (0u32, 0u32);
    let mut out = Vec::new();
    for ev in events {
        if let Some(b) = ev.pin.rgb_bit() {
            rgb = (rgb & !(1 << b)) | ((ev.level as u8) << b);
            continue;
        }
        match ev.pin {
            HubPin::Clk => {
                if ev.level && !clk {
                    sr.pop_front();
                    sr.push_back(rgb);
                    clocks += 1;
                    if lat {
                        lat_clocks += 1;
                    }
                }
                clk = ev.level;
            }
            HubPin::Lat => {
                if lat && !ev.level {
                    out.push(ShiftWrite { t_ns: ev.t_ns, lat_clocks, clocks, columns: sr.iter().copied().collect(), oe });
                    lat_clocks = 0;
                    clocks = 0;
                }
                lat = ev.level;
            }
            HubPin::Oe => oe = ev.level,
            _ => {}
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The event sequence `fm6124init` produces (leddrivers.cpp:48-105), with PIXELS_PER_ROW = 128.
    fn fm6124init_events() -> Vec<GpioEvent> {
        const REG1: [bool; 16] = [false, false, false, false, false, true, true, true, true, true, true, false, false, false, false, false];
        const REG2: [bool; 16] = [false, false, false, false, false, false, false, false, false, true, false, false, false, false, false, false];
        let mut t = 0u64;
        let mut ev = Vec::new();
        let mut set = |pin: HubPin, level: bool, ev: &mut Vec<GpioEvent>| {
            t += 10;
            ev.push(GpioEvent { t_ns: t, pin, level });
        };
        let colours = [HubPin::R1, HubPin::R2, HubPin::G1, HubPin::G2, HubPin::B1, HubPin::B2];
        for p in colours.iter().chain(&[HubPin::Clk, HubPin::Lat, HubPin::Oe]) {
            set(*p, false, &mut ev);
        }
        set(HubPin::Oe, true, &mut ev);
        for (reg, lat_after) in [(REG1, 116), (REG2, 115)] {
            for l in 0..128 {
                for p in colours {
                    set(p, reg[l % 16], &mut ev);
                }
                if l > lat_after {
                    set(HubPin::Lat, true, &mut ev);
                }
                set(HubPin::Clk, true, &mut ev);
                set(HubPin::Clk, false, &mut ev);
            }
            set(HubPin::Lat, false, &mut ev);
        }
        for p in colours {
            set(p, false, &mut ev);
        }
        for _ in 0..128 {
            set(HubPin::Clk, true, &mut ev);
            set(HubPin::Clk, false, &mut ev);
        }
        set(HubPin::Lat, true, &mut ev);
        set(HubPin::Clk, true, &mut ev);
        set(HubPin::Clk, false, &mut ev);
        set(HubPin::Lat, false, &mut ev);
        set(HubPin::Oe, false, &mut ev);
        set(HubPin::Clk, true, &mut ev);
        set(HubPin::Clk, false, &mut ev);
        ev
    }

    #[test]
    fn fm6124init_replays_as_reg1_reg2_and_a_blank_line() {
        let w = replay_shift_writes(&fm6124init_events(), 128);
        assert_eq!(w.len(), 3);
        assert_eq!((w[0].lat_clocks, w[0].clocks, w[0].oe), (11, 128, true));
        assert_eq!((w[1].lat_clocks, w[1].clocks, w[1].oe), (12, 128, true));
        assert_eq!((w[2].lat_clocks, w[2].clocks), (1, 129));
        // REG1 bits 5..10 of each 16-column group set on all six lines, REG2 bit 9 only
        assert_eq!(w[0].columns[5], 0x3f);
        assert_eq!(w[0].columns[4], 0);
        assert_eq!(w[0].columns[16 + 10], 0x3f);
        assert_eq!(w[0].columns[16 + 11], 0);
        assert_eq!(w[1].columns.iter().filter(|c| **c == 0x3f).count(), 8);
        assert_eq!(w[1].columns[9], 0x3f);
        assert!(w[2].columns.iter().all(|c| *c == 0));
    }

    #[test]
    fn waveshare_pin_map() {
        let m = PinMap::WAVESHARE_S3_RGB_MATRIX;
        assert_eq!(m.pin_for_gpio(41), Some(HubPin::Clk));
        assert_eq!(m.pin_for_gpio(2), Some(HubPin::Oe));
        assert_eq!(m.gpio_for(HubPin::E), 9);
        assert_eq!(m.pin_for_gpio(1), None);
    }
}
