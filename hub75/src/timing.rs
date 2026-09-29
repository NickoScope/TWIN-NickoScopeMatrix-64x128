//! PCLK and refresh timing.
//!
//! The decoder counts clocks; wall time is clocks / f_PCLK. f_PCLK comes from the LCD_CAM
//! clock register, by ESP32-S3 TRM §29.3.3.1 "LCD Clock" and Register 29.1
//! LCD_CAM_LCD_CLOCK_REG (retrieved 2026-09-29 through the espressif-docs index of
//! documentation.espressif.com/esp32-s3_technical_reference_manual_en.html#page=1080 and #page=1093):
//!
//! ```text
//! f_LCD_CLK  = f_LCD_CLK_S / (N + b/a)     N = DIV_NUM, except DIV_NUM 0 -> 256, 1 -> 2
//! f_LCD_PCLK = f_LCD_CLK / MO              MO = 1 if CLK_EQU_SYSCLK = 1, else CLKCNT_N + 1
//! ```
//! LCD_CLK_SEL: 1 = XTAL_CLK, 2 = PLL_D2_CLK, 3 = PLL_F160M_CLK.
//!
//! What the firmware asks for, from source:
//! - The firmware never sets `i2sspeed` (`matrix_display.h:53-57` sets only driver, clkphase,
//!   double_buff), so it keeps the `HUB75_I2S_CFG` default `HZ_8M = 8000000`
//!   (`ESP32-HUB75-MatrixPanel-I2S-DMA.h:259, :330`). `setupDMA` passes it on as
//!   `bus_cfg.bus_freq` (`.cpp:340`).
//! - The S3 bus does not divide to 8 MHz. `Bus_Parallel16::init` picks `_div_num = 16` for any
//!   `bus_freq <= 10 MHz` (`gdma_lcd_parallel16.cpp:140-142`), with `lcd_clk_sel = 3` (PLL_F160M,
//!   `:100`), `lcd_clk_equ_sysclk = 1` (`:108`), `div_a = 1`, `div_b = 0` (`:161-162`).
//!   Neither `SPIRAM_DMA_BUFFER` (`:121-136`) nor `S3_LCD_DIV_NUM` (`:149-151`) is defined by the
//!   firmware (`platformio.ini` env `matrix-waveshare-rgb`, and a grep of `src/`).
//! - So f_PCLK = 160 MHz / 16 = **10 MHz**. The 8 MHz in the research's Python model is the
//!   *requested* `i2sspeed`, which only feeds the library's refresh-rate estimate
//!   (`.cpp:149-176`, 84 Hz). Derived from source and the TRM, not measured (ADR-TWIN-01 §6 #9).

/// PLL_F160M_CLK, LCD_CLK_SEL = 3 (TRM Register 29.1).
pub const PLL_F160M_HZ: f64 = 160_000_000.0;

/// The LCD_CAM PCLK for the given clock register fields (TRM §29.3.3.1).
pub fn lcd_pclk_hz(src_hz: f64, div_num: u32, div_a: u32, div_b: u32, clk_equ_sysclk: bool, clkcnt_n: u32) -> f64 {
    let n = match div_num {
        0 => 256.0,
        1 => 2.0,
        v => v as f64,
    };
    // "For integer divider, DIV_A and DIV_B are cleared": a = 0 means no fractional part.
    let frac = if div_a == 0 { 0.0 } else { div_b as f64 / div_a as f64 };
    let lcd_clk = src_hz / (n + frac);
    let mo = if clk_equ_sysclk { 1.0 } else { (clkcnt_n + 1) as f64 };
    lcd_clk / mo
}

/// `lcd_clkm_div_num` that library 3.0.14 programs for a requested bus frequency, internal-RAM
/// DMA buffers and no `S3_LCD_DIV_NUM` override (`gdma_lcd_parallel16.cpp:140-153`).
pub fn library_div_num(bus_freq_hz: u32) -> u32 {
    if bus_freq_hz <= 10_000_000 {
        16
    } else if bus_freq_hz < 20_000_000 {
        10
    } else {
        7
    }
}

/// The library's own refresh-rate estimate and `lsbMsbTransitionBit`
/// (`ESP32-HUB75-MatrixPanel-I2S-DMA.cpp:149-176`), integer arithmetic as in C.
/// Returns `(lsb_msb_transition_bit, calculated_refresh_rate)`.
pub fn library_refresh_estimate(i2sspeed_hz: u32, pixels_per_row: u32, depth: u32, rows_per_frame: u32, min_refresh: u32) -> (u32, i64) {
    let mut t = 0u32;
    loop {
        let ps_per_clock = 1_000_000_000_000i64 / i2sspeed_hz as i64;
        let ns_per_latch = (pixels_per_row as i64 * ps_per_clock) / 1000; // CLKS_DURING_LATCH = 0 (.h:88)
        let mut ns_per_row = depth as i64 * ns_per_latch;
        for i in t + 1..depth {
            ns_per_row += (1i64 << (i - t - 1)) * ns_per_latch;
        }
        let ns_per_frame = ns_per_row * rows_per_frame as i64;
        let rate = 1_000_000_000i64 / ns_per_frame;
        if rate >= min_refresh as i64 {
            return (t, rate);
        }
        if t < depth - 1 {
            t += 1;
        } else {
            return (t, rate);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn firmware_pclk_is_10_mhz() {
        // i2sspeed HZ_8M -> div 16; PLL_F160M, a=1, b=0, CLK_EQU_SYSCLK=1, clkcnt_n=1 (gdma_lcd_parallel16.cpp:100-162)
        let div = library_div_num(8_000_000);
        assert_eq!(div, 16);
        assert_eq!(lcd_pclk_hz(PLL_F160M_HZ, div, 1, 0, true, 1), 10_000_000.0);
        // one 94,208-word pass = 9.4208 ms (~106.15 Hz); a 2,944-word row slot = 294.4 us
        let f = lcd_pclk_hz(PLL_F160M_HZ, div, 1, 0, true, 1);
        assert!((94_208.0 / f - 9.4208e-3).abs() < 1e-12);
        assert!((f / 94_208.0 - 106.148).abs() < 1e-3);
        assert!((2_944.0 / f - 294.4e-6).abs() < 1e-12);
    }

    #[test]
    fn trm_divider_edge_cases() {
        assert_eq!(lcd_pclk_hz(160e6, 0, 0, 0, true, 0), 160e6 / 256.0);
        assert_eq!(lcd_pclk_hz(160e6, 1, 0, 0, true, 0), 80e6);
        assert_eq!(lcd_pclk_hz(160e6, 4, 2, 1, true, 0), 160e6 / 4.5);
        assert_eq!(lcd_pclk_hz(160e6, 16, 1, 0, false, 3), 160e6 / 16.0 / 4.0);
    }

    #[test]
    fn library_refresh_estimate_is_84_hz() {
        assert_eq!(library_refresh_estimate(8_000_000, 128, 8, 32, 60), (3, 84));
    }
}
