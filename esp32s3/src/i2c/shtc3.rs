//! Sensirion SHTC3 humidity and temperature sensor, as the master sees it on the I2C bus.
//!
//! Source for every number: Sensirion, *Datasheet SHTC3*, Version 4 - December 2022 (DS),
//! https://sensirion.com/media/documents/643F9C8E/63A5A436/Datasheet_SHTC3.pdf. Each line names its
//! table or section. What the datasheet leaves open is marked "not verified" and says what we chose.
//!
//! States (DS §5.2, §5.4, §5.5, Table 3):
//! - **idle** after power-up (DS §5.2; Table 3: "after power-up the sensor remains in the idle state
//!   unless a sleep command is issued"). A fresh model is a sensor just powered: idle.
//! - **sleep** after 0xB098 (Table 9). Only the wake-up 0x3517 (Table 10) is taken: "when in sleep mode,
//!   the sensor requires a dedicated wake-up command to enable further I2C communication" (Table 3).
//!   The write header is ACKed, since the wake-up has to get in; any other command byte and every read
//!   header are NACKed. Not verified: which byte a sleeping part refuses (the DS does not say); we NACK
//!   the first byte that cannot be the wake-up.
//! - **waking / resetting** for tPU / tSR after the wake-up or the soft reset (Table 5, max 240 us;
//!   Figure 7 points the wake-up time to Table 5). Not verified: the DS gives only the time before the
//!   sensor "enters the idle state"; we NACK headers until then.
//! - **measuring** for tMEAS after a measurement command (Table 5, max 12.1 ms normal, 0.8 ms low
//!   power). "The sensor does not respond to any I2C activity during measurement, i.e. I2C read and
//!   write headers are not acknowledged (NACK)" (§5.5). With clock stretching (Table 11) a read header
//!   is ACKed and SCL held low until the result is ready (§5.5). The controller model has no SCL, so a
//!   stretched read gets its bytes at once, as if the stretch had just ended: firmware using stretching
//!   sees no delay here (a real master waits up to tMEAS and needs a bus timeout that allows it).
//! - **result ready**: a read header is ACKed and the six bytes follow, T first or RH first as the
//!   command said (Table 11, §5.6, Figure 7); each word is two bytes and a CRC (§5.6, §5.10). The
//!   master may stop after any byte (§5.6). The sensor is then idle (Figure 7, "SHTC3 in idle
//!   state"). Not verified: what a second read header gets; the DS does not say, we NACK it (the
//!   result is read once), and past the six bytes the bus reads 0xFF.
//! - Also not verified: a wake-up sent to an awake sensor (ACKed, nothing happens: every cycle in
//!   Figure 7 starts with it, and the firmware sends it on its first contact, when the part is idle
//!   from power-up), an unknown command (NACK at its second byte), a third command byte (NACK).
//!
//! The ID (0xEFC8, Table 14) is a 16-bit word and its CRC (§5.9). Table 15 fixes bits 11 and 5:0 to
//! the SHTC3 code (xxxx'1xxx'xx00'0111); the x bits vary from part to part, ours are 0 (our choice).
//!
//! Timing uses the **max** column of Table 5 (measured at -40 °C, §3.1), the worst case a firmware must
//! survive. The panel firmware does not depend on anything shorter: it waits 1 ms after the wake-up and
//! the soft reset and 15 ms after its measurement command (0x7866, normal mode, T first, no clock
//! stretching, `src/climate/shtc3.h:26`), and a NACK on the read makes it retry 5 ms later, three
//! times (AnimatedPixelClock-twin f2cb9b2, `src/climate/climate_reader.h:51-54, :195-206`).
//!
//! Time: the device reads the board's cycle from `clock` (`VirtualCycle`, `periph::CPU_HZ` per
//! second). The bus flushes device time before every peripheral register write (`bus.rs`
//! `periph_write_inner`), and the board stores the cycle in `advance_to`, so the write that starts an
//! I2C transaction sees the exact cycle.
//!
//! Not modelled: the reset through the general call (DS §5.8, address 0x00, a bus-wide reset the panel
//! firmware does not send), and self-heating: the model reads exactly the air it is given.
use super::I2cDevice;
use crate::periph::CPU_HZ;
use std::sync::atomic::{AtomicU64, Ordering::Relaxed};
use std::sync::{Arc, Mutex};

/// DS §5.1 Table 8.
pub const SHTC3_ADDR: u8 = 0x70;
/// DS Table 10.
pub const CMD_WAKEUP: u16 = 0x3517;
/// DS Table 9.
pub const CMD_SLEEP: u16 = 0xB098;
/// DS §5.7 Table 12.
pub const CMD_SOFT_RESET: u16 = 0x805D;
/// DS §5.9 Table 14.
pub const CMD_READ_ID: u16 = 0xEFC8;
/// The ID this model returns: Table 15's fixed bits with the free ones at 0 (our choice).
pub const SHTC3_ID: u16 = 0x0807;

const CYCLES_PER_US: u64 = CPU_HZ / 1_000_000;
/// DS Table 5, max: tPU and tSR 240 us, tMEAS 12.1 ms normal and 0.8 ms low power.
pub const T_WAKE_US: u64 = 240;
pub const T_RESET_US: u64 = 240;
pub const T_MEAS_NORMAL_US: u64 = 12_100;
pub const T_MEAS_LOW_POWER_US: u64 = 800;

/// A measurement command of DS Table 11.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Measure { pub low_power: bool, pub stretch: bool, pub rh_first: bool }

/// DS §5.3 Table 11: the eight measurement commands.
pub fn measure_command(cmd: u16) -> Option<Measure> {
    let m = |low_power, stretch, rh_first| Some(Measure { low_power, stretch, rh_first });
    match cmd {
        0x7CA2 => m(false, true, false),
        0x5C24 => m(false, true, true),
        0x7866 => m(false, false, false),
        0x58E0 => m(false, false, true),
        0x6458 => m(true, true, false),
        0x44DE => m(true, true, true),
        0x609C => m(true, false, false),
        0x401A => m(true, false, true),
        _ => None,
    }
}

/// DS §5.10 Table 16: CRC-8, polynomial 0x31, init 0xFF, no reflection, final XOR 0x00.
pub fn crc8(data: &[u8]) -> u8 {
    let mut crc = 0xffu8;
    for &b in data {
        crc ^= b;
        for _ in 0..8 { crc = if crc & 0x80 != 0 { (crc << 1) ^ 0x31 } else { crc << 1 }; }
    }
    crc
}

/// DS §5.11 inverted: T = -45 + 175 * S_T / 2^16, so S_T = (T + 45) * 2^16 / 175, rounded.
pub fn raw_temperature(celsius: f64) -> u16 { ((celsius + 45.0) * 65536.0 / 175.0).round().clamp(0.0, 65535.0) as u16 }
/// DS §5.11 inverted: RH = 100 * S_RH / 2^16, so S_RH = RH * 2^16 / 100, rounded.
pub fn raw_humidity(percent: f64) -> u16 { (percent * 65536.0 / 100.0).round().clamp(0.0, 65535.0) as u16 }

/// DS §5.11.
pub fn temperature_of(raw: u16) -> f64 { -45.0 + 175.0 * f64::from(raw) / 65536.0 }
/// DS §5.11.
pub fn humidity_of(raw: u16) -> f64 { 100.0 * f64::from(raw) / 65536.0 }

/// One word as the sensor sends it: MSB, LSB, CRC (DS §5.6).
fn word(v: u16) -> [u8; 3] { let b = v.to_be_bytes(); [b[0], b[1], crc8(&b)] }

/// The air at the sensor and what the sensor did: shared between the device, the board's
/// `climate` verb and its report.
#[derive(Clone, Debug, PartialEq)]
pub struct Shtc3Air {
    pub temp_c: f64,
    pub humidity: f64,
    /// Measurement commands taken, ID reads answered, bytes or headers refused with a NACK.
    pub measurements: u64,
    pub id_reads: u64,
    pub nacks: u64,
}

/// A plausible room (our choice): 22.5 °C, 45 %RH.
impl Default for Shtc3Air {
    fn default() -> Self { Shtc3Air { temp_c: 22.5, humidity: 45.0, measurements: 0, id_reads: 0, nacks: 0 } }
}

/// The range a `climate` verb may set: DS Table 2 (T, -40 to 125 °C) and Table 1 (RH, 0 to 100 %RH).
pub fn check_air(temp_c: f64, humidity: f64) -> Result<(), String> {
    if !(-40.0..=125.0).contains(&temp_c) { return Err(format!("temperature {temp_c} outside the SHTC3's -40..125 °C")); }
    if !(0.0..=100.0).contains(&humidity) { return Err(format!("humidity {humidity} outside 0..100 %RH")); }
    Ok(())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Sleep,
    Idle,
    /// Waking up or soft-resetting: idle from `until` on.
    Starting { until: u64 },
    /// Measuring until `until`, then the result waits in `frame`.
    Measuring { until: u64, stretch: bool, frame: [u8; 6] },
}

pub struct Shtc3 {
    air: Arc<Mutex<Shtc3Air>>,
    clock: Arc<AtomicU64>,
    state: State,
    /// Command bytes received in this write transaction (a command is two, DS §5).
    cmd: [u8; 2],
    cmd_len: u8,
    /// What a read header gets: the ID or a finished measurement, and how much has gone out.
    out: [u8; 6],
    out_len: u8,
    out_pos: u8,
    reading: bool,
}

impl Shtc3 {
    pub fn new(air: Arc<Mutex<Shtc3Air>>, clock: Arc<AtomicU64>) -> Self {
        Shtc3 { air, clock, state: State::Idle, cmd: [0; 2], cmd_len: 0, out: [0; 6], out_len: 0, out_pos: 0, reading: false }
    }

    fn air(&self) -> std::sync::MutexGuard<'_, Shtc3Air> { self.air.lock().expect("SHTC3 air mutex poisoned") }

    fn nack(&self) -> bool { self.air().nacks += 1; false }

    /// Let the clock finish what was running.
    fn settle(&mut self) -> u64 {
        let now = self.clock.load(Relaxed);
        match self.state {
            State::Starting { until } if now >= until => self.state = State::Idle,
            State::Measuring { until, frame, .. } if now >= until => self.ready(frame),
            _ => {}
        }
        now
    }

    fn ready(&mut self, frame: [u8; 6]) {
        self.state = State::Idle;
        self.out = frame;
        self.out_len = 6;
    }

    /// A complete two-byte command, taken at the ACK of its last byte (DS §5.7: tSR runs from that ACK).
    fn execute(&mut self, cmd: u16, now: u64) -> bool {
        if self.state == State::Sleep {
            if cmd != CMD_WAKEUP { return self.nack(); }
            self.state = State::Starting { until: now + T_WAKE_US * CYCLES_PER_US };
            return true;
        }
        // Headers are refused while busy (start()), so the sensor is idle here.
        match cmd {
            CMD_WAKEUP => {}                                   // already awake: nothing to do (not verified; the DS starts every cycle with it, Figure 7)
            CMD_SLEEP => { self.state = State::Sleep; self.out_len = 0; }
            CMD_SOFT_RESET => { self.state = State::Starting { until: now + T_RESET_US * CYCLES_PER_US }; self.out_len = 0; }
            CMD_READ_ID => {
                let id = word(SHTC3_ID);
                self.out[..3].copy_from_slice(&id);
                self.out_len = 3;
                self.air().id_reads += 1;
            }
            _ => {
                let Some(m) = measure_command(cmd) else { return self.nack(); };   // an unknown command: NACK (not verified)
                let (t, rh) = {
                    let mut air = self.air();
                    air.measurements += 1;
                    (word(raw_temperature(air.temp_c)), word(raw_humidity(air.humidity)))
                };
                let (first, second) = if m.rh_first { (rh, t) } else { (t, rh) };
                let mut frame = [0; 6];
                frame[..3].copy_from_slice(&first);
                frame[3..].copy_from_slice(&second);
                let us = if m.low_power { T_MEAS_LOW_POWER_US } else { T_MEAS_NORMAL_US };
                self.state = State::Measuring { until: now + us * CYCLES_PER_US, stretch: m.stretch, frame };
                self.out_len = 0;
            }
        }
        true
    }
}

impl I2cDevice for Shtc3 {
    fn start(&mut self, read: bool) -> bool {
        self.settle();
        self.cmd_len = 0;
        self.reading = read;
        self.out_pos = 0;
        let ack = match self.state {
            State::Sleep => !read,                                      // only the wake-up gets in
            State::Starting { .. } => false,
            State::Measuring { stretch: true, frame, .. } if read => { self.ready(frame); true }   // SCL held until done (§5.5)
            State::Measuring { .. } => false,                           // §5.5
            State::Idle => !read || self.out_len > 0,                   // nothing to send: NACK (not verified)
        };
        if ack { true } else { self.nack() }
    }

    fn write(&mut self, b: u8) -> bool {
        if self.reading || self.cmd_len >= 2 { return self.nack(); }   // a third command byte: NACK (not verified)
        self.cmd[self.cmd_len as usize] = b;
        self.cmd_len += 1;
        if self.cmd_len == 1 {
            if self.state == State::Sleep && b != (CMD_WAKEUP >> 8) as u8 { return self.nack(); }
            return true;
        }
        let now = self.settle();
        self.execute(u16::from_be_bytes(self.cmd), now)
    }

    /// Past the result the sensor drives nothing and the pull-ups read 0xFF (not verified).
    fn read(&mut self) -> u8 {
        if self.out_pos >= self.out_len { return 0xff; }
        let v = self.out[self.out_pos as usize];
        self.out_pos += 1;
        v
    }

    fn stop(&mut self) {
        // The master may stop after any byte (§5.6); what it did not take is gone.
        if self.reading && self.out_pos > 0 { self.out_len = 0; }
        self.reading = false;
        self.cmd_len = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const US: u64 = CYCLES_PER_US;

    struct Bench { dev: Shtc3, air: Arc<Mutex<Shtc3Air>>, clock: Arc<AtomicU64> }
    impl Bench {
        fn new() -> Self {
            let (air, clock) = (Arc::new(Mutex::new(Shtc3Air::default())), Arc::new(AtomicU64::new(1_000)));
            Bench { dev: Shtc3::new(air.clone(), clock.clone()), air, clock }
        }
        fn at_us(&self, us: u64) { self.clock.store(1_000 + us * US, Relaxed); }
        /// One write transaction as the IDF driver sends it: header, bytes, stop. The ACKs.
        fn command(&mut self, cmd: u16) -> Vec<bool> {
            let mut acks = vec![self.dev.start(false)];
            for b in cmd.to_be_bytes() {
                if !acks.last().unwrap() { break; }
                acks.push(self.dev.write(b));
            }
            if acks.iter().all(|a| *a) { self.dev.stop(); }
            acks
        }
        /// One read transaction: None if the header is refused.
        fn read(&mut self, n: usize) -> Option<Vec<u8>> {
            if !self.dev.start(true) { return None; }
            let v = (0..n).map(|_| self.dev.read()).collect();
            self.dev.stop();
            Some(v)
        }
    }

    #[test]
    fn crc_and_conversions_match_the_datasheet_examples() {
        // Table 16.
        assert_eq!(crc8(&[0x00]), 0xac);
        assert_eq!(crc8(&[0xbe, 0xef]), 0x92);
        // Figure 7: humidity A1 33 CRC 1C, temperature 64 8B CRC C7: "63 %RH and 23.7 °C".
        assert_eq!(crc8(&[0xa1, 0x33]), 0x1c);
        assert_eq!(crc8(&[0x64, 0x8b]), 0xc7);
        assert_eq!(format!("{:.0} {:.1}", humidity_of(0xa133), temperature_of(0x648b)), "63 23.7");
        // The inverse lands on the nearest word, and the ends clamp.
        for t in [-40.0, -12.3, 0.0, 19.5, 21.8, 22.5, 60.0, 125.0] { assert!((temperature_of(raw_temperature(t)) - t).abs() <= 175.0 / 65536.0 / 2.0 + 1e-12, "{t}"); }
        for rh in [0.0, 45.0, 52.0, 60.0, 99.99] { assert!((humidity_of(raw_humidity(rh)) - rh).abs() <= 100.0 / 65536.0 / 2.0 + 1e-12, "{rh}"); }
        assert_eq!((raw_humidity(100.0), raw_temperature(-50.0)), (65535, 0));
    }

    #[test]
    fn id_read_is_the_word_and_its_crc() {
        let mut b = Bench::new();
        assert_eq!(b.command(CMD_READ_ID), [true, true, true]);
        let id = b.read(3).unwrap();
        assert_eq!(id, [0x08, 0x07, crc8(&[0x08, 0x07])]);
        assert_eq!(u16::from_be_bytes([id[0], id[1]]) & 0x083f, 0x0807, "Table 15's product code");
        assert_eq!(b.read(3), None, "read once; nothing left to send");
        assert_eq!(b.air.lock().unwrap().id_reads, 1);
    }

    #[test]
    fn the_firmware_cycle_reads_the_room_after_the_measurement_time() {
        let mut b = Bench::new();
        // climate_reader.h: wake-up, 1 ms, 0x7866, 15 ms, read 6, sleep.
        assert_eq!(b.command(CMD_WAKEUP), [true; 3]);
        b.at_us(1_000);
        assert_eq!(b.command(0x7866), [true; 3]);
        b.at_us(1_000 + T_MEAS_NORMAL_US - 1);
        assert_eq!(b.read(6), None, "still measuring: the read header is refused (§5.5)");
        assert_eq!(b.command(CMD_SLEEP), [false], "and so is a write header");
        b.at_us(1_000 + T_MEAS_NORMAL_US);
        let f = b.read(6).unwrap();
        assert_eq!(f[..3], word(raw_temperature(22.5)), "temperature first");
        assert_eq!(f[3..], word(raw_humidity(45.0)));
        assert_eq!(crc8(&f[..2]), f[2]);
        assert_eq!(crc8(&f[3..5]), f[5]);
        // The firmware's own arithmetic (shtc3.h centiDegrees/centiPercent) gives the room back.
        let (st, srh) = (i64::from(u16::from_be_bytes([f[0], f[1]])), i64::from(u16::from_be_bytes([f[3], f[4]])));
        assert_eq!((-4500 + ((17500 * st + 32768) >> 16), (10000 * srh + 32768) >> 16), (2250, 4500));
        assert_eq!(b.read(6), None, "the result is read once");
        assert_eq!(b.command(CMD_SLEEP), [true; 3]);
        let air = b.air.lock().unwrap();
        assert_eq!((air.measurements, air.nacks), (1, 3));
    }

    #[test]
    fn asleep_it_takes_only_the_wake_up() {
        let mut b = Bench::new();
        assert_eq!(b.command(CMD_SLEEP), [true; 3]);
        assert_eq!(b.command(CMD_READ_ID), [true, false], "header ACKed so the wake-up can get in; 0xEF refused");
        assert_eq!(b.command(0x7866), [true, false]);
        assert_eq!(b.command(0x3500), [true, true, false], "the wake-up's first byte, then not its second");
        assert_eq!(b.read(1), None);
        assert_eq!(b.command(CMD_WAKEUP), [true; 3]);
        b.at_us(T_WAKE_US - 1);
        assert_eq!(b.command(CMD_READ_ID), [false], "still waking up (tPU, Table 5)");
        b.at_us(T_WAKE_US);
        assert_eq!(b.command(CMD_READ_ID), [true; 3]);
        assert!(b.read(3).is_some());
    }

    #[test]
    fn soft_reset_clears_and_takes_t_sr() {
        let mut b = Bench::new();
        assert_eq!(b.command(CMD_READ_ID), [true; 3]);
        assert_eq!(b.command(CMD_SOFT_RESET), [true; 3]);
        b.at_us(T_RESET_US - 1);
        assert_eq!(b.read(3), None);
        b.at_us(T_RESET_US);
        assert_eq!(b.read(3), None, "the ID asked for before the reset is gone");
        assert_eq!(b.command(0x1234), [true, true, false], "an unknown command");
    }

    #[test]
    fn rh_first_low_power_and_clock_stretching() {
        let mut b = Bench::new();
        b.air.lock().unwrap().temp_c = 19.5;
        b.air.lock().unwrap().humidity = 60.0;
        // Low power, RH first, no stretching: 0.8 ms (Table 5).
        assert_eq!(b.command(0x401A), [true; 3]);
        b.at_us(T_MEAS_LOW_POWER_US - 1);
        assert_eq!(b.read(6), None);
        b.at_us(T_MEAS_LOW_POWER_US);
        let f = b.read(6).unwrap();
        assert_eq!(f[..3], word(raw_humidity(60.0)), "RH first");
        assert_eq!(f[3..], word(raw_temperature(19.5)));
        // Normal mode, T first, stretching: the read header is ACKed at once and the result follows.
        b.at_us(2_000);
        assert_eq!(b.command(0x7CA2), [true; 3]);
        let f = b.read(6).unwrap();
        assert_eq!(f[..3], word(raw_temperature(19.5)));
        // The master may stop early (§5.6); past the six bytes the bus reads 0xFF.
        assert_eq!(b.command(0x5C24), [true; 3]);
        assert_eq!(b.read(8).unwrap()[..], [&word(raw_humidity(60.0))[..], &word(raw_temperature(19.5))[..], &[0xff, 0xff]].concat()[..]);
        assert_eq!(b.command(0x5C24), [true; 3]);
        assert_eq!(b.read(2).unwrap(), word(raw_humidity(60.0))[..2]);
        assert_eq!(b.read(1), None);
        assert!(measure_command(0x7866).is_some() && measure_command(0x7867).is_none());
    }

    #[test]
    fn verb_bounds_follow_the_specified_ranges() {
        assert!(check_air(21.8, 52.0).is_ok());
        assert!(check_air(-40.0, 0.0).is_ok() && check_air(125.0, 100.0).is_ok());
        assert!(check_air(-40.1, 50.0).is_err() && check_air(20.0, 100.1).is_err() && check_air(f64::NAN, 50.0).is_err());
    }
}
