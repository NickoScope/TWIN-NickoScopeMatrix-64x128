//! Real-time pacing (`--web`, `--realtime`): emulated time is held to the host's clock.
//!
//! The chip's clocks (systimer, RTC slow ticks) count emulated cycles, so any emulated time the
//! pacer gives up is time by which the firmware's clock of day falls behind the world until the
//! firmware next asks NTP (the AnimatedPixelClock does so once an hour: user_config.h
//! NTP_RESYNC_INTERVAL 3600000, main.cpp:1352). Up to 2026-09-29 any lag over 0.5 s was given up
//! at once, and the twin's clock slid 1.0 s, 2.5 s, 3.9 s behind the panel's over half an hour
//! (fidelity check 2026-09-29). Lags come from the host (other work on the Mac: a neighbour's
//! emulator test suites left a test twin 0.75 to 10.3 s behind, four times in 27 minutes, and it
//! ended 17.5 s behind) and from the firmware (a Lua effect loading, the clock's minute animation:
//! 50 to 300 ms, now and then 0.7 to 1.1 s). Now a lag is caught up; only time in which the
//! engine did not run at all, or a lag no catching up should repay, is given up.
//!
//! - Ahead of the host's clock by more than `AHEAD`: sleep the difference (as before).
//! - Behind: run, but no faster than `CATCH_UP` emulated seconds per host second, so the page is
//!   not sent more than that many frames a second and the network's round trips, which are real,
//!   do not look shorter to the firmware than `1 / CATCH_UP` of what they are.
//! - The engine did not run for more than `STALL` (the process was stopped, a debugger held it):
//!   that time is given up, as the old resynchronisation did, but said (`Step::Stalled`). A lag
//!   the machine had before stays and is caught up.
//! - Behind by more than `MAX_BEHIND` though running (a host that cannot keep up): given up and
//!   said (`Step::GiveUp`).
//! - The host slept: `Instant` is `CLOCK_UPTIME_RAW` on Darwin (Rust std `Instant` docs, platform
//!   table), which "does not increment while the system is asleep" (macOS clock_gettime(3)), so
//!   the pacer does not see the sleep, but the chip stood still through it all the same. The
//!   host's wall clock (`SystemTime`) outrunning the monotonic one between two checks shows it
//!   (`Step::HostSlept`); that time is not caught up either.
//!
//! The numbers below are design choices, not figures from a source; the measurements they rest on
//! are in the comments.
use std::time::{Duration, Instant, SystemTime};

/// Emulated seconds per host second at most while behind. The engine runs the panel firmware at
/// about 2.5 times real time on the owner's Mac while the firmware idles (40 % of a core at
/// `--realtime`, 2026-09-29), so 1.5 is within reach after a busy stretch (the test runs caught
/// up at 1.5); it repays a second of lag in two seconds.
pub const CATCH_UP: f64 = 1.5;
/// Behind by more than this though running, the lag is given up: at `CATCH_UP` it would take
/// `MAX_BEHIND / (CATCH_UP - 1)` = 60 s of a quickened page, no more. Three times the worst lag
/// a busy Mac caused in the test runs of 2026-09-29 (10.3 s, 10.4 s; see the commit).
pub const MAX_BEHIND: Duration = Duration::from_secs(30);
/// A gap this long between two checks, less the sleep the first asked for, means the engine did
/// not run. Checks come every 2^16 cycles, 273 us of emulated time; the slowest 20 ms of emulated
/// time seen took 87 ms of the host's (a busy Mac, 2026-09-29), so checks were at most ~1.2 ms apart.
pub const STALL: Duration = Duration::from_secs(1);
/// Ahead by more than this, sleep (the value the pacer always had).
const AHEAD: Duration = Duration::from_millis(2);
/// How far the catch-up line may run ahead of an engine that is slower than it: the burst allowed
/// after a sleep that overslept. Without a bound, a slow stretch would bank credit and the engine
/// would then run at full speed until it used it up.
const SLACK: Duration = Duration::from_millis(20);
/// The host's wall clock outrunning its monotonic clock by more than this between two checks
/// means the host slept. A clock step by NTP is normally far smaller (macOS slews its clock); one
/// bigger than this is reported as a sleep.
const HOST_SLEEP: Duration = Duration::from_secs(1);

/// What the machine does after a check.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Step {
    /// On time, or behind and allowed to run.
    Run,
    /// Ahead of the host's clock, or of the catch-up line: sleep this long.
    Sleep(Duration),
    /// The engine did not run for this long: given up.
    Stalled(Duration),
    /// Behind by this much (> `MAX_BEHIND`) though running: given up.
    GiveUp(Duration),
    /// The host slept this long: the chip stood still meanwhile, and that is not caught up.
    HostSlept(Duration),
}

#[derive(Default)]
pub struct Pace {
    /// The instant at which emulated time was zero; moved only when time is given up.
    start: Option<Instant>,
    /// While behind: the point (instant, emulated time) from which emulated time may grow at `CATCH_UP`.
    line: Option<(Instant, Duration)>,
    /// The previous check, on both host clocks.
    last: Option<(Instant, SystemTime)>,
    /// The sleep the previous check asked for: part of the gap to this one, not a stall.
    asked: Duration,
    /// How far behind the host's clock the last check found the machine (zero when ahead).
    lag: Duration,
}

impl Pace {
    /// How far behind the host's clock the last check found the machine.
    pub fn lag(&self) -> Duration { self.lag }

    /// One check at host instant `now` (wall clock `real`), `emulated` seconds into the run.
    pub fn check(&mut self, now: Instant, real: SystemTime, emulated: Duration) -> Step {
        let last = self.last.replace((now, real));
        let asked = std::mem::take(&mut self.asked);
        if let Some((t, r)) = last {
            let gap = now.saturating_duration_since(t);
            // The wall clock's own step, less the monotonic one: what the monotonic clock missed.
            let slept = real.duration_since(r).unwrap_or_default().saturating_sub(gap);
            if slept > HOST_SLEEP { return Step::HostSlept(slept); }
            let stalled = gap.saturating_sub(asked);
            if stalled > STALL {
                // The host's clock moves on by what the engine missed; the lag it had stays.
                if let Some(s) = &mut self.start { *s += stalled; }
                self.line = None;
                return Step::Stalled(stalled);
            }
        }
        // The first check anchors the host's clock to wherever the machine is.
        let start = *self.start.get_or_insert_with(|| now.checked_sub(emulated).unwrap_or(now));
        let wall = now.saturating_duration_since(start);
        if emulated >= wall {
            self.line = None;
            self.lag = Duration::ZERO;
            let ahead = emulated - wall;
            return if ahead > AHEAD { self.sleep(ahead) } else { Step::Run };
        }
        self.lag = wall - emulated;
        if self.lag > MAX_BEHIND {
            let behind = std::mem::take(&mut self.lag);
            self.start = Some(now.checked_sub(emulated).unwrap_or(now));
            self.line = None;
            return Step::GiveUp(behind);
        }
        let (t0, e0) = *self.line.get_or_insert((now, emulated));
        let allowed = e0 + now.saturating_duration_since(t0).mul_f64(CATCH_UP);
        if emulated > allowed + AHEAD { return self.sleep((emulated - allowed).div_f64(CATCH_UP)); }
        if allowed > emulated + SLACK { self.line = Some((now, emulated + SLACK)); }
        Step::Run
    }

    fn sleep(&mut self, d: Duration) -> Step { self.asked = d; Step::Sleep(d) }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MS: Duration = Duration::from_millis(1);
    /// 2^16 cycles at 240 MHz: how much emulated time passes between two of the machine's checks.
    const STEP: Duration = Duration::from_nanos(273_067);

    /// A run against a simulated host: the engine runs `speed` emulated seconds per host second,
    /// the machine checks every `STEP` of emulated time and sleeps what it is told to.
    struct Host { pace: Pace, t0: Instant, r0: SystemTime, host: Duration, slept: Duration, emulated: Duration }

    impl Host {
        fn new() -> Host {
            let mut h = Host { pace: Pace::default(), t0: Instant::now(), r0: SystemTime::now(), host: Duration::ZERO, slept: Duration::ZERO, emulated: Duration::ZERO };
            assert_eq!(h.check(), Step::Run);
            h
        }
        fn check(&mut self) -> Step { self.pace.check(self.t0 + self.host, self.r0 + self.host + self.slept, self.emulated) }
        /// Run `host_time` of host time at `speed`; the steps other than `Run` and `Sleep`.
        fn run(&mut self, host_time: Duration, speed: f64) -> Vec<Step> {
            let end = self.host + host_time;
            let mut out = Vec::new();
            while self.host < end {
                self.emulated += STEP;
                self.host += STEP.div_f64(speed);
                match self.check() {
                    Step::Run => {}
                    Step::Sleep(d) => self.host += d,
                    s => out.push(s),
                }
            }
            out
        }
        /// The process does not run for `d`: the next check comes that much later.
        fn stop(&mut self, d: Duration) -> Step {
            self.host += d;
            self.emulated += STEP;
            self.check()
        }
        fn lag(&self) -> f64 { self.host.as_secs_f64() - self.emulated.as_secs_f64() }
    }

    #[test]
    fn on_time_and_ahead() {
        let mut h = Host::new();
        // An engine faster than real time sleeps down to it and never falls behind.
        assert!(h.run(Duration::from_secs(3), 2.5).is_empty());
        assert!(h.lag().abs() < 0.003, "{}", h.lag());
        // Ahead by 10 ms more (on top of the up to 2 ms it runs ahead between sleeps): sleep that.
        h.emulated += 10 * MS;
        match h.check() { Step::Sleep(d) => assert!((10 * MS..=13 * MS).contains(&d), "{d:?}"), s => panic!("{s:?}") }
    }

    #[test]
    fn a_long_sleep_is_not_a_stall() {
        // Far ahead (the machine skipped idle time): the sleep the pacer asked for, however long,
        // is not time in which the engine failed to run.
        let mut h = Host::new();
        h.emulated += Duration::from_secs(3);
        let d = match h.check() { Step::Sleep(d) => d, s => panic!("{s:?}") };
        h.host += d;
        assert!(h.run(Duration::from_secs(1), 2.5).is_empty());
        assert!(h.lag().abs() < 0.003, "{}", h.lag());
    }

    #[test]
    fn a_busy_stretch_is_caught_up_not_given_up() {
        // The 2026-09-29 test run: a stretch of a busy Mac that the engine ran at about 0.45 of
        // real time left it 5.9 s behind. The old pacer gave up every 0.5 s of it; now none.
        let mut h = Host::new();
        assert!(h.run(Duration::from_secs(10), 2.5).is_empty());
        assert!(h.run(Duration::from_secs(11), 0.45).is_empty(), "nothing is given up");
        assert!((h.lag() - 6.05).abs() < 0.01, "{}", h.lag());
        assert!((h.pace.lag().as_secs_f64() - 6.05).abs() < 0.01);
        // Then an engine that could run 2.5 times real time is held to CATCH_UP: 6.05 s is repaid
        // in 6.05 / 0.5 = 12.1 s, not in 4. Measured over each second while behind.
        let mut caught = None;
        for s in 0..16 {
            let (e0, lag0) = (h.emulated, h.lag());
            assert!(h.run(Duration::from_secs(1), 2.5).is_empty(), "nothing is given up");
            let rate = (h.emulated - e0).as_secs_f64();
            if lag0 > 0.6 { assert!((CATCH_UP - 0.05..=CATCH_UP + 0.03).contains(&rate), "second {s}: {rate} emulated s per s"); }
            if caught.is_none() && h.lag() < 0.003 { caught = Some(s + 1); }
        }
        assert_eq!(caught, Some(13), "caught up after {caught:?} s");
        assert!(h.lag().abs() < 0.003, "{}", h.lag());
    }

    #[test]
    fn a_slow_stretch_banks_no_credit() {
        // Behind and slower than the line for 2 s, then fast: the first 100 ms of fast running is
        // held to the line plus SLACK, not to the 2 s of line the slow stretch left unused.
        let mut h = Host::new();
        h.run(Duration::from_secs(2), 0.5);
        let e0 = h.emulated;
        h.run(Duration::from_millis(100), 2.5);
        let ran = (h.emulated - e0).as_secs_f64();
        assert!(ran <= 0.1 * CATCH_UP + SLACK.as_secs_f64() + 0.003, "{ran}");
    }

    #[test]
    fn a_stop_is_given_up_and_reported() {
        let mut h = Host::new();
        h.run(Duration::from_secs(1), 2.5);
        // SIGSTOP for 30 s: the next check comes 30 s later.
        match h.stop(Duration::from_secs(30)) {
            Step::Stalled(d) => assert!((d.as_secs_f64() - 30.0).abs() < 0.01, "{d:?}"),
            s => panic!("{s:?}"),
        }
        // On time again at once: no 30 s of quickened running follows.
        let e0 = h.emulated;
        assert!(h.run(Duration::from_secs(1), 2.5).is_empty());
        assert!(((h.emulated - e0).as_secs_f64() - 1.0).abs() < 0.01);
        assert!(h.pace.lag() < MS, "{:?}", h.pace.lag());
    }

    #[test]
    fn a_stop_keeps_the_lag_it_found() {
        // 1 s behind after a busy stretch, then stopped for 10 s: the 10 s go, the 1 s is repaid.
        let mut h = Host::new();
        h.run(Duration::from_secs(2), 0.5);
        assert!(matches!(h.stop(Duration::from_secs(10)), Step::Stalled(_)));
        assert!((h.pace.lag().as_secs_f64() - 1.0).abs() < 0.01, "{:?}", h.pace.lag());
        assert!(h.run(Duration::from_secs(3), 2.5).is_empty());
        assert!(h.pace.lag() < 3 * MS, "{:?}", h.pace.lag());
    }

    #[test]
    fn a_short_stop_is_caught_up() {
        // Held for 0.8 s, under STALL: repaid at CATCH_UP, 0.8 / 0.5 = 1.6 s.
        let mut h = Host::new();
        assert_eq!(h.stop(800 * MS), Step::Run);
        assert!(h.run(Duration::from_millis(1400), 2.5).is_empty());
        assert!(h.lag() > 0.05, "{}", h.lag());
        assert!(h.run(Duration::from_millis(400), 2.5).is_empty());
        assert!(h.lag().abs() < 0.003, "{}", h.lag());
    }

    #[test]
    fn a_host_that_cannot_keep_up_gives_up_now_and_then() {
        // Half real time for 125 s: 62.5 s of lag, given up each time it passes MAX_BEHIND.
        let mut h = Host::new();
        let steps = h.run(Duration::from_secs(125), 0.5);
        assert_eq!(steps.len(), 2, "{steps:?}");
        for s in steps { assert!(matches!(s, Step::GiveUp(behind) if behind > MAX_BEHIND && behind < MAX_BEHIND + MS)); }
    }

    #[test]
    fn a_host_sleep_is_reported() {
        // macOS asleep for an hour: the monotonic clock stands, the wall clock moves.
        let mut h = Host::new();
        h.run(Duration::from_secs(1), 2.5);
        h.slept += Duration::from_secs(3600);
        match h.check() { Step::HostSlept(d) => assert!((d.as_secs_f64() - 3600.0).abs() < 0.01, "{d:?}"), s => panic!("{s:?}") }
        // Nothing to pace: the monotonic clock did not move.
        assert!(h.run(Duration::from_secs(1), 2.5).is_empty());
        assert!(h.lag().abs() < 0.003, "{}", h.lag());
        // The wall clock stepping back (a clock set) is not a sleep.
        h.slept = Duration::ZERO;
        assert!(matches!(h.check(), Step::Run | Step::Sleep(_)));
    }
}
