//! ANT+ device profile encoding for the simulator.
//!
//! A profile is two things: the constants that put a channel on the air where a
//! receiver expects to find it (device type and channel period), and the eight
//! payload bytes it broadcasts.
//!
//! **The counters are the point.** ANT+ profiles do not transmit speed; they
//! transmit a cumulative revolution count and the time of the revolution that
//! bumped it, and the receiver divides. So a simulated sensor is not "a bike
//! going 25 km/h", it is a counter turning at a constant rate, sampled whenever
//! the radio asks for a payload.
//!
//! **That sampling is why the values repeat, and the repeats are deliberate.**
//! A real sensor's counter only moves when a magnet passes, which is not in step
//! with its 4 Hz broadcast, so at any speed below ~4 revolutions a second the
//! same count and the same event time go out several broadcasts running. A
//! simulator that incremented per broadcast would never produce that, and a
//! parser that mishandled it would sail through the test. [`Revolutions::at`]
//! reports the state as of the last revolution to have actually happened, so the
//! repeats fall out of the arithmetic rather than being bolted on.

use std::time::Duration;

/// Combined bike speed and cadence. The period is 4.05 Hz, and it is the same
/// 8086 ticks the bench capture measured as the median gap between frames from
/// real sensors, so a simulated fleet sits in the same cadence as a real one.
pub const CSC_DEVICE_TYPE: u8 = 121;
pub const CSC_CHANNEL_PERIOD: u16 = 8086;

/// Both counters in the combined page roll at 16 bits: the revolution count
/// straightforwardly, and the event time because it is measured in 1/1024 s,
/// which puts its wrap at exactly 64 seconds. A run of any length crosses the
/// second one constantly, so it is not an edge case a receiver can postpone.
const ROLLOVER: u64 = 1 << 16;

/// Ticks per second in the 1/1024 s unit the speed and cadence pages count in.
const TICKS_PER_SECOND: f64 = 1024.0;

/// What one counter looks like on the wire at some moment.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RevolutionState {
    /// Cumulative revolutions since start, rolled to 16 bits.
    pub revolutions: u16,
    /// When the revolution that produced `revolutions` happened, in 1/1024 s
    /// since start, rolled to 16 bits. Not the time of the broadcast: a
    /// broadcast that carries no new revolution repeats the previous stamp.
    pub event_time: u16,
}

/// A counter turning at a constant rate.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Revolutions {
    per_second: f64,
}

impl Revolutions {
    /// A counter turning `per_second` times a second. A rate at or below zero
    /// is a stopped wheel, which is a real state rather than an error: its
    /// counter and its event time both stand still, which is exactly what a
    /// parked bike transmits.
    #[must_use]
    pub fn per_second(per_second: f64) -> Self {
        Self {
            per_second: if per_second.is_finite() && per_second > 0.0 {
                per_second
            } else {
                0.0
            },
        }
    }

    /// A crank turning at `rpm`.
    #[must_use]
    pub fn from_rpm(rpm: f64) -> Self {
        Self::per_second(rpm / 60.0)
    }

    /// A wheel of `circumference_m` rolling at `kph`.
    #[must_use]
    pub fn from_speed(kph: f64, circumference_m: f64) -> Self {
        if circumference_m <= 0.0 {
            return Self::per_second(0.0);
        }
        Self::per_second(kph / 3.6 / circumference_m)
    }

    /// The counter as of the last revolution to have happened by `elapsed`.
    ///
    /// The floor is the whole mechanism: between two revolutions it returns the
    /// earlier one's count and the earlier one's stamp, unchanged, however many
    /// times it is asked.
    #[must_use]
    pub fn at(&self, elapsed: Duration) -> RevolutionState {
        if self.per_second == 0.0 {
            return RevolutionState::default();
        }
        let turns = (elapsed.as_secs_f64() * self.per_second).floor();
        // Recovering the revolution's own time from its index is what keeps the
        // stamp and the count describing the same event. Timestamping "now"
        // instead would drift them apart and hand the receiver a speed that
        // sagged toward zero between revolutions.
        let ticks = (turns / self.per_second * TICKS_PER_SECOND).round();
        RevolutionState {
            revolutions: (turns as u64 % ROLLOVER) as u16,
            event_time: (ticks as u64 % ROLLOVER) as u16,
        }
    }
}

/// The combined speed and cadence page (device type 121).
///
/// There is no page number: all eight bytes are counters, little-endian, in the
/// order cadence-then-speed. Getting the halves the wrong way round produces a
/// page that parses cleanly and reports a bicycle pedalling at road speed, so
/// the layout is pinned by a golden test rather than trusted to reading.
#[must_use]
pub fn csc_page(cadence: RevolutionState, speed: RevolutionState) -> [u8; 8] {
    let mut page = [0u8; 8];
    page[0..2].copy_from_slice(&cadence.event_time.to_le_bytes());
    page[2..4].copy_from_slice(&cadence.revolutions.to_le_bytes());
    page[4..6].copy_from_slice(&speed.event_time.to_le_bytes());
    page[6..8].copy_from_slice(&speed.revolutions.to_le_bytes());
    page
}

#[cfg(test)]
mod tests {
    use super::*;

    fn secs(s: f64) -> Duration {
        Duration::from_secs_f64(s)
    }

    /// The behaviour the whole module exists to reproduce: a counter sampled
    /// faster than it turns repeats itself, byte for byte, until it turns again.
    #[test]
    fn a_counter_sampled_between_revolutions_repeats_the_last_one() {
        let wheel = Revolutions::per_second(2.0);

        let third = wheel.at(secs(1.0));
        assert_eq!(
            third,
            RevolutionState {
                revolutions: 2,
                event_time: 1024,
            }
        );

        // Four more samples before the next revolution is due, all identical.
        for t in [1.1, 1.2, 1.3, 1.4] {
            assert_eq!(wheel.at(secs(t)), third, "sample at {t}s moved early");
        }

        // And then it moves, with a stamp that is the revolution's own time
        // rather than the sample's.
        assert_eq!(
            wheel.at(secs(1.5)),
            RevolutionState {
                revolutions: 3,
                event_time: 1536,
            }
        );
        assert_eq!(wheel.at(secs(1.9)).event_time, 1536);
    }

    /// Both counters wrap, and the event time wraps often: every 64 seconds,
    /// because 1/1024 s in 16 bits is 64 seconds and nothing more.
    #[test]
    fn the_event_time_wraps_every_64_seconds_and_the_count_at_65536() {
        let wheel = Revolutions::per_second(2.0);

        assert_eq!(wheel.at(secs(63.5)).event_time, 65024);
        // 64 s is 65536 ticks, which is 0.
        assert_eq!(wheel.at(secs(64.0)).event_time, 0);
        assert_eq!(wheel.at(secs(64.0)).revolutions, 128);
        assert_eq!(wheel.at(secs(64.5)).event_time, 512);

        // 65536 revolutions at 2/s is 32768 s, where the count wraps too.
        assert_eq!(wheel.at(secs(32767.5)).revolutions, 65535);
        assert_eq!(wheel.at(secs(32768.0)).revolutions, 0);
    }

    /// A parked bike is a state, not a failure, and a zero-length wheel is the
    /// nonsense input that would otherwise divide by zero.
    #[test]
    fn a_stopped_counter_stands_still_rather_than_dividing_by_zero() {
        for stopped in [
            Revolutions::per_second(0.0),
            Revolutions::per_second(-5.0),
            Revolutions::from_rpm(0.0),
            Revolutions::from_speed(0.0, 2.096),
            Revolutions::from_speed(25.0, 0.0),
            Revolutions::per_second(f64::NAN),
        ] {
            assert_eq!(stopped.at(secs(0.0)), RevolutionState::default());
            assert_eq!(stopped.at(secs(3600.0)), RevolutionState::default());
        }
    }

    #[test]
    fn speed_and_cadence_convert_to_the_rate_the_profile_counts_in() {
        // 90 rpm is 1.5 crank revolutions a second, so 3 revolutions in 2 s.
        assert_eq!(Revolutions::from_rpm(90.0).at(secs(2.0)).revolutions, 3);

        // 25 km/h on a 2.096 m wheel is 6.944 m/s, so 3.313 rev/s: three whole
        // revolutions in the first second and no more.
        let wheel = Revolutions::from_speed(25.0, 2.096);
        assert_eq!(wheel.at(secs(1.0)).revolutions, 3);
        assert_eq!(wheel.at(secs(10.0)).revolutions, 33);
    }

    /// Cadence occupies the low half and speed the high half. Swapping them
    /// yields a page that parses without complaint, so this is a golden.
    #[test]
    fn the_combined_page_puts_cadence_first_and_speed_second_little_endian() {
        let cadence = RevolutionState {
            event_time: 0x1234,
            revolutions: 0x5678,
        };
        let speed = RevolutionState {
            event_time: 0x9ABC,
            revolutions: 0xDEF0,
        };
        assert_eq!(
            csc_page(cadence, speed),
            [0x34, 0x12, 0x78, 0x56, 0xBC, 0x9A, 0xF0, 0xDE]
        );
        assert_eq!(
            csc_page(RevolutionState::default(), RevolutionState::default()),
            [0; 8]
        );
    }
}
