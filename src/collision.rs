//! Collision quarantine for promiscuous ANT+ capture.
//!
//! Some dongle firmware garbles two overlapping RF transmissions into
//! corrupt-but-well-formed packets: the USB checksum covers the already-corrupt
//! bytes, so only timing gives them away. ANT+ sensors broadcast at ~4 Hz, so two
//! messages from one device inside the threshold cannot both be genuine, and
//! nothing says which one is the garbage, so both are dropped. No message is
//! delivered until it has survived quarantine.
//!
//! `raceble`'s `src/lib/devices/ant/collision.ts` is the same algorithm for the
//! browser's WebUSB path; the two are expected to agree packet for packet.

use crate::message::{DeviceKey, rx_timestamp};
use ant::messages::AntMessage;
use std::collections::HashMap;
use std::time::{Duration, Instant};

const RX_TICKS_PER_SECOND: f64 = 32768.0;

/// Minimum quarantine hold before release. Generous next to the 1 ms default
/// threshold, so a pair split by a host-side read stall still meets in
/// quarantine. A threshold longer than this raises it (see `hold`), or a
/// message could be released while a collision could still be declared against
/// it, and only one of the pair would be dropped.
const MIN_HOLD: Duration = Duration::from_millis(50);

/// A u16 tick delta aliases every 2 s, so a small one is only trusted as "same
/// window" when the wall clocks are close too.
const RX_TICK_ALIAS_GUARD: Duration = Duration::from_millis(1_500);

struct KeyState {
    last_wall: Instant,
    last_rx_ticks: Option<u16>,
    pending: Option<AntMessage>,
}

pub struct CollisionDetector {
    threshold: Duration,
    /// None when the threshold outruns what the counter can express without
    /// aliasing, leaving the wall clock as the only usable source.
    threshold_ticks: Option<u16>,
    hold: Duration,
    device_state: HashMap<DeviceKey, KeyState>,
    dropped: u64,
}

impl CollisionDetector {
    pub fn new(threshold: Duration) -> Self {
        // Ceiling, not rounding: the comparison is `delta < limit` on an integer
        // counter, so anything below it has to be inside the threshold.
        let ticks = (threshold.as_secs_f64() * RX_TICKS_PER_SECOND).ceil();
        Self {
            threshold,
            threshold_ticks: (ticks < u16::MAX as f64).then_some(ticks as u16),
            hold: threshold.max(MIN_HOLD),
            device_state: HashMap::new(),
            dropped: 0,
        }
    }

    pub fn is_disabled(&self) -> bool {
        self.threshold == Duration::ZERO
    }

    pub fn dropped_count(&self) -> u64 {
        self.dropped
    }

    /// Returns the previously quarantined same-key message when the newcomer
    /// proves it survived its window; None when the newcomer was quarantined or
    /// a collision killed both.
    pub fn feed(&mut self, key: DeviceKey, msg: AntMessage) -> Option<AntMessage> {
        self.feed_at(Instant::now(), key, msg)
    }

    pub fn feed_at(&mut self, now: Instant, key: DeviceKey, msg: AntMessage) -> Option<AntMessage> {
        let ticks = rx_timestamp(&msg);
        let Some(entry) = self.device_state.get_mut(&key) else {
            self.device_state.insert(
                key,
                KeyState {
                    last_wall: now,
                    last_rx_ticks: ticks,
                    pending: Some(msg),
                },
            );
            return None;
        };

        if is_collision(entry, now, ticks, self.threshold, self.threshold_ticks) {
            println!("WARNING: Collision on {key}, dropping messages");
            self.dropped += if entry.pending.is_some() { 2 } else { 1 };
            entry.pending = None;
            entry.last_wall = now;
            entry.last_rx_ticks = ticks;
            return None;
        }

        let survivor = entry.pending.take();
        entry.pending = Some(msg);
        entry.last_wall = now;
        entry.last_rx_ticks = ticks;
        survivor
    }

    /// Release every quarantined message older than the hold.
    pub fn flush_expired(&mut self) -> Vec<(DeviceKey, AntMessage)> {
        self.flush_expired_at(Instant::now())
    }

    pub fn flush_expired_at(&mut self, now: Instant) -> Vec<(DeviceKey, AntMessage)> {
        let hold = self.hold;
        self.device_state
            .iter_mut()
            .filter(|(_, state)| {
                state.pending.is_some() && now.duration_since(state.last_wall) >= hold
            })
            .map(|(key, state)| (*key, state.pending.take().unwrap()))
            .collect()
    }

    /// Drop all quarantine state (dongle reopen); the counter survives.
    pub fn reset(&mut self) {
        self.device_state.clear();
    }

    /// Forget devices silent for longer than `ttl`. Keys come off the air, so
    /// a long-running process otherwise holds every phantom number a garbled
    /// frame ever minted. A message still in quarantine is never dropped.
    pub fn evict_idle(&mut self, now: Instant, ttl: Duration) -> usize {
        let before = self.device_state.len();
        self.device_state.retain(|_, state| {
            state.pending.is_some() || now.duration_since(state.last_wall) <= ttl
        });
        before - self.device_state.len()
    }
}

fn is_collision(
    entry: &KeyState,
    now: Instant,
    ticks: Option<u16>,
    threshold: Duration,
    threshold_ticks: Option<u16>,
) -> bool {
    if let (Some(last), Some(current), Some(limit)) = (entry.last_rx_ticks, ticks, threshold_ticks)
    {
        let delta = current.wrapping_sub(last);
        return delta < limit && now.duration_since(entry.last_wall) < RX_TICK_ALIAS_GUARD;
    }
    now.duration_since(entry.last_wall) < threshold
}

#[cfg(test)]
mod tests {
    use super::*;
    use ant::messages::data::{BroadcastData, ExtendedInfo, FlagByte, TimestampOutput};
    use ant::messages::{AntMessage, RxMessage};
    use packed_struct::PackedStruct;

    const THRESHOLD: Duration = Duration::from_millis(1);
    /// 1 ms at 32768 Hz.
    const THRESHOLD_TICKS: u16 = 33;

    fn key_a() -> DeviceKey {
        DeviceKey {
            device_number: 327780,
            device_type_id: 11,
        }
    }

    fn key_b() -> DeviceKey {
        DeviceKey {
            device_number: 655560,
            device_type_id: 12,
        }
    }

    /// No extended info at all: the clone-dongle case.
    fn plain() -> AntMessage {
        AntMessage::default()
    }

    fn stamped(rx_timestamp: u16) -> AntMessage {
        let mut msg = AntMessage::default();
        let mut brd = BroadcastData::new(0, [0; 8]);
        brd.extended_info = Some(ExtendedInfo {
            flag_byte: FlagByte::unpack(&[0b0010_0000]).unwrap(),
            channel_id_output: None,
            rssi_output: None,
            timestamp_output: Some(TimestampOutput { rx_timestamp }),
        });
        msg.message = RxMessage::BroadcastData(brd);
        msg
    }

    #[test]
    fn first_message_is_buffered() {
        let mut det = CollisionDetector::new(THRESHOLD);
        assert!(det.feed_at(Instant::now(), key_a(), plain()).is_none());
    }

    #[test]
    fn message_flushed_when_next_arrives_outside_threshold() {
        let mut det = CollisionDetector::new(THRESHOLD);
        let t0 = Instant::now();

        det.feed_at(t0, key_a(), plain());
        let result = det.feed_at(t0 + Duration::from_millis(5), key_a(), plain());
        assert!(result.is_some());
    }

    #[test]
    fn collision_drops_both_messages() {
        let mut det = CollisionDetector::new(THRESHOLD);
        let t0 = Instant::now();
        let t1 = t0 + Duration::from_micros(500);

        det.feed_at(t0, key_a(), plain());
        assert!(det.feed_at(t1, key_a(), plain()).is_none());
        assert_eq!(det.dropped_count(), 2);

        // Nothing is left quarantined, so a later message only buffers.
        let t2 = t1 + Duration::from_millis(5);
        assert!(det.feed_at(t2, key_a(), plain()).is_none());
    }

    #[test]
    fn different_devices_do_not_collide() {
        let mut det = CollisionDetector::new(THRESHOLD);
        let t0 = Instant::now();

        det.feed_at(t0, key_a(), plain());
        det.feed_at(t0 + Duration::from_micros(100), key_b(), plain());

        let flushed = det.flush_expired_at(t0 + MIN_HOLD + Duration::from_millis(1));
        assert_eq!(flushed.len(), 2);
        assert_eq!(det.dropped_count(), 0);
    }

    #[test]
    fn quarantine_is_held_for_the_hold_period_not_the_threshold() {
        let mut det = CollisionDetector::new(THRESHOLD);
        let t0 = Instant::now();

        det.feed_at(t0, key_a(), plain());

        // Past the threshold, well inside the hold.
        assert!(
            det.flush_expired_at(t0 + Duration::from_millis(5))
                .is_empty()
        );

        let flushed = det.flush_expired_at(t0 + MIN_HOLD);
        assert_eq!(flushed.len(), 1);
        assert_eq!(flushed[0].0, key_a());
        assert!(det.flush_expired_at(t0 + Duration::from_secs(1)).is_empty());
    }

    #[test]
    fn rx_timestamps_collide_a_pair_the_wall_clock_pulled_apart() {
        let mut det = CollisionDetector::new(THRESHOLD);
        let t0 = Instant::now();

        det.feed_at(t0, key_a(), stamped(1_000));
        // The host read them 20ms apart; the radio heard them 10 ticks apart.
        let result = det.feed_at(t0 + Duration::from_millis(20), key_a(), stamped(1_010));
        assert!(result.is_none());
        assert_eq!(det.dropped_count(), 2);
    }

    #[test]
    fn rx_timestamps_do_not_collide_a_genuine_pair_the_host_delivered_together() {
        let mut det = CollisionDetector::new(THRESHOLD);
        let t0 = Instant::now();

        det.feed_at(t0, key_a(), stamped(1_000));
        // Same wall-clock instant, 8192 ticks apart: a full 4 Hz period.
        let result = det.feed_at(t0, key_a(), stamped(1_000 + 8_192));
        assert!(result.is_some());
        assert_eq!(det.dropped_count(), 0);
    }

    #[test]
    fn a_tick_counter_that_aliased_is_not_read_as_a_collision() {
        let mut det = CollisionDetector::new(THRESHOLD);
        let t0 = Instant::now();

        det.feed_at(t0, key_a(), stamped(1_000));
        // Exactly one counter wrap later, so the tick delta is tiny again.
        let result = det.feed_at(t0 + Duration::from_secs(2), key_a(), stamped(1_005));
        assert!(result.is_some());
        assert_eq!(det.dropped_count(), 0);
    }

    #[test]
    fn falls_back_to_the_wall_clock_when_a_dongle_reports_no_timestamps() {
        let mut det = CollisionDetector::new(THRESHOLD);
        let t0 = Instant::now();

        det.feed_at(t0, key_a(), plain());
        assert!(
            det.feed_at(t0 + Duration::from_micros(200), key_a(), plain())
                .is_none()
        );
        assert_eq!(det.dropped_count(), 2);
    }

    #[test]
    fn a_dongle_that_starts_reporting_timestamps_mid_stream_still_decides() {
        let mut det = CollisionDetector::new(THRESHOLD);
        let t0 = Instant::now();

        // One side missing ticks leaves the wall clock as the only shared source.
        det.feed_at(t0, key_a(), plain());
        let result = det.feed_at(t0 + Duration::from_micros(200), key_a(), stamped(50));
        assert!(result.is_none());
        assert_eq!(det.dropped_count(), 2);
    }

    #[test]
    fn threshold_in_ticks_matches_the_threshold_in_time() {
        let det = CollisionDetector::new(THRESHOLD);
        assert_eq!(det.threshold_ticks, Some(THRESHOLD_TICKS));

        // Beyond what a u16 at 32768 Hz can express.
        let det = CollisionDetector::new(Duration::from_secs(3));
        assert_eq!(det.threshold_ticks, None);
    }

    #[test]
    fn threshold_ticks_round_up_so_the_integer_compare_stays_strict() {
        // 1.01 ms is 33.095 ticks: a 33-tick gap is inside the threshold and
        // must collide, which `33 < 33` would miss.
        let det = CollisionDetector::new(Duration::from_micros(1_010));
        assert_eq!(det.threshold_ticks, Some(34));
    }

    #[test]
    fn a_threshold_longer_than_the_minimum_hold_raises_the_hold() {
        let threshold = Duration::from_millis(100);
        let mut det = CollisionDetector::new(threshold);
        let t0 = Instant::now();

        // Releasing at MIN_HOLD would hand out the first message, and the 75ms
        // one would then collide with a peer that had already been delivered.
        det.feed_at(t0, key_a(), plain());
        assert!(det.flush_expired_at(t0 + MIN_HOLD).is_empty());
        assert!(
            det.feed_at(t0 + Duration::from_millis(75), key_a(), plain())
                .is_none()
        );
        assert_eq!(det.dropped_count(), 2);
        assert!(det.flush_expired_at(t0 + threshold).is_empty());
    }

    #[test]
    fn reset_clears_quarantine_but_keeps_the_counter() {
        let mut det = CollisionDetector::new(THRESHOLD);
        let t0 = Instant::now();

        det.feed_at(t0, key_a(), plain());
        det.feed_at(t0 + Duration::from_micros(100), key_a(), plain());
        assert_eq!(det.dropped_count(), 2);

        det.reset();
        assert!(det.flush_expired_at(t0 + Duration::from_secs(1)).is_empty());
        assert_eq!(det.dropped_count(), 2);
    }

    #[test]
    fn evict_idle_forgets_silent_devices_but_never_a_quarantined_message() {
        let mut det = CollisionDetector::new(THRESHOLD);
        let t0 = Instant::now();
        let ttl = Duration::from_secs(60);

        det.feed_at(t0, key_a(), plain());
        det.flush_expired_at(t0 + MIN_HOLD);
        det.feed_at(t0 + Duration::from_secs(30), key_b(), plain());

        // Key A is silent past the TTL; key B still holds a message.
        assert_eq!(det.evict_idle(t0 + ttl + Duration::from_secs(1), ttl), 1);
        assert_eq!(
            det.flush_expired_at(t0 + ttl + Duration::from_secs(2))
                .len(),
            1
        );
        // Key A comes back as a new device: buffered, not collided against its old state.
        assert!(
            det.feed_at(t0 + ttl + Duration::from_secs(3), key_a(), plain())
                .is_none()
        );
        assert_eq!(det.dropped_count(), 0);
    }

    #[test]
    fn disabled_when_zero_threshold() {
        assert!(CollisionDetector::new(Duration::ZERO).is_disabled());
        assert!(!CollisionDetector::new(THRESHOLD).is_disabled());
    }
}
