//! Collision quarantine for promiscuous ANT+ capture.
//!
//! The dongle garbles two overlapping RF transmissions into
//! corrupt-but-well-formed packets. **No checksum can catch this**: the radio's
//! CRC passed upstream on each packet on its own, and the USB checksum is an XOR
//! the dongle computes at serialization time over the buffer it is about to
//! send, so anything corrupted before or during staging is checksum-valid by
//! construction. Only timing gives them away. ANT+ sensors broadcast at ~4 Hz, so
//! two messages from one device inside the threshold cannot both be genuine, and
//! nothing says which one is the garbage, so both are dropped. No message is
//! delivered until it has survived quarantine.
//!
//! **Timing is the host's arrival clock, deliberately.** The dongle's own RX
//! timestamp resolves finer, but it rides in the frame and a garbled frame can
//! carry a garbled stamp — timing collisions with a clock the fault itself
//! corrupts. It buys nothing here anyway: a venue capture puts the normal cadence
//! at the 200-300 ms channel period and collisions under 1 ms, so the two are
//! 200x apart and an arrival clock separates them with room to spare.
//!
//! Dropping a legitimate message costs nothing, because ANT+ profiles carry
//! CUMULATIVE counters and the next broadcast restores the full state. Admitting
//! a garbled one costs a wrong number that nothing downstream can detect. So the
//! threshold is set generously: it is the cheap direction to err in.
//!
//! `raceble`'s `src/lib/devices/ant/collision.ts` is the same algorithm for the
//! browser's WebUSB path; the two are expected to agree packet for packet.

use crate::message::DeviceKey;
use ant::messages::AntMessage;
use std::collections::HashMap;
use std::time::{Duration, Instant};

/// Minimum quarantine hold before release. Generous next to the default
/// threshold, so a pair split by a host-side read stall still meets in
/// quarantine. A threshold longer than this raises it (see `hold`), or a
/// message could be released while a collision could still be declared against
/// it, and only one of the pair would be dropped.
const MIN_HOLD: Duration = Duration::from_millis(50);

struct KeyState {
    last_wall: Instant,
    pending: Option<AntMessage>,
}

pub struct CollisionDetector {
    threshold: Duration,
    hold: Duration,
    device_state: HashMap<DeviceKey, KeyState>,
    dropped: u64,
}

impl CollisionDetector {
    pub fn new(threshold: Duration) -> Self {
        Self {
            threshold,
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
        let Some(entry) = self.device_state.get_mut(&key) else {
            self.device_state.insert(
                key,
                KeyState {
                    last_wall: now,
                    pending: Some(msg),
                },
            );
            return None;
        };

        if now.duration_since(entry.last_wall) < self.threshold {
            println!("WARNING: Collision on {key}, dropping messages");
            self.dropped += if entry.pending.is_some() { 2 } else { 1 };
            entry.pending = None;
            entry.last_wall = now;
            return None;
        }

        let survivor = entry.pending.take();
        entry.pending = Some(msg);
        entry.last_wall = now;
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

#[cfg(test)]
mod tests {
    use super::*;
    use ant::messages::AntMessage;

    const THRESHOLD: Duration = Duration::from_millis(1);

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
