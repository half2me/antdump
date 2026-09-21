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
//!
//! ## Why the drops are broken down
//!
//! One discard total cannot say whether a box is sitting in a noisy room or
//! whether the HOST is the problem, and those want opposite fixes.
//! `CollisionStats` answers it two ways, neither of which costs anything to
//! keep.
//!
//! **The gap that convicted the pair.** Overlapping transmissions are garbled
//! by the radio and handed up together, which is why a venue capture puts real
//! collisions under a millisecond. A pair several milliseconds apart is one the
//! air almost certainly delivered cleanly, and what collapsed it is the host: a
//! read loop that stalls lets the dongle buffer, and the backlog then arrives
//! back to back on an arrival clock that cannot tell a queue from a collision.
//! So a count in the upper buckets says go and look at the reader rather than
//! at the antenna, and one that crowds the threshold says the threshold is too
//! generous for that venue. The boundaries are ABSOLUTE rather than fractions
//! of the threshold, because the sub-millisecond claim is a fact about the
//! radio and does not move when a caller retunes.
//!
//! **Pairs against burst continuations.** A collision kills two messages, and
//! every further arrival inside the sliding window kills one, the quarantined
//! half being already gone. Splitting them turns the ratio into a shape: a
//! discard total near twice the event count is isolated pairs, one near the
//! event count is a device spraying garbage in long runs.

use crate::message::DeviceKey;
use ant::messages::AntMessage;
use indexmap::IndexMap;
use std::time::{Duration, Instant};

/// Under this, the pair is what the radio does to overlapping transmissions.
const GAP_TIGHT: Duration = Duration::from_millis(1);
/// Under this, a pair too far apart to be a clean radio collision and too close
/// to be any sensor's cadence. At or above it, suspect the reader.
const GAP_LOOSE: Duration = Duration::from_millis(5);

/// What the quarantine threw away, by reason and by how close the pair was.
///
/// Counted per collision EVENT, where `dropped_count` counts PACKETS. The two
/// are deliberately different numbers, tied by `pairs * 2 + bursts == dropped`.
/// Every field is a total since the detector was built, and `reset` leaves them
/// alone for the same reason it leaves `dropped` alone: a dongle reopen throws
/// away quarantine state, not the record of what has been discarded.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CollisionStats {
    /// Events that killed a quarantined message and its newcomer.
    pub pairs: u64,
    /// Events that killed a newcomer alone, the quarantined half having already
    /// died to an earlier collision in the same burst.
    pub bursts: u64,
    /// Events whose gap was under `GAP_TIGHT`: the radio's own doing.
    pub gap_under_1ms: u64,
    /// Events whose gap fell in `GAP_TIGHT..GAP_LOOSE`.
    pub gap_1_to_5ms: u64,
    /// Events whose gap was `GAP_LOOSE` or more, up to the threshold.
    pub gap_over_5ms: u64,
}

impl CollisionStats {
    /// Collisions declared, whatever each one cost.
    #[must_use]
    pub fn events(&self) -> u64 {
        self.pairs + self.bursts
    }

    /// Messages those events threw away. Matches `dropped_count`.
    #[must_use]
    pub fn dropped(&self) -> u64 {
        self.pairs * 2 + self.bursts
    }
}

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
    /// Insertion-ordered on purpose, and this is load-bearing rather than a
    /// preference. `flush_expired_at` releases held messages by iterating this
    /// map, so its order IS the order frames reach the pipeline when several
    /// devices come out of quarantine together. A `HashMap` randomizes that
    /// per process, which made a replay of the same bytes produce a different
    /// frame order on every run and put the conformance goldens permanently
    /// out of reach. `collision.ts` keys a JS `Map`, which iterates in
    /// insertion order, so this is what "agree packet for packet" actually
    /// requires.
    device_state: IndexMap<DeviceKey, KeyState>,
    dropped: u64,
    stats: CollisionStats,
}

impl CollisionDetector {
    pub fn new(threshold: Duration) -> Self {
        Self {
            threshold,
            hold: threshold.max(MIN_HOLD),
            device_state: IndexMap::new(),
            dropped: 0,
            stats: CollisionStats::default(),
        }
    }

    pub fn is_disabled(&self) -> bool {
        self.threshold == Duration::ZERO
    }

    pub fn dropped_count(&self) -> u64 {
        self.dropped
    }

    /// The same discards as `dropped_count`, by reason and by gap.
    #[must_use]
    pub fn stats(&self) -> CollisionStats {
        self.stats
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

        let gap = now.duration_since(entry.last_wall);
        if gap < self.threshold {
            println!("WARNING: Collision on {key} after {gap:?}, dropping messages");
            let paired = entry.pending.is_some();
            self.dropped += if paired { 2 } else { 1 };
            if paired {
                self.stats.pairs += 1;
            } else {
                self.stats.bursts += 1;
            }
            if gap < GAP_TIGHT {
                self.stats.gap_under_1ms += 1;
            } else if gap < GAP_LOOSE {
                self.stats.gap_1_to_5ms += 1;
            } else {
                self.stats.gap_over_5ms += 1;
            }
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
    /// The receiver firmware's default, and the only one wide enough for every
    /// gap bucket to be reachable.
    const WIDE: Duration = Duration::from_millis(25);

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

    /// The release order is a contract, not an accident. `flush_expired_at`
    /// hands its vector straight to the pipeline, so this IS the order frames
    /// are decoded and batched in when a roomful of devices leaves quarantine
    /// together, and `collision.ts` iterates a JS `Map`, which is insertion
    /// ordered. Under the `HashMap` this replaced, the order was randomized per
    /// process: replaying one venue capture through the receiver produced a
    /// different frame order on every run, so the conformance goldens could
    /// never pass and the "agree packet for packet" claim was unprovable past
    /// a handful of devices. One device is not enough to catch it, which is why
    /// this uses sixteen.
    #[test]
    fn quarantine_releases_in_the_order_the_devices_were_first_heard() {
        let keys: Vec<DeviceKey> = (0..16u32)
            .map(|i| DeviceKey {
                device_number: 1000 + i,
                device_type_id: 11,
            })
            .collect();
        let mut det = CollisionDetector::new(THRESHOLD);
        let t0 = Instant::now();
        for (step, key) in (0u64..).zip(&keys) {
            det.feed_at(t0 + Duration::from_millis(step), *key, plain());
        }

        let released: Vec<DeviceKey> = det
            .flush_expired_at(t0 + MIN_HOLD + Duration::from_secs(1))
            .into_iter()
            .map(|(key, _)| key)
            .collect();

        assert_eq!(released, keys, "released out of first-heard order");
    }

    /// The stats and the packet counter answer different questions, so the one
    /// thing that has to hold between them is the arithmetic.
    #[test]
    fn a_pair_and_its_burst_are_counted_apart_and_add_up() {
        let mut det = CollisionDetector::new(WIDE);
        let t0 = Instant::now();

        det.feed_at(t0, key_a(), plain());
        // The pair: a quarantined message and a newcomer inside the window.
        det.feed_at(t0 + Duration::from_micros(200), key_a(), plain());
        // Two more arrivals while the window slides, each killing itself alone.
        det.feed_at(t0 + Duration::from_micros(400), key_a(), plain());
        det.feed_at(t0 + Duration::from_micros(600), key_a(), plain());

        let stats = det.stats();
        assert_eq!(stats.pairs, 1);
        assert_eq!(stats.bursts, 2);
        assert_eq!(stats.events(), 3, "three collisions, four messages");
        assert_eq!(det.dropped_count(), 4);
        assert_eq!(stats.dropped(), det.dropped_count());
    }

    /// The boundaries are what the buckets mean, so they are pinned at the
    /// edges rather than in the middle of each band.
    #[test]
    fn the_gap_buckets_split_the_radio_from_the_reader() {
        let mut det = CollisionDetector::new(WIDE);
        let t0 = Instant::now();

        // Each pair is its own key, so every one is a fresh `pairs` event
        // rather than a burst continuation, and the gap is the one under test.
        for (i, gap) in [
            Duration::from_micros(999),
            Duration::from_millis(1),
            Duration::from_micros(4999),
            Duration::from_millis(5),
            WIDE - Duration::from_micros(1),
        ]
        .into_iter()
        .enumerate()
        {
            let key = DeviceKey {
                device_number: 2000 + u32::try_from(i).unwrap(),
                device_type_id: 11,
            };
            det.feed_at(t0, key, plain());
            det.feed_at(t0 + gap, key, plain());
        }

        let stats = det.stats();
        assert_eq!(stats.gap_under_1ms, 1, "only the sub-millisecond pair");
        assert_eq!(stats.gap_1_to_5ms, 2, "1ms is in, 5ms is out");
        assert_eq!(
            stats.gap_over_5ms, 2,
            "5ms and everything up to the threshold"
        );
        assert_eq!(stats.pairs, 5);
        assert_eq!(stats.events(), stats.pairs);
    }

    /// A dongle reopen throws away quarantine state, not the record of what
    /// this process has discarded. `dropped_count` already survives it.
    #[test]
    fn a_reset_keeps_the_stats_like_it_keeps_the_count() {
        let mut det = CollisionDetector::new(WIDE);
        let t0 = Instant::now();
        det.feed_at(t0, key_a(), plain());
        det.feed_at(t0 + Duration::from_micros(100), key_a(), plain());

        det.reset();

        assert_eq!(det.stats().pairs, 1);
        assert_eq!(det.stats().gap_under_1ms, 1);
        assert_eq!(det.dropped_count(), 2);
    }
}
