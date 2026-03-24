use crate::message::DeviceKey;
use ant::messages::AntMessage;
use std::collections::HashMap;
use std::time::{Duration, Instant};

pub struct CollisionDetector {
    threshold: Duration,
    device_state: HashMap<DeviceKey, (Instant, Option<AntMessage>)>,
}

impl CollisionDetector {
    pub fn new(threshold: Duration) -> Self {
        Self {
            threshold,
            device_state: HashMap::new(),
        }
    }

    pub fn is_disabled(&self) -> bool {
        self.threshold == Duration::ZERO
    }

    /// Feed a message into the detector. Returns the previously buffered message
    /// if it survived the collision window. Returns None if the message was
    /// buffered or both messages were dropped due to collision.
    pub fn feed(&mut self, key: DeviceKey, msg: AntMessage) -> Option<AntMessage> {
        self.feed_at(Instant::now(), key, msg)
    }

    /// Like `feed`, but with an explicit timestamp for testability.
    pub fn feed_at(&mut self, now: Instant, key: DeviceKey, msg: AntMessage) -> Option<AntMessage> {
        if let Some((last_time, pending)) = self.device_state.get_mut(&key) {
            if now.duration_since(*last_time) < self.threshold {
                println!(
                    "WARNING: Collision on {} ({:.3}ms apart), dropping messages",
                    key,
                    now.duration_since(*last_time).as_secs_f64() * 1000.0
                );
                *pending = None;
                *last_time = now;
                return None;
            }

            let flushed = pending.take();
            *last_time = now;
            *pending = Some(msg);
            flushed
        } else {
            self.device_state.insert(key, (now, Some(msg)));
            None
        }
    }

    /// Flush messages that have survived past the collision window.
    pub fn flush_expired(&mut self) -> Vec<(DeviceKey, AntMessage)> {
        self.flush_expired_at(Instant::now())
    }

    /// Like `flush_expired`, but with an explicit timestamp for testability.
    pub fn flush_expired_at(&mut self, now: Instant) -> Vec<(DeviceKey, AntMessage)> {
        let threshold = self.threshold;

        self.device_state
            .iter_mut()
            .filter(|(_, (last_time, pending))| {
                pending.is_some() && now.duration_since(*last_time) >= threshold
            })
            .map(|(key, (_, pending))| (*key, pending.take().unwrap()))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const THRESHOLD: Duration = Duration::from_millis(1);

    fn key_a() -> DeviceKey {
        DeviceKey {
            device_number: 100,
            device_type_id: 11,
        }
    }

    fn key_b() -> DeviceKey {
        DeviceKey {
            device_number: 200,
            device_type_id: 12,
        }
    }

    #[test]
    fn first_message_is_buffered() {
        let mut det = CollisionDetector::new(THRESHOLD);
        let t0 = Instant::now();

        // First message for a device is always buffered, not returned
        let result = det.feed_at(t0, key_a(), AntMessage::default());
        assert!(result.is_none());
    }

    #[test]
    fn message_flushed_when_next_arrives_outside_threshold() {
        let mut det = CollisionDetector::new(THRESHOLD);
        let t0 = Instant::now();
        let t1 = t0 + Duration::from_millis(5);

        det.feed_at(t0, key_a(), AntMessage::default());

        // Second message arrives well after threshold — first should be flushed
        let result = det.feed_at(t1, key_a(), AntMessage::default());
        assert!(result.is_some());
    }

    #[test]
    fn collision_drops_both_messages() {
        let mut det = CollisionDetector::new(THRESHOLD);
        let t0 = Instant::now();
        let t1 = t0 + Duration::from_micros(500); // within 1ms threshold

        det.feed_at(t0, key_a(), AntMessage::default());

        // Second message arrives within threshold — collision
        let result = det.feed_at(t1, key_a(), AntMessage::default());
        assert!(result.is_none());

        // Third message arrives well after — nothing buffered to flush
        let t2 = t1 + Duration::from_millis(5);
        let result = det.feed_at(t2, key_a(), AntMessage::default());
        assert!(result.is_none()); // only buffered, no prior pending
    }

    #[test]
    fn different_devices_do_not_collide() {
        let mut det = CollisionDetector::new(THRESHOLD);
        let t0 = Instant::now();
        let t1 = t0 + Duration::from_micros(100); // within threshold, but different device

        det.feed_at(t0, key_a(), AntMessage::default());
        det.feed_at(t1, key_b(), AntMessage::default());

        // Both should be pending, neither dropped. Flush after threshold.
        let t2 = t1 + Duration::from_millis(5);
        let flushed = det.flush_expired_at(t2);
        assert_eq!(flushed.len(), 2);
    }

    #[test]
    fn flush_expired_returns_messages_past_threshold() {
        let mut det = CollisionDetector::new(THRESHOLD);
        let t0 = Instant::now();

        det.feed_at(t0, key_a(), AntMessage::default());

        // Before threshold — nothing to flush
        let flushed = det.flush_expired_at(t0 + Duration::from_micros(500));
        assert!(flushed.is_empty());

        // After threshold — message is flushed
        let flushed = det.flush_expired_at(t0 + Duration::from_millis(2));
        assert_eq!(flushed.len(), 1);
        assert_eq!(flushed[0].0, key_a());

        // Calling again — nothing left
        let flushed = det.flush_expired_at(t0 + Duration::from_millis(5));
        assert!(flushed.is_empty());
    }

    #[test]
    fn disabled_when_zero_threshold() {
        let det = CollisionDetector::new(Duration::ZERO);
        assert!(det.is_disabled());

        let det = CollisionDetector::new(Duration::from_millis(1));
        assert!(!det.is_disabled());
    }
}
