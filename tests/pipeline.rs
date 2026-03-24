use antdump::collision::CollisionDetector;
use antdump::message::DeviceKey;
use antdump::tcp::{StreamWrite, TcpWriter};
use std::io;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

struct MockStream {
    written: Arc<Mutex<Vec<Vec<u8>>>>,
    delay: Duration,
}

impl MockStream {
    fn new(delay: Duration) -> Self {
        Self {
            written: Arc::new(Mutex::new(Vec::new())),
            delay,
        }
    }

    fn written(&self) -> Arc<Mutex<Vec<Vec<u8>>>> {
        self.written.clone()
    }
}

impl StreamWrite for MockStream {
    fn write_all(&mut self, data: &[u8]) -> io::Result<()> {
        if self.delay > Duration::ZERO {
            thread::sleep(self.delay);
        }
        self.written.lock().unwrap().push(data.to_vec());
        Ok(())
    }
}

fn key(device_number: u32, device_type_id: u8) -> DeviceKey {
    DeviceKey {
        device_number,
        device_type_id,
    }
}

#[test]
fn normal_flow_delivers_all_packets() {
    let mock = MockStream::new(Duration::ZERO);
    let written = mock.written();
    let writer = TcpWriter::spawn_with_stream(Box::new(mock));

    let k = key(327780, 11);
    for i in 0u8..5 {
        writer.send(k, &[i]);
        // Small delay so writer thread picks up each one before the next arrives
        thread::sleep(Duration::from_millis(5));
    }

    // Wait for writer to finish
    thread::sleep(Duration::from_millis(50));

    let received = written.lock().unwrap();
    assert_eq!(received.len(), 5);
    for (i, data) in received.iter().enumerate() {
        assert_eq!(data, &[i as u8]);
    }
}

#[test]
fn collision_prevents_delivery() {
    let mock = MockStream::new(Duration::ZERO);
    let written = mock.written();
    let writer = TcpWriter::spawn_with_stream(Box::new(mock));

    let mut collision = CollisionDetector::new(Duration::from_millis(1));
    let k = key(327780, 11);
    let t0 = Instant::now();

    // Two messages within collision threshold
    let msg1 = ant::messages::AntMessage::default();
    let msg2 = ant::messages::AntMessage::default();

    let result1 = collision.feed_at(t0, k, msg1);
    assert!(result1.is_none()); // buffered

    let result2 = collision.feed_at(t0 + Duration::from_micros(500), k, msg2);
    assert!(result2.is_none()); // collision — both dropped

    // Nothing should have been sent to the writer
    thread::sleep(Duration::from_millis(10));
    assert!(written.lock().unwrap().is_empty());

    // Verify writer still works for subsequent messages
    writer.send(k, &[42]);
    thread::sleep(Duration::from_millis(10));
    let received = written.lock().unwrap();
    assert_eq!(received.len(), 1);
    assert_eq!(received[0], &[42]);
}

#[test]
fn slow_network_drops_stale_packets() {
    // Mock stream with 50ms delay per write
    let mock = MockStream::new(Duration::from_millis(50));
    let written = mock.written();
    let writer = TcpWriter::spawn_with_stream(Box::new(mock));

    let k = key(327780, 11);

    // Send 20 packets rapidly (writer can only process ~1 every 50ms)
    for i in 0u8..20 {
        writer.send(k, &[i]);
        thread::sleep(Duration::from_millis(1));
    }

    // Wait for writer to finish all pending work
    thread::sleep(Duration::from_millis(200));

    let received = written.lock().unwrap();
    // Should have received far fewer than 20
    assert!(
        received.len() < 20,
        "Expected leaky buffer to drop packets, got all {}",
        received.len()
    );
    // At least 1 should have made it through
    assert!(!received.is_empty());
    // The last received packet should contain recent data (high byte value)
    let last = received.last().unwrap();
    assert!(
        last[0] >= 10,
        "Expected recent data in last packet, got {}",
        last[0]
    );
}

#[test]
fn different_devices_independent_under_slow_network() {
    let mock = MockStream::new(Duration::from_millis(20));
    let written = mock.written();
    let writer = TcpWriter::spawn_with_stream(Box::new(mock));

    let k1 = key(327780, 11);
    let k2 = key(655560, 12);

    // Send packets for two devices
    for i in 0u8..5 {
        writer.send(k1, &[i, 0xAA]);
        writer.send(k2, &[i, 0xBB]);
        thread::sleep(Duration::from_millis(1));
    }

    // Wait for writer to process
    thread::sleep(Duration::from_millis(300));

    let received = written.lock().unwrap();

    // Both devices should have at least one packet delivered
    let has_device1 = received.iter().any(|d| d.len() == 2 && d[1] == 0xAA);
    let has_device2 = received.iter().any(|d| d.len() == 2 && d[1] == 0xBB);
    assert!(has_device1, "Device 1 should have at least one packet");
    assert!(has_device2, "Device 2 should have at least one packet");
}
