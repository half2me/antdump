use crate::message::DeviceKey;
use std::collections::{HashMap, HashSet};
use std::io;
use std::io::Write;
use std::net::{TcpStream, ToSocketAddrs};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

pub trait StreamWrite: Send {
    fn write_all(&mut self, data: &[u8]) -> io::Result<()>;
}

pub struct DurableTcpStream {
    addr: String,
    hello: Option<String>,
    stream: Arc<Mutex<TcpStream>>,
    connected: Arc<Mutex<bool>>,
}

impl DurableTcpStream {
    fn establish_connection(addr: &str, hello: Option<&str>) -> TcpStream {
        loop {
            println!("connecting to server: {addr:?}");
            let result = TcpStream::connect_timeout(
                &addr.to_socket_addrs().unwrap().next().unwrap(),
                Duration::from_secs(10),
            );

            match result {
                Ok(mut stream) => {
                    stream
                        .set_write_timeout(Some(Duration::from_secs(10)))
                        .unwrap();
                    if let Some(hello) = hello {
                        println!("sending hello: {hello:?}");
                        if let Err(e) = writeln!(stream, "{hello}") {
                            println!("Error sending hello: {e:?}");
                            thread::sleep(Duration::from_secs(5));
                            continue;
                        }
                    }
                    return stream;
                }
                Err(e) => {
                    println!("Error connecting to server: {e:?}");
                    thread::sleep(Duration::from_secs(5));
                }
            }
        }
    }

    pub fn connect(addr: String, hello: Option<String>) -> Self {
        let stream = Self::establish_connection(&addr, hello.as_deref());
        Self {
            addr,
            hello,
            stream: Arc::new(Mutex::new(stream)),
            connected: Arc::new(Mutex::new(true)),
        }
    }
}

impl StreamWrite for DurableTcpStream {
    fn write_all(&mut self, data: &[u8]) -> io::Result<()> {
        if !*self.connected.lock().unwrap() {
            return Err(io::Error::other("Reconnecting"));
        }
        match self.stream.lock().unwrap().write_all(data) {
            Ok(()) => Ok(()),
            Err(e) => {
                println!("Error writing to server: {e:?}");
                *self.connected.lock().unwrap() = false;

                let stream = self.stream.clone();
                let connected = self.connected.clone();
                let addr = self.addr.clone();
                let hello = self.hello.clone();
                thread::spawn(move || {
                    *stream.lock().unwrap() =
                        DurableTcpStream::establish_connection(&addr, hello.as_deref());
                    *connected.lock().unwrap() = true;
                });
                Err(e)
            }
        }
    }
}

struct WriteQueue {
    keys: Vec<DeviceKey>,
    pending: HashSet<DeviceKey>,
}

impl WriteQueue {
    fn new() -> Self {
        Self {
            keys: Vec::new(),
            pending: HashSet::new(),
        }
    }

    /// Add a key to the queue. Returns true if newly added, false if already queued (leak).
    fn mark_dirty(&mut self, key: DeviceKey) -> bool {
        if self.pending.insert(key) {
            self.keys.push(key);
            true
        } else {
            false
        }
    }

    /// Swap out the queued keys for processing. Clears the pending set.
    fn take(&mut self, out: &mut Vec<DeviceKey>) {
        std::mem::swap(&mut self.keys, out);
        self.pending.clear();
    }
}

/// Non-blocking writer with a leaky per-device buffer.
/// Each device keeps only its latest serialized message; newer packets
/// overwrite older unsent ones.
pub struct TcpWriter {
    buffers: Arc<Mutex<HashMap<DeviceKey, Vec<u8>>>>,
    queue: Arc<Mutex<WriteQueue>>,
}

impl TcpWriter {
    pub fn spawn(addr: String, hello: Option<String>) -> Self {
        let stream = DurableTcpStream::connect(addr, hello);
        Self::spawn_with_stream(Box::new(stream))
    }

    pub fn spawn_with_stream(mut stream: Box<dyn StreamWrite>) -> Self {
        let buffers: Arc<Mutex<HashMap<DeviceKey, Vec<u8>>>> = Arc::new(Mutex::new(HashMap::new()));
        let queue = Arc::new(Mutex::new(WriteQueue::new()));

        let writer = Self {
            buffers: buffers.clone(),
            queue: queue.clone(),
        };

        thread::spawn(move || {
            let mut local_keys = Vec::new();
            let mut send_buf = Vec::new();

            loop {
                {
                    queue.lock().unwrap().take(&mut local_keys);
                }

                if local_keys.is_empty() {
                    thread::sleep(Duration::from_millis(1));
                    continue;
                }

                for key in local_keys.drain(..) {
                    {
                        let bufs = buffers.lock().unwrap();
                        if let Some(buf) = bufs.get(&key) {
                            send_buf.clear();
                            send_buf.extend_from_slice(buf);
                        } else {
                            continue;
                        }
                    }
                    if let Err(e) = stream.write_all(&send_buf) {
                        println!("Msg skipped for {key}: {e:?}");
                    }
                }
            }
        });

        writer
    }

    /// Queue serialized data for a device. Non-blocking; overwrites any
    /// previous unsent data for this device.
    pub fn send(&self, key: DeviceKey, data: &[u8]) {
        {
            let mut bufs = self.buffers.lock().unwrap();
            let buf = bufs.entry(key).or_default();
            buf.clear();
            buf.extend_from_slice(data);
        }
        {
            let mut q = self.queue.lock().unwrap();
            if !q.mark_dirty(key) {
                println!("WARNING: Dropped unsent packet for {key} (TCP too slow)");
            }
        }
    }
}
