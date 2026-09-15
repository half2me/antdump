use ant::drivers::*;
use ant::messages::RxMessage;
use antdump::collision::CollisionDetector;
use antdump::init::{InitError, LibConfigOutcome, configure};
use antdump::message::{DeviceKey, serialize_broadcast};
use antdump::tcp::TcpWriter;
use clap::Parser;
use std::io;
use std::time::Duration;

/// Dump ANT+ data from the air
#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    /// Address to a TCP server to connect to and send the data
    #[arg(long)]
    server: Option<String>,

    /// Optional hello message to send to the TCP server before sending ANT+ messages
    /// a newline character will also be sent after the hello msg
    #[arg(long)]
    hello_msg: Option<String>,

    /// Collision detection threshold in milliseconds. Messages arriving closer together
    /// than this are considered collisions and dropped. Set to 0 to disable.
    #[arg(long, default_value_t = 1.0)]
    collision_threshold_ms: f64,

    /// Only show warnings (collisions, dropped packets, errors). Suppress per-packet output.
    #[arg(long, short)]
    quiet: bool,
}

/// How many times to bring the dongle up before giving up. A stale handle is
/// cleared by a port reset and a fresh look at the bus, which is what the
/// retry does; more than a couple of failures is a dongle that needs a human.
const INIT_ATTEMPTS: u32 = 3;

/// A port reset re-enumerates the device, so it is gone from the bus for a
/// moment and has to be found again rather than reused.
const REENUMERATE_DELAY: Duration = Duration::from_millis(500);

fn find_dongle() -> Option<rusb::Device<rusb::GlobalContext>> {
    rusb::DeviceList::new()
        .ok()?
        .iter()
        .find(is_ant_usb_device_from_device)
}

/// Reset the dongle at the USB level and let it re-enumerate.
///
/// `UsbDriver::new` resets the handle it then claims the interface on, and a
/// reset that re-enumerates invalidates that handle: libusb's own answer is to
/// close it and rediscover the device, which nothing did. That is the state a
/// restarted process inherited, where every write succeeded and no answer ever
/// came back and only unplugging the stick cleared it.
fn reset_dongle() {
    if let Some(device) = find_dongle()
        && let Ok(handle) = device.open()
    {
        let _ = handle.reset();
    }
    // The handle is dropped here, which is the half that was missing.
    std::thread::sleep(REENUMERATE_DELAY);
}

fn open_dongle() -> Result<UsbDriver<rusb::GlobalContext>, InitError> {
    let device = find_dongle().ok_or(InitError::Write {
        message: "no ANT+ dongle found",
    })?;
    UsbDriver::new(device).map_err(|_| InitError::Write {
        message: "the dongle could not be opened",
    })
}

fn init_driver() -> UsbDriver<rusb::GlobalContext> {
    for attempt in 1..=INIT_ATTEMPTS {
        let mut driver = match open_dongle() {
            Ok(driver) => driver,
            Err(err) => {
                eprintln!("ERROR: {err}");
                reset_dongle();
                continue;
            }
        };

        match configure(&mut driver) {
            Ok(outcome) => {
                match outcome {
                    LibConfigOutcome::Accepted => (),
                    LibConfigOutcome::Rejected(code) => eprintln!(
                        "WARNING: the dongle rejected LibConfig ({code:?}); no RSSI or RX timestamps, and collision detection falls back to wall-clock timing"
                    ),
                    LibConfigOutcome::Unanswered => eprintln!(
                        "WARNING: the dongle did not answer LibConfig; no RSSI or RX timestamps, and collision detection falls back to wall-clock timing"
                    ),
                }
                return driver;
            }
            Err(err) => {
                eprintln!("ERROR: {err} (attempt {attempt} of {INIT_ATTEMPTS})");
                drop(driver);
                reset_dongle();
            }
        }
    }

    // Exiting is the honest outcome: a supervisor can restart us, and a restart
    // now stands a chance because each attempt above reset the device properly.
    eprintln!("FATAL: the dongle never answered. Unplug it and plug it back in.");
    std::process::exit(1);
}

fn main() -> io::Result<()> {
    let args = Args::parse();
    let writer = args.server.map(|url| TcpWriter::spawn(url, args.hello_msg));
    let quiet = args.quiet;
    let mut driver = init_driver();
    let mut collision = CollisionDetector::new(Duration::from_secs_f64(
        args.collision_threshold_ms / 1000.0,
    ));
    let mut raw = Vec::new();

    loop {
        for (key, msg) in collision.flush_expired() {
            handle_broadcast(&key, &msg, &writer, &mut raw, quiet);
        }

        match driver.get_message() {
            Ok(None) => (),
            Ok(Some(msg)) => match &msg.message {
                RxMessage::BroadcastData(brd) => match DeviceKey::from_broadcast(brd) {
                    Some(key) if !collision.is_disabled() => {
                        if let Some(flushed) = collision.feed(key, msg) {
                            handle_broadcast(&key, &flushed, &writer, &mut raw, quiet);
                        }
                    }
                    key_opt => {
                        let key = key_opt.unwrap_or_default();
                        handle_broadcast(&key, &msg, &writer, &mut raw, quiet);
                    }
                },
                other if !quiet => println!("Got: {other:#?}"),
                _ => (),
            },
            msg => panic!("Error: {msg:#?}"),
        }
    }
}

fn handle_broadcast(
    key: &DeviceKey,
    msg: &ant::messages::AntMessage,
    writer: &Option<TcpWriter>,
    raw: &mut Vec<u8>,
    quiet: bool,
) {
    if let RxMessage::BroadcastData(brd) = &msg.message {
        if !quiet {
            println!(
                "[{key}] [{}] {:02X?}",
                brd.payload.channel_number, brd.payload.data
            );
        }
        if let Some(writer) = writer
            && serialize_broadcast(msg, raw).is_ok()
        {
            writer.send(*key, raw);
        }
    }
}
