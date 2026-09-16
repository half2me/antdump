use ant::drivers::Driver;
use ant::messages::RxMessage;
use antdump::collision::CollisionDetector;
use antdump::message::{DeviceKey, serialize_broadcast};
use antdump::tcp::TcpWriter;
use antdump::usb::{BringUp, Dongle, INIT_ATTEMPTS, bring_up};
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

fn init_driver() -> Dongle {
    let report =
        |attempt, err: &_| eprintln!("ERROR: {err} (attempt {attempt} of {INIT_ATTEMPTS})");
    match bring_up(INIT_ATTEMPTS, report) {
        Ok(BringUp { driver, lib_config }) => {
            if let Some(warning) = lib_config.warning() {
                eprintln!("WARNING: {warning}");
            }
            driver
        }
        Err(err) => {
            // Exiting is the honest outcome: a supervisor can restart us, and a
            // restart now stands a chance because each attempt reset the device
            // properly.
            eprintln!("FATAL: {err}. Unplug the dongle and plug it back in.");
            std::process::exit(1);
        }
    }
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
