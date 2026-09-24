use ant::drivers::Driver;
use ant::messages::RxMessage;
use antdump::collision::CollisionDetector;
use antdump::message::{DeviceKey, serialize_broadcast};
use antdump::tcp::TcpWriter;
use antdump::usb::{Dongle, INIT_ATTEMPTS, bring_up, list_dongles};
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
    ///
    /// A venue capture puts the normal ANT+ cadence at the 200-300 ms channel
    /// period and collisions under 1 ms, so anything in between separates them.
    /// Erring high is the cheap direction: profiles carry cumulative counters, so
    /// a legitimate message dropped here costs nothing, while a garbled one
    /// admitted costs a wrong number nothing downstream can catch.
    #[arg(long, default_value_t = 25.0)]
    collision_threshold_ms: f64,

    /// Only show warnings (collisions, dropped packets, errors). Suppress per-packet output.
    #[arg(long, short)]
    quiet: bool,

    /// Which dongle to use when several are plugged in: its USB serial or its
    /// port as `--list-dongles` prints them. Default: the first found.
    #[arg(long)]
    dongle: Option<String>,

    /// Print every ANT+ dongle on the bus and exit.
    #[arg(long)]
    list_dongles: bool,
}

fn print_dongles() -> io::Result<()> {
    let dongles = list_dongles().map_err(io::Error::other)?;
    if dongles.is_empty() {
        eprintln!("no ANT+ dongle found");
        std::process::exit(1);
    }
    for dongle in dongles {
        println!("{dongle}");
    }
    Ok(())
}

fn init_driver(selector: Option<&str>) -> Dongle {
    let report =
        |attempt, err: &_| eprintln!("ERROR: {err} (attempt {attempt} of {INIT_ATTEMPTS})");
    match bring_up(INIT_ATTEMPTS, selector, report) {
        Ok(driver) => driver,
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
    if args.list_dongles {
        return print_dongles();
    }
    let writer = args.server.map(|url| TcpWriter::spawn(url, args.hello_msg));
    let quiet = args.quiet;
    let mut driver = init_driver(args.dongle.as_deref());
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
                        // The warning `--quiet` promises to keep. It lives here
                        // rather than in the detector because the library never
                        // prints, and because a write per collision inside the
                        // read loop is what makes a backlog look like the next
                        // collision. On stderr, since stdout carries the packet
                        // stream this may be piped out of.
                        let before = collision.dropped_count();
                        let flushed = collision.feed(key, msg);
                        if collision.dropped_count() != before {
                            eprintln!("WARNING: collision on {key}, dropping messages");
                        }
                        if let Some(flushed) = flushed {
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
