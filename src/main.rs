use ant::drivers::*;
use ant::messages::RxMessage;
use ant::messages::config::{
    AssignChannel, ChannelId, ChannelRfFrequency, ChannelType, DeviceType, EnableExtRxMessages,
    SetNetworkKey, TransmissionType,
};
use ant::messages::control::{OpenRxScanMode, ResetSystem};
use antdump::collision::CollisionDetector;
use antdump::message::{DeviceKey, serialize_broadcast};
use antdump::tcp::TcpWriter;
use clap::Parser;
use std::io;
use std::time::Duration;

const NETWORK_KEY: [u8; 8] = [0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45];
const RF_FREQ: u8 = 57;

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

fn init_driver() -> UsbDriver<rusb::GlobalContext> {
    let device = rusb::DeviceList::new()
        .expect("Unable to lookup usb devices")
        .iter()
        .find(is_ant_usb_device_from_device)
        .expect("No ANT+ dongle found");

    let mut driver = UsbDriver::new(device).expect("Unable to initialize driver");

    driver.send_message(&ResetSystem::new()).unwrap();
    driver
        .send_message(&SetNetworkKey::new(0, NETWORK_KEY))
        .unwrap();
    driver
        .send_message(&AssignChannel::new(
            0,
            ChannelType::SharedReceiveOnly,
            0,
            None,
        ))
        .unwrap();
    driver
        .send_message(&ChannelId::new(
            0,
            0,
            DeviceType::new(0.into(), false),
            TransmissionType::new_wildcard(),
        ))
        .unwrap();
    driver
        .send_message(&ChannelRfFrequency::new(0, RF_FREQ))
        .unwrap();
    driver
        .send_message(&EnableExtRxMessages::new(true))
        .unwrap();
    driver
        .send_message(&OpenRxScanMode {
            synchronous_channel_packets_only: None,
        })
        .unwrap();

    driver
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
