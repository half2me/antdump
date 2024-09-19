use ant::drivers::*;
use ant::messages::config::{
    AssignChannel, ChannelId, ChannelRfFrequency, ChannelType, DeviceType, EnableExtRxMessages,
    SetNetworkKey, TransmissionType,
};
use ant::messages::control::{OpenRxScanMode, ResetSystem};
use ant::messages::{AntMessage, RxMessage};
use clap::Parser;
use packed_struct::PackedStruct;
use rusb::{Device, DeviceList};
use std::error::Error;
use std::io;
use std::io::Write;
use std::net::{TcpStream, ToSocketAddrs};
use std::sync::{Arc, Mutex};
use std::thread;
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
}

pub struct DurableTCPStream {
    addr: String,
    hello: Option<String>,
    stream: Arc<Mutex<TcpStream>>,
    connected: Arc<Mutex<bool>>,
}

impl DurableTCPStream {
    fn establish_connection(addr: String, hello: Option<String>) -> TcpStream {
        loop {
            println!("connecting to server: {:?}", addr);
            let stream = TcpStream::connect_timeout(
                &addr.to_socket_addrs().unwrap().next().unwrap(),
                Duration::from_secs(10),
            );
            match stream {
                Ok(mut stream) => {
                    stream
                        .set_write_timeout(Some(Duration::from_secs(10)))
                        .unwrap();
                    match hello {
                        Some(ref hello) => {
                            println!("sending hello:: {:?}", hello);
                            let r = stream.write_all(format!("{}\n", hello).as_bytes());
                            match r {
                                Ok(_) => return stream,
                                Err(why) => {
                                    println!("Error connecting to server: {:?}", why);
                                    thread::sleep(Duration::from_secs(5));
                                    continue;
                                }
                            }
                        }
                        None => return stream,
                    }
                }
                Err(why) => {
                    println!("Error connecting to server: {:?}", why);
                    thread::sleep(Duration::from_secs(5));
                    continue;
                }
            }
        }
    }

    pub fn connect(addr: String, hello: Option<String>) -> Self {
        let stream = Self::establish_connection(addr.clone(), hello.clone());
        Self {
            hello,
            addr,
            stream: Arc::new(Mutex::new(stream)),
            connected: Arc::new(Mutex::new(true)),
        }
    }

    pub fn write_all(&mut self, data: &[u8]) -> io::Result<()> {
        if !*self.connected.lock().unwrap() {
            return Err(io::Error::new(io::ErrorKind::Other, "Reconnecting"));
        }
        match self.stream.lock().unwrap().write_all(data) {
            Ok(_) => Ok(()),
            Err(why) => {
                println!("Error writing to server: {:?}", why);
                *self.connected.lock().unwrap() = false;

                // reconnect
                let stream = self.stream.clone();
                let connected = self.connected.clone();
                let addr = self.addr.clone();
                let hello = self.hello.clone();
                thread::spawn(move || {
                    *stream.lock().unwrap() = DurableTCPStream::establish_connection(
                        addr,
                        hello,
                    );
                    *connected.lock().unwrap() = true
                });
                Err(why)
            }
        }
    }
}

fn main() -> io::Result<()> {
    let args = Args::parse();

    let mut stream = match args.server {
        Some(url) => Some(DurableTCPStream::connect(url, args.hello_msg)),
        None => None,
    };

    let devices: Vec<Device<_>> = DeviceList::new()
        .expect("Unable to lookup usb devices")
        .iter()
        .filter(|x| is_ant_usb_device_from_device(x))
        .collect();

    let device = devices.into_iter().nth(0).expect("No ANT+ dongle found");
    let mut driver = UsbDriver::new(device).expect("Unable to initialize driver");

    // open RX Scan mode
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
    loop {
        match driver.get_message() {
            Ok(None) => (),
            Ok(Some(msg)) => match &msg.message {
                RxMessage::BroadcastData(brd) => {
                    let mut raw = vec![];
                    to_slice(&msg, &mut raw).unwrap();
                    println!("[{}] {:02X?}", brd.payload.channel_number, brd.payload.data);
                    if let Some(stream) = &mut stream {
                        match stream.write_all(raw.as_slice()) {
                            Ok(_) => {}
                            Err(e) => {
                                println!("Msg skipped: {:?}", e);
                            }
                        }
                    }
                }
                _msg => println!("Got: {:#?}", _msg),
            },
            msg => panic!("Error: {:#?}", msg),
        }
    }
}

fn to_slice(msg: &AntMessage, out: &mut Vec<u8>) -> Result<(), Box<dyn Error>> {
    match msg.message {
        RxMessage::BroadcastData(msg2) => {
            out.extend(msg.header.pack()?);
            out.extend(msg2.payload.pack()?);
            let ext_info = msg2.extended_info.ok_or("missing extended info")?;
            out.extend(ext_info.flag_byte.pack()?);
            out.extend(
                ext_info
                    .channel_id_output
                    .ok_or("missing channel id")?
                    .pack()?,
            );
            out.push(msg.checksum);
            Ok(())
        }
        _ => Err("Only broadcast data is supported for now".into()),
    }
}
