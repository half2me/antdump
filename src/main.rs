use std::error::Error;
use ant::drivers::*;
use ant::messages::{RxMessage, AntMessage};
use ant::messages::config::{ChannelId, ChannelRfFrequency, ChannelType, DeviceType, EnableExtRxMessages, SetNetworkKey, AssignChannel, TransmissionType};
use ant::messages::control::{ResetSystem, OpenRxScanMode};
use rusb::{Device, DeviceList};
use packed_struct::PackedStruct;

const NETWORK_KEY: [u8; 8] = [0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45];
const RF_FREQ: u8 = 57;

fn main() -> std::io::Result<()> {
    //let mut stream = TcpStream::connect("127.0.0.1:9999").unwrap();

    let devices: Vec<Device<_>> = DeviceList::new()
        .expect("Unable to lookup usb devices")
        .iter()
        .filter(|x| is_ant_usb_device_from_device(x))
        .collect();

    let device = devices.into_iter().nth(0).unwrap();
    let mut driver = UsbDriver::new(device).unwrap();

    // open RX Scan mode
    driver.send_message(&ResetSystem::new()).unwrap();
    driver.send_message(&SetNetworkKey::new(0, NETWORK_KEY)).unwrap();
    driver.send_message(&AssignChannel::new(0, ChannelType::SharedReceiveOnly, 0, None)).unwrap();
    driver.send_message(&ChannelId::new(
        0,
        0,
        DeviceType::new(0.into(), false),
        TransmissionType::new_wildcard(),
    )).unwrap();
    driver.send_message(&ChannelRfFrequency::new(0, RF_FREQ)).unwrap();
    driver.send_message(&EnableExtRxMessages::new(true)).unwrap();
    driver.send_message(&OpenRxScanMode { synchronous_channel_packets_only: None }).unwrap();
    loop {
        match driver.get_message() {
            Ok(None) => (),
            Ok(Some(msg)) => match &msg.message {
                RxMessage::BroadcastData(_) => {
                    let mut raw = vec![];
                    raw_data(&msg, &mut raw).unwrap();
                    println!("{:02X?}", raw);
                }
                //_msg => println!("Got: {:#?}", _msg),
                _msg => (),
            },
            msg => panic!("Error: {:#?}", msg),
        }
    }
}

fn raw_data(msg: &AntMessage, out: &mut Vec<u8>) -> Result<(), Box<dyn Error>> {
    match msg.message {
        RxMessage::BroadcastData(msg2) => {
            out.extend(msg.header.pack()?);
            out.extend(msg2.payload.pack()?);
            match msg2.extended_info {
                Some(ext_info) => {
                    out.extend(ext_info.flag_byte.pack()?);
                    match ext_info.channel_id_output {
                        Some(chan_id) => {
                            out.extend(chan_id.pack()?);
                        }
                        None => {
                            return Err("missing channel id".into())
                        }
                    }

                }
                None => {
                    return Err("missing extended info".into())
                }
            }

            out.push(msg.checksum);
            Ok(())
        }
        _ => Err("Only broadcast data is supported for now".into())
    }
}