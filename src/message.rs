use ant::messages::AntMessage;
use ant::messages::RxMessage;
use ant::messages::data::BroadcastData;
use packed_struct::PackedStruct;
use packed_struct::types::SizedInteger;
use std::error::Error;
use std::fmt;

#[derive(Hash, Eq, PartialEq, Clone, Copy, Debug, Default)]
pub struct DeviceKey {
    pub device_number: u16,
    pub device_type_id: u8,
}

impl DeviceKey {
    pub fn from_broadcast(brd: &BroadcastData) -> Option<Self> {
        let chan_id = brd.extended_info?.channel_id_output?;
        Some(Self {
            device_number: chan_id.device_number,
            device_type_id: chan_id.device_type.device_type_id.to_primitive(),
        })
    }
}

impl fmt::Display for DeviceKey {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}:{}", self.device_number, self.device_type_id)
    }
}

pub fn serialize_broadcast(msg: &AntMessage, out: &mut Vec<u8>) -> Result<(), Box<dyn Error>> {
    match msg.message {
        RxMessage::BroadcastData(brd) => {
            out.clear();
            out.extend(msg.header.pack()?);
            out.extend(brd.payload.pack()?);
            let ext_info = brd.extended_info.ok_or("missing extended info")?;
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
        _ => Err("Only broadcast data is supported".into()),
    }
}
