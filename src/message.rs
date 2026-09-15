use ant::messages::AntMessage;
use ant::messages::RxMessage;
use ant::messages::data::{BroadcastData, RssiMeasurementValue};
use packed_struct::PackedStruct;
use packed_struct::PrimitiveEnum;
use packed_struct::types::SizedInteger;
use std::error::Error;
use std::fmt;

#[derive(Hash, Eq, PartialEq, Clone, Copy, Debug, Default)]
pub struct DeviceKey {
    /// Full 20-bit device number (device_number | device_number_extension << 16)
    pub device_number: u32,
    pub device_type_id: u8,
}

impl DeviceKey {
    pub fn from_broadcast(brd: &BroadcastData) -> Option<Self> {
        let chan_id = brd.extended_info?.channel_id_output?;
        let ext = chan_id
            .transmission_type
            .device_number_extension
            .to_primitive() as u32;
        let full_device_number = chan_id.device_number as u32 | (ext << 16);
        Some(Self {
            device_number: full_device_number,
            device_type_id: chan_id.device_type.device_type_id.to_primitive(),
        })
    }
}

/// The dongle's own RX timestamp, in u16 ticks of its 32768 Hz clock. Absent
/// unless `LibConfig` enabled it.
pub fn rx_timestamp(msg: &AntMessage) -> Option<u16> {
    match &msg.message {
        RxMessage::BroadcastData(brd) => Some(brd.extended_info?.timestamp_output?.rx_timestamp),
        _ => None,
    }
}

impl fmt::Display for DeviceKey {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}:{}", self.device_number, self.device_type_id)
    }
}

/// Rebuild a broadcast's wire bytes for the TCP forward.
///
/// Every block the flag byte announces has to be written, in the order the
/// dongle sent them: the flag byte and the header's length are copied from the
/// original, so an announced block that is missing leaves the reader parsing the
/// checksum as payload.
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
            if let Some(rssi) = ext_info.rssi_output {
                // The measurement type decides the block's length (dBm 3 bytes,
                // AGC 4), which shifts the timestamp block behind it.
                out.push(rssi.measurement_type.to_primitive());
                match rssi.measurement_value {
                    RssiMeasurementValue::Dbm(v) => out.extend(v.pack()?),
                    RssiMeasurementValue::Agc(v) => out.extend(v.pack()?),
                }
            }
            if let Some(timestamp) = ext_info.timestamp_output {
                out.extend(timestamp.pack()?);
            }
            out.push(msg.checksum);
            Ok(())
        }
        _ => Err("Only broadcast data is supported".into()),
    }
}
