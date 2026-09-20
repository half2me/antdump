//! Simulating ANT+ sensors: a fleet of virtual bikes, transmitting for real.
//!
//! This is the mirror image of [`crate::init`]. That module opens one
//! promiscuous receiver; this one opens up to eight transmitters per stick and
//! feeds them. Everything else is shared on purpose — the same network key, the
//! same RF frequency, the same confirm-every-step discipline — because a
//! simulator that disagrees with the receiver about any of those is not wrong,
//! it is silent, and silence is indistinguishable from a dongle fault.
//!
//! **Eight devices per dongle, and that is the radio's number.** Both stick
//! types this crate has seen (0fcf:1008 and 0fcf:1009) are nRF24AP2-USB parts,
//! which are eight-channel ANT network processors. There is no setting that
//! raises it, so a bigger fleet means more sticks: see [`CHANNELS_PER_DONGLE`].
//!
//! **One stick cannot collide with itself.** The ANT stack time-division
//! schedules the channels it owns, so eight masters on one dongle are staggered
//! deliberately and never overlap on the air. That makes a single stick the
//! right tool for checking that counters parse, and the wrong tool entirely for
//! exercising [`crate::collision`] — for that the transmissions have to come
//! from radios that do not know about each other, which means two dongles or
//! more.
//!
//! **The radio sets the pace, not us.** An open master channel transmits on its
//! own period and raises `EVENT_TX` when it has done so; the loop in [`run`]
//! answers each one with the next payload. So there is no timer here and no
//! sleeping: the dongle's own clock is what the fleet is paced by, which is
//! also why the packet counts are counts of transmissions that actually
//! happened rather than of payloads handed over.

use crate::init::{InitError, NETWORK_KEY, RF_FREQ, confirm, reset, send};
use crate::profile::{
    CSC_CHANNEL_PERIOD, CSC_DEVICE_TYPE, POWER_CHANNEL_PERIOD, POWER_DEVICE_TYPE, RevolutionState,
    Revolutions, csc_page, power_page,
};
use crate::usb::{BringUpError, BringUpOps, Dongle, DongleId, open_dongle, reset_dongle};
use ant::drivers::Driver;
use ant::messages::RxMessage;
use ant::messages::channel::MessageCode;
use ant::messages::config::{
    AssignChannel, ChannelId, ChannelPeriod, ChannelRfFrequency, ChannelType, DeviceType,
    SetNetworkKey, TransmissionChannelType, TransmissionGlobalDataPages, TransmissionType,
};
use ant::messages::control::OpenChannel;
use ant::messages::data::BroadcastData;
use std::fmt;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

/// The nRF24AP2-USB is an eight-channel part, so eight masters is the whole
/// stick. Asking for a ninth is not slow, it is refused.
pub const CHANNELS_PER_DONGLE: usize = 8;

/// ANT reserves device number 0 as the wildcard a searching slave uses to mean
/// "any". A master must not claim it, so the fleet starts at 1 at the lowest.
pub const MIN_DEVICE_NUMBER: u32 = 1;

/// Device numbers are 20 bits: 16 in the channel id, and the top 4 in the
/// transmission type's extension nibble.
pub const MAX_DEVICE_NUMBER: u32 = 0xF_FFFF;

/// Which ANT+ profile a simulated channel pretends to be.
///
/// Fitness equipment (type 17, period 8192) joins here, and the only other
/// thing it needs is its own page builder in [`crate::profile`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Profile {
    /// Combined bike speed and cadence.
    #[default]
    SpeedAndCadence,
    /// Bicycle power.
    Power,
}

/// A bike carrying both sensors, which is what a real one with a power meter
/// looks like on the air: two separate profiles, two separate channels, two
/// different channel periods.
pub const SPEED_CADENCE_AND_POWER: &[Profile] = &[Profile::SpeedAndCadence, Profile::Power];

/// Speed and cadence alone, which fits twice as many bikes on a stick.
pub const SPEED_CADENCE_ONLY: &[Profile] = &[Profile::SpeedAndCadence];

impl Profile {
    #[must_use]
    pub fn device_type(self) -> u8 {
        match self {
            Self::SpeedAndCadence => CSC_DEVICE_TYPE,
            Self::Power => POWER_DEVICE_TYPE,
        }
    }

    #[must_use]
    pub fn channel_period(self) -> u16 {
        match self {
            Self::SpeedAndCadence => CSC_CHANNEL_PERIOD,
            Self::Power => POWER_CHANNEL_PERIOD,
        }
    }

    /// How often the channel transmits, for display.
    #[must_use]
    pub fn hz(self) -> f64 {
        32768.0 / f64::from(self.channel_period())
    }
}

impl fmt::Display for Profile {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::SpeedAndCadence => write!(f, "speed&cadence"),
            Self::Power => write!(f, "power"),
        }
    }
}

/// What a device's counters read at some moment, for the status display.
/// Structured rather than formatted, because how it is laid out on a screen is
/// the display's business and which counters exist is the profile's.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DeviceState {
    SpeedAndCadence { wheel_revs: u16, crank_revs: u16 },
    Power { events: u8, accumulated: u16 },
}

/// One virtual bike.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SimDevice {
    /// The full 20-bit ANT+ device number, which is what `antdump` prints.
    pub device_number: u32,
    pub profile: Profile,
    pub speed_kph: f64,
    pub cadence_rpm: f64,
    pub power_watts: u16,
    wheel: Revolutions,
    crank: Revolutions,
}

impl SimDevice {
    #[must_use]
    pub fn new(
        device_number: u32,
        profile: Profile,
        speed_kph: f64,
        cadence_rpm: f64,
        power_watts: u16,
        wheel_circumference_m: f64,
    ) -> Self {
        Self {
            device_number,
            profile,
            speed_kph,
            cadence_rpm,
            power_watts,
            wheel: Revolutions::from_speed(speed_kph, wheel_circumference_m),
            crank: Revolutions::from_rpm(cadence_rpm),
        }
    }

    /// The eight payload bytes this device would broadcast at `elapsed`.
    #[must_use]
    pub fn page(&self, elapsed: Duration) -> [u8; 8] {
        match self.profile {
            Profile::SpeedAndCadence => csc_page(self.crank.at(elapsed), self.wheel.at(elapsed)),
            Profile::Power => {
                power_page(self.crank.at(elapsed), self.power_watts, self.cadence_rpm)
            }
        }
    }

    /// What its counters read at `elapsed`, for the status display.
    ///
    /// Both profiles are driven off the same crank, so a bike's power events
    /// and its cadence revolutions are the same number and can be read against
    /// each other: they are the one counter this simulator advances twice.
    #[must_use]
    pub fn state(&self, elapsed: Duration) -> DeviceState {
        let crank = self.crank.at(elapsed);
        match self.profile {
            Profile::SpeedAndCadence => DeviceState::SpeedAndCadence {
                wheel_revs: self.wheel.at(elapsed).revolutions,
                crank_revs: crank.revolutions,
            },
            Profile::Power => DeviceState::Power {
                events: (crank.revolutions % 256) as u8,
                accumulated: accumulated_power(crank, self.power_watts),
            },
        }
    }

    /// The channel id that puts this device on the air under its own number.
    ///
    /// The split is the part worth checking: the low 16 bits ride in the
    /// channel id and the top 4 in the transmission type's extension nibble,
    /// which is exactly how `DeviceKey::from_broadcast` puts them back
    /// together. A device numbered above 65535 therefore exercises a branch of
    /// the receiver that a fleet numbered below it never touches.
    #[must_use]
    pub fn channel_id(&self, channel: u8) -> ChannelId {
        ChannelId::new(
            channel,
            (self.device_number & 0xFFFF) as u16,
            DeviceType::new(self.profile.device_type().into(), false),
            TransmissionType::new(
                TransmissionChannelType::IndependentChannel,
                TransmissionGlobalDataPages::GlobalDataPagesNotUsed,
                (((self.device_number >> 16) & 0xF) as u8).into(),
            ),
        )
    }
}

/// The accumulator the power page carries, recomputed here so the display can
/// show the same number that went on the air.
fn accumulated_power(crank: RevolutionState, watts: u16) -> u16 {
    (u64::from(watts) * u64::from(crank.revolutions) % (1 << 16)) as u16
}

/// What to simulate, before it is turned into devices.
#[derive(Clone, Copy, Debug)]
pub struct FleetSpec {
    /// How many bikes, not how many channels. Each one occupies a channel per
    /// profile in [`Self::profiles`], so turning power on halves how many fit
    /// on a stick.
    pub devices: usize,
    pub start_id: u32,
    /// Which sensors each bike carries. All of them share the bike's device
    /// number and differ by device type, which is exactly how a real bike with
    /// a power meter appears and what makes `DeviceKey`'s type field earn its
    /// keep: keyed on the number alone, a bike's two streams would arrive
    /// interleaved at ~4 Hz each and false-collide continuously.
    pub profiles: &'static [Profile],
    pub speed_kph: f64,
    pub cadence_rpm: f64,
    /// How far the fastest device is above the slowest, in km/h. Zero makes
    /// every device identical.
    pub spread_kph: f64,
    /// The same for cadence, and deliberately independent of `spread_kph`
    /// rather than scaled from it. Scaling looks right over a few km/h and
    /// falls apart over a wide span: a fleet fanned from 5 to 60 km/h with
    /// cadence following the speed ratio would put its fast end at 1020 rpm.
    /// The caller decides what the two spans are; this only walks them.
    pub cadence_spread_rpm: f64,
    pub power_watts: f64,
    pub power_spread_w: f64,
    pub wheel_circumference_m: f64,
}

#[derive(Debug, PartialEq, Eq)]
pub enum FleetError {
    /// A fleet of nothing, or of bikes carrying no sensors.
    Empty,
    /// Device number 0 is ANT's wildcard and cannot be transmitted.
    WildcardDeviceNumber,
    /// The fleet would run past the 20-bit device number ceiling.
    DeviceNumberOverflow { last: u64 },
    /// More devices than the sticks on the bus have channels.
    NotEnoughDongles {
        wanted: usize,
        dongles: usize,
        capacity: usize,
    },
}

impl fmt::Display for FleetError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Empty => write!(
                f,
                "a fleet needs at least one device carrying at least one sensor"
            ),
            Self::WildcardDeviceNumber => write!(
                f,
                "device number 0 is the ANT wildcard and cannot be transmitted; start at {MIN_DEVICE_NUMBER} or above"
            ),
            Self::DeviceNumberOverflow { last } => write!(
                f,
                "the fleet would end at device number {last}, past the 20-bit ceiling of {MAX_DEVICE_NUMBER}"
            ),
            Self::NotEnoughDongles {
                wanted,
                dongles,
                capacity,
            } => write!(
                f,
                "{wanted} devices need more than the {capacity} channels on {dongles} dongle(s); \
                 each dongle carries {CHANNELS_PER_DONGLE}, so plug in {} more or ask for fewer devices",
                wanted
                    .div_ceil(CHANNELS_PER_DONGLE)
                    .saturating_sub(*dongles)
            ),
        }
    }
}

impl std::error::Error for FleetError {}

impl FleetSpec {
    /// Turn the spec into devices, numbered upwards from `start_id`.
    ///
    /// Speed and cadence are each spread linearly across the fleet so that no
    /// two devices advance their counters at the same rate. That is not
    /// cosmetic: with an identical fleet, a receiver that attributed one
    /// device's page to another would produce output indistinguishable from
    /// correct.
    pub fn build(&self) -> Result<Vec<SimDevice>, FleetError> {
        if self.devices == 0 || self.profiles.is_empty() {
            return Err(FleetError::Empty);
        }
        if self.start_id < MIN_DEVICE_NUMBER {
            return Err(FleetError::WildcardDeviceNumber);
        }
        let last = u64::from(self.start_id) + self.devices as u64 - 1;
        if last > u64::from(MAX_DEVICE_NUMBER) {
            return Err(FleetError::DeviceNumberOverflow { last });
        }

        let span = (self.devices - 1).max(1) as f64;
        Ok((0..self.devices)
            .flat_map(|i| {
                let fraction = if self.devices <= 1 {
                    0.0
                } else {
                    i as f64 / span
                };
                let speed = self.speed_kph + self.spread_kph * fraction;
                let cadence = self.cadence_rpm + self.cadence_spread_rpm * fraction;
                let watts = (self.power_watts + self.power_spread_w * fraction)
                    .round()
                    .clamp(0.0, f64::from(u16::MAX)) as u16;
                // A bike's sensors go out adjacent, so a chunk of eight keeps
                // whole bikes on one stick rather than splitting one across two.
                self.profiles.iter().map(move |&profile| {
                    SimDevice::new(
                        self.start_id + i as u32,
                        profile,
                        speed,
                        cadence,
                        watts,
                        self.wheel_circumference_m,
                    )
                })
            })
            .collect())
    }

    /// How many channels one bike occupies.
    #[must_use]
    pub fn channels_per_device(&self) -> usize {
        self.profiles.len().max(1)
    }

    /// Split a built fleet across the dongles it will run on, eight at a time.
    pub fn shard(
        devices: Vec<SimDevice>,
        dongles: usize,
    ) -> Result<Vec<Vec<SimDevice>>, FleetError> {
        let capacity = dongles * CHANNELS_PER_DONGLE;
        if devices.len() > capacity {
            return Err(FleetError::NotEnoughDongles {
                wanted: devices.len(),
                dongles,
                capacity,
            });
        }
        Ok(devices
            .chunks(CHANNELS_PER_DONGLE)
            .map(<[SimDevice]>::to_vec)
            .collect())
    }
}

/// Configure one dongle's channels as ANT+ masters and open them.
///
/// Every step is confirmed, for the same reason the receiver confirms its own:
/// `send_message` returns once the bytes are on the endpoint, so a stale handle
/// accepts the entire configuration and transmits nothing. `on_channel` is
/// called with each channel number as that channel is taken up, so a caller can
/// say which one a failure landed on — the error itself only names the step.
///
/// The channels are all configured before any are opened. An open channel
/// starts transmitting immediately and raises `EVENT_TX` while later channels
/// are still being set up, and those events would be swallowed by the response
/// waits; missing one costs nothing (the radio repeats the payload and raises
/// it again next period) but there is no reason to invite it.
pub fn configure_master<E, D: Driver<E>>(
    driver: &mut D,
    devices: &[SimDevice],
    mut on_channel: impl FnMut(u8),
) -> Result<(), InitError> {
    reset(driver)?;
    confirm(
        driver,
        "the network key",
        &SetNetworkKey::new(0, NETWORK_KEY),
    )?;

    for (index, device) in devices.iter().enumerate() {
        let channel = index as u8;
        on_channel(channel);
        confirm(
            driver,
            "the master channel assignment",
            &AssignChannel::new(channel, ChannelType::MasterTransmitOnly, 0, None),
        )?;
        confirm(driver, "the master channel id", &device.channel_id(channel))?;
        confirm(
            driver,
            "the master RF frequency",
            &ChannelRfFrequency::new(channel, RF_FREQ),
        )?;
        confirm(
            driver,
            "the channel period",
            &ChannelPeriod::new(channel, device.profile.channel_period()),
        )?;
    }

    for (index, device) in devices.iter().enumerate() {
        let channel = index as u8;
        on_channel(channel);
        confirm(
            driver,
            "opening the master channel",
            &OpenChannel::new(channel),
        )?;
        // Prime the buffer, or the first transmission on each channel goes out
        // as eight zero bytes before the first EVENT_TX arrives to fill it.
        send(
            driver,
            "the first payload",
            &BroadcastData::new(channel, device.page(Duration::ZERO)),
        )?;
    }

    Ok(())
}

/// Why a running simulator stopped. All three are the dongle going away in one
/// form or another; none of them are recoverable in place.
#[derive(Debug, PartialEq, Eq)]
pub enum SimError {
    /// The driver failed to read. The stick is gone or the handle is stale.
    Driver,
    /// A channel reported itself closed, so it is no longer transmitting.
    ChannelClosed { channel: u8 },
    /// Handing over a payload failed.
    Write { channel: u8 },
}

impl fmt::Display for SimError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Driver => write!(f, "the dongle stopped answering"),
            Self::ChannelClosed { channel } => {
                write!(f, "channel {channel} closed and is no longer transmitting")
            }
            Self::Write { channel } => {
                write!(f, "handing a payload to channel {channel} failed")
            }
        }
    }
}

impl std::error::Error for SimError {}

/// Feed one dongle's open master channels until something breaks.
///
/// `sent[i]` counts transmissions on channel `i`. It is bumped on `EVENT_TX`,
/// which the radio raises after a transmission has gone out, so the number is
/// packets on the air rather than payloads offered.
pub fn run<E, D: Driver<E>>(
    driver: &mut D,
    devices: &[SimDevice],
    start: Instant,
    sent: &[AtomicU64],
) -> SimError {
    loop {
        let msg = match driver.get_message() {
            Ok(Some(msg)) => msg,
            Ok(None) => continue,
            Err(_) => return SimError::Driver,
        };
        let RxMessage::ChannelEvent(event) = msg.message else {
            continue;
        };
        let channel = event.payload.channel_number;
        match event.payload.message_code {
            MessageCode::EventTx => {
                let Some(device) = devices.get(channel as usize) else {
                    continue;
                };
                let page = device.page(start.elapsed());
                if driver
                    .send_message(&BroadcastData::new(channel, page))
                    .is_err()
                {
                    return SimError::Write { channel };
                }
                if let Some(counter) = sent.get(channel as usize) {
                    counter.fetch_add(1, Ordering::Relaxed);
                }
            }
            MessageCode::EventChannelClosed => return SimError::ChannelClosed { channel },
            _ => (),
        }
    }
}

/// The retry loop from [`crate::usb`], configuring masters instead of a
/// receiver. The selector is a specific stick's port, because a fleet spread
/// over several dongles must not reset or claim its neighbours'.
pub struct MasterOps<'a> {
    selector: Option<&'a str>,
    devices: &'a [SimDevice],
    /// The channel the last attempt had reached, so a caller can report where
    /// a failure landed; the error only names the step.
    pub progress: Option<u8>,
}

impl<'a> MasterOps<'a> {
    #[must_use]
    pub fn new(selector: Option<&'a str>, devices: &'a [SimDevice]) -> Self {
        Self {
            selector,
            devices,
            progress: None,
        }
    }
}

impl BringUpOps for MasterOps<'_> {
    type Driver = Dongle;

    fn open(&mut self) -> Result<Dongle, BringUpError> {
        open_dongle(self.selector)
    }

    fn configure(&mut self, driver: &mut Dongle) -> Result<(), InitError> {
        let devices = self.devices;
        self.progress = None;
        configure_master(driver, devices, |channel| self.progress = Some(channel))
    }

    fn reset(&mut self) {
        reset_dongle(self.selector);
    }
}

/// How many devices the dongles on the bus can carry between them.
#[must_use]
pub fn capacity(dongles: &[DongleId]) -> usize {
    dongles.len() * CHANNELS_PER_DONGLE
}

#[cfg(test)]
mod tests {
    use super::*;
    use ant::drivers::DriverError;
    use ant::messages::channel::{ChannelEvent, ChannelEventPayload, ChannelResponse};
    use ant::messages::notifications::StartUpMessage;
    use ant::messages::{AntMessage, TransmitableMessage, TxMessageId};
    use packed_struct::PackedStructSlice;
    use packed_struct::PrimitiveEnum;
    use packed_struct::types::SizedInteger;

    #[derive(Debug)]
    struct FakeError;

    /// A dongle that accepts everything, remembers what it was told, and can be
    /// scripted to answer channel events.
    struct FakeDongle {
        /// Every message it was asked to send, as (id, serialized bytes).
        sent: Vec<(TxMessageId, Vec<u8>)>,
        pending: Vec<AntMessage>,
        /// A message id to refuse rather than accept.
        refuse: Option<TxMessageId>,
        read_fails: bool,
    }

    impl FakeDongle {
        fn new() -> Self {
            Self {
                sent: Vec::new(),
                pending: Vec::new(),
                refuse: None,
                read_fails: false,
            }
        }

        fn ids(&self) -> Vec<TxMessageId> {
            self.sent.iter().map(|(id, _)| *id).collect()
        }

        /// The payloads of the broadcasts it was handed, as (channel, page).
        fn broadcasts(&self) -> Vec<(u8, [u8; 8])> {
            self.sent
                .iter()
                .filter(|(id, _)| *id == TxMessageId::BroadcastData)
                .map(|(_, bytes)| {
                    let mut page = [0u8; 8];
                    page.copy_from_slice(&bytes[1..9]);
                    (bytes[0], page)
                })
                .collect()
        }
    }

    impl Driver<FakeError> for FakeDongle {
        fn get_message(&mut self) -> Result<Option<AntMessage>, DriverError<FakeError>> {
            if self.read_fails {
                return Err(DriverError::BadLength(0, 0));
            }
            Ok(if self.pending.is_empty() {
                None
            } else {
                Some(self.pending.remove(0))
            })
        }

        fn send_message(
            &mut self,
            msg: &dyn TransmitableMessage,
        ) -> Result<(), DriverError<FakeError>> {
            let id = msg.get_tx_msg_id();
            let mut buf = [0u8; 32];
            let len = msg.serialize_message(&mut buf).expect("serialize");
            self.sent.push((id, buf[..len].to_vec()));

            match id {
                TxMessageId::ResetSystem => self.pending.push(startup()),
                // Broadcast data on an open master is answered by an event when
                // it goes out, not by a response, so it is not confirmed.
                TxMessageId::BroadcastData => (),
                _ => {
                    let code = if self.refuse == Some(id) {
                        MessageCode::InvalidMessage
                    } else {
                        MessageCode::ResponseNoError
                    };
                    self.pending.push(response(id, code));
                }
            }
            Ok(())
        }
    }

    fn startup() -> AntMessage {
        AntMessage {
            message: RxMessage::StartUpMessage(StartUpMessage {
                hardware_reset_line: false,
                watch_dog_reset: false,
                command_reset: true,
                synchronous_reset: false,
                suspend_reset: false,
            }),
            ..Default::default()
        }
    }

    fn response(id: TxMessageId, code: MessageCode) -> AntMessage {
        AntMessage {
            message: RxMessage::ChannelResponse(ChannelResponse {
                channel_number: 0,
                message_id: id,
                message_code: code,
            }),
            ..Default::default()
        }
    }

    /// Built from bytes because the payload's reserved bits are private: byte 1
    /// is the reserved pattern the format requires (seven zeroes and a one).
    fn channel_event(channel: u8, code: MessageCode) -> AntMessage {
        let payload = ChannelEventPayload::unpack_from_slice(&[channel, 0x01, code.to_primitive()])
            .expect("channel event payload");
        AntMessage {
            message: RxMessage::ChannelEvent(ChannelEvent {
                payload,
                extended_info: None,
            }),
            ..Default::default()
        }
    }

    fn spec(devices: usize) -> FleetSpec {
        FleetSpec {
            devices,
            start_id: 1,
            profiles: SPEED_CADENCE_ONLY,
            speed_kph: 25.0,
            cadence_rpm: 85.0,
            power_watts: 200.0,
            spread_kph: 0.0,
            cadence_spread_rpm: 0.0,
            power_spread_w: 0.0,
            wheel_circumference_m: 2.096,
        }
    }

    #[test]
    fn a_fleet_is_numbered_upwards_from_the_start_id() {
        let fleet = FleetSpec {
            start_id: 5000,
            ..spec(4)
        }
        .build()
        .unwrap();
        assert_eq!(
            fleet.iter().map(|d| d.device_number).collect::<Vec<_>>(),
            [5000, 5001, 5002, 5003]
        );
    }

    /// Without a spread every device is identical, which is the simpler thing
    /// to read; with one, no two advance at the same rate.
    #[test]
    fn a_spread_fans_the_fleet_out_and_takes_cadence_with_it() {
        let flat = spec(5).build().unwrap();
        assert!(flat.iter().all(|d| d.speed_kph == 25.0));
        assert!(flat.iter().all(|d| d.cadence_rpm == 85.0));

        let fanned = FleetSpec {
            spread_kph: 10.0,
            cadence_spread_rpm: 20.0,
            ..spec(5)
        }
        .build()
        .unwrap();
        let speeds: Vec<f64> = fanned.iter().map(|d| d.speed_kph).collect();
        assert_eq!(speeds, [25.0, 27.5, 30.0, 32.5, 35.0]);
        // Cadence walks its own span, so a wide speed fan cannot drag it
        // somewhere no crank turns.
        assert_eq!(
            fanned.iter().map(|d| d.cadence_rpm).collect::<Vec<_>>(),
            [85.0, 90.0, 95.0, 100.0, 105.0]
        );
        // Every device advances at its own rate, which is the whole point.
        let mut sorted = speeds.clone();
        sorted.dedup();
        assert_eq!(sorted.len(), speeds.len());

        // A lone device sits at the base of both spans rather than dividing by
        // the zero-width fraction.
        let single = FleetSpec {
            spread_kph: 10.0,
            cadence_spread_rpm: 20.0,
            ..spec(1)
        }
        .build()
        .unwrap();
        assert_eq!(single[0].speed_kph, 25.0);
        assert_eq!(single[0].cadence_rpm, 85.0);
    }

    #[test]
    fn a_fleet_that_cannot_go_on_the_air_is_refused_before_a_dongle_is_touched() {
        assert_eq!(spec(0).build(), Err(FleetError::Empty));
        assert_eq!(
            FleetSpec {
                start_id: 0,
                ..spec(4)
            }
            .build(),
            Err(FleetError::WildcardDeviceNumber)
        );
        assert_eq!(
            FleetSpec {
                start_id: MAX_DEVICE_NUMBER,
                ..spec(2)
            }
            .build(),
            Err(FleetError::DeviceNumberOverflow {
                last: u64::from(MAX_DEVICE_NUMBER) + 1
            })
        );
        // Right up to the ceiling is fine.
        assert!(
            FleetSpec {
                start_id: MAX_DEVICE_NUMBER,
                ..spec(1)
            }
            .build()
            .is_ok()
        );
    }

    #[test]
    fn a_fleet_is_split_eight_to_a_dongle_and_refused_if_it_does_not_fit() {
        let shards = FleetSpec::shard(spec(20).build().unwrap(), 3).unwrap();
        assert_eq!(
            shards.iter().map(Vec::len).collect::<Vec<_>>(),
            [8, 8, 4],
            "channels are filled a stick at a time"
        );

        assert_eq!(
            FleetSpec::shard(spec(9).build().unwrap(), 1),
            Err(FleetError::NotEnoughDongles {
                wanted: 9,
                dongles: 1,
                capacity: 8,
            })
        );
        assert!(FleetSpec::shard(spec(8).build().unwrap(), 1).is_ok());
    }

    /// The split the receiver reverses: low 16 bits in the channel id, top 4 in
    /// the transmission type. Getting it wrong puts the device on the air under
    /// a number nobody asked for.
    #[test]
    fn a_device_number_above_65535_rides_in_the_transmission_type_extension() {
        let device = SimDevice::new(70_000, Profile::SpeedAndCadence, 25.0, 85.0, 200, 2.096);
        let id = device.channel_id(3);
        assert_eq!(id.channel_number, 3);
        assert_eq!(id.device_number, 0x1170);
        assert_eq!(
            id.transmission_type.device_number_extension.to_primitive(),
            1
        );
        assert_eq!(
            id.device_type.device_type_id.to_primitive(),
            CSC_DEVICE_TYPE
        );

        // Reassembled the way `DeviceKey::from_broadcast` does it.
        let extension =
            u32::from(id.transmission_type.device_number_extension.to_primitive()) << 16;
        assert_eq!(u32::from(id.device_number) | extension, 70_000);

        // A number that fits in 16 bits leaves the nibble alone.
        let small = SimDevice::new(1234, Profile::SpeedAndCadence, 25.0, 85.0, 200, 2.096);
        assert_eq!(
            small
                .channel_id(0)
                .transmission_type
                .device_number_extension
                .to_primitive(),
            0
        );
    }

    #[test]
    fn every_channel_is_configured_before_any_of_them_is_opened() {
        let mut dongle = FakeDongle::new();
        let fleet = spec(2).build().unwrap();
        assert_eq!(configure_master(&mut dongle, &fleet, |_| {}), Ok(()));

        use TxMessageId::*;
        assert_eq!(
            dongle.ids(),
            vec![
                ResetSystem,
                SetNetworkKey,
                // Channel 0, then channel 1, configured but not yet running.
                AssignChannel,
                ChannelId,
                ChannelRfFrequency,
                ChannelPeriod,
                AssignChannel,
                ChannelId,
                ChannelRfFrequency,
                ChannelPeriod,
                // Only now do they go on the air, each primed as it opens.
                OpenChannel,
                BroadcastData,
                OpenChannel,
                BroadcastData,
            ]
        );
    }

    /// The failure `init` exists to catch, on the transmit side: a stick that
    /// answers nothing is caught at the reset rather than configured into the
    /// void.
    #[test]
    fn a_deaf_dongle_is_caught_before_any_channel_is_assigned() {
        struct Silent;
        impl Driver<FakeError> for Silent {
            fn get_message(&mut self) -> Result<Option<AntMessage>, DriverError<FakeError>> {
                Ok(None)
            }
            fn send_message(
                &mut self,
                _: &dyn TransmitableMessage,
            ) -> Result<(), DriverError<FakeError>> {
                Ok(())
            }
        }
        let fleet = spec(2).build().unwrap();
        assert_eq!(
            configure_master(&mut Silent, &fleet, |_| {}),
            Err(InitError::Deaf { message: "a reset" })
        );
    }

    #[test]
    fn a_refused_message_stops_the_setup_and_says_which_channel_it_reached() {
        let mut dongle = FakeDongle::new();
        dongle.refuse = Some(TxMessageId::ChannelPeriod);
        let fleet = spec(4).build().unwrap();

        let mut reached = None;
        assert_eq!(
            configure_master(&mut dongle, &fleet, |channel| reached = Some(channel)),
            Err(InitError::Rejected {
                message: "the channel period",
                code: MessageCode::InvalidMessage,
            })
        );
        assert_eq!(reached, Some(0));
        assert!(!dongle.ids().contains(&TxMessageId::OpenChannel));
    }

    /// Each channel must be fed its own device's page. A fleet whose devices all
    /// look alike could not show this, so device 0 is parked and device 1 is not.
    #[test]
    fn each_event_is_answered_with_that_channels_own_device() {
        let mut dongle = FakeDongle::new();
        let devices = vec![
            SimDevice::new(1, Profile::SpeedAndCadence, 0.0, 0.0, 0, 2.096),
            SimDevice::new(2, Profile::SpeedAndCadence, 25.0, 85.0, 200, 2.096),
        ];
        let sent: Vec<AtomicU64> = (0..2).map(|_| AtomicU64::new(0)).collect();

        dongle.pending.push(channel_event(0, MessageCode::EventTx));
        dongle.pending.push(channel_event(1, MessageCode::EventTx));
        dongle
            .pending
            .push(channel_event(1, MessageCode::EventChannelClosed));

        // Ten seconds in, so the moving device has counters and the parked one
        // still does not.
        let start = Instant::now() - Duration::from_secs(10);
        assert_eq!(
            run(&mut dongle, &devices, start, &sent),
            SimError::ChannelClosed { channel: 1 }
        );

        let broadcasts = dongle.broadcasts();
        assert_eq!(broadcasts.len(), 2);
        assert_eq!(
            broadcasts[0].0, 0,
            "channel 0's event answered on channel 0"
        );
        assert_eq!(broadcasts[0].1, [0; 8], "a parked bike transmits zeroes");
        assert_eq!(broadcasts[1].0, 1);
        assert_ne!(broadcasts[1].1, [0; 8], "a moving bike does not");

        // Counted per channel, and only for transmissions that happened.
        assert_eq!(sent[0].load(Ordering::Relaxed), 1);
        assert_eq!(sent[1].load(Ordering::Relaxed), 1);
    }

    #[test]
    fn a_dongle_that_stops_answering_ends_the_run_rather_than_spinning() {
        let mut dongle = FakeDongle::new();
        dongle.read_fails = true;
        let devices = spec(1).build().unwrap();
        let sent = vec![AtomicU64::new(0)];
        assert_eq!(
            run(&mut dongle, &devices, Instant::now(), &sent),
            SimError::Driver
        );
    }

    /// An event for a channel this dongle has no device for is ignored rather
    /// than indexed into.
    #[test]
    fn an_event_for_an_unknown_channel_is_ignored() {
        let mut dongle = FakeDongle::new();
        dongle.pending.push(channel_event(7, MessageCode::EventTx));
        dongle
            .pending
            .push(channel_event(0, MessageCode::EventChannelClosed));
        let devices = spec(1).build().unwrap();
        let sent = vec![AtomicU64::new(0)];
        assert_eq!(
            run(&mut dongle, &devices, Instant::now(), &sent),
            SimError::ChannelClosed { channel: 0 }
        );
        assert!(dongle.broadcasts().is_empty());
    }

    /// A bike's two channels carry two different profiles at two different
    /// periods, and each must get its own page: swapping them would put a
    /// power page on the speed channel, which parses as a plausible bicycle.
    #[test]
    fn a_bikes_two_channels_carry_their_own_profiles_pages_and_periods() {
        let fleet = FleetSpec {
            profiles: SPEED_CADENCE_AND_POWER,
            ..spec(1)
        }
        .build()
        .unwrap();
        assert_eq!(fleet.len(), 2);

        let (csc, power) = (fleet[0], fleet[1]);
        assert_eq!(csc.profile, Profile::SpeedAndCadence);
        assert_eq!(power.profile, Profile::Power);
        // One bike, one number; the device type is what tells them apart.
        assert_eq!(csc.device_number, power.device_number);
        assert_eq!(
            csc.channel_id(0).device_type.device_type_id.to_primitive(),
            121
        );
        assert_eq!(
            power
                .channel_id(1)
                .device_type
                .device_type_id
                .to_primitive(),
            11
        );
        assert_eq!(csc.profile.channel_period(), CSC_CHANNEL_PERIOD);
        assert_eq!(power.profile.channel_period(), POWER_CHANNEL_PERIOD);

        // The power page announces itself; the combined page has no page byte
        // at all, so its first byte is a timestamp and not 0x10.
        let elapsed = Duration::from_secs(30);
        assert_eq!(power.page(elapsed)[0], 0x10);
        assert_eq!(
            u16::from_le_bytes([power.page(elapsed)[6], power.page(elapsed)[7]]),
            200,
            "instantaneous watts"
        );

        // Both are driven off the same crank, so the power event count and the
        // combined page's cadence revolutions are the same number.
        let DeviceState::SpeedAndCadence { crank_revs, .. } = csc.state(elapsed) else {
            panic!("speed and cadence device reported power state");
        };
        let DeviceState::Power { events, .. } = power.state(elapsed) else {
            panic!("power device reported speed and cadence state");
        };
        assert_eq!(u16::from(events), crank_revs % 256);
    }

    #[test]
    fn a_fleet_of_bikes_with_no_sensors_is_refused() {
        assert_eq!(
            FleetSpec {
                profiles: &[],
                ..spec(4)
            }
            .build(),
            Err(FleetError::Empty)
        );
    }

    #[test]
    fn the_profile_reports_the_rate_its_period_works_out_to() {
        let hz = Profile::SpeedAndCadence.hz();
        assert!((hz - 4.053).abs() < 0.001, "got {hz}");
        assert_eq!(
            Profile::SpeedAndCadence.channel_period(),
            CSC_CHANNEL_PERIOD
        );
        assert_eq!(Profile::SpeedAndCadence.device_type(), 121);
    }
}
