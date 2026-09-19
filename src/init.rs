//! Bringing a dongle up, and proving it is listening.
//!
//! `send_message` returns once the bytes are on the bulk endpoint, so it says
//! nothing about whether the dongle received them. A dongle whose handle has
//! gone stale accepts every write and answers none of them, which looks exactly
//! like a working setup until the race starts and no telemetry ever arrives.
//! Observed on a Pi: after restarting the process the stick stayed deaf until it
//! was physically unplugged.
//!
//! So every message the channel depends on is confirmed here. A reset always
//! produces a startup notification, which makes it the probe: nothing coming
//! back from it means the dongle is not listening, and no amount of further
//! configuration will change that. Passing it once settles nothing for later:
//! a stick plugged back in mid-run has answered its reset and then delivered
//! nothing, so a quiet caller asks again with [`probe_channel`].

use ant::drivers::Driver;
use ant::messages::channel::MessageCode;
use ant::messages::config::{
    AssignChannel, ChannelId, ChannelRfFrequency, ChannelType, DeviceType, EnableExtRxMessages,
    SetNetworkKey, TransmissionType,
};
use ant::messages::control::{OpenRxScanMode, RequestMessage, RequestableMessageId, ResetSystem};
use ant::messages::requested_response::ChannelState;
use ant::messages::{AntMessage, RxMessage, TransmitableMessage, TxMessageId};
use std::fmt;
use std::time::{Duration, Instant};

const NETWORK_KEY: [u8; 8] = [0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45];
const RF_FREQ: u8 = 57;

/// How long to wait for an answer. The two dongles on the bench answered in
/// 1.8 ms and 10.4 ms, so this is generous by two orders of magnitude and only
/// has to be short enough that a deaf dongle is noticed rather than waited on.
pub const RESPONSE_TIMEOUT: Duration = Duration::from_millis(500);

#[derive(Debug, PartialEq)]
pub enum InitError {
    /// The dongle did not answer something it always answers. It is not
    /// listening, whatever the writes returned.
    Deaf { message: &'static str },
    /// It answered, and refused.
    Rejected {
        message: &'static str,
        code: MessageCode,
    },
    /// The write itself failed.
    Write { message: &'static str },
    /// It answered a status request, and the channel it was configured with
    /// is no longer open.
    ChannelClosed { state: ChannelState },
}

impl fmt::Display for InitError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Deaf { message } => write!(
                f,
                "the dongle did not answer {message}, so it is not listening and no data will arrive"
            ),
            Self::Rejected { message, code } => {
                write!(f, "the dongle rejected {message}: {code:?}")
            }
            Self::Write { message } => write!(f, "writing {message} to the dongle failed"),
            Self::ChannelClosed { state } => write!(
                f,
                "the dongle reports its channel as {state:?} rather than open, so no data will arrive"
            ),
        }
    }
}

/// Configure channel 0 as a promiscuous ANT+ receiver, confirming every step.
///
/// `EnableExtRxMessages` is the legacy switch and turns on the channel id block,
/// which is the only extended data anything here reads. **`LibConfig` is
/// deliberately not sent**, because the two things it adds are both unwanted:
///
/// - RSSI, because every stick on the bench reports the AGC register rather than
///   dBm, and that register was measured byte-identical from point-blank to out
///   of range, so the block would cost bytes a frame and tell nobody anything.
/// - RX timestamps, because the collision detector times on the host's arrival
///   clock instead. A stamp riding inside a frame can be garbled by the very
///   fault the detector exists to catch, and it resolves a distinction nothing
///   needs: a venue capture puts the normal cadence 200x away from the collision
///   window, which an arrival clock separates comfortably.
pub fn configure<E, D: Driver<E>>(driver: &mut D) -> Result<(), InitError> {
    reset(driver)?;

    confirm(
        driver,
        "the network key",
        &SetNetworkKey::new(0, NETWORK_KEY),
    )?;
    confirm(
        driver,
        "the channel assignment",
        &AssignChannel::new(0, ChannelType::SharedReceiveOnly, 0, None),
    )?;
    confirm(
        driver,
        "the channel id",
        &ChannelId::new(
            0,
            0,
            DeviceType::new(0.into(), false),
            TransmissionType::new_wildcard(),
        ),
    )?;
    confirm(
        driver,
        "the RF frequency",
        &ChannelRfFrequency::new(0, RF_FREQ),
    )?;
    confirm(
        driver,
        "extended RX messages",
        &EnableExtRxMessages::new(true),
    )?;

    // Scan mode answers with broadcast data rather than a response, so there is
    // nothing to confirm; everything it depends on is confirmed above.
    send(driver, "scan mode", &OpenRxScanMode::default())?;

    Ok(())
}

/// Reset the dongle and wait for the startup notification it always answers
/// with. This is the probe: a dongle that does not answer this one is deaf.
fn reset<E, D: Driver<E>>(driver: &mut D) -> Result<(), InitError> {
    send(driver, "a reset", &ResetSystem::new())?;

    let deadline = Instant::now() + RESPONSE_TIMEOUT;
    while Instant::now() < deadline {
        if let Ok(Some(msg)) = driver.get_message()
            && matches!(msg.message, RxMessage::StartUpMessage(_))
        {
            return Ok(());
        }
    }
    Err(InitError::Deaf { message: "a reset" })
}

/// Is channel 0 still open? `Ok` means the air is merely quiet. Broadcasts
/// arriving while the answer is awaited go to `passthrough`, not the floor.
pub fn probe_channel<E, D: Driver<E>>(
    driver: &mut D,
    mut passthrough: impl FnMut(AntMessage),
) -> Result<(), InitError> {
    send(
        driver,
        "a channel status request",
        &RequestMessage::new(0, RequestableMessageId::ChannelStatus, None),
    )?;
    let deadline = Instant::now() + RESPONSE_TIMEOUT;
    while Instant::now() < deadline {
        if let Ok(Some(msg)) = driver.get_message() {
            match msg.message {
                RxMessage::ChannelStatus(status) if status.channel_number == 0 => {
                    return match status.channel_state {
                        ChannelState::Searching | ChannelState::Tracking => Ok(()),
                        state => Err(InitError::ChannelClosed { state }),
                    };
                }
                _ => passthrough(msg),
            }
        }
    }
    Err(InitError::Deaf {
        message: "a channel status request",
    })
}

fn send<E, D: Driver<E>>(
    driver: &mut D,
    message: &'static str,
    msg: &dyn TransmitableMessage,
) -> Result<(), InitError> {
    driver
        .send_message(msg)
        .map_err(|_| InitError::Write { message })
}

/// Send a message and require the dongle to accept it.
fn confirm<E, D: Driver<E>>(
    driver: &mut D,
    message: &'static str,
    msg: &dyn TransmitableMessage,
) -> Result<(), InitError> {
    let id = msg.get_tx_msg_id();
    send(driver, message, msg)?;
    match await_response(driver, id) {
        Some(MessageCode::ResponseNoError) => Ok(()),
        Some(code) => Err(InitError::Rejected { message, code }),
        None => Err(InitError::Deaf { message }),
    }
}

/// Wait for the dongle's `ChannelResponse` to one message, discarding answers to
/// earlier ones. Safe to drain here because the channel is not open yet, so
/// nothing is arriving but responses.
fn await_response<E, D: Driver<E>>(driver: &mut D, id: TxMessageId) -> Option<MessageCode> {
    let deadline = Instant::now() + RESPONSE_TIMEOUT;
    while Instant::now() < deadline {
        if let Ok(Some(msg)) = driver.get_message()
            && let RxMessage::ChannelResponse(resp) = msg.message
            && resp.message_id == id
        {
            return Some(resp.message_code);
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use ant::drivers::DriverError;
    use ant::messages::channel::ChannelResponse;
    use ant::messages::data::BroadcastData;
    use ant::messages::notifications::StartUpMessage;
    use ant::messages::requested_response::ChannelStatus;
    use ant::messages::{AntMessage, RxMessage};

    #[derive(Debug)]
    struct FakeError;

    /// A dongle that answers the messages it is told to answer, and stays silent
    /// for the rest. `sent` is what it was actually asked, in order.
    struct FakeDongle {
        answers_reset: bool,
        answers: Vec<(TxMessageId, MessageCode)>,
        /// What a channel status request is answered with; `None` is silence.
        channel_state: Option<ChannelState>,
        sent: Vec<TxMessageId>,
        pending: Vec<AntMessage>,
    }

    impl FakeDongle {
        fn healthy() -> Self {
            Self {
                answers_reset: true,
                channel_state: Some(ChannelState::Searching),
                answers: vec![
                    (TxMessageId::SetNetworkKey, MessageCode::ResponseNoError),
                    (TxMessageId::AssignChannel, MessageCode::ResponseNoError),
                    (TxMessageId::ChannelId, MessageCode::ResponseNoError),
                    (
                        TxMessageId::ChannelRfFrequency,
                        MessageCode::ResponseNoError,
                    ),
                    (
                        TxMessageId::EnableExtRxMessages,
                        MessageCode::ResponseNoError,
                    ),
                ],
                sent: Vec::new(),
                pending: Vec::new(),
            }
        }

        fn silent() -> Self {
            Self {
                answers_reset: false,
                answers: Vec::new(),
                channel_state: None,
                sent: Vec::new(),
                pending: Vec::new(),
            }
        }

        fn with_channel(mut self, state: ChannelState) -> Self {
            self.channel_state = Some(state);
            self
        }

        fn without(mut self, id: TxMessageId) -> Self {
            self.answers.retain(|(msg_id, _)| *msg_id != id);
            self
        }

        fn rejecting(mut self, id: TxMessageId, code: MessageCode) -> Self {
            for answer in &mut self.answers {
                if answer.0 == id {
                    answer.1 = code;
                }
            }
            self
        }
    }

    impl Driver<FakeError> for FakeDongle {
        fn get_message(&mut self) -> Result<Option<AntMessage>, DriverError<FakeError>> {
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
            self.sent.push(id);
            if id == TxMessageId::ResetSystem {
                if self.answers_reset {
                    self.pending.push(startup());
                }
                return Ok(());
            }
            if id == TxMessageId::RequestMessage {
                if let Some(state) = self.channel_state {
                    self.pending.push(channel_status(state));
                }
                return Ok(());
            }
            if let Some((_, code)) = self.answers.iter().find(|(msg_id, _)| *msg_id == id) {
                self.pending.push(response(id, *code));
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

    fn channel_status(state: ChannelState) -> AntMessage {
        AntMessage {
            message: RxMessage::ChannelStatus(ChannelStatus {
                channel_number: 0,
                channel_type: ChannelType::SharedReceiveOnly,
                network_number: 0,
                channel_state: state,
            }),
            ..Default::default()
        }
    }

    fn broadcast() -> AntMessage {
        AntMessage {
            message: RxMessage::BroadcastData(BroadcastData::default()),
            ..Default::default()
        }
    }

    #[test]
    fn an_open_channel_answers_the_probe_and_a_closed_or_silent_one_fails_it() {
        let mut dongle = FakeDongle::healthy();
        assert_eq!(probe_channel(&mut dongle, |_| {}), Ok(()));
        assert_eq!(dongle.sent, vec![TxMessageId::RequestMessage]);

        let mut dongle = FakeDongle::healthy().with_channel(ChannelState::Tracking);
        assert_eq!(probe_channel(&mut dongle, |_| {}), Ok(()));

        let mut dongle = FakeDongle::healthy().with_channel(ChannelState::Assigned);
        assert_eq!(
            probe_channel(&mut dongle, |_| {}),
            Err(InitError::ChannelClosed {
                state: ChannelState::Assigned
            })
        );

        let mut dongle = FakeDongle::silent();
        assert_eq!(
            probe_channel(&mut dongle, |_| {}),
            Err(InitError::Deaf {
                message: "a channel status request"
            })
        );
    }

    #[test]
    fn a_broadcast_arriving_during_the_probe_is_handed_on_not_dropped() {
        let mut dongle = FakeDongle::healthy();
        dongle.pending.push(broadcast());
        let mut passed = Vec::new();
        assert_eq!(probe_channel(&mut dongle, |msg| passed.push(msg)), Ok(()));
        assert_eq!(passed.len(), 1);
        assert!(matches!(passed[0].message, RxMessage::BroadcastData(_)));
    }

    #[test]
    fn a_healthy_dongle_is_configured_in_order() {
        let mut dongle = FakeDongle::healthy();
        assert_eq!(configure(&mut dongle), Ok(()));
        assert_eq!(
            dongle.sent,
            vec![
                TxMessageId::ResetSystem,
                TxMessageId::SetNetworkKey,
                TxMessageId::AssignChannel,
                TxMessageId::ChannelId,
                TxMessageId::ChannelRfFrequency,
                // The legacy switch is what carries the channel ids; LibConfig
                // is deliberately never sent.
                TxMessageId::EnableExtRxMessages,
                TxMessageId::OpenRxScanMode,
            ]
        );
    }

    // The failure this module exists for: every write succeeds and nothing comes
    // back. Before the reset was confirmed, configuration ran to the end and the
    // process sat waiting for data that could never arrive.
    #[test]
    fn a_deaf_dongle_is_caught_at_the_reset() {
        let mut dongle = FakeDongle::silent();
        assert_eq!(
            configure(&mut dongle),
            Err(InitError::Deaf { message: "a reset" })
        );
        // It stopped there rather than configuring into the void.
        assert_eq!(dongle.sent, vec![TxMessageId::ResetSystem]);
    }

    #[test]
    fn a_config_message_that_goes_unanswered_stops_the_sequence() {
        let mut dongle = FakeDongle::healthy().without(TxMessageId::ChannelId);
        assert_eq!(
            configure(&mut dongle),
            Err(InitError::Deaf {
                message: "the channel id"
            })
        );
        assert!(!dongle.sent.contains(&TxMessageId::OpenRxScanMode));
    }

    #[test]
    fn a_refused_config_message_names_what_was_refused() {
        let mut dongle = FakeDongle::healthy()
            .rejecting(TxMessageId::SetNetworkKey, MessageCode::InvalidMessage);
        assert_eq!(
            configure(&mut dongle),
            Err(InitError::Rejected {
                message: "the network key",
                code: MessageCode::InvalidMessage
            })
        );
    }
}
