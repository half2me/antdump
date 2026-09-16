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
//! configuration will change that.

use ant::drivers::Driver;
use ant::messages::channel::MessageCode;
use ant::messages::config::{
    AssignChannel, ChannelId, ChannelRfFrequency, ChannelType, DeviceType, EnableExtRxMessages,
    LibConfig, SetNetworkKey, TransmissionType,
};
use ant::messages::control::{OpenRxScanMode, ResetSystem};
use ant::messages::{RxMessage, TransmitableMessage, TxMessageId};
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
        }
    }
}

/// What became of the one optional step, for the caller to report.
#[derive(Debug, PartialEq)]
pub enum LibConfigOutcome {
    Accepted,
    /// A dongle that refuses it still works on the legacy extension, with
    /// channel ids but no RX timestamps.
    Rejected(MessageCode),
    /// It answered everything else, so it is listening; it just said nothing
    /// about this one.
    Unanswered,
}

impl LibConfigOutcome {
    /// What to tell the operator when the step did not land: one line naming
    /// what was lost, or nothing when it was accepted.
    pub fn warning(&self) -> Option<String> {
        const LOST: &str =
            "no RX timestamps, so collision detection falls back to wall-clock timing";
        match self {
            Self::Accepted => None,
            Self::Rejected(code) => {
                Some(format!("the dongle rejected LibConfig ({code:?}); {LOST}"))
            }
            Self::Unanswered => Some(format!("the dongle did not answer LibConfig; {LOST}")),
        }
    }
}

/// Configure channel 0 as a promiscuous ANT+ receiver, confirming every step.
///
/// The order of the last two is load-bearing: `EnableExtRxMessages` is the
/// legacy switch and turns on the channel id block only, `LibConfig` supersedes
/// it and adds RX timestamps. Legacy first leaves a clone that ignores
/// LibConfig still reporting channel ids.
///
/// RSSI is deliberately NOT requested. Every stick on the bench reports the AGC
/// register rather than dBm, and that register was measured byte-identical
/// from point-blank to out of range, so the block would cost three or four
/// bytes a frame and tell nobody anything.
pub fn configure<E, D: Driver<E>>(driver: &mut D) -> Result<LibConfigOutcome, InitError> {
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

    let lib_config = lib_config(driver)?;

    // Scan mode answers with broadcast data rather than a response, so there is
    // nothing to confirm; everything it depends on is confirmed above.
    send(driver, "scan mode", &OpenRxScanMode::default())?;

    Ok(lib_config)
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

/// LibConfig is the one step the channel does not depend on, so a refusal
/// degrades rather than fails: the legacy switch above already carries channel
/// ids, which is what the collision detector and the device registry need.
fn lib_config<E, D: Driver<E>>(driver: &mut D) -> Result<LibConfigOutcome, InitError> {
    // Channel id, no RSSI, RX timestamps.
    send(driver, "LibConfig", &LibConfig::new(true, false, true))?;
    Ok(match await_response(driver, TxMessageId::LibConfig) {
        Some(MessageCode::ResponseNoError) => LibConfigOutcome::Accepted,
        Some(code) => LibConfigOutcome::Rejected(code),
        None => LibConfigOutcome::Unanswered,
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
    use ant::messages::notifications::StartUpMessage;
    use ant::messages::{AntMessage, RxMessage};

    #[derive(Debug)]
    struct FakeError;

    /// A dongle that answers the messages it is told to answer, and stays silent
    /// for the rest. `sent` is what it was actually asked, in order.
    struct FakeDongle {
        answers_reset: bool,
        answers: Vec<(TxMessageId, MessageCode)>,
        sent: Vec<TxMessageId>,
        /// The packed bytes of the LibConfig it was sent, if any.
        lib_config: Option<Vec<u8>>,
        pending: Vec<AntMessage>,
    }

    impl FakeDongle {
        fn healthy() -> Self {
            Self {
                answers_reset: true,
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
                    (TxMessageId::LibConfig, MessageCode::ResponseNoError),
                ],
                sent: Vec::new(),
                lib_config: None,
                pending: Vec::new(),
            }
        }

        fn silent() -> Self {
            Self {
                answers_reset: false,
                answers: Vec::new(),
                sent: Vec::new(),
                lib_config: None,
                pending: Vec::new(),
            }
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
            if id == TxMessageId::LibConfig {
                let mut buf = [0u8; 16];
                let len = msg.serialize_message(&mut buf).expect("LibConfig packs");
                self.lib_config = Some(buf[..len].to_vec());
            }
            if id == TxMessageId::ResetSystem {
                if self.answers_reset {
                    self.pending.push(startup());
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

    // Byte 1 of LibConfig: channel id (0x80), RSSI (0x40), RX timestamp (0x20).
    #[test]
    fn lib_config_asks_for_channel_ids_and_timestamps_but_not_rssi() {
        let mut dongle = FakeDongle::healthy();
        configure(&mut dongle).unwrap();
        assert_eq!(dongle.lib_config.as_deref(), Some(&[0x00, 0xA0][..]));
    }

    #[test]
    fn a_healthy_dongle_is_configured_in_order() {
        let mut dongle = FakeDongle::healthy();
        assert_eq!(configure(&mut dongle), Ok(LibConfigOutcome::Accepted));
        assert_eq!(
            dongle.sent,
            vec![
                TxMessageId::ResetSystem,
                TxMessageId::SetNetworkKey,
                TxMessageId::AssignChannel,
                TxMessageId::ChannelId,
                TxMessageId::ChannelRfFrequency,
                // The legacy switch before LibConfig, so a dongle that ignores
                // LibConfig still reports channel ids.
                TxMessageId::EnableExtRxMessages,
                TxMessageId::LibConfig,
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

    // A dongle that refuses LibConfig keeps the legacy extension, so it is a
    // degraded capture rather than a failure.
    #[test]
    fn a_refused_lib_config_still_opens_the_channel() {
        let mut dongle =
            FakeDongle::healthy().rejecting(TxMessageId::LibConfig, MessageCode::InvalidMessage);
        assert_eq!(
            configure(&mut dongle),
            Ok(LibConfigOutcome::Rejected(MessageCode::InvalidMessage))
        );
        assert!(dongle.sent.contains(&TxMessageId::OpenRxScanMode));
    }

    #[test]
    fn an_unanswered_lib_config_still_opens_the_channel() {
        let mut dongle = FakeDongle::healthy().without(TxMessageId::LibConfig);
        assert_eq!(configure(&mut dongle), Ok(LibConfigOutcome::Unanswered));
        assert!(dongle.sent.contains(&TxMessageId::OpenRxScanMode));
    }
}
