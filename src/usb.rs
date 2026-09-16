//! Bringing the first ANT+ dongle on the bus up, and bringing it back.
//!
//! A stale handle is the failure this module exists for. `UsbDriver::new`
//! resets the handle it then claims the interface on, and a reset that
//! re-enumerates the device invalidates that handle: every write succeeds and
//! no answer ever comes back, and only unplugging the stick cleared it. That is
//! the state a restarted process inherited on a Pi. libusb's own answer is to
//! close the handle and rediscover the device, which is what each retry does:
//! reset at the USB level, drop the handle, wait out re-enumeration, look the
//! device up again.
//!
//! This lives in the library rather than the binary because the same loop is
//! the front end of every program built on this crate: the receiver firmware
//! runs it forever, reporting "missing" and "error" as two different states,
//! while `antdump` itself gives up after a few attempts and exits.

use crate::init::{InitError, LibConfigOutcome, configure};
use ant::drivers::{UsbDriver, UsbError, is_ant_usb_device_from_device};
use rusb::{Device, GlobalContext};
use std::fmt;
use std::time::Duration;

/// How many times `antdump` brings the dongle up before giving up. A stale
/// handle is cleared by a port reset and a fresh look at the bus, which is what
/// the retry does; more than a couple of failures is a dongle that needs a
/// human.
pub const INIT_ATTEMPTS: u32 = 3;

/// A port reset re-enumerates the device, so it is gone from the bus for a
/// moment and has to be found again rather than reused.
pub const REENUMERATE_DELAY: Duration = Duration::from_millis(500);

/// The driver type every USB caller ends up holding.
pub type Dongle = UsbDriver<GlobalContext>;

/// Why the dongle could not be brought up. The cases are different states to
/// whoever is watching: nothing on the bus is a stick that was unplugged,
/// while a bus that cannot be enumerated or a stick that will not answer is
/// something wrong on this side.
#[derive(Debug)]
pub enum BringUpError {
    /// The bus enumerated fine and no ANT+ dongle was on it.
    NoDongle,
    /// libusb could not list the bus at all.
    Enumerate(rusb::Error),
    /// A dongle was found and could not be opened or claimed.
    Open(UsbError),
    /// A dongle was found and every attempt failed; this is the last failure.
    Init(InitError),
}

impl fmt::Display for BringUpError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoDongle => write!(f, "no ANT+ dongle found"),
            Self::Enumerate(err) => write!(f, "listing USB devices failed: {err}"),
            Self::Open(err) => write!(f, "the dongle could not be opened: {err:?}"),
            Self::Init(err) => write!(f, "{err}"),
        }
    }
}

impl std::error::Error for BringUpError {}

/// A dongle that is configured and listening, plus what became of the one
/// optional configuration step.
pub struct BringUp {
    pub driver: Dongle,
    pub lib_config: LibConfigOutcome,
}

/// The first ANT+ dongle on the bus, if any. `Err` is the bus itself failing
/// to enumerate, which is not the same thing as an empty bus.
pub fn find_dongle() -> Result<Option<Device<GlobalContext>>, rusb::Error> {
    Ok(rusb::DeviceList::new()?
        .iter()
        .find(is_ant_usb_device_from_device))
}

/// Reset the dongle at the USB level and let it re-enumerate.
pub fn reset_dongle() {
    if let Ok(Some(device)) = find_dongle()
        && let Ok(handle) = device.open()
    {
        let _ = handle.reset();
    }
    // The handle is dropped here, which is the half that was missing.
    std::thread::sleep(REENUMERATE_DELAY);
}

/// Open the first dongle on the bus without configuring it.
pub fn open_dongle() -> Result<Dongle, BringUpError> {
    let device = match find_dongle() {
        Ok(Some(device)) => device,
        Ok(None) => return Err(BringUpError::NoDongle),
        Err(err) => return Err(BringUpError::Enumerate(err)),
    };
    UsbDriver::new(device).map_err(BringUpError::Open)
}

/// The three operations the retry loop is made of, so the loop itself can be
/// tested without a bus.
pub trait BringUpOps {
    type Driver;
    fn open(&mut self) -> Result<Self::Driver, BringUpError>;
    fn configure(&mut self, driver: &mut Self::Driver) -> Result<LibConfigOutcome, InitError>;
    fn reset(&mut self);
}

struct UsbOps;

impl BringUpOps for UsbOps {
    type Driver = Dongle;

    fn open(&mut self) -> Result<Dongle, BringUpError> {
        open_dongle()
    }

    fn configure(&mut self, driver: &mut Dongle) -> Result<LibConfigOutcome, InitError> {
        configure(driver)
    }

    fn reset(&mut self) {
        reset_dongle();
    }
}

/// Bring the dongle up as a promiscuous ANT+ receiver, resetting and
/// re-finding it between attempts. Each failed attempt goes to `report` with
/// its one-based number; the error returned is the last one.
pub fn bring_up(
    attempts: u32,
    report: impl FnMut(u32, &BringUpError),
) -> Result<BringUp, BringUpError> {
    bring_up_with(attempts, &mut UsbOps, report)
        .map(|(driver, lib_config)| BringUp { driver, lib_config })
}

/// [`bring_up`] over any set of operations.
pub fn bring_up_with<O: BringUpOps>(
    attempts: u32,
    ops: &mut O,
    mut report: impl FnMut(u32, &BringUpError),
) -> Result<(O::Driver, LibConfigOutcome), BringUpError> {
    let attempts = attempts.max(1);
    let mut last = BringUpError::NoDongle;
    for attempt in 1..=attempts {
        let mut driver = match ops.open() {
            Ok(driver) => driver,
            Err(err) => {
                report(attempt, &err);
                last = err;
                ops.reset();
                continue;
            }
        };

        match ops.configure(&mut driver) {
            Ok(lib_config) => return Ok((driver, lib_config)),
            Err(err) => {
                let err = BringUpError::Init(err);
                report(attempt, &err);
                last = err;
                // The handle goes before the reset: a reset that re-enumerates
                // invalidates it, and holding it is what left the stick deaf.
                drop(driver);
                ops.reset();
            }
        }
    }
    Err(last)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;

    #[derive(Debug, PartialEq)]
    enum Step {
        Open,
        Configure,
        Reset,
    }

    /// Scripted outcomes for each open and configure, in order, and a log of
    /// what the loop did with them.
    struct FakeOps {
        opens: VecDeque<Result<(), BringUpError>>,
        configures: VecDeque<Result<LibConfigOutcome, InitError>>,
        steps: Vec<Step>,
    }

    impl FakeOps {
        fn new(
            opens: Vec<Result<(), BringUpError>>,
            configures: Vec<Result<LibConfigOutcome, InitError>>,
        ) -> Self {
            Self {
                opens: opens.into(),
                configures: configures.into(),
                steps: Vec::new(),
            }
        }
    }

    impl BringUpOps for FakeOps {
        type Driver = ();

        fn open(&mut self) -> Result<(), BringUpError> {
            self.steps.push(Step::Open);
            self.opens.pop_front().expect("more opens than scripted")
        }

        fn configure(&mut self, (): &mut ()) -> Result<LibConfigOutcome, InitError> {
            self.steps.push(Step::Configure);
            self.configures
                .pop_front()
                .expect("more configures than scripted")
        }

        fn reset(&mut self) {
            self.steps.push(Step::Reset);
        }
    }

    fn deaf() -> InitError {
        InitError::Deaf { message: "a reset" }
    }

    fn unreported(_: u32, _: &BringUpError) {}

    #[test]
    fn a_healthy_dongle_comes_up_on_the_first_attempt_without_a_reset() {
        let mut ops = FakeOps::new(vec![Ok(())], vec![Ok(LibConfigOutcome::Unanswered)]);
        let ((), outcome) = bring_up_with(3, &mut ops, unreported).unwrap();
        assert_eq!(outcome, LibConfigOutcome::Unanswered);
        assert_eq!(ops.steps, [Step::Open, Step::Configure]);
    }

    #[test]
    fn a_deaf_dongle_is_reset_and_re_found_between_attempts() {
        let mut ops = FakeOps::new(
            vec![Ok(()), Ok(())],
            vec![Err(deaf()), Ok(LibConfigOutcome::Accepted)],
        );
        assert!(bring_up_with(3, &mut ops, unreported).is_ok());
        assert_eq!(
            ops.steps,
            [
                Step::Open,
                Step::Configure,
                Step::Reset,
                Step::Open,
                Step::Configure
            ]
        );
    }

    #[test]
    fn every_failed_attempt_is_reported_as_it_happens_and_a_success_is_not() {
        let mut ops = FakeOps::new(
            vec![Err(BringUpError::NoDongle), Ok(()), Ok(())],
            vec![Err(deaf()), Ok(LibConfigOutcome::Accepted)],
        );
        let mut reported = Vec::new();
        assert!(
            bring_up_with(3, &mut ops, |attempt, err| reported
                .push((attempt, err.to_string())))
            .is_ok()
        );
        assert_eq!(
            reported,
            [
                (1, "no ANT+ dongle found".to_owned()),
                (2, deaf().to_string()),
            ]
        );
    }

    #[test]
    fn an_open_failure_says_so_rather_than_posing_as_a_write() {
        let text = BringUpError::Open(UsbError::FailedToOpenDevice(rusb::Error::Busy)).to_string();
        assert!(
            text.starts_with("the dongle could not be opened: "),
            "{text}"
        );
        assert!(!text.contains("writing"), "{text}");
    }

    #[test]
    fn the_error_returned_is_the_last_attempts() {
        let mut ops = FakeOps::new(vec![Err(BringUpError::NoDongle), Ok(())], vec![Err(deaf())]);
        assert!(matches!(
            bring_up_with(2, &mut ops, unreported),
            Err(BringUpError::Init(_))
        ));
        assert_eq!(
            ops.steps,
            [
                Step::Open,
                Step::Reset,
                Step::Open,
                Step::Configure,
                Step::Reset
            ]
        );

        let mut ops = FakeOps::new(vec![Ok(()), Err(BringUpError::NoDongle)], vec![Err(deaf())]);
        assert!(matches!(
            bring_up_with(2, &mut ops, unreported),
            Err(BringUpError::NoDongle)
        ));
    }

    #[test]
    fn an_enumeration_failure_is_not_a_missing_dongle() {
        let mut ops = FakeOps::new(vec![Err(BringUpError::Enumerate(rusb::Error::Io))], vec![]);
        assert!(matches!(
            bring_up_with(1, &mut ops, unreported),
            Err(BringUpError::Enumerate(_))
        ));
    }

    #[test]
    fn zero_attempts_means_one() {
        let mut ops = FakeOps::new(vec![Err(BringUpError::NoDongle)], vec![]);
        assert!(matches!(
            bring_up_with(0, &mut ops, unreported),
            Err(BringUpError::NoDongle)
        ));
        assert_eq!(ops.steps, [Step::Open, Step::Reset]);
    }
}
