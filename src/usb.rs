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
use ant::drivers::{UsbDriver, is_ant_usb_device_from_device};
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

/// Why the dongle could not be brought up. The two cases are different states
/// to whoever is watching: nothing on the bus is a stick that was unplugged,
/// while a stick that is there and will not answer is one that needs a reset.
#[derive(Debug)]
pub enum BringUpError {
    /// No ANT+ dongle on the bus at all.
    NoDongle,
    /// A dongle was found and every attempt failed; this is the last failure.
    Init(InitError),
}

impl fmt::Display for BringUpError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoDongle => write!(f, "no ANT+ dongle found"),
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

/// The first ANT+ dongle on the bus, if any.
pub fn find_dongle() -> Option<Device<GlobalContext>> {
    rusb::DeviceList::new()
        .ok()?
        .iter()
        .find(is_ant_usb_device_from_device)
}

/// Reset the dongle at the USB level and let it re-enumerate.
pub fn reset_dongle() {
    if let Some(device) = find_dongle()
        && let Ok(handle) = device.open()
    {
        let _ = handle.reset();
    }
    // The handle is dropped here, which is the half that was missing.
    std::thread::sleep(REENUMERATE_DELAY);
}

/// Open the first dongle on the bus without configuring it.
pub fn open_dongle() -> Result<Dongle, BringUpError> {
    let device = find_dongle().ok_or(BringUpError::NoDongle)?;
    UsbDriver::new(device).map_err(|_| {
        BringUpError::Init(InitError::Write {
            message: "the dongle could not be opened",
        })
    })
}

/// Bring the dongle up as a promiscuous ANT+ receiver, resetting and
/// re-finding it between attempts.
///
/// Each failed attempt is reported on stderr as it happens, and the error
/// returned is the last one. `attempts` of zero is treated as one.
pub fn bring_up(attempts: u32) -> Result<BringUp, BringUpError> {
    let attempts = attempts.max(1);
    let mut last = BringUpError::NoDongle;
    for attempt in 1..=attempts {
        let mut driver = match open_dongle() {
            Ok(driver) => driver,
            Err(err) => {
                eprintln!("ERROR: {err} (attempt {attempt} of {attempts})");
                last = err;
                reset_dongle();
                continue;
            }
        };

        match configure(&mut driver) {
            Ok(lib_config) => return Ok(BringUp { driver, lib_config }),
            Err(err) => {
                eprintln!("ERROR: {err} (attempt {attempt} of {attempts})");
                last = BringUpError::Init(err);
                drop(driver);
                reset_dongle();
            }
        }
    }
    Err(last)
}
