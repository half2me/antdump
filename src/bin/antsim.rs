//! Flood a room with simulated ANT+ bikes.
//!
//! The counterpart to `antdump`: this puts real packets on the air from a fleet
//! of virtual sensors so the receiver can be checked against traffic whose
//! every counter is known in advance.

use antdump::sim::{CHANNELS_PER_DONGLE, FleetSpec, MasterOps, Profile, SimDevice, capacity, run};
use antdump::usb::{DongleId, INIT_ATTEMPTS, bring_up_with, list_dongles};
use clap::Parser;
use std::io::{IsTerminal, Write};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use std::{io, process, thread};

/// Twice a second: fast enough to look live, slow enough that the numbers can
/// be read as they change.
const REFRESH: Duration = Duration::from_millis(500);

/// A fleet larger than this is summarised rather than listed, so the redraw
/// keeps fitting on a screen. The totals always cover every device.
const MAX_ROWS: usize = 32;

/// Simulate ANT+ bike sensors and transmit them on the air
#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    /// How many devices to simulate. Each dongle carries 8, so a bigger fleet
    /// uses more dongles; `--list-dongles` prints the ceiling.
    #[arg(long, short = 'n', default_value_t = 8)]
    devices: usize,

    /// ANT+ device number of the first device; the rest count up from it.
    /// Numbers above 65535 are legal and exercise the receiver's 20-bit path.
    #[arg(long, default_value_t = 1)]
    start_id: u32,

    /// Constant speed in km/h.
    #[arg(long, default_value_t = 25.0)]
    speed: f64,

    /// Constant cadence in rpm.
    #[arg(long, default_value_t = 85.0)]
    cadence: f64,

    /// Spread speeds across the fleet, in km/h: the last device rides this much
    /// faster than the first and the rest are spaced evenly between, with
    /// cadence following the same ratio. Zero makes every device identical,
    /// which is simpler to read but cannot reveal a receiver that attributes
    /// one device's page to another.
    #[arg(long, default_value_t = 0.0)]
    spread: f64,

    /// Wheel circumference in metres. The default is a 700x23c.
    #[arg(long, default_value_t = 2.096)]
    wheel: f64,

    /// Transmit from this dongle only: its USB serial or its port as
    /// `--list-dongles` prints it. Default: as many as the fleet needs.
    #[arg(long)]
    dongle: Option<String>,

    /// Print every ANT+ dongle on the bus and how many devices they can carry,
    /// then exit.
    #[arg(long)]
    list_dongles: bool,

    /// Transmit without drawing the status display.
    #[arg(long, short)]
    quiet: bool,
}

/// One dongle's share of the fleet, and what it has transmitted so far.
struct Assignment {
    id: DongleId,
    devices: Vec<SimDevice>,
    /// Transmissions per channel, bumped by the dongle's own thread.
    sent: Arc<Vec<AtomicU64>>,
}

fn main() -> io::Result<()> {
    let args = Args::parse();
    if args.list_dongles {
        return print_dongles();
    }

    let dongles = usable_dongles(args.dongle.as_deref())?;
    let spec = FleetSpec {
        devices: args.devices,
        start_id: args.start_id,
        profile: Profile::SpeedAndCadence,
        speed_kph: args.speed,
        cadence_rpm: args.cadence,
        spread_kph: args.spread,
        wheel_circumference_m: args.wheel,
    };

    let fleet = spec.build().unwrap_or_else(|err| fatal(&err.to_string()));
    let shards =
        FleetSpec::shard(fleet, dongles.len()).unwrap_or_else(|err| fatal(&err.to_string()));

    let assignments: Vec<Assignment> = shards
        .into_iter()
        .zip(dongles)
        .map(|(devices, id)| Assignment {
            sent: Arc::new((0..devices.len()).map(|_| AtomicU64::new(0)).collect()),
            devices,
            id,
        })
        .collect();

    let total: usize = assignments.iter().map(|a| a.devices.len()).sum();
    eprintln!(
        "Bringing up {total} device(s) on {} dongle(s)...",
        assignments.len()
    );

    // One origin for every device on every stick, fixed before any of them come
    // up: the counters then describe one shared clock rather than each dongle's
    // own start, which is what makes two sticks' output comparable.
    let start = Instant::now();
    let stopped: Arc<Mutex<Option<String>>> = Arc::new(Mutex::new(None));

    for assignment in &assignments {
        spawn_dongle(assignment, start, &stopped);
    }

    display_loop(&assignments, start, &stopped, args.quiet)
}

/// Every ANT+ dongle on the bus, and the fleet size they add up to.
fn print_dongles() -> io::Result<()> {
    let dongles = list_dongles().map_err(io::Error::other)?;
    if dongles.is_empty() {
        eprintln!("no ANT+ dongle found");
        process::exit(1);
    }
    for dongle in &dongles {
        println!("{dongle}");
    }
    println!();
    println!(
        "{} dongle(s) x {CHANNELS_PER_DONGLE} channels = {} simulated devices maximum.",
        dongles.len(),
        capacity(&dongles)
    );
    println!(
        "{CHANNELS_PER_DONGLE} is the radio's limit, not a setting: these sticks are eight-channel parts."
    );
    if dongles.len() == 1 {
        println!();
        println!(
            "Note: one dongle staggers its own channels, so its devices never collide with \
             each other. Testing collision detection needs a second dongle transmitting, \
             and a third to receive on."
        );
    }
    Ok(())
}

/// The dongles the fleet may use, which is every one on the bus unless the
/// caller named a single stick.
fn usable_dongles(selector: Option<&str>) -> io::Result<Vec<DongleId>> {
    let dongles: Vec<DongleId> = list_dongles()
        .map_err(io::Error::other)?
        .into_iter()
        .filter(|dongle| dongle.matches(selector))
        .collect();
    if dongles.is_empty() {
        match selector {
            Some(wanted) => fatal(&format!("no ANT+ dongle matching {wanted}")),
            None => fatal("no ANT+ dongle found"),
        }
    }
    Ok(dongles)
}

/// Bring one stick up as a set of masters and feed it until it stops.
///
/// The stick is named by its port rather than taken as "the first one found",
/// because a fleet spread over several dongles must not reset or claim a
/// neighbour's: resetting the wrong stick is precisely how one process leaves
/// another's deaf.
fn spawn_dongle(assignment: &Assignment, start: Instant, stopped: &Arc<Mutex<Option<String>>>) {
    let label = assignment.id.to_string();
    let port = assignment.id.port.clone();
    let devices = assignment.devices.clone();
    let sent = Arc::clone(&assignment.sent);
    let stopped = Arc::clone(stopped);

    thread::spawn(move || {
        let (brought_up, progress) = {
            let mut ops = MasterOps::new(Some(&port), &devices);
            let result = bring_up_with(INIT_ATTEMPTS, &mut ops, |attempt, err| {
                eprintln!("ERROR: {label}: {err} (attempt {attempt} of {INIT_ATTEMPTS})");
            });
            (result, ops.progress)
        };

        let failure = match brought_up {
            Ok(mut driver) => run(&mut driver, &devices, start, &sent).to_string(),
            Err(err) => match progress {
                Some(channel) => format!("{err} (reached channel {channel})"),
                None => err.to_string(),
            },
        };
        // First failure wins: the rest of the fleet is about to be torn down
        // anyway, and the first stick to go is the one worth naming.
        stopped
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .get_or_insert(format!("{label}: {failure}"));
    });
}

/// Draw the fleet until a dongle stops, which ends the process.
fn display_loop(
    assignments: &[Assignment],
    start: Instant,
    stopped: &Arc<Mutex<Option<String>>>,
    quiet: bool,
) -> io::Result<()> {
    let live = !quiet && io::stdout().is_terminal();
    if !live && !quiet {
        // Without a terminal to redraw in, the roster is printed once and the
        // totals tick past on their own lines instead.
        print!("{}", roster(assignments));
        io::stdout().flush()?;
    }

    let mut drawn = 0usize;
    loop {
        if let Some(failure) = stopped
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .clone()
        {
            if live && drawn > 0 {
                println!();
            }
            fatal(&failure);
        }

        if live {
            let frame = frame(assignments, start.elapsed());
            let mut out = io::stdout().lock();
            if drawn > 0 {
                // Back to the top of the last frame and wipe from there down,
                // so a frame that shrinks does not leave its old tail behind.
                write!(out, "\x1b[{drawn}F\x1b[0J")?;
            }
            write!(out, "{frame}")?;
            out.flush()?;
            drawn = frame.lines().count();
        } else if !quiet {
            let elapsed = start.elapsed();
            let sent = total_sent(assignments);
            println!(
                "{}  sent {}  ({:.1} pkt/s)",
                clock(elapsed),
                commas(sent),
                rate(sent, elapsed)
            );
        }
        thread::sleep(REFRESH);
    }
}

/// The static part: which device is on which stick.
fn roster(assignments: &[Assignment]) -> String {
    let mut out = String::new();
    for assignment in assignments {
        out.push_str(&format!("{}\n", assignment.id));
        for (channel, device) in assignment.devices.iter().enumerate() {
            out.push_str(&format!(
                "  ch{channel} {:>11}  {:>6.1} kph  {:>5.1} rpm\n",
                key(device),
                device.speed_kph,
                device.cadence_rpm
            ));
        }
    }
    out
}

/// One redraw of the whole fleet.
fn frame(assignments: &[Assignment], elapsed: Duration) -> String {
    let devices: usize = assignments.iter().map(|a| a.devices.len()).sum();
    let sent = total_sent(assignments);
    let profile = assignments
        .first()
        .and_then(|a| a.devices.first())
        .map_or(Profile::SpeedAndCadence, |d| d.profile);
    // What the radios should be managing between them. A measured rate below
    // this is the fleet losing transmissions, which is worth seeing at a glance.
    let expected = devices as f64 * profile.hz();

    let mut out = String::new();
    out.push_str(&format!(
        "antsim  {devices} device(s)  {} dongle(s)  {profile}  {:.2} Hz each\n",
        assignments.len(),
        profile.hz()
    ));
    out.push_str(&format!(
        "up {}   sent {}   {:.1} pkt/s   expected {expected:.1} pkt/s\n\n",
        clock(elapsed),
        commas(sent),
        rate(sent, elapsed),
    ));
    out.push_str(&format!(
        "  {:<12} {:<12} {:>2}  {:>8} {:>9} {:>8} {:>8} {:>10}\n",
        "KEY", "DONGLE", "CH", "SPEED", "CADENCE", "S.REV", "C.REV", "SENT"
    ));

    let mut rows = 0;
    for assignment in assignments {
        for (channel, device) in assignment.devices.iter().enumerate() {
            if rows == MAX_ROWS {
                out.push_str(&format!(
                    "  ... and {} more device(s); the totals above cover them all\n",
                    devices - rows
                ));
                return out;
            }
            let (wheel, crank) = device.revolutions(elapsed);
            out.push_str(&format!(
                "  {:<12} {:<12} {channel:>2}  {:>6.1}kph {:>6.1}rpm {wheel:>8} {crank:>8} {:>10}\n",
                key(device),
                assignment.id.port,
                device.speed_kph,
                device.cadence_rpm,
                commas(assignment.sent[channel].load(Ordering::Relaxed)),
            ));
            rows += 1;
        }
    }
    out
}

/// `device_number:device_type_id`, which is the same key `antdump` prints in
/// front of every packet, so the two outputs line up by eye and by `grep`.
fn key(device: &SimDevice) -> String {
    format!("{}:{}", device.device_number, device.profile.device_type())
}

fn total_sent(assignments: &[Assignment]) -> u64 {
    assignments
        .iter()
        .flat_map(|a| a.sent.iter())
        .map(|counter| counter.load(Ordering::Relaxed))
        .sum()
}

fn rate(sent: u64, elapsed: Duration) -> f64 {
    let secs = elapsed.as_secs_f64();
    if secs > 0.0 { sent as f64 / secs } else { 0.0 }
}

fn clock(elapsed: Duration) -> String {
    let secs = elapsed.as_secs();
    format!("{:02}:{:02}:{:02}", secs / 3600, secs / 60 % 60, secs % 60)
}

/// Thousands separators, because the packet counts are the numbers being read
/// and six undivided digits do not read.
fn commas(n: u64) -> String {
    let digits = n.to_string();
    let mut out = String::with_capacity(digits.len() + digits.len() / 3);
    for (i, ch) in digits.chars().enumerate() {
        if i > 0 && (digits.len() - i).is_multiple_of(3) {
            out.push(',');
        }
        out.push(ch);
    }
    out
}

fn fatal(message: &str) -> ! {
    eprintln!("FATAL: {message}");
    process::exit(1);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn counts_read_with_separators_and_short_ones_are_left_alone() {
        assert_eq!(commas(0), "0");
        assert_eq!(commas(999), "999");
        assert_eq!(commas(1000), "1,000");
        assert_eq!(commas(12_345), "12,345");
        assert_eq!(commas(1_234_567), "1,234,567");
    }

    #[test]
    fn the_clock_carries_hours_and_pads() {
        assert_eq!(clock(Duration::from_secs(0)), "00:00:00");
        assert_eq!(clock(Duration::from_secs(83)), "00:01:23");
        assert_eq!(clock(Duration::from_secs(3661)), "01:01:01");
        assert_eq!(clock(Duration::from_secs(360_000)), "100:00:00");
    }

    /// The key is what ties a row of the display to a line of `antdump`, so its
    /// shape is not cosmetic.
    #[test]
    fn the_key_matches_the_one_antdump_prints() {
        let device = SimDevice::new(70_000, Profile::SpeedAndCadence, 25.0, 85.0, 2.096);
        assert_eq!(key(&device), "70000:121");
    }

    /// Assignments shaped directly, rather than through `shard`, so a test can
    /// ask for a layout that sharding would never produce.
    fn assignments(dongles: usize, per_dongle: usize) -> Vec<Assignment> {
        let fleet = FleetSpec {
            devices: dongles * per_dongle,
            start_id: 65_533,
            profile: Profile::SpeedAndCadence,
            speed_kph: 25.0,
            cadence_rpm: 85.0,
            spread_kph: 10.0,
            wheel_circumference_m: 2.096,
        }
        .build()
        .unwrap();

        fleet
            .chunks(per_dongle)
            .enumerate()
            .map(|(i, devices)| Assignment {
                id: DongleId {
                    port: format!("20-1.{}", i + 1),
                    serial: Some(format!("15508033{i}")),
                },
                devices: devices.to_vec(),
                sent: Arc::new(
                    (0..devices.len())
                        .map(|ch| AtomicU64::new((1000 + i * per_dongle + ch) as u64))
                        .collect(),
                ),
            })
            .collect()
    }

    /// The display is the whole interface while it runs, so its frame is
    /// checked for the pieces that make it readable rather than left to chance.
    #[test]
    fn a_frame_carries_the_totals_the_roster_and_a_row_per_device() {
        let assignments = assignments(2, 3);
        let frame = frame(&assignments, Duration::from_secs(83));

        assert!(frame.contains("6 device(s)  2 dongle(s)"), "{frame}");
        assert!(frame.contains("up 00:01:23"), "{frame}");
        // 6 devices at 4.05 Hz is what the radios should be managing.
        assert!(frame.contains("expected 24.3 pkt/s"), "{frame}");
        // A key per device, in antdump's own format, including one that has
        // crossed 65535 and so rides in the extension nibble.
        assert!(frame.contains("65533:121"), "{frame}");
        assert!(frame.contains("65536:121"), "{frame}");
        // Both sticks named, and every device given a row.
        assert!(
            frame.contains("20-1.1") && frame.contains("20-1.2"),
            "{frame}"
        );
        assert_eq!(frame.lines().filter(|l| l.contains(":121")).count(), 6);
    }

    /// A fleet too tall to redraw is summarised instead, but the totals still
    /// count every device.
    #[test]
    fn an_oversized_fleet_is_truncated_with_a_note_rather_than_scrolling_away() {
        let assignments = assignments(6, 8);
        let frame = frame(&assignments, Duration::from_secs(10));
        assert!(frame.contains("48 device(s)"), "{frame}");
        assert!(frame.contains("... and 16 more device(s)"), "{frame}");
        assert_eq!(
            frame.lines().filter(|l| l.contains(":121")).count(),
            MAX_ROWS
        );
    }

    #[test]
    fn the_rate_is_zero_before_any_time_has_passed_rather_than_infinite() {
        assert_eq!(rate(10, Duration::ZERO), 0.0);
        assert!((rate(100, Duration::from_secs(4)) - 25.0).abs() < f64::EPSILON);
    }
}
