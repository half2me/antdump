//! Flood a room with simulated ANT+ bikes.
//!
//! The counterpart to `antdump`: this puts real packets on the air from a fleet
//! of virtual sensors so the receiver can be checked against traffic whose
//! every counter is known in advance.

use antdump::init::reset;
use antdump::sim::{
    CHANNELS_PER_DONGLE, DeviceState, FleetSpec, MasterOps, Profile, SPEED_CADENCE_AND_POWER,
    SPEED_CADENCE_ONLY, SimDevice, capacity, pump, shut_down,
};
use antdump::usb::{Dongle, DongleId, INIT_ATTEMPTS, bring_up_with, list_dongles, open_dongle};
use clap::Parser;
use std::io::{IsTerminal, Write};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::time::{Duration, Instant};
use std::{io, process};

/// Set by a signal handler, read by every transmit loop. An open master channel
/// outlives the process that opened it, so Ctrl-C has to mean "close the
/// channels and then exit" rather than just "exit".
static SHUTDOWN: AtomicBool = AtomicBool::new(false);

/// The only thing a signal handler may safely do here: flip a flag. The
/// closing down is done by the threads that own the dongles, once they notice.
extern "C" fn on_signal(_: libc::c_int) {
    SHUTDOWN.store(true, Ordering::SeqCst);
}

fn catch_interrupts() {
    // SAFETY: `on_signal` only stores to an atomic, which is async-signal-safe.
    unsafe {
        libc::signal(libc::SIGINT, on_signal as *const () as libc::sighandler_t);
        libc::signal(libc::SIGTERM, on_signal as *const () as libc::sighandler_t);
    }
}

/// Twice a second: fast enough to look live, slow enough that the numbers can
/// be read as they change.
const REFRESH: Duration = Duration::from_millis(500);

/// A fleet larger than this is summarised rather than listed, so the redraw
/// keeps fitting on a screen. The totals always cover every device.
const MAX_ROWS: usize = 32;

/// The span `--max-spread` fans the fleet across.
///
/// The speed endpoints are picked to straddle the profile's own 4.05 Hz
/// broadcast rate rather than to be round numbers. On the default wheel, 5 km/h
/// is about 0.66 wheel revolutions a second, so six broadcasts running carry an
/// identical count and event time; 60 km/h is nearly 8 a second, so the count
/// climbs by two between broadcasts. Those are the two regimes a receiver can
/// get wrong, and this puts both on the air at once.
///
/// Cadence gets its own span because a crank does not speed up with the road:
/// 40 to 120 rpm covers a plausible range and stays below the broadcast rate
/// throughout, which is where real cadence always sits.
const SPREAD_SLOWEST_KPH: f64 = 5.0;
const SPREAD_FASTEST_KPH: f64 = 60.0;
const SPREAD_SLOWEST_RPM: f64 = 40.0;
const SPREAD_FASTEST_RPM: f64 = 120.0;
const SPREAD_LOWEST_W: f64 = 50.0;
const SPREAD_HIGHEST_W: f64 = 600.0;

/// Simulate ANT+ bike sensors and transmit them on the air
#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    /// How many bikes to simulate. Each bike takes one channel per sensor and
    /// each dongle has 8, so a dongle carries 4 bikes with power or 8 with
    /// `--no-power`; `--list-dongles` prints both ceilings.
    /// [default: one dongle's worth]
    #[arg(long, short = 'n', conflicts_with = "max")]
    devices: Option<usize>,

    /// Fill every dongle: as many bikes as the hardware can carry, which is 4
    /// per stick with power or 8 with `--no-power`. Respects `--dongle`, so
    /// naming one stick fills that stick alone and leaves the others free to
    /// receive on.
    #[arg(long, conflicts_with = "devices")]
    max: bool,

    /// ANT+ device number of the first device; the rest count up from it.
    /// Numbers above 65535 are legal and exercise the receiver's 20-bit path.
    #[arg(long, default_value_t = 1)]
    start_id: u32,

    /// Constant speed in km/h.
    #[arg(long, default_value_t = 25.0, conflicts_with = "max_spread")]
    speed: f64,

    /// Constant cadence in rpm.
    #[arg(long, default_value_t = 85.0, conflicts_with = "max_spread")]
    cadence: f64,

    /// Spread speeds across the fleet, in km/h: the last device rides this much
    /// faster than the first and the rest are spaced evenly between, with
    /// cadence following the same ratio. Zero makes every device identical,
    /// which is simpler to read but cannot reveal a receiver that attributes
    /// one device's page to another.
    #[arg(long, default_value_t = 0.0, conflicts_with = "max_spread")]
    spread: f64,

    /// Fan the fleet across the whole plausible range instead of a base speed
    /// and an offset: 5 to 60 km/h, 40 to 120 rpm, 50 to 600 W. The speed
    /// endpoints straddle
    /// the profile's own broadcast rate, so the slow devices repeat a counter
    /// for several broadcasts running while the fast ones advance it by two
    /// between broadcasts, and both cases are on the air at once.
    #[arg(long, conflicts_with_all = ["speed", "spread"])]
    max_spread: bool,

    /// Constant power in watts, on each bike's power channel.
    #[arg(long, default_value_t = 200, conflicts_with = "max_spread")]
    watts: u16,

    /// Leave the power meter off, so each bike is speed and cadence alone.
    /// That is one channel per bike instead of two, which fits twice as many
    /// bikes on the same dongles: 8 per stick rather than 4.
    #[arg(long)]
    no_power: bool,

    /// Wheel circumference in metres. The default is a 700x23c.
    #[arg(long, default_value_t = 2.096)]
    wheel: f64,

    /// Transmit from this dongle only: its USB serial or its port as
    /// `--list-dongles` prints it. Default: as many as the fleet needs.
    #[arg(long)]
    dongle: Option<String>,

    /// Print every ANT+ dongle on the bus and how many bikes they can carry,
    /// then exit.
    #[arg(long)]
    list_dongles: bool,

    /// Reset the dongles and exit, silencing any channels left transmitting by
    /// a previous run that did not shut down cleanly. Respects `--dongle`.
    #[arg(long)]
    reset: bool,

    /// Transmit without drawing the status display.
    #[arg(long, short)]
    quiet: bool,
}

/// One dongle, its share of the fleet, and what it has transmitted so far.
struct Stick {
    /// How the stick describes itself, for messages and the roster. Taken once
    /// rather than formatted twice a second, and it means nothing here depends
    /// on the shape of `DongleId`, which is `non_exhaustive` and grows fields.
    label: String,
    /// The bus and port chain alone, which is what the table's narrow column
    /// shows and what names this stick to the driver.
    port: String,
    driver: Dongle,
    devices: Vec<SimDevice>,
    /// Transmissions per channel.
    sent: Vec<AtomicU64>,
}

/// What the display needs from a stick, which is everything except the driver.
///
/// Split out because a `Dongle` cannot be constructed without hardware, and the
/// layout is worth testing on a machine that has none. Plain strings rather
/// than a `DongleId` for the same reason: that type is `non_exhaustive`, so a
/// test outside the library crate cannot build one.
struct Panel<'a> {
    label: &'a str,
    port: &'a str,
    devices: &'a [SimDevice],
    sent: &'a [AtomicU64],
}

impl Stick {
    fn panel(&self) -> Panel<'_> {
        Panel {
            label: &self.label,
            port: &self.port,
            devices: &self.devices,
            sent: &self.sent,
        }
    }
}

fn main() -> io::Result<()> {
    let args = Args::parse();
    if args.list_dongles {
        return print_dongles();
    }
    if args.reset {
        return reset_dongles(args.dongle.as_deref());
    }
    catch_interrupts();

    let dongles = usable_dongles(args.dongle.as_deref())?;
    let profiles = if args.no_power {
        SPEED_CADENCE_ONLY
    } else {
        SPEED_CADENCE_AND_POWER
    };
    let spans = Spans::of(&args);
    let spec = FleetSpec {
        devices: fleet_size(args.max, args.devices, dongles.len(), profiles.len()),
        start_id: args.start_id,
        profiles,
        speed_kph: spans.speed,
        cadence_rpm: spans.cadence,
        power_watts: spans.watts,
        spread_kph: spans.speed_span,
        cadence_spread_rpm: spans.cadence_span,
        power_spread_w: spans.watts_span,
        wheel_circumference_m: args.wheel,
    };

    let fleet = spec.build().unwrap_or_else(|err| fatal(&err.to_string()));
    let shards =
        FleetSpec::shard(fleet, dongles.len()).unwrap_or_else(|err| fatal(&err.to_string()));

    let channels: usize = shards.iter().map(Vec::len).sum();
    eprintln!(
        "Bringing up {channels} channel(s) on {} dongle(s)...",
        shards.len()
    );

    // One origin for every device on every stick, fixed before any of them come
    // up: the counters then describe one shared clock rather than each dongle's
    // own start, which is what makes two sticks' output comparable.
    let start = Instant::now();

    let mut sticks: Vec<Stick> = Vec::new();
    for (devices, id) in shards.into_iter().zip(dongles) {
        match bring_up_stick(&id, devices) {
            Ok(stick) => sticks.push(stick),
            Err(message) => {
                // Sticks already up are transmitting, so they have to be put
                // back before this gives up. Exiting straight to `fatal` would
                // leave them broadcasting.
                shut_down_all(&mut sticks);
                fatal(&message);
            }
        }
    }

    let outcome = transmit(&mut sticks, start, args.quiet);
    eprintln!(
        "Stopping: closing channels on {} dongle(s)...",
        sticks.len()
    );
    shut_down_all(&mut sticks);

    match outcome? {
        Some(message) => fatal(&message),
        None => Ok(()),
    }
}

/// Configure one stick's channels and open them.
///
/// The stick is named by its port rather than taken as "the first one found",
/// because a fleet spread over several dongles must not reset or claim a
/// neighbour's: resetting the wrong stick is precisely how one process leaves
/// another's deaf.
fn bring_up_stick(id: &DongleId, devices: Vec<SimDevice>) -> Result<Stick, String> {
    let label = id.to_string();
    let (brought_up, progress) = {
        let mut ops = MasterOps::new(Some(&id.port), &devices);
        let result = bring_up_with(INIT_ATTEMPTS, &mut ops, |attempt, err| {
            eprintln!("ERROR: {label}: {err} (attempt {attempt} of {INIT_ATTEMPTS})");
        });
        (result, ops.progress)
    };

    match brought_up {
        Ok(driver) => Ok(Stick {
            port: id.port.clone(),
            label,
            driver,
            sent: (0..devices.len()).map(|_| AtomicU64::new(0)).collect(),
            devices,
        }),
        Err(err) => Err(match progress {
            Some(channel) => format!("{label}: {err} (reached channel {channel})"),
            None => format!("{label}: {err}"),
        }),
    }
}

/// Drive every stick from this one thread, redrawing between turns.
///
/// **One thread is the point, not a simplification.** Giving each dongle a
/// thread segfaulted on macOS: the handles are `Send` and the Rust side is
/// sound, but two threads doing concurrent synchronous bulk transfers on a
/// shared libusb context is not a path that library is reliably safe on. It
/// also bought nothing — see [`pump`] for why a round-robin has a comfortable
/// margin over the rate the radios ask for payloads at.
fn transmit(sticks: &mut [Stick], start: Instant, quiet: bool) -> io::Result<Option<String>> {
    let live = !quiet && io::stdout().is_terminal();
    if !live && !quiet {
        // Without a terminal to redraw in, the roster is printed once and the
        // totals tick past on their own lines instead.
        let panels: Vec<Panel> = sticks.iter().map(Stick::panel).collect();
        print!("{}", roster(&panels));
        io::stdout().flush()?;
    }

    let mut drawn = 0usize;
    let mut next_draw = Instant::now();
    loop {
        if SHUTDOWN.load(Ordering::SeqCst) {
            if live && drawn > 0 {
                println!();
            }
            return Ok(None);
        }

        for stick in &mut *sticks {
            if let Err(err) = pump(&mut stick.driver, &stick.devices, start, &stick.sent) {
                if live && drawn > 0 {
                    println!();
                }
                return Ok(Some(format!("{}: {err}", stick.label)));
            }
        }

        if Instant::now() >= next_draw {
            next_draw += REFRESH;
            let panels: Vec<Panel> = sticks.iter().map(Stick::panel).collect();
            draw(&panels, start, live, &mut drawn, quiet)?;
        }
    }
}

/// One refresh of whatever this run is showing.
fn draw(
    sticks: &[Panel],
    start: Instant,
    live: bool,
    drawn: &mut usize,
    quiet: bool,
) -> io::Result<()> {
    if live {
        let frame = frame(sticks, start.elapsed());
        let mut out = io::stdout().lock();
        if *drawn > 0 {
            // Back to the top of the last frame and wipe from there down, so a
            // frame that shrinks does not leave its old tail behind.
            write!(out, "\x1b[{drawn}F\x1b[0J")?;
        }
        write!(out, "{frame}")?;
        out.flush()?;
        *drawn = frame.lines().count();
    } else if !quiet {
        let elapsed = start.elapsed();
        let sent = total_sent(sticks);
        println!(
            "{}  sent {}  ({:.1} pkt/s)",
            clock(elapsed),
            commas(sent),
            rate(sent, elapsed)
        );
    }
    Ok(())
}

/// Put every stick back. An open master channel is the dongle's state, not
/// this process's, and outlives it.
fn shut_down_all(sticks: &mut [Stick]) {
    for stick in sticks {
        if let Err(err) = shut_down(&mut stick.driver) {
            eprintln!(
                "WARNING: {}: {err}. It may still be transmitting; \
                 `antsim --reset` or a replug will silence it.",
                stick.label
            );
        }
    }
}

/// Reset every dongle the selector takes, so a stick left transmitting by a
/// previous run goes quiet.
fn reset_dongles(selector: Option<&str>) -> io::Result<()> {
    let dongles = usable_dongles(selector)?;
    let mut failed = false;
    for dongle in &dongles {
        match open_dongle(Some(&dongle.port))
            .map_err(|err| err.to_string())
            .and_then(|mut driver| reset(&mut driver).map_err(|err| err.to_string()))
        {
            Ok(()) => println!("{dongle}: reset"),
            Err(err) => {
                eprintln!("{dongle}: {err}");
                failed = true;
            }
        }
    }
    if failed {
        process::exit(1);
    }
    Ok(())
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
    let channels = capacity(dongles.len());
    println!();
    println!(
        "{} dongle(s) x {CHANNELS_PER_DONGLE} channels = {channels} channels.",
        dongles.len()
    );
    println!(
        "  {} bikes with speed&cadence + power (2 channels each)",
        channels / SPEED_CADENCE_AND_POWER.len()
    );
    println!("  {channels} bikes with speed&cadence only (--no-power, 1 channel each)");
    println!(
        "{CHANNELS_PER_DONGLE} is the radio's limit, not a setting: these sticks are eight-channel parts."
    );
    println!("Run `antsim --max` to fill them, or `--max --dongle <serial>` to fill just one.");
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

/// Where each quantity starts and how far it fans across the fleet.
struct Spans {
    speed: f64,
    speed_span: f64,
    cadence: f64,
    cadence_span: f64,
    watts: f64,
    watts_span: f64,
}

impl Spans {
    /// Outside `--max-spread`, cadence and power keep the proportional
    /// behaviour that reads naturally over a few km/h: a bike rolling 20%
    /// faster also pedals and pushes 20% harder. That scaling lives here rather
    /// than in `FleetSpec` because it is a choice about what looks right and
    /// not about how a fleet is built — and it is exactly the choice
    /// `--max-spread` has to make differently, since following the speed ratio
    /// from 5 to 60 km/h would ask for 1020 rpm and 2400 W.
    fn of(args: &Args) -> Self {
        if args.max_spread {
            return Self {
                speed: SPREAD_SLOWEST_KPH,
                speed_span: SPREAD_FASTEST_KPH - SPREAD_SLOWEST_KPH,
                cadence: SPREAD_SLOWEST_RPM,
                cadence_span: SPREAD_FASTEST_RPM - SPREAD_SLOWEST_RPM,
                watts: SPREAD_LOWEST_W,
                watts_span: SPREAD_HIGHEST_W - SPREAD_LOWEST_W,
            };
        }
        let ratio = if args.speed > 0.0 {
            args.spread / args.speed
        } else {
            0.0
        };
        Self {
            speed: args.speed,
            speed_span: args.spread,
            cadence: args.cadence,
            cadence_span: args.cadence * ratio,
            watts: f64::from(args.watts),
            watts_span: f64::from(args.watts) * ratio,
        }
    }
}

/// How many bikes to simulate: as many as the hardware carries under `--max`,
/// otherwise what was asked for, otherwise one dongle's worth.
///
/// `--max` divides by the channels each bike needs, so turning the power meter
/// on halves the answer rather than asking for a fleet that cannot fit.
fn fleet_size(
    max: bool,
    devices: Option<usize>,
    dongles: usize,
    channels_per_device: usize,
) -> usize {
    let per_dongle = CHANNELS_PER_DONGLE / channels_per_device.max(1);
    if max {
        capacity(dongles) / channels_per_device.max(1)
    } else {
        devices.unwrap_or(per_dongle)
    }
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

/// The static part: which device is on which stick.
fn roster(sticks: &[Panel]) -> String {
    let mut out = String::new();
    for stick in sticks {
        out.push_str(&format!("{}\n", stick.label));
        for (channel, device) in stick.devices.iter().enumerate() {
            out.push_str(&format!(
                "  ch{channel} {:>11}  {:<13}  {}\n",
                key(device),
                device.profile.to_string(),
                riding(device)
            ));
        }
    }
    out
}

/// One redraw of the whole fleet.
fn frame(sticks: &[Panel], elapsed: Duration) -> String {
    let devices: usize = sticks.iter().map(|s| s.devices.len()).sum();
    let sent = total_sent(sticks);
    // What the radios should be managing between them, summed over every
    // channel because the two profiles broadcast at slightly different rates.
    // A measured rate below this is the fleet losing transmissions, which is
    // worth seeing at a glance.
    let expected: f64 = sticks
        .iter()
        .flat_map(|s| s.devices.iter())
        .map(|d| d.profile.hz())
        .sum();
    let bikes = sticks
        .iter()
        .flat_map(|s| s.devices.iter())
        .map(|d| d.device_number)
        .collect::<std::collections::BTreeSet<_>>()
        .len();

    let mut out = String::new();
    out.push_str(&format!(
        "antsim  {bikes} bike(s)  {devices} channel(s)  {} dongle(s)\n",
        sticks.len(),
    ));
    out.push_str(&format!(
        "up {}   sent {}   {:.1} pkt/s   expected {expected:.1} pkt/s\n\n",
        clock(elapsed),
        commas(sent),
        rate(sent, elapsed),
    ));
    out.push_str(&format!(
        "  {:<12} {:<8} {:>2}  {:<14} {:<18} {:<24} {:>10}\n",
        "KEY", "DONGLE", "CH", "PROFILE", "RIDING", "COUNTERS", "SENT"
    ));

    let mut rows = 0;
    for stick in sticks {
        for (channel, device) in stick.devices.iter().enumerate() {
            if rows == MAX_ROWS {
                out.push_str(&format!(
                    "  ... and {} more channel(s); the totals above cover them all\n",
                    devices - rows
                ));
                return out;
            }
            out.push_str(&format!(
                "  {:<12} {:<8} {channel:>2}  {:<14} {:<18} {:<24} {:>10}\n",
                key(device),
                stick.port,
                device.profile.to_string(),
                riding(device),
                counters(device, elapsed),
                commas(stick.sent[channel].load(Ordering::Relaxed)),
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

/// What the bike is doing, in the terms its own profile transmits: a speed and
/// cadence sensor knows nothing about watts, and a power meter knows nothing
/// about road speed.
fn riding(device: &SimDevice) -> String {
    match device.profile {
        Profile::SpeedAndCadence => {
            format!("{:.1}kph {:.0}rpm", device.speed_kph, device.cadence_rpm)
        }
        Profile::Power => format!("{}W {:.0}rpm", device.power_watts, device.cadence_rpm),
    }
}

/// The counters as they went on the air. These are what `antdump` has to get
/// right, so they are shown as the numbers in the payload rather than as the
/// speed or power a receiver would derive from them.
fn counters(device: &SimDevice, elapsed: Duration) -> String {
    match device.state(elapsed) {
        DeviceState::SpeedAndCadence {
            wheel_revs,
            crank_revs,
        } => format!("s.rev {wheel_revs}  c.rev {crank_revs}"),
        DeviceState::Power {
            events,
            accumulated,
        } => format!("events {events}  acc {}", commas(u64::from(accumulated))),
    }
}

fn total_sent(sticks: &[Panel]) -> u64 {
    sticks
        .iter()
        .flat_map(|s| s.sent.iter())
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
        let device = SimDevice::new(70_000, Profile::SpeedAndCadence, 25.0, 85.0, 200, 2.096);
        assert_eq!(key(&device), "70000:121");
    }

    /// One stick's worth of display data, owned by the test so panels can
    /// borrow it.
    struct Bench {
        label: String,
        port: String,
        devices: Vec<SimDevice>,
        sent: Vec<AtomicU64>,
    }

    impl Bench {
        fn panel(&self) -> Panel<'_> {
            Panel {
                label: &self.label,
                port: &self.port,
                devices: &self.devices,
                sent: &self.sent,
            }
        }
    }

    fn panels(benches: &[Bench]) -> Vec<Panel<'_>> {
        benches.iter().map(Bench::panel).collect()
    }

    /// Sticks shaped directly, rather than through `shard`, so a test can ask
    /// for a layout that sharding would never produce.
    fn benches(dongles: usize, per_dongle: usize) -> Vec<Bench> {
        let fleet = FleetSpec {
            devices: dongles * per_dongle,
            start_id: 65_533,
            profiles: SPEED_CADENCE_ONLY,
            speed_kph: 25.0,
            cadence_rpm: 85.0,
            power_watts: 200.0,
            spread_kph: 10.0,
            cadence_spread_rpm: 20.0,
            power_spread_w: 0.0,
            wheel_circumference_m: 2.096,
        }
        .build()
        .unwrap();

        fleet
            .chunks(per_dongle)
            .enumerate()
            .map(|(i, devices)| Bench {
                port: format!("20-1.{}", i + 1),
                label: format!("20-1.{} 0fcf:1009 serial 15508033{i}", i + 1),
                devices: devices.to_vec(),
                sent: (0..devices.len())
                    .map(|ch| AtomicU64::new((1000 + i * per_dongle + ch) as u64))
                    .collect(),
            })
            .collect()
    }

    /// The display is the whole interface while it runs, so its frame is
    /// checked for the pieces that make it readable rather than left to chance.
    #[test]
    fn a_frame_carries_the_totals_the_roster_and_a_row_per_device() {
        let benches = benches(2, 3);
        let frame = frame(&panels(&benches), Duration::from_secs(83));

        assert!(
            frame.contains("6 bike(s)  6 channel(s)  2 dongle(s)"),
            "{frame}"
        );
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
        let benches = benches(6, 8);
        let frame = frame(&panels(&benches), Duration::from_secs(10));
        assert!(frame.contains("48 bike(s)  48 channel(s)"), "{frame}");
        assert!(frame.contains("... and 16 more channel(s)"), "{frame}");
        assert_eq!(
            frame.lines().filter(|l| l.contains(":121")).count(),
            MAX_ROWS
        );
    }

    /// With the power meter on, a bike is two rows sharing one device number
    /// and differing only by type — which is the arrangement that makes
    /// `DeviceKey`'s type field matter.
    #[test]
    fn a_bike_with_power_shows_as_two_channels_under_one_device_number() {
        let fleet = FleetSpec {
            devices: 2,
            start_id: 5000,
            profiles: SPEED_CADENCE_AND_POWER,
            speed_kph: 25.0,
            cadence_rpm: 85.0,
            power_watts: 200.0,
            spread_kph: 0.0,
            cadence_spread_rpm: 0.0,
            power_spread_w: 0.0,
            wheel_circumference_m: 2.096,
        }
        .build()
        .unwrap();

        // Two bikes, four channels, and each bike's pair adjacent so a chunk of
        // eight never splits one across two sticks.
        assert_eq!(fleet.len(), 4);
        assert_eq!(
            fleet.iter().map(key).collect::<Vec<_>>(),
            ["5000:121", "5000:11", "5001:121", "5001:11"]
        );

        let bench = Bench {
            port: "1-1.2".to_owned(),
            label: "1-1.2 0fcf:1009 serial 134".to_owned(),
            sent: (0..4).map(|_| AtomicU64::new(7)).collect(),
            devices: fleet,
        };
        let frame = frame(&[bench.panel()], Duration::from_secs(60));
        assert!(frame.contains("2 bike(s)  4 channel(s)"), "{frame}");
        // Each profile is shown in the terms it actually transmits.
        assert!(frame.contains("25.0kph 85rpm"), "{frame}");
        assert!(frame.contains("200W 85rpm"), "{frame}");
        assert!(frame.contains("s.rev "), "{frame}");
        assert!(frame.contains("events "), "{frame}");
        // 2 channels at 4.053 Hz plus 2 at 4.005 Hz.
        assert!(frame.contains("expected 16.1 pkt/s"), "{frame}");
    }

    #[test]
    fn max_fills_every_dongle_and_an_explicit_count_still_wins() {
        assert_eq!(fleet_size(true, None, 2, 1), 16);
        assert_eq!(fleet_size(false, Some(3), 2, 1), 3);
        // No flag and no count is one dongle's worth, whatever is plugged in,
        // so the default leaves the other sticks free to receive on.
        assert_eq!(fleet_size(false, None, 2, 1), 8);
        assert_eq!(fleet_size(false, None, 1, 1), 8);
        // `--max` with a single stick selected fills that stick alone.
        assert_eq!(fleet_size(true, None, 1, 1), 8);

        // With the power meter on, a bike costs two channels, so the same
        // hardware carries half as many and --max says so rather than asking
        // for a fleet that cannot fit.
        assert_eq!(fleet_size(true, None, 2, 2), 8);
        assert_eq!(fleet_size(true, None, 1, 2), 4);
        assert_eq!(fleet_size(false, None, 2, 2), 4);
    }

    #[test]
    fn the_rate_is_zero_before_any_time_has_passed_rather_than_infinite() {
        assert_eq!(rate(10, Duration::ZERO), 0.0);
        assert!((rate(100, Duration::from_secs(4)) - 25.0).abs() < f64::EPSILON);
    }
}
