# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Antdump is a Rust CLI tool that sniffs ANT+ wireless data from the air using a USB dongle in `OpenRXScanMode`. Captured data is printed as raw hex to the console and can optionally be forwarded to a TCP server. The USB traffic can also be captured by Wireshark for analysis with an ANT+ dissector.

The crate ships a second binary, **`antsim`**, which is the other half of the test loop:
it transmits a fleet of simulated ANT+ bike sensors so `antdump` can be checked against
traffic whose every counter is known in advance. Nothing off the shelf does this on Linux
or macOS — SimulANT+ is Windows-only, and the open-source ANT+ transmitters that do run
here (antifier, FortiusANT, openant's examples) each simulate exactly one device.

## Build & Run

```bash
cargo build --locked     # Debug build (CI uses --locked; see Cargo.lock below)
cargo build --release    # Release build
cargo test               # Unit + pipeline tests
cargo run                # Run (auto-detects first ANT+ USB dongle)
cargo run -- --server <host:port>                  # Forward data to TCP server
cargo run -- --server <host:port> --hello_msg <msg> # Send hello before streaming

cargo run --bin antsim -- --list-dongles           # Sticks on the bus and the fleet ceiling
cargo run --bin antsim                             # 8 simulated bikes on one dongle
cargo run --bin antsim -- --max                    # Fill every dongle found
cargo run --bin antsim -- --max --no-power         # Twice as many bikes, speed&cadence only
cargo run --bin antsim -- --max --max-spread       # Fill them and fan across the whole range
cargo run --bin antsim -- --reset                  # Silence dongles left transmitting
cargo fmt --check        # What CI checks
cargo clippy --all-targets --locked -- -D warnings   # What CI GATES on
```

**CI's clippy is a gate, not an annotation.** It used to run through
`giraffate/clippy-action`, which defaults to `fail_on_error: false` and to
`filter_mode: added` — so findings were annotations on changed lines that never failed
the job, and `--all-targets` was never passed, so test code went unlinted. The receiver
firmware that consumes this crate runs exactly the command above and fails on it, so
running anything weaker here just moved the discovery downstream.

**`Cargo.lock` is committed and `ant` is pinned by rev.** This crate ships as fleet
firmware, so a rebuild of a released tag must resolve the same dependency tree; an
unpinned git dependency silently changes what an old tag contains. CI builds with
`--locked` so a stale lock fails rather than quietly updating.

Requires `libusb` on the system (e.g. `brew install libusb` on macOS, `apk add libusb-dev` on Alpine).

Docker build: `docker build -t antdump .`

## Architecture

- **CLI args** (`src/main.rs`) — `clap` derive-based `Args` struct for `--server`,
  `--hello_msg`, `--collision_threshold_ms`, `--quiet`, `--dongle` and `--list-dongles`
- **USB driver** — Uses `ant-rs` crate (`ant::drivers::UsbDriver`) from a git dependency to
  communicate with ANT+ USB dongles via `rusb`
- **ANT+ protocol setup** — Channel 0 as shared receive-only with the ANT+ public network key,
  RF frequency 57, then extended RX data, then RX scan mode
- **Message loop** — Reads messages from the driver in a busy loop; prints `BroadcastData`
  payloads as hex, forwards raw serialized ANT messages to the optional TCP stream
- **`TcpWriter`** (`src/tcp.rs`) — `TcpStream` with automatic reconnection on write failure
  (spawns a background thread to re-establish the connection)
- **`serialize_broadcast`** (`src/message.rs`) — Rebuilds an `AntMessage`'s wire bytes
  (header + payload + every extended block the flag byte announces + checksum)
- **`CollisionDetector`** (`src/collision.rs`) — Per-device quarantine that drops the
  garbled pairs some firmware produces when two transmissions overlap on the air.
  `dropped_count` counts PACKETS and `stats()` counts EVENTS, split into pairs against
  burst continuations and bucketed by the gap that convicted them, because one total
  cannot say whether a box is in a noisy room or whether its own reader is stalling
- **`bring_up`** (`src/usb.rs`) — Finds, opens and configures a dongle, resetting and
  re-finding it between attempts. A selector (`DongleId`: the USB serial or the bus and
  port chain, `20-1.4`) names one stick when several share the bus, and the reset honors
  it too, since resetting the first stick found is how a second process on the same
  machine knocks the first one deaf; `None` takes the first. The serial is what the stick
  sends before the first NUL: both bench sticks claim a longer descriptor than they fill, so
  libusb returns the serial, a NUL and stale buffer bytes. In the library, not the binary, because the raceble
  receiver firmware (`racetogether/firmware`) runs the same loop forever and reports
  `NoDongle` and an init failure as two different states; `antdump` exits after three.
  The library never prints: each failed attempt goes to the caller's `report` callback,
  so the receiver logs it with a timestamp and `antdump` writes it to stderr
- **`antsim`** (`src/bin/antsim.rs`) — The simulator's CLI and its live status display:
  a redrawn table of every device with its key, dongle, channel, speed, cadence, live
  revolution counts and transmitted-packet count, plus fleet totals and the packet rate
  the radios *should* be managing, since a measured rate below it is the fleet losing
  transmissions. The key column is formatted `device_number:device_type_id`, which is
  exactly what `antdump` prints in front of every packet, so the two outputs line up by
  eye and by `grep`. Falls back to a periodic one-line summary when stdout is not a
  terminal. **A binary under `src/bin` is a separate crate from the library**, so
  `non_exhaustive` types like `DongleId` cannot be built with a struct literal there —
  which is why its display takes a `Panel` of plain strings rather than a `DongleId`, and
  why `capacity` and `fleet_size` take a dongle count. This only breaks on CI, which builds
  the PR merged with its base, so a local build on an older base will not show it
- **`SimDevice` / `FleetSpec`** (`src/sim.rs`) — A fleet of virtual bikes and the master
  channels that put them on the air. Mirrors `init.rs` deliberately: same network key,
  same RF frequency, same confirm-every-step discipline, because a transmitter that
  disagrees with the receiver on any of those is not wrong, it is silent
- **`Revolutions`** (`src/profile.rs`) — The counter model, plus the combined speed and
  cadence page (type 121, period 8086) and the standard power-only page (type 11, period
  8182). Fitness equipment (type 17, period 8192) is the next one in, and needs only its
  own page builder here and an arm in `Profile`
- **`probe_channel`** (`src/init.rs`) — A channel status request for a caller that has heard
  nothing for a while: silence on an open channel is also what an empty room sounds like, so
  this is how a stick that went deaf mid-run is told apart from one with nothing to hear

### Extended RX data: the channel id block, and nothing else

`configure` sends **`EnableExtRxMessages` and nothing else** (`src/init.rs`). It is the legacy
switch and turns on the channel id block, which is the only extended data anything here reads:
`DeviceKey` is built from it and the collision quarantine is keyed by it.

**`LibConfig` is deliberately not sent**, because both things it adds are unwanted. RSSI,
because every stick on the bench reports the AGC register rather than dBm and that register was
measured byte-identical from point-blank to out of range (below), so the block would cost bytes
a frame and tell nobody anything. And RX timestamps, because `CollisionDetector` times on the
HOST's arrival clock instead: a stamp riding inside a frame can be garbled by the very fault the
detector exists to catch, and the resolution it buys is not needed, a venue capture putting the
normal cadence 200x away from the collision window. That choice has a cost, and
`CollisionStats`'s gap buckets are what measures it: arrival gaps collapse toward zero when the
host batches several USB reads after a stall, which false-collides good messages, and a gap
several milliseconds wide is how one of those is told from a real collision.

So a frame today carries the channel id block alone. `serialize_broadcast` still writes back the
RSSI and timestamp blocks, because a dongle that sends one unasked has to be round-tripped
faithfully: the flag byte and the header's length are copied from the original, so a block that
is announced but not written leaves the reader parsing the checksum as payload.

**Whether the dongle ACCEPTED anything is a separate message.** `send_message` returns once
the bytes are on the bulk endpoint, so its `Ok` means "written", not "accepted". `src/init.rs`
therefore confirms every message the channel depends on, and a reset is the probe: it always
answers with a startup notification, so silence there means the dongle is not listening and
nothing after it will change that.

**A deaf dongle is the failure that matters, and a restart does not clear it.** Seen on a Pi:
restart antdump and the stick stays silent until it is physically unplugged. `UsbDriver::new`
resets the handle it then claims the interface on, and a reset that re-enumerates the device
invalidates that handle — libusb's own answer is to close it and rediscover, which nothing
did. So `usb::bring_up` retries: reset at the USB level, drop the handle, wait out
re-enumeration, look the device up again. `antdump` gives it three attempts, then exits
non-zero rather than sit there configured into the void.

**Passing the reset probe once proves nothing about later.** On a laptop, a stick pulled
mid-race and plugged back in answered its reset, accepted every configuration message and
then delivered nothing for as long as it was left; a process restart found it not answering
a reset at all. A long-running caller therefore cannot treat "dongle up" as settled:
`probe_channel` asks the stick for channel 0's status whenever the air has been quiet for a
few seconds, and a missing answer or a channel that is not open means bring it up again.

**Bench results, HISTORICAL (two genuine Dynastream sticks, 0fcf:1009 and 0fcf:1008, 307
frames).** They were captured back when `configure` still sent `LibConfig` and still asked for
RSSI, so those frames carry all three blocks (`flag=E0`) where today's carry the channel id
alone. They are kept because they are the evidence behind two decisions that still stand: both
sticks answered LibConfig with `ResponseNoError` in 1.8 ms and 10.4 ms, and both report **AGC**
RSSI (`0x10`, 4 bytes), so the dBm branch is unexercised and, no clone having been available,
the fallback path stays unobserved.

**There is no usable RSSI on this hardware.** Every stick tried reports AGC, and the register
does not move: walking a sensor to the edge of range and out of it left it byte-identical, and
then the packets stopped. So signal strength is not a thing this tool can report, and anything
that wants a proximity or link-quality signal has to count packets over time instead. RX
timestamps WERE a real clock while they were requested: per-device medians landed on the ANT+
channel periods (8086 and 8182 ticks) to two decimals, every gap an integer multiple. That is
what makes not using them a choice rather than a workaround, and the reasoning is above. Zero
checksum or length mismatches across those 307 frames, which is what confirms
`serialize_broadcast` writes back every block the flag byte announces.

One consequence worth knowing, about a dongle that sends a block unasked:

- **The RSSI block is variable width and shifts the timestamp behind it** — dBm (`0x20`) is 3
  bytes, AGC (`0x10`) is 4. `ant-rs` unpacks this correctly; `serialize_broadcast` has to
  write it back the same way, because the flag byte and the header's length are copied from
  the original and a block that is announced but missing leaves the reader parsing the
  checksum as payload.

### The simulator: eight per stick, and why one stick cannot test collisions

**Eight CHANNELS per dongle is the radio's number, not a setting.** Both stick types this
crate has seen (0fcf:1008 and 0fcf:1009) are nRF24AP2-USB parts, which are eight-channel
ANT network processors. A bike needs one channel per sensor it carries, so the eight buy
four bikes with speed&cadence and power, or eight with `--no-power`. `antsim --list-dongles`
prints both ceilings and `--max` fills whichever applies without being told a count. The
default is deliberately one dongle's worth rather than `--max`: a machine testing this needs
a stick left over for `antdump` to listen on, so filling everything is opt-in.

**A bike's sensors share its device number and differ only by device type.** That is how a
real bike with a power meter appears, and it is what makes `DeviceKey`'s type field earn its
keep: keyed on the number alone, a bike's CSC and power streams would arrive interleaved at
~4 Hz each and `CollisionDetector` would false-collide them continuously. `FleetSpec::build`
emits a bike's profiles adjacently so a chunk of eight keeps whole bikes on one stick.

**Power is an accumulator, not a reading, and it advances on crank revolutions.** The page
carries a running sum of instantaneous watts plus the update event count, and a receiver
divides the two differences to get average power — so they have to move together or not at
all. Both are driven off the same `Revolutions` as cadence, which means a rider at 85 rpm
produces 1.4 events a second against a 4 Hz broadcast: roughly two broadcasts in three
repeat the previous pair exactly. A receiver that treated each broadcast as a new event
would compute average power a third too low, which is precisely the bug this catches. The air is nowhere near the constraint — 24
devices at ~4 Hz is ~97 packets a second and an ANT+ packet is ~150 µs on the air, under
2% duty cycle — so channel count is the only thing in the way.

**A single stick cannot produce a collision, by design.** The ANT stack time-division
schedules the channels it owns, so eight masters on one dongle are staggered deliberately
and never overlap. That makes one stick the right tool for checking that counters parse
and the wrong tool entirely for exercising `CollisionDetector`: for that the transmissions
have to come from radios that do not know about each other, which means two dongles
transmitting and a third receiving.

**The counters repeat on purpose, because real ones do.** ANT+ profiles do not transmit
speed; they transmit a cumulative revolution count and the time of the revolution that
bumped it, and the receiver divides. A real sensor's counter only moves when a magnet
passes, which is not in step with its 4 Hz broadcast, so below ~4 rev/s the same count and
the same event time go out several broadcasts running. `Revolutions::at` reports the state
as of the last revolution to have actually happened, so the repeats fall out of the
arithmetic — a simulator that incremented per broadcast would never produce them, and a
parser that mishandled them would pass the test. Both counters roll at 16 bits, which puts
the event time's wrap at exactly 64 seconds: a run of any length crosses it constantly.

**Device numbers are 20 bits and the top four are not in the channel id.** `SimDevice::channel_id`
puts the low 16 bits in the channel id and the top 4 in the transmission type's extension
nibble, which is exactly how `DeviceKey::from_broadcast` puts them back together. So
`--start-id 70000` exercises a branch of the receiver that a fleet numbered below 65536
never touches. Device number 0 is ANT's wildcard and is refused rather than transmitted.

**The radio sets the pace.** An open master channel transmits on its own period and raises
`EVENT_TX` when it has done so; `sim::pump` answers each one with the next payload. There is
no timer and no sleeping in the transmit loop — `UsbDriver::get_message` already blocks up
to 1 ms on its bulk read, so it self-throttles. It also means the displayed packet counts
are transmissions that actually happened rather than payloads handed over.

**One thread drives every dongle, and that is not a simplification.** Giving each stick its
own thread segfaulted on macOS within a second of the channels opening. The Rust side is
sound — `rusb` marks `DeviceHandle` `Send`, and each thread owned its own driver — but two
threads doing concurrent synchronous bulk transfers on the shared `GlobalContext` is not a
path libusb is reliably safe on. `sim::pump` therefore serves at most one message and
returns, and `antsim` takes turns across its sticks from one thread. The margin is
comfortable rather than tight: `get_message` blocks at most 1 ms, so a cycle over N sticks
costs about N ms, while a dongle with all eight channels open raises an `EVENT_TX` roughly
every 31 ms — and a missed event costs nothing, since the radio repeats the payload and
raises it again a period later. `sim::run` is the single-dongle loop over `pump`, kept for
a caller that only has one.

**An open master channel outlives the process that opened it, and this is the surprise
that matters.** The dongle is an autonomous radio: once `OpenChannel` succeeds its firmware
transmits at the channel period on its own and only asks the host for the *next* payload.
Kill the process and the channel stays open — the stick keeps broadcasting the last payload
it was handed, at full rate, until something resets it or it is unplugged. Observed
directly: `antsim` killed, packets still on the air. `ant-rs` has no `Drop` that tears a
channel down, so nothing does it implicitly. `antsim` therefore catches SIGINT and SIGTERM,
and `shut_down_all` resets every stick before the process exits — on the way out of a
normal run, and on the failure path too, where a stick that came up before a later one
failed would otherwise be left broadcasting. `antsim --reset` is the remedy for a stick
left transmitting by something that died without doing this, and `configure_master`'s
opening reset is why simply starting a new run also clears it.

**Untested on hardware.** The simulator was written and unit-tested against a fake driver;
no ANT+ dongle was available to the environment it was built in, so nothing below the USB
boundary has been exercised on air.

## Key Dependencies

- `ant` — ANT+ protocol library (git dep: `github.com/cujomalainey/ant-rs`)
- `rusb` — USB device access
- `packed_struct` — Binary serialization for ANT message packing
- `clap` — CLI argument parsing with derive
