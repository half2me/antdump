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
cargo run --bin antsim -- -n 24 --start-id 5000 --spread 10   # 24 bikes over 3 dongles
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
  garbled pairs some firmware produces when two transmissions overlap on the air
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
  terminal
- **`SimDevice` / `FleetSpec`** (`src/sim.rs`) — A fleet of virtual bikes and the master
  channels that put them on the air. Mirrors `init.rs` deliberately: same network key,
  same RF frequency, same confirm-every-step discipline, because a transmitter that
  disagrees with the receiver on any of those is not wrong, it is silent
- **`Revolutions`** (`src/profile.rs`) — The counter model and the ANT+ page layouts
- **`probe_channel`** (`src/init.rs`) — A channel status request for a caller that has heard
  nothing for a while: silence on an open channel is also what an empty room sounds like, so
  this is how a stick that went deaf mid-run is told apart from one with nothing to hear

### Extended RX data: the ORDER of the two enable messages matters

`configure` sends `EnableExtRxMessages` and THEN `LibConfig`, and that order is load-bearing.
`EnableExtRxMessages` is the legacy switch and turns on the channel id block only; `LibConfig`
supersedes it and is the only way to get RX timestamps. Legacy first means a clone
dongle that quietly ignores LibConfig still reports channel ids, while real firmware ends up
with both blocks because LibConfig lands last. Failure is logged, never fatal. **RSSI is not
requested at all** (`LibConfig::new(true, false, true)`): the register never moved on any
stick we own (below), so the block would only cost bytes. `serialize_broadcast` still writes
one back if a dongle sends it anyway.

**Whether the dongle ACCEPTED anything is a separate message.** `send_message` returns once
the bytes are on the bulk endpoint, so its `Ok` means "written", not "accepted". `src/init.rs`
therefore confirms every message the channel depends on, and a reset is the probe: it always
answers with a startup notification, so silence there means the dongle is not listening and
nothing after it will change that. A refused `LibConfig` is the one degradation rather than a
failure, since the legacy switch already carries channel ids.

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

**Bench results (two genuine Dynastream sticks, 0fcf:1009 and 0fcf:1008, 307 frames, captured
while LibConfig still requested RSSI; today's frames carry two blocks, `flag=A0`):** the
order lands — `flag=E0`, all three blocks, on every frame. Both answered LibConfig with
`ResponseNoError` in 1.8 ms and 10.4 ms, so the 500 ms window is generous. Both report **AGC**
RSSI (`0x10`, 4 bytes), so the dBm branch is still unexercised, and no clone was available, so
the fallback path stays unobserved.

**There is no usable RSSI on this hardware.** Every stick tried reports AGC, and the register
does not move: walking a sensor to the edge of range and out of it left it byte-identical, and
then the packets stopped. So signal strength is not a thing this tool can report, and anything
that wants a proximity or link-quality signal has to count packets over time instead. RX timestamps are a real clock: per-device medians landed
on the ANT+ channel periods (8086 and 8182 ticks) to two decimals, every gap an integer
multiple. Zero checksum or length mismatches across 307 frames, which is what confirms
`serialize_broadcast` writes back every block the flag byte announces.

Two consequences worth knowing:

- **The RSSI block is variable width and shifts the timestamp behind it** — dBm (`0x20`) is 3
  bytes, AGC (`0x10`) is 4. `ant-rs` unpacks this correctly; `serialize_broadcast` has to
  write it back the same way, because the flag byte and the header's length are copied from
  the original and a block that is announced but missing leaves the reader parsing the
  checksum as payload.
- **RX timestamps are not a diagnostic luxury.** `CollisionDetector` times by them when they
  are present, because wall-clock gaps collapse toward zero whenever the host batches several
  USB reads after a stall, which false-collides perfectly good messages. Without them it falls
  back to wall-clock timing, which is what a clone gets.

### The simulator: eight per stick, and why one stick cannot test collisions

**Eight devices per dongle is the radio's number, not a setting.** Both stick types this
crate has seen (0fcf:1008 and 0fcf:1009) are nRF24AP2-USB parts, which are eight-channel
ANT network processors. A bigger fleet means more sticks; `antsim --list-dongles` prints
what is on the bus and multiplies it out, and `--max` fills it without being told a count.
The default is deliberately one dongle's worth rather than `--max`: a machine testing this
needs a stick left over for `antdump` to listen on, so filling everything is opt-in. The air is nowhere near the constraint — 24
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
`EVENT_TX` when it has done so; `sim::run` answers each one with the next payload. There is
no timer and no sleeping in the transmit loop — `UsbDriver::get_message` already blocks up
to 1 ms on its bulk read, so it self-throttles. It also means the displayed packet counts
are transmissions that actually happened rather than payloads handed over.

**Untested on hardware.** The simulator was written and unit-tested against a fake driver;
no ANT+ dongle was available to the environment it was built in, so nothing below the USB
boundary has been exercised on air.

## Key Dependencies

- `ant` — ANT+ protocol library (git dep: `github.com/cujomalainey/ant-rs`)
- `rusb` — USB device access
- `packed_struct` — Binary serialization for ANT message packing
- `clap` — CLI argument parsing with derive
