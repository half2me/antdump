# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Antdump is a Rust CLI tool that sniffs ANT+ wireless data from the air using a USB dongle in `OpenRXScanMode`. Captured data is printed as raw hex to the console and can optionally be forwarded to a TCP server. The USB traffic can also be captured by Wireshark for analysis with an ANT+ dissector.

## Build & Run

```bash
cargo build --locked     # Debug build (CI uses --locked; see Cargo.lock below)
cargo build --release    # Release build
cargo test               # Unit + pipeline tests
cargo run                # Run (auto-detects first ANT+ USB dongle)
cargo run -- --server <host:port>                  # Forward data to TCP server
cargo run -- --server <host:port> --hello_msg <msg> # Send hello before streaming
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
- **RX timestamps are NOT what the collision detector times on**, and this file said the
  opposite until the breakdown below was added. `CollisionDetector` times on the host's
  arrival clock, deliberately and only: a garbled frame can carry a garbled stamp, so timing
  collisions by the dongle's own clock means timing them with something the fault itself
  corrupts (`src/collision.rs`, and `src/init.rs` does not request the block). The cost is
  real and is the reason the drops are now broken down: arrival gaps DO collapse toward zero
  when the host batches several USB reads after a stall, which false-collides good messages,
  and `CollisionStats`'s gap buckets are what tells that apart from a genuinely noisy room.

## Key Dependencies

- `ant` — ANT+ protocol library (git dep: `github.com/cujomalainey/ant-rs`)
- `rusb` — USB device access
- `packed_struct` — Binary serialization for ANT message packing
- `clap` — CLI argument parsing with derive
