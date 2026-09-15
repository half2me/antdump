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
cargo fmt                # Format code
cargo clippy             # Lint
```

**`Cargo.lock` is committed and `ant` is pinned by rev.** This crate ships as fleet
firmware, so a rebuild of a released tag must resolve the same dependency tree; an
unpinned git dependency silently changes what an old tag contains. CI builds with
`--locked` so a stale lock fails rather than quietly updating.

Requires `libusb` on the system (e.g. `brew install libusb` on macOS, `apk add libusb-dev` on Alpine).

Docker build: `docker build -t antdump .`

## Architecture

- **CLI args** (`src/main.rs`) — `clap` derive-based `Args` struct for `--server`,
  `--hello_msg`, `--collision_threshold_ms` and `--quiet`
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

### Extended RX data: the ORDER of the two enable messages matters

`init_driver` sends `EnableExtRxMessages` and THEN `LibConfig`, and that order is load-bearing.
`EnableExtRxMessages` is the legacy switch and turns on the channel id block only; `LibConfig`
supersedes it and is the only way to get RSSI and RX timestamps. Legacy first means a clone
dongle that quietly ignores LibConfig still reports channel ids, while real firmware ends up
with all three blocks because LibConfig lands last. Failure is logged, never fatal.

**Whether the dongle ACCEPTED it is a separate message.** `send_message` returns once the
bytes are on the bulk endpoint, so its `Ok` means "written", not "accepted" — the verdict
comes back as a `ChannelResponse` carrying `TxMessageId::LibConfig`, which `await_response`
drains for during init (the channel is not open yet, so nothing else is arriving). Without
that read a rejection is invisible: the blocks simply never appear and collision timing
silently falls back to the wall clock.

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

## Key Dependencies

- `ant` — ANT+ protocol library (git dep: `github.com/cujomalainey/ant-rs`)
- `rusb` — USB device access
- `packed_struct` — Binary serialization for ANT message packing
- `clap` — CLI argument parsing with derive
