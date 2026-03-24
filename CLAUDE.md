# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Antdump is a Rust CLI tool that sniffs ANT+ wireless data from the air using a USB dongle in `OpenRXScanMode`. Captured data is printed as raw hex to the console and can optionally be forwarded to a TCP server. The USB traffic can also be captured by Wireshark for analysis with an ANT+ dissector.

## Build & Run

```bash
cargo build              # Debug build
cargo build --release    # Release build
cargo run                # Run (auto-detects first ANT+ USB dongle)
cargo run -- --server <host:port>                  # Forward data to TCP server
cargo run -- --server <host:port> --hello_msg <msg> # Send hello before streaming
cargo fmt                # Format code
cargo clippy             # Lint
```

Requires `libusb` on the system (e.g. `brew install libusb` on macOS, `apk add libusb-dev` on Alpine).

Docker build: `docker build -t antdump .`

## Architecture

Single-file application (`src/main.rs`) with no tests. Key components:

- **CLI args** — `clap` derive-based `Args` struct for `--server` and `--hello_msg` options
- **USB driver** — Uses `ant-rs` crate (`ant::drivers::UsbDriver`) from git dependency to communicate with ANT+ USB dongles via `rusb`
- **ANT+ protocol setup** — Configures channel 0 as shared receive-only with the ANT+ public network key, RF frequency 57, extended RX messages enabled, then opens RX scan mode
- **Message loop** — Reads messages from the driver in a busy loop; prints `BroadcastData` payloads as hex, forwards raw serialized ANT messages to the optional TCP stream
- **`DurableTCPStream`** — Wrapper around `TcpStream` with automatic reconnection on write failure (spawns a background thread to re-establish the connection)
- **`to_slice`** — Serializes an `AntMessage` (header + payload + extended info + checksum) into bytes using `packed_struct`

## Key Dependencies

- `ant` — ANT+ protocol library (git dep: `github.com/cujomalainey/ant-rs`)
- `rusb` — USB device access
- `packed_struct` — Binary serialization for ANT message packing
- `clap` — CLI argument parsing with derive
