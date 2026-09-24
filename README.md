## Antdump

Sniff ANT+ data from the air using a usb dongle and the `OpenRXScanMode` command.

### Usage
`antdump` will automatically use the first compatible device it finds; with several plugged in, `antdump --list-dongles` prints each one's port and USB serial and `--dongle <serial or port>` picks one. Any ANT+ data captured will be shown in raw hex format on the console.
Once this tool is running, you can run [Wireshark](https://www.wireshark.org) to [capture USB traffic](https://wiki.wireshark.org/CaptureSetup/USB).
Use [Wireshark ANT+ Dissector](https://github.com/half2me/wireshark-antplus-dissector) to analyze the data.
You can also replay captured ANT+ packets saved in a `.pcap` file using [antreplay](https://github.com/half2me/antreplay).

### antsim

`antsim` is the other half of the test loop: it transmits a fleet of simulated ANT+ bike
sensors, so `antdump` can be pointed at traffic whose every counter is known in advance.
SimulANT+ does this on Windows only; this runs wherever `antdump` does.

```
antsim --list-dongles                          # sticks on the bus, and the fleet ceilings
antsim                                         # one dongle's worth of bikes
antsim --max                                   # fill every dongle found
antsim --max --no-power                        # twice as many bikes, speed&cadence only
antsim --max --max-spread                      # fill them and fan across the whole range
antsim --max --dongle 168 --start-id 65532     # fill one stick, leave the rest to receive on
antsim --reset                                 # silence dongles left transmitting
```

**Every dongle is driven from one thread.** Threading them segfaulted on macOS — libusb is
not reliably safe under concurrent synchronous transfers on a shared context — and it bought
nothing: a round-robin poll has roughly a 15x margin over the rate the radios ask for
payloads at.

**Stopping it matters.** An ANT dongle is an autonomous radio: once a channel is open its
firmware transmits on its own schedule and only asks the host for the next payload, so a
process that dies without closing the channel leaves the stick broadcasting its last
payload — frozen counters at full rate — until it is reset or unplugged. `antsim` closes
its channels on Ctrl-C. If something killed it harder than that, `antsim --reset` silences
the sticks, and so does simply starting another run.

Each bike transmits as a **combined speed and cadence sensor** (device type 121, 4.05 Hz)
and a **power meter** (device type 11, 4.00 Hz). Both carry the bike's device number and
differ only by device type, which is how a real bike with a power meter appears — and it is
what makes `DeviceKey`'s type field earn its keep: keyed on the number alone, a bike's two
streams would arrive interleaved at ~4 Hz each and false-collide continuously.

Each dongle has **8 channels** — the radio's limit, not a setting. A bike needs one channel
per sensor, so a stick carries **4 bikes**, or **8 with `--no-power`**, and a bigger fleet
needs more sticks. `--max` fills whatever is plugged in rather than making you count, and
divides by whichever profile is in force; with `--dongle` it fills that one stick and leaves
the others free.

Device numbers count upwards from `--start-id`; a start above 65535 is legal and exercises
the receiver's 20-bit device number path.

While it runs, `antsim` draws a live table of every device — its key, dongle, channel,
speed, cadence, revolution counts and packets sent — with fleet totals and the rate the
radios should be managing. The key column is formatted the same way `antdump` prints it,
so the two outputs line up by eye and by `grep`.

Two notes on testing with it:

- **One dongle cannot produce a collision.** The ANT stack staggers the channels a single
  stick owns, so its devices never overlap on the air. Exercising `antdump`'s collision
  detection takes two dongles transmitting and a third to receive on.
- **`--spread` is worth using, and `--max-spread` more so.** Without a spread every bike
  rides identically and their counters advance in lockstep, so a receiver that attributed
  one bike's page to another would produce output indistinguishable from correct.
  `--max-spread` fans the fleet from 5 to 60 km/h, whose endpoints straddle the profile's
  own broadcast rate: the slow bikes repeat a counter for six broadcasts running while the
  fast ones advance it by two between broadcasts, so both regimes are on the air at once.
