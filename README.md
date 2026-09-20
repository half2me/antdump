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
```

Each dongle carries **8 devices** — that is the radio's limit, not a setting, so a bigger
fleet needs more sticks. `--max` fills whatever is plugged in rather than making you count;
combined with `--dongle` it fills that one stick and leaves the others free. Devices are combined speed and cadence sensors (ANT+ device type
121) transmitting at the profile's 4.05 Hz, numbered upwards from `--start-id`; a start
above 65535 is legal and exercises the receiver's 20-bit device number path.

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
