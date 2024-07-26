## Antdump

Sniff ANT+ data from the air using a usb dongle and the `OpenRXScanMode` command.

### Usage
`antdump` will automatically use the first compatible device it finds. Any ANT+ data captured will be shown in raw hex format on the console.
Once this tool is running, you can run [Wireshark](https://www.wireshark.org) to [capture USB traffic](https://wiki.wireshark.org/CaptureSetup/USB).
Use [Wireshark ANT+ Dissector](https://github.com/half2me/wireshark-antplus-dissector) to analyze the data.
You can also replay captured ANT+ packets saved in a `.pcap` file using [antreplay](https://github.com/half2me/antreplay).
