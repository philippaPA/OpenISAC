# USRP B205mini transmit test

This standalone program transmits a low-amplitude tone for 10 seconds on TX
channel 0. Its defaults place the tone at **2.401 GHz**: a 1 MHz baseband tone
with a 2.4 GHz RF centre frequency.

## Safety

Connect the B205mini **TRX** port to a spectrum analyzer through suitable RF
attenuation. Start with the analyzer input attenuation enabled. Do not connect
the USRP directly to another receiver unless its maximum input level is known.

## Build

```bash
cd "USRP test"
cmake -S . -B build
cmake --build build -j
```

Confirm that UHD can see the radio:

```bash
uhd_find_devices --args="type=b200"
uhd_usrp_probe --args="type=b200"
```

## Run

```bash
./build/usrp_tx_test
```

For a specific device and a slightly higher gain:

```bash
./build/usrp_tx_test --args "type=b200,serial=YOUR_SERIAL" --gain 10
```

Useful options are shown by:

```bash
./build/usrp_tx_test --help
```

With the defaults, set the analyzer near 2.401 GHz with a span of roughly
5 MHz. Increase gain gradually only if necessary.
