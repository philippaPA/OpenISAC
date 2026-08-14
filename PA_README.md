OpenISAC allows a demonstration of using a communication wave as sensing. It is part of a larger internship project on drone detection conducted by Henry Teall and Philippa Flintoff in the summer of 2026.

During this experimentation a short range isac sensor was able to receive micro doppler signals detecting a dji neo drone. 

The equipment used:

Drone: DJI NEO. (Risk assessment only covered the use of a drone with covered propellers. 

Antennas: pcb logarithmic antennas. (tested omnidirectional although nothing detected. PA has horn antennas in the optics lab which could produce a better result.)

SDR: ettus B205mini 

laptop: ubuntu laptop (not pa locked). Could also try running on a windows laptop with wsl. I tried briefly but gave up quick.

## Requirements

Only Ubuntu 24.04 is supported/tested — the Ettus UHD guide's dependency instructions only go up to 24.04, and that's what OpenISAC itself has been tested against. Check your version with `lsb_release -a` (or `cat /etc/os-release`); if you're on anything else (including newer non-LTS releases like 26.04), you need Ubuntu 24.04 instead:

- **On Windows:** install it via WSL:
  ```bash
  wsl --list --online
  wsl --install -d Ubuntu-24.04
  ```
  Then do all of the setup below inside that Ubuntu 24.04 WSL instance.
- **On a native Ubuntu laptop running a different version:** you'll need to reinstall/dual-boot Ubuntu 24.04 itself (e.g. from the [official Ubuntu 24.04 image](https://releases.ubuntu.com/24.04/)) — WSL doesn't apply here, it's only for running Ubuntu inside Windows.

### 1. Clone OpenISAC

```bash
cd ~
git clone https://github.com/philippaPA/OpenISAC.git
```

### 2. UHD (USRP Hardware Driver)

This step is quite confusing.

Install the UHD toolchain by following the official Ettus guide (Ubuntu 24.04 tutorial):
[Building and Installing the USRP Open-Source Toolchain on Linux](https://kb.ettus.com/Building_and_Installing_the_USRP_Open-Source_Toolchain_(UHD_and_GNU_Radio)_on_Linux#Update_and_Install_dependencies)

Tested on UHD v4.9.0.1 (`git checkout v4.9.0.1`).

### 3. Aff3ct (Forward Error Correction library)

```bash
sudo apt-get install nlohmann-json3-dev
git clone https://github.com/aff3ct/aff3ct.git
cd aff3ct
git submodule update --init --recursive
mkdir build
cd build
cmake .. -G"Unix Makefiles" -DCMAKE_CXX_COMPILER="g++" -DCMAKE_BUILD_TYPE="Release" -DCMAKE_CXX_FLAGS="-funroll-loops -march=native" -DAFF3CT_COMPILE_EXE="OFF" -DAFF3CT_COMPILE_SHARED_LIB="ON" -DSPU_STACKTRACE="OFF" -DSPU_STACKTRACE_SEGFAULT="OFF" -DCMAKE_CXX_FLAGS="${CMAKE_CXX_FLAGS} -faligned-new"
make -j$(nproc)
sudo make install
```

### 4. Build OpenISAC

```bash
sudo apt-get install libyaml-cpp-dev libzmq3-dev cppzmq-dev
cd ~/OpenISAC
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

This produces `build/BS` and `build/UE`.

### 5. Frontend (Python)

Python 3.13, dependencies listed in `requirements.txt` at the repo root (numpy, matplotlib, scipy, PyQt6, pyqtgraph, PyYAML, pyzmq, etc).

## Setting up and running the venv

```bash
cd ~/OpenISAC
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

`source .venv/bin/activate` needs to be re-run in every new terminal you use for frontend scripts (e.g. `plot_sensing_fast.py`, `calibrate_hsys.py`); you'll see `(.venv)` in your prompt once it's active. Run `deactivate` to leave it.

## Config

Before running anything, choose the B205mini config, edit it with `nano` for your settings, then copy it into `build/` as `BS.yaml`:

```bash
nano config/BS_B205.yaml
cd build
cp ../config/BS_B205.yaml BS.yaml
```

The fields we edited for the drone detection run: `downlink.tx_gain`, the per-channel `rx_gain` under `sensing.rx_channels`, and `downlink.center_freq` (see [Parameters used](#parameters-used) below for our values).

Do this first — both calibration and the sensor run below start `BS` from `build/`, and it reads `BS.yaml` from that directory.

## CPU performance (core prioritisation, optional)

Not required to get the sensor running, but the script is in the repo if you hit real-time performance issues (dropped samples, jitter). Run it before starting `BS`, whether for calibration or a real run:

```bash
./scripts/set_performance.bash
```

This tunes the host for real-time processing (socket buffers, CPU governor, NIC ring/MTU, and pinning NIC IRQs off onto their own cores so they don't fight the app for CPU time). If you have isolated CPU cores set up (`/sys/devices/system/cpu/isolated`) it'll use those automatically for IRQ pinning; otherwise set `OPENISAC_IRQ_CORE_LIST` yourself, e.g.:

```bash
OPENISAC_IRQ_CORE_LIST=14-15 ./scripts/set_performance.bash
```

Keep the app's own real-time threads off whatever cores you dedicate to IRQs.

## Calibration

Do this before running the sensor for real. Two separate calibrations, both done with a direct RF cable between TX and RX instead of the normal antennas:

**IMPORTANT — keep gains low while directly connected.** This applies to both calibrations below. A direct cable has almost none of the path loss your normal antennas give you, so your normal operating gains will saturate/clip the RX front end. We used RX gain 30 and TX gain 10 for the direct-connection calibration.

**Hsys (system response) calibration** — needed whenever you change frequency. This characterizes the SDR's own TX/RX filter+DAC/ADC response. Follow the instructions below to run it — you need BS and the fast sensing plot both running, then click `Calibrate Hsys` in the viewer (or run `scripts/calibrate_hsys.py` directly). It drops TX/RX gain for the loopback capture and restores your normal gains afterwards.

**Tip** The MTI (Moving Target Indication) filter suppresses static content, but the loopback signal during Hsys calibration is static — so with MTI on you won't see it come through. Turn off the `MTI` toggle in the sensing viewer whilst calibrating.

**Timing / system-delay calibration** — only needed when the hardware changes (new cable lengths, new RF path, new antennas etc), not when you just change frequency. Set `enable_system_delay_estimation: true` on the sensing channel in `BS.yaml` (before starting BS below), connect the direct RF cable, and watch the BS console for `alignment_suggest=<value>` (`suggest=<value>` on CUDA builds). This comes through fast, within a few seconds of starting — no need to wait around. Once you've got a stable value, write it into the channel's `alignment` field, set `enable_system_delay_estimation` back to `false`, then restore the normal antenna connection.

Run system-delay calibration before Hsys — Hsys assumes the RX frame is already aligned to the direct-path timing reference.

**Terminal 1 — start BS:**
```bash
cd build
sudo ../scripts/isolate_cpus.py
sudo ../scripts/isolate_cpus.py run ./BS
```

**Terminal 2 — run the calibration you need:**

For Hsys, run the fast sensing plot and click `Calibrate Hsys` in the viewer:
```bash
python3 ./scripts/plot_sensing_fast.py
```
(or run `python3 ./scripts/calibrate_hsys.py` directly instead of using the viewer).

For timing/system-delay calibration, you don't need a second terminal — just watch Terminal 1's console for `alignment_suggest`.

## Running the sensor

**Terminal 1 — BS backend:**
```bash
cd build
sudo ../scripts/isolate_cpus.py
sudo ../scripts/isolate_cpus.py run ./BS
```

**Terminal 2 — fast sensing plot:**
```bash
python3 ./scripts/plot_sensing_fast.py
```

## Troubleshooting: USRP not found / connection issues

If `BS` can't find or initialize the B205mini:

1. Check it shows up over USB at all:
   ```bash
   lsusb
   ```
   Look for an Ettus Research / National Instruments entry.

2. Confirm UHD itself can see the device:
   ```bash
   uhd_find_devices
   ```

## Parameters used

For the drone detection run above: `symbol_stride` (STRD) 50, TX gain 70, RX gain 42, centre frequency 5.8 GHz. These worked for our setup but are worth playing with for yours — stride trades off Doppler resolution/Nyquist range, and gains depend on your antennas and range to target.
