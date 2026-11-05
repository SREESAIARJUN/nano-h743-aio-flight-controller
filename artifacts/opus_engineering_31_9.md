# README.md: nano-h743-aio-flight-controller

I don't have your schematic, `hwdef.dat`, or test data, and I don't know what "Cycle 31" refers to in your process. This README is therefore built so that:

- **Verifiable facts are filled in.** This covers the ArduPilot build system, the DFU flashing workflow, and STM32H743 silicon characteristics.
- **Board-specific values are marked `⚠️ VERIFY`.** This covers pin assignments, current ratings, and thermal measurements. Replace them with numbers from your schematic and test results before publishing.

Publishing invented pinouts or current ratings in a flight controller README could get hardware damaged or people hurt, so I've avoided guessing.

---

````markdown
# Nano H743 AIO Flight Controller

> All-in-one STM32H743 flight controller with integrated 4-in-1 ESC, designed for ArduPilot.

![Board Top](docs/img/board_top.png)
![Board Bottom](docs/img/board_bottom.png)

[![Firmware](https://img.shields.io/badge/firmware-ArduPilot-blue)](https://ardupilot.org)
[![License](https://img.shields.io/badge/hardware-CERN--OHL--S--2.0-green)](LICENSE-HARDWARE)
[![License](https://img.shields.io/badge/software-GPLv3-green)](LICENSE)
<!-- Release / revision badge -->
![Revision](https://img.shields.io/badge/hw_rev-Cycle_31-orange)

---

## Table of Contents

1. [Overview](#overview)
2. [Specifications](#specifications)
3. [Pinout](#pinout)
4. [UART Mapping](#uart-mapping)
5. [PWM / Motor Outputs](#pwm--motor-outputs)
6. [Analog & Power Monitoring](#analog--power-monitoring)
7. [Building ArduPilot](#building-ardupilot)
8. [Flashing](#flashing)
9. [Initial Configuration](#initial-configuration)
10. [Thermal Specifications](#thermal-specifications)
11. [Mechanical](#mechanical)
12. [Troubleshooting](#troubleshooting)
13. [Revision History](#revision-history)
14. [License](#license)

---

## Overview

The Nano H743 AIO combines an STM32H743 flight controller and a 4-in-1 BLHeli/AM32 ESC on a single
board for lightweight multirotors. It targets ArduPilot (Copter, Plane, Rover) and supports
bidirectional DShot, OSD, and MAVLink telemetry.

**Intended use:** sub-250 g to mid-size whoops and toothpicks, 2S–4S. ⚠️ VERIFY

---

## Specifications

| Parameter            | Value                                                         |
|----------------------|---------------------------------------------------------------|
| MCU                  | STM32H743VIT6, Cortex-M7 @ 480 MHz, 2 MB flash, 1 MB RAM      |
| IMU                  | ⚠️ VERIFY (e.g. ICM-42688-P, SPI)                             |
| Barometer            | ⚠️ VERIFY (e.g. DPS310 / BMP280, I2C)                         |
| OSD                  | ⚠️ VERIFY (AT7456E analog / MSP DisplayPort)                  |
| Blackbox / Logging   | ⚠️ VERIFY (microSD / onboard flash, size)                     |
| ESC                  | 4-in-1, ⚠️ VERIFY A continuous / A burst per channel          |
| ESC Firmware         | ⚠️ VERIFY (BLHeli_S / Bluejay / AM32)                         |
| Input Voltage        | ⚠️ VERIFY (e.g. 2S–4S LiPo, 7.4–16.8 V)                        |
| BEC                  | 5 V @ ⚠️ VERIFY A, 3.3 V @ ⚠️ VERIFY mA                        |
| UARTs                | ⚠️ VERIFY count                                               |
| Current Sensor       | ⚠️ VERIFY (scale in A/V)                                      |
| Mounting             | ⚠️ VERIFY (e.g. 25.5 × 25.5 mm, M2)                           |
| Dimensions / Weight  | ⚠️ VERIFY mm / g                                              |
| USB                  | USB-C, ⚠️ VERIFY                                              |

---

## Pinout

> **Source of truth:** `ardupilot/libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/hwdef.dat`.
> If this table and `hwdef.dat` disagree, `hwdef.dat` wins. Please open an issue.

![Pinout Diagram](docs/img/pinout.png)

### Top-side pads

| Pad Label | Function            | MCU Pin       | Notes                      |
|-----------|---------------------|---------------|----------------------------|
| VBAT      | Battery input       | —             | Direct battery, unregulated|
| GND       | Ground              | —             |                            |
| 5V        | BEC output          | —             | ⚠️ VERIFY max current       |
| 3V3       | 3.3 V rail          | —             | Low-current only           |
| T1 / R1   | UART1 TX / RX       | ⚠️ VERIFY      | USB-adjacent               |
| T2 / R2   | UART2 TX / RX       | ⚠️ VERIFY      |                            |
| SBUS      | RC input (inverted) | ⚠️ VERIFY      |                            |
| LED       | WS2812 strip        | ⚠️ VERIFY      | PWM-capable timer          |
| BZ+ / BZ- | Buzzer              | ⚠️ VERIFY      | Active buzzer, 5 V         |
| CAM       | Camera video in     | —             | Analog OSD only            |
| VTX       | Video out to VTX    | —             | Analog OSD only            |
| SCL / SDA | I2C (compass)       | ⚠️ VERIFY      | External mag               |

### Motor pads

| Pad | Output    | Timer / Channel | DShot | Bidir DShot |
|-----|-----------|-----------------|-------|-------------|
| M1  | SERVO1    | ⚠️ VERIFY        | ✅    | ✅          |
| M2  | SERVO2    | ⚠️ VERIFY        | ✅    | ✅          |
| M3  | SERVO3    | ⚠️ VERIFY        | ✅    | ✅          |
| M4  | SERVO4    | ⚠️ VERIFY        | ✅    | ✅          |

---

## UART Mapping

ArduPilot `SERIALn` ports map to physical UARTs via the `SERIAL_ORDER` line in `hwdef.dat`.

| ArduPilot Port | Physical UART | Default Protocol            | Default Baud |
|----------------|---------------|-----------------------------|--------------|
| SERIAL0        | USB (OTG1)    | MAVLink2                    | 115200       |
| SERIAL1        | ⚠️ VERIFY      | MAVLink2 (telemetry)        | 57600        |
| SERIAL2        | ⚠️ VERIFY      | ⚠️ VERIFY                    | ⚠️ VERIFY     |
| SERIAL3        | ⚠️ VERIFY      | GPS                         | 115200       |
| SERIAL4        | ⚠️ VERIFY      | ⚠️ VERIFY                    | ⚠️ VERIFY     |
| SERIAL5        | ⚠️ VERIFY      | ESC Telemetry               | 115200       |
| SERIAL6        | ⚠️ VERIFY      | RCIN                        | —            |

---

## PWM / Motor Outputs

- Outputs within the same timer group must share a protocol and rate.
- Bidirectional DShot requires the output timer channels to have DMA (`BIDIR` flag in `hwdef.dat`).

| Group | Outputs   | Timer      |
|-------|-----------|------------|
| 1     | 1–4       | ⚠️ VERIFY   |
| 2     | 5 (LED)   | ⚠️ VERIFY   |

Enable bidirectional DShot and RPM filtering:

```
SERVO_BLH_BDMASK = 15        # outputs 1-4
SERVO_BLH_POLES  = 14        # motor magnet pole count
MOT_PWM_TYPE     = 6         # DShot600 (5 = DShot300)
INS_HNTCH_ENABLE = 1
INS_HNTCH_MODE   = 3         # ESC telemetry RPM
```

---

## Analog & Power Monitoring

| Parameter          | Value          |
|--------------------|----------------|
| `BATT_MONITOR`     | 4 (Analog V+I) |
| `BATT_VOLT_PIN`    | ⚠️ VERIFY       |
| `BATT_CURR_PIN`    | ⚠️ VERIFY       |
| `BATT_VOLT_MULT`   | ⚠️ VERIFY       |
| `BATT_AMP_PERVLT`  | ⚠️ VERIFY       |

Calibrate `BATT_VOLT_MULT` against a multimeter. Calibrate `BATT_AMP_PERVLT` against a known
load or a measured mAh-in after charging.

---

## Building ArduPilot

### Prerequisites

Linux (Ubuntu 22.04/24.04) or WSL2 is recommended.

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

If the board is not yet merged upstream, copy the hwdef into the tree:

```bash
cp -r /path/to/nano-h743-aio-flight-controller/hwdef/NanoH743AIO \
      libraries/AP_HAL_ChibiOS/hwdef/
```

The directory should contain:

```
NanoH743AIO/
├── hwdef.dat         # main firmware hardware definition
├── hwdef-bl.dat      # bootloader hardware definition
├── defaults.parm     # board default parameters
└── README.md         # upstream board documentation
```

### Build firmware

```bash
./waf configure --board NanoH743AIO
./waf copter          # or: plane, rover, heli
```

Output files are written to `build/NanoH743AIO/bin/`:

| File                       | Purpose                                      |
|----------------------------|----------------------------------------------|
| `arducopter.apj`           | Firmware for Mission Planner / QGC upload    |
| `arducopter.bin`           | Raw firmware (no bootloader)                 |
| `arducopter_with_bl.hex`   | Firmware + bootloader, for first-time DFU    |

### Build the bootloader

```bash
./waf configure --board NanoH743AIO --bootloader
./waf bootloader
# or, to build and install into Tools/bootloaders/:
Tools/scripts/build_bootloaders.py NanoH743AIO
```

### Upload directly over USB (board already running ArduPilot)

```bash
./waf configure --board NanoH743AIO
./waf copter --upload
```

---

## Flashing

### First-time flash (DFU mode)

1. Unplug the battery.
2. Hold the **BOOT** button and connect USB. The board enumerates as
   `STM32 BOOTLOADER` (VID:PID `0483:df11`).
3. Flash with one of the following tools.

**STM32CubeProgrammer (Windows/macOS/Linux):**

```bash
STM32_Programmer_CLI -c port=USB1 -w arducopter_with_bl.hex -v -rst
```

**dfu-util (Linux/macOS):**

```bash
# dfu-util needs a .bin; convert the hex first:
arm-none-eabi-objcopy -I ihex -O binary arducopter_with_bl.hex arducopter_with_bl.bin
dfu-util -a 0 --dfuse-address 0x08000000:leave -D arducopter_with_bl.bin
```

**Betaflight Configurator:** Use "Load Firmware [Local]" with the `_with_bl.hex` file and enable
"Full chip erase".

### Subsequent updates

After the ArduPilot bootloader is installed, no DFU is required:

- **Mission Planner:** *Setup → Install Firmware → Load custom firmware* → select `.apj`
- **QGroundControl:** *Vehicle Setup → Firmware → Advanced → Custom firmware file*
- **CLI:** `Tools/scripts/uploader.py build/NanoH743AIO/bin/arducopter.apj`

### Linux udev rule (if DFU is not detected)

```bash
echo 'SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="df11", MODE="0666"' | \
  sudo tee /etc/udev/rules.d/50-stm32-dfu.rules
sudo udevadm control --reload-rules
```

### ESC firmware

Flash ESC firmware via ArduPilot passthrough (`SERVO_BLH_AUTO = 1`) using BLHeliSuite32,
ESC-Configurator, or the AM32 configurator. **Remove propellers first.**

---

## Initial Configuration

1. Flash firmware and connect a ground control station.
2. Set the frame: `FRAME_CLASS = 1` (Quad), `FRAME_TYPE = 1` (X) — ⚠️ VERIFY motor order.
3. Complete accelerometer calibration, plus compass calibration if fitted.
4. Run radio calibration and confirm `RC_PROTOCOLS`.
5. Run the motor test **without propellers** and confirm order and direction.
6. Calibrate the battery monitor.
7. Set failsafes: `FS_THR_ENABLE`, `BATT_FS_LOW_ACT`, `BATT_LOW_VOLT`.

---

## Thermal Specifications

### Silicon limits (from manufacturer datasheets)

| Component            | Parameter                      | Limit                                   |
|----------------------|--------------------------------|-----------------------------------------|
| STM32H743VIT6        | Operating ambient (suffix 6)   | −40 °C to +85 °C                        |
| STM32H743VIT6        | Max junction temperature       | See ST DS12110 for your exact part code |
| ESC M