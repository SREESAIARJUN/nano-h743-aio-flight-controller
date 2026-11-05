# nano-h743-aio-flight-controller

> **About this document:** I don't have access to this repository's schematics, `hwdef.dat`, or test data, so this README is a **production-structured template**. Generic STM32H743 and ArduPilot details (toolchain, build system, DFU flashing, MCU limits) are accurate as written. Every **board-specific value** (pin mappings, timer assignments, current ratings, thermal measurements, board ID) is marked `⚠️ VERIFY`. Replace each one with data from your schematic and validated hardware before publishing.

---

<p align="center">
  <b>Nano H743 AIO</b> · STM32H743 · Integrated 4-in-1 ESC · ArduPilot-native
</p>

<p align="center">
  <img alt="MCU" src="https://img.shields.io/badge/MCU-STM32H743-blue">
  <img alt="Firmware" src="https://img.shields.io/badge/firmware-ArduPilot-green">
  <img alt="License" src="https://img.shields.io/badge/license-GPLv3-lightgrey">
  <img alt="Status" src="https://img.shields.io/badge/status-REV_X-orange">
</p>

---

## Table of Contents

1. [Overview](#overview)
2. [Specifications](#specifications)
3. [Board Layout & Pinout](#board-layout--pinout)
4. [UART / Serial Mapping](#uart--serial-mapping)
5. [PWM / Motor Output Mapping](#pwm--motor-output-mapping)
6. [Analog Sensing (Voltage / Current / RSSI)](#analog-sensing)
7. [Building ArduPilot](#building-arduplot-firmware)
8. [Flashing](#flashing)
9. [First-Time Configuration](#first-time-configuration)
10. [Thermal Specifications](#thermal-specifications)
11. [Electrical Limits & Power](#electrical-limits--power)
12. [Troubleshooting](#troubleshooting)
13. [Repository Structure](#repository-structure)
14. [Contributing](#contributing)
15. [Safety Notice](#safety-notice)
16. [License](#license)

---

## Overview

The **Nano H743 AIO** is a compact all-in-one flight controller combining an STM32H743 flight controller and an integrated 4-in-1 BLHeli_S/AM32 ESC (`⚠️ VERIFY` ESC firmware) on a single board. It targets sub-250 g to 3" class multirotors running **ArduPilot** (Copter, with Plane/Rover builds supported where outputs allow).

**Key features**

- STM32H743 @ 480 MHz, 2 MB flash, 1 MB RAM
- Integrated 4-in-1 ESC with DShot support and bidirectional DShot (`⚠️ VERIFY`)
- On-board IMU, barometer, and OSD (`⚠️ VERIFY` part numbers)
- Battery voltage and current sensing
- USB-C with native DFU bootloader support
- 25.5 × 25.5 mm M2 mounting pattern (`⚠️ VERIFY`)

---

## Specifications

| Parameter | Value |
|---|---|
| MCU | STM32H743VIT6 / IIT6 (`⚠️ VERIFY` package) |
| Core / Clock | ARM Cortex-M7 @ 480 MHz |
| Flash / RAM | 2 MB / 1 MB |
| IMU | `⚠️ VERIFY` (e.g., ICM-42688-P, BMI270) on SPI`x` |
| Barometer | `⚠️ VERIFY` (e.g., DPS310, BMP280) on I2C`x` |
| OSD | `⚠️ VERIFY` (e.g., AT7456E) on SPI`x` |
| Blackbox / Logging | `⚠️ VERIFY` (SPI flash or microSD) |
| UARTs | `⚠️ VERIFY` count |
| ESC | 4-in-1, `⚠️ VERIFY` A continuous / A burst per channel |
| Input voltage | `⚠️ VERIFY` (e.g., 1S–4S LiPo) |
| BEC outputs | 5 V @ `⚠️` A, 3.3 V @ `⚠️` A, (optional) 9 V @ `⚠️` A |
| Current sensor | On-board shunt/hall, scale `⚠️ VERIFY` A/V |
| Mounting | `⚠️ VERIFY` mm, M2 / M3 |
| Dimensions | `⚠️ VERIFY` mm |
| Weight | `⚠️ VERIFY` g |
| Firmware target | `NanoH743AIO` (`⚠️ VERIFY` hwdef name) |

---

## Board Layout & Pinout

### Top side

```
          ┌─────────────────────────────────────────┐
          │  M4 ●                         ● M1      │
          │                                         │
          │   [5V][GND][T1][R1]     [USB-C]         │
          │                                         │
          │   [T2][R2]     ┌──────────┐   [BOOT]    │
          │                │ STM32H743│             │
          │   [SDA][SCL]   └──────────┘   [LED]     │
          │                                         │
          │   [VTX][CAM][9V]       [BAT+][BAT-]     │
          │  M3 ●                         ● M2      │
          └─────────────────────────────────────────┘
           ⚠️ VERIFY: illustrative layout only — replace
              with a rendered image from the PCB files
```

> Recommended: replace the ASCII art with `docs/img/pinout_top.png` and `docs/img/pinout_bottom.png` exported from your PCB tool.

### Solder pad reference

| Pad | Function | MCU Pin | Voltage | Notes |
|---|---|---|---|---|
| `BAT+` / `BAT-` | Battery input | — | `⚠️` V max | Main power, use ≥ `⚠️` AWG |
| `5V` | 5 V BEC out | — | 5 V | Receiver / GPS |
| `9V` | 9 V BEC out | — | 9 V | VTX / camera (`⚠️ VERIFY`) |
| `3V3` | 3.3 V out | — | 3.3 V | Low-current peripherals only |
| `T1` / `R1` | UART1 TX/RX | `⚠️` / `⚠️` | 3.3 V | Default: MAVLink |
| `T2` / `R2` | UART2 TX/RX | `⚠️` / `⚠️` | 3.3 V | Default: GPS |
| `SDA` / `SCL` | I2C`x` | `⚠️` / `⚠️` | 3.3 V | Compass / external baro |
| `CAM` | Analog video in | — | — | To OSD |
| `VTX` | Analog video out | — | — | From OSD |
| `LED` | WS2812 data | `⚠️` | 5 V tolerant? `⚠️` | Serial LED |
| `BZ+` / `BZ-` | Buzzer | `⚠️` | 5 V | |
| `RSSI` | Analog RSSI | `⚠️` | 3.3 V max | |
| `BOOT` | DFU button | BOOT0 | — | Hold while plugging USB |

---

## UART / Serial Mapping

ArduPilot's `SERIALn` numbering follows the `SERIAL_ORDER` line in `hwdef.dat`, **not** the physical UART number.

| ArduPilot Port | MCU Peripheral | Pads | Default Protocol | DMA |
|---|---|---|---|---|
| `SERIAL0` | OTG1 (USB) | USB-C | MAVLink2 | — |
| `SERIAL1` | `⚠️` USART1 | T1 / R1 | MAVLink2 (`SERIAL1_PROTOCOL=2`) | `⚠️` |
| `SERIAL2` | `⚠️` USART2 | T2 / R2 | GPS (`SERIAL2_PROTOCOL=5`) | `⚠️` |
| `SERIAL3` | `⚠️` USART3 | T3 / R3 | RC input (`SERIAL3_PROTOCOL=23`) | `⚠️` |
| `SERIAL4` | `⚠️` UART4 | T4 / R4 | ESC telemetry (`16`) | `⚠️` |
| `SERIAL5` | `⚠️` | — | — | — |

**RC input:** CRSF/ELRS, SBUS (inverted — `⚠️ VERIFY` whether a hardware inverter is present), and other serial protocols are supported via `SERIALn_PROTOCOL=23`.

---

## PWM / Motor Output Mapping

| Output | Function | MCU Pin | Timer / Channel | DShot | BDShot |
|---|---|---|---|---|---|
| M1 | Motor 1 (internal ESC) | `⚠️` | `⚠️ TIMx_CHy` | ✅ | `⚠️` |
| M2 | Motor 2 (internal ESC) | `⚠️` | `⚠️` | ✅ | `⚠️` |
| M3 | Motor 3 (internal ESC) | `⚠️` | `⚠️` | ✅ | `⚠️` |
| M4 | Motor 4 (internal ESC) | `⚠️` | `⚠️` | ✅ | `⚠️` |
| LED | Serial LED | `⚠️` | `⚠️` | — | — |

> **Timer groups:** Outputs sharing a timer must run the same protocol and rate. Document the groups explicitly here once confirmed from `hwdef.dat` (e.g., "Group 1: M1–M2 (TIM3), Group 2: M3–M4 (TIM4)").

Recommended motor settings for the integrated ESC:

```
MOT_PWM_TYPE      6      # DShot600 (use 5 for DShot300 if unstable)
SERVO_BLH_AUTO    1
SERVO_BLH_BDMASK  15     # Bidirectional DShot on M1–M4 (only if supported)
SERVO_DSHOT_ESC   1      # BLHeli_S/BLHeli32/AM32 telemetry type (⚠️ VERIFY per ESC firmware)
```

---

## Analog Sensing

| Parameter | Value | Notes |
|---|---|---|
| `BATT_MONITOR` | `4` | Analog voltage + current |
| `BATT_VOLT_PIN` | `⚠️` | ADC channel from hwdef |
| `BATT_CURR_PIN` | `⚠️` | ADC channel from hwdef |
| `BATT_VOLT_MULT` | `⚠️` | Divider ratio, calibrate with multimeter |
| `BATT_AMP_PERVLT` | `⚠️` | Calibrate against a known load |
| `BATT_AMP_OFFSET` | `⚠️` | Zero-current offset |
| `RSSI_ANA_PIN` | `⚠️` | If analog RSSI is used |

These defaults should be baked into `hwdef.dat` via `define HAL_BATT_VOLT_PIN`, `HAL_BATT_VOLT_SCALE`, etc. They should then be verified on every hardware revision.

---

## Building ArduPilot Firmware

### 1. Set up the build environment

**Ubuntu / Debian (recommended):**

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

**macOS:**

```bash
Tools/environment_install/install-prereqs-mac.sh -y
```

**Windows:** Use WSL2 with Ubuntu, then follow the Ubuntu steps.

If you cloned without submodules:

```bash
git submodule update --init --recursive
```

### 2. Add the board definition

Copy this repository's hwdef files into the ArduPilot tree:

```bash
cp -r <this-repo>/hwdef/NanoH743AIO \
      libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO
```

Expected contents:

```
libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/
├── hwdef.dat        # Main firmware pin/peripheral definition
├── hwdef-bl.dat     # Bootloader definition
├── defaults.parm    # Board default parameters (optional)
└── README.md        # Board docs (ArduPilot wiki source)
```

> **Board ID:** `APJ_BOARD_ID` in `hwdef.dat` must be unique. Reserve one in `Tools/AP_Bootloader/board_types.txt` via a PR to ArduPilot before distributing firmware. `⚠️ VERIFY`: current ID = `____`.

### 3. Build the bootloader

```bash
Tools/scripts/build_bootloaders.py NanoH743AIO
```

Output: `Tools/bootloaders/NanoH743AIO_bl.bin` (and `.hex`). Commit this file so the main firmware build can embed it.

### 4. Build the firmware

```bash
./waf configure --board NanoH743AIO
./waf copter          # or: plane, rover, heli
```

Artifacts in `build/NanoH743AIO/bin/`:

| File | Purpose |
|---|---|
| `arducopter.apj` | Upload via Mission Planner / QGC (ArduPilot bootloader already installed) |
| `arducopter.bin` | Raw application image |
| `arducopter_with_bl.hex` | Bootloader + firmware. Use for **first flash** via DFU / STM32CubeProgrammer |

### 5. Useful build variants

```bash
./waf configure --board NanoH743AIO --debug     # Debug symbols, -Og
./waf clean                                      # Clean current board
./waf distclean                                  # Wipe all configuration
./waf copter --upload                            # Build and upload over USB
```

### 6. Custom feature selection (optional)

Use `Tools/scripts/build_options.py` or the [ArduPilot Custom Firmware Builder](https://custom.ardupilot.org/) to strip unused features if flash usage is high.

---

## Flashing

### Method A: First flash / recovery (DFU)

Use this method for a blank board, a board running Betaflight/INAV, or a bricked bootloader.

1. **Enter DFU:** Hold the `BOOT` button and connect USB-C. Release once connected.
2. **Confirm detection:**
   ```bash
   dfu-util -l
   # Expect: Found DFU: [0483:df11] ... name="@Internal Flash  /0x08000000/..."
   ```
3. **Flash with STM