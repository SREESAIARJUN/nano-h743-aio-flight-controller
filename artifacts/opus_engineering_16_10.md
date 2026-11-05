**Note:** I don't have access to this repository's schematics, `hwdef.dat`, or test data, so I can't know its actual pin assignments or measured thermal results. The README below is complete and structured. ArduPilot build and flash commands and STM32H743 silicon facts are accurate. Board-specific values are marked **`⚠ VERIFY`** or shown as `<placeholders>`. Fill these in from your schematic and test logs before publishing. Pinout and thermal data that look authoritative but are wrong can damage hardware.

---

````markdown
# Nano H743 AIO Flight Controller

> Compact all-in-one (FC + 4-in-1 ESC) flight controller based on the STM32H743, designed for ArduPilot.

![Board Revision](https://img.shields.io/badge/hardware-rev%20<X.Y>-blue)
![Firmware](https://img.shields.io/badge/ArduPilot-4.5%2B-green)
![License](https://img.shields.io/badge/license-<LICENSE>-lightgrey)

---

## Table of Contents
1. [Overview](#overview)
2. [Specifications](#specifications)
3. [Board Layout & Pinout](#board-layout--pinout)
4. [UART Mapping](#uart-mapping)
5. [PWM / Motor Outputs](#pwm--motor-outputs)
6. [Power & Analog Sensing](#power--analog-sensing)
7. [Building ArduPilot](#building-arduPilot)
8. [Flashing Firmware](#flashing-firmware)
9. [First-Time Setup](#first-time-setup)
10. [Thermal Specifications](#thermal-specifications)
11. [Troubleshooting](#troubleshooting)
12. [Repository Structure](#repository-structure)
13. [Revision History](#revision-history)
14. [License & Safety](#license--safety)

---

## Overview

The Nano H743 AIO integrates a flight controller and 4-in-1 ESC on a single
board for small multirotors (2.5"–4" class). It runs ArduPilot natively via a
custom `hwdef` and supports DShot, bidirectional DShot (RPM filtering), OSD,
and onboard blackbox logging.

---

## Specifications

| Parameter            | Value                                              |
|----------------------|----------------------------------------------------|
| MCU                  | STM32H743VIT6 — Cortex-M7 @ 480 MHz, 2 MB Flash, 1 MB RAM |
| IMU                  | `<e.g. ICM-42688-P>` (SPI) ⚠ VERIFY                 |
| Barometer            | `<e.g. DPS310 / BMP280>` (I²C) ⚠ VERIFY             |
| OSD                  | `<AT7456E / none>` ⚠ VERIFY                         |
| Logging              | `<16 MB SPI flash / microSD>` ⚠ VERIFY              |
| ESC                  | `<4-in-1, XX A continuous / XX A burst, BLHeli_S / AM32>` ⚠ VERIFY |
| Input voltage        | `<2–6S LiPo>` ⚠ VERIFY                              |
| BEC outputs          | 5 V @ `<X>` A, `<9 V / 10 V>` @ `<X>` A ⚠ VERIFY      |
| UARTs                | `<N>` ⚠ VERIFY                                      |
| Mounting             | `<25.5 × 25.5 mm, M2>` ⚠ VERIFY                     |
| Dimensions / Weight  | `<XX × XX mm / X.X g>` ⚠ VERIFY                     |
| USB                  | USB-C (DFU-capable)                                |

---

## Board Layout & Pinout

> Replace with rendered images from `/docs/images/`.

```
          ┌──────────────────────────────────────┐
   M1 ●───┤                                      ├───● M2
          │   [USB-C]          [BOOT]            │
          │                                      │
          │      ┌────────────┐                  │
          │      │ STM32H743  │      [IMU]       │
          │      └────────────┘                  │
          │                                      │
   M4 ●───┤  5V G TX1 RX1  SDA SCL  VTX CAM      ├───● M3
          └──────────────────────────────────────┘
                     [ BAT+ ]  [ BAT− ]
```

![Top side pinout](docs/images/pinout_top.png)
![Bottom side pinout](docs/images/pinout_bottom.png)

### Solder Pad Reference ⚠ VERIFY ALL

| Pad      | Function                   | MCU Pin  | Notes                     |
|----------|----------------------------|----------|---------------------------|
| 5V       | 5 V BEC output             | —        | Max `<X>` A shared        |
| 9V/10V   | VTX power                  | —        | Switchable via `RELAY` (if fitted) |
| G        | Ground                     | —        |                           |
| TX1/RX1  | UART1                      | `<PAx>`  | Default: MAVLink / telemetry |
| TX2/RX2  | UART2                      | `<PAx>`  | Default: GPS              |
| RX       | UART for RC input          | `<PBx>`  | CRSF / ELRS / SBUS        |
| SDA/SCL  | I²C (external compass)     | `<PBx>`  | 3.3 V logic, pull-ups `<onboard?>` |
| CAM      | Analog video in            | —        | To OSD                    |
| VTX      | Analog video out           | —        |                           |
| LED      | WS2812 LED strip           | `<PAx>`  | Timer `<TIMx_CHx>`        |
| BZ+/BZ−  | Buzzer                     | `<PCx>`  | 5 V active buzzer         |
| CURR     | Current sense (internal)   | `<PCx>`  | ADC                       |

---

## UART Mapping

ArduPilot `SERIALn` order for this board (from `hwdef.dat` `SERIAL_ORDER`):

| ArduPilot Port | Hardware UART | Default Protocol | Default Baud |
|----------------|---------------|------------------|--------------|
| SERIAL0        | USB (OTG1)    | MAVLink2         | 115200       |
| SERIAL1        | `<USARTx>`    | MAVLink2         | 57600        |
| SERIAL2        | `<USARTx>`    | `<MAVLink2>`     | 57600        |
| SERIAL3        | `<USARTx>`    | GPS              | 38400        |
| SERIAL4        | `<USARTx>`    | `<ESC Telem>`    | 115200       |
| SERIAL5        | `<USARTx>`    | `<DisplayPort>`  | 115200       |
| SERIAL6        | `<USARTx>`    | RCIN (23)        | —            |

All UARTs support DMA unless noted. ⚠ VERIFY against `SERIAL_ORDER` and DMA allocation in the build output.

---

## PWM / Motor Outputs

| Output | Pad | Timer Group  | DShot | Bi-Dir DShot |
|--------|-----|--------------|-------|--------------|
| 1      | M1  | `<TIMx>` (G1)| ✅    | ✅           |
| 2      | M2  | `<TIMx>` (G1)| ✅    | ✅           |
| 3      | M3  | `<TIMx>` (G2)| ✅    | ✅           |
| 4      | M4  | `<TIMx>` (G2)| ✅    | ✅           |
| 5      | LED | `<TIMx>` (G3)| ✅    | ❌           |

> **All outputs in the same timer group must use the same protocol/rate.**
> Recommended: `MOT_PWM_TYPE = 6` (DShot600) and `SERVO_BLH_BDMASK = 15`
> for bidirectional DShot on motors 1–4.

---

## Power & Analog Sensing

| Parameter         | Default Value | Notes                        |
|-------------------|---------------|------------------------------|
| `BATT_MONITOR`    | 4             | Analog voltage + current     |
| `BATT_VOLT_PIN`   | `<n>`         | ⚠ VERIFY                     |
| `BATT_CURR_PIN`   | `<n>`         | ⚠ VERIFY                     |
| `BATT_VOLT_MULT`  | `<11.0>`      | Calibrate with a multimeter  |
| `BATT_AMP_PERVLT` | `<xx.x>`      | Calibrate with a known load  |

---

## Building ArduPilot

### Prerequisites

Ubuntu 22.04 / 24.04 (or WSL2):

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

### Add the board definition

Copy this repo's hwdef into the ArduPilot tree. Skip this step if the board is already upstream.

```bash
cp -r <this-repo>/ardupilot/hwdef/NanoH743AIO \
      libraries/AP_HAL_ChibiOS/hwdef/
```

The directory must contain:
- `hwdef.dat` — main firmware definition
- `hwdef-bl.dat` — bootloader definition
- `defaults.parm` — board default parameters (optional)
- `README.md` — upstream board documentation (required for upstream PRs)

### Build the bootloader (first time / bootloader changes only)

```bash
Tools/scripts/build_bootloaders.py NanoH743AIO
# Output: Tools/bootloaders/NanoH743AIO_bl.bin / .hex
```

### Build the vehicle firmware

```bash
./waf configure --board NanoH743AIO
./waf copter           # ArduCopter
# ./waf plane          # ArduPlane
# ./waf rover          # Rover
```

Artifacts are written to `build/NanoH743AIO/bin/`:

| File                          | Use                                         |
|-------------------------------|---------------------------------------------|
| `arducopter.apj`              | Upload via Mission Planner / QGC (bootloader present) |
| `arducopter_with_bl.hex`      | First-time flash via DFU (includes bootloader) |
| `arducopter.bin`              | Raw app image (offset 0x08020000 on H7)     |

### Clean build

```bash
./waf distclean
./waf configure --board NanoH743AIO
./waf copter
```

---

## Flashing Firmware

### Method A: First-time flash (DFU, includes bootloader)

1. Hold **BOOT** and connect USB-C. Release BOOT.
2. Verify that DFU mode is detected:
   ```bash
   dfu-util -l
   # Expect: [0483:df11] ... "@Internal Flash  /0x08000000/..."
   ```
3. Flash with **STM32CubeProgrammer** (recommended for `.hex`):
   ```bash
   STM32_Programmer_CLI -c port=USB1 -e all \
       -w build/NanoH743AIO/bin/arducopter_with_bl.hex -v -rst
   ```
   Or convert the hex to bin and use `dfu-util`:
   ```bash
   arm-none-eabi-objcopy -I ihex -O binary \
       arducopter_with_bl.hex arducopter_with_bl.bin
   dfu-util -a 0 --dfuse-address 0x08000000:leave -D arducopter_with_bl.bin
   ```
4. Power-cycle the board. The status LED should show the ArduPilot boot pattern.

> **Linux:** if `dfu-util` reports permission errors, install the udev rules
> from `Tools/environment_install/` or run with `sudo`.

### Method B: Updates (ArduPilot bootloader already installed)

- **Mission Planner:** Setup → Install Firmware → *Load custom firmware* → select `.apj`
- **QGroundControl:** Vehicle Setup → Firmware → *Advanced* → *Custom firmware file*
- **Command line:**
  ```bash
  ./waf copter --upload
  # or
  Tools/scripts/uploader.py build/NanoH743AIO/bin/arducopter.apj
  ```

### Migrating from Betaflight / INAV

Flash `arducopter_with_bl.hex` using Method A. A full chip erase is required. Betaflight Configurator can also flash the `_with_bl.hex` in DFU mode with **Full chip erase** enabled.

---

## First-Time Setup

1. Load board defaults (if not embedded): `Config → Full Parameter List → Load from file → defaults.parm`
2. Set the frame: `FRAME_CLASS = 1` (Quad), `FRAME_TYPE = 1` (X)
3. Calibrate the accelerometer and (if fitted) the external compass
4. Configure RC: `SERIALx_PROTOCOL = 23`, `RSSI_TYPE = 3` (CRSF/ELRS)
5. Verify motor order and direction in **Motor Test** *with props removed*
6. Enable the harmonic notch with ESC RPM: `INS_HNTCH_ENABLE = 1`, `INS_HNTCH_MODE = 3`
7. Calibrate the battery monitor (see [Power & Analog Sensing](#power--analog-sensing))

---

## Thermal Specifications

### Silicon limits (manufacturer datasheets)

| Component            | Parameter                      | Limit            |
|----------------------|--------------------------------|------------------|
| STM32H743VIT6        | Ambient operating range (suffix 6) | −40 °C to +85 °C |
| STM32H743VIT6        | Max junction temperature (Tj)  | +125 °C          |
| IMU `<part>`         | Operating range                | ⚠ VERIFY datasheet |
| ESC MOSFETs `<part>` | Max Tj                         | ⚠ VERIFY datasheet (typ. 150 °C) |
| BEC regulator `<part>` | Thermal shutdown             | ⚠ VERIFY datasheet |

### Board-level characterization ⚠ REPLACE WITH MEASURED DATA

| Condition                                  | MCU Temp | ESC FET Temp | BEC Temp | Ambient |
|--------------------------------------------|----------|--------------|----------|---------|
| Idle on USB, no airflow                    | `<°C>`   | —            | `<°C>`   | 25 °C   |
| Armed, 4S, motors idle, no airflow         | `<°C>`   | `<°C>`       | `<°C>`   | 25 °C   |
| Hover, `<X>` A total, in-frame airflow     | `<°C>`   | `<°C>`       | `<°C>`   | 25 °C   |
| Sustained `<X>` A per motor, 60 s          | `<°C>`   | `<°C>`       | `<°C>`   | 25 °C   |
| Burst `<X>` A per motor, 10 s              | `<°C>`   | `<°C>`       | `<°C>`   | 25 °C   |

*Test method: `<thermocouple / FLIR model>`, `<frame>`, `<motor/prop>`, log reference `<file>`.*

### Thermal guidance

- **Do not run the ESC at rated current without airflow