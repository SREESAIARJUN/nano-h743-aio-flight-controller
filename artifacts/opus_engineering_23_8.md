I don't have access to the contents of the `nano-h743-aio-flight-controller` repository or its "Cycle 23" revision data. The README below is therefore a production-ready **template**:

- **Accurate:** ArduPilot build, bootloader, and flashing commands, and the documented STM32H743 MCU characteristics.
- **Placeholders:** Board-specific values (pin assignments, current-sensor scale, thermal measurements, board ID) are marked `⚠️ VERIFY` or shown as `TBD`. Replace them with values from your schematic, `hwdef.dat`, and test reports before publishing. Unverified pinouts in flight-controller docs can cause miswired ESCs or damaged hardware.

---

````markdown
# Nano H743 AIO Flight Controller

![Hardware Rev](https://img.shields.io/badge/hardware-Cycle%2023-blue)
![MCU](https://img.shields.io/badge/MCU-STM32H743-informational)
![Firmware](https://img.shields.io/badge/firmware-ArduPilot-green)
![License](https://img.shields.io/badge/license-TBD-lightgrey)

A compact all-in-one (AIO) flight controller built around the STM32H743 MCU, designed for
ArduPilot (Copter / Plane / Rover). It targets small-frame multirotors and fixed-wing
platforms where weight and board area are constrained.

> **Revision:** Cycle 23 (hardware) · **Document status:** ⚠️ VERIFY all board-specific values against the Cycle 23 schematic and `hwdef.dat` before release.

---

## Table of Contents

1. [Specifications](#specifications)
2. [Board Layout & Pinout](#board-layout--pinout)
3. [UART Mapping](#uart-mapping)
4. [PWM / Motor Outputs](#pwm--motor-outputs)
5. [Power & Battery Monitoring](#power--battery-monitoring)
6. [Thermal Specifications](#thermal-specifications)
7. [Building ArduPilot](#building-ardupilot)
8. [Flashing](#flashing)
9. [First-Time Configuration](#first-time-configuration)
10. [Troubleshooting](#troubleshooting)
11. [Repository Structure](#repository-structure)
12. [Revision History](#revision-history)
13. [Contributing](#contributing)
14. [License](#license)

---

## Specifications

| Parameter            | Value                                              |
|----------------------|----------------------------------------------------|
| MCU                  | STM32H743 (Arm Cortex-M7, up to 480 MHz, 2 MB flash, 1 MB RAM) |
| IMU                  | TBD ⚠️ VERIFY (e.g., ICM-42688-P / BMI270)          |
| Barometer            | TBD ⚠️ VERIFY (e.g., DPS310 / BMP280)               |
| OSD                  | TBD ⚠️ VERIFY (e.g., AT7456E analog OSD)            |
| Onboard flash (logs) | TBD ⚠️ VERIFY (e.g., 16 MB SPI NOR) or microSD     |
| Integrated ESC       | TBD ⚠️ VERIFY (e.g., 4-in-1, xx A continuous / xx A burst) |
| Input voltage        | TBD ⚠️ VERIFY (e.g., 2S–6S LiPo)                    |
| BEC outputs          | TBD ⚠️ VERIFY (e.g., 5 V / x A, 9 V / x A)          |
| UARTs                | TBD ⚠️ VERIFY                                       |
| CAN                  | TBD ⚠️ VERIFY (DroneCAN supported if present)       |
| Mounting pattern     | TBD ⚠️ VERIFY (e.g., 25.5 × 25.5 mm, M2)            |
| Dimensions           | TBD ⚠️ VERIFY                                       |
| Weight               | TBD ⚠️ VERIFY                                       |

---

## Board Layout & Pinout

> Place top and bottom renders in `docs/images/` and update the links below.

| Top View | Bottom View |
|----------|-------------|
| ![Top](docs/images/nano-h743-top.png) | ![Bottom](docs/images/nano-h743-bottom.png) |

### Solder Pad Reference ⚠️ VERIFY

| Pad Label | Function                 | MCU Pin | Voltage Level | Notes                       |
|-----------|--------------------------|---------|---------------|-----------------------------|
| `BAT+`    | Battery input            | —       | VBAT          | Main power, see input range |
| `GND`     | Ground                   | —       | 0 V           |                             |
| `5V`      | 5 V BEC output           | —       | 5 V           | Max current: TBD            |
| `9V`      | 9 V BEC (VTX / camera)   | —       | 9 V           | Switchable? TBD             |
| `T1/R1`   | UART1 TX / RX            | TBD     | 3.3 V         |                             |
| `T2/R2`   | UART2 TX / RX            | TBD     | 3.3 V         |                             |
| `T3/R3`   | UART3 TX / RX            | TBD     | 3.3 V         |                             |
| `T4/R4`   | UART4 TX / RX            | TBD     | 3.3 V         |                             |
| `SCL/SDA` | I²C (compass)            | TBD     | 3.3 V         |                             |
| `LED`     | WS2812 LED strip         | TBD     | 3.3 V logic   |                             |
| `BZ+/BZ-` | Buzzer                   | TBD     | 5 V           |                             |
| `CURR`    | Current sensor (ADC)     | TBD     | 0–3.3 V       |                             |
| `RSSI`    | Analog RSSI              | TBD     | 0–3.3 V       |                             |
| `CAM`     | Camera video in          | —       | Analog        |                             |
| `VTX`     | Video out                | —       | Analog        |                             |
| `BOOT`    | Bootloader / DFU button  | BOOT0   | —             | Hold while plugging USB     |

> ⚠️ **All UART and signal pads are 3.3 V logic.** Do not connect 5 V signal lines directly.

---

## UART Mapping

ArduPilot `SERIALn` ports map to physical UARTs through the `SERIAL_ORDER` line in `hwdef.dat`.

| ArduPilot Port | Physical UART | Default Protocol      | Default Baud | Notes          |
|----------------|---------------|-----------------------|--------------|----------------|
| `SERIAL0`      | USB (OTG1)    | MAVLink2              | 115200       | Console        |
| `SERIAL1`      | TBD           | MAVLink2 (telemetry)  | 57600        |                |
| `SERIAL2`      | TBD           | MAVLink2              | 57600        |                |
| `SERIAL3`      | TBD           | GPS                   | 230400       |                |
| `SERIAL4`      | TBD           | GPS                   | 230400       |                |
| `SERIAL5`      | TBD           | ESC Telemetry         | 115200       | From onboard ESC |
| `SERIAL6`      | TBD           | RCIN (SBUS/CRSF/ELRS) | —            | DMA-capable    |
| `SERIAL7`      | TBD           | DisplayPort / VTX     | 115200       |                |

To assign a protocol at runtime, set `SERIALn_PROTOCOL` and `SERIALn_BAUD` in your GCS.

---

## PWM / Motor Outputs

| Output | Timer / Channel | Group | DShot | Bidir-DShot | Default Function |
|--------|-----------------|-------|-------|-------------|------------------|
| M1     | TBD             | 1     | ✅    | ✅ TBD      | Motor 1          |
| M2     | TBD             | 1     | ✅    | ✅ TBD      | Motor 2          |
| M3     | TBD             | 1     | ✅    | ✅ TBD      | Motor 3          |
| M4     | TBD             | 1     | ✅    | ✅ TBD      | Motor 4          |
| S5     | TBD             | 2     | ✅    | TBD         | Servo / Aux      |
| S6     | TBD             | 2     | ✅    | TBD         | Servo / Aux      |
| LED    | TBD             | 3     | —     | —           | NeoPixel (`SERVO_FUNCTION = 120`) |

> Outputs in the same **group** share a timer and must use the same protocol and rate.
> For example, you cannot mix DShot and PWM within one group.

Recommended ESC setup for the onboard 4-in-1:

```text
MOT_PWM_TYPE        = 6      # DShot600 (use 5 for DShot300 if needed)
SERVO_BLH_AUTO      = 1
SERVO_BLH_BDMASK    = 15     # enable bidirectional DShot on M1–M4 (if supported)
SERIAL5_PROTOCOL    = 16     # ESC telemetry (if wired)
```

---

## Power & Battery Monitoring

| Parameter         | Default Value | Notes                                |
|-------------------|---------------|--------------------------------------|
| `BATT_MONITOR`    | 4             | Analog voltage + current             |
| `BATT_VOLT_PIN`   | TBD ⚠️ VERIFY | ADC channel                          |
| `BATT_CURR_PIN`   | TBD ⚠️ VERIFY | ADC channel                          |
| `BATT_VOLT_MULT`  | TBD ⚠️ VERIFY | Divider ratio from schematic         |
| `BATT_AMP_PERVLT` | TBD ⚠️ VERIFY | Current-sensor scale (A/V)           |
| `BATT_AMP_OFFSET` | TBD ⚠️ VERIFY |                                      |

Calibrate against a known multimeter or power-meter reading after first assembly.

---

## Thermal Specifications

### MCU Ratings (STM32H743 datasheet)

| Parameter                       | Value |
|---------------------------------|-------|
| Max junction temperature (Tj)   | 125 °C (STM32H743xI datasheet; verify for your exact part number) |
| Ambient range (temp-range suffix 6) | −40 °C to +85 °C |
| Ambient range (temp-range suffix 3) | −40 °C to +125 °C (reduced max frequency applies) |

> ⚠️ VERIFY which ordering code (temperature suffix) is populated on Cycle 23 boards. Check
> the datasheet for any frequency derating at high temperature.

### Board-Level Thermal Characterization (Cycle 23) ⚠️ VERIFY: replace with measured data

| Condition                                  | MCU Temp | ESC FET Temp | BEC Temp | Notes |
|--------------------------------------------|----------|--------------|----------|-------|
| Idle on USB, 25 °C ambient, no airflow     | TBD      | —            | TBD      |       |
| Armed, 0% throttle, 25 °C, no airflow      | TBD      | TBD          | TBD      |       |
| Hover (~xx A), 25 °C, prop wash            | TBD      | TBD          | TBD      |       |
| Sustained xx A, 25 °C, no airflow (bench)  | TBD      | TBD          | TBD      | Worst case |
| Max 9 V BEC load, 40 °C ambient            | TBD      | —            | TBD      |       |

### Operating Guidance

- **Recommended operating ambient:** TBD (e.g., −10 °C to +45 °C).
- **Do not run the integrated ESC at rated current without airflow.** Bench tests without
  propellers provide no cooling.
- **IMU heater:** If `BRD_HEAT_TARG` is supported, set it to keep the IMU temperature stable.
  This reduces drift in cold conditions.
- **Monitoring:**
  - The MCU temperature is logged in the `POWR`/`MCU` messages. Check with `MAV_SYS_STATUS` or a log review.
  - ESC temperatures are available via ESC telemetry (`ESC` log messages).
- **Thermal protection:** The integrated ESC firmware (e.g., AM32/BLHeli_32) may implement
  over-temperature throttling. Document the thresholds here: TBD.

---

## Building ArduPilot

### 1. Prerequisites

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

For macOS and Windows (WSL2), see the
[ArduPilot build environment docs](https://ardupilot.org/dev/docs/building-the-code.html).

### 2. Add the Board Definition (if not yet upstream)

Copy this repository's hwdef into the ArduPilot tree:

```bash
cp -r <this-repo>/hwdef/NanoH743AIO \
      libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO
```

The directory must contain:

```text
NanoH743AIO/
├── hwdef.dat        # main firmware definition
├── hwdef-bl.dat     # bootloader definition
├── defaults.parm    # board default parameters
└── README.md        # upstream board documentation
```

> The `APJ_BOARD_ID` in `hwdef.dat` must be unique and registered in
> `Tools/AP_Bootloader/board_types.txt`. Request an ID via a PR to ArduPilot before release.
> Board ID: **TBD ⚠️ VERIFY**

### 3. Configure & Build

```bash
./waf configure --board NanoH743AIO
./waf copter          # or: plane, rover, heli, sub, antennatracker
```

Output artifacts are written to `build/NanoH743AIO/bin/`:

| File                        | Purpose                                         |
|-----------------------------|-------------------------------------------------|
| `arducopter.apj`            | Upload via GCS / bootloader (no BL included)    |
| `arducopter_with_bl.hex`    | Full image incl. bootloader, for DFU / SWD      |
| `arducopter.bin`            | Raw application binary                          |

### 4. Build the Bootloader

```bash
Tools/scripts/build_bootloaders.py NanoH743AIO
# Output: Tools/bootloaders/NanoH743AIO_bl.bin / .hex
```

Rebuild the firmware afterwards so `*_with_bl.hex` embeds the new bootloader.

### 5. Optional Build Flags

```bash
./waf configure --board NanoH743AIO --debug        # debug symbols, no optimization
./waf configure --board NanoH743AIO --enable-dds   # ROS 2 / DDS support (if flash allows)
./waf copter --upload                              # build and flash over USB (ArduPilot BL required)
```

---

## Flashing

### Option A: Board Already Running the ArduPilot Bootloader (recommended)

Use any of these methods:

- **Mission Planner:** *Setup → Install Firmware → Load custom firmware* → select `arducopter.apj`.
- **QGroundControl:** *Vehicle Setup → Firmware → Advanced → Custom firmware file*.
- **Command line:**
  ```bash
  ./waf copter --upload
  # or
  python3 Tools/sc