I don't have access to the `nano-h743-aio-flight-controller` repository, its schematic, or its `hwdef.dat`, and I have no record of earlier "cycles." Below is a complete README with accurate, board-agnostic content: ArduPilot build commands, DFU flashing, and STM32H743 facts.

Board-specific values (pin assignments, current ratings, measured temperatures) are marked **`TBD`** or `<!-- VERIFY -->`. Inventing those would be risky, because a wrong UART, motor, or voltage mapping can damage hardware. Fill them in from your schematic and hwdef, and Section 4.4 shows how to extract them.

---

```markdown
# Nano H743 AIO Flight Controller

<!-- Badges: replace URLs with your CI / release locations -->
![Hardware Rev](https://img.shields.io/badge/hardware-rev%20TBD-blue)
![Firmware](https://img.shields.io/badge/ArduPilot-4.5%2B-green)
![License](https://img.shields.io/badge/license-TBD-lightgrey)

A compact all-in-one (AIO) flight controller built around the **STM32H743** MCU,
targeting ArduPilot (Copter / Plane / Rover). This document covers hardware
specifications, pinouts, firmware build and flashing procedures, and thermal
characteristics.

> **Safety:** Remove propellers before connecting USB, flashing firmware, or
> testing outputs. Incorrect configuration can spin motors unexpectedly.

---

## Table of Contents
1. [Hardware Overview](#1-hardware-overview)
2. [Specifications](#2-specifications)
3. [Pinout](#3-pinout)
4. [Building ArduPilot Firmware](#4-building-ardupilot-firmware)
5. [Flashing](#5-flashing)
6. [First-Time Configuration](#6-first-time-configuration)
7. [Thermal Specifications](#7-thermal-specifications)
8. [Troubleshooting](#8-troubleshooting)
9. [Repository Layout](#9-repository-layout)
10. [Contributing](#10-contributing)
11. [License](#11-license)

---

## 1. Hardware Overview

| Item               | Value                                   |
|--------------------|-----------------------------------------|
| Board name (hwdef) | `NanoH743AIO` <!-- VERIFY -->           |
| Hardware revision  | TBD                                     |
| Form factor        | TBD (e.g. 25.5 × 25.5 mm, M2 mounting)  |
| Weight             | TBD g                                   |
| Integrated ESC     | TBD (e.g. 4-in-1, BLHeli_S / AM32)      |

## 2. Specifications

### 2.1 Processor & Memory
| Item   | Value                                                   |
|--------|---------------------------------------------------------|
| MCU    | STM32H743 (Arm Cortex-M7, up to 480 MHz; ArduPilot typically clocks at 400 MHz) |
| Flash  | 2 MB internal                                           |
| RAM    | 1 MB                                                    |
| Logging| TBD (onboard SPI flash / microSD)                       |

### 2.2 Sensors
| Function   | Part | Bus / CS pin |
|------------|------|--------------|
| IMU 1      | TBD  | TBD          |
| IMU 2      | TBD  | TBD          |
| Barometer  | TBD  | TBD          |
| OSD        | TBD (e.g. AT7456E) | TBD |

### 2.3 Power
| Parameter              | Value  |
|------------------------|--------|
| Input voltage          | TBD S LiPo (TBD–TBD V) |
| BEC 5 V                | TBD A  |
| BEC 9/10 V (VTX)       | TBD A  |
| ESC continuous current | TBD A per channel |
| ESC burst current      | TBD A (TBD s) |
| Voltage sense scale    | `BATT_VOLT_MULT` = TBD |
| Current sense scale    | `BATT_AMP_PERVLT` = TBD |

---

## 3. Pinout

> All values below **must match `hwdef.dat`**. If they disagree, the hwdef is
> authoritative for firmware behavior and the schematic is authoritative for
> hardware.

### 3.1 Board Diagram
![Top view](docs/img/pinout_top.png)
![Bottom view](docs/img/pinout_bottom.png)

### 3.2 UART Mapping
| ArduPilot SERIALn | MCU UART | TX pin | RX pin | Pad label | Default protocol      |
|-------------------|----------|--------|--------|-----------|-----------------------|
| SERIAL0           | USB      | —      | —      | USB-C     | MAVLink2              |
| SERIAL1           | TBD      | TBD    | TBD    | TBD       | MAVLink2 (telemetry)  |
| SERIAL2           | TBD      | TBD    | TBD    | TBD       | RCIN (CRSF/ELRS)      |
| SERIAL3           | TBD      | TBD    | TBD    | TBD       | GPS                   |
| SERIAL4           | TBD      | TBD    | TBD    | TBD       | DisplayPort / VTX     |
| SERIAL5           | TBD      | TBD    | TBD    | TBD       | ESC telemetry         |

### 3.3 PWM / Motor Outputs
| Output | MCU pin | Timer/Channel | DShot | BiDir DShot | Group |
|--------|---------|---------------|-------|-------------|-------|
| M1     | TBD     | TBD           | ✅/❌ | ✅/❌       | 1     |
| M2     | TBD     | TBD           |       |             | 1     |
| M3     | TBD     | TBD           |       |             | 2     |
| M4     | TBD     | TBD           |       |             | 2     |
| LED    | TBD     | TBD           | —     | —           | 3 (NeoPixel) |

> Outputs in the same timer group must share a protocol (e.g. all DShot600 or all PWM).

### 3.4 Other I/O
| Function          | MCU pin | Notes                         |
|-------------------|---------|-------------------------------|
| I²C (GPS compass) | TBD/TBD | SCL / SDA                     |
| Battery voltage   | TBD     | ADC channel TBD               |
| Battery current   | TBD     | ADC channel TBD               |
| RSSI (analog)     | TBD     | optional                      |
| Buzzer            | TBD     | active-high/low TBD           |
| Status LED(s)     | TBD     |                               |
| VTX power switch  | TBD     | `RELAY`/GPIO control, if fitted |
| Boot (DFU) button | BOOT0   | Hold during power-up for DFU  |

---

## 4. Building ArduPilot Firmware

### 4.1 Prerequisites (Ubuntu 22.04/24.04)
```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile   # reload PATH (or log out and back in)
```
Other platforms: see the [ArduPilot build docs](https://ardupilot.org/dev/docs/building-the-code.html).

### 4.2 Install the Board Definition
Until the target is merged upstream, copy the hwdef from this repo:
```bash
cp -r <this-repo>/hwdef/NanoH743AIO \
      libraries/AP_HAL_ChibiOS/hwdef/
```
The directory must contain `hwdef.dat` and `hwdef-bl.dat`, and may also contain
a `defaults.parm` and a `README.md`.

### 4.3 Build
```bash
# Bootloader (required once per new board definition)
Tools/scripts/build_bootloaders.py NanoH743AIO
# Output: Tools/bootloaders/NanoH743AIO_bl.bin / .hex

# Firmware
./waf configure --board NanoH743AIO
./waf copter        # or: ./waf plane / ./waf rover
```
Artifacts are written to `build/NanoH743AIO/bin/`:

| File                       | Use                                            |
|----------------------------|------------------------------------------------|
| `arducopter.apj`           | Upload through Mission Planner / QGC (bootloader already present) |
| `arducopter_with_bl.hex`   | Full image (bootloader + firmware) for DFU/SWD |

Clean rebuild:
```bash
./waf distclean && ./waf configure --board NanoH743AIO && ./waf copter
```

### 4.4 Verifying the Pinout Against the Build
After configuring, the generated header shows the resolved pin map:
```bash
less build/NanoH743AIO/hwdef.h
```
Cross-check the UART order (`SERIAL_ORDER`), timer assignments, and ADC
scaling against Section 3 before publishing a release.

---

## 5. Flashing

### 5.1 First Flash (DFU, includes bootloader)
1. Disconnect the battery and remove propellers.
2. Hold **BOOT**, connect USB, then release. The board enumerates as
   `STM32 BOOTLOADER` (`0483:df11`).
3. Flash with **one** of these tools:

**STM32CubeProgrammer (GUI or CLI):**
```bash
STM32_Programmer_CLI -c port=USB1 -e all -w arducopter_with_bl.hex -v -rst
```

**dfu-util** (requires a `.bin` and an explicit address):
```bash
arm-none-eabi-objcopy -I ihex -O binary arducopter_with_bl.hex arducopter_with_bl.bin
dfu-util -a 0 --dfuse-address 0x08000000:leave -D arducopter_with_bl.bin
```

**Mission Planner:** Use *Install Firmware → Load custom firmware* with the
`.hex` while the board is in DFU mode (Windows only).

> **Linux:** If `dfu-util` reports permission errors, install ArduPilot's
> udev rules or run it with `sudo`.

### 5.2 Subsequent Updates (bootloader already installed)
- **Mission Planner:** *Install Firmware → Load custom firmware* → select `.apj`.
- **QGroundControl:** *Firmware → Advanced → Custom firmware file* → `.apj`.
- **CLI:**
  ```bash
  ./waf copter --upload
  # or
  python3 Tools/scripts/uploader.py build/NanoH743AIO/bin/arducopter.apj
  ```

### 5.3 ESC Firmware
TBD. Document the ESC firmware family (BLHeli_S / BLHeli_32 / AM32), the
passthrough procedure (`SERVO_BLH_AUTO = 1`), and the configurator to use.

---

## 6. First-Time Configuration
Recommended baseline parameters (also shipped as `defaults.parm`):

```
FRAME_CLASS        1        # Quad
FRAME_TYPE         1        # X
MOT_PWM_TYPE       6        # DShot600
SERVO_BLH_BDMASK   15       # Bi-directional DShot on M1–M4 (if supported)
SERIAL2_PROTOCOL   23       # RCIN
BATT_MONITOR       4        # Analog voltage + current
BATT_VOLT_MULT     TBD
BATT_AMP_PERVLT    TBD
OSD_TYPE           1        # Analog OSD (5 for DisplayPort)
NTF_LED_TYPES      TBD
```

Then complete these steps in order: accelerometer calibration, compass
calibration (if fitted), radio calibration, ESC/motor test (props off), and
failsafe configuration.

---

## 7. Thermal Specifications

### 7.1 Component Limits (datasheet)
| Component             | Limit                                     | Source |
|-----------------------|-------------------------------------------|--------|
| STM32H743 (MCU)       | T_J max 125 °C (industrial grade); confirm suffix of fitted part | ST datasheet |
| IMU                   | TBD operating range                       | datasheet |
| ESC MOSFETs           | TBD T_J max                               | datasheet |
| BEC regulators        | TBD, thermal shutdown TBD °C              | datasheet |

### 7.2 Measured Performance <!-- VERIFY: populate from test data -->
| Condition                                  | MCU °C | ESC FET hotspot °C | BEC °C | Ambient °C |
|--------------------------------------------|--------|--------------------|--------|------------|
| USB only, idle, 10 min                     | TBD    | —                  | TBD    | 25         |
| Battery, disarmed, VTX at max power, 10 min| TBD    | TBD                | TBD    | 25         |
| Bench, TBD A/ch continuous, no airflow     | TBD    | TBD                | TBD    | 25         |
| Hover, in-frame, TBD min                   | TBD    | TBD                | TBD    | TBD        |

**Test method:** Use a thermal camera (emissivity set for each surface) or a
thermocouple on the component package. Record the maximum value after
temperature stabilizes (< 1 °C change over 2 min). Log `IMU.T` and the MCU
temperature (`POWR`/`MCU` log messages) in parallel.

### 7.3 Operating Guidance
- **Operating ambient:** TBD to TBD °C.
- **Airflow:** Rated ESC current assumes prop-wash airflow. Bench tests
  without airflow should be derated to TBD %.
- **VTX heat:** Don't run a high-power VTX on the bench without airflow.
  Adjacent heat raises IMU temperature and can increase drift.
- **IMU heating:** TBD. State whether `BRD_HEAT_TARG` is supported. Most
  AIO boards have no heater.
- **Monitoring:** Use the OSD/GCS MCU-temperature display, and set a warning
  threshold of TBD °C.

---

## 8. Troubleshooting
| Symptom                               | Likely cause / fix |
|---------------------------------------|--------------------|
| Board not detected in DFU             | BOOT not held during plug-in, charge-only cable, or missing Windows driver (use Zadig → WinUSB) |
| "No IO / bootloader" after `.apj` flash | Bootloader missing → reflash `_with_bl.hex` via DFU |
| Motors don't spin with DShot          | `MOT_PWM_TYPE` mismatch within a timer group; check