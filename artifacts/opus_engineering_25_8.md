I don't have access to the actual `nano-h743-aio-flight-controller` repository, its schematics, or its test data. I also don't know what "Cycle 25" refers to in your project (hardware revision, release cycle, or document iteration). Rather than invent pin assignments and thermal numbers that could cause a miswired ESC or an overheated board, I've written a complete README in which:

- **ArduPilot build and flash commands are real** and match the standard upstream workflow.
- **Board-specific values** (pin mappings, current limits, thermal data) are marked `⚠️ VERIFY` or `TBD` so you can fill them in from your schematic, `hwdef.dat`, and test reports.

---

````markdown
# Nano H743 AIO Flight Controller

> Compact all-in-one STM32H743 flight controller with integrated 4-in-1 ESC,
> designed for ArduPilot (Copter / Plane / Rover).

![Status](https://img.shields.io/badge/hardware-rev%20TBD-blue)
![Firmware](https://img.shields.io/badge/ArduPilot-4.5%2B-green)
![License](https://img.shields.io/badge/license-TBD-lightgrey)

**Document revision:** Cycle 25 · **Hardware revision:** `TBD` · **Last updated:** `YYYY-MM-DD`

---

## Table of Contents

1. [Overview](#overview)
2. [Specifications](#specifications)
3. [Board Layout](#board-layout)
4. [Pinout](#pinout)
5. [UART Mapping](#uart-mapping)
6. [PWM / Motor Outputs](#pwm--motor-outputs)
7. [Power & Current Sensing](#power--current-sensing)
8. [Building ArduPilot](#building-arduplot)
9. [Flashing](#flashing)
10. [First-Time Configuration](#first-time-configuration)
11. [Thermal Specifications](#thermal-specifications)
12. [Troubleshooting](#troubleshooting)
13. [Repository Structure](#repository-structure)
14. [Contributing](#contributing)
15. [License](#license)

---

## Overview

The Nano H743 AIO combines a flight controller and a 4-in-1 ESC on a single
PCB for space-constrained multirotors (e.g., 2.5"–4" class). It runs ArduPilot
natively via a custom `hwdef` target.

**Key features**

- STM32H743 MCU @ 480 MHz, 2 MB flash, 1 MB RAM
- Onboard IMU(s): `⚠️ VERIFY` (e.g., ICM-42688-P)
- Barometer: `⚠️ VERIFY` (e.g., DPS310 / BMP280)
- OSD: `⚠️ VERIFY` (e.g., AT7456E analog OSD)
- Integrated 4-in-1 ESC: `⚠️ VERIFY` current rating, BLHeli_S / AM32 / BLHeli_32
- microSD logging (SDMMC) or onboard dataflash: `⚠️ VERIFY`
- `TBD` × UARTs, I²C, CAN (optional): `⚠️ VERIFY`

---

## Specifications

| Parameter               | Value                         | Notes                         |
|-------------------------|-------------------------------|-------------------------------|
| MCU                     | STM32H743VIH6 / VIT6          | ⚠️ VERIFY package             |
| Core clock              | 480 MHz                       | Cortex-M7                     |
| Flash / RAM             | 2 MB / 1 MB                   |                               |
| IMU                     | TBD                           | SPI bus TBD                   |
| Barometer               | TBD                           | I²C / SPI                     |
| OSD                     | TBD                           |                               |
| Blackbox                | TBD                           | microSD / SPI flash           |
| Input voltage           | TBD (e.g., 2S–6S LiPo)        | Absolute max: TBD             |
| ESC continuous current  | TBD A per motor               | See [Thermal](#thermal-specifications) |
| ESC burst current       | TBD A (TBD s)                 |                               |
| BEC outputs             | 5 V @ TBD A, 9/10 V @ TBD A   | ⚠️ VERIFY                     |
| Mounting                | TBD × TBD mm, M2 / M3         |                               |
| Dimensions              | TBD × TBD × TBD mm            |                               |
| Weight                  | TBD g                         | Without wires                 |

---

## Board Layout

```
        ┌──────────────────────────────────────┐
        │  M1 ●                          ● M2  │
        │                                      │
        │   [USB-C]       [MCU]     [BOOT]     │
        │                                      │
        │   UART pads     IMU       SD slot    │
        │                                      │
        │  M4 ●      [ BAT+ / BAT- ]     ● M3  │
        └──────────────────────────────────────┘
          ⚠️ Replace with actual top/bottom renders in /docs/img/
```

- Top view: `docs/img/top.png`
- Bottom view: `docs/img/bottom.png`

---

## Pinout

> ⚠️ **All MCU pin assignments below are placeholders.** Populate them directly
> from the schematic and keep them in sync with
> `hwdef/NanoH743AIO/hwdef.dat`. The `hwdef.dat` file is the single source of
> truth.

### Pad Pinout

| Pad label | Function             | MCU pin | Voltage | Notes                  |
|-----------|----------------------|---------|---------|------------------------|
| 5V        | 5 V BEC out          | —       | 5 V     | Max TBD A              |
| 9V / 10V  | VTX / camera supply  | —       | TBD     | Max TBD A              |
| GND       | Ground               | —       | —       |                        |
| T1 / R1   | UART1 TX / RX        | TBD     | 3.3 V   | Default: MAVLink/GPS   |
| T2 / R2   | UART2 TX / RX        | TBD     | 3.3 V   | Default: RC input      |
| T3 / R3   | UART3 TX / RX        | TBD     | 3.3 V   |                        |
| T4 / R4   | UART4 TX / RX        | TBD     | 3.3 V   |                        |
| SDA / SCL | I²C (compass)        | TBD     | 3.3 V   | External mag           |
| LED       | WS2812 data          | TBD     | 3.3 V   | NeoPixel               |
| BZ+ / BZ- | Buzzer               | TBD     | 5 V     |                        |
| CAM       | Camera video in      | —       | —       | Analog OSD             |
| VTX       | Video out            | —       | —       |                        |
| CURR      | Current sense        | TBD     | 3.3 V   | Internal ADC           |
| RSSI      | Analog RSSI          | TBD     | 3.3 V   |                        |

### Internal Peripherals

| Peripheral   | Bus   | CS / Pins | Notes        |
|--------------|-------|-----------|--------------|
| IMU          | SPI1  | TBD       | DRDY: TBD    |
| OSD          | SPI2  | TBD       |              |
| Flash / SD   | TBD   | TBD       |              |
| Barometer    | I²C2  | TBD       |              |

---

## UART Mapping

ArduPilot `SERIALn` → hardware mapping (must match `SERIAL_ORDER` in `hwdef.dat`):

| SERIALn | Hardware | Default protocol         | Parameter value |
|---------|----------|--------------------------|-----------------|
| SERIAL0 | USB      | MAVLink2                 | 2               |
| SERIAL1 | UART1    | MAVLink2                 | 2               |
| SERIAL2 | UART2    | RC Input                 | 23              |
| SERIAL3 | UART3    | GPS                      | 5               |
| SERIAL4 | UART4    | TBD                      | TBD             |
| SERIAL5 | UART5    | ESC Telemetry            | 16              |
| SERIAL6 | UART6    | DisplayPort / MSP        | 42              |

---

## PWM / Motor Outputs

| Output | Timer / Ch | Function | DShot | Bidir DShot |
|--------|------------|----------|-------|-------------|
| M1     | TBD        | Motor 1  | ✅    | ⚠️ VERIFY   |
| M2     | TBD        | Motor 2  | ✅    | ⚠️ VERIFY   |
| M3     | TBD        | Motor 3  | ✅    | ⚠️ VERIFY   |
| M4     | TBD        | Motor 4  | ✅    | ⚠️ VERIFY   |
| S5     | TBD        | Servo / LED | —  | —           |

> Outputs that share a timer must use the same protocol and rate. Document
> timer groups here once they are confirmed.

---

## Power & Current Sensing

| Parameter         | Value | Notes                              |
|-------------------|-------|------------------------------------|
| `BATT_MONITOR`    | 4     | Analog voltage + current           |
| `BATT_VOLT_PIN`   | TBD   | From hwdef                         |
| `BATT_CURR_PIN`   | TBD   | From hwdef                         |
| `BATT_VOLT_MULT`  | TBD   | Divider ratio, calibrate per unit  |
| `BATT_AMP_PERVLT` | TBD   | Shunt / sensor scale               |
| `BATT_AMP_OFFSET` | TBD   |                                    |

---

## Building ArduPilot

### 1. Prerequisites

Ubuntu 22.04 or 24.04 is recommended. WSL2 also works.

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

If you cloned without submodules:

```bash
git submodule update --init --recursive
```

### 2. Add the Board Definition

Copy this repo's hwdef into the ArduPilot tree:

```bash
cp -r /path/to/nano-h743-aio-flight-controller/hwdef/NanoH743AIO \
      libraries/AP_HAL_ChibiOS/hwdef/
```

The expected structure is:

```
libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/
├── hwdef.dat        # Main firmware definition
├── hwdef-bl.dat     # Bootloader definition
├── defaults.parm    # Default parameters (optional)
└── README.md        # Upstream board docs (optional)
```

> A `board_id` / `APJ_BOARD_ID` must be allocated in
> `Tools/AP_Bootloader/board_types.txt` before upstreaming.
> Current ID: `TBD`.

### 3. Build the Bootloader

```bash
Tools/scripts/build_bootloaders.py NanoH743AIO
```

Output: `Tools/bootloaders/NanoH743AIO_bl.bin` (and `.hex`)

### 4. Build Firmware

```bash
./waf configure --board NanoH743AIO
./waf copter        # or: plane, rover, heli
```

Build artifacts are written to `build/NanoH743AIO/bin/`:

| File                     | Use                                   |
|--------------------------|---------------------------------------|
| `arducopter.apj`         | Upload via Mission Planner / QGC      |
| `arducopter.bin`         | Raw app binary                        |
| `arducopter_with_bl.hex` | Full image (bootloader + app) for DFU |

### 5. Optional Build Flags

```bash
./waf configure --board NanoH743AIO --debug          # Debug symbols
./waf configure --board NanoH743AIO --enable-scripting
./waf clean                                          # Clean current board
./waf distclean                                      # Full reset
```

---

## Flashing

### Method A: First Flash via DFU (Bootloader + Firmware)

Use this for a blank board or a board that currently runs Betaflight or INAV.

1. Hold the **BOOT** button and connect USB. Release the button after the
   board enumerates.
2. Confirm that DFU mode is active:

   ```bash
   dfu-util -l
   # Expect: "Found DFU: [0483:df11] ... name="@Internal Flash  /0x08000000/..."
   ```

3. Flash the full image.

   **STM32CubeProgrammer (recommended for `.hex`):**

   ```bash
   STM32_Programmer_CLI -c port=USB1 -e all \
       -w build/NanoH743AIO/bin/arducopter_with_bl.hex -v -rst
   ```

   **dfu-util (convert to `.bin` first if needed):**

   ```bash
   dfu-util -a 0 -s 0x08000000:mass-erase:force:leave \
       -D NanoH743AIO_with_bl.bin
   ```

   **GUI options:** STM32CubeProgrammer (USB mode) or the Betaflight
   Configurator flash tab with "Full chip erase" and a local `.hex` file.

4. Power-cycle the board. It should now enumerate as an ArduPilot device.

> **Windows:** If the board is not detected in DFU mode, install the WinUSB
> driver with [Zadig](https://zadig.akeo.ie/).

### Method B: Firmware Update (ArduPilot Bootloader Already Present)

Use one of these options:

- **waf upload:**

  ```bash
  ./waf copter --upload
  ```

- **Mission Planner:** *Setup → Install Firmware → Load custom firmware* →
  select `arducopter.apj`.
- **QGroundControl:** *Vehicle Setup → Firmware → Advanced → Custom firmware
  file*.

### Method C: SWD (Recovery / Development)

Connect an ST-Link to the SWDIO, SWCLK, GND, and 3V3 pads (`⚠️ VERIFY` pad
locations), then run:

```bash
STM32_Programmer_CLI -c port=SWD -e all -w arducopter_with_bl.hex -v -rst
```

### Updating the Bootloader Only

From a running ArduPilot install, set the following parameter and reboot:

```
FLASH_BOOTLOADER = 1
```

---

## First-Time Configuration

1. Load defaults: *Config → Full Parameter List → Reset to Defaults*.
2. Set the frame: `FRAME_CLASS = 1` (Quad), `FRAME_TYPE = 1` (X).
3. Configure ESCs:

   ```
   MOT_PWM_TYPE      = 6    # DShot600 (or 5 = DShot300)
   SERVO_BLH_AUTO    = 1
   SERVO_BLH_BDMASK  = 15   # Bidirectional DShot on M1–M4 (if supported)
   ```

4. Run the calibrations: accelerometer, compass (if fitted), radio, and
   battery voltage and current.
5. Check motor order and direction with Mission Planner's *Motor Test*.
