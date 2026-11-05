# README.md: nano-h743-aio-flight-controller

> **Before you publish:** I don't have your board's schematic, `hwdef.dat`, or thermal test data, so I can't know its actual pin assignments, power ratings, or measured temperatures. Board-specific values below are marked **`⚠️ VERIFY`**. Replace them with data from your schematic and bench tests. Some are typical STM32H743 AIO values, but publishing an unverified pinout can lead users to wire ESCs or batteries to the wrong pads and destroy hardware. The ArduPilot build and flashing commands are standard and accurate as written.

---

```markdown
# Nano H743 AIO Flight Controller

![Board Render](docs/images/board-top.png)

[![ArduPilot](https://img.shields.io/badge/ArduPilot-4.5%2B-blue)](https://ardupilot.org)
[![License: CERN-OHL-S-2.0](https://img.shields.io/badge/Hardware-CERN--OHL--S--2.0-green)](LICENSE-HARDWARE)
[![License: GPL-3.0](https://img.shields.io/badge/Firmware-GPL--3.0-blue)](LICENSE-FIRMWARE)

A compact all-in-one (AIO) flight controller built on the STM32H743. It integrates
a flight controller, a 4-in-1 ESC (⚠️ VERIFY), and an OSD on a single 25.5 × 25.5 mm
(⚠️ VERIFY) board for sub-250 g multirotors running ArduPilot.

---

## Table of Contents

1. [Specifications](#specifications)
2. [Board Layout](#board-layout)
3. [Pinout](#pinout)
4. [UART Mapping](#uart-mapping)
5. [Motor & PWM Outputs](#motor--pwm-outputs)
6. [Building ArduPilot](#building-arduplot)
7. [Flashing](#flashing)
8. [First-Time Configuration](#first-time-configuration)
9. [Thermal Specifications](#thermal-specifications)
10. [Electrical Ratings](#electrical-ratings)
11. [Troubleshooting](#troubleshooting)
12. [Repository Structure](#repository-structure)
13. [Contributing](#contributing)
14. [License](#license)

---

## Specifications

| Parameter            | Value                                              |
|----------------------|----------------------------------------------------|
| MCU                  | STM32H743VIT6, Cortex-M7, 400 MHz (ArduPilot default), 2 MB flash, 1 MB RAM |
| IMU                  | ⚠️ VERIFY (e.g., ICM-42688-P, SPI)                 |
| Barometer            | ⚠️ VERIFY (e.g., DPS310 / BMP280, I²C)             |
| OSD                  | ⚠️ VERIFY (e.g., AT7456E, SPI)                     |
| Blackbox / Logging   | ⚠️ VERIFY (microSD via SDMMC, or SPI flash)        |
| Integrated ESC       | ⚠️ VERIFY 4-in-1, BLHeli_S / AM32, xA continuous   |
| Input voltage        | ⚠️ VERIFY (e.g., 2–4S LiPo)                        |
| BEC outputs          | ⚠️ VERIFY (e.g., 5 V / 2 A, 10 V / 2 A for VTX)    |
| UARTs                | ⚠️ VERIFY                                          |
| Mounting             | ⚠️ VERIFY (e.g., 25.5 × 25.5 mm, M2)               |
| Weight               | ⚠️ VERIFY                                          |
| Firmware             | ArduPilot (Copter / Plane / Rover)                 |

---

## Board Layout

```
                 ┌─────────────────────────────┐
     M1 ●        │  [USB-C]          [BOOT]    │        ● M2
                 │                             │
                 │   ┌─────────┐               │
     VBAT+ ■     │   │ STM32   │   [IMU]       │     ■ GND
                 │   │ H743    │               │
                 │   └─────────┘   [BARO]      │
                 │                             │
     M4 ●        │  [SD]    [LEDs: PWR/ACT]    │        ● M3
                 └─────────────────────────────┘
                     (Top view — ⚠️ VERIFY against PCB)
```

Detailed renders: [`docs/images/`](docs/images/)

---

## Pinout

> All pin assignments must match `hwdef/NanoH743AIO/hwdef.dat`. If this table and
> the hwdef ever disagree, **the hwdef is authoritative**. Please file an issue.

### Solder Pads

| Pad    | Function              | MCU Pin    | Voltage | Notes                        |
|--------|-----------------------|------------|---------|------------------------------|
| VBAT   | Battery input         | —          | 2–4S    | ⚠️ VERIFY                    |
| GND    | Ground                | —          | 0 V     |                              |
| 5V     | BEC output            | —          | 5 V     | ⚠️ VERIFY current limit      |
| 10V    | VTX BEC output        | —          | 10 V    | ⚠️ VERIFY                    |
| T1/R1  | UART1 TX/RX           | ⚠️ VERIFY  | 3.3 V   |                              |
| T2/R2  | UART2 TX/RX           | ⚠️ VERIFY  | 3.3 V   |                              |
| T3/R3  | UART3 TX/RX           | ⚠️ VERIFY  | 3.3 V   |                              |
| SDA    | I²C SDA               | ⚠️ VERIFY  | 3.3 V   | External compass             |
| SCL    | I²C SCL               | ⚠️ VERIFY  | 3.3 V   | External compass             |
| LED    | WS2812 LED data       | ⚠️ VERIFY  | 5 V     |                              |
| BZ+/BZ−| Buzzer                | ⚠️ VERIFY  | 5 V     |                              |
| CURR   | Current sensor (ADC)  | ⚠️ VERIFY  | 3.3 V   | Internal                     |
| VID    | Analog video in/out   | —          | —       | Via OSD                      |

### Internal Peripherals

| Device  | Bus  | CS / Address | Rotation         |
|---------|------|--------------|------------------|
| IMU     | SPI1 | ⚠️ VERIFY    | ⚠️ VERIFY        |
| OSD     | SPI2 | ⚠️ VERIFY    | —                |
| Baro    | I²C2 | ⚠️ VERIFY    | —                |

---

## UART Mapping

ArduPilot `SERIALn` ordering is defined by `SERIAL_ORDER` in the hwdef.

| ArduPilot Port | Hardware | Default Protocol   | Param               |
|----------------|----------|--------------------|---------------------|
| SERIAL0        | USB      | MAVLink2           | `SERIAL0_PROTOCOL=2`  |
| SERIAL1        | UART1    | ⚠️ VERIFY (MAVLink / telemetry) | `SERIAL1_PROTOCOL` |
| SERIAL2        | UART2    | ⚠️ VERIFY (RC input, e.g., ELRS/CRSF) | `SERIAL2_PROTOCOL=23` |
| SERIAL3        | UART3    | ⚠️ VERIFY (GPS)    | `SERIAL3_PROTOCOL=5`  |
| SERIAL4        | UART4    | ⚠️ VERIFY (DisplayPort / VTX) | `SERIAL4_PROTOCOL=42` |

---

## Motor & PWM Outputs

| Output | Timer/Channel | Group | DShot | BiDir DShot |
|--------|---------------|-------|-------|-------------|
| M1     | ⚠️ VERIFY     | 1     | Yes   | ⚠️ VERIFY   |
| M2     | ⚠️ VERIFY     | 1     | Yes   | ⚠️ VERIFY   |
| M3     | ⚠️ VERIFY     | 2     | Yes   | ⚠️ VERIFY   |
| M4     | ⚠️ VERIFY     | 2     | Yes   | ⚠️ VERIFY   |

> Outputs in the same timer group **must** share a protocol and rate.

Recommended ESC setup for the integrated 4-in-1:

```
MOT_PWM_TYPE      = 6     # DShot600
SERVO_BLH_AUTO    = 1
SERVO_DSHOT_ESC   = 1     # BLHeli_S/BLHeli32 (set 3 for AM32 if applicable)
SERVO_BLH_BDMASK  = 15    # Bidirectional DShot on M1–M4 (if supported)
```

---

## Building ArduPilot

### 1. Prerequisites (Ubuntu 22.04 / 24.04)

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

macOS: `Tools/environment_install/install-prereqs-mac.sh`
Windows: use WSL2 with the Ubuntu instructions.

### 2. Add the Board Definition

Until the board is merged upstream, copy the hwdef from this repo:

```bash
cp -r /path/to/nano-h743-aio-flight-controller/hwdef/NanoH743AIO \
      libraries/AP_HAL_ChibiOS/hwdef/
```

This places the following files in the ArduPilot tree:

```
libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/
├── hwdef.dat        # Main firmware definition
├── hwdef-bl.dat     # Bootloader definition
├── defaults.parm    # Board default parameters
└── README.md        # Upstream board documentation
```

### 3. Build the Bootloader

```bash
Tools/scripts/build_bootloaders.py NanoH743AIO
# Output: Tools/bootloaders/NanoH743AIO_bl.bin (+ .hex)
```

### 4. Build the Firmware

```bash
./waf configure --board NanoH743AIO
./waf copter        # or: ./waf plane / ./waf rover
```

Artifacts are written to `build/NanoH743AIO/bin/`:

| File                       | Use                                         |
|----------------------------|---------------------------------------------|
| `arducopter.apj`           | Updates via Mission Planner / QGC (bootloader present) |
| `arducopter_with_bl.hex`   | First-time flash via DFU (includes bootloader) |
| `arducopter.bin`           | Raw application image                       |

### 5. Clean Build

```bash
./waf distclean
./waf configure --board NanoH743AIO
./waf copter
```

---

## Flashing

### Method A: First Flash via DFU (blank board or recovery)

1. Hold the **BOOT** button while connecting USB-C. Release it after connecting.
2. Confirm that DFU mode is detected:
   ```bash
   dfu-util -l
   # Expect: "Found DFU: [0483:df11] ... name="@Internal Flash"
   ```
3. Flash using one of these options.

   **Option 1: STM32CubeProgrammer (recommended, accepts .hex)**
   ```bash
   STM32_Programmer_CLI -c port=USB1 -e all \
       -w build/NanoH743AIO/bin/arducopter_with_bl.hex -v -rst
   ```

   **Option 2: dfu-util (requires .bin)**
   ```bash
   # Flash the bootloader at the base of flash:
   dfu-util -a 0 --dfuse-address 0x08000000:force:mass-erase:leave \
       -D Tools/bootloaders/NanoH743AIO_bl.bin
   # Then load arducopter.apj via a GCS (Method B).
   ```

   **Option 3: Mission Planner.** Use *Install Firmware → Load custom firmware* with
   the `_with_bl.hex` file. This works only if the board is in DFU and the
   Mission Planner DFU driver is installed.

4. Unplug and replug the board without holding BOOT. The status LED should begin
   flashing within a few seconds.

### Method B: Firmware Update (ArduPilot bootloader already installed)

```bash
./waf copter --upload     # auto-detects the board over USB
```

Alternatively, in Mission Planner use *Setup → Install Firmware → Load custom firmware*
and select the `.apj` file.

### Method C: Recovery via SWD

If DFU is unreachable (for example, because option bytes are corrupted):

```bash
STM32_Programmer_CLI -c port=SWD mode=UR -e all \
    -w arducopter_with_bl.hex -v -rst
```

SWD test pads: SWDIO / SWCLK / GND / 3V3 (⚠️ VERIFY location; see `docs/images/board-bottom.png`).

### Windows DFU Driver

If `dfu-util` or Cube cannot see the device, use [Zadig](https://zadig.akeo.ie/) to
install the **WinUSB** driver for "STM32 BOOTLOADER".

---

## First-Time Configuration

1. Connect to Mission Planner or QGroundControl over USB.
2. Load the board defaults (if they are not already applied):
   *Config → Full Parameter List → Load from file → `hwdef/NanoH743AIO/defaults.parm`*.
3. Set the frame type:
   ```
   FRAME_CLASS = 1   # Quad
   FRAME_TYPE  = 1   # X
   ```
4. Calibrate the accelerometer, compass (if external), and radio.
5. Run the motor test (*Setup → Optional Hardware → Motor Test*) **with propellers removed**.
6. Configure battery monitoring:
   ```
   BATT_MONITOR    = 4
   BATT_VOLT_PIN   = ⚠️ VERIFY
   BATT_CURR_PIN   = ⚠️ VERIFY
   BATT_VOLT_MULT  = ⚠️ VERIFY (calibrate against a multimeter)
   BATT_AMP_PERVLT = ⚠️ VERIFY (calibrate against a known load)
   ```

---

## Thermal Specifications

> ⚠️ All values in this section must come from your own bench and flight testing.
> The MCU limits come from the STM32H743 datasheet. The board-level values are
> **targets** until they are measured and replaced.

### Component Limits (datasheet)

| Component            | Max Junction / Operating Temp | Source                    |
|----------------------|-------------------------------|---------------------------|
| STM32H743VIT6        | T<sub>J