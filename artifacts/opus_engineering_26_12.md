I don't have access to your repository's schematic, `hwdef.dat`, or test data, so this README is built on real STM32H743 and ArduPilot conventions with clearly marked placeholders (`⚠️ VERIFY`, `<!-- TODO -->`) wherever board-specific values are needed. Replace those with values from your schematic and bench measurements before publishing. Pin assignments shown are a self-consistent reference layout, not a description of your actual hardware.

---

````markdown
# Nano H743 AIO Flight Controller

[![Firmware](https://img.shields.io/badge/firmware-ArduPilot-blue)](https://ardupilot.org)
[![MCU](https://img.shields.io/badge/MCU-STM32H743-03234B)](https://www.st.com/en/microcontrollers-microprocessors/stm32h743-753.html)
[![License](https://img.shields.io/badge/license-GPLv3-green)](LICENSE)
[![Hardware Rev](https://img.shields.io/badge/hw%20rev-<!--TODO-->-orange)](#hardware-revisions)

A compact all-in-one (AIO) flight controller built around the STM32H743 with an integrated
4-in-1 ESC, designed for 2.5"–3.5" multirotors running ArduPilot.

> **Safety notice:** Always remove propellers when flashing, configuring, or bench-testing.
> The integrated ESC can spin motors as soon as battery power is applied and the vehicle is armed.

---

## Table of Contents

1. [Specifications](#specifications)
2. [Board Layout](#board-layout)
3. [Pinout](#pinout)
4. [UART Mapping](#uart-mapping)
5. [Building ArduPilot](#building-ardupilot)
6. [Flashing](#flashing)
7. [First-Time Configuration](#first-time-configuration)
8. [Thermal Specifications](#thermal-specifications)
9. [Electrical Ratings](#electrical-ratings)
10. [Troubleshooting](#troubleshooting)
11. [Hardware Revisions](#hardware-revisions)
12. [Contributing](#contributing)
13. [License](#license)

---

## Specifications

| Category          | Specification                                               |
|-------------------|-------------------------------------------------------------|
| MCU               | STM32H743VIT6, Arm Cortex-M7 @ 480 MHz, 2 MB flash, 1 MB RAM |
| IMU               | <!-- TODO: e.g. ICM-42688-P --> (SPI)                       |
| Barometer         | <!-- TODO: e.g. DPS310 / BMP280 --> (I²C)                   |
| OSD               | <!-- TODO: AT7456E (analog) or none -->                     |
| Blackbox / Logging| <!-- TODO: microSD / 16 MB SPI flash -->                    |
| Integrated ESC    | 4-in-1, <!-- TODO --> A continuous / <!-- TODO --> A burst per channel |
| ESC Firmware      | <!-- TODO: BLHeli_S / Bluejay / AM32 -->                    |
| Motor Protocol    | DShot150/300/600, bidirectional DShot supported             |
| Input Voltage     | 2S–<!-- TODO -->S LiPo (<!-- TODO --> V – <!-- TODO --> V)  |
| BEC Outputs       | 5 V @ <!-- TODO --> A, 10 V @ <!-- TODO --> A (VTX)          |
| UARTs             | <!-- TODO --> available                                     |
| Mounting          | 25.5 × 25.5 mm (M2) <!-- ⚠️ VERIFY -->                       |
| Dimensions        | <!-- TODO --> × <!-- TODO --> × <!-- TODO --> mm            |
| Weight            | <!-- TODO --> g (without wires)                              |
| Connector         | USB-C                                                       |

---

## Board Layout

```
                         FRONT (arrow on silkscreen)
        ┌───────────────────────────────────────────────┐
        │ (M2)                                     (M2) │
        │  M4 ●                                   ● M2  │
        │                                               │
        │   [BOOT]      ┌───────────────┐    [LED]      │
        │               │  STM32H743    │               │
        │   T1 R1       │               │    T2 R2      │
        │   5V GND      └───────────────┘    SDA SCL    │
        │                                               │
        │   VTX 10V     [IMU]   [BARO]       RX3 TX3    │
        │                                               │
        │  M3 ●          [USB-C]                  ● M1  │
        │ (M2)          BAT+   BAT-               (M2)  │
        └───────────────────────────────────────────────┘
                              REAR
```
<!-- TODO: Replace ASCII art with a rendered top/bottom image: docs/img/top.png, docs/img/bottom.png -->

---

## Pinout

> ⚠️ **VERIFY** every row against the schematic and `hwdef.dat` before release.
> The table below must match `libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/hwdef.dat` exactly.

### Motor Outputs

| Output | MCU Pin | Timer / Channel | DMA | Bidir DShot | Default Function |
|--------|---------|-----------------|-----|-------------|------------------|
| M1     | PB0     | TIM3_CH3        | Yes | Yes         | Motor 1          |
| M2     | PB1     | TIM3_CH4        | Yes | Yes         | Motor 2          |
| M3     | PA0     | TIM5_CH1        | Yes | Yes         | Motor 3          |
| M4     | PA1     | TIM5_CH2        | Yes | Yes         | Motor 4          |
| LED    | PA8     | TIM1_CH1        | Yes | —           | NeoPixel / WS2812 |

Outputs sharing a timer must use the same protocol/rate. Group 1: M1–M2 (TIM3); Group 2: M3–M4 (TIM5).

### Solder Pads

| Pad      | Function                        | Notes                          |
|----------|---------------------------------|--------------------------------|
| BAT+     | Battery positive                | Main power input               |
| BAT−     | Battery ground                  |                                |
| 5V       | 5 V BEC output                  | Receiver, GPS, peripherals     |
| 10V      | 10 V BEC output                 | VTX / camera; enable via GPIO? <!-- TODO --> |
| 4V5      | USB-powered 5 V rail            | Powers RX/GPS when on USB only |
| GND      | Ground                          | Multiple pads                  |
| T1 / R1  | UART1 TX / RX                   |                                |
| T2 / R2  | UART2 TX / RX                   |                                |
| T3 / R3  | UART3 TX / RX                   |                                |
| T4 / R4  | UART4 TX / RX                   |                                |
| T6 / R6  | UART6 TX / RX                   |                                |
| SDA / SCL| I²C1                            | External compass               |
| CURR     | Current sensor (internal)       | ESC current to ADC             |
| VBAT     | Battery voltage sense           | ADC, internal divider          |
| CAM      | Analog camera video in          | If analog OSD fitted           |
| VTX      | Video out to VTX                | If analog OSD fitted           |
| BZ+ / BZ−| Buzzer                          | Active buzzer, 5 V             |
| BOOT     | Bootloader button               | Hold during USB plug-in for DFU |

### Internal Bus Assignments

| Device   | Bus   | CS / Address | Data-Ready / IRQ | Notes            |
|----------|-------|--------------|------------------|------------------|
| IMU      | SPI1  | PA4          | <!-- TODO -->     | <!-- TODO -->    |
| OSD      | SPI2  | PB12         | —                | If fitted        |
| Flash    | SPI3  | PA15         | —                | Dataflash logging|
| Baro     | I²C2  | 0x76 / 0x77  | —                | <!-- TODO -->    |

### ADC

| Signal | MCU Pin | Scale Parameter                  | Default Value      |
|--------|---------|----------------------------------|--------------------|
| VBAT   | PC0     | `BATT_VOLT_MULT`                 | <!-- TODO -->      |
| CURR   | PC1     | `BATT_AMP_PERVLT`                | <!-- TODO -->      |
| RSSI   | PC5     | `RSSI_ANA_PIN`                   | —                  |

---

## UART Mapping

| ArduPilot `SERIALn` | Hardware | Default Protocol            | Default Baud | Notes                         |
|---------------------|----------|-----------------------------|--------------|-------------------------------|
| SERIAL0             | USB      | MAVLink2 (2)                | 115200       | Console                       |
| SERIAL1             | UART1    | MAVLink2 (2)                | 57600        | Telemetry radio               |
| SERIAL2             | UART2    | RCIN (23)                   | 115200       | CRSF/ELRS/SBUS receiver       |
| SERIAL3             | UART3    | GPS (5)                     | 230400 (auto)| GPS + compass on I²C          |
| SERIAL4             | UART4    | ESC Telemetry (16) / None   | 115200       | Not needed with bidir DShot   |
| SERIAL5             | UART6    | DisplayPort (42) / VTX (37) | 115200       | HD VTX or SmartAudio/Tramp    |

> SBUS requires an inverted input. On the H7 the UART can invert in hardware, so SBUS works on any
> UART RX with `SERIALn_OPTIONS` bit 0 set (handled automatically when `SERIALn_PROTOCOL=23`).

---

## Building ArduPilot

### 1. Prerequisites

Tested on Ubuntu 22.04 LTS. For Windows, use WSL2. For macOS, see the
[ArduPilot macOS guide](https://ardupilot.org/dev/docs/building-setup-mac.html).

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

### 2. Add the Board Definition

Copy this repo's hardware definition into the ArduPilot tree:

```bash
cp -r /path/to/nano-h743-aio-flight-controller/hwdef/NanoH743AIO \
      libraries/AP_HAL_ChibiOS/hwdef/
```

Expected layout:

```
libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/
├── hwdef.dat         # Main application hardware definition
├── hwdef-bl.dat      # Bootloader hardware definition
├── defaults.parm     # Board default parameters
└── README.md         # Board docs (shown on ardupilot.org if upstreamed)
```

> Each new board needs a unique `APJ_BOARD_ID`. Request one by PR to
> `Tools/AP_Bootloader/board_types.txt` before distributing firmware. Do **not** reuse another board's ID.

### 3. Build the Bootloader

```bash
./waf configure --board NanoH743AIO --bootloader
./waf bootloader
# Output: build/NanoH743AIO/bin/AP_Bootloader.bin
```

To embed the bootloader in the main firmware tree:

```bash
Tools/scripts/build_bootloaders.py NanoH743AIO
# Output: Tools/bootloaders/NanoH743AIO_bl.bin (+ .hex)
```

### 4. Build the Vehicle Firmware

```bash
./waf configure --board NanoH743AIO
./waf copter          # Multirotor
# ./waf plane         # Fixed-wing
# ./waf rover         # Rover / boat
```

Artifacts in `build/NanoH743AIO/bin/`:

| File                       | Use                                              |
|----------------------------|--------------------------------------------------|
| `arducopter.apj`           | Upload via Mission Planner / QGC / `uploader.py` |
| `arducopter_with_bl.hex`   | First flash via DFU (bootloader + firmware)      |
| `arducopter.bin`           | Raw application image                            |

### 5. Useful Build Options

```bash
./waf configure --board NanoH743AIO --debug       # Debug symbols, -Og
./waf configure --board NanoH743AIO --enable-scripting
./waf clean                                        # Clean current board
./waf distclean                                    # Full reset (re-run configure after)
```

### 6. Reference `hwdef.dat` Excerpt

```text
# ⚠️ VERIFY: must match the pinout tables above
include ../include/minimize_fpv_osd.inc   # optional, for flash-constrained builds

MCU STM32H7xx STM32H743xx
APJ_BOARD_ID TODO_ASSIGNED_ID
OSCILLATOR_HZ 8000000
FLASH_SIZE_KB 2048
FLASH_RESERVE_START_KB 128

# USB
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1
PA9  VBUS INPUT OPENDRAIN

# Serial order: SERIAL0..SERIAL5
SERIAL_ORDER OTG1 USART1 USART2 USART3 UART4 USART6

# Motors
PB0 TIM3_CH3 TIM3 PWM(1) GPIO(50) BIDIR
PB1 TIM3_CH4 TIM3 PWM(2) GPIO(51)
PA0 TIM5_CH1 TIM5 PWM(3) GPIO(52) BIDIR
PA1 TIM5_CH2 TIM5 PWM(4) GPIO(53)
PA8 TIM1_CH1 TIM1 PWM(5) GPIO(54)    # LED

# Battery
PC0 BATT_VOLTAGE_SENS ADC1 SCALE(1)
PC1 BATT_CURRENT_SENS ADC1 SCALE(1)
define HAL_BATT_MONITOR_DEFAULT 4
define HAL_BATT_VOLT_SCALE TODO
define HAL_BATT_CURR_SCALE TODO

define HAL_FRAME_TYPE_DEFAULT 12     # BetaFlightX motor order
```

---

## Flashing

### Method A — First Flash via DFU (blank board or recovering a bricked board)

1. Hold the **BOOT** button.
2. Connect USB-C while still holding BOOT; release after ~1 s.
3. Confirm the device is in DFU mode:

   ```bash
   dfu-util -l
   # Expect: Found DFU: [0483:df11] ... name="@Internal Flash  /0x08000000/..."
   ```

4. Flash bootloader + firmware in one step:

   **Linux / macOS (dfu-util):**
   ```bash
   # Convert hex to bin if needed, or flash the bootloader-only bin then upload .apj:
   dfu-util -a 0 --dfuse-address 0x08000000:leave \
            -D Tools/bootloaders/NanoH743AIO_bl.bin
   ```

   **Any OS (STM32CubeProgrammer):**
   - Connection: **USB**, click **Connect**
   - Open `arducopter_with_bl.hex`, click **Download**
   - Disconnect and power-cycle

   **Windows (Mission Planner):** If DFU drivers are missing, install them with
   