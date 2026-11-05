I don't have your board's schematic, hwdef, or test data, and I'm not sure what "Cycle 28" refers to in your workflow. So this README uses the real ArduPilot build and flash workflow, but marks every board-specific value with ⚠️ **VERIFY**. Those placeholders follow common STM32H743 AIO conventions. Replace them with values from your schematic and bench tests before publishing, because a wrong pinout or current rating in a flight controller README can destroy hardware.

---

````markdown
# Nano H743 AIO Flight Controller

> Compact STM32H743-based all-in-one flight controller for ArduPilot (Copter / Plane / Rover / Sub)

![Board Revision](https://img.shields.io/badge/revision-C28-blue)
![MCU](https://img.shields.io/badge/MCU-STM32H743-green)
![Firmware](https://img.shields.io/badge/firmware-ArduPilot-orange)
![License](https://img.shields.io/badge/license-GPLv3-lightgrey)

---

## Table of Contents

1. [Overview](#overview)
2. [Specifications](#specifications)
3. [Board Layout](#board-layout)
4. [Pinout Charts](#pinout-charts)
5. [UART / Serial Mapping](#uart--serial-mapping)
6. [PWM Outputs & Timer Groups](#pwm-outputs--timer-groups)
7. [Power & Analog Sensing](#power--analog-sensing)
8. [Building ArduPilot](#building-arduplilot)
9. [Flashing Instructions](#flashing-instructions)
10. [Thermal Specifications](#thermal-specifications)
11. [First-Time Setup](#first-time-setup)
12. [Troubleshooting](#troubleshooting)
13. [Revision History](#revision-history)
14. [Safety Notice](#safety-notice)
15. [License](#license)

---

## Overview

The **Nano H743 AIO** combines an STM32H743 flight controller, an integrated
4-in-1 ESC (AIO variant), an analog/digital OSD, and an onboard barometer
on a single compact PCB. It targets 2.5"–5" multirotors and small fixed-wing
aircraft running ArduPilot.

| Item | Value |
|---|---|
| ArduPilot board name | `NanoH743AIO` ⚠️ VERIFY |
| APJ Board ID | `XXXX` ⚠️ Must be registered in `Tools/AP_Bootloader/board_types.txt` |
| Hardware revision | C28 |

---

## Specifications

### Processor & Sensors

| Component | Part | Bus | Notes |
|---|---|---|---|
| MCU | STM32H743VIT6 | — | 480 MHz Cortex-M7, 2 MB Flash, 1 MB RAM |
| IMU | ICM-42688-P ⚠️ VERIFY | SPI1 | Soft-mounted footprint |
| Barometer | DPS310 / BMP280 ⚠️ VERIFY | I2C2 | Foam-covered |
| OSD | AT7456E ⚠️ VERIFY | SPI2 | Analog OSD |
| Logging | 16 MB SPI flash / microSD ⚠️ VERIFY | SPI3 / SDMMC | |
| Oscillator | 8 MHz HSE ⚠️ VERIFY | — | |

### Electrical

| Parameter | Value |
|---|---|
| Input voltage | 2S–6S LiPo (7.4–25.2 V) ⚠️ VERIFY |
| 5 V BEC | 5 V / 2 A ⚠️ VERIFY |
| 10 V BEC (VTX) | 10 V / 2 A, switchable via GPIO ⚠️ VERIFY |
| 3.3 V LDO | 3.3 V / 250 mA (external peripherals) ⚠️ VERIFY |
| ESC continuous current | XX A per channel ⚠️ VERIFY (see [Thermal](#thermal-specifications)) |
| ESC burst current | XX A per channel, ≤10 s ⚠️ VERIFY |
| ESC firmware | BLHeli_S / Bluejay / AM32 ⚠️ VERIFY |
| ESC protocol | DShot300/600, Bidirectional DShot |

### Mechanical

| Parameter | Value |
|---|---|
| Dimensions | XX × XX mm ⚠️ VERIFY |
| Mounting pattern | 25.5 × 25.5 mm, M2 ⚠️ VERIFY |
| Weight | X.X g ⚠️ VERIFY |
| Connectors | JST-SH 1.0 mm, solder pads |

---

## Board Layout

```
                 ┌────────────────────────────────────┐
     M4 ●────────┤ [ESC4]                    [ESC1]   ├────────● M1
                 │                                    │
                 │   ┌──────┐          ┌──────────┐   │
                 │   │ BARO │          │   USB-C  │   │
                 │   └──────┘          └──────────┘   │
                 │          ┌──────────────┐          │
                 │          │  STM32H743   │  [BOOT]  │
                 │          └──────────────┘          │
                 │   ┌──────┐                         │
                 │   │ IMU  │  ▲ FWD                  │
                 │   └──────┘                         │
     M3 ●────────┤ [ESC3]                    [ESC2]   ├────────● M2
                 └──────────┬───────────────┬─────────┘
                            │  VBAT+  GND   │
```

> ⚠️ Replace with a rendered top/bottom image: `docs/img/board_top.png`, `docs/img/board_bottom.png`

---

## Pinout Charts

### Top Side Pads

| Pad | Function | MCU Pin | Voltage | Notes |
|---|---|---|---|---|
| `5V` | 5 V BEC out | — | 5 V | Shared with receiver/GPS |
| `GND` | Ground | — | 0 V | |
| `T1` / `R1` | USART1 TX/RX | PA9 / PA10 | 3.3 V | SERIAL1 – Telemetry |
| `T2` / `R2` | USART2 TX/RX | PD5 / PD6 | 3.3 V | SERIAL2 – RC Input |
| `T3` / `R3` | USART3 TX/RX | PD8 / PD9 | 3.3 V | SERIAL3 – GPS |
| `SCL` / `SDA` | I2C1 | PB8 / PB9 | 3.3 V | External compass |
| `10V` | VTX power | — | 10 V | GPIO switchable |
| `VID` | Camera video in | — | — | |
| `VTX` | Video out | — | — | |
| `LED` | WS2812 data | PA8 | 3.3 V | |
| `BZ+` / `BZ-` | Buzzer | PC15 | VBAT | Open drain |

### Bottom Side Pads

| Pad | Function | MCU Pin | Notes |
|---|---|---|---|
| `T4` / `R4` | UART4 TX/RX | PB9 / PB8 | SERIAL4 – Spare |
| `T6` / `R6` | USART6 TX/RX | PC6 / PC7 | SERIAL5 – DJI/HD VTX |
| `T7` / `R7` | UART7 TX/RX | PE8 / PE7 | SERIAL6 – ESC Telemetry |
| `CUR` | Current sense | PC1 | Internal shunt (AIO) |
| `RSSI` | Analog RSSI | PC5 | 0–3.3 V |
| `S5`–`S8` | Aux PWM | See below | Servo/LED/extra motors |

> ⚠️ **All MCU pin assignments above are illustrative placeholders.** The authoritative
> source is `hwdef/NanoH743AIO/hwdef.dat`. Keep this table generated from or
> cross-checked against that file.

### Connector: Receiver (JST-SH 4-pin)

| Pin | Signal |
|---|---|
| 1 | GND |
| 2 | 5 V |
| 3 | RX2 (SBUS/CRSF/ELRS input) |
| 4 | TX2 |

### Connector: DJI / HD VTX (JST-SH 6-pin)

| Pin | Signal |
|---|---|
| 1 | 10 V |
| 2 | GND |
| 3 | TX6 |
| 4 | RX6 |
| 5 | GND |
| 6 | SBUS (RX2 passthrough) |

---

## UART / Serial Mapping

| ArduPilot | Hardware | Default Protocol | `SERIALn_PROTOCOL` |
|---|---|---|---|
| SERIAL0 | USB (OTG1) | MAVLink2 | 2 |
| SERIAL1 | USART1 | MAVLink2 | 2 |
| SERIAL2 | USART2 | RC Input | 23 |
| SERIAL3 | USART3 | GPS | 5 |
| SERIAL4 | UART4 | None | -1 |
| SERIAL5 | USART6 | DisplayPort | 42 |
| SERIAL6 | UART7 | ESC Telemetry | 16 |

All UARTs support DMA, except where noted in `hwdef.dat`.

---

## PWM Outputs & Timer Groups

Outputs that share a timer **must** use the same protocol and rate.

| Output | MCU Pin | Timer | Group | Bi-Dir DShot |
|---|---|---|---|---|
| M1 | PB0 | TIM3_CH3 | 1 | ✅ |
| M2 | PB1 | TIM3_CH4 | 1 | ✅ |
| M3 | PA0 | TIM5_CH1 | 2 | ✅ |
| M4 | PA1 | TIM5_CH2 | 2 | ✅ |
| S5 | PA2 | TIM5_CH3 | 2 | ✅ |
| S6 | PA3 | TIM5_CH4 | 2 | ✅ |
| S7 | PD12 | TIM4_CH1 | 3 | ❌ |
| LED | PA8 | TIM1_CH1 | 4 | — (NeoPixel) |

> ⚠️ VERIFY: timers/channels must match the `PWM(n)` entries in `hwdef.dat`.

---

## Power & Analog Sensing

| Parameter | Default Value |
|---|---|
| `BATT_MONITOR` | 4 (Analog voltage and current) |
| `BATT_VOLT_PIN` | XX ⚠️ VERIFY |
| `BATT_CURR_PIN` | XX ⚠️ VERIFY |
| `BATT_VOLT_MULT` | 11.0 ⚠️ VERIFY (divider ratio) |
| `BATT_AMP_PERVLT` | XX ⚠️ Calibrate against a reference meter |

**Calibration procedure:** Fly or bench-load one pack. Compare the mAh consumed
against the mAh a charger puts back. Then adjust `BATT_AMP_PERVLT` using
`new = old × (charger_mAh / logged_mAh)`.

---

## Building ArduPilot

### 1. Environment Setup (Ubuntu 22.04 / 24.04)

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

For other platforms, see the [official build docs](https://ardupilot.org/dev/docs/building-the-code.html).

### 2. Add the Board Definition

Copy this repository's hwdef into the ArduPilot tree:

```bash
cp -r hwdef/NanoH743AIO libraries/AP_HAL_ChibiOS/hwdef/
```

Expected directory contents:

```
libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/
├── hwdef.dat        # Main firmware definition
├── hwdef-bl.dat     # Bootloader definition
├── defaults.parm    # Board default parameters
└── README.md        # (optional) wiki-style board doc
```

Minimal `hwdef.dat` header (example only):

```text
# Nano H743 AIO - hwdef.dat
MCU STM32H7xx STM32H743xx
APJ_BOARD_ID XXXX          # ⚠️ must be officially allocated
OSCILLATOR_HZ 8000000
FLASH_SIZE_KB 2048
FLASH_RESERVE_START_KB 128

SERIAL_ORDER OTG1 USART1 USART2 USART3 UART4 USART6 UART7

PA9  USART1_TX USART1
PA10 USART1_RX USART1

PB0  TIM3_CH3 TIM3 PWM(1) GPIO(50) BIDIR
PB1  TIM3_CH4 TIM3 PWM(2) GPIO(51)
```

### 3. Build the Bootloader (first time only)

```bash
./waf configure --board NanoH743AIO --bootloader
./waf bootloader
# Output: build/NanoH743AIO/bin/AP_Bootloader.bin
```

To regenerate the bootloader binaries in the tree:

```bash
Tools/scripts/build_bootloaders.py NanoH743AIO
```

### 4. Build the Firmware

```bash
./waf configure --board NanoH743AIO
./waf copter        # or: plane | rover | sub | heli
```

Output artifacts:

| File | Use |
|---|---|
| `build/NanoH743AIO/bin/arducopter.apj` | Upload via Mission Planner / QGC / uploader.py |
| `build/NanoH743AIO/bin/arducopter_with_bl.hex` | Full-chip flash (bootloader + firmware) via DFU/SWD |
| `build/NanoH743AIO/bin/arducopter.bin` | Raw binary |

### 5. Useful Build Variants

```bash
./waf configure --board NanoH743AIO --debug       # debug symbols, -Og
./waf configure --board NanoH743AIO --enable-scripting
./waf clean && ./waf copter                       # clean rebuild
```

---

## Flashing Instructions

### Method A — First Flash via DFU (blank board or recovery)

1. Hold the **BOOT** button.
2. Connect USB-C while holding the button. Release it after 2 seconds.
3. Verify that DFU mode is detected:
   ```bash
   dfu-util -l
   # Expect: Found DFU: [0483:df11] ... "@Internal Flash"
   ```
4. Flash the combined bootloader and firmware image:
   - **STM32CubeProgrammer (recommended):** Select USB → Connect → open
     `arducopter_with_bl.hex` → Download.
   - **dfu-util (with .bin):**
     ```bash
     dfu-util -a 0 --dfuse-address 0x08000000:leave \
              -D build/NanoH743AIO/bin/arducopter_with_bl.bin
     ```
     > ⚠️ Use a `_with_bl` binary if your build produces one. Otherwise, convert
     > the `.