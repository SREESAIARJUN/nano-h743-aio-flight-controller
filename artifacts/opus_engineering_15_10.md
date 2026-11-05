I don't have the schematic, `hwdef.dat`, or earlier cycles for `nano-h743-aio-flight-controller`, so I can't confirm its actual pin assignments or thermal measurements. The README below is complete in structure, and its ArduPilot build and flash procedures are accurate. Board-specific values are marked **⚠️ VERIFY** and filled with plausible STM32H743 AIO defaults. Replace them with your real data before publishing, because a wrong pinout or voltage rating can destroy hardware. If you paste your `hwdef.dat` and schematic net list, I can fill in exact values.

---

````markdown
# Nano H743 AIO Flight Controller

![Board](docs/img/board-top.png)

[![ArduPilot](https://img.shields.io/badge/ArduPilot-4.5%2B-blue)](https://ardupilot.org)
[![License: CERN-OHL-S-2.0](https://img.shields.io/badge/HW%20License-CERN--OHL--S--2.0-green)](LICENSE-HARDWARE)
[![License: GPL-3.0](https://img.shields.io/badge/FW%20License-GPL--3.0-green)](LICENSE)

A compact all-in-one (AIO) flight controller built around the **STM32H743** MCU.
It has an integrated 4-in-1 ESC, OSD, and Blackbox, and is designed for 2"–4" multirotors running **ArduPilot**.

> **Status:** Hardware rev **⚠️ VERIFY (e.g., v1.2)** · Firmware target `NanoH743AIO`

---

## Table of Contents
1. [Specifications](#specifications)
2. [Board Layout](#board-layout)
3. [Pinout](#pinout)
4. [UART Mapping](#uart-mapping)
5. [PWM / Motor Outputs](#pwm--motor-outputs)
6. [Analog & Power Sensing](#analog--power-sensing)
7. [Building ArduPilot](#building-arducopter-firmware)
8. [Flashing](#flashing)
9. [First-Boot Configuration](#first-boot-configuration)
10. [Thermal Specifications](#thermal-specifications)
11. [Troubleshooting](#troubleshooting)
12. [Repository Structure](#repository-structure)
13. [Contributing & License](#contributing--license)

---

## Specifications

| Parameter          | Value |
|--------------------|-------|
| MCU                | STM32H743VIT6, Cortex-M7 @ 480 MHz, 2 MB Flash, 1 MB RAM |
| IMU                | ⚠️ VERIFY: ICM-42688-P (SPI1) |
| Barometer          | ⚠️ VERIFY: DPS310 / BMP280 (I2C2) |
| OSD                | ⚠️ VERIFY: AT7456E (SPI2) |
| Blackbox           | ⚠️ VERIFY: 16 MB SPI NOR flash (W25Q128) or microSD |
| Integrated ESC     | ⚠️ VERIFY: 4-in-1, 20 A continuous / 25 A burst per channel, BLHeli_S / Bluejay |
| ESC protocol       | DShot150/300/600, bidirectional DShot supported |
| Input voltage      | ⚠️ VERIFY: 2S–4S LiPo (7.4–16.8 V) |
| BEC outputs        | ⚠️ VERIFY: 5 V / 2 A, 3.3 V / 500 mA (+ 10 V / 2 A VTX if fitted) |
| UARTs              | ⚠️ VERIFY: 6 usable |
| Current sensor     | Onboard shunt, ⚠️ VERIFY scale |
| Mounting           | ⚠️ VERIFY: 25.5 × 25.5 mm, M2 |
| Dimensions         | ⚠️ VERIFY: 33 × 33 × 6 mm |
| Weight             | ⚠️ VERIFY: ~6.5 g |
| USB                | USB-C (FS, DFU-capable) |

---

## Board Layout

```
            ┌──────────────────────────────────┐
     M4 ●───┤ M4                           M2  ├───● M2
            │   [BOOT]                         │
            │          ┌──────────┐            │
            │          │ STM32H743│   [IMU]    │
            │          └──────────┘            │
            │  [OSD]          [BARO]  [FLASH]  │
            │                                  │
     M3 ●───┤ M3                [USB-C]   M1   ├───● M1
            └───────────┬──────────┬───────────┘
                     BAT+        BAT−
```
*Top view. Arrow on silkscreen = forward. ⚠️ VERIFY against your PCB render.*

---

## Pinout

### Solder Pads

| Pad   | Function                  | MCU Pin | Notes |
|-------|---------------------------|---------|-------|
| BAT+  | Battery positive          | —       | Main power input |
| BAT−  | Ground                    | —       | |
| 5V    | 5 V BEC out               | —       | Receiver / GPS |
| 9V/10V| VTX supply                | —       | ⚠️ VERIFY, switchable via PINIO1 |
| 3V3   | 3.3 V out                 | —       | Low-current only |
| GND   | Ground                    | —       | Multiple pads |
| T1/R1 | UART1 TX/RX               | PA9/PA10 | ⚠️ VERIFY |
| T2/R2 | UART2 TX/RX               | PD5/PD6  | ⚠️ VERIFY |
| T3/R3 | UART3 TX/RX               | PD8/PD9  | ⚠️ VERIFY |
| T4/R4 | UART4 TX/RX               | PB9/PB8  | ⚠️ VERIFY |
| T6/R6 | USART6 TX/RX              | PC6/PC7  | ⚠️ VERIFY |
| T7/R7 | UART7 TX/RX               | PE8/PE7  | ⚠️ VERIFY |
| SDA/SCL | I2C (external compass) | PB11/PB10 | ⚠️ VERIFY |
| LED   | WS2812 strip              | PA8      | ⚠️ VERIFY |
| BZ+/BZ− | Buzzer                  | PA15     | ⚠️ VERIFY |
| CAM   | Analog camera video in    | —        | To OSD |
| VTX   | Video out                 | —        | From OSD |
| CURR  | External current sense in | —        | Unused when onboard ESC sensor active |
| RSSI  | Analog RSSI               | PC5      | ⚠️ VERIFY |

### Internal Bus Assignments

| Bus  | Device   | CS Pin | DRDY/INT |
|------|----------|--------|----------|
| SPI1 | IMU      | ⚠️ PA4 | ⚠️ PB2 |
| SPI2 | OSD      | ⚠️ PB12 | — |
| SPI3 | Flash    | ⚠️ PA15 | — |
| I2C2 | Baro     | —       | — |

---

## UART Mapping

| ArduPilot `SERIALn` | Hardware | Default Protocol | Default Use |
|---------------------|----------|------------------|-------------|
| SERIAL0 | USB      | MAVLink2 (2)   | GCS |
| SERIAL1 | UART1    | MAVLink2 (2)   | Telemetry |
| SERIAL2 | UART2    | RCIN (23)      | Receiver (CRSF/ELRS/SBUS) |
| SERIAL3 | UART3    | GPS (5)        | GPS |
| SERIAL4 | UART4    | MSP DisplayPort (42) | HD VTX |
| SERIAL5 | USART6   | ESC Telem (16) | 4-in-1 ESC telemetry |
| SERIAL6 | UART7    | None (−1)      | Spare |

⚠️ VERIFY that this matches `SERIAL_ORDER` in `hwdef.dat`.

> **SBUS:** If inversion is supported on the RX pad (H7 USARTs support `SERIALn_OPTIONS=1`), SBUS can connect directly.

---

## PWM / Motor Outputs

| Output | Pad | Timer   | DMA | Group |
|--------|-----|---------|-----|-------|
| SERVO1 | M1  | ⚠️ TIM3_CH3 | ✔ | 1 |
| SERVO2 | M2  | ⚠️ TIM3_CH4 | ✔ | 1 |
| SERVO3 | M3  | ⚠️ TIM5_CH1 | ✔ | 2 |
| SERVO4 | M4  | ⚠️ TIM5_CH2 | ✔ | 2 |
| SERVO5 | LED | ⚠️ TIM1_CH1 | ✔ | 3 (NeoPixel) |

Outputs in the same timer group must use the same protocol and rate.
Bidirectional DShot requires `SERVO_BLH_BDMASK` and firmware built with the `-bdshot` variant, or a `hwdef` that enables `HAL_WITH_BIDIR_DSHOT`.

---

## Analog & Power Sensing

| Parameter        | Default |
|------------------|---------|
| `BATT_MONITOR`   | 4 (Analog V & I) |
| `BATT_VOLT_PIN`  | ⚠️ VERIFY (e.g., 10) |
| `BATT_CURR_PIN`  | ⚠️ VERIFY (e.g., 11) |
| `BATT_VOLT_MULT` | ⚠️ VERIFY (e.g., 11.0) |
| `BATT_AMP_PERVLT`| ⚠️ VERIFY (e.g., 40.0) |

**Calibrate** voltage against a multimeter, and current against a known load, before relying on failsafes.

---

## Building ArduCopter Firmware

### 1. Prerequisites (Ubuntu 22.04 / 24.04)

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

macOS: `Tools/environment_install/install-prereqs-mac.sh`. Windows: use WSL2.

### 2. Add the Board Definition

```bash
cp -r <this-repo>/hwdef/NanoH743AIO libraries/AP_HAL_ChibiOS/hwdef/
```

The directory must contain:
- `hwdef.dat` (main firmware)
- `hwdef-bl.dat` (bootloader)
- `defaults.parm` (optional board defaults)
- `README.md` (optional, shown on ardupilot.org)

> The board ID (`APJ_BOARD_ID`) must be unique. Request one via a PR to `Tools/AP_Bootloader/board_types.txt` before upstreaming.

### 3. Build the Bootloader (first time / new hardware)

```bash
./waf configure --board NanoH743AIO --bootloader
./waf bootloader
cp build/NanoH743AIO/bin/AP_Bootloader.bin Tools/bootloaders/NanoH743AIO_bl.bin
```

Alternatively, run `Tools/scripts/build_bootloaders.py NanoH743AIO`.

### 4. Build the Vehicle Firmware

```bash
./waf configure --board NanoH743AIO
./waf copter        # or: plane, rover, sub, heli
```

Artifacts are written to `build/NanoH743AIO/bin/`:

| File | Use |
|------|-----|
| `arducopter.apj` | Upload via Mission Planner / QGC / `--upload` (requires bootloader) |
| `arducopter.bin` | Raw application image |
| `arducopter_with_bl.hex` | Bootloader + firmware, for blank boards via DFU / STM32CubeProgrammer |

### 5. Optional Build Flags

```bash
./waf configure --board NanoH743AIO --debug          # debug symbols
./waf configure --board NanoH743AIO --enable-scripting
./waf clean && ./waf copter                          # clean rebuild
```

To build a custom feature set, use `Tools/scripts/build_options.py` or the [Custom Firmware Server](https://custom.ardupilot.org).

---

## Flashing

### A. Board Already Has the ArduPilot Bootloader (normal case)

```bash
./waf copter --upload
```

Or in Mission Planner: **Setup → Install Firmware → Load custom firmware** → select `arducopter.apj`.

### B. Blank Board / Betaflight / INAV → ArduPilot (DFU)

1. Unplug the battery.
2. Hold the **BOOT** button and connect USB-C. The board enumerates as `STM32 BOOTLOADER` (`0483:df11`).
3. Flash using one of the following methods:

**STM32CubeProgrammer (recommended for `.hex`)**
```bash
STM32_Programmer_CLI -c port=USB1 -w arducopter_with_bl.hex -v -rst
```

**dfu-util (requires `.bin`, flashes bootloader only)**
```bash
dfu-util -a 0 -d 0483:df11 -s 0x08000000:mass-erase:force:leave \
         -D Tools/bootloaders/NanoH743AIO_bl.bin
```
After this step, power-cycle the board and upload `arducopter.apj` via method A.

**Betaflight Configurator:** Load `arducopter_with_bl.hex` with **Full Chip Erase** enabled. This works on most H7 boards.

> **Linux permissions:** If DFU is not detected, add a udev rule:
> ```bash
> echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="df11", MODE="0666"' \
>   | sudo tee /etc/udev/rules.d/50-stm32-dfu.rules && sudo udevadm control --reload
> ```

### C. Reverting to Betaflight

Enter DFU as described above, then flash the Betaflight target with Full Chip Erase. ⚠️ VERIFY that a matching Betaflight target exists.

---

## First-Boot Configuration

1. Connect to Mission Planner or QGroundControl over USB.
2. Set **Frame Class/Type** (`FRAME_CLASS=1`, `FRAME_TYPE=1` for Quad X).
3. Run **Accelerometer** and **Compass** calibration (if an external compass is fitted).
4. Set the ESC protocol:
   ```
   MOT_PWM_TYPE      6      # DShot600
   SERVO_BLH_AUTO    1
   SERVO_BLH_BDMASK  15     # bidirectional DShot on M1-M4 (if supported)
   SERVO_DS