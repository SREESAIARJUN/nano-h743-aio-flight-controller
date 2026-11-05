# Cycle 1 Draft: README.md

> **Before you publish:** I don't have access to this board's schematic, `hwdef.dat`, or test data. ArduPilot build and flashing commands below follow real upstream conventions. Anything that depends on the specific hardware (pin assignments, timers, current sensor scale, thermal measurements) is marked **`⚠️ VERIFY`** or `TBD`. Replace those values from the schematic and bench testing. Unverified pinouts in a flight-controller README can cause wrong wiring and damaged hardware.

---

````markdown
# Nano H743 AIO Flight Controller

![Board Render](docs/img/board-top.png)

[![ArduPilot](https://img.shields.io/badge/ArduPilot-4.5%2B-blue)](https://ardupilot.org)
[![License: GPL-3.0](https://img.shields.io/badge/License-GPLv3-green.svg)](LICENSE)
[![Hardware Rev](https://img.shields.io/badge/HW%20Rev-1.0-orange)](#revision-history)

A compact all-in-one (AIO) flight controller built on the **STM32H743** (480 MHz Cortex-M7). It integrates a 4-in-1 ESC, OSD, and barometer. It is designed for 2"–4" multirotors running **ArduPilot**.

---

## Table of Contents

1. [Specifications](#specifications)
2. [Board Layout & Pinout](#board-layout--pinout)
3. [UART Mapping](#uart-mapping)
4. [PWM / Motor Outputs](#pwm--motor-outputs)
5. [Analog & Power Monitoring](#analog--power-monitoring)
6. [Building ArduPilot](#building-ardupilot)
7. [Flashing Firmware](#flashing-firmware)
8. [First-Time Configuration](#first-time-configuration)
9. [Thermal Specifications](#thermal-specifications)
10. [Electrical Ratings](#electrical-ratings)
11. [Mechanical](#mechanical)
12. [Troubleshooting](#troubleshooting)
13. [Revision History](#revision-history)
14. [License](#license)

---

## Specifications

| Parameter | Value |
|---|---|
| MCU | STM32H743VIT6 / VIH6, 480 MHz, 2 MB Flash, 1 MB RAM |
| IMU | `TBD` (e.g., ICM-42688-P) on SPI1 ⚠️ VERIFY |
| Barometer | `TBD` (e.g., DPS310 / BMP280) on I2C/SPI ⚠️ VERIFY |
| OSD | AT7456E (analog) on SPI ⚠️ VERIFY |
| Blackbox / Logging | `TBD` MB SPI flash **or** microSD |
| Integrated ESC | 4-in-1, `TBD` A continuous / `TBD` A burst, BLHeli_S / Bluejay / AM32 ⚠️ VERIFY |
| Input Voltage | 2S–`TBD`S LiPo (`TBD`–`TBD` V) |
| BEC Outputs | 5 V @ `TBD` A, 10 V @ `TBD` A (VTX), 3.3 V @ `TBD` mA |
| UARTs | `TBD` |
| Motor Outputs | 4 (ESC) + `TBD` aux PWM |
| Mounting | `TBD` × `TBD` mm, M2 |
| Dimensions | `TBD` × `TBD` mm |
| Weight | `TBD` g |

---

## Board Layout & Pinout

![Top Pinout](docs/img/pinout-top.png)
![Bottom Pinout](docs/img/pinout-bottom.png)

> ⚠️ **All tables below must match `hwdef.dat` and the production schematic.** If they disagree, the hwdef file is authoritative for firmware behavior.

### Solder Pad Reference

| Pad Label | Function | MCU Pin | Voltage Level | Notes |
|---|---|---|---|---|
| `BAT+` | Battery input | — | VBAT | Main power, feeds ESC + BECs |
| `GND` | Ground | — | 0 V | Multiple pads |
| `5V` | 5 V BEC out | — | 5.0 V | Receiver, GPS |
| `10V` | 10 V BEC out | — | 10.0 V | VTX / camera; switchable? ⚠️ |
| `3V3` | 3.3 V out | — | 3.3 V | Low-current peripherals only |
| `T1` / `R1` | UART1 TX/RX | `TBD` | 3.3 V | — |
| `T2` / `R2` | UART2 TX/RX | `TBD` | 3.3 V | — |
| `SDA` / `SCL` | I2C (ext. compass) | `TBD` | 3.3 V | Pull-ups on board? ⚠️ |
| `LED` | WS2812 strip | `TBD` | 5 V tolerant? ⚠️ | — |
| `BZ+` / `BZ-` | Buzzer | `TBD` | 5 V | Active buzzer |
| `CAM` | Camera video in | — | Analog | To OSD |
| `VTX` | Video out | — | Analog | From OSD |
| `CURR` | Current sense | `TBD` (ADC) | 0–3.3 V | Internal on AIO |
| `RSSI` | Analog RSSI | `TBD` (ADC) | 0–3.3 V | — |
| `BOOT` | DFU bootloader | BOOT0 | — | Bridge/hold during power-up |

### Connector Pinouts

**Receiver / GPS JST-SH (`TBD`-pin) ⚠️ VERIFY**

| Pin | Signal |
|---|---|
| 1 | 5V |
| 2 | GND |
| 3 | TX |
| 4 | RX |
| 5 | SCL |
| 6 | SDA |

---

## UART Mapping

ArduPilot assigns `SERIALn` by the `SERIAL_ORDER` line in `hwdef.dat`.

| ArduPilot Port | Hardware UART | Pads | Default Protocol | Default Baud |
|---|---|---|---|---|
| SERIAL0 | USB (OTG1) | USB-C | MAVLink2 | 115200 |
| SERIAL1 | `TBD` | `T1/R1` | MAVLink2 (Telem) | 57600 |
| SERIAL2 | `TBD` | `T2/R2` | GPS | 230400 |
| SERIAL3 | `TBD` | `T3/R3` | RCIN (CRSF/ELRS) | 420000 |
| SERIAL4 | `TBD` | `T4/R4` | DisplayPort / VTX | 115200 |
| SERIAL5 | `TBD` | ESC telem | ESC Telemetry | 115200 |

**RC input:** Set `SERIALn_PROTOCOL = 23` (RCIN) on the receiver's UART. For CRSF/ELRS, also set `RSSI_TYPE = 3`.

---

## PWM / Motor Outputs

Outputs that share a hardware timer must use the same protocol and rate.

| Output | MCU Pin | Timer/Channel | Timer Group | DShot | Bidir DShot |
|---|---|---|---|---|---|
| M1 | `TBD` | `TIMx_CHy` | 1 | ✅ | ✅ ⚠️ |
| M2 | `TBD` | `TIMx_CHy` | 1 | ✅ | ✅ ⚠️ |
| M3 | `TBD` | `TIMx_CHy` | 1 | ✅ | ✅ ⚠️ |
| M4 | `TBD` | `TIMx_CHy` | 1 | ✅ | ✅ ⚠️ |
| LED | `TBD` | `TIMx_CHy` | 2 | — | — |

Recommended motor configuration:

```text
MOT_PWM_TYPE      = 6     # DShot600
SERVO_BLH_BDMASK  = 15    # Bidirectional DShot on outputs 1-4
SERVO_BLH_POLES   = 14    # Motor magnet count
INS_HNTCH_MODE    = 3     # ESC-telemetry-driven harmonic notch
```

---

## Analog & Power Monitoring

| Parameter | Value | Notes |
|---|---|---|
| `BATT_MONITOR` | 4 | Analog voltage + current |
| `BATT_VOLT_PIN` | `TBD` | ⚠️ From hwdef |
| `BATT_CURR_PIN` | `TBD` | ⚠️ From hwdef |
| `BATT_VOLT_MULT` | `TBD` | Divider ratio. Calibrate with a multimeter. |
| `BATT_AMP_PERVLT` | `TBD` | Shunt/amp scale. Calibrate with a load test. |
| `BATT_AMP_OFFSET` | `TBD` | — |

**Calibration procedure:**

1. Measure the pack voltage with a calibrated DMM.
2. Adjust `BATT_VOLT_MULT` until the reading matches within 0.05 V.
3. Fly or bench-run a known mAh. Compare it against the charger's put-back value.
4. Scale `BATT_AMP_PERVLT` by the ratio of the two.

---

## Building ArduPilot

### 1. Environment Setup (Ubuntu 22.04/24.04)

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

macOS users: `Tools/environment_install/install-prereqs-mac.sh`.
Windows users: use WSL2 with the Ubuntu steps above.

### 2. Add the Board Definition

Copy this repo's hwdef files into the ArduPilot tree:

```bash
BOARD=NanoH743AIO   # ⚠️ Must match directory name exactly
mkdir -p libraries/AP_HAL_ChibiOS/hwdef/$BOARD
cp /path/to/nano-h743-aio-flight-controller/hwdef/{hwdef.dat,hwdef-bl.dat,defaults.parm,README.md} \
   libraries/AP_HAL_ChibiOS/hwdef/$BOARD/
```

> A new board needs a unique `APJ_BOARD_ID`. Request one by PR to
> `Tools/AP_Bootloader/board_types.txt` in the ArduPilot repo. Do not reuse another board's ID.

### 3. Build the Bootloader

```bash
Tools/scripts/build_bootloaders.py $BOARD
# Output: Tools/bootloaders/${BOARD}_bl.bin / _bl.hex
```

### 4. Build the Firmware

```bash
./waf configure --board $BOARD
./waf copter        # or: plane, rover, heli, sub
```

Artifacts are written to `build/$BOARD/bin/`:

| File | Use |
|---|---|
| `arducopter.apj` | Upload through Mission Planner / QGC / uploader.py (bootloader already present) |
| `arducopter_with_bl.hex` | Full image for DFU / STM32CubeProgrammer (blank boards) |
| `arducopter.bin` | Raw app binary |

### 5. Clean Rebuild

```bash
./waf distclean
./waf configure --board $BOARD && ./waf copter
```

---

## Flashing Firmware

### Method A: ArduPilot Bootloader (normal updates)

Use this when the board already runs ArduPilot.

```bash
./waf copter --upload
# or
python3 Tools/scripts/uploader.py build/$BOARD/bin/arducopter.apj
```

You can also use Mission Planner → *Install Firmware* → *Load custom firmware* → select the `.apj`.

### Method B: DFU (blank board, Betaflight/INAV → ArduPilot, or recovery)

1. Hold **BOOT** (or bridge the BOOT pads) and plug in USB.
2. Confirm the board appears in DFU mode:
   ```bash
   dfu-util -l    # expect "STM32 BOOTLOADER", alt=0 Internal Flash
   ```
3. Flash the combined image:
   - **Option 1: STM32CubeProgrammer (recommended, accepts .hex):**
     ```bash
     STM32_Programmer_CLI -c port=USB1 -e all -d arducopter_with_bl.hex -v -rst
     ```
   - **Option 2: dfu-util (requires .bin, bootloader first):**
     ```bash
     dfu-util -a 0 -s 0x08000000:mass-erase:force -D Tools/bootloaders/${BOARD}_bl.bin
     # Then power-cycle and upload the .apj with Method A
     ```
4. Power-cycle the board. Normal LED pattern: `TBD` ⚠️.

On Windows, use **ImpulseRC Driver Fixer** or **Zadig** (WinUSB) if DFU is not detected.

> ⚠️ `-e all` erases the stored parameters. Back up `.param` files before flashing.

### Flash Memory Layout (STM32H743, 128 KB sectors)

| Region | Address | Size |
|---|---|---|
| Bootloader | `0x08000000` | 128 KB |
| Application | `0x08020000` | Remainder ⚠️ VERIFY `FLASH_RESERVE_START_KB` |

---

## First-Time Configuration

1. Connect via USB and open Mission Planner or QGroundControl.
2. Set `FRAME_CLASS = 1` (Quad) and `FRAME_TYPE` to match your motor layout. Betaflight-style "Quad X" uses `FRAME_TYPE = 12` (BF X).
3. Run accelerometer calibration, then compass calibration if a compass is fitted.
4. Configure RC (see [UART Mapping](#uart-mapping)). Calibrate the radio.
5. Set up the ESCs with `MOT_PWM_TYPE` and bidir DShot.
6. Test motor order and direction with *Motor Test*. **Remove the props first.**
7. Set the battery failsafe: `BATT_LOW_VOLT`, `BATT_FS_LOW_ACT`.
8. Do a maiden hover in `STABILIZE`, then run `AUTOTUNE`.

---

## Thermal Specifications

> ⚠️ Board-level values in this section **must come from your own thermal testing**. MCU and component limits are taken from manufacturer datasheets. Verify the exact part grades on your BOM.

### Component Limits (Datasheet)

| Component | Parameter | Limit | Source |
|---|---|---|---|
| STM32H743 (suffix 6) | Ambient operating | −40