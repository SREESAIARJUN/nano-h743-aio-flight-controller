I don't have access to your repository, schematic, or prior "cycles," so I can't reproduce your board's actual pin assignments or measured thermal data. Below is a complete README built on **real STM32H743 and ArduPilot conventions**. Board-specific values are marked **`⟨VERIFY⟩`**. Please replace them from your schematic, `hwdef.dat`, and thermal test logs before publishing. Pinouts and current ratings that aren't checked against hardware can damage hardware or cause flyaways.

---

```markdown
# Nano H743 AIO Flight Controller

![Board Render](docs/img/board-top.png)

[![Firmware](https://img.shields.io/badge/firmware-ArduPilot-blue)](https://ardupilot.org)
[![MCU](https://img.shields.io/badge/MCU-STM32H743-green)]()
[![License](https://img.shields.io/badge/license-GPLv3-lightgrey)](LICENSE)
[![Hardware Rev](https://img.shields.io/badge/hw%20rev-Cycle%2018-orange)]()

A compact all-in-one (AIO) flight controller built on the STM32H743 (480 MHz Cortex-M7). It has an integrated 4-in-1 ESC, OSD, and barometer, and is designed for 2–4" multirotors running **ArduPilot** (Copter / Plane / Rover).

> **Hardware revision:** Cycle 18 · **Document revision:** 1.0 · **Status:** ⟨VERIFY: Beta / Production⟩

---

## Table of Contents

1. [Specifications](#specifications)
2. [Board Layout & Pinout](#board-layout--pinout)
3. [UART Mapping](#uart-mapping)
4. [PWM / Motor Outputs](#pwm--motor-outputs)
5. [Power & Current Sensing](#power--current-sensing)
6. [Thermal Specifications](#thermal-specifications)
7. [Building ArduPilot](#building-ardupilot)
8. [Flashing Firmware](#flashing-firmware)
9. [First-Time Setup](#first-time-setup)
10. [Troubleshooting](#troubleshooting)
11. [Revision History](#revision-history)
12. [License](#license)

---

## Specifications

| Category | Specification |
|---|---|
| MCU | STM32H743VIT6, 480 MHz, 2 MB Flash, 1 MB RAM |
| IMU | ⟨VERIFY: e.g., ICM-42688-P⟩ (SPI) |
| Barometer | ⟨VERIFY: e.g., DPS310 / BMP280⟩ (I²C) |
| OSD | AT7456E (SPI) analog OSD |
| Blackbox | ⟨VERIFY: 16 MB SPI flash / microSD⟩ |
| ESC | Integrated 4-in-1, ⟨VERIFY⟩ A continuous / ⟨VERIFY⟩ A burst per channel |
| ESC firmware | ⟨VERIFY: BLHeli_S / Bluejay / AM32⟩ |
| ESC protocol | DShot 150/300/600, bidirectional DShot ⟨VERIFY⟩ |
| Input voltage | 2S–⟨VERIFY: 4S/6S⟩ LiPo (⟨VERIFY⟩ V – ⟨VERIFY⟩ V) |
| BEC outputs | 5 V @ ⟨VERIFY⟩ A, 10 V @ ⟨VERIFY⟩ A (VTX), 3.3 V @ ⟨VERIFY⟩ mA |
| UARTs | ⟨VERIFY⟩ hardware UARTs |
| I²C | 1× (compass / external sensors) |
| USB | USB-C, native USB (MAVLink / DFU) |
| Mounting | 25.5 × 25.5 mm, M2 ⟨VERIFY⟩ |
| Dimensions | ⟨VERIFY⟩ × ⟨VERIFY⟩ × ⟨VERIFY⟩ mm |
| Weight | ⟨VERIFY⟩ g |

---

## Board Layout & Pinout

![Top Pinout](docs/img/pinout-top.png)
![Bottom Pinout](docs/img/pinout-bottom.png)

### Solder Pad Reference

| Pad | Function | MCU Pin | Voltage | Notes |
|---|---|---|---|---|
| BAT+ | Battery positive | — | VBAT | Main power input |
| BAT− | Battery ground | — | GND | |
| 5V | 5 V BEC output | — | 5.0 V | Receiver, GPS, peripherals |
| 10V | VTX BEC output | — | 10.0 V | ⟨VERIFY: switchable via GPIO?⟩ |
| 3V3 | 3.3 V output | — | 3.3 V | Low-current only |
| G | Ground | — | GND | |
| T1 / R1 | UART1 TX / RX | ⟨VERIFY⟩ | 3.3 V | See UART table |
| T2 / R2 | UART2 TX / RX | ⟨VERIFY⟩ | 3.3 V | |
| T3 / R3 | UART3 TX / RX | ⟨VERIFY⟩ | 3.3 V | |
| T4 / R4 | UART4 TX / RX | ⟨VERIFY⟩ | 3.3 V | |
| SDA / SCL | I²C1 | ⟨VERIFY⟩ | 3.3 V | Onboard pull-ups ⟨VERIFY⟩ |
| LED | WS2812 LED strip | ⟨VERIFY⟩ | 5 V data-tolerant ⟨VERIFY⟩ | |
| BZ+ / BZ− | Buzzer | ⟨VERIFY⟩ | 5 V | Active buzzer |
| VTX | Analog video out | — | — | |
| CAM | Analog camera in | — | — | |
| RSSI | Analog RSSI | ⟨VERIFY⟩ | 0–3.3 V | |
| BOOT | DFU boot button | BOOT0 | — | Hold while plugging in USB |

> ⚠️ **All UART and GPIO pads are 3.3 V logic.** ⟨VERIFY: state which, if any, are 5 V tolerant (FT pins on the H743)⟩. Never connect battery voltage to signal pads.

---

## UART Mapping

ArduPilot assigns `SERIALn` parameters in the order defined by `SERIAL_ORDER` in `hwdef.dat`.

| ArduPilot Port | Hardware | Pads | Default Protocol | Default Baud |
|---|---|---|---|---|
| SERIAL0 | OTG1 (USB) | USB-C | MAVLink2 | 115200 |
| SERIAL1 | ⟨VERIFY: USART1⟩ | T1/R1 | MAVLink2 (telemetry) | 57600 |
| SERIAL2 | ⟨VERIFY: USART2⟩ | T2/R2 | GPS | 230400 |
| SERIAL3 | ⟨VERIFY: USART3⟩ | T3/R3 | RCIN (CRSF/ELRS) | 420000 |
| SERIAL4 | ⟨VERIFY: UART4⟩ | T4/R4 | DisplayPort / VTX | 115200 |
| SERIAL5 | ⟨VERIFY⟩ | Internal | ESC telemetry | 115200 |

The matching `hwdef.dat` excerpt is:

```
SERIAL_ORDER OTG1 USART1 USART2 USART3 UART4 ⟨VERIFY⟩
```

---

## PWM / Motor Outputs

| Output | Timer / Channel | Function | DShot | BiDir DShot |
|---|---|---|---|---|
| M1 | ⟨VERIFY: TIM3_CH3⟩ | Motor 1 | ✅ | ✅ |
| M2 | ⟨VERIFY: TIM3_CH4⟩ | Motor 2 | ✅ | ✅ |
| M3 | ⟨VERIFY: TIM2_CH1⟩ | Motor 3 | ✅ | ✅ |
| M4 | ⟨VERIFY: TIM2_CH2⟩ | Motor 4 | ✅ | ✅ |
| LED | ⟨VERIFY⟩ | NeoPixel / Serial LED | — | — |

> Outputs sharing a timer **must use the same protocol and rate**. Timer groups: ⟨VERIFY: {1,2} / {3,4}⟩.

The default motor order is ArduCopter **Quad-X**. Check the order in Mission Planner → *Motor Test* before the first flight.

---

## Power & Current Sensing

| Parameter | Value |
|---|---|
| `BATT_MONITOR` | 4 (Analog voltage and current) |
| `BATT_VOLT_PIN` | ⟨VERIFY⟩ |
| `BATT_CURR_PIN` | ⟨VERIFY⟩ |
| `BATT_VOLT_MULT` | ⟨VERIFY: e.g., 11.0⟩ |
| `BATT_AMP_PERVLT` | ⟨VERIFY: calibrate per board⟩ |
| `BATT_AMP_OFFSET` | ⟨VERIFY⟩ |

**Calibrating current:**
1. Fly or bench-load a known capacity.
2. Compare the mAh consumed against what the charger reports.
3. Adjust `BATT_AMP_PERVLT` proportionally.

---

## Thermal Specifications

> Values below marked ⟨VERIFY⟩ must come from **measured test data** for Cycle 18 hardware. Do not publish datasheet-derived estimates as board ratings.

### Component Limits (from manufacturer datasheets)

| Component | Parameter | Limit |
|---|---|---|
| STM32H743 (temp range 6) | Ambient operating | −40 °C to +85 °C |
| STM32H743 | Max junction (T<sub>J</sub>) | ⟨VERIFY against your exact part suffix⟩ |
| ESC MOSFETs | Max junction | ⟨VERIFY: per MOSFET datasheet, typically 150–175 °C⟩ |
| IMU | Operating range | ⟨VERIFY per IMU datasheet⟩ |

### Board-Level Ratings (measured)

| Condition | Value |
|---|---|
| Rated ambient operating range | ⟨VERIFY⟩ °C to ⟨VERIFY⟩ °C |
| ESC continuous current (with airflow, ⟨VERIFY⟩ m/s) | ⟨VERIFY⟩ A/ch |
| ESC continuous current (still air) | ⟨VERIFY⟩ A/ch |
| ESC burst current (⟨VERIFY⟩ s) | ⟨VERIFY⟩ A/ch |
| MOSFET hotspot @ rated continuous, 25 °C ambient | ⟨VERIFY⟩ °C |
| MCU temperature, idle on USB | ⟨VERIFY⟩ °C |
| BEC regulator hotspot @ full load | ⟨VERIFY⟩ °C |
| Recommended thermal shutdown / warning threshold | ⟨VERIFY⟩ °C |

### Test Methodology

Document the method alongside the results so others can reproduce them:
- Thermal camera model and emissivity setting
- Ambient temperature
- Airflow speed
- Load profile
- Test duration

**Raw data location:** `docs/thermal/cycle18/`

### Thermal Guidelines

- Do **not** run the ESC at rated current on the bench without airflow.
- Mount the board so prop wash reaches the ESC section.
- Avoid conformal coating over the MOSFETs unless thermal testing was done with the coating applied.
- Monitor IMU temperature in logs (`IMU.T`). Large swings degrade the EKF.
- ⟨VERIFY: if IMU heater supported, document `BRD_HEAT_TARG`⟩

---

## Building ArduPilot

### Prerequisites

Supported build environments are Ubuntu 22.04+, WSL2, or macOS.

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

### Adding the Board Definition

Copy the hardware definition from this repository into the ArduPilot tree:

```bash
mkdir -p libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO
cp /path/to/this-repo/hwdef/hwdef.dat     libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/
cp /path/to/this-repo/hwdef/hwdef-bl.dat  libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/
cp /path/to/this-repo/hwdef/defaults.parm libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/
```

> The `APJ_BOARD_ID` in `hwdef.dat` must be unique. Request an official ID from the ArduPilot team before distributing firmware publicly: ⟨VERIFY: assigned ID⟩.

### Build the Bootloader

```bash
./waf configure --board NanoH743AIO --bootloader
./waf bootloader
# Output: build/NanoH743AIO/bin/AP_Bootloader.bin
```

To bundle the bootloader into the main firmware:

```bash
Tools/scripts/build_bootloaders.py NanoH743AIO
```

### Build the Firmware

```bash
./waf configure --board NanoH743AIO
./waf copter        # or: plane, rover, sub
```

Output files appear in `build/NanoH743AIO/bin/`:

| File | Use |
|---|---|
| `arducopter.apj` | Upload via Mission Planner / QGC / `uploader.py` |
| `arducopter_with_bl.hex` | Full flash including bootloader (DFU / ST-Link) |
| `arducopter.bin` | Raw application binary |

### Clean Rebuild

```bash
./waf distclean
./waf configure --board NanoH743AIO
./waf copter
```

---

## Flashing Firmware

### Method A — First Flash via DFU (no ArduPilot bootloader yet)

1. Hold the **BOOT** button.
2. Connect USB-C, then release the button.
3. Confirm the device shows up in DFU mode:
   ```bash
   dfu-util -l    # expect: STM32 BOOTLOADER, 0483:df11
   ```
4. Flash the combined image. Use **STM32CubeProgrammer** with `arducopter_with_bl.hex`, or use dfu-util:
   ```bash
   # Convert hex to bin first if needed, then:
   dfu-util -a 0 -s 0x08000000:leave -D arducopter_with_bl.bin
   ```
5. Unplug and replug the USB. The board now boots ArduPilot.

> **Linux:** if `dfu-util` reports permission errors, install udev rules or run with `sudo`.
> **Windows:** use Zadig to install the WinUSB driver for the DFU device, or use STM32CubeProgrammer.

### Method B — Updates via ArduPilot Bootloader

Use a ground control station:
- **Mission Planner:** *Setup → Install Firmware → Load custom firmware* → select the `.apj`
- **QGroundControl:** *Vehicle Setup → Firmware → Advanced → Custom firmware file*

Or use the command line:

```bash
./waf copter --upload
# or
Tools/