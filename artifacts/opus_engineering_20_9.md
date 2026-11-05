# nano-h743-aio-flight-controller README (Cycle 20)

> **Before you merge:** I don't have access to your schematic, `hwdef.dat`, or thermal test data. Values marked **⚠ VERIFY** are reasonable defaults for an STM32H743 AIO design but **must be checked against your hardware**. A wrong pin mapping or current rating in a flight-controller README can damage hardware or cause a crash. The ArduPilot build and flashing commands use the standard upstream workflow.

---

````markdown
# Nano H743 AIO Flight Controller

![Board](docs/img/board-top.png)

[![ArduPilot](https://img.shields.io/badge/firmware-ArduPilot-blue)](https://ardupilot.org)
[![MCU](https://img.shields.io/badge/MCU-STM32H743-orange)](https://www.st.com/en/microcontrollers-microprocessors/stm32h743-753.html)
[![License](https://img.shields.io/badge/license-GPLv3-green)](LICENSE)

A compact all-in-one (AIO) flight controller for small multirotors, built around the
STM32H743 and targeting ArduPilot (Copter, Plane, Rover).

**Document revision:** Cycle 20 · **Hardware revision:** ⚠ VERIFY (e.g. Rev C)

---

## Table of Contents

1. [Features](#features)
2. [Specifications](#specifications)
3. [Pinout](#pinout)
4. [UART Mapping](#uart-mapping)
5. [PWM / Motor Outputs](#pwm--motor-outputs)
6. [Building ArduPilot](#building-arducopter-firmware)
7. [Flashing](#flashing)
8. [Thermal Specifications](#thermal-specifications)
9. [Default Parameters](#default-parameters)
10. [Troubleshooting](#troubleshooting)
11. [Safety](#safety)
12. [License](#license)

---

## Features

- STM32H743 Cortex-M7 @ 480 MHz, 2 MB flash, 1 MB RAM
- Dual IMU ⚠ VERIFY part numbers (e.g. ICM-42688-P + BMI270)
- Barometer ⚠ VERIFY (e.g. DPS310 / SPL06)
- OSD: AT7456E analog OSD ⚠ VERIFY
- Onboard flash logging (⚠ VERIFY: 16 MB SPI flash or microSD)
- 6 UARTs, I²C, CAN ⚠ VERIFY
- Integrated BEC: 5 V and 10 V (VTX) ⚠ VERIFY ratings
- 8 PWM outputs, DShot / bidirectional DShot capable
- Mounting: 25.5 × 25.5 mm (M2) ⚠ VERIFY

## Specifications

| Parameter            | Value                          | Notes |
|----------------------|--------------------------------|-------|
| MCU                  | STM32H743VIT6                  | 480 MHz, 2 MB flash |
| Input voltage        | 2S–6S LiPo (7–26 V) ⚠ VERIFY   | Absolute max must exceed 6S full charge (25.2 V) with margin |
| 5 V BEC              | 5 V / 2 A ⚠ VERIFY              | Continuous |
| 10 V BEC             | 10 V / 2 A ⚠ VERIFY             | VTX / camera |
| Current sensor       | ⚠ VERIFY scale & range          | See `BATT_AMP_PERVLT` |
| Dimensions           | ⚠ VERIFY                        | |
| Weight               | ⚠ VERIFY                        | Without wires |
| Operating temp       | See [Thermal](#thermal-specifications) | |

---

## Pinout

![Top pinout](docs/img/pinout-top.png)
![Bottom pinout](docs/img/pinout-bottom.png)

### Pad Reference

> Reference mapping. **The authoritative source is
> `libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/hwdef.dat`.** If this table and
> the hwdef disagree, the hwdef wins. File a bug against this README.

| Pad        | Function             | MCU Pin ⚠ VERIFY | Voltage |
|------------|----------------------|------------------|---------|
| BAT+       | Battery input        | —                | VBAT    |
| GND        | Ground               | —                | 0 V     |
| 5V         | 5 V BEC out          | —                | 5 V     |
| 10V        | 10 V BEC out         | —                | 10 V    |
| 4V5        | USB-powered rail     | —                | 4.5 V   |
| T1 / R1    | UART1 (MAVLink)      | PA9 / PA10       | 3.3 V   |
| T2 / R2    | UART2 (GPS)          | PD5 / PD6        | 3.3 V   |
| T3 / R3    | UART3                | PD8 / PD9        | 3.3 V   |
| T4 / R4    | UART4 (VTX/MSP)      | PB9 / PB8        | 3.3 V   |
| T6 / R6    | UART6 (RC input)     | PC6 / PC7        | 3.3 V   |
| T7 / R7    | UART7 (ESC telem)    | PE8 / PE7        | 3.3 V   |
| SDA / SCL  | I²C1 (compass)       | PB7 / PB6        | 3.3 V   |
| M1–M8      | Motor outputs        | see below        | 3.3 V   |
| LED        | WS2812 LED strip     | ⚠ VERIFY         | 3.3 V   |
| BZ+ / BZ-  | Buzzer               | ⚠ VERIFY         | 5 V     |
| CURR       | Current sense in     | ⚠ VERIFY (ADC)   | 0–3.3 V |
| RSSI       | Analog RSSI          | ⚠ VERIFY (ADC)   | 0–3.3 V |
| CAM        | Camera video in      | —                | —       |
| VTX        | Video out            | —                | —       |
| BOOT       | DFU boot button      | BOOT0            | —       |

> **All signal pads are 3.3 V logic.** Most are 5 V tolerant on the H743, but
> **ADC pins are not**. Never apply more than 3.3 V to CURR or RSSI.

---

## UART Mapping

ArduPilot `SERIALn` ordering is defined by `SERIAL_ORDER` in `hwdef.dat`.

| ArduPilot | Hardware | Default Protocol      | Param                |
|-----------|----------|-----------------------|----------------------|
| SERIAL0   | USB      | MAVLink2              | —                    |
| SERIAL1   | UART1    | MAVLink2 (2)          | `SERIAL1_PROTOCOL=2` |
| SERIAL2   | UART2    | GPS (5)               | `SERIAL2_PROTOCOL=5` |
| SERIAL3   | UART3    | User                  | —                    |
| SERIAL4   | UART4    | MSP DisplayPort (42)  | `SERIAL4_PROTOCOL=42`|
| SERIAL5   | UART6    | RCIN (23)             | `SERIAL5_PROTOCOL=23`|
| SERIAL6   | UART7    | ESC telemetry (16)    | `SERIAL6_PROTOCOL=16`|

RC input supports CRSF/ELRS, SBUS (inverted handled per hwdef), and other
protocols auto-detected by ArduPilot.

---

## PWM / Motor Outputs

Outputs in the same timer group **must share protocol and rate**.

| Output | Pin ⚠ VERIFY | Timer   | Group | Bidir DShot |
|--------|--------------|---------|-------|-------------|
| M1     | PB0          | TIM3_CH3| 1     | Yes |
| M2     | PB1          | TIM3_CH4| 1     | Yes |
| M3     | PA0          | TIM5_CH1| 2     | Yes |
| M4     | PA1          | TIM5_CH2| 2     | Yes |
| M5     | PA2          | TIM5_CH3| 2     | Yes |
| M6     | PA3          | TIM5_CH4| 2     | Yes |
| M7     | PD12         | TIM4_CH1| 3     | No  |
| M8     | PD13         | TIM4_CH2| 3     | No  |

Recommended for quad with BLHeli_32 / AM32:

```
MOT_PWM_TYPE      6     # DShot600
SERVO_BLH_BDMASK  15    # Bidirectional DShot on M1-M4
SERVO_BLH_AUTO    1
```

---

## Building ArduCopter Firmware

### 1. Environment setup (Ubuntu 22.04 / 24.04)

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

macOS: `Tools/environment_install/install-prereqs-mac.sh`
Windows: use WSL2 and follow the Ubuntu steps.

### 2. Add the board definition

Copy the hwdef files from this repo into the ArduPilot tree:

```bash
cp -r /path/to/nano-h743-aio-flight-controller/hwdef/NanoH743AIO \
      libraries/AP_HAL_ChibiOS/hwdef/
```

Expected files:

```
libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/
├── hwdef.dat        # main firmware definition
├── hwdef-bl.dat     # bootloader definition
├── defaults.parm    # board default parameters
└── README.md        # upstream board docs (optional)
```

> **Board ID:** `APJ_BOARD_ID` in `hwdef.dat` must be unique. For upstream
> submission, register an ID in `Tools/AP_Bootloader/board_types.txt`.
> Current value: ⚠ VERIFY.

### 3. Build the vehicle firmware

```bash
./waf configure --board NanoH743AIO
./waf copter        # or: plane, rover, heli
```

Output:

```
build/NanoH743AIO/bin/arducopter.apj          # upload via bootloader
build/NanoH743AIO/bin/arducopter_with_bl.hex  # full image incl. bootloader
build/NanoH743AIO/bin/arducopter.bin
```

### 4. Build the bootloader (only needed for new boards / bootloader changes)

```bash
./waf configure --board NanoH743AIO --bootloader
./waf bootloader
```

Or use the helper script, which also copies the result into
`Tools/bootloaders/` so it is embedded in `_with_bl.hex` builds:

```bash
Tools/scripts/build_bootloaders.py NanoH743AIO
```

### 5. Clean rebuild

```bash
./waf distclean
./waf configure --board NanoH743AIO
./waf copter
```

---

## Flashing

### Method A: First flash (blank board, DFU)

Use this on a new board or to recover a corrupted bootloader.

1. Disconnect the battery.
2. Hold **BOOT**, plug in USB, then release.
3. Confirm DFU mode:

   ```bash
   dfu-util -l
   # Expect: Found DFU: [0483:df11] ... name="@Internal Flash"
   ```

4. Flash the combined image with **STM32CubeProgrammer** (recommended for `.hex`):

   ```bash
   STM32_Programmer_CLI -c port=USB1 -e all \
       -w build/NanoH743AIO/bin/arducopter_with_bl.hex -v -rst
   ```

   Or with `dfu-util` using a `.bin`, converted from the `_with_bl.hex` and
   written at the flash base:

   ```bash
   arm-none-eabi-objcopy -I ihex -O binary \
       build/NanoH743AIO/bin/arducopter_with_bl.hex arducopter_with_bl.bin
   dfu-util -a 0 --dfuse-address 0x08000000:mass-erase:force:leave \
       -D arducopter_with_bl.bin
   ```

5. Unplug and replug USB. The board should enumerate as a serial device.

### Method B: Firmware update (bootloader already installed)

**Mission Planner:** *Setup → Install Firmware → Load custom firmware* →
select `arducopter.apj`.

**QGroundControl:** *Vehicle Setup → Firmware → Advanced → Custom firmware file*.

**Command line:**

```bash
./waf copter --upload
# or
Tools/scripts/uploader.py --port /dev/ttyACM0 \
    build/NanoH743AIO/bin/arducopter.apj
```

### Method C: Restoring Betaflight / INAV

Enter DFU (Method A, steps 1–3) and flash with Betaflight Configurator.
This erases the ArduPilot bootloader.

### Linux udev rule (if DFU or serial needs root)

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="df11", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/50-stm32-dfu.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

---

## Thermal Specifications

### Component limits

| Component             | Limit                         | Source |
|-----------------------|-------------------------------|--------|
| STM32H743 (junction)  | 125 °C max                    | ST datasheet (check suffix: "6" = −40…85 °C ambient, "3" = −40…125 °C) |
| IMU operating range   | −40 … 85 °C (typical)         | ⚠ VERIFY per IMU datasheet |
| Barometer             | −40 … 85 °C (typical)         | ⚠ VERIFY |
| BEC regulators        | ⚠ VERIFY Tj max and thermal shutdown | Regulator datasheet |

### Board operating envelope ⚠ VERIFY: replace with measured data

