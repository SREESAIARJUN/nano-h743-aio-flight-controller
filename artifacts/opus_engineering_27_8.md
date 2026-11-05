I don't have access to this repository's schematic, hwdef, or test data, so I can't know your board's actual pin assignments or measured thermal figures. The README below is complete in structure and accurate wherever it relies on public facts (STM32H743 specs, the ArduPilot build system, DFU flashing). **Everything board-specific is marked `⚠ VERIFY`.** Replace those values from your schematic and bench tests before publishing, because a wrong pinout in a flight controller README can destroy ESCs or receivers.

---

````markdown
# Nano H743 AIO Flight Controller

> Compact STM32H743 all-in-one flight controller with integrated 4-in-1 ESC, targeting ArduPilot (Copter / Plane / Rover).

![Board top view](docs/img/board-top.png)
![Board bottom view](docs/img/board-bottom.png)

| Document revision | Hardware revision | Firmware baseline |
|---|---|---|
| Cycle 27 | rev ⚠ VERIFY | ArduPilot ⚠ VERIFY (e.g. Copter-4.5.x) |

---

## Table of Contents

1. [Overview](#overview)
2. [Specifications](#specifications)
3. [Pinout](#pinout)
4. [UART Mapping](#uart-mapping)
5. [PWM / Motor Outputs](#pwm--motor-outputs)
6. [Analog, Power & Battery Monitoring](#analog-power--battery-monitoring)
7. [Building ArduPilot](#building-ardupilot)
8. [Flashing](#flashing)
9. [First-Time Configuration](#first-time-configuration)
10. [Thermal Specifications](#thermal-specifications)
11. [Mechanical](#mechanical)
12. [Troubleshooting](#troubleshooting)
13. [Safety](#safety)
14. [Contributing](#contributing)
15. [License](#license)

---

## Overview

The Nano H743 AIO combines a flight controller and a 4-in-1 ESC on a single PCB for sub-250 g and micro-class airframes. It runs ArduPilot natively through a ChibiOS hwdef target, `NanoH743AIO`.

**Key features**
- STM32H743 @ 480 MHz, 2 MB flash, 1 MB RAM
- Integrated 4-in-1 ESC (BLHeli_S / Bluejay / AM32: ⚠ VERIFY)
- Onboard barometer, IMU, OSD, and blackbox storage
- DShot, bidirectional DShot, and ESC telemetry support

---

## Specifications

| Parameter | Value |
|---|---|
| MCU | STM32H743VIT6 (Cortex-M7, 480 MHz, 2 MB flash, 1 MB RAM) ⚠ VERIFY package |
| IMU | ⚠ VERIFY (e.g. ICM-42688-P) on SPI1 |
| Barometer | ⚠ VERIFY (e.g. DPS310 / BMP280) on I2C2 |
| OSD | AT7456E (analog) on SPI2 ⚠ VERIFY |
| Blackbox | ⚠ VERIFY (microSD via SDMMC / 16 MB SPI flash) |
| UARTs | ⚠ VERIFY count (see mapping) |
| ESC | 4-in-1, ⚠ VERIFY A continuous / ⚠ VERIFY A burst per motor |
| ESC firmware | ⚠ VERIFY |
| Input voltage | ⚠ VERIFY (e.g. 2S–4S LiPo, 7.4–16.8 V) |
| BEC outputs | 5 V @ ⚠ VERIFY A, 10 V @ ⚠ VERIFY A (VTX), 3.3 V internal |
| Current sensor | Onboard shunt, ⚠ VERIFY A max |
| Mounting | ⚠ VERIFY (e.g. 25.5 × 25.5 mm, M2) |
| Dimensions | ⚠ VERIFY mm |
| Weight | ⚠ VERIFY g |

---

## Pinout

### Board connectors

```
                   ┌─────────────────────────────────────┐
     M1 ●──────────┤                                     ├──────────● M2
                   │   [USB-C]                [BOOT]     │
                   │                                     │
   5V  GND RX1 TX1 │   ┌───────────────┐                 │ 10V GND VTX CAM
   ●   ●   ●   ●   │   │  STM32H743    │                 │ ●   ●   ●   ●
                   │   └───────────────┘                 │
   5V  GND RX2 TX2 │                                     │ SDA SCL 5V GND
   ●   ●   ●   ●   │      ( IMU )                        │ ●   ●   ●   ●
                   │                                     │
     M4 ●──────────┤   BAT+                      BAT-    ├──────────● M3
                   └─────────────────────────────────────┘
```

> ⚠ VERIFY: Regenerate this diagram from the actual PCB silkscreen. Replace it with a rendered image in `docs/img/pinout.png` for the release.

### MCU pin assignments

| Function | MCU Pin | Peripheral | Notes |
|---|---|---|---|
| IMU SCK / MISO / MOSI | ⚠ PA5 / PA6 / PD7 | SPI1 | |
| IMU CS | ⚠ PC15 | GPIO | |
| IMU INT | ⚠ PB2 | EXTI | Optional |
| OSD SCK / MISO / MOSI | ⚠ PB13 / PB14 / PB15 | SPI2 | |
| OSD CS | ⚠ PB12 | GPIO | |
| Baro SCL / SDA | ⚠ PB10 / PB11 | I2C2 | |
| External I2C SCL / SDA | ⚠ PB8 / PB9 | I2C1 | Compass |
| VBAT sense | ⚠ PC0 | ADC | |
| Current sense | ⚠ PC1 | ADC | |
| RSSI (analog) | ⚠ PC5 | ADC | Optional |
| LED (status) | ⚠ PE3 | GPIO | |
| Buzzer | ⚠ PA15 | GPIO / TIM2 | |
| USB D− / D+ | PA11 / PA12 | OTG_FS | Fixed by MCU |

---

## UART Mapping

ArduPilot assigns `SERIALn` by the `SERIAL_ORDER` line in hwdef.dat.

| ArduPilot | UART | Pins (TX / RX) | Default protocol | Typical use |
|---|---|---|---|---|
| SERIAL0 | USB | — | MAVLink2 | Ground station |
| SERIAL1 | USART1 | ⚠ PA9 / PA10 | MAVLink2 | Telemetry |
| SERIAL2 | USART2 | ⚠ PD5 / PD6 | GPS | GPS |
| SERIAL3 | USART3 | ⚠ PD8 / PD9 | RCIN | Receiver (CRSF/ELRS/SBUS) |
| SERIAL4 | UART4 | ⚠ PB9 / PB8 | — | VTX control / spare |
| SERIAL5 | UART7 | ⚠ —   / PE7 | ESC Telem | 4-in-1 ESC telemetry (RX only) |

> **Receivers:** For CRSF/ELRS, set `SERIALx_PROTOCOL = 23` (RCIN). For SBUS, use the inverted RX pad if one is provided.

---

## PWM / Motor Outputs

| Output | Pin | Timer / Channel | DShot | BDShot | Group |
|---|---|---|---|---|---|
| M1 | ⚠ PB0 | TIM3_CH3 | ✓ | ✓ | 1 |
| M2 | ⚠ PB1 | TIM3_CH4 | ✓ | ✓ | 1 |
| M3 | ⚠ PA0 | TIM5_CH1 | ✓ | ✓ | 2 |
| M4 | ⚠ PA1 | TIM5_CH2 | ✓ | ✓ | 2 |
| LED strip | ⚠ PA8 | TIM1_CH1 | — | — | 3 |

**Output groups:** All outputs in a group share a timer and must use the same protocol and rate. Do not mix DShot and PWM within a group.

---

## Analog, Power & Battery Monitoring

| Parameter | Default | Notes |
|---|---|---|
| `BATT_MONITOR` | 4 | Analog voltage + current |
| `BATT_VOLT_PIN` | ⚠ VERIFY | ADC channel from hwdef |
| `BATT_CURR_PIN` | ⚠ VERIFY | |
| `BATT_VOLT_MULT` | ⚠ VERIFY (e.g. 11.0) | Divider ratio |
| `BATT_AMP_PERVLT` | ⚠ VERIFY | Calibrate against a reference meter |

**Calibration procedure**
1. Measure the pack voltage with a multimeter. Set `BATT_VOLT_MULT = old × (measured / reported)`.
2. Fly or bench-run a known load. Adjust `BATT_AMP_PERVLT` until consumed mAh matches the charger's returned mAh within ±5 %.

---

## Building ArduPilot

### Prerequisites

Ubuntu 22.04 / 24.04 is recommended. WSL2 works on Windows.

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

### Add the board target

Copy this repository's hwdef into the ArduPilot tree:

```bash
cp -r <this-repo>/hwdef/NanoH743AIO \
      libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO
```

The directory must contain:

```
NanoH743AIO/
├── hwdef.dat        # Main firmware hardware definition
├── hwdef-bl.dat     # Bootloader hardware definition
├── defaults.parm    # Board default parameters
└── README.md        # Board docs (ArduPilot wiki format)
```

### Build firmware

```bash
./waf configure --board NanoH743AIO
./waf copter        # or: plane | rover | heli
```

Output files are written to `build/NanoH743AIO/bin/`:

| File | Purpose |
|---|---|
| `arducopter.apj` | Upload through Mission Planner / QGC / uploader.py |
| `arducopter_with_bl.hex` | Full image with bootloader (DFU / SWD) |
| `arducopter.bin` | Raw application binary |

### Build the bootloader

```bash
Tools/scripts/build_bootloaders.py NanoH743AIO
```

This writes `Tools/bootloaders/NanoH743AIO_bl.bin` (and `.hex`). Rebuild the firmware afterward so `*_with_bl.hex` embeds the new bootloader.

### Clean build

```bash
./waf distclean
./waf configure --board NanoH743AIO
./waf copter
```

### Example hwdef.dat excerpt

⚠ This is illustrative only. It must match the real schematic.

```
MCU STM32H7xx STM32H743xx
APJ_BOARD_ID ⚠ VERIFY       # Request an official ID from ArduPilot
OSCILLATOR_HZ 8000000       # ⚠ VERIFY crystal
FLASH_SIZE_KB 2048
FLASH_RESERVE_START_KB 128

SERIAL_ORDER OTG1 USART1 USART2 USART3 UART4 UART7

PA9  USART1_TX USART1
PA10 USART1_RX USART1

PB0 TIM3_CH3 TIM3 PWM(1) GPIO(50) BIDIR
PB1 TIM3_CH4 TIM3 PWM(2) GPIO(51)
PA0 TIM5_CH1 TIM5 PWM(3) GPIO(52) BIDIR
PA1 TIM5_CH2 TIM5 PWM(4) GPIO(53)

SPIDEV imu1 SPI1 DEVID1 IMU1_CS MODE3 2*MHZ 16*MHZ
IMU Invensensev3 SPI:imu1 ROTATION_NONE   # ⚠ VERIFY rotation

BARO DPS310 I2C:0:0x76                    # ⚠ VERIFY
define HAL_BATT_MONITOR_DEFAULT 4
```

---

## Flashing

### Option A: First flash via DFU (no ArduPilot bootloader present)

1. Disconnect the battery.
2. Hold the **BOOT** button and connect USB. The board enumerates as `STM32 BOOTLOADER` (VID:PID `0483:df11`).
3. Flash the combined image.

**Linux / macOS (dfu-util):**
```bash
# Convert hex to bin if needed, or use the bootloader .bin directly
dfu-util -a 0 -s 0x08000000:leave -D Tools/bootloaders/NanoH743AIO_bl.bin
```

**STM32CubeProgrammer (all OSes):**
```bash
STM32_Programmer_CLI -c port=USB1 -w build/NanoH743AIO/bin/arducopter_with_bl.hex -v -rst
```

**Windows GUI:** Use STM32CubeProgrammer → USB → Connect → open `arducopter_with_bl.hex` → Download.

> **Windows driver note:** If the DFU device is not detected, install the WinUSB driver for `STM32 BOOTLOADER` with Zadig.

### Option B: Upgrade an existing ArduPilot install

**Command line:**
```bash
./waf copter --upload
# or
Tools/scripts/uploader.py build/NanoH743AIO/bin/arducopter.apj
```

**Mission Planner:** Setup → Install Firmware → *Load custom firmware* → select `arducopter.apj`.

**QGroundControl:** Vehicle Setup → Firmware → replug the board → *Advanced settings* → *Custom firmware file*.

### Option C: SWD (recovery)

If the board is bricked, connect an ST-Link to the SWDIO / SWCLK / GND / 3V3 test pads (⚠ VERIFY location) and run:

```bash
STM32_Programmer_CLI -c port=SWD -e all
STM32_Programmer_CLI -c port=SWD -w arducopter_with_bl.hex -v -rst
```

### Post-flash verification
- The status LED shows the ArduPilot boot pattern.
- The ground station connects over USB at 115200 baud (USB ignores the baud rate).
- `Messages` tab shows `ArduCopter V4.x.x` and `NanoH743AIO`.

---

## First-Time Configuration

1. **Frame:** Set `FRAME_CLASS = 1` (Quad) and `FRAME_TYPE` as required (e.g. 1 = X).
2. **Motor protocol:** Set `MOT_PWM_TYPE = 6` (DShot600) or `5` (DShot300).
3. **Bidirectional DShot:** Set `SERVO_BLH_BDMASK = 15`, `SERVO_BLH_POLES = 14` (⚠ match your motors), and `SERVO_BLH_AUTO = 1`.
4. **Motor order:** Verify with Mission Planner