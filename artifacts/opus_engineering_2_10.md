# README.md: nano-h743-aio-flight-controller

**Note:** I don't have access to your repository, schematic, or Cycle 1 output. I've written verifiable content in full: ArduPilot build commands, flashing procedures, and STM32H743 silicon facts. Board-specific values that only your schematic or test data can supply are marked `⟨TBD⟩`. I did this instead of guessing, because a wrong pin or current rating in a flight controller README can destroy hardware. Paste your `hwdef.dat` or schematic net list and I'll fill every `⟨TBD⟩` in Cycle 3.

---

````markdown
# Nano H743 AIO Flight Controller

![Board Top](docs/img/board_top.png) ![Board Bottom](docs/img/board_bottom.png)

A compact all-in-one (AIO) flight controller built on the **STM32H743** (Arm Cortex-M7, 480 MHz),
with integrated ⟨TBD: 4-in-1 ESC rating⟩, OSD, barometer, and dual-gyro support. It runs **ArduPilot** (Copter / Plane / Rover)
using the ChibiOS HAL.

> **Status:** Hardware rev ⟨TBD⟩ · hwdef target `NanoH743AIO` · ArduPilot ≥ ⟨TBD: 4.5.x⟩

---

## Table of Contents
1. [Specifications](#specifications)
2. [Pinout](#pinout)
3. [UART / Serial Mapping](#uart--serial-mapping)
4. [Motor & PWM Outputs](#motor--pwm-outputs)
5. [Building ArduPilot](#building-ardupilot)
6. [Flashing](#flashing)
7. [Thermal Specifications](#thermal-specifications)
8. [Power & Electrical Limits](#power--electrical-limits)
9. [Default Parameters](#default-parameters)
10. [Troubleshooting](#troubleshooting)
11. [Repository Layout](#repository-layout)
12. [License](#license)

---

## Specifications

| Item | Value |
|---|---|
| MCU | STM32H743⟨VIT6 / VIH6⟩, Cortex-M7 @ 480 MHz, 2 MB flash, 1 MB RAM |
| IMU 1 | ⟨TBD, e.g. ICM-42688-P⟩ on SPI⟨n⟩ |
| IMU 2 | ⟨TBD / not fitted⟩ |
| Barometer | ⟨TBD, e.g. DPS310 / BMP280⟩ on I2C⟨n⟩ |
| OSD | ⟨TBD, e.g. AT7456E⟩ on SPI⟨n⟩ |
| Blackbox / logging | ⟨TBD: microSD (SDMMC) / SPI flash⟩ |
| Integrated ESC | ⟨TBD⟩ A continuous / ⟨TBD⟩ A burst per channel, ⟨BLHeli_S / AM32 / BLHeli_32⟩ |
| Input voltage | ⟨TBD⟩ – ⟨TBD⟩ S LiPo (⟨TBD⟩ – ⟨TBD⟩ V) |
| BEC outputs | 5 V @ ⟨TBD⟩ A, 9 V/10 V @ ⟨TBD⟩ A (VTX) |
| Current sensor | ⟨TBD⟩ A full-scale, scale ⟨TBD⟩ A/V |
| Mounting | ⟨TBD: 25.5 × 25.5 mm / 20 × 20 mm⟩, M⟨2⟩ |
| Dimensions / mass | ⟨TBD⟩ mm · ⟨TBD⟩ g |

---

## Pinout

### Board connector diagram
See `docs/img/pinout_top.png` and `docs/img/pinout_bottom.png`
(generated from the KiCad project in `hardware/`).

### Solder pads / connectors

| Pad | Function | MCU Pin | Notes |
|---|---|---|---|
| VBAT / GND | Battery input | — | ⟨TBD⟩ S max |
| 5V | 5 V BEC out | — | ⟨TBD⟩ A max |
| 9V | VTX supply | — | Switchable via ⟨GPIO / RELAY⟩ ⟨TBD⟩ |
| T1 / R1 | UART1 | ⟨PA9 / PA10⟩ | ⟨TBD⟩ |
| T2 / R2 | UART2 | ⟨TBD⟩ | |
| SDA / SCL | I2C⟨n⟩ (external compass) | ⟨TBD⟩ | 2.2 kΩ pull-ups ⟨fitted?⟩ |
| LED | WS2812 | ⟨TBD⟩ | Timer ⟨TBD⟩ |
| BZ+ / BZ- | Buzzer | ⟨TBD⟩ | |
| RSSI | Analog RSSI | ⟨TBD⟩ | ADC⟨n⟩ |
| CURR | Current sense | ⟨TBD⟩ | ADC⟨n⟩ |
| VID / VOUT | Analog video | — | OSD passthrough |
| BOOT | DFU boot pad | BOOT0 | Bridge to 3V3 while powering for ROM DFU |

---

## UART / Serial Mapping

| ArduPilot `SERIALn` | STM32 UART | TX / RX Pins | DMA | Default Protocol | Pad |
|---|---|---|---|---|---|
| SERIAL0 | USB (OTG1) | PA11 / PA12 | — | MAVLink2 | USB-C |
| SERIAL1 | ⟨USART1⟩ | ⟨TBD⟩ | ⟨Y/N⟩ | MAVLink2 (`1=2`) | T1/R1 |
| SERIAL2 | ⟨USART2⟩ | ⟨TBD⟩ | ⟨Y/N⟩ | ⟨TBD⟩ | T2/R2 |
| SERIAL3 | ⟨USART3⟩ | ⟨TBD⟩ | ⟨Y/N⟩ | GPS (`5`) | T3/R3 |
| SERIAL4 | ⟨UART4⟩ | ⟨TBD⟩ | ⟨Y/N⟩ | ⟨TBD⟩ | T4/R4 |
| SERIAL5 | ⟨TBD⟩ | ⟨TBD⟩ | ⟨Y/N⟩ | ESC Telemetry (`16`) | Internal (ESC) |
| SERIAL6 | ⟨TBD⟩ | ⟨TBD⟩ | ⟨Y/N⟩ | RCIN (`23`) | RX pad |

> UARTs without DMA are not suitable for high-rate protocols (e.g. CRSF at 416 kbaud or DisplayPort MSP).
> Check `SERIAL_ORDER` and the `# UART` section in `hwdef.dat`.

---

## Motor & PWM Outputs

DShot requires every output on a timer to use the **same protocol and rate**.

| Output | MCU Pin | Timer / Channel | Group | DShot | Bidir DShot |
|---|---|---|---|---|---|
| M1 | ⟨TBD⟩ | TIM⟨n⟩_CH⟨n⟩ | 1 | ✔ | ⟨✔/✘⟩ |
| M2 | ⟨TBD⟩ | TIM⟨n⟩_CH⟨n⟩ | 1 | ✔ | ⟨✔/✘⟩ |
| M3 | ⟨TBD⟩ | TIM⟨n⟩_CH⟨n⟩ | 2 | ✔ | ⟨✔/✘⟩ |
| M4 | ⟨TBD⟩ | TIM⟨n⟩_CH⟨n⟩ | 2 | ✔ | ⟨✔/✘⟩ |
| LED | ⟨TBD⟩ | TIM⟨n⟩_CH⟨n⟩ | 3 | — | — |

Motor order follows ArduPilot **Quad-X** (`FRAME_CLASS=1`, `FRAME_TYPE=1`) unless otherwise stated.

---

## Building ArduPilot

### 1. Prerequisites (Ubuntu 22.04 / 24.04)

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

macOS: `Tools/environment_install/install-prereqs-mac.sh`.
Windows: use WSL2 with the Ubuntu steps.

### 2. Add the board definition

Copy the hwdef from this repo into the ArduPilot tree:

```bash
cp -r <this-repo>/ardupilot/hwdef/NanoH743AIO \
      libraries/AP_HAL_ChibiOS/hwdef/
```

The directory must contain:
- `hwdef.dat`: main firmware definition
- `hwdef-bl.dat`: bootloader definition
- `README.md` (optional): rendered on the ArduPilot wiki if upstreamed
- `defaults.parm` (optional): board default parameters

### 3. Build the bootloader (first time / bootloader changes only)

```bash
Tools/scripts/build_bootloaders.py NanoH743AIO
# Output: Tools/bootloaders/NanoH743AIO_bl.bin / .hex
```

### 4. Configure and build firmware

```bash
./waf configure --board NanoH743AIO
./waf copter          # or: plane, rover, heli, sub
```

Artifacts in `build/NanoH743AIO/bin/`:

| File | Use |
|---|---|
| `arducopter.apj` | Upload via Mission Planner / QGC / `--upload` (requires ArduPilot bootloader) |
| `arducopter_with_bl.hex` | First-time flash via DFU / STM32CubeProgrammer (includes bootloader) |
| `arducopter.bin` | Raw image (offset `0x08020000` on H743 with 128 KB bootloader) |

### Useful variants

```bash
./waf configure --board NanoH743AIO --debug        # debug symbols, -Og
./waf configure --board NanoH743AIO --enable-dds   # ROS 2 / DDS (if flash permits)
./waf clean && ./waf copter                         # clean rebuild
./waf copter --upload                               # build + upload over USB
```

---

## Flashing

### A. First-time flash (board has Betaflight/INAV or is blank)

1. Hold the **BOOT** button (or bridge the BOOT pad) and plug in USB.
   The board enumerates as `STM32 BOOTLOADER` (ROM DFU, VID:PID `0483:df11`).
2. Flash the combined image using one of the options below.

**Option 1: STM32CubeProgrammer (Windows/macOS/Linux)**
```text
Connect → USB → Open file: arducopter_with_bl.hex → Download
```

**Option 2: dfu-util (Linux/macOS)**
```bash
# Convert hex → bin if needed
arm-none-eabi-objcopy -I ihex -O binary arducopter_with_bl.hex arducopter_with_bl.bin

dfu-util -a 0 --dfuse-address 0x08000000:leave -D arducopter_with_bl.bin
```

**Option 3: Betaflight/INAV Configurator**
Load `arducopter_with_bl.hex` with *Full chip erase* enabled.

> ⚠️ Windows: if the device isn't detected in DFU mode, install the WinUSB driver with **ImpulseRC Driver Fixer** or **Zadig**.

### B. Updates (ArduPilot bootloader already installed)

- **Mission Planner:** *Setup → Install Firmware → Load custom firmware* → `arducopter.apj`
- **QGroundControl:** *Vehicle Setup → Firmware → Advanced → Custom firmware file*
- **CLI:**
  ```bash
  ./waf copter --upload
  # or
  Tools/scripts/uploader.py build/NanoH743AIO/bin/arducopter.apj
  ```

### C. Updating only the bootloader
From a running ArduPilot: Mission Planner → *Config → Full Parameter List* is not used;
instead use **MAVFTP / Mission Planner → "Bootloader update"**, or reflash `*_with_bl.hex` via DFU.

### D. Recovery
If the board does not boot, enter ROM DFU (BOOT pad) and reflash `*_with_bl.hex`.
ROM DFU is in system memory and cannot be erased by user firmware.

---

## Thermal Specifications

### Silicon limits (from ST datasheet DS12110, verify against your exact part suffix)

| Parameter | Value |
|---|---|
| MCU ambient operating range | Suffix **6**: −40 °C to +85 °C · Suffix **3**: −40 °C to +125 °C ⟨verify ordering code⟩ |
| MCU max junction temperature (T<sub>J</sub>) | ⟨per datasheet for fitted suffix⟩ |
| ArduPilot MCU temp telemetry | `MCU_TEMP` in `POWR`/`MCU` log messages; `HAL_WITH_MCU_MONITORING` |

### Board-level measurements (characterize and fill in)

| Condition | MCU (°C) | IMU (°C) | ESC FETs (°C) | BEC (°C) | Ambient |
|---|---|---|---|---|---|
| Idle, USB powered, no airflow | ⟨TBD⟩ | ⟨TBD⟩ | — | ⟨TBD⟩ | 25 °C |
| Armed idle, ⟨n⟩S, no airflow | ⟨TBD⟩ | ⟨TBD⟩ | ⟨TBD⟩ | ⟨TBD⟩ | 25 °C |
| Hover, ⟨TBD⟩ A total | ⟨TBD⟩ | ⟨TBD⟩ | ⟨TBD⟩ | ⟨TBD⟩ | 25 °C |
| Sustained ⟨TBD⟩ A/ch, 60 s | ⟨TBD⟩ | ⟨TBD⟩ | ⟨TBD⟩ | ⟨TBD⟩ | 25 °C |

**Test method:** ⟨thermocouple / IR camera⟩, steady state after ⟨n⟩ min, propwash ⟨present/absent⟩.

### Design guidance
- **ESC continuous ratings assume airflow.** On the bench with no props, keep throttle runs short. FETs can exceed safe temperature within seconds.
- **IMU temperature drift:** if `BRD_HEAT_TARG` / IMU heater is supported (⟨TBD⟩), set it to a value above expected ambient for consistent calibration.
- **Run temperature calib