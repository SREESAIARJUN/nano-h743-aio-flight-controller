I don't have access to the specific schematic, hwdef, or test data for `nano-h743-aio-flight-controller`, so I can't verify its actual pin assignments or thermal measurements. Below is a complete README built on real STM32H743 and ArduPilot conventions. Every board-specific value is marked **`⟨VERIFY⟩`** so that no unverified numbers ship as fact. Replace these with values from your schematic, `hwdef.dat`, and bench tests.

---

```markdown
# Nano H743 AIO Flight Controller

![Board Render](docs/img/board-top.png)

A compact all-in-one (AIO) flight controller built around the **STM32H743** MCU,
designed for ArduPilot (Copter, Plane, Rover, Sub) on small multirotors and
lightweight fixed-wing airframes.

> **Status:** ⟨VERIFY: Prototype / Beta / Production⟩ · **HW Revision:** ⟨VERIFY: rev X.Y⟩ · **Doc Cycle:** 19

---

## Table of Contents
1. [Specifications](#specifications)
2. [Board Layout](#board-layout)
3. [Pinout](#pinout)
4. [UART Mapping](#uart-mapping)
5. [PWM / Motor Outputs](#pwm--motor-outputs)
6. [Power & Battery Monitoring](#power--battery-monitoring)
7. [Thermal Specifications](#thermal-specifications)
8. [Building ArduPilot](#building-ardupilot)
9. [Flashing](#flashing)
10. [First-Time Setup](#first-time-setup)
11. [Troubleshooting](#troubleshooting)
12. [Repository Structure](#repository-structure)
13. [Contributing](#contributing)
14. [License](#license)

---

## Specifications

| Parameter            | Value |
|----------------------|-------|
| MCU                  | STM32H743VIT6 (Arm Cortex-M7, 480 MHz, 2 MB flash, 1 MB RAM) ⟨VERIFY package⟩ |
| IMU                  | ⟨VERIFY: e.g., ICM-42688-P⟩ (SPI) |
| Barometer            | ⟨VERIFY: e.g., DPS310 / BMP280⟩ (I2C/SPI) |
| OSD                  | ⟨VERIFY: AT7456E analog / none⟩ |
| Blackbox / Logging   | ⟨VERIFY: microSD / onboard flash, size⟩ |
| Integrated ESC       | ⟨VERIFY: 4-in-1, XX A continuous / XX A burst, BLHeli_32 / AM32 / Bluejay⟩ |
| Input Voltage        | ⟨VERIFY: 2S–6S LiPo (7.4–25.2 V)⟩ |
| BEC Outputs          | 5 V @ ⟨VERIFY⟩ A, 10 V @ ⟨VERIFY⟩ A (VTX/camera) |
| UARTs                | ⟨VERIFY count⟩ |
| PWM Outputs          | ⟨VERIFY count⟩ (DShot-capable on motor groups) |
| I2C                  | ⟨VERIFY count⟩ (compass / external sensors) |
| CAN                  | ⟨VERIFY: none / 1× FDCAN⟩ |
| USB                  | ⟨VERIFY: USB-C⟩ |
| Mounting             | ⟨VERIFY: 20×20 mm / 25.5×25.5 mm, M2 / M3⟩ |
| Dimensions           | ⟨VERIFY⟩ mm |
| Weight               | ⟨VERIFY⟩ g |
| ArduPilot board name | `NanoH743AIO` ⟨VERIFY matches hwdef directory⟩ |

---

## Board Layout

| Top | Bottom |
|-----|--------|
| ![Top](docs/img/pinout-top.png) | ![Bottom](docs/img/pinout-bottom.png) |

> Silkscreen labels are authoritative. If this document and the silkscreen disagree, open an issue.

---

## Pinout

### Solder Pads

| Pad Label | Function | MCU Pin | Notes |
|-----------|----------|---------|-------|
| `BAT+` | Battery input | — | Main power, ⟨VERIFY range⟩ |
| `GND` | Ground | — | Multiple pads |
| `5V` | 5 V BEC out | — | ⟨VERIFY⟩ A max |
| `10V` | 10 V BEC out | — | VTX/camera, ⟨VERIFY⟩ A max |
| `4V5` | USB-backed 5 V | — | Receiver power when on USB only |
| `TX1` / `RX1` | UART1 | ⟨VERIFY PA9/PA10⟩ | |
| `TX2` / `RX2` | UART2 | ⟨VERIFY⟩ | |
| `TX3` / `RX3` | UART3 | ⟨VERIFY⟩ | |
| `TX4` / `RX4` | UART4 | ⟨VERIFY⟩ | |
| `SCL` / `SDA` | I2C1 | ⟨VERIFY PB8/PB9⟩ | External compass |
| `LED` | WS2812 strip | ⟨VERIFY⟩ | |
| `BZ+` / `BZ-` | Buzzer | ⟨VERIFY⟩ | |
| `CAM` | Camera video in | — | Analog OSD |
| `VTX` | Video out | — | Analog OSD |
| `CURR` | Current sense | ⟨VERIFY ADC pin⟩ | |
| `RSSI` | Analog RSSI | ⟨VERIFY ADC pin⟩ | |
| `S5`–`S8` | Aux PWM | ⟨VERIFY⟩ | Servos / extra motors |

### Connectors

| Connector | Type | Pin 1 → N |
|-----------|------|-----------|
| GPS/Compass | ⟨VERIFY: JST-SH 6-pin⟩ | 5V, TX, RX, SCL, SDA, GND |
| Receiver | ⟨VERIFY⟩ | ⟨VERIFY⟩ |
| VTX (DJI/HDZero) | ⟨VERIFY: JST-SH 6-pin⟩ | 10V, GND, TX, RX, GND, SBUS |

---

## UART Mapping

ArduPilot `SERIALn` order is set by `SERIAL_ORDER` in `hwdef.dat`.

| ArduPilot Port | Hardware | Default Protocol | Default Use | DMA |
|----------------|----------|------------------|-------------|-----|
| `SERIAL0` | USB (OTG1) | MAVLink2 | GCS | — |
| `SERIAL1` | UART1 | MAVLink2 (`2`) | Telemetry | ⟨VERIFY⟩ |
| `SERIAL2` | UART2 | MAVLink2 (`2`) | Telemetry / DJI OSD | ⟨VERIFY⟩ |
| `SERIAL3` | UART3 | GPS (`5`) | GPS | ⟨VERIFY⟩ |
| `SERIAL4` | UART4 | GPS (`5`) | Secondary GPS | ⟨VERIFY⟩ |
| `SERIAL5` | UART6 | RCIN (`23`) | Receiver | ⟨VERIFY⟩ |
| `SERIAL6` | UART7 | ESC Telem (`16`) | ESC telemetry | ⟨VERIFY⟩ |

**Receivers:** connect CRSF/ELRS/SBUS to the RCIN UART. For CRSF/ELRS, also set `RSSI_TYPE = 3`.

---

## PWM / Motor Outputs

| Output | MCU Pin | Timer | Group | DShot | BiDir DShot |
|--------|---------|-------|-------|-------|-------------|
| M1 | ⟨VERIFY⟩ | ⟨TIMx_CHy⟩ | 1 | ✅ | ⟨VERIFY⟩ |
| M2 | ⟨VERIFY⟩ | ⟨TIMx_CHy⟩ | 1 | ✅ | ⟨VERIFY⟩ |
| M3 | ⟨VERIFY⟩ | ⟨TIMx_CHy⟩ | 1 | ✅ | ⟨VERIFY⟩ |
| M4 | ⟨VERIFY⟩ | ⟨TIMx_CHy⟩ | 1 | ✅ | ⟨VERIFY⟩ |
| S5 | ⟨VERIFY⟩ | ⟨TIMx_CHy⟩ | 2 | ✅ | ⟨VERIFY⟩ |
| S6 | ⟨VERIFY⟩ | ⟨TIMx_CHy⟩ | 2 | ✅ | ⟨VERIFY⟩ |
| S7 | ⟨VERIFY⟩ | ⟨TIMx_CHy⟩ | 3 | ✅ | ⟨VERIFY⟩ |
| S8 | ⟨VERIFY⟩ | ⟨TIMx_CHy⟩ | 3 | ✅ | ⟨VERIFY⟩ |
| LED | ⟨VERIFY⟩ | ⟨TIMx_CHy⟩ | 4 | — | — |

> ⚠️ All outputs in one timer group must use the same protocol and rate.
> For example, you cannot mix DShot and PWM servos within one group.

---

## Power & Battery Monitoring

| Parameter | Value |
|-----------|-------|
| `BATT_MONITOR` | `4` (Analog voltage + current) |
| `BATT_VOLT_PIN` | ⟨VERIFY ADC channel⟩ |
| `BATT_CURR_PIN` | ⟨VERIFY ADC channel⟩ |
| `BATT_VOLT_MULT` | ⟨VERIFY: from divider ratio, e.g., 11.0⟩ |
| `BATT_AMP_PERVLT` | ⟨VERIFY: from shunt/amp calibration⟩ |

**Calibration:** fly or bench-load a known pack. Compare the measured mAh with the charger's
reported mAh, then adjust `BATT_AMP_PERVLT` proportionally.

---

## Thermal Specifications

> Component limits below come from the manufacturer datasheets. **Board-level figures must be
> measured** on your hardware (see the test method below) before you publish them.

### Component Limits (datasheet)

| Component | Parameter | Limit | Source |
|-----------|-----------|-------|--------|
| STM32H743 (temp-range suffix 6) | Ambient operating | −40 °C to +85 °C | ST datasheet DS12110 |
| STM32H743 | Max junction temp | ⟨VERIFY per suffix: 105/125 °C⟩ | ST datasheet |
| IMU | Operating range | ⟨VERIFY⟩ | IMU datasheet |
| ESC MOSFETs | Max junction temp | ⟨VERIFY, typ. 150–175 °C⟩ | MOSFET datasheet |
| BEC regulator | Thermal shutdown | ⟨VERIFY⟩ | Regulator datasheet |

### Board-Level Ratings (measured)

| Condition | Value |
|-----------|-------|
| Recommended ambient operating range | ⟨VERIFY⟩ °C |
| MCU temp at idle, 25 °C ambient, no airflow | ⟨VERIFY⟩ °C |
| ESC temp at XX A continuous, prop-wash airflow | ⟨VERIFY⟩ °C |
| ESC temp at XX A continuous, **no airflow** (bench) | ⟨VERIFY⟩ °C, time to limit ⟨VERIFY⟩ s |
| 10 V BEC at full load | ⟨VERIFY⟩ °C |

### Test Method
1. Hold ambient at 25 °C ± 2 °C. Mount the board in its intended frame.
2. Log the MCU temperature (`MCU_TEMP` in the `POWR` log message) and IMU temperature (`IMU.T`).
3. Place type-K thermocouples on the ESC FETs and BEC IC, attached with Kapton tape.
4. Apply a stepped load: 25 %, 50 %, 75 %, and 100 % of rated current, holding each step for 60 s.
   Use propeller airflow, then repeat with no airflow.
5. Record steady-state values and time-to-limit. Store raw data in `docs/thermal/`.

### Thermal Guidance
- Do not run motors on the bench without airflow at high throttle. AIO ESCs rely on prop wash for cooling.
- Use the soft-mounting grommets provided. Do not pot or fully enclose the board without a thermal analysis.
- To get stable gyro bias in cold environments, consider `INS_TCAL_OPTIONS` / temperature calibration.

---

## Building ArduPilot

### Prerequisites
Follow the official environment setup: <https://ardupilot.org/dev/docs/building-the-code.html>

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

### Add the Board Definition
Copy this repo's hwdef into the ArduPilot tree:

```bash
cp -r <this-repo>/hwdef/NanoH743AIO libraries/AP_HAL_ChibiOS/hwdef/
```

The directory should contain:
```
NanoH743AIO/
├── hwdef.dat        # Main firmware definition
├── hwdef-bl.dat     # Bootloader definition
├── defaults.parm    # Default parameters
└── README.md        # ArduPilot-style board doc
```

### Build Firmware

```bash
./waf configure --board NanoH743AIO
./waf copter        # or: plane, rover, sub, heli
```

The output is written to `build/NanoH743AIO/bin/`:
| File | Purpose |
|------|---------|
| `arducopter.apj` | Upload via a ground control station (requires an ArduPilot bootloader) |
| `arducopter_with_bl.hex` | Full image, flash via DFU or SWD |
| `arducopter.bin` | Raw binary |

### Build the Bootloader

```bash
./waf configure --board NanoH743AIO --bootloader
./waf bootloader
# Or, the canonical method, which updates Tools/bootloaders/:
Tools/scripts/build_bootloaders.py NanoH743AIO
```

> After building the bootloader, re-run `./waf configure --board NanoH743AIO` (without
> `--bootloader`) and rebuild the main firmware so that `_with_bl.hex` embeds the new bootloader.

---

## Flashing

### Method 1: First Flash via DFU (no ArduPilot bootloader yet)
1. Hold the **BOOT** button and plug in USB. The board enumerates as `STM32 BOOTLOADER`.
2. Flash using one of the following tools.

   **STM32CubeProgrammer (recommended):**
   ```bash
   STM32_Programmer_CLI -c port=USB1 -w arducopter_with_bl.hex -v -rst
   ```

   **dfu-util (convert hex → bin first, base address 0x08000000):**
   ```bash
   arm-none-eabi