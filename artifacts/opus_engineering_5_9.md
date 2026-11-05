I don't have access to the `nano-h743-aio-flight-controller` repository or its schematics, and I don't know what "Cycle 5" refers to in your workflow. Rather than invent pin assignments or thermal numbers, which could cause miswired peripherals or damaged hardware, I've written a production-grade README in which:

- **Verified generic facts** are filled in: STM32H743 specs, ArduPilot build and flash workflow, DFU procedure.
- **Board-specific values** are marked `⟨TBD⟩` and must come from your schematic, `hwdef.dat`, and test data.

Paste in your hwdef or schematic net list and I can populate the pinout tables precisely.

---

````markdown
# Nano H743 AIO Flight Controller

![Status](https://img.shields.io/badge/status-⟨TBD: alpha|beta|production⟩-blue)
![Firmware](https://img.shields.io/badge/firmware-ArduPilot-green)
![MCU](https://img.shields.io/badge/MCU-STM32H743-orange)
![License](https://img.shields.io/badge/license-⟨TBD⟩-lightgrey)

A compact all-in-one (AIO) flight controller built around the STMicroelectronics
STM32H743 MCU, with an integrated ⟨TBD: 4-in-1 ESC rated N A⟩, designed for
⟨TBD: 2–4" / 3–6S⟩ multirotor platforms running ArduPilot.

> **Revision:** Hardware ⟨TBD: rev X.Y⟩ · Document cycle 5 · Last updated ⟨YYYY-MM-DD⟩

---

## Table of Contents
1. [Specifications](#specifications)
2. [Board Layout](#board-layout)
3. [Pinout](#pinout)
4. [UART Mapping](#uart-mapping)
5. [Building ArduPilot](#building-ardupilot)
6. [Flashing](#flashing)
7. [Thermal Specifications](#thermal-specifications)
8. [Power](#power)
9. [Initial Configuration](#initial-configuration)
10. [Troubleshooting](#troubleshooting)
11. [Repository Structure](#repository-structure)
12. [Contributing](#contributing)
13. [License](#license)

---

## Specifications

| Parameter            | Value                                                        |
|----------------------|--------------------------------------------------------------|
| MCU                  | STM32H743 (Arm Cortex-M7, 480 MHz, 2 MB flash, 1 MB RAM)     |
| IMU                  | ⟨TBD: e.g. ICM-42688-P⟩ on SPI⟨n⟩                            |
| Barometer            | ⟨TBD: e.g. DPS310 / BMP280⟩ on I2C⟨n⟩ / SPI⟨n⟩               |
| OSD                  | ⟨TBD: AT7456E / none⟩                                        |
| Onboard flash/logging| ⟨TBD: W25Q128 16 MB / microSD⟩                               |
| ESC                  | ⟨TBD: 4-in-1, N A cont. / M A burst, BLHeli_S / AM32⟩        |
| Input voltage        | ⟨TBD: 2–6S (7.4–25.2 V)⟩                                     |
| BEC outputs          | ⟨TBD: 5 V @ x A, 9/10 V @ y A, 3.3 V @ z mA⟩                 |
| Current sensor       | ⟨TBD: scale A/V, offset⟩                                     |
| UARTs                | ⟨TBD: n⟩                                                     |
| Motor outputs        | ⟨TBD: 4 (DShot capable)⟩                                     |
| Mounting             | ⟨TBD: 25.5 × 25.5 mm, M2⟩                                    |
| Dimensions / weight  | ⟨TBD: L × W × H mm, g⟩                                       |
| USB                  | ⟨TBD: USB-C⟩                                                 |

---

## Board Layout

```
⟨TBD: Insert top/bottom silkscreen render or ASCII diagram.
 Store images in docs/img/ and reference them here:⟩
![Top](docs/img/top.png)
![Bottom](docs/img/bottom.png)
```

---

## Pinout

> ⚠️ All values must match `hwdef/NanoH743AIO/hwdef.dat`. Treat hwdef as the
> single source of truth. Update this table in the same commit as any hwdef change.

### MCU Peripheral Assignment

| Function       | MCU Pin | Peripheral    | Notes                    |
|----------------|---------|---------------|--------------------------|
| IMU SCK        | ⟨P?⟩    | SPI⟨n⟩_SCK    |                          |
| IMU MISO       | ⟨P?⟩    | SPI⟨n⟩_MISO   |                          |
| IMU MOSI       | ⟨P?⟩    | SPI⟨n⟩_MOSI   |                          |
| IMU CS         | ⟨P?⟩    | GPIO          |                          |
| IMU INT/DRDY   | ⟨P?⟩    | EXTI          |                          |
| Baro SDA/SCL   | ⟨P?/P?⟩ | I2C⟨n⟩        |                          |
| OSD CS         | ⟨P?⟩    | SPI⟨n⟩        |                          |
| Flash CS       | ⟨P?⟩    | SPI⟨n⟩        |                          |
| VBAT sense     | ⟨P?⟩    | ADC⟨n⟩_INP⟨x⟩ | Divider ⟨TBD⟩            |
| Current sense  | ⟨P?⟩    | ADC⟨n⟩_INP⟨x⟩ |                          |
| RSSI (analog)  | ⟨P?⟩    | ADC           | Optional                 |
| LED (status)   | ⟨P?⟩    | GPIO          |                          |
| Buzzer         | ⟨P?⟩    | TIM⟨n⟩_CH⟨x⟩  |                          |
| WS2812 LED     | ⟨P?⟩    | TIM⟨n⟩_CH⟨x⟩  | DMA required             |

### Motor Outputs

| Output | MCU Pin | Timer        | DMA | Group  |
|--------|---------|--------------|-----|--------|
| M1     | ⟨P?⟩    | TIM⟨n⟩_CH⟨x⟩ | ✔︎   | ⟨1⟩    |
| M2     | ⟨P?⟩    | TIM⟨n⟩_CH⟨x⟩ | ✔︎   | ⟨1⟩    |
| M3     | ⟨P?⟩    | TIM⟨n⟩_CH⟨x⟩ | ✔︎   | ⟨2⟩    |
| M4     | ⟨P?⟩    | TIM⟨n⟩_CH⟨x⟩ | ✔︎   | ⟨2⟩    |

> Outputs sharing a timer must use the same protocol (e.g. all DShot600).

### External Connectors

| Connector | Pin 1 | Pin 2 | Pin 3 | Pin 4 | Pin 5 | Pin 6 |
|-----------|-------|-------|-------|-------|-------|-------|
| VTX       | ⟨TBD⟩ |       |       |       |       |       |
| Receiver  | ⟨TBD⟩ |       |       |       |       |       |
| GPS       | ⟨TBD⟩ |       |       |       |       |       |
| Camera    | ⟨TBD⟩ |       |       |       |       |       |

---

## UART Mapping

| ArduPilot Serial | Hardware | Default Protocol      | Default Use        | DMA |
|------------------|----------|-----------------------|--------------------|-----|
| SERIAL0          | USB      | MAVLink2              | GCS                | —   |
| SERIAL1          | ⟨UARTn⟩  | ⟨TBD⟩                 | ⟨TBD: VTX / DJI⟩   | ⟨?⟩ |
| SERIAL2          | ⟨UARTn⟩  | ⟨TBD⟩                 | ⟨TBD⟩              | ⟨?⟩ |
| SERIAL3          | ⟨UARTn⟩  | GPS (5)               | GPS                | ⟨?⟩ |
| SERIAL4          | ⟨UARTn⟩  | ⟨TBD⟩                 | ⟨TBD⟩              | ⟨?⟩ |
| SERIAL5          | ⟨UARTn⟩  | RCIN (23)             | Receiver           | ⟨?⟩ |
| SERIAL6          | ⟨UARTn⟩  | ESC Telemetry (16)    | ESC telemetry      | ⟨?⟩ |

---

## Building ArduPilot

### Prerequisites

Linux (Ubuntu 22.04 LTS recommended), macOS, or WSL2.

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

### Add the board definition (if not yet upstream)

```bash
cp -r <this-repo>/hwdef/NanoH743AIO libraries/AP_HAL_ChibiOS/hwdef/
```

The directory must contain:

| File             | Purpose                                         |
|------------------|-------------------------------------------------|
| `hwdef.dat`      | Main firmware hardware definition               |
| `hwdef-bl.dat`   | Bootloader hardware definition                  |
| `defaults.parm`  | Board default parameters (optional)             |
| `README.md`      | Upstream-format board docs (required for PR)    |

### Build the bootloader

```bash
Tools/scripts/build_bootloaders.py NanoH743AIO
# Output: Tools/bootloaders/NanoH743AIO_bl.bin / .hex
```

### Build the firmware

```bash
./waf configure --board NanoH743AIO
./waf copter            # ArduCopter
# ./waf plane           # ArduPlane
# ./waf rover           # Rover
```

Outputs are in `build/NanoH743AIO/bin/`:

| File                       | Use                                    |
|----------------------------|----------------------------------------|
| `arducopter.apj`           | Mission Planner / QGC upload           |
| `arducopter_with_bl.hex`   | First-time DFU flash (bootloader + FW) |
| `arducopter.bin`           | Raw binary                             |

### Clean rebuild

```bash
./waf distclean
./waf configure --board NanoH743AIO
./waf copter
```

---

## Flashing

### Method A — First flash via DFU (no ArduPilot bootloader present)

1. Disconnect the battery.
2. Hold the **BOOT** button, plug in USB, and release BOOT.
3. Verify that DFU mode is detected:
   ```bash
   dfu-util -l
   # Expect: Found DFU: [0483:df11] ... "@Internal Flash"
   ```
4. Flash the combined image. Choose one:
   - **STM32CubeProgrammer (recommended for .hex):**
     ```bash
     STM32_Programmer_CLI -c port=USB1 -w arducopter_with_bl.hex -v -rst
     ```
   - **dfu-util (.bin with bootloader):**
     ```bash
     dfu-util -a 0 -s 0x08000000:leave -D arducopter_with_bl.bin
     ```
5. Unplug, then reconnect. The status LED should show ⟨TBD pattern⟩.

> **Note:** If a previous Betaflight/INAV install left flash protection enabled,
> perform a full chip erase in STM32CubeProgrammer first.

### Method B — Updates (ArduPilot bootloader present)

```bash
./waf copter --upload
```

Alternatively, use Mission Planner (**Setup → Install Firmware → Load custom firmware**) or
QGroundControl to load `arducopter.apj`.

### Recovery

If the board does not enumerate, re-enter DFU using the BOOT button and repeat Method A.
The bootloader resides at `0x08000000` (⟨TBD: 128 KB⟩ sector on H743).

---

## Thermal Specifications

### Component limits (datasheet values — verify against your exact part numbers)

| Component        | Parameter                    | Limit                    | Source               |
|------------------|------------------------------|--------------------------|----------------------|
| STM32H743        | Ambient operating (suffix 6) | −40 to +85 °C            | ST DS12110           |
| STM32H743        | Max junction                 | ⟨verify: 125 °C / grade⟩ | ST DS12110           |
| IMU              | Operating range              | ⟨TBD⟩                    | Vendor datasheet     |
| ESC MOSFETs      | Max junction                 | ⟨TBD, typ. 150–175 °C⟩   | Vendor datasheet     |
| BEC regulator    | Thermal shutdown             | ⟨TBD⟩                    | Vendor datasheet     |

### Measured board performance (fill in from test campaign)

| Condition                              | Hotspot      | Steady-state temp | Ambient | Airflow |
|----------------------------------------|--------------|-------------------|---------|---------|
| Idle on USB                            | MCU          | ⟨TBD⟩ °C          | 25 °C   | None    |
| Armed, hover (⟨x⟩ A total)             | ESC FETs     | ⟨TBD⟩ °C          | 25 °C   | Prop wash |
| Continuous rated current (⟨N⟩ A/ch)    | ESC FETs     | ⟨TBD⟩ °C          | 25 °C   | ⟨TBD⟩ m/s |
| Burst (⟨M⟩ A, ⟨t⟩ s)                   | ESC FETs     | ⟨TBD⟩ °C          | 25 °C   | ⟨TBD⟩ m/s |
| 5 V BEC at full load                   | Regulator    | ⟨TBD⟩ °C          | 25 °C   | None    |
| VTX powered on bench (no airflow)      | Board        | ⟨TBD⟩ °C          | 25 °C   | None    |

**Test method:** ⟨TBD: thermocouple type, IR camera model, emissivity setting,
dwell time to steady state, number of samples⟩.

### Design guidance

- Rated ESC current assumes **in-flight prop-wash cooling**. Do not run sustained
  high throttle on the bench.
- IMU temperature drift is mitigated by ⟨TBD: board heater via `BRD_HEAT_TARG` / none⟩.
- Monitor MCU temperature in flight via the `POWR` / `MCU` log messages.
- Derate continuous current by ⟨TBD⟩ % above ⟨TBD⟩ °C ambient.
- Avoid stacking a VTX directly against the ESC section without a ≥ ⟨TBD⟩ mm gap.

---

## Power

| Parameter          | Value                       | ArduPilot param        |
|--------------------|-----------------------------|------------------------|
| Voltage scale      | ⟨TBD⟩                       | `BATT_VOLT_MULT`       |
| Current scale      | ⟨TBD⟩ A/V                   | `BATT_AMP_PERVLT`      |
| Current offset     | ⟨TBD⟩ V                     | `BATT_AMP_OFFSET`      |
| Voltage pin        | ⟨TBD⟩                       | `BATT_VOLT_P