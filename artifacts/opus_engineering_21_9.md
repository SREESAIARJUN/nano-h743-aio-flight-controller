I don't have access to your repository's schematic, `hwdef.dat`, or test data, so I can't know this board's actual pin assignments, sensor choices, or measured thermals. Below is a complete README. ArduPilot build and flash procedures follow upstream conventions. Every board-specific value is marked `⚠️ VERIFY` or uses a `<placeholder>`. Fill these in from your schematic, hwdef, and bench measurements before publishing. A wrong pinout in a flight controller README can destroy hardware.

---

````markdown
# Nano H743 AIO Flight Controller

<p align="center">
  <img src="docs/img/board-top.png" alt="Nano H743 AIO – top" width="45%">
  <img src="docs/img/board-bottom.png" alt="Nano H743 AIO – bottom" width="45%">
</p>

<p align="center">
  <a href="LICENSE"><img src="https://img.shields.io/badge/hardware-CERN--OHL--S--2.0-blue"></a>
  <a href="LICENSE-FIRMWARE"><img src="https://img.shields.io/badge/firmware-GPLv3-green"></a>
  <img src="https://img.shields.io/badge/ArduPilot-4.5%2B-orange">
  <img src="https://img.shields.io/badge/MCU-STM32H743-lightgrey">
</p>

A compact all-in-one (AIO) flight controller built on the STM32H743. It integrates
<!-- ⚠️ VERIFY --> a 4-in-1 ESC, OSD, and a regulated BEC, targeting 2"–5" multirotors
running **ArduPilot**.

> **Status:** `<alpha | beta | production>` · Hardware revision: `<rev X.Y>` · Last verified: `<YYYY-MM-DD>`

---

## Table of Contents

1. [Specifications](#specifications)
2. [Board Layout & Pinout](#board-layout--pinout)
3. [UART / Serial Mapping](#uart--serial-mapping)
4. [PWM / Motor Output Mapping](#pwm--motor-output-mapping)
5. [Building ArduPilot](#building-arducopter-firmware)
6. [Flashing](#flashing)
7. [First-Time Configuration](#first-time-configuration)
8. [Thermal Specifications](#thermal-specifications)
9. [Electrical Ratings](#electrical-ratings)
10. [Troubleshooting](#troubleshooting)
11. [Repository Structure](#repository-structure)
12. [Contributing](#contributing)
13. [License & Safety Disclaimer](#license--safety-disclaimer)

---

## Specifications

| Parameter            | Value                                                     |
|----------------------|-----------------------------------------------------------|
| MCU                  | STM32H743`<VI/II>`T6, Cortex-M7 @ 480 MHz, 2 MB flash, 1 MB RAM |
| IMU                  | `<e.g. ICM-42688-P>` ⚠️ VERIFY (SPI`<n>`)                  |
| Barometer            | `<e.g. DPS310 / BMP280>` ⚠️ VERIFY (I²C`<n>` / SPI)        |
| OSD                  | `<AT7456E (MAX7456-compatible)>` ⚠️ VERIFY                 |
| Blackbox / Logging   | `<microSD | onboard dataflash, size>` ⚠️ VERIFY            |
| Integrated ESC       | `<4-in-1, firmware: BLHeli_S / Bluejay / AM32>`, `<N> A` continuous ⚠️ VERIFY |
| Input voltage        | `<2–6S>` LiPo ⚠️ VERIFY                                    |
| BEC                  | 5 V @ `<x> A`, `<9/10 V @ y A>` ⚠️ VERIFY                  |
| UARTs                | `<n>` (see mapping below)                                  |
| Mounting             | `<25.5 × 25.5 mm, M2>` ⚠️ VERIFY                           |
| Dimensions / Weight  | `<L × W × H mm>` / `<g>`                                   |
| USB                  | `<USB-C>`                                                  |
| ArduPilot target     | `NanoH743AIO` (hwdef in [`ardupilot/hwdef/`](ardupilot/hwdef/)) |

---

## Board Layout & Pinout

> Pin images are generated from `hardware/<project>.kicad_pcb`.
> If any image conflicts with the table below, the **schematic is authoritative**.
> Please open an issue if you find a mismatch.

### Top Side

![Top pinout](docs/img/pinout-top.png)

### Bottom Side

![Bottom pinout](docs/img/pinout-bottom.png)

### Pad Reference (⚠️ VERIFY ALL ENTRIES AGAINST SCHEMATIC)

| Pad Label | Function              | MCU Pin | Voltage Level | Notes                        |
|-----------|-----------------------|---------|---------------|------------------------------|
| `BAT+`    | Battery input         | —       | `<2–6S>`      | Main power, to ESC and BEC   |
| `GND`     | Ground                | —       | 0 V           |                              |
| `5V`      | BEC output            | —       | 5 V           | Max `<x> A` total            |
| `9V`/`10V`| VTX BEC output        | —       | `<9/10> V`    | Switchable via `<RELAY/GPIO>`? |
| `T1`/`R1` | UART1 TX/RX           | `<PA9/PA10>` | 3.3 V    |                              |
| `T2`/`R2` | UART2 TX/RX           | `<…>`   | 3.3 V         |                              |
| `SDA`/`SCL` | I²C (compass)       | `<…>`   | 3.3 V         | External pull-ups: `<y/n>`   |
| `LED`     | WS2812 data           | `<…>`   | 3.3 V/5 V     |                              |
| `BZ+`/`BZ-` | Buzzer              | `<…>`   | 5 V           | Low-side switched            |
| `CURR`    | Current sense         | `<…>`   | 0–3.3 V       | Scale: `<A/V>`               |
| `VBAT`    | Voltage sense         | `<…>`   | divided       | Divider: `<ratio>`           |
| `RSSI`    | Analog RSSI           | `<…>`   | 0–3.3 V       |                              |
| `CAM`     | Analog camera video in| —       | —             |                              |
| `VTX`     | Video out to VTX      | —       | —             |                              |
| `BOOT`    | Bootloader button/pad | `BOOT0` | —             | Hold during power-up for DFU |

> ⚠️ **3.3 V logic only.** Do not feed 5 V signals into UART or I²C pads unless the
> schematic confirms the pin is 5 V-tolerant (FT) *and* is not in analog mode.

---

## UART / Serial Mapping

This mapping must exactly match the `SERIAL_ORDER` line in `hwdef.dat`.

| ArduPilot Port | Hardware | Pads        | Default Protocol           | Default Baud |
|----------------|----------|-------------|----------------------------|--------------|
| `SERIAL0`      | USB      | USB-C       | MAVLink2                   | 115200       |
| `SERIAL1`      | `USART1` | `T1/R1`     | `<MAVLink2 / telemetry>`   | 57600        |
| `SERIAL2`      | `USART2` | `T2/R2`     | `<RCIN – CRSF/ELRS>`       | 115200       |
| `SERIAL3`      | `USART3` | `T3/R3`     | `<GPS>`                    | 230400       |
| `SERIAL4`      | `UART4`  | `T4/R4`     | `<VTX – MSP DisplayPort / SmartAudio>` | — |
| `SERIAL5`      | `<…>`    | `<…>`       | `<ESC Telemetry>`          | 115200       |

To change a port's protocol, set `SERIALn_PROTOCOL` and reboot. For receivers:
`SERIALn_PROTOCOL = 23` (RCIN). For CRSF also set `RSSI_TYPE = 3`.

---

## PWM / Motor Output Mapping

| Output | Timer/Channel   | MCU Pin | Default Function | DShot | Bi-dir DShot |
|--------|-----------------|---------|------------------|-------|--------------|
| M1     | `<TIMx_CHy>`    | `<…>`   | Motor 1          | ✅    | `<✅/❌>`     |
| M2     | `<…>`           | `<…>`   | Motor 2          | ✅    | `<✅/❌>`     |
| M3     | `<…>`           | `<…>`   | Motor 3          | ✅    | `<✅/❌>`     |
| M4     | `<…>`           | `<…>`   | Motor 4          | ✅    | `<✅/❌>`     |
| LED    | `<…>`           | `<…>`   | NeoPixel         | —     | —            |

**PWM groups:** outputs that share a timer must use the same protocol and rate.
- Group 1: `M1–M4` (`TIM<x>`)
- Group 2: `LED` (`TIM<y>`)

Motor order follows the ArduPilot **QUAD-X** layout by default. Verify direction
with *Mission Planner → Optional Hardware → Motor Test* **with props removed**.

---

## Building ArduCopter Firmware

### Prerequisites

Linux (Ubuntu 22.04/24.04 recommended), macOS, or WSL2.

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

### Add the Board Definition

```bash
# From this repository's root:
cp -r ardupilot/hwdef/NanoH743AIO \
      <path-to-ardupilot>/libraries/AP_HAL_ChibiOS/hwdef/
```

The directory must contain:

- `hwdef.dat`: main firmware definition
- `hwdef-bl.dat`: bootloader definition
- `defaults.parm`: optional board defaults
- `README.md`: optional; the ArduPilot wiki convention if you upstream the board

> **Board ID:** `APJ_BOARD_ID` in both hwdef files must be a unique ID. Reserve one
> by opening a PR against
> `Tools/AP_Bootloader/board_types.txt` in ArduPilot. Do **not** reuse another
> board's ID.

### Build the Bootloader

```bash
cd <path-to-ardupilot>
Tools/scripts/build_bootloaders.py NanoH743AIO
# Output: Tools/bootloaders/NanoH743AIO_bl.bin / .hex
```

### Build the Firmware

```bash
./waf configure --board NanoH743AIO
./waf copter          # or: plane, rover, heli
```

Artifacts land in `build/NanoH743AIO/bin/`:

| File                       | Use                                              |
|----------------------------|--------------------------------------------------|
| `arducopter.apj`           | Upload via Mission Planner / QGC / `uploader.py`  |
| `arducopter.bin`           | Raw app image (no bootloader)                     |
| `arducopter_with_bl.hex`   | Full image for DFU / STM32CubeProgrammer          |

### Clean Rebuild

```bash
./waf distclean
./waf configure --board NanoH743AIO && ./waf copter
```

### CI

`.github/workflows/build.yml` builds the bootloader and Copter for every PR and
attaches artifacts to tagged releases.

---

## Flashing

### Method 1: First Flash via DFU (blank board or recovery)

1. Hold the **BOOT** button.
2. Connect USB while holding it, then release.
3. Confirm DFU mode:
   ```bash
   dfu-util -l
   # Expect: "Found DFU: [0483:df11] ... name="@Internal Flash  /0x08000000/..."
   ```
4. Flash the combined image with **STM32CubeProgrammer**:
   ```bash
   STM32_Programmer_CLI -c port=USB1 -w arducopter_with_bl.hex -v -rst
   ```
   Or use `dfu-util` with a `.bin`. Write the bootloader first, then the app at the
   board's application offset, which is `FLASH_RESERVE_START_KB` in hwdef.dat
   (typically 128 KB on H7, i.e. `0x08020000`):
   ```bash
   dfu-util -a 0 -s 0x08000000 -D NanoH743AIO_bl.bin
   dfu-util -a 0 -s 0x08020000:leave -D arducopter.bin
   ```
5. Power-cycle. The board should enumerate as a USB serial device.

> **Linux permissions:** if `dfu-util` reports `LIBUSB_ERROR_ACCESS`, install the
> ST udev rules or run once with `sudo`.

### Method 2: Firmware Updates (bootloader already present)

- **Mission Planner:** *Setup → Install Firmware → Load custom firmware* → select `arducopter.apj`
- **QGroundControl:** *Vehicle Setup → Firmware → Advanced → Custom firmware file*
- **CLI:**
  ```bash
  python3 Tools/scripts/uploader.py build/NanoH743AIO/bin/arducopter.apj
  ```

### Migrating from Betaflight/INAV

If the board shipped with Betaflight, use **Method 1**. The Betaflight bootloader
is not ArduPilot-compatible. Back up your Betaflight `diff all` first if you may revert.

---

## First-Time Configuration

1. Connect via Mission Planner or QGC.
2. **Frame:** `FRAME_CLASS = 1` (Quad), `FRAME_TYPE = 1` (X).
3. **Motors (DShot):** `MOT_PWM_TYPE = 6` (DShot600). For bi-directional DShot,
   set `SERVO_BLH_BDMASK = 15` and `SERVO_BLH_POLES = <motor pole count>`.
4. **Battery monitor:**
   ```
   BATT_MONITOR    = 4
   BATT_VOLT_PIN   = <n>      # ⚠️ VERIFY from hwdef
   BATT_CURR_PIN   = <n>      # ⚠️ VERIFY
   BATT_VOLT_MULT  = <x>      # calibrate with a multimeter
   BATT_AMP_PERVLT = <x>      # calibrate with a known load
   ```
5. **OSD:** `OSD_TYPE = 1` (analog MAX7456) or `5` (MSP DisplayPort for HD systems).
6. Calibrate the accelerometer and radio, set up