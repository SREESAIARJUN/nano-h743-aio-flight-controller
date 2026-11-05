I don't have access to this repository, its schematics, or previous "cycles," so I can't know this board's actual pin assignments, sensor part numbers, or measured thermal data. Rather than invent hardware facts that someone might wire or fly against, I've written a complete README in which:

- **Generic, verifiable content is filled in.** This includes STM32H743 core specs, the ArduPilot/waf build flow, DFU flashing, and hwdef structure.
- **Board-specific values are marked `⟨TBD⟩`.** Fill these in from your schematic, `hwdef.dat`, and bench measurements.

If you paste your `hwdef.dat`, schematic netlist, or previous README draft, I can fill in every placeholder accurately.

---

```markdown
# Nano H743 AIO Flight Controller

> Compact all-in-one (AIO) STM32H743 flight controller with integrated ESC ⟨TBD: yes/no⟩,
> designed for ArduPilot (Copter / Plane / Rover / Sub).

![Board top](docs/img/board_top.png) ![Board bottom](docs/img/board_bottom.png)

| Status | Value |
|---|---|
| Hardware revision | ⟨TBD: e.g. Rev C⟩ |
| ArduPilot board name | `⟨TBD: NanoH743AIO⟩` |
| Board ID (APJ) | ⟨TBD: must be registered in `Tools/AP_Bootloader/board_types.txt`⟩ |
| License (HW / FW) | ⟨TBD: CERN-OHL-S-2.0 / GPLv3⟩ |

---

## Table of Contents
1. [Specifications](#specifications)
2. [Pinout](#pinout)
3. [UART Mapping](#uart-mapping)
4. [PWM / Motor Outputs](#pwm--motor-outputs)
5. [Power & Analog Sensing](#power--analog-sensing)
6. [Thermal Specifications](#thermal-specifications)
7. [Building ArduPilot](#building-arduplot)
8. [Flashing](#flashing)
9. [First-Time Setup](#first-time-setup)
10. [Troubleshooting](#troubleshooting)
11. [Repository Layout](#repository-layout)
12. [Contributing & License](#contributing--license)

---

## Specifications

| Item | Specification |
|---|---|
| MCU | STM32H743VIT6, Arm Cortex-M7 @ 480 MHz, 2 MB flash, 1 MB RAM |
| IMU | ⟨TBD: e.g. ICM-42688-P (SPI)⟩ |
| Secondary IMU | ⟨TBD / none⟩ |
| Barometer | ⟨TBD: e.g. DPS310 / SPL06 (I²C)⟩ |
| Compass | None onboard (external via I²C) ⟨verify⟩ |
| OSD | ⟨TBD: AT7456E (SPI) / none⟩ |
| Blackbox / logging | ⟨TBD: microSD / onboard flash (size)⟩ |
| Integrated ESC | ⟨TBD: 4-in-1, xx A continuous / yy A burst, BLHeli_S / AM32 / BLHeli_32⟩ |
| Input voltage | ⟨TBD: 2–6S LiPo (7–26 V)⟩ |
| BEC outputs | ⟨TBD: 5 V @ x A, 10 V @ y A (VTX)⟩ |
| UARTs | ⟨TBD: count⟩ |
| PWM outputs | ⟨TBD: count⟩ |
| CAN | ⟨TBD: yes/no⟩ |
| Dimensions | ⟨TBD: xx × xx mm⟩ |
| Mounting | ⟨TBD: 25.5 × 25.5 mm, M2⟩ |
| Weight | ⟨TBD: g⟩ |

---

## Pinout

> **Source of truth:** `hwdef/⟨BoardName⟩/hwdef.dat`. If this table and the hwdef
> disagree, the hwdef wins. Please open an issue so the README can be corrected.

### Pad diagram

```
        ┌─────────────────────────────────────┐
  M1 ◉──┤ ⟨TBD: ASCII or link to SVG pinout⟩  ├──◉ M2
        │                                     │
        │                                     │
  M4 ◉──┤                                     ├──◉ M3
        └─────────────────────────────────────┘
```
Full-resolution diagram: [`docs/pinout.svg`](docs/pinout.svg)

### Pad table

| Pad label | MCU pin | Function | Voltage level | Notes |
|---|---|---|---|---|
| ⟨TX1⟩ | ⟨P??⟩ | USART1_TX | 3.3 V | |
| ⟨RX1⟩ | ⟨P??⟩ | USART1_RX | 3.3 V | |
| ⟨SBUS⟩ | ⟨P??⟩ | RC input (inverted) | 3.3 V | |
| ⟨SCL⟩ | ⟨P??⟩ | I2C⟨n⟩_SCL | 3.3 V | External compass |
| ⟨SDA⟩ | ⟨P??⟩ | I2C⟨n⟩_SDA | 3.3 V | |
| ⟨LED⟩ | ⟨P??⟩ | WS2812 / NeoPixel | 3.3 V | |
| ⟨BZ-⟩ | ⟨P??⟩ | Buzzer (low-side) | — | |
| ⟨CURR⟩ | ⟨P??⟩ | Current sense ADC | 0–3.3 V | |
| ⟨VBAT⟩ | ⟨P??⟩ | Battery voltage ADC | divided | |
| 5V / 10V / GND | — | Power | — | |

### Internal SPI / I²C buses

| Bus | Device | CS / Address | Notes |
|---|---|---|---|
| SPI1 | ⟨IMU⟩ | ⟨P??⟩ | DRDY on ⟨P??⟩ |
| SPI2 | ⟨OSD⟩ | ⟨P??⟩ | |
| SPI3 / SDMMC | ⟨SD / flash⟩ | ⟨P??⟩ | |
| I2C⟨n⟩ | ⟨Baro⟩ | ⟨0x76/0x77⟩ | |

---

## UART Mapping

ArduPilot serial ports (`SERIALn_*`) map to hardware as follows:

| ArduPilot | Hardware | Default protocol | Pads / connector |
|---|---|---|---|
| SERIAL0 | USB (OTG1) | MAVLink2 | USB-C |
| SERIAL1 | USART⟨?⟩ | MAVLink2 (telemetry) | ⟨TX1/RX1⟩ |
| SERIAL2 | USART⟨?⟩ | ⟨DJI / MSP DisplayPort⟩ | ⟨VTX connector⟩ |
| SERIAL3 | USART⟨?⟩ | GPS | ⟨TX3/RX3⟩ |
| SERIAL4 | USART⟨?⟩ | ⟨ESC telemetry⟩ | internal |
| SERIAL5 | USART⟨?⟩ | RCIN | ⟨SBUS / RX⟩ |
| SERIAL6 | ⟨…⟩ | ⟨…⟩ | ⟨…⟩ |

All UARTs support DMA unless noted ⟨verify against hwdef `# no DMA` notes⟩.

---

## PWM / Motor Outputs

| Output | MCU pin | Timer | Group | DShot | BiDir DShot |
|---|---|---|---|---|---|
| M1 | ⟨P??⟩ | TIM⟨?⟩_CH⟨?⟩ | 1 | ✔ | ⟨✔/✘⟩ |
| M2 | ⟨P??⟩ | TIM⟨?⟩_CH⟨?⟩ | 1 | ✔ | ⟨✔/✘⟩ |
| M3 | ⟨P??⟩ | TIM⟨?⟩_CH⟨?⟩ | 1 | ✔ | ⟨✔/✘⟩ |
| M4 | ⟨P??⟩ | TIM⟨?⟩_CH⟨?⟩ | 1 | ✔ | ⟨✔/✘⟩ |
| LED | ⟨P??⟩ | TIM⟨?⟩_CH⟨?⟩ | 2 | — | — |

> Outputs that share a timer group must use the same protocol and rate.
> Do not mix DShot and PWM within a group.

---

## Power & Analog Sensing

| Parameter | Default | Notes |
|---|---|---|
| `BATT_MONITOR` | 4 | Analog voltage + current |
| `BATT_VOLT_PIN` | ⟨TBD⟩ | |
| `BATT_CURR_PIN` | ⟨TBD⟩ | |
| `BATT_VOLT_MULT` | ⟨TBD: from divider ratio⟩ | Verify with a multimeter |
| `BATT_AMP_PERVLT` | ⟨TBD: from shunt + amp gain⟩ | Calibrate against a known load |

---

## Thermal Specifications

> ⚠️ All values below must come from **measured bench data** on the current hardware
> revision. Document the test conditions alongside each result.

### Component limits (from datasheets)

| Component | Rated range | Source |
|---|---|---|
| STM32H743**VIT6** (suffix “6”) | Ambient −40 °C to +85 °C | ST datasheet DS12110 |
| IMU ⟨part⟩ | ⟨TBD⟩ | ⟨datasheet⟩ |
| ESC MOSFETs ⟨part⟩ | T<sub>j</sub> max ⟨TBD, often 150–175 °C⟩ | ⟨datasheet⟩ |
| 5 V / 10 V regulators ⟨part⟩ | ⟨TBD⟩, thermal shutdown ⟨TBD⟩ | ⟨datasheet⟩ |

### Measured performance

| Test | Conditions | Result |
|---|---|---|
| MCU idle temp | ⟨ambient °C, no airflow, USB-powered⟩ | ⟨°C⟩ |
| MCU under load | ⟨scheduler loop rate, logging on⟩ | ⟨°C⟩ |
| ESC continuous | ⟨xx A per channel, duration, airflow⟩ | ⟨°C MOSFET case⟩ |
| ESC burst | ⟨yy A for z s⟩ | ⟨°C⟩ |
| 10 V BEC at max load | ⟨load⟩ | ⟨°C⟩ |
| IMU drift vs temp | ⟨range⟩ | ⟨see `docs/thermal/`⟩ |

**Recommended operating envelope:** ⟨TBD, e.g. ambient −10 °C to +50 °C with prop-wash airflow⟩.

**Design guidance:**
- The ESC current ratings assume prop-wash airflow. Bench testing without airflow will overheat the MOSFETs.
- Avoid enclosing the board in foam or tight canopies.
- If IMU heater support is configured (`BRD_HEAT_TARG`), document the target temperature here. ⟨TBD / not supported⟩

Raw thermal logs and IR images: [`docs/thermal/`](docs/thermal/)

---

## Building ArduPilot

### 1. Prerequisites (Ubuntu / WSL2)

```bash
git clone --recursive https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

### 2. Add the hardware definition (if not yet upstream)

```bash
cp -r /path/to/this-repo/hwdef/⟨BoardName⟩ \
      libraries/AP_HAL_ChibiOS/hwdef/⟨BoardName⟩
# Contains: hwdef.dat, hwdef-bl.dat, README.md, defaults.parm (optional)
```

### 3. Build the firmware

```bash
./waf configure --board ⟨BoardName⟩
./waf copter        # or: plane | rover | sub | heli
```

Output files are written to `build/⟨BoardName⟩/bin/`:

| File | Use |
|---|---|
| `arducopter.apj` | Upload via Mission Planner / QGC / `uploader.py` (requires bootloader) |
| `arducopter_with_bl.hex` | Full image (bootloader + firmware) for DFU / ST-Link |
| `arducopter.bin` | Raw application binary |

### 4. Build the bootloader (only needed when hwdef-bl.dat changes)

```bash
Tools/scripts/build_bootloaders.py ⟨BoardName⟩
# Output: Tools/bootloaders/⟨BoardName⟩_bl.bin / .hex
```

You must rebuild the firmware after updating the bootloader, because `*_with_bl.hex` embeds it.

---

## Flashing

### Option A: Initial flash via DFU (blank board or Betaflight/INAV installed)

1. Hold the **BOOT** button ⟨TBD: location⟩ and plug in USB.
2. Confirm the board is detected:
   ```bash
   dfu-util -l     # expect "STM32 BOOTLOADER", 0483:df11
   ```
3. Flash the combined image. The easiest method is STM32CubeProgrammer (GUI or CLI):
   ```bash
   STM32_Programmer_CLI -c port=USB1 -e all -w arducopter_with_bl.hex -v -rst
   ```
   Alternatively, use the **ArduPilot firmware tab in Betaflight Configurator** with the `.hex` file.

   To use dfu-util, convert the hex to a bin first:
   ```bash
   arm-none-eabi-objcopy -I ihex -O binary arducopter_with_bl.hex fw.bin
   dfu-util -a 0 -s 0x08000000:mass-erase:force:leave -D fw.bin
   ```

### Option B: Updates (ArduPilot bootloader already installed)

- **Mission Planner / QGroundControl:** Load custom firmware and select the `.apj` file.
- **From source:**
  ```bash
  ./waf copter --upload
  ```
- **Standalone:**
  ```bash
  Tools/scripts/uploader.py build/⟨BoardName⟩/bin/arducopter.apj
  ```

### Option C: SWD (recovery)

Connect an ST-Link to the SWDIO, SWCLK, GND, and 3V3 pads ⟨TBD: location⟩:
```bash
STM32_Programmer_CLI -c port=SWD -e all -w arducopter_with_bl.hex -v -rst
```

