I don't have access to the actual contents of `osmo-pocket-esp32-gimbal` or earlier cycles, so this README is a complete reference design. **Before merging, verify every pin number, part number, and gain value against your schematic and firmware.** Placeholders you'll need to fill in are marked `<!-- TODO -->`.

---

````markdown
# osmo-pocket-esp32-gimbal

> Open-source, pocket-sized 3-axis handheld camera gimbal built on the ESP32-S3,
> using field-oriented control (FOC) of gimbal BLDC motors with absolute magnetic encoders
> and a high-rate 6-axis IMU.

![build](https://img.shields.io/badge/build-passing-brightgreen)
![platform](https://img.shields.io/badge/platform-ESP32--S3-blue)
![framework](https://img.shields.io/badge/framework-Arduino%20%2B%20SimpleFOC-orange)
![license](https://img.shields.io/badge/license-MIT-lightgrey)

> **Trademark notice:** This is an independent hobby project. It is not affiliated with,
> endorsed by, or derived from DJI. "Osmo Pocket" refers only to the target form factor.

---

## Table of Contents

1. [Features](#features)
2. [System Architecture](#system-architecture)
3. [Hardware](#hardware)
4. [Wiring & Pinout](#wiring--pinout)
5. [Bill of Materials](#bill-of-materials)
6. [Firmware](#firmware)
7. [Build & Flash](#build--flash)
8. [First Power-Up Checklist](#first-power-up-checklist)
9. [FOC Tuning Procedure](#foc-tuning-procedure)
10. [Stabilization Loop Tuning](#stabilization-loop-tuning)
11. [Serial Command Reference](#serial-command-reference)
12. [Troubleshooting](#troubleshooting)
13. [Safety](#safety)
14. [Roadmap](#roadmap)
15. [Contributing](#contributing)
16. [License](#license)

---

## Features

- **3-axis stabilization** (pan / tilt / roll) with FOC on every axis
- **Absolute encoders** (AS5048A, 14-bit SPI): no homing sweep, and the gimbal holds position at power-on
- **1 kHz stabilization loop** and **≥ 5 kHz FOC loop**, pinned to separate cores
- **ICM-42688-P IMU** with a Mahony attitude filter (quaternion, gyro-bias estimation)
- **Modes:** Follow, Pan-Follow, Lock, FPV, Recenter (double-tap)
- **2S Li-ion** power with brown-out detection and low-battery throttling
- **Wireless control:** BLE GATT service for app/remote control; Wi-Fi tuning UI (optional build flag)
- **Persistent calibration** in NVS: encoder offsets, IMU bias, and PID gains

---

## System Architecture

### Top-level block diagram

```
                         ┌───────────────────────────────────────────┐
   2S Li-ion 7.4 V ──┬──►│  Power Board                              │
   (USB-C PD charge) │   │  • BMS / protection                       │
                     │   │  • Buck 7.4 V → 3.3 V (logic, 1 A)        │
                     │   │  • VBAT divider → ADC                     │
                     │   └──────┬───────────────────────┬────────────┘
                     │          │ 3V3                   │ VMOT (VBAT)
                     │          ▼                       ▼
┌────────────┐  SPI  │   ┌──────────────┐  3×PWM ┌──────────────┐   ┌──────────┐
│ ICM-42688-P│◄──────┼──►│              │───────►│ DRV8313 (Yaw)│──►│ Yaw BLDC │
│    IMU     │──INT─►│   │              │  3×PWM ├──────────────┤   ├──────────┤
└────────────┘       │   │   ESP32-S3   │───────►│ DRV8313 (Pit)│──►│ Pitch    │
                     │   │   (WROOM-1)  │  3×PWM ├──────────────┤   ├──────────┤
┌────────────┐  SPI  │   │              │───────►│ DRV8313 (Rol)│──►│ Roll     │
│ AS5048A ×3 │◄──────┴──►│ Core 0: comms│        └──────────────┘   └──────────┘
│ (1/axis)   │           │ Core 1: ctrl │                                 ▲
└────────────┘           └──┬────┬────┬─┘           diametric magnet ───┘
                            │    │    │              on each motor shaft
                     Button─┘    │    └─ USB-C (native USB CDC / JTAG)
                          WS2812 LED
```

### Control-loop hierarchy

```
 User / Mode logic          Stabilization (1 kHz, Core 1)          FOC (≥5 kHz, Core 1)
┌───────────────┐   q_ref  ┌──────────────────────────────┐ θ_ref ┌───────────────────┐
│ Joystick/BLE  │─────────►│ Attitude error (quaternion)  │──────►│ Angle P           │
│ Follow filter │          │  → per-axis PID              │       │  → Velocity PI    │
└───────────────┘          │  + gyro feed-forward         │       │  → Uq (voltage)   │
        ▲                  └──────────────▲───────────────┘       │  → SVPWM          │
        │                                 │ q_meas                └────────▲──────────┘
        │                        ┌────────┴────────┐                       │ θ_enc
        └────────────────────────│ Mahony AHRS     │◄── IMU (gyro+accel)   │
                                 └─────────────────┘        AS5048A ───────┘
```

### Task layout (FreeRTOS)

| Task          | Core | Priority | Rate          | Responsibility                              |
|---------------|------|----------|---------------|---------------------------------------------|
| `foc_task`    | 1    | 24 (max) | free-run ≥5 kHz | `motor.loopFOC()` ×3, `motor.move()`       |
| `stab_task`   | 1    | 22       | 1 kHz (IMU INT) | AHRS update, stabilization PID              |
| `comms_task`  | 0    | 10       | event         | BLE, Wi-Fi tuning UI, serial commander      |
| `power_task`  | 0    | 5        | 10 Hz         | VBAT monitor, thermal and current limits    |
| `ui_task`     | 0    | 3        | 50 Hz         | Button FSM, LED status                      |

---

## Hardware

### Mechanical

- Arm geometry: Yaw (handle) → Roll → Pitch (camera). The order is configurable in `config.h`.
- Payload: camera modules ≤ 60 g (e.g. Caddx/RunCam-class, or a phone-camera module).
- **Balance the payload mechanically before tuning.** FOC cannot compensate for a badly
  unbalanced load without overheating.
- 3D-printable parts are in `/hardware/mechanical` (STEP + 3MF). Use PETG or CF-Nylon; PLA creeps.

### Electrical

- Motor supply (VMOT) is the raw 2S battery, 6.0–8.4 V.
- The firmware sets the motor voltage limit (`MOTOR_VOLTAGE_LIMIT`) so phase current stays inside the motor's thermal budget.
- Logic runs at 3.3 V from a synchronous buck. Do **not** use an LDO from 8.4 V; it will overheat.

---

## Wiring & Pinout

> Target module: **ESP32-S3-WROOM-1 (N8R8)**. Several pins are avoided on purpose:
> - Strapping pins: GPIO 0, 3, 45, 46
> - Flash/PSRAM pins: GPIO 26–37 (octal PSRAM variants)
> - USB pins: GPIO 19/20, reserved for native USB

### ESP32-S3 pin map

| Function              | GPIO | Peripheral      | Notes                                  |
|-----------------------|------|-----------------|----------------------------------------|
| **Pitch** PWM A/B/C   | 4 / 5 / 6   | MCPWM0 | 25 kHz, center-aligned                 |
| **Pitch** EN          | 7    | GPIO            | DRV8313 EN1-3 tied together             |
| **Roll** PWM A/B/C    | 15 / 16 / 17 | MCPWM0/1 |                                       |
| **Roll** EN           | 18   | GPIO            |                                        |
| **Yaw** PWM A/B/C     | 8 / 9 / 10 | MCPWM1 |                                        |
| **Yaw** EN            | 11   | GPIO            |                                        |
| SPI SCK               | 12   | SPI2 (FSPI)     | Shared bus, 10 MHz                     |
| SPI MOSI              | 13   | SPI2            |                                        |
| SPI MISO              | 14   | SPI2            |                                        |
| CS Encoder Pitch      | 38   | GPIO            | 10 kΩ pull-up                          |
| CS Encoder Roll       | 39   | GPIO            | 10 kΩ pull-up                          |
| CS Encoder Yaw        | 40   | GPIO            | 10 kΩ pull-up                          |
| CS IMU                | 41   | GPIO            | 10 kΩ pull-up                          |
| IMU INT1 (data ready) | 42   | GPIO ISR        | Triggers `stab_task` at 1 kHz          |
| VBAT sense            | 1    | ADC1_CH0        | Divider 100 k / 33 k → max 2.08 V      |
| nFAULT (all drivers)  | 2    | GPIO ISR        | Wired-OR, 10 kΩ pull-up, active low    |
| User button           | 21   | GPIO            | Internal pull-up, active low           |
| Status LED (WS2812)   | 48   | RMT             | Onboard LED on DevKitC-1               |
| Debug UART TX / RX    | 43 / 44 | UART0        | Optional; USB CDC is the default       |
| USB D− / D+           | 19 / 20 | USB-OTG      | Flashing and console                   |

<!-- TODO: confirm against /hardware/pcb/*.kicad_sch rev before release -->

### Motor driver wiring (×3, DRV8313)

```
 ESP32-S3                DRV8313                     Gimbal BLDC
 PWM_A  ────────────────► IN1           OUT1 ───────────► Phase U
 PWM_B  ────────────────► IN2           OUT2 ───────────► Phase V
 PWM_C  ────────────────► IN3           OUT3 ───────────► Phase W
 EN     ────────────────► EN1,EN2,EN3
 nFAULT ◄──────────────── nFAULT (open-drain, wired-OR)
 3V3    ────────────────► nSLEEP, nRESET (via 10 kΩ)
 VBAT   ────────────────► VM  (+ 100 µF low-ESR bulk + 100 nF per driver)
 GND    ────────────────► GND, PGND
```

### Encoder wiring (×3, AS5048A)

```
 AS5048A          ESP32-S3
 VDD3V  ───────── 3V3   (+ 100 nF decoupling at the sensor)
 GND    ───────── GND
 CLK    ───────── GPIO12
 MOSI   ───────── GPIO13
 MISO   ───────── GPIO14
 CSn    ───────── GPIO38 / 39 / 40
```

- Magnet: 6 × 2.5 mm diametric N35. Air gap 0.5–2.5 mm. Center it within ±0.25 mm.
- Shield the SPI wiring through the slip ring or hollow shaft. Keep runs under 15 cm, or drop SCK to 4 MHz.

### IMU wiring (ICM-42688-P)

```
 ICM-42688-P      ESP32-S3
 VDD / VDDIO ──── 3V3
 SCLK  ────────── GPIO12      SDI ── GPIO13      SDO ── GPIO14
 CS    ────────── GPIO41      INT1 ─ GPIO42
```

Mount the IMU on the **camera (pitch) plate**, not the handle. Isolate it from motor vibration with a
foam pad or a soft-mounted sub-board. Record its axis alignment in `config.h → IMU_ROTATION`.

---

## Bill of Materials

| # | Qty | Part | Example P/N | Approx. unit (USD) | Notes |
|---|-----|------|-------------|------------------:|-------|
| 1 | 1 | MCU module | ESP32-S3-WROOM-1-N8R8 | 3.50 | Or ESP32-S3-DevKitC-1 for prototyping (~10) |
| 2 | 3 | BLDC gimbal motor | iPower GM2204 / 2204 260KV hollow-shaft | 14.00 | 12N14P, ~10 Ω phase |
| 3 | 3 | 3-phase driver | TI DRV8313PWPR | 3.20 | Or SimpleFOC Mini board (~6) |
| 4 | 3 | Magnetic encoder | ams AS5048A-HTSP | 6.50 | 14-bit SPI |
| 5 | 3 | Diametric magnet | 6×2.5 mm N35 | 0.40 | |
| 6 | 1 | IMU | TDK ICM-42688-P | 4.00 | Alt: BMI270 (firmware flag) |
| 7 | 1 | Buck regulator | TI TPS62162 (3.3 V, 1 A) | 1.40 | + 2.2 µH inductor |
| 8 | 1 | 2S charger / BMS | TI BQ25887 + protection IC | 4.50 | USB-C 5 V in, cell balancing |
| 9 | 1 | USB-C receptacle | GCT USB4105-GF-A | 0.70 | + 5.1 kΩ CC resistors |
| 10 | 2 | Li-ion cell | 18350 / 14500, ≥ 2 A | 4.00 | Matched pair |
| 11 | 1 | RGB LED | WS2812B-2020 | 0.15 | |
| 12 | 1 | Tactile switch