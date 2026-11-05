I don't have access to your repository's prior revisions, schematic, or source, so this README describes a **reference design**: ESP32-S3, three DRV8313 drivers, MT6701 encoders, an ICM-42688-P IMU, and SimpleFOC. Anywhere your hardware differs, especially pin assignments and motor specs, replace the values before you publish. Those spots are marked with `⚠️ VERIFY`.

---

````markdown
# osmo-pocket-esp32-gimbal

> A compact, open-source, 3-axis handheld stabilizer in the spirit of the DJI Osmo Pocket.
> It runs on an ESP32-S3 with closed-loop Field-Oriented Control (FOC) of three BLDC gimbal motors.

![status](https://img.shields.io/badge/status-beta-orange)
![platform](https://img.shields.io/badge/MCU-ESP32--S3-blue)
![framework](https://img.shields.io/badge/framework-Arduino%20%2B%20SimpleFOC-green)
![license](https://img.shields.io/badge/license-MIT%20%2F%20CERN--OHL--P-lightgrey)

**Document revision:** Cycle 27

---

## Table of Contents

1. [Features](#features)
2. [System Overview](#system-overview)
3. [Block Diagrams](#block-diagrams)
4. [Hardware](#hardware)
5. [Wiring & Pinout](#wiring--pinout)
6. [Bill of Materials](#bill-of-materials)
7. [Firmware Architecture](#firmware-architecture)
8. [Building & Flashing](#building--flashing)
9. [First Power-Up Checklist](#first-power-up-checklist)
10. [FOC Tuning Procedure](#foc-tuning-procedure)
11. [Stabilization Loop Tuning](#stabilization-loop-tuning)
12. [Configuration Reference](#configuration-reference)
13. [Serial Command Interface](#serial-command-interface)
14. [Troubleshooting](#troubleshooting)
15. [Safety](#safety)
16. [Repository Layout](#repository-layout)
17. [Contributing](#contributing)
18. [License](#license)

---

## Features

- **3-axis stabilization**: pan (yaw), tilt (pitch), and roll.
- **Closed-loop FOC** using 14-bit absolute magnetic encoders on every axis. There is no open-loop "stepping" and no cogging jitter.
- **1 kHz attitude estimation** from an ICM-42688-P IMU, fused with a Mahony filter.
- **Follow modes**: Lock, Pan-Follow, Pan+Tilt Follow, FPV (all axes follow).
- **Joystick** control of pan and tilt, plus a multi-function button (single, double, and triple press).
- **BLE control** for mode, recenter, and live parameter tuning.
- **Persistent calibration**: encoder zero offsets, IMU bias, and PID gains are stored in NVS.
- **Fault handling**: driver nFAULT, under-voltage cutoff, stall detection, and motor over-temperature estimation.
- **Dual-core scheduling**: core 1 is dedicated to FOC, and core 0 handles fusion, UI, and radio.

---

## System Overview

```
   ┌──────────────┐   handle   ┌─────────────────────────────────────────┐
   │ User inputs  │──────────▶ │           ESP32-S3 controller           │
   │ joystick/btn │            │  IMU fusion → setpoints → 3× FOC loops  │
   └──────────────┘            └───────┬───────────────┬─────────────────┘
                                       │ PWM (9 ch)    │ SPI (shared bus)
                         ┌─────────────┼───────────────┼──────────────┐
                         ▼             ▼               ▼              ▼
                    DRV8313 ×3    Pan / Tilt / Roll  MT6701 ×3    ICM-42688-P
                    (3-phase)     gimbal motors      encoders     (on camera
                                                                   plate)
```

The IMU is mounted on the **camera plate**, which is the stabilized payload. The controller measures the payload's attitude, compares it with the user's target, and commands each motor's electrical angle through FOC to cancel the error.

---

## Block Diagrams

### Electrical

```
             ┌──────────────────────────────────────────────────────────┐
  2S Li-ion  │                                                          │
  7.4 V ─────┼──▶ [Fuse 3A] ─▶ [Rev-pol PFET] ─┬─▶ VMOT (7.4 V) ───┐    │
  1000 mAh   │                                │                   │    │
             │                                ├─▶ [Buck 3.3V/1A] ─┼─┐  │
             │                                │                   │ │  │
             │                [R-divider] ◀───┘                   │ │  │
             │                     │ VBAT_SENSE                   │ │  │
             │                     ▼                              │ │  │
             │   ┌───────────────────────────────┐   3V3          │ │  │
             │   │         ESP32-S3-WROOM-1      │◀───────────────┼─┘  │
             │   │                               │                │    │
             │   │  MCPWM0 ─▶ PWM A/B/C ─────────┼─▶ DRV8313 #1 ◀─┤    │─▶ PAN motor
             │   │  MCPWM1 ─▶ PWM A/B/C ─────────┼─▶ DRV8313 #2 ◀─┤    │─▶ TILT motor
             │   │  LEDC   ─▶ PWM A/B/C ─────────┼─▶ DRV8313 #3 ◀─┘    │─▶ ROLL motor
             │   │                               │      │ nFAULT       │
             │   │  GPIO42 ◀────── wired-OR ──────┼──────┘              │
             │   │                               │                     │
             │   │  FSPI ─┬─ CS0 ─▶ ICM-42688-P   │ (camera plate)      │
             │   │        ├─ CS1 ─▶ MT6701 PAN    │                     │
             │   │        ├─ CS2 ─▶ MT6701 TILT   │                     │
             │   │        └─ CS3 ─▶ MT6701 ROLL   │                     │
             │   │                               │                     │
             │   │  ADC1 ◀─ Joystick X/Y, VBAT   │                     │
             │   │  GPIO ◀─ Button   GPIO ─▶ RGB │                     │
             │   │  USB-CDC (GPIO19/20) ◀─▶ Host │                     │
             │   └───────────────────────────────┘                     │
             └──────────────────────────────────────────────────────────┘
```

### Control loops (per axis)

```
 Joystick / mode ─▶ ┌──────────────┐  θ_target   ┌─────────────┐  ω_target  ┌─────────────┐  Uq   ┌──────────┐
                    │ Setpoint gen │────────────▶│ Stab. PID   │───────────▶│ Velocity PI │──────▶│ FOC      │─▶ Motor
 IMU (1 kHz) ─────▶ │ (follow/lock)│  θ_payload  │ (IMU angle) │            │ (encoder ω) │       │ SVPWM    │
                    └──────────────┘ ──────────▶ └─────────────┘            └──────▲──────┘       └────▲─────┘
                                                      ~500 Hz                      │ ~1 kHz            │ ≥ 5 kHz
                                                                          encoder ─┴───────────────────┘
                                                                          (θ_mech, θ_elec)
```

- **Inner loop (FOC, ≥ 5 kHz per motor):** voltage-mode torque control. It uses the encoder's electrical angle and applies SVPWM.
- **Middle loop (velocity PI, 1 kHz):** produces a smooth torque command and damps resonance.
- **Outer loop (stabilization PID, 500 Hz):** acts on the IMU-measured payload angle error.

---

## Hardware

| Subsystem | Part | Notes |
|---|---|---|
| MCU | ESP32-S3-WROOM-1-N8R2 | Dual-core 240 MHz, native USB, 2 × MCPWM units |
| Motor driver | TI DRV8313 (×3) | 65 V / 2.5 A peak; integrated half-bridges; nFAULT |
| Motors | iPower GBM2204H-100T (×3) ⚠️ VERIFY | 12N14P, **7 pole pairs**, ~10 Ω phase resistance |
| Encoders | MagnTek MT6701 (×3) | 14-bit, SSI/SPI mode, diametric 6×2.5 mm N35 magnet |
| IMU | TDK ICM-42688-P | SPI at 10 MHz, 1 kHz ODR, ±2000 dps / ±4 g |
| Power | 2S Li-ion 7.4 V 1000 mAh | Buck to 3.3 V (TPS62162 or equivalent) |
| UI | 2-axis thumb joystick, tactile button, WS2812B LED | |

> **Why voltage-mode FOC without current sensing?** High-resistance gimbal motors (≈10 Ω) draw well under 1 A at 7.4 V. Voltage control with a phase-resistance estimate is accurate enough, and it saves 6 ADC channels and the shunt amplifiers.

---

## Wiring & Pinout

### ESP32-S3 pin assignment ⚠️ VERIFY against your schematic

| Function | GPIO | Peripheral | Notes |
|---|---|---|---|
| **PAN** PWM A / B / C | 4 / 5 / 6 | MCPWM0 | |
| PAN EN | 7 | GPIO | Drives all three ENx on DRV8313 #1 |
| **TILT** PWM A / B / C | 17 / 18 / 8 | MCPWM1 | |
| TILT EN | 3 | GPIO | ⚠️ Strapping pin (JTAG select); safe as an output after boot. Add a 10 k pull-down. |
| **ROLL** PWM A / B / C | 38 / 39 / 40 | LEDC (or MCPWM0 op2) | |
| ROLL EN | 41 | GPIO | |
| DRV nSLEEP (all) | 46 | GPIO | ⚠️ Strapping pin; keep a 10 k pull-down so drivers stay asleep at boot |
| DRV nFAULT (wired-OR) | 42 | GPIO input, interrupt | 10 k pull-up to 3V3 |
| SPI SCLK | 12 | FSPI | 10 MHz max (encoders limited to ~15 MHz) |
| SPI MOSI | 11 | FSPI | IMU only; MT6701 is read-only |
| SPI MISO | 13 | FSPI | |
| CS — IMU | 10 | GPIO | |
| CS — Encoder PAN | 14 | GPIO | |
| CS — Encoder TILT | 15 | GPIO | |
| CS — Encoder ROLL | 16 | GPIO | |
| IMU INT1 (data-ready) | 21 | GPIO interrupt | |
| VBAT sense | 1 | ADC1_CH0 | Divider 100 k / 33 k → 7.4 V maps to ~1.84 V |
| Joystick X | 2 | ADC1_CH1 | |
| Joystick Y | 9 | ADC1_CH8 | |
| Button | 47 | GPIO, internal pull-up | Active low |
| Status LED (WS2812B) | 48 | RMT | |
| USB D− / D+ | 19 / 20 | USB-OTG/CDC | Reserved |
| UART0 TX / RX | 43 / 44 | UART0 | Reserved for logs |

**Pins to avoid:**
- GPIO 26–32 are used by SPI flash.
- GPIO 33–37 are used by oct