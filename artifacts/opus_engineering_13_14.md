I don't have access to the actual `osmo-pocket-esp32-gimbal` repository or the prior 12 cycles, so this README is a **complete reference draft** built on a typical ESP32-S3 + SimpleFOC 3-axis gimbal architecture. Before merging, check every item marked `⚠️ VERIFY` against your schematic and firmware, especially GPIO assignments, motor part numbers, and tuning defaults. If you paste your current README, `platformio.ini`, or pin header file, I can reconcile this draft to match exactly.

---

````markdown
# osmo-pocket-esp32-gimbal

> Open-source, pocket-sized 3-axis brushless camera gimbal built on the ESP32-S3,
> using Field-Oriented Control (FOC) with absolute magnetic encoders and an IMU-based
> stabilization loop.

![build](https://img.shields.io/badge/build-PlatformIO-orange)
![mcu](https://img.shields.io/badge/MCU-ESP32--S3-blue)
![foc](https://img.shields.io/badge/FOC-SimpleFOC%202.3.x-green)
![license](https://img.shields.io/badge/license-MIT-lightgrey)

---

## Table of Contents

1. [Overview](#overview)
2. [Features](#features)
3. [System Architecture](#system-architecture)
4. [Hardware](#hardware)
5. [Wiring & Pinouts](#wiring--pinouts)
6. [Power Tree](#power-tree)
7. [Firmware: Build & Flash](#firmware-build--flash)
8. [First Boot & Calibration](#first-boot--calibration)
9. [FOC Tuning Procedure](#foc-tuning-procedure)
10. [Stabilization Loop Tuning](#stabilization-loop-tuning)
11. [Serial Command Reference](#serial-command-reference)
12. [Troubleshooting](#troubleshooting)
13. [Bill of Materials](#bill-of-materials)
14. [Safety](#safety)
15. [Contributing](#contributing)
16. [License](#license)

---

## Overview

This project is a handheld 3-axis gimbal (pan / tilt / roll) inspired by the DJI Osmo
Pocket form factor. It is designed for small action cameras and camera modules
(≤ 120 g payload). All three axes use low-KV gimbal BLDC motors driven in
**voltage-mode FOC** with **absolute magnetic encoders**, so the motors need no
alignment wiggle after the first calibration.

| Spec | Value |
|---|---|
| MCU | ESP32-S3 (dual-core Xtensa LX7 @ 240 MHz) |
| Axes | 3 (Yaw / Pitch / Roll) |
| Motor control | SimpleFOC, voltage-mode torque, SVPWM |
| Position feedback | MT6701 14-bit magnetic encoder (SSI) per axis |
| Attitude sensor | ICM-42688-P 6-axis IMU (SPI, 1 kHz) |
| Supply | 2S Li-ion (7.4 V nominal, 6.0–8.4 V) |
| FOC loop rate | ≥ 5 kHz per motor (target) |
| Stabilization loop | 1 kHz |
| Payload | ≤ 120 g ⚠️ VERIFY |
| Connectivity | USB-CDC (config/debug), BLE (remote control, optional) |

---

## Features

- Closed-loop FOC on all three axes with absolute encoders and no startup alignment after calibration.
- Complementary or Mahony attitude estimation from the IMU.
- Modes:
  - **Lock**: hold the camera in a fixed world orientation.
  - **Follow**: yaw (and optionally pitch) smoothly follows the handle.
  - **FPV**: all axes follow the handle, with roll leveled.
  - **Recenter**: double-press the button to return to home.
- Calibration and parameters persist in NVS (flash).
- Live tuning over the serial command interface, compatible with the SimpleFOC Studio protocol.
- Battery monitoring with low-voltage cutoff.
- Fault handling: motor stall detection, encoder CRC/timeout, IMU watchdog, and over-temperature derating.

---

## System Architecture

### Hardware block diagram

```
                         ┌──────────────────────────────────────────┐
  2S Li-ion ──► [Fuse] ──┤ VBAT (6.0–8.4 V)                         │
     │                   │                                          │
     │            ┌──────┴──────┐        ┌───────────────────────┐  │
     │            │ Buck 3.3 V  │──3V3──►│      ESP32-S3         │  │
     │            │  (≥ 1 A)    │        │                       │  │
     │            └─────────────┘        │  Core 0: FOC ×3       │  │
     │                                   │  Core 1: IMU, attitude│  │
     │                                   │          ctrl, comms  │  │
     │                                   └──┬────┬────┬────┬─────┘  │
     │                     3×PWM + EN       │    │    │    │ SPI    │
     │          ┌──────────────────────────┘    │    │    │        │
     │          │            ┌──────────────────┘    │    │        │
     │          │            │          ┌─────────────┘    │        │
     ▼          ▼            ▼          ▼                  ▼        │
 ┌────────┐ ┌────────┐ ┌────────┐               ┌─────────────────┐ │
 │DRV8313 │ │DRV8313 │ │DRV8313 │               │ SPI bus         │ │
 │ YAW    │ │ PITCH  │ │ ROLL   │               │ ├ ICM-42688-P   │ │
 └───┬────┘ └───┬────┘ └───┬────┘               │ ├ MT6701 (YAW)  │ │
     │U V W     │U V W     │U V W               │ ├ MT6701 (PITCH)│ │
     ▼          ▼          ▼                    │ └ MT6701 (ROLL) │ │
 ┌───────┐  ┌───────┐  ┌───────┐                └─────────────────┘ │
 │GM2804 │  │GM2208 │  │GM2804 │   ◄── diametric magnet on each     │
 │ Yaw   │  │ Pitch │  │ Roll  │       shaft, read by MT6701        │
 └───────┘  └───────┘  └───────┘                                    │
                                                                    │
  [Joystick] [Mode btn] [WS2812 LED] [VBAT divider] ───────────────┘
```

### Firmware architecture

```
┌──────────────────────── Core 1 (APP_CPU) ────────────────────────┐
│                                                                  │
│  IMU ISR (1 kHz) ─► Attitude Estimator ─► Gimbal Controller      │
│   (data-ready)      (Mahony, q_world)     (mode logic,           │
│                                            setpoint smoothing,   │
│                                            outer PID per axis)   │
│                                                   │              │
│  BLE / USB-CDC ─► Command Parser ─► Param Store (NVS)            │
│                                                   │              │
└───────────────────────────────────────────────────┼──────────────┘
                                                    │ target angle /
                                                    │ target velocity
                          lock-free double buffer   ▼
┌──────────────────────── Core 0 (PRO_CPU) ────────────────────────┐
│                                                                  │
│  for each axis:  sensor.update() ─► motor.loopFOC()              │
│                  motor.move(target)  (angle → velocity → Uq)     │
│                                                                  │
│  Fault monitor: encoder timeout, stall, VBAT, temperature        │
└──────────────────────────────────────────────────────────────────┘
```

### Control cascade (per axis)

```
 Desired attitude            Outer loop (1 kHz)        Inner loops (≥ 5 kHz, SimpleFOC)
 (mode logic)                                         
   θ_ref ──►(+)──► [PID_stab] ──► ω_ref ──►(+)──► [PID_vel] ──► Uq ──► [FOC/SVPWM] ──► Motor
            ▲−                              ▲−                                          │
            │                               │                                           │
   θ_IMU ◄──┴── IMU (camera frame)    ω_enc ◄┴── MT6701 encoder ◄────────────────────────┘
```

The IMU closes the **outer** loop, which determines where the camera points in world space.
The encoder closes the **inner** loop, which handles commutation, velocity, and damping.

---

## Hardware

| Subsystem | Part | Notes |
|---|---|---|
| MCU | ESP32-S3-WROOM-1-N8R2 | Use the N8R2 (quad PSRAM). Octal-PSRAM modules (R8) reserve GPIO 33–37. |
| Motor driver | TI DRV8313 ×3 | 65 V / 2.5 A peak, 3 half-bridges with independent EN. |
| Motors | iPower GM2804 (yaw, roll), GM2208 (pitch) ⚠️ VERIFY | Gimbal motors, about 10 Ω phase resistance, 7 pole pairs. |
| Encoders | MagnTek MT6701 ×3 | SSI mode, 14-bit. Use a diametric 6×2.5 mm N35 magnet. |
| IMU | TDK ICM-42688-P | Mount on the camera plate, not the handle. |
| 3V3 regulator | TPS62162 (or AP63203) | Synchronous buck. Avoid an LDO from 8.4 V. |
| Battery | 2× 18350 or 2S 800 mAh LiPo | With a 2S BMS/protection board. |
| Charging | IP2326 2S USB-C boost charger | Optional. |

> **MCPWM limitation:** The ESP32-S3 has two MCPWM units. SimpleFOC binds one
> 3-PWM driver per MCPWM operator group, so **two motors use MCPWM and the third
> falls back to LEDC**. LEDC is not center-aligned, so the third motor runs slightly
> noisier. Assign the LEDC motor to the axis with the least load (roll).
> ⚠️ VERIFY against the SimpleFOC version you pin.

---

## Wiring & Pinouts

### ESP32-S3 GPIO map ⚠️ VERIFY against `include/pins.h`

| Function | GPIO | Dir | Peripheral | Notes |
|---|---|---|---|---|
| **Yaw** PWM A / B / C | 4 / 5 / 6 | O | MCPWM0 | |
| Yaw EN | 7 | O | GPIO | DRV8313 EN1–3 tied together |
| **Pitch** PWM A / B / C | 15 / 16 / 17 | O | MCPWM1 | |
| Pitch EN | 18 | O | GPIO | |
| **Roll** PWM A / B / C | 8 / 9 / 10 | O | LEDC | See MCPWM note |
| Roll EN | 11 | O | GPIO | |
| DRV8313 nFAULT (wired-OR) | 3 | I | GPIO | 10 kΩ pull-up. Strapping pin; safe as an input once booted. |
| DRV8313 nSLEEP / nRESET (all) | 46 | O | GPIO | Strapping pin; keep low at boot (default). |
| SPI SCK | 12 | O | SPI2 (FSPI) | Shared bus |
| SPI MISO | 13 | I | SPI2 | Shared bus |
| SPI MOSI | 14 | O | SPI2 | IMU only. The MT6701 in SSI mode is read-only. |
| CS IMU | 38 | O | GPIO | |
| CS Yaw encoder | 21 | O | GPIO | |
| CS Pitch encoder | 47 | O | GPIO | |
| CS Roll encoder | 48 | O | GPIO | |
| IMU INT1 (data-ready) | 39 | I | GPIO ISR | |
| VBAT sense | 1 | I | ADC1_CH0 | 100 kΩ / 22 kΩ divider plus 100 nF |
| Joystick X / Y | 2 / 40* | I | ADC1_CH1 / — | *GPIO40 is not ADC-capable. If you need an analog Y axis, move it to an ADC1 pin. ⚠️ VERIFY |
| Mode button | 41 | I | GPIO | Active-low, internal pull-up |
| Status LED (WS2812) | 42 | O | RMT | |
| USB D− / D+ | 19 / 20 | — | USB-OTG/CDC | |
| BOOT | 0 | I | — | Do not use |

**Avoid:** GPIO 26–32 (SPI flash) and 33–37 (octal PSRAM, if fitted). Do not drive
GPIO 0, 3, 45, or 46 externally during reset.

### Motor driver wiring (repeat per axis)

```
 ESP32-S3                 DRV8313                    Motor
 ─────────                ───────                    ─────
 PWM_A  ───────────────►  IN1          OUT1 ────────► U
 PWM_B  ───────────────►  IN2          OUT2 ────────► V
 PWM_C  ───────────────►  IN3          OUT3 ────────► W
 EN     ───────────────►  EN1+EN2+EN3
 nSLEEP ───────────────►  nSLEEP, nRESET (shared)
 nFAULT ◄───────────────  nFAULT (open-drain, wired-OR)
                          VM  ◄── VBAT (+ 100 µF bulk + 0.1 µF per driver)
                          PGND/GND ── star ground
                          V3P3OUT ── 0.47 µF to GND (do not load)
                          CPL/CPH ── 0.01 µF ; VCP ── 0.1 µF to VM
```

### Encoder wiring (MT6701,