# ArduPilot Harmonic Notch Configuration, Cycle 1

**Target:** STM32H743 flight controller with bidirectional DShot (ESC RPM telemetry)
**Goal of Cycle 1:** Get RPM data working, apply a conservative notch configuration, and log pre- and post-filter gyro data. Cycle 2 refines the settings using that data.

> Parameter names and bitmask values below reflect ArduPilot 4.4 and 4.5. Check them against the parameter documentation for your firmware version before writing them to the vehicle.

---

## 1. Prerequisites: Bidirectional DShot RPM Feedback

| Parameter | Value | Notes |
|---|---|---|
| `MOT_PWM_TYPE` | `6` (DShot600) | Use `5` (DShot300) if your ESCs or wiring are marginal |
| `SERVO_BLH_BDMASK` | `15` | Outputs 1–4 for a quad. Use `63` for a hexa and `255` for an octa |
| `SERVO_BLH_POLES` | `14` | Typical for 5"–7" motors. Count the magnets on your motor bell |
| `SERVO_BLH_AUTO` | `1` | Maps motor outputs to the BLHeli/DShot handling |
| `SERVO_DSHOT_ESC` | `1` (BLHeli32/AM32) or `3` (BLHeli_S + Bluejay) | Must match the ESC firmware |
| `SERVO_DSHOT_RATE` | `0` or `1` | `1` (loop rate) is fine on H7 |
| `BRD_IO_DSHOT` | `0` | Only relevant on IOMCU boards (Pixhawk-style). Bidirectional DShot works on FMU outputs |

Bidirectional DShot also requires firmware built with BDShot support. Many H743 targets ship a `*-bdshot` firmware variant, so flash that if your board has one.

**Verify before going further:**
- In the Mission Planner status tab or the logs, check `ESC[n].RPM`. All motors should report sensible RPM.
- Check that the reported error rate is near 0%.
- If any motor reads 0 RPM, stop and fix this before configuring the notch.

---

## 2. Harmonic Notch #1: RPM-Tracking (Primary)

| Parameter | Cycle 1 Value | Rationale |
|---|---|---|
| `INS_HNTCH_ENABLE` | `1` | Reboot after enabling to expose the other parameters |
| `INS_HNTCH_MODE` | `3` | ESC telemetry (RPM) tracking |
| `INS_HNTCH_REF` | `1` | Scaling reference. Use 1.0 for ESC mode |
| `INS_HNTCH_FREQ` | `≈ 0.8 × hover motor frequency`, see below | Floor frequency. The notch never tracks below this |
| `INS_HNTCH_BW` | `INS_HNTCH_FREQ / 2` | Initial bandwidth |
| `INS_HNTCH_ATT` | `40` dB | Default attenuation |
| `INS_HNTCH_HMNCS` | `3` | 1st and 2nd harmonics. Add the 3rd (`7`) only if the logs show it |
| `INS_HNTCH_OPTS` | `2` (per-motor) or `6` (per-motor + loop-rate update) | See the CPU note below |
| `INS_HNTCH_FM_RAT` | `1.0` | Mainly relevant in throttle mode. Leave at default with ESC RPM |

**Computing the floor frequency:**

`hover_Hz = hover_RPM / 60`

ESC telemetry already reports mechanical RPM, provided `SERVO_BLH_POLES` is correct.

Typical starting values for `INS_HNTCH_FREQ`:

| Frame | Floor frequency |
|---|---|
| 5" | 80–100 Hz |
| 7" | 60–80 Hz |
| 10" | 40–50 Hz |
| 15"+ | 25–35 Hz |

**`INS_HNTCH_OPTS` bitmask:**

| Value | Option |
|---|---|
| 1 | Double notch |
| 2 | Multi-source (one notch per motor) |
| 4 | Update at loop rate |
| 8 | Apply to all IMUs |
| 16 | Triple notch |

**H743 CPU note:**
- On a quad, per-motor notches with 2 harmonics create 8 notches per axis, per IMU.
- The H743 can usually handle `6` (per-motor + loop-rate update).
- Add `8` (all IMUs, giving `14`) only after confirming CPU headroom. Check that `PM.Load` stays below ~60% and `PM.NLon` (long loops) stays low.

---

## 3. Notch #2: Optional Secondary Filter

Use the second notch only if the Cycle 1 logs show a non-RPM peak, such as a frame resonance.

| Parameter | Value |
|---|---|
| `INS_HNTC2_ENABLE` | `1` |
| `INS_HNTC2_MODE` | `0` (static) for a fixed frame resonance, or `4` (in-flight FFT) for a moving unknown peak |
| `INS_HNTC2_FREQ` / `INS_HNTC2_BW` | Taken from the observed peak |

Leave this disabled for Cycle 1 unless you already know about a resonance.

---

## 4. Gyro Sampling and Low-Pass Filters

| Parameter | Cycle 1 Value | Notes |
|---|---|---|
| `INS_GYRO_RATE` | `1` (2 kHz) | Try `2` (4 kHz) in Cycle 2 if the CPU allows |
| `SCHED_LOOP_RATE` | `400` | Default for most copters. 5" racers may use 800+ |
| `INS_GYRO_FILTER` | `40` Hz | Conservative while the notch is unproven. Raise to 60–80 Hz once the notch is verified |
| `INS_ACCEL_FILTER` | `20` Hz | Default |
| `ATC_RAT_RLL_FLTD` / `ATC_RAT_PIT_FLTD` | `20` Hz | D-term filter. Can rise after a clean notch |
| `ATC_RAT_YAW_FLTE` | `2` Hz | Default yaw error filter |

---

## 5. Logging for Analysis

| Parameter | Value | Purpose |
|---|---|---|
| `INS_LOG_BAT_MASK` | `1` | Batch-sample IMU 1 |
| `INS_LOG_BAT_OPT` | `4` | Log pre- and post-filter data. Use `5` to also enable sensor-rate logging |
| `LOG_BITMASK` | Default + IMU | Ensure IMU and ESC data are logged |

---

## 6. Cycle 1 Flight Procedure

1. Arm, take off, and hover in AltHold for 30–60 seconds.
2. Perform gentle roll and pitch inputs, then a few throttle changes.
3. Land and download the `.bin` log.
4. Open the log in the **ArduPilot WebTools Filter Review** tool or the Mission Planner FFT tool.
5. Check the following:
   - **Pre-filter spectrum:** the main peak should sit at roughly the motor frequency and track RPM.
   - **Post-filter spectrum:** the peak should be attenuated by at least 20–40 dB.
   - **Residual peaks:**
     - At 2× or 3× the motor frequency: adjust `HMNCS`.
     - At a fixed frequency: enable `INS_HNTC2` as a static notch.
   - **Peaks below `INS_HNTCH_FREQ` at idle or low throttle:** lower `FREQ`.
   - **CPU health:** check `PM.Load` and `PM.NLon`.

---

## 7. Cycle 2 Preview

After Cycle 1, adjust based on what the logs show:

- Tighten or widen `BW` based on the measured peak width.
- Raise `INS_GYRO_FILTER` toward 60–80 Hz and the D-term filters toward 30–40 Hz.
- Enable 4 kHz gyro rate or the all-IMU option if CPU headroom allows.
- Re-run Autotune with the cleaned-up gyro signal.