# ArduPilot Dynamic Harmonic Notch Configuration: STM32H743 with Bidirectional DShot RPM

I don't know what "Cycle 7" refers to in your process. I've treated it as one iteration of a log → tune → verify loop and structured this as a complete, repeatable procedure. The parameter values target a typical 5–10" quad. Adjust the values marked ⚙ to your airframe after reviewing your logs.

---

## 1. Prerequisites: Bidirectional DShot Telemetry

| Parameter | Value | Notes |
|---|---|---|
| `MOT_PWM_TYPE` | `6` (DShot600) or `5` (DShot300) | Use DShot300 if ESCs struggle with bidir at 600 |
| `SERVO_BLH_BDMASK` | `15` (motors 1–4) | Bitmask of outputs with bidirectional DShot. Use `63` for a hex, `255` for an octo |
| `SERVO_BLH_POLES` | `14` ⚙ | Motor magnet count. 14 is typical for 22xx–28xx motors. **A wrong value scales every notch incorrectly.** |
| `SERVO_DSHOT_ESC` | `1` (BLHeli32/BLHeli_S-bluejay/AM32) | Must match firmware type |
| `SERVO_DSHOT_RATE` | `0`–`2` | Rate 0 ties DShot to the loop rate |

Bidirectional DShot is assigned per timer group on H743 boards. All outputs on a shared timer must use the same protocol, so check your board's hwdef.

**Verify before proceeding:** arm on the bench without props. `ESC1..4 RPM` must read non-zero and plausible in the MAVLink inspector or `ESC` log messages. A dropout on any channel invalidates per-motor notching.

---

## 2. Primary Harmonic Notch (INS_HNTCH_*): RPM-Tracked Motor Noise

| Parameter | Value | Rationale |
|---|---|---|
| `INS_HNTCH_ENABLE` | `1` | Reboot after setting to expose the remaining parameters |
| `INS_HNTCH_MODE` | `3` | ESC telemetry |
| `INS_HNTCH_REF` | `1` | Scaling reference (1.0 = use RPM directly) |
| `INS_HNTCH_FREQ` | `≈ 0.6–0.8 × hover motor freq` ⚙ | Acts as the **minimum** notch centre. Example: hover at ~9000 RPM gives 150 Hz, so set ~80–100 Hz |
| `INS_HNTCH_BW` | `≈ FREQ / 2` ⚙ | Example: 40–50 Hz |
| `INS_HNTCH_ATT` | `40` dB | Default; raise only if the residual peak persists |
| `INS_HNTCH_HMNCS` | `7` (1st, 2nd, 3rd) ⚙ | Bitmask. Add `8` (4th) if logs show it. Remove unneeded harmonics to save CPU |
| `INS_HNTCH_OPTS` | `6` (Multi-Source + Loop-rate update) | Bit 1 = per-motor notches, bit 2 = update at loop rate. Add `8` to apply on all IMUs (recommended): **`14`** |
| `INS_HNTCH_FM_RAT` | `1.0` | Lower bound ratio when RPM falls below FREQ. Lower to ~0.7 only if low-throttle noise is observed |

**Why Multi-Source on H743:** each motor gets its own notch set, so 4 motors × 3 harmonics = 12 notches per axis. That tracks individual motors during yaw and attitude changes much more tightly than a single averaged notch. The H743 handles this comfortably at 400 Hz with fast sampling, but confirm `PM.Load` stays below ~60%.

Option-bit meanings have shifted slightly between versions. Confirm them against the parameter docs for your exact firmware (4.4/4.5/4.6).

---

## 3. Secondary Notch (INS_HNTC2_*): Frame or Prop-Wash Resonance

Use this only if post-filter logs show a peak that does **not** track RPM, such as an arm or frame resonance.

| Parameter | Value |
|---|---|
| `INS_HNTC2_ENABLE` | `1` |
| `INS_HNTC2_MODE` | `0` (fixed), or `4` (in-flight FFT) if the peak drifts |
| `INS_HNTC2_FREQ` | Measured resonance centre ⚙ |
| `INS_HNTC2_BW` | ~20–30% of FREQ |
| `INS_HNTC2_HMNCS` | `1` |
| `INS_HNTC2_ATT` | `30–40` |

If no fixed resonance exists, leave `INS_HNTC2_ENABLE = 0` to save phase lag.

---

## 4. Gyro Low-Pass and Loop Settings

| Parameter | Value | Notes |
|---|---|---|
| `SCHED_LOOP_RATE` | `400` (up to `800`/`1000` for small, agile craft) | H743 supports higher rates |
| `INS_FAST_SAMPLE` | `1` (or `3` for dual IMU) | Fast sampling on supported IMUs |
| `INS_GYRO_RATE` | `0` (1 kHz) or `1` (2 kHz) | Higher rates reduce notch phase lag |
| `INS_GYRO_FILTER` | `80–120` Hz ⚙ | Raise once the notches are effective; 5" craft often runs 100+ |
| `INS_ACCEL_FILTER` | `20` | Default is fine |
| `ATC_RAT_*_FLTD` / `FLTE` | ~½ `INS_GYRO_FILTER` | D-term filter |

---

## 5. Logging for Verification

| Parameter | Value |
|---|---|
| `INS_LOG_BAT_MASK` | `1` (first IMU) |
| `INS_LOG_BAT_OPT` | `4` (pre + post-filter batch) |
| `LOG_BITMASK` | Include IMU_RAW, or use batch sampling only |

1. Hover for 60+ seconds, then perform throttle sweeps and moderate maneuvers.
2. Load the `.bin` file into the **ArduPilot WebTools Filter Review tool**.
3. Pass criteria:
   - Post-filter motor fundamental and harmonics are attenuated by ≥ 20–30 dB.
   - No residual peak lies within ~1.5× the `INS_GYRO_FILTER` cutoff.
   - Notch centres in the tool overlay the RPM traces across the full throttle range.
4. Disable batch logging after tuning.

---

## 6. Iteration Loop (One Cycle)

1. **Log** a hover plus sweeps with the current parameters.
2. **Inspect:**
   - If the notch is offset from the peak, check `SERVO_BLH_POLES`.
   - If a low-throttle peak remains, lower `INS_HNTCH_FREQ` or `FM_RAT`.
   - If a residual harmonic remains, add it to `HMNCS`.
   - If a non-tracking peak appears, use `HNTC2`.
3. **Adjust** one group of parameters at a time.
4. **Raise** `INS_GYRO_FILTER` in 10–20 Hz steps while the post-filter spectrum stays clean.
5. **Re-run autotune or manual PID tuning** after filter changes are final, because filter phase lag affects optimal gains.

---

## Quick-Start Summary (Typical 7" Quad)

```
MOT_PWM_TYPE      6
SERVO_BLH_BDMASK  15
SERVO_BLH_POLES   14
INS_HNTCH_ENABLE  1
INS_HNTCH_MODE    3
INS_HNTCH_REF     1
INS_HNTCH_FREQ    80
INS_HNTCH_BW      40
INS_HNTCH_ATT     40
INS_HNTCH_HMNCS   7
INS_HNTCH_OPTS    14
INS_GYRO_FILTER   90
INS_LOG_BAT_MASK  1
INS_LOG_BAT_OPT   4
```

If you share your frame size, motor KV and pole count, prop size, battery cell count, and firmware version, I can tighten the values marked ⚙. A filter-review screenshot or log would let me suggest exact changes for your next cycle.