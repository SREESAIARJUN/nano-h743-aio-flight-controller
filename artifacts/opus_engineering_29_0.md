# ArduPilot Gyro and Harmonic Notch Setup: STM32H743 with Bidirectional DShot RPM

**About "Cycle 29":** This isn't a standard ArduPilot term. If it refers to your own tuning iteration, batch, or log number, share what changed in the previous cycle or attach FFT results and I can refine the values. Below is a solid baseline for an H743 with bidirectional DShot (ArduCopter 4.4/4.5+).

---

## 1. Prerequisites: Bidirectional DShot and RPM Telemetry

| Parameter | Value | Notes |
|---|---|---|
| `MOT_PWM_TYPE` | 6 (DShot600) | Use 5 (DShot300) if you see signal errors |
| `SERVO_BLH_BDMASK` | e.g. 15 (motors 1–4) | Bitmask of outputs with bidirectional DShot |
| `SERVO_BLH_POLES` | 14 | **Must match the motor pole count** (typical 5" motor: 12N14P → 14) |
| `SERVO_BLH_AUTO` | 1 | Auto-map outputs |
| `SERVO_DSHOT_ESC` | 1 (BLHeli32/AM32) or 3 (BLHeli_S Bluejay) | Match your ESC firmware |
| `SERVO_DSHOT_RATE` | 0 or 1 | Leave low unless needed |

**Verify before flying:** with props off, arm and check `esc[n].rpm` in MAVLink Inspector or the ESC log. All motors should read sane, non-zero RPM with an error rate below about 1%.

---

## 2. Gyro Sampling and Low-Pass Filters (H743 has CPU headroom)

| Parameter | Value | Rationale |
|---|---|---|
| `SCHED_LOOP_RATE` | 400 (5–7"), 800 (3" whoops) | Higher rates on small craft only |
| `INS_GYRO_RATE` | 1 (2 kHz) or 2 (4 kHz) | Fast sampling; H7 handles 4 kHz with headroom |
| `INS_GYRO_FILTER` | 80 Hz (3–5"), 60 (7"), 40 (10"+) | Raise once notches are effective |
| `INS_ACCEL_FILTER` | 20 Hz | Default is fine |
| `ATC_RAT_RLL/PIT_FLTD` | ≈ `INS_GYRO_FILTER` / 2 | D-term filter |
| `ATC_RAT_RLL/PIT_FLTT` | 0 or equal to `INS_GYRO_FILTER` | |
| `ATC_RAT_YAW_FLTE` | 2 Hz | Default |

---

## 3. Harmonic Notch #1 (RPM-tracked, per-motor)

| Parameter | Value | Notes |
|---|---|---|
| `INS_HNTCH_ENABLE` | 1 | Reboot after setting to expose the other parameters |
| `INS_HNTCH_MODE` | 3 | ESC telemetry (bidirectional DShot RPM) |
| `INS_HNTCH_REF` | 1.0 | Scale factor (RPM-based modes use 1) |
| `INS_HNTCH_FREQ` | ~80% of hover motor frequency, floor ~40–50 Hz | See the formula below |
| `INS_HNTCH_BW` | `INS_HNTCH_FREQ` / 2 | e.g. 40 Hz when FREQ is 80 |
| `INS_HNTCH_ATT` | 40 dB | |
| `INS_HNTCH_HMNCS` | 7 (1st + 2nd + 3rd) | Use 15 to add the 4th for tri-blade props |
| `INS_HNTCH_OPTS` | 14 | Bit 1 per-motor (2) + bit 2 loop-rate update (4) + bit 3 all IMUs (8) |
| `INS_HNTCH_FM_RAT` | 1.0 | 4.5+: minimum frequency ratio. Lower it (e.g. 0.6) if motors idle below FREQ |

**Hover frequency formula:**
f_hover (Hz) = hover RPM / 60. The ESC reports electrical RPM, and `SERVO_BLH_POLES` converts it to mechanical RPM.

*Example:* a 5" quad hovering at 9,600 RPM gives 160 Hz. Set `INS_HNTCH_FREQ` to about 128 Hz and `INS_HNTCH_BW` to about 64 Hz.

**CPU load:** 4 motors × 3 harmonics × 2–3 IMUs is roughly 24–36 biquads, which is trivial on an H743.
- Watch `PM.Load` (should stay under ~60%) and `PM.NLon` (long-loop count).
- If headroom is tight, drop bit 3 (all IMUs) first, then reduce harmonics.

---

## 4. Harmonic Notch #2 (optional static notch for frame resonance)

Use this only if the FFT shows a fixed peak that doesn't track throttle, such as frame or arm resonance.

| Parameter | Value |
|---|---|
| `INS_HNTC2_ENABLE` | 1 |
| `INS_HNTC2_MODE` | 0 (fixed) |
| `INS_HNTC2_FREQ` | Measured peak (e.g. 220 Hz) |
| `INS_HNTC2_BW` | ~25% of FREQ |
| `INS_HNTC2_ATT` | 30 dB |
| `INS_HNTC2_HMNCS` | 1 |

---

## 5. Batch Sampler Logging (to validate)

| Parameter | Value |
|---|---|
| `INS_LOG_BAT_MASK` | 1 (first IMU) |
| `INS_LOG_BAT_OPT` | 4 (pre- and post-filter) |
| `INS_LOG_BAT_LGIN` / `INS_LOG_BAT_LGCT` | 20 / 32 |
| `LOG_BITMASK` | Include IMU_RAW (or use the batch sampler) |

**Validation flight procedure:**
1. Hover for about 30 s.
2. Do gentle throttle sweeps and moderate rolls/pitches.
3. Open the log in the **ArduPilot Filter Review Tool** or **Mission Planner → FFT**.

**What to look for:**
- **Pre-filter:** motor peaks should track the `ESC.RPM`/60 harmonics.
- **Post-filter:** peaks should be attenuated by at least 20 dB, and the noise floor between 50 and 300 Hz should be flat.
- **Residual 1st-harmonic peak:** widen `BW` or enable triple notch (`INS_HNTCH_OPTS` bit 4, +16).
- **Peaks below FREQ at idle or descent:** lower `INS_HNTCH_FREQ` or `FM_RAT`.
- **Clean post-filter spectrum:** raise `INS_GYRO_FILTER` in 10–20 Hz steps, then rerun autotune.

---

## 6. Quick Starting Template (5" quad, H743)

```
MOT_PWM_TYPE      6
SERVO_BLH_BDMASK  15
SERVO_BLH_POLES   14
SCHED_LOOP_RATE   400
INS_GYRO_RATE     1
INS_GYRO_FILTER   80
INS_HNTCH_ENABLE  1
INS_HNTCH_MODE    3
INS_HNTCH_REF     1
INS_HNTCH_FREQ    120
INS_HNTCH_BW      60
INS_HNTCH_ATT     40
INS_HNTCH_HMNCS   7
INS_HNTCH_OPTS    14
INS_HNTCH_FM_RAT  1.0
INS_LOG_BAT_MASK  1
INS_LOG_BAT_OPT   4
```

Run autotune after the notch is validated. Filter changes shift phase lag, so earlier PIDs won't be valid. If you share the log FFT or your frame, prop, and motor specs for "Cycle 29," I can compute exact frequencies and bandwidths.