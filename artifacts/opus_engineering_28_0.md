# ArduPilot Dynamic Harmonic Notch Configuration
**STM32H743 with bidirectional DShot RPM feedback**

> **About "Cycle 28":** This isn't a standard ArduPilot term. I've treated it as iteration 28 of your tuning process, so the parameters below are a structured baseline plus refinement rules. If it refers to something else (a firmware branch, test campaign, or frame revision), tell me and I'll adjust.

---

## 1. Prerequisites: RPM Telemetry Path

The notch can only track what the ESCs report. Confirm this works first.

| Parameter | Value | Notes |
|---|---|---|
| `MOT_PWM_TYPE` | `6` (DShot600) or `5` (DShot300) | Use DShot600 on H743 if ESCs support it |
| `SERVO_BLH_BDMASK` | e.g. `15` (outputs 1–4) | Bitmask of motor outputs with bidirectional DShot |
| `SERVO_BLH_POLES` | `14` | Typical 5" motor (12N14P). **Count your magnets.** |
| `SERVO_DSHOT_ESC` | `1` BLHeli32 / `3` BLHeli_S-Bluejay / `4` AM32 | Match firmware |
| `SERVO_DSHOT_RATE` | `0`–`2` | Higher rates help at high loop rates |

**Verify:** Arm with props off and spin motors. Check `ESC[n].RPM` in the log or the MAVLink ESC telemetry. The values should be stable, non-zero and plausible. A wrong pole count shifts every notch by the ratio error.

---

## 2. Gyro Sampling & Loop Rate (H743)

| Parameter | Value | Rationale |
|---|---|---|
| `INS_FAST_SAMPLE` | `1` (or `3` for dual IMU) | Fast sampling on primary IMU(s) |
| `INS_GYRO_RATE` | `1` (2 kHz) or `2` (4 kHz) | H743 handles 2–4 kHz comfortably |
| `SCHED_LOOP_RATE` | `400` (default); `800` for small/racy frames | Notch updates benefit from higher rates |
| `FSTRATE_ENABLE` | `1` (ArduPilot 4.6+, optional) | Runs the rate loop at gyro rate on H7 |
| `INS_GYRO_FILTER` | `80`–`120` Hz (5"), `40`–`60` Hz (7–10"), `20`–`40` Hz (large) | Raise it once the notch is effective |
| `INS_ACCEL_FILTER` | `10`–`20` Hz | Default is fine |

---

## 3. Primary Harmonic Notch: ESC RPM Tracking

| Parameter | Value | Explanation |
|---|---|---|
| `INS_HNTCH_ENABLE` | `1` | Reboot after setting to expose the other parameters |
| `INS_HNTCH_MODE` | `3` | ESC telemetry (RPM) tracking |
| `INS_HNTCH_REF` | `1` | Scale factor; keep at 1 for ESC mode |
| `INS_HNTCH_FREQ` | ≈ 0.5–0.8 × hover motor frequency (Hz) | Lower clamp; tracking won't go below this |
| `INS_HNTCH_BW` | ≈ ½ × `INS_HNTCH_FREQ` | Notch width at the reference frequency |
| `INS_HNTCH_ATT` | `40` dB | Default; reduce to 30 if phase lag shows up in tuning |
| `INS_HNTCH_HMNCS` | `3` (1st+2nd) or `7` (1st+2nd+3rd) | Use 7 for 3-blade props or visible 3rd harmonic |
| `INS_HNTCH_OPTS` | `2 + 4 + 8 = 14` | See the bit table below |
| `INS_HNTCH_FM_RAT` | `1.0` (default) | Lower to ~0.7 to let the notch follow below FREQ at low throttle |

**`INS_HNTCH_OPTS` bits:**
- `1` Double notch: wider and deeper, but more lag. Usually not needed with per-motor tracking.
- `2` Multi-Source: one notch set per motor. **This is the key RPM-feedback benefit, and the H743 has the CPU for it.**
- `4` Update at loop rate: smoother tracking.
- `8` Enable on all IMUs.
- `16` Triple notch: avoid this with multi-source, because of the CPU and phase cost.

**Computing the FREQ baseline:**
```
f_motor_hover (Hz) = hover_RPM / 60
INS_HNTCH_FREQ     ≈ 0.6 × f_motor_hover   (min clamp)
INS_HNTCH_BW       ≈ INS_HNTCH_FREQ / 2
```

**Example (5" quad, hover ≈ 9,000 RPM, so 150 Hz):**
`FREQ = 90`, `BW = 45`, `HMNCS = 7`, `OPTS = 14`

**CPU load:** 4 motors × 3 harmonics × 3 axes × IMUs is many biquads. Check `PM.Load`/`PM.NLon` stays healthy. Drop to `HMNCS = 3` or remove bit `8` if the scheduler overruns.

---

## 4. Secondary Notch: Static Frame Resonance (Optional)

Use the second notch for a fixed frame or arm resonance that does not scale with RPM.

| Parameter | Value |
|---|---|
| `INS_HNTC2_ENABLE` | `1` |
| `INS_HNTC2_MODE` | `0` (fixed) |
| `INS_HNTC2_FREQ` | Measured resonance peak (e.g. 220 Hz) |
| `INS_HNTC2_BW` | ~¼–½ of FREQ |
| `INS_HNTC2_ATT` | `30`–`40` |
| `INS_HNTC2_HMNCS` | `1` |
| `INS_HNTC2_OPTS` | `0` |

---

## 5. Logging for Verification

| Parameter | Value |
|---|---|
| `INS_LOG_BAT_MASK` | `1` (primary IMU) |
| `INS_LOG_BAT_OPT` | `4` (pre- and post-filter, ArduPilot 4.2+) |
| `LOG_BITMASK` | Include IMU and ESC telemetry |

Turn batch logging off after analysis because it is log-heavy.

**Procedure:**
1. Fly a 30–60 s steady hover plus some throttle sweeps.
2. Load the `.bin` into the **ArduPilot Filter Review tool** (firmware.ardupilot.org/Tools/FilterReview).
3. Compare the pre- and post-filter FFT. Motor peaks and harmonics should be attenuated by more than 20 dB in the post-filter trace.

---

## 6. Iterative Refinement Rules

| Observation in FFT/log | Action |
|---|---|
| Residual peak just below the notch at low throttle | Lower `INS_HNTCH_FREQ` or `INS_HNTCH_FM_RAT` |
| Residual peak at 2× or 3× | Add the harmonic bit in `HMNCS` |
| Broad smeared peak around the fundamental | Increase `BW` by ~25%. Check pole count and prop balance. |
| Peak that doesn't move with throttle | Handle it with the static notch (Section 4) |
| Oscillation or sluggish attitude after enabling | Reduce `ATT`, remove the double-notch bit, or remove harmonics |
| Post-filter noise floor clean | Raise `INS_GYRO_FILTER` in 20 Hz steps, then retune PIDs (QuickTune/AutoTune) |
| High CPU / scheduler overruns | Drop bit `8`, lower `HMNCS`, or reduce `INS_GYRO_RATE` |

---

## 7. Compact Baseline Parameter Set (5" quad example)

```
MOT_PWM_TYPE      6
SERVO_BLH_BDMASK  15
SERVO_BLH_POLES   14
SERVO_DSHOT_ESC   1
INS_FAST_SAMPLE   1
INS_GYRO_RATE     1
INS_GYRO_FILTER   100
INS_HNTCH_ENABLE  1
INS_HNTCH_MODE    3
INS_HNTCH_REF     1
INS_HNTCH_FREQ    90
INS_HNTCH_BW      45
INS_HNTCH_ATT     40
INS_HNTCH_HMNCS   7
INS_HNTCH_OPTS    14
INS_HNTCH_FM_RAT  1.0
INS_LOG_BAT_MASK  1
INS_LOG_BAT_OPT   4
```

Scale `FREQ`, `BW` and `GYRO_FILTER` down for larger props. For a 10" frame, roughly `FREQ 40`, `BW 20`, `GYRO_FILTER 40–60`.

If you share the frame size, prop and motor KV, battery cell count, hover throttle, and your last log's FFT peaks, I can compute exact values for this tuning cycle.