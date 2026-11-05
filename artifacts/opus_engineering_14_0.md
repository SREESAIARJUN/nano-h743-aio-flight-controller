# ArduPilot Dynamic Harmonic Notch Configuration

**Target:** STM32H743 flight controller with bidirectional DShot (ESC RPM telemetry)

> **Note on "Cycle 14":** This isn't a standard ArduPilot term. I've assumed it's your internal tuning iteration. If it instead refers to **14 magnetic poles** (common on 12N14 motors), that maps directly to `SERVO_BLH_POLES = 14` below.

---

## 1. Enable Bidirectional DShot RPM Telemetry

| Parameter | Value | Purpose |
|---|---|---|
| `MOT_PWM_TYPE` | `6` (DShot600) or `5` (DShot300) | Digital ESC protocol |
| `SERVO_BLH_MASK` | `15` (outputs 1–4, quad) | Outputs that use DShot |
| `SERVO_BLH_BDMASK` | `15` | Enables bidirectional DShot on those outputs |
| `SERVO_BLH_POLES` | `14` | Motor magnet count, used to convert eRPM to mechanical RPM |
| `SERVO_BLH_AUTO` | `1` | Automatically maps BLHeli outputs |

**Requirements:**
- The ESCs must run firmware that supports bidirectional DShot (BLHeli_32, Bluejay, or AM32).
- The board must use a `-bdshot` firmware build if its target requires one.

**Verification:** Reboot, spin the motors (props off), and confirm that `ESC[n].RPM` in the log or GCS reads sensibly. The ratio of 14 poles to 7 pole pairs must be correct, or every notch will be misplaced.

---

## 2. Primary Harmonic Notch (RPM-Tracking)

| Parameter | Value | Rationale |
|---|---|---|
| `INS_HNTCH_ENABLE` | `1` | Enable (reboot, then refresh parameters) |
| `INS_HNTCH_MODE` | `3` | Tracks ESC telemetry |
| `INS_HNTCH_REF` | `1` | Scaling reference (unused in ESC mode, keep at 1) |
| `INS_HNTCH_FREQ` | ~80% of hover motor fundamental (e.g. `80`) | Lower tracking limit |
| `INS_HNTCH_BW` | `INS_HNTCH_FREQ / 2` (e.g. `40`) | Notch width |
| `INS_HNTCH_ATT` | `40` dB | Attenuation depth |
| `INS_HNTCH_HMNCS` | `3` (1st + 2nd), or `7` (1st–3rd) | Choose based on the FFT |
| `INS_HNTCH_OPTS` | `6` (bit 1 per-motor + bit 2 loop-rate update) | Explained below |
| `INS_HNTCH_FM_RAT` | `1.0` | Low-throttle frequency ratio |

### OPTS Bitmask

| Bit | Value | Effect |
|---|---|---|
| 0 | 1 | Double notch (wider, deeper) |
| 1 | 2 | **Dynamic per-motor notch**. One notch set per ESC; strongly recommended with bdshot. |
| 2 | 4 | **Update frequency every loop** rather than at 200 Hz. The H743 can afford this. |
| 3 | 8 | Apply to all IMUs (default on recent firmware) |
| 4 | 16 | Triple notch |

**Recommended value:** `6`. Add `+1` (giving `7`) only if post-filter logs show residual broadband energy around the motor peak.

### Fundamental Frequency Calculation

```
f_motor (Hz) = mechanical RPM / 60
```

Example: a hover RPM of 7200 gives 120 Hz. Set `INS_HNTCH_FREQ ≈ 90–100` and `BW ≈ 45–50`.

---

## 3. Secondary Notch (`INS_HNTC2_*`) for Frame Resonance

Use this only if the FFT shows a fixed, throttle-independent peak (arm or frame resonance).

| Parameter | Value |
|---|---|
| `INS_HNTC2_ENABLE` | `1` |
| `INS_HNTC2_MODE` | `0` (static) |
| `INS_HNTC2_FREQ` | Measured peak (e.g. `190`) |
| `INS_HNTC2_BW` | ~¼ to ½ of FREQ |
| `INS_HNTC2_ATT` | `30–40` |
| `INS_HNTC2_HMNCS` | `1` |

**Alternative:** set `MODE = 1` (throttle-based) as a fallback if ESC telemetry is unreliable.

---

## 4. Loop Rate, Gyro Rate and Base Filters (H743)

| Parameter | Value | Notes |
|---|---|---|
| `SCHED_LOOP_RATE` | `400` default; `800` or `1000` for small, high-KV quads | H743 has the headroom |
| `INS_GYRO_RATE` | `1` (2 kHz) or `2` (4 kHz) | Fast sampling, IMU dependent |
| `INS_GYRO_FILTER` | `40–80` Hz (larger craft) or `80–120` Hz (5") | Can be raised once the notches are effective |
| `ATC_RAT_*_FLTD` / `FLTT` | Per tuning | D-term filtering |

### CPU Budget

```
filters ≈ motors × harmonics × IMUs × (1, 2, or 3 for single/double/triple)
```

Example: 4 motors × 2 harmonics × 3 IMUs × 1 = 24 biquads per axis. This is fine on an H743 at 1 kHz.

**Check:** Watch `PM.Load` and `PM.NLon`. Keep load below ~80%.

---

## 5. Logging for Verification

| Parameter | Value | Purpose |
|---|---|---|
| `INS_LOG_BAT_MASK` | `1` | First IMU |
| `INS_LOG_BAT_OPT` | `4` (pre- and post-filter batch) | Add `+1` for sensor-rate logging |
| `INS_RAW_LOG_OPT` | `1` (raw gyro on newer firmware) | Optional, for deeper analysis |
| `LOG_BITMASK` | Include IMU + PID + RCOUT | |

---

## 6. Tuning Workflow

1. **Baseline:** Set up bdshot and confirm RPM telemetry. Fly a 60 s hover with notches disabled, using batch logging.
2. **Analyze:** Open the log in the ArduPilot Filter Review Tool (web). Identify:
   - the motor fundamental
   - its harmonics
   - any static peaks
3. **Configure:**
   - Set `FREQ`/`BW` from the hover peak.
   - Set `HMNCS` to cover the harmonics that are visible.
4. **Simulate:** Load the parameters into the Filter Review Tool against the same log and confirm the post-filter spectrum is attenuated.
5. **Verify in flight:** Fly again with pre- and post-filter logging. The motor peaks should be reduced by more than 20–30 dB.
6. **Iterate:**
   - Raise `INS_GYRO_FILTER` and the D-term filters.
   - Then retune the PIDs, either with AutoTune or manually.

---

## Quick-Load Summary (5"–7" quad, 14-pole motors)

```
MOT_PWM_TYPE      6
SERVO_BLH_MASK    15
SERVO_BLH_BDMASK  15
SERVO_BLH_POLES   14
INS_HNTCH_ENABLE  1
INS_HNTCH_MODE    3
INS_HNTCH_REF     1
INS_HNTCH_FREQ    80
INS_HNTCH_BW      40
INS_HNTCH_ATT     40
INS_HNTCH_HMNCS   3
INS_HNTCH_OPTS    6
INS_HNTCH_FM_RAT  1.0
INS_GYRO_FILTER   80
INS_LOG_BAT_MASK  1
INS_LOG_BAT_OPT   4
```

If you share the hover RPM, prop size, and a batch log, I can calculate exact `FREQ`, `BW`, and harmonic settings rather than these starting values.