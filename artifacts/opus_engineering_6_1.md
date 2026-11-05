# ArduPilot Harmonic Notch Configuration: STM32H743 + Bidirectional DShot RPM

**About "Cycle 6":** I don't know what that refers to in your project, such as a tuning iteration or a test-plan stage. If you share the previous cycle's logs or parameters, I can make this a delta from them. The configuration below is a complete baseline for ArduPilot 4.3+ (4.4/4.5 recommended).

---

## 1. Prerequisites: RPM Telemetry Source

Bidirectional DShot must work before any notch tuning. Verify with `ESC` messages in the log and RPM values in the GCS status.

| Parameter | Value | Notes |
|---|---|---|
| `MOT_PWM_TYPE` | `6` (DShot600) or `5` (DShot300) | DShot300 is often more robust for bidirectional DShot on long leads |
| `SERVO_BLH_BDMASK` | e.g. `15` (outputs 1–4), `255` (1–8) | Bitmask of outputs with bidirectional DShot. Use FMU outputs, not IOMCU |
| `SERVO_BLH_POLES` | `14` (typical 5" motors), `12` for some larger motors | Count the stator magnets. A wrong value scales every notch frequency incorrectly |
| `SERVO_BLH_AUTO` | `1` | Optional. Auto-maps outputs |
| `SERVO_DSHOT_ESC` | ESC type (e.g. `1` BLHeli32/AM32, `3` BLHeli_S w/ Bluejay) | Required for correct command handling |

ESC firmware must support bidirectional DShot: BLHeli32, AM32, or Bluejay. Stock BLHeli_S does not.

**Validation:** Hover and check that `ESC.RPM` is smooth on all motors and that the ESC error rate is below about 1%. In 4.4+, `ESC.Err` or the `SERVO_DSHOT_RATE`-related stats show this.

---

## 2. Primary Harmonic Notch: RPM-Tracked, Per-Motor

| Parameter | Value | Rationale |
|---|---|---|
| `INS_HNTCH_ENABLE` | `1` | Reboot after setting to expose the other parameters |
| `INS_HNTCH_MODE` | `3` | ESC telemetry (RPM) tracking |
| `INS_HNTCH_REF` | `1` | Scaling is unused for RPM mode. Keep at 1 |
| `INS_HNTCH_FREQ` | ~`0.8 × hover motor freq` (e.g. `80` Hz for 5"; `40–50` for 10"+) | Minimum tracking frequency (floor) |
| `INS_HNTCH_BW` | `FREQ / 2` (e.g. `40`) | Starting point. Widen if post-filter peaks remain |
| `INS_HNTCH_ATT` | `40` dB | Default. Rarely needs to change |
| `INS_HNTCH_HMNCS` | `7` (1st+2nd+3rd) | Bitmask. Use `3` if the 3rd harmonic is absent in the FFT, which saves CPU |
| `INS_HNTCH_OPTS` | `6` | Bit 1 (2) = per-motor notches. Bit 2 (4) = update at loop rate |
| `INS_HNTCH_FM_RAT` | `1.0` (default) | Lower to ~`0.7` to let the notch track below `FREQ` on rapid throttle chops |

Hover motor frequency is `hover RPM / 60`. Read it from `ESC.RPM` in a hover log.

**Optional option bits:**
- **+1 (double notch):** use if a single notch per harmonic is insufficient on noisy frames.
- **+16 (triple notch, 4.5+):** for large props with broad peaks.
- **+8:** enables the notch on all IMUs. This is default behavior on newer versions; check your version.

**H743 CPU budget:** total notch count ≈ motors × harmonics × (1/2/3 for single/double/triple), applied per IMU. Guidelines:
- A quad with 3 harmonics and a single notch is trivial for the H743.
- An octo with 3 harmonics, a double notch, and 2 IMUs is heavy. Watch `PM.Load` (should stay below ~60%) and `PM.NLon` (long loops).

---

## 3. Secondary Notch: Static Frame Resonance (if present)

Use this only if the FFT shows a fixed-frequency peak that does not move with throttle, such as arm or frame resonance:

| Parameter | Value |
|---|---|
| `INS_HNT2_ENABLE` | `1` |
| `INS_HNT2_MODE` | `0` (fixed) |
| `INS_HNT2_FREQ` | measured peak (e.g. `180`) |
| `INS_HNT2_BW` | ~`FREQ / 4` |
| `INS_HNT2_ATT` | `30–40` |
| `INS_HNT2_HMNCS` | `1` |

---

## 4. Sampling and Low-Pass Filters

| Parameter | Value | Notes |
|---|---|---|
| `INS_FAST_SAMPLE` | `1` (or IMU mask) | Default on H7 |
| `INS_GYRO_RATE` | `1` (2 kHz) baseline; `2` (4 kHz) for small, fast quads | Higher rates increase CPU load from the notches |
| `SCHED_LOOP_RATE` | `400` (≥7"), `800` for 3–5" if `PM.Load` permits | |
| `INS_GYRO_FILTER` | `20` → raise to `40–80` once the notch is effective | The main benefit of good notching is less low-pass lag |
| `INS_ACCEL_FILTER` | `20` | |
| `ATC_RAT_*_FLTD` / `FLTT` | Keep ≤ `INS_GYRO_FILTER`; D-term often ~½ of it | |

---

## 5. Logging for Validation

| Parameter | Value |
|---|---|
| `INS_LOG_BAT_MASK` | `1` (first IMU) |
| `INS_LOG_BAT_OPT` | `4` (pre+post filter, 4.3+). Older versions: run `0`, then `2` |
| `LOG_BITMASK` | include IMU_RAW / FAST_IMU as needed |

**Procedure:**
1. Fly a 30–60 s hover with gentle throttle sweeps and some forward flight.
2. Load the log into the **ArduPilot Filter Review Tool** (firmware.ardupilot.org/Tools/FilterReview).
3. Compare pre-filter and post-filter spectra:
   - Motor peaks should be attenuated by at least ~20 dB post-filter.
   - Remaining peaks between harmonics usually mean the `SERVO_BLH_POLES` value is wrong.
   - A residual peak below `INS_HNTCH_FREQ` means you should lower `FREQ` or `FM_RAT`.
   - If peaks are broad, increase `BW` or enable the double notch.
4. Iterate on `BW`, `HMNCS`, and `OPTS`. Then raise `INS_GYRO_FILTER` and re-run autotune.

---

## 6. Example Baseline: 5" Quad, 14-Pole Motors, Hover ~6,000 RPM (100 Hz)

```
MOT_PWM_TYPE        6
SERVO_BLH_BDMASK    15
SERVO_BLH_POLES     14
INS_HNTCH_ENABLE    1
INS_HNTCH_MODE      3
INS_HNTCH_REF       1
INS_HNTCH_FREQ      80
INS_HNTCH_BW        40
INS_HNTCH_ATT       40
INS_HNTCH_HMNCS     7
INS_HNTCH_OPTS      6
INS_HNTCH_FM_RAT    1.0
INS_GYRO_RATE       1
INS_GYRO_FILTER     60
SCHED_LOOP_RATE     800
INS_LOG_BAT_MASK    1
INS_LOG_BAT_OPT     4
```

Scale `FREQ`, `BW`, and `GYRO_FILTER` down proportionally for larger airframes. For example, a 10" frame might use `FREQ 45`, `BW 22`, `GYRO_FILTER 30`, and loop rate 400.

If you share a pre/post-filter batch log, or your previous cycle's parameter file, I can refine these values for your airframe.