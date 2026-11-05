# ArduPilot Dynamic Harmonic Notch Configuration (STM32H743, Bidirectional DShot RPM)

I don't have context for what "Cycle 5" refers to. I've treated it as a **fifth refinement pass**: the RPM source is assumed working, and this pass tightens notch placement based on prior logs. If it means something specific in your project, such as a test plan stage, tell me and I'll adjust.

These settings target **ArduCopter 4.4–4.6**. Bitmask meanings shift between versions, so confirm each bit against the parameter documentation for your firmware.

---

## 1. Prerequisites: RPM Telemetry Path

| Parameter | Value | Notes |
|---|---|---|
| `MOT_PWM_TYPE` | `6` (DShot600) | Use `5` (DShot300) if the ESC or wiring is marginal |
| `SERVO_BLH_BDMASK` | e.g. `15` | Bitmask of outputs with bidirectional DShot (motors 1–4 = 15) |
| `SERVO_BLH_POLES` | `14` | **Must match the motor magnet count.** Typical 5" motors have 14; small motors often have 12 |
| `SERVO_DSHOT_ESC` | `1` (BLHeli32) or `3` (AM32) | Per ESC firmware. BLHeli_S also needs Bluejay firmware |
| `SERVO_DSHOT_RATE` | `0`–`2` | Higher values improve RPM sample freshness |

**Check:** in the MAVLink inspector, `ESC_TELEMETRY_1_TO_4` should show RPM on every motor. RPM error rate should stay below about 1%; check the `EDT2`/`ESC` log messages.

Motor fundamental frequency:

$$f_{motor}\,[\text{Hz}] = \frac{\text{RPM}_{mech}}{60}, \qquad \text{RPM}_{mech} = \frac{\text{eRPM}}{\text{poles}/2}$$

---

## 2. Gyro Sampling and Loop Rate (H743)

| Parameter | Value | Rationale |
|---|---|---|
| `INS_GYRO_RATE` | `1` (2 kHz), or `2` (4 kHz) for small, fast quads | The H7 has CPU headroom for high gyro rates |
| `SCHED_LOOP_RATE` | `400` (large frames) / `800`–`1000` (≤7") | Keep above about 2× the highest notch frequency |
| `FSTRATE_ENABLE` | `1` (4.5+), optional | Runs the rate loop at gyro rate |
| `INS_GYRO_FILTER` | `40`–`80` Hz (5"); `20`–`40` Hz (10"+) | Raise from the default 20 Hz **only after** the notches are effective |
| `INS_ACCEL_FILTER` | `10`–`20` | |

---

## 3. Harmonic Notch #1: RPM-Tracking (Primary)

| Parameter | Value | Explanation |
|---|---|---|
| `INS_HNTCH_ENABLE` | `1` | Reboot afterward to expose the remaining parameters |
| `INS_HNTCH_MODE` | `3` | ESC telemetry (DShot RPM) |
| `INS_HNTCH_REF` | `1.0` | Scaling reference. Keep at 1 in ESC mode |
| `INS_HNTCH_FREQ` | ≈ 0.8 × idle-to-hover fundamental (e.g. `80` Hz on a 5", `40` Hz on a 10") | Floor frequency. Below this, the notch clamps rather than tracking |
| `INS_HNTCH_BW` | ≈ ½ × `INS_HNTCH_FREQ` (e.g. `40`) | Narrower is acceptable because RPM tracking is precise |
| `INS_HNTCH_ATT` | `40` dB | Range 30–50 dB |
| `INS_HNTCH_HMNCS` | `7` (1st+2nd+3rd) | Add 4th (`15`) if the logs show it. Keep the count minimal to save CPU and phase |
| `INS_HNTCH_OPTS` | `6` (Multi-Source + Loop-rate update) | Bit 1 = per-motor notches; bit 2 = update at loop rate |
| `INS_HNTCH_FM_RAT` | `1.0` | Throttle-low clamp ratio. The RPM source is reliable, so no extra lowering is needed |

### Notes on `INS_HNTCH_OPTS`

- **Bit 1, Multi-Source:** places a notch per motor per harmonic. This is highly effective because motors spin at different speeds in maneuvers.
- **CPU cost:** notches = motors × harmonics × IMUs. For a quad with 3 harmonics on 2 IMUs, that is 24 notches. This is fine on an H743, but watch the `PM` log for load and long loops.
- **Bit 3, EnableOnAllIMUs:** default behavior in recent versions. Set it explicitly if your version requires it.
- **Double/Triple notch bits:** add these only if the residual peak is broad. They are usually unnecessary with Multi-Source.

---

## 4. Harmonic Notch #2: Static Frame Resonance (Optional)

Use this only if the FFT shows a **fixed-frequency** peak that doesn't move with throttle, such as arm or stack resonance.

| Parameter | Value |
|---|---|
| `INS_HNTC2_ENABLE` | `1` |
| `INS_HNTC2_MODE` | `0` (fixed) |
| `INS_HNTC2_FREQ` | Measured peak (e.g. `180`) |
| `INS_HNTC2_BW` | ≈ ¼–½ × FREQ |
| `INS_HNTC2_ATT` | `30`–`40` |
| `INS_HNTC2_HMNCS` | `1` |

---

## 5. Logging for Validation

| Parameter | Value |
|---|---|
| `INS_LOG_BAT_MASK` | `1` (first IMU) |
| `INS_LOG_BAT_OPT` | `4` (pre- and post-filter capture). In older versions, use `2` and fly twice |
| `INS_RAW_LOG_OPT` | `1` (4.5+, raw gyro log, alternative to batch sampler) |
| `LOG_BITMASK` | Include IMU, plus `FTN` notch-frequency logging |

---

## 6. Cycle-5 Refinement Procedure

1. **Fly** a 60 s hover plus moderate throttle sweeps and rolls. Avoid hard maneuvers on the first flight after changes.
2. **Analyze** in the ArduPilot Filter Review Tool (web) or the Mission Planner FFT:
   - Compare pre- and post-filter spectra.
   - Confirm the `FTN` notch-frequency trace overlays the motor peak across all throttle.
3. **Apply these criteria:**
   - Post-filter motor peaks are at least 20–30 dB below pre-filter.
   - Post-filter noise floor below the gyro filter cutoff is under about 0.1 (deg/s)²/Hz.
   - There is no unexplained residual peak. If a harmonic remains, add it to `HMNCS`.
4. **Iterate only one group at a time:**
   - Residual at idle or low throttle: lower `INS_HNTCH_FREQ`.
   - Residual is broad: increase `BW` or enable the double notch.
   - Notches are clean: raise `INS_GYRO_FILTER` in 10–20 Hz steps, check the motor temperature after each step, then re-tune PIDs or rerun AutoTune.
5. **Check CPU health:** in the `PM` log, `NLon` (long loops) should stay near 0 and load under about 80%. If it is too high, reduce harmonics or set `INS_GYRO_RATE` one step lower.

---

## 7. Example Final Set (5" Quad, 14-Pole Motors, H743)

```
MOT_PWM_TYPE      6
SERVO_BLH_BDMASK  15
SERVO_BLH_POLES   14
INS_GYRO_RATE     1
SCHED_LOOP_RATE   800
INS_HNTCH_ENABLE  1
INS_HNTCH_MODE    3
INS_HNTCH_REF     1
INS_HNTCH_FREQ    80
INS_HNTCH_BW      40
INS_HNTCH_ATT     40
INS_HNTCH_HMNCS   7
INS_HNTCH_OPTS    6
INS_HNTCH_FM_RAT  1.0
INS_GYRO_FILTER   60
INS_LOG_BAT_MASK  1
INS_LOG_BAT_OPT   4
```

If you share the frame size, motor KV/poles, firmware version, or a post-flight FFT, I can give you exact `FREQ`, `BW`, and `HMNCS` values for this cycle.