# ArduPilot Dynamic Harmonic Notch with Bidirectional DShot RPM (STM32H743)

I don't know what "Cycle 9" refers to in your workflow, such as a tuning iteration or a test-plan revision. So this is a complete baseline you can diff against your previous cycle. Values marked ⚙ depend on your airframe and should come from your own logs.

---

## 1. Prerequisites: RPM Telemetry Path

| Parameter | Value | Notes |
|---|---|---|
| `MOT_PWM_TYPE` | `6` (DShot600) or `5` (DShot300) | DShot300 is more robust on long ESC leads |
| `SERVO_BLH_AUTO` | `1` | Auto-maps the BLHeli outputs to the motors |
| `SERVO_BLH_BDMASK` | `15` (quad), `63` (hex), `255` (octo) | Bitmask of outputs with bidirectional DShot |
| `SERVO_BLH_POLES` | `14` ⚙ | Count the magnets on the motor bell. Typical 22xx/23xx motors have 14 |
| `SERVO_DSHOT_RATE` | `0` or `1` | Higher rates are optional on the H7 |
| `SERVO_BLH_TRATE` | `10` | Only relevant if you also use serial telemetry |

**Additional requirements:**
- **ESC firmware** must support bidirectional DShot: BLHeli_32, Bluejay, or AM32. BLHeli_S without Bluejay does not.
- **Flight controller firmware** must include BDShot. Most current H743 targets include it. Older builds required the `-bdshot` variant.
- **Verification:** in Mission Planner's Status tab, check that `esc1_rpm` through `escN_rpm` read sensibly on the bench with props off. Each motor's RPM must track its own throttle.

---

## 2. Primary Harmonic Notch (ESC Telemetry Tracking)

### Reference frequency formula

f₁ (Hz) = hover RPM / 60. Get the hover RPM from the `ESC` log messages in a hover log, not from estimates.

Example for a 5″ quad hovering at about 11,000 RPM: f₁ ≈ 183 Hz.

### Parameters

| Parameter | Value | Rationale |
|---|---|---|
| `INS_HNTCH_ENABLE` | `1` | Reboot after setting to expose the remaining parameters |
| `INS_HNTCH_MODE` | `3` | ESC telemetry tracking |
| `INS_HNTCH_REF` | `1` | Scaling is unused in ESC mode; leave at 1 |
| `INS_HNTCH_FREQ` | ⚙ ≈ 0.5–0.6 × f₁ (e.g., `90`–`100`) | Floor frequency. The notch will not track below this |
| `INS_HNTCH_BW` | ⚙ ≈ `INS_HNTCH_FREQ / 2` (e.g., `45`–`50`) | Bandwidth of the fundamental notch. It scales up with the harmonics |
| `INS_HNTCH_ATT` | `40` | Attenuation in dB. Raise to 45–50 only if logs show a residual peak |
| `INS_HNTCH_HMNCS` | `3` (1st + 2nd), or `7` if the 3rd harmonic shows | This is a bitmask: 1 = fundamental, 2 = 2nd harmonic, 4 = 3rd |
| `INS_HNTCH_OPTS` | `14` | See the breakdown below |
| `INS_HNTCH_FM_RAT` | `1.0` (default) | Lower (e.g., `0.75`) only if the motors idle below the floor and the noise shows there |

### `INS_HNTCH_OPTS` = 14 breakdown

| Bit | Value | Function |
|---|---|---|
| 1 | 2 | Multi-source: one notch per motor, each tracking its own RPM |
| 2 | 4 | Update the notch frequency at loop rate |
| 3 | 8 | Apply the filter on all IMUs, which matters for EKF lane switching |

Bit 0 (value 1, double notch) is unnecessary with per-motor notches. Add it (OPTS = `15`) only if the peaks remain broad. Use bit 4 (value 16, triple notch) only for severe broadband motor noise.

### CPU load

Per-motor tracking costs: motors × harmonics × IMUs × notches-per-stage. A quad with `HMNCS=3` on 3 IMUs runs 24 biquads per axis, which is trivial for a 480 MHz H743. An octo with `HMNCS=7` and the triple notch is where you should check the `PM` log message for overruns.

---

## 3. Secondary Static Notch (Optional, for Frame Resonance)

Use this only if post-filter logs show a fixed-frequency peak that does not move with throttle, such as arm or frame flex.

| Parameter | Value |
|---|---|
| `INS_HNTC2_ENABLE` | `1` |
| `INS_HNTC2_MODE` | `0` (fixed) |
| `INS_HNTC2_FREQ` | ⚙ The peak frequency from the FFT |
| `INS_HNTC2_BW` | ⚙ About 1/4 to 1/2 of `INS_HNTC2_FREQ` |
| `INS_HNTC2_HMNCS` | `1` |
| `INS_HNTC2_ATT` | `40` |

---

## 4. Gyro Low-Pass and Loop Rate (H743 Headroom)

| Parameter | Value | Notes |
|---|---|---|
| `INS_GYRO_FILTER` | `40`–`80` ⚙ | Raise from the default of 20 only after the notch is verified. Less LPF means less phase lag and better tune |
| `INS_ACCEL_FILTER` | `20` | Default is fine |
| `INS_FAST_SAMPLE` | `1` (or `7` for all IMUs) | Samples at the IMU's native rate |
| `INS_GYRO_RATE` | `1` (2 kHz) or `2` (4 kHz) | 0 = 1 kHz. The H7 handles 2–4 kHz comfortably |
| `SCHED_LOOP_RATE` | `400` (or `800` on small, agile frames) | Retune the PIDs if you change this |
| `ATC_RAT_*_FLTD` / `FLTT` | Raise to about `INS_GYRO_FILTER / 2` | Only after the noise floor is confirmed low |

---

## 5. Logging for Validation

| Parameter | Value | Function |
|---|---|---|
| `INS_LOG_BAT_MASK` | `1` | Batch-sample the first IMU |
| `INS_LOG_BAT_OPT` | `4` | Logs both pre- and post-filter data, so one flight shows both |
| `INS_RAW_LOG_OPT` | `1` (optional) | Logs raw gyro at loop rate. Use only for short tests because of the log size |
| `LOG_BITMASK` | Include IMU and ESC bits | Needed for RPM correlation |

---

## 6. Validation Procedure

1. **Baseline flight:** about 60 s of hover plus some throttle sweeps, with `INS_HNTCH_ENABLE=1`.
2. **Analyze:** open the `.bin` log in the ArduPilot Filter Review Tool (firmware.ardupilot.org/Tools/FilterTool). Compare the pre-filter and post-filter spectra.
3. **Check pass criteria:**
   - The motor fundamental and harmonics are attenuated by at least 20 dB post-filter.
   - The post-filter noise floor stays below about –30 dB across 50–400 Hz, roughly.
   - The notch tracking line follows the `ESC` RPM traces with no dropouts. Dropouts indicate a telemetry issue: check `ESC.Err` and the DShot wiring.
4. **Iterate:**
   - If a residual peak sits below the notch, lower `INS_HNTCH_FREQ`.
   - If a peak is only partly suppressed, widen `BW` or enable the double notch.
   - If a peak shows at 3× the fundamental, set `HMNCS=7`.
   - If a fixed peak remains, configure the second notch.
5. **Raise the LPF:** once the spectrum is clean, raise `INS_GYRO_FILTER` in steps of about 20 Hz, then run an AutoTune.
6. **Record:** log your parameter diffs against the previous iteration so you can track cycle-to-cycle changes.

---

## Quick-Load Summary (5″ Quad Example)

```
MOT_PWM_TYPE      6
SERVO_BLH_AUTO    1
SERVO_BLH_BDMASK  15
SERVO_BLH_POLES   14
INS_HNTCH_ENABLE  1
INS_HNTCH_MODE    3
INS_HNTCH_REF     1
INS_HNTCH_FREQ    90
INS_HNTCH_BW      45
INS_HNTCH_ATT     40
INS_HNTCH_HMNCS   3
INS_HNTCH_OPTS    14
INS_GYRO_FILTER   60
INS_FAST_SAMPLE   1
INS_GYRO_RATE     1
INS_LOG_BAT_MASK  1
INS_LOG_BAT_OPT   4
```

If you share your frame size, motor KV, pole count, battery cell count, or a log from your previous cycle, I can tighten the ⚙ values to your specific airframe.