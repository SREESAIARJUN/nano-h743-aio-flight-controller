# ArduPilot Dynamic Harmonic Notch Configuration
**Target:** STM32H743 flight controller, bidirectional DShot with per-motor RPM telemetry

> **Note on "Cycle 25":** I don't recognize this as an ArduPilot release or standard designation. If it's an internal revision tag or a specific firmware version, verify parameter names against your firmware's parameter list. The values below assume **ArduCopter 4.4/4.5+**.

---

## 1. Prerequisites: Bidirectional DShot RPM Telemetry

The notch can only track motor frequency if the ESCs report valid RPM.

| Parameter | Value | Notes |
|---|---|---|
| `MOT_PWM_TYPE` | `6` (DShot600) or `5` (DShot300) | DShot600 is fine on H7 with short wiring. Use 300 if you see telemetry errors. |
| `SERVO_BLH_AUTO` | `1` | Auto-enables BLHeli passthrough on motor outputs |
| `SERVO_BLH_BDMASK` | e.g. `15` (motors 1–4) | Bitmask of outputs with bidirectional DShot. Use `255` for an octo. |
| `SERVO_BLH_POLES` | `14` | **Must match the motor's magnet count** (count the magnets, typically 12 or 14). A wrong value scales every frequency incorrectly. |
| `SERVO_DSHOT_RATE` | `2`–`4` | Multiple of loop rate. Higher gives fresher RPM data. |
| `SERVO_DSHOT_ESC` | `1` (BLHeli32/BLHeli_S-Bluejay) or `3` (AM32) | ESC firmware type |

**ESC firmware:** BLHeli_32 ≥ 32.7, Bluejay, or AM32 with bidirectional DShot enabled. Stock BLHeli_S does not support it.

**Verify before flying:** In Mission Planner's Status tab, check `esc1_rpm` through `escN_rpm` while spinning the motors on the bench (props off). Each ESC's error rate should stay below about 1%.

---

## 2. Loop Rate and Gyro Sampling (H743 Headroom)

| Parameter | Value | Rationale |
|---|---|---|
| `SCHED_LOOP_RATE` | `400` (default), up to `800` on small craft | H743 handles 800 Hz with multiple notches |
| `INS_GYRO_RATE` | `1` (2 kHz) or `2` (4 kHz) | Fast sampling (4.5+). Higher rates raise CPU load with many notches. |
| `INS_FAST_SAMPLE` | `1` (IMU1) or `3` (IMU1+2) | Enable on IMUs that support it |

---

## 3. Primary Harmonic Notch: ESC Telemetry Tracking

| Parameter | Recommended | Explanation |
|---|---|---|
| `INS_HNTCH_ENABLE` | `1` | Reboot afterward to expose the other parameters |
| `INS_HNTCH_MODE` | `3` | ESC telemetry (RPM-tracked) |
| `INS_HNTCH_REF` | `1` | Scaling reference. Must be 1 for ESC mode. |
| `INS_HNTCH_FREQ` | ≈ 0.5–0.8 × hover motor freq | Minimum tracking frequency (see formula below) |
| `INS_HNTCH_BW` | ≈ `FREQ / 2` | Notch bandwidth in Hz |
| `INS_HNTCH_ATT` | `40` | Attenuation in dB |
| `INS_HNTCH_HMNCS` | `3` (1st+2nd) or `7` (1st–3rd) | Harmonics bitmask. Start with 3. Add the 3rd if logs show it. |
| `INS_HNTCH_OPTS` | `6` | Bit 1 = per-motor notches (multi-source), bit 2 = update at loop rate |
| `INS_HNTCH_FM_RAT` | `1.0` | Lower bound ratio. Below 1 lets notches drop under `FREQ` at low throttle. |

### Frequency formula

$$f_{motor}\,(\text{Hz}) = \frac{\text{RPM}}{60}$$

The ESC reports mechanical RPM after the `SERVO_BLH_POLES` conversion, so the fundamental equals RPM/60. Harmonic *n* sits at *n × f_motor*.

**Setting `INS_HNTCH_FREQ`:** take the hover RPM from a log (`ESC.RPM`), convert to Hz, and multiply by about 0.6–0.8. Keep it above about 30–40 Hz so the notch doesn't add phase lag in the control band.

### Why `OPTS = 6` on H743
- **Multi-source (bit 1):** places a separate notch on each motor's actual frequency instead of one averaged notch. On a quad with harmonics 1+2, that is 8 notches per axis.
- **Loop-rate update (bit 2):** recomputes notch coefficients every loop, so tracking stays tight during rapid throttle changes.
- The H743's FPU and clock handle this load comfortably at 400–800 Hz. Watch `PM.Load` and `PM.NLon` in logs. Load should stay below about 80%.

Avoid double/triple-notch options (bits 0/4) with multi-source unless analysis shows wide peaks. They multiply the notch count and phase lag.

---

## 4. Example Configurations

### 5" FPV quad (2207 1900–2400 KV, 6S)
Hover ≈ 10,800 RPM → **180 Hz**
```
INS_HNTCH_ENABLE  1
INS_HNTCH_MODE    3
INS_HNTCH_REF     1
INS_HNTCH_FREQ    120
INS_HNTCH_BW      60
INS_HNTCH_ATT     40
INS_HNTCH_HMNCS   3
INS_HNTCH_OPTS    6
INS_GYRO_FILTER   100
SCHED_LOOP_RATE   800
```

### 7"–10" long-range / mid-size quad
Hover ≈ 5,400 RPM → **90 Hz**
```
INS_HNTCH_FREQ    60
INS_HNTCH_BW      30
INS_HNTCH_HMNCS   3
INS_HNTCH_OPTS    6
INS_GYRO_FILTER   60
SCHED_LOOP_RATE   400
```

### Large multirotor (15"+ props)
Hover ≈ 3,000 RPM → **50 Hz**
```
INS_HNTCH_FREQ    35
INS_HNTCH_BW      18
INS_HNTCH_HMNCS   1     (add 2nd only if the logs show it)
INS_GYRO_FILTER   40
```

---

## 5. Secondary Notch (`INS_HNT2_*`): Non-Motor Resonances

Use the second notch for frame or arm resonance that does **not** track RPM, or as an FFT-based backup.

| Use case | Settings |
|---|---|
| Fixed frame resonance (e.g., 220 Hz arm flex) | `INS_HNT2_ENABLE=1`, `INS_HNT2_MODE=0`, `INS_HNT2_FREQ=220`, `INS_HNT2_BW=40`, `INS_HNT2_ATT=30`, `INS_HNT2_HMNCS=1` |
| In-flight FFT tracking | `FFT_ENABLE=1`, `INS_HNT2_MODE=4`, `FFT_MINHZ`/`FFT_MAXHZ` bracketing the peak |
| Throttle-based fallback | `INS_HNT2_MODE=1`, with `INS_HNT2_REF` = hover throttle (`MOT_THST_HOVER`) |

Leave it disabled unless the post-filter spectrum shows a residual peak.

---

## 6. Low-Pass Filters After Notching

| Parameter | Guidance |
|---|---|
| `INS_GYRO_FILTER` | Raise once notches are effective (5": 80–120 Hz, 10": 50–70 Hz). This is the main latency payoff of good notching. |
| `INS_ACCEL_FILTER` | Leave at 20 Hz |
| `ATC_RAT_*_FLTD` | Rate D-term filter. Can often go from 20 up to about 30–40 Hz on small craft once noise is controlled. |

---

## 7. Validation Workflow

1. **Baseline log (notch off or default):**
   - `INS_LOG_BAT_MASK = 1`
   - `INS_LOG_BAT_OPT = 4` (pre- and post-filter capture, 4.2+). Use `2` for post-filter only.
   - Hover for 60 s or more, plus some throttle sweeps.
2. **Analyze** with the ArduPilot Web Filter Review Tool or the Mission Planner FFT: identify the fundamental and harmonic peaks and confirm they line up with `ESC.RPM / 60`.
3. **Apply the settings** above and fly the same profile.
4. **Confirm:**
   - The post-filter spectrum shows peaks reduced by about 20–40 dB.
   - Log message `FTN1` shows notch frequencies tracking the ESC RPM.
   - Motor temperatures are lower after a 2-minute hover. Hot motors mean noise is passing into the D-term.
   - `PM.Load` stays within budget.
5. **Retune:** rerun QuickTune/AutoTune afterward. Lower phase lag usually supports higher gains.

---

## 8. Common Pitfalls

| Symptom | Likely cause |
|---|---|
| Notch frequency is off by a fixed ratio | Wrong `SERVO_BLH_POLES` |
| Notch stuck at `INS_HNTCH_FREQ` | No RPM data. Check `SERVO_BLH_BDMASK`, ESC firmware, and DShot rate. |
| Oscillation at low throttle | `INS_HNTCH_FREQ` too low, adding phase lag in the control band. Raise it. |
| CPU overload / slow loop warnings | Too many harmonics × motors × IMUs. Trim `HMNCS`, drop the loop rate, or disable the all-IMU option. |
| Residual broadband noise | Mechanical issue (props, bearings, FC mounting). Filters can't fully fix it. |

If you share your frame size, motor KV, cell count, and a pre-filter log FFT, I can calculate exact `FREQ`/`BW`/`HMNCS` values.