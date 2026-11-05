# ArduPilot Dynamic Harmonic Notch Configuration
**Target:** STM32H743 flight controller, bidirectional DShot RPM telemetry

I don't know what "Cycle 20" refers to in your context, such as a tuning iteration, curriculum step or firmware build. Below is a complete baseline configuration and the procedure to refine it. If "Cycle 20" implies specific constraints, such as frame size or a log you've already captured, share them and I'll tailor the numbers.

---

## 1. Prerequisites: Bidirectional DShot and RPM Telemetry

| Parameter | Value | Notes |
|---|---|---|
| `MOT_PWM_TYPE` | 6 (DShot600) | Use 5 (DShot300) on large, slow props if you see telemetry errors |
| `SERVO_BLH_BDMASK` | e.g. 15 | Bitmask of outputs with bidirectional DShot (motors 1–4 = 15, 1–8 = 255) |
| `SERVO_BLH_POLES` | 14 | **Motor magnet pole count.** 14 is typical for 5"; count the magnets. A wrong value gives a wrong notch frequency. |
| `SERVO_BLH_AUTO` | 1 | Optional, enables BLHeli passthrough |
| `SERVO_DSHOT_ESC` | 1 (BLHeli32/AM32) or 3 (BLHeli_S/Bluejay) | Match your ESC firmware |
| `SERVO_DSHOT_RATE` | 0–2 | Rate multiplier. The H743 handles 2 easily at 1 kHz+ loops. |

**Verify before going further:** in Mission Planner → Status (or the `ESC` log message), confirm all motors report sensible RPM and error rates are near 0%.

---

## 2. Primary Dynamic Harmonic Notch (INS_HNTCH_*)

| Parameter | Value | Rationale |
|---|---|---|
| `INS_HNTCH_ENABLE` | 1 | Enable (reboot, then refresh params) |
| `INS_HNTCH_MODE` | **3** | ESC telemetry (RPM) tracking |
| `INS_HNTCH_REF` | 1.0 | Scaling reference. Leave at 1 for ESC mode. |
| `INS_HNTCH_FREQ` | **≈ 0.8 × hover motor freq** | The floor frequency. The notch won't track below it (see `FM_RAT`). |
| `INS_HNTCH_BW` | **≈ FREQ / 2** | Notch bandwidth in Hz |
| `INS_HNTCH_ATT` | 40 | Attenuation in dB |
| `INS_HNTCH_HMNCS` | **3** | Fundamental + 2nd harmonic (use 7 to add the 3rd if logs show it) |
| `INS_HNTCH_OPTS` | **6** | Bit 1 (2) = per-motor notches; bit 2 (4) = update at loop rate |
| `INS_HNTCH_FM_RAT` | 0.7–1.0 | Lets the notch drop below FREQ at low throttle (e.g. 0.7 = 70% of FREQ) |

**Why OPTS = 6 on an H743:** A per-motor notch places a separate notch on each motor's actual RPM rather than the average. This handles yaw and asymmetric loading far better. A quad with 2 harmonics, 3 axes and per-motor notches runs 24 biquads per IMU, well within the H743's budget.

Optional additions to the bitmask:
- **+16 (triple notch):** use if noise peaks are broad. Costs more CPU.
- **+8 (all IMUs):** use if `INS_ENABLE_MASK` uses multiple IMUs. Often already the default in 4.4+.

### Calculating the frequency

```
Motor freq (Hz) = eRPM / (poles/2) / 60  = mechanical RPM / 60
```

Read hover mechanical RPM from the ESC log (`ESC.RPM`) in a hover flight.

**Example (5" quad):** hover ≈ 10,800 RPM → 180 Hz.
- `INS_HNTCH_FREQ` = 145
- `INS_HNTCH_BW` = 72

**Typical starting points by frame size:**

| Frame | Hover freq | FREQ | BW |
|---|---|---|---|
| 3" | 250–350 Hz | 200 | 100 |
| 5" | 150–220 Hz | 120–150 | 60–75 |
| 7" | 100–140 Hz | 80 | 40 |
| 10"+ | 50–90 Hz | 40–60 | 20–30 |

---

## 3. Secondary Notch (INS_HNTC2_*) for Static Frame Resonance

Use this only if post-filter FFT shows a fixed peak that doesn't move with throttle, such as arm or frame resonance.

| Parameter | Value |
|---|---|
| `INS_HNTC2_ENABLE` | 1 |
| `INS_HNTC2_MODE` | 0 (fixed) |
| `INS_HNTC2_FREQ` | Measured resonance peak |
| `INS_HNTC2_BW` | ~¼–½ of FREQ |
| `INS_HNTC2_ATT` | 40 |
| `INS_HNTC2_HMNCS` | 1 |

---

## 4. Supporting Filters and Loop Rate

| Parameter | Value | Notes |
|---|---|---|
| `SCHED_LOOP_RATE` | 400 (default) to 800+ | H743 handles higher rates. Raise in steps and check CPU load (`PM.Load`). |
| `INS_GYRO_RATE` | 1 (2 kHz) or 2 (4 kHz) | Fast sampling; the H743 supports it |
| `INS_GYRO_FILTER` | 80–120 (5") / 40–60 (large) | With good notching, the LPF can be raised to cut latency |
| `ATC_RAT_*_FLTD` | ~ half of `INS_GYRO_FILTER` | D-term filter |

---

## 5. Verification Workflow

1. **Set up logging.** Set `INS_LOG_BAT_MASK` = 1 (first IMU) and `INS_LOG_BAT_OPT` = 4 (pre- and post-filter capture). Alternatively use 2 (post-filter only) with batch logging at sensor rate.
2. **Fly a test.** Hover for 30–60 s, then do some gentle throttle sweeps.
3. **Analyze.** In Mission Planner use Ctrl+F → FFT, or use the ArduPilot online Filter Review Tool. Check that:
   - The pre-filter plot shows peaks at the motor fundamental and harmonics.
   - The post-filter plot shows those peaks suppressed by ≥20–30 dB.
   - Any remaining static peak gets a `HNTC2` notch.
4. **Clean up the logging.** Disable batch logging (`INS_LOG_BAT_MASK` = 0) after tuning.
5. **Retune the rate PIDs.** Run a QuickTune or AutoTune now that the noise floor is lower.

---

## 6. Common Pitfalls

- **Wrong `SERVO_BLH_POLES`:** the notch sits at a multiple of the true frequency. Always cross-check the FFT peak against the ESC RPM.
- **FREQ set too high:** the notch can't follow at low throttle, which causes oscillation on descent. Lower FREQ or `FM_RAT`.
- **BW too wide:** adds phase lag and makes the craft feel soft.
- **Missing RPM on one motor:** per-motor mode then loses that notch. Check `SERVO_BLH_BDMASK` and the ESC firmware.

If you share a `.bin` log or your frame, motor and prop specs, I can compute exact FREQ and BW values.