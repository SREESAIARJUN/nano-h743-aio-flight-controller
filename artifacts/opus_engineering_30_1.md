# ArduPilot Harmonic Notch Configuration: H743 with Bidirectional DShot RPM

I'm not sure what "Cycle 30" refers to. It might be your own iteration label, a firmware build, or a tuning log number. The settings below are a complete baseline for ArduPilot 4.4 or later on an STM32H743. If "Cycle 30" implies specific constraints, such as an existing log or frame size, share them and I'll adjust the numbers.

---

## 1. Prerequisites: RPM Telemetry Path

| Parameter | Value | Notes |
|---|---|---|
| `MOT_PWM_TYPE` | 6 (DShot600) | Use 5 (DShot300) if the ESC is limited |
| `SERVO_BLH_BDMASK` | 15 (quad), 63 (hex), 255 (octo) | Bitmask of motor outputs with bidirectional DShot |
| `SERVO_BLH_POLES` | 14 | Motor magnet count. Most 5" motors have 14 and smaller motors often 12. Verify yours, because a wrong value scales the notch to the wrong frequency. |
| `SERVO_DSHOT_RATE` | 0–2 | Loop-rate multiple. 0 is usually fine at 400 Hz. |
| `SERVO_BLH_AUTO` | 1 | Optional. Auto-maps motor outputs. |

- **ESC firmware** must support bidirectional DShot (EDT): BLHeli_32, Bluejay, or AM32.
- **Firmware build:** use an H7 firmware with bidirectional DShot enabled (most `*-bdshot` or standard H7 targets).
- **Verify before tuning:** confirm `ESC[n].RPM` values appear in the logs or in the Mission Planner status tab.

---

## 2. Sampling and Loop Rates (H743 Headroom)

| Parameter | Value | Notes |
|---|---|---|
| `INS_GYRO_RATE` | 1 (2 kHz) or 2 (4 kHz) | The H7 handles 2 kHz comfortably with 12+ notches |
| `SCHED_LOOP_RATE` | 400 | Use 800–1000 for small, high-KV craft |
| `INS_FAST_SAMPLE` | 1 (bit for primary IMU) | |
| `INS_GYRO_FILTER` | 60–80 Hz | Raise it after the notch is working. 5" craft often run 80–120 Hz. |
| `INS_ACCEL_FILTER` | 10–20 Hz | |

---

## 3. Primary Harmonic Notch (Dynamic, RPM-Driven, Per-Motor)

| Parameter | Value | Rationale |
|---|---|---|
| `INS_HNTCH_ENABLE` | 1 | Reboot after setting to expose the other parameters |
| `INS_HNTCH_MODE` | 3 | ESC telemetry (bidirectional DShot RPM) |
| `INS_HNTCH_REF` | 1.0 | Scaling factor. Keep at 1 for ESC RPM. |
| `INS_HNTCH_FREQ` | ~0.6–0.8 × hover motor frequency | Acts as the *minimum* tracking frequency. Example: hover at 6000 RPM is 100 Hz, so set about 70 Hz. |
| `INS_HNTCH_BW` | ≈ FREQ / 2 | Example: 35 Hz |
| `INS_HNTCH_ATT` | 40 dB | |
| `INS_HNTCH_HMNCS` | 7 | 1st, 2nd and 3rd harmonics. Add 4th (15) if the FFT shows it. |
| `INS_HNTCH_OPTS` | 2 | bit1 = one notch set per motor. Optionally add bit2 (4) to update at loop rate. Use bit3 (8) to apply to all IMUs. |
| `INS_HNTCH_FM_RAT` | 1.0 | Minimum-frequency ratio. Lower it (e.g. 0.7) to let the notch track below FREQ at low throttle. |

**CPU load check:** 4 motors × 3 harmonics × 3 axes gives 36 biquads per IMU at 2 kHz. This is fine on the H743. Watch `PM.Load` and keep it below about 60%.

**Per-motor versus averaged:** per-motor tracking (OPTS bit1) is the main advantage of RPM feedback. It handles asymmetric motor speeds during maneuvers. Single-notch averaged tracking is only worth using if CPU is constrained, which is unlikely on an H7.

---

## 4. Secondary Notch (Optional, Fixed Frame Resonance)

Use this only if the FFT shows a static peak that doesn't move with RPM, such as an arm or frame mode.

| Parameter | Value |
|---|---|
| `INS_HNTC2_ENABLE` | 1 |
| `INS_HNTC2_MODE` | 0 (fixed) |
| `INS_HNTC2_FREQ` | Measured peak (e.g. 180 Hz) |
| `INS_HNTC2_BW` | 20–40 Hz |
| `INS_HNTC2_ATT` | 30–40 dB |
| `INS_HNTC2_HMNCS` | 1 |

---

## 5. Validation Workflow

1. **Baseline log:** set `INS_LOG_BAT_MASK = 1` and `INS_LOG_BAT_OPT = 4`. Option 4 logs pre- and post-filter data in the same flight.
2. **Fly** a 60 s hover plus some throttle sweeps.
3. **Analyze** with the ArduPilot Filter Review web tool or the Mission Planner FFT. Confirm:
   - The pre-filter peaks track RPM×harmonic lines.
   - Post-filter attenuation is at least 20–30 dB at each harmonic.
   - The minimum `ESC.RPM` during flight produces a frequency at or above `INS_HNTCH_FREQ × FM_RAT`.
4. **Then:** raise `INS_GYRO_FILTER`, reduce `ATC_RAT_*_FLTD`/`FLTT` lag, and run AutoTune.
5. **Clean up:** set `INS_LOG_BAT_MASK = 0` after tuning to save log bandwidth.

---

## 6. Example: 5" Quad, 2207 1750KV, 6S

```
MOT_PWM_TYPE      6
SERVO_BLH_BDMASK  15
SERVO_BLH_POLES   14
INS_GYRO_RATE     1
SCHED_LOOP_RATE   800
INS_HNTCH_ENABLE  1
INS_HNTCH_MODE    3
INS_HNTCH_REF     1
INS_HNTCH_FREQ    120
INS_HNTCH_BW      60
INS_HNTCH_ATT     40
INS_HNTCH_HMNCS   7
INS_HNTCH_OPTS    6      # per-motor + loop-rate update
INS_HNTCH_FM_RAT  0.8
INS_GYRO_FILTER   100
```

**Common pitfalls:**
- A wrong `SERVO_BLH_POLES` value puts every notch at the wrong frequency.
- ESC RPM dropouts make the notch fall back to the minimum frequency. Check `ESC.Err` or the error-rate fields.
- Forgetting the reboot after `INS_HNTCH_ENABLE` hides the other parameters.