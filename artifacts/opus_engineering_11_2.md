# ArduPilot Harmonic Notch Configuration: STM32H743 with Bidirectional DShot RPM

This setup uses per-motor RPM tracking from bidirectional DShot telemetry. The H743 has enough CPU headroom for per-motor notches across all IMUs.

> **Note on "Cycle 11":** I've treated this as your eleventh tuning iteration. I can't see your previous logs, so the values below are a baseline. Each cycle should be validated with pre- and post-filter FFT logs (see Section 6). If you share your frame size, motor pole count and Cycle 10 FFT results, I can tighten the numbers.

---

## 1. Prerequisites: RPM Telemetry Source

| Parameter | Value | Notes |
|---|---|---|
| `MOT_PWM_TYPE` | `6` (DShot600) or `5` (DShot300) | DShot600 is standard on H7 |
| `SERVO_BLH_BDMASK` | e.g. `15` (outputs 1–4) or `255` (1–8) | Bitmask of motor outputs with bidirectional DShot |
| `SERVO_BLH_POLES` | `14` (typical 5" motor) | Count the **magnets**, not the stator slots. A wrong value scales every notch incorrectly. |
| `SERVO_DSHOT_ESC` | `1` (BLHeli32/Bluejay/AM32) | Match your ESC firmware |
| `SERVO_DSHOT_RATE` | `0` or `1` | 1 = DShot updates at loop rate |

**Verify before continuing:** In the GCS, `ESC_RPM` (or `esc_telemetry` in the Status tab) should show plausible RPM on every motor when spun up with props off. The error rate in the ESC log should be below about 1%.

---

## 2. Primary Harmonic Notch (RPM-Tracked)

| Parameter | Value | Rationale |
|---|---|---|
| `INS_HNTCH_ENABLE` | `1` | Reboot after setting, so the remaining parameters appear |
| `INS_HNTCH_MODE` | `3` | ESC telemetry (bidirectional DShot) |
| `INS_HNTCH_REF` | `1` | No throttle scaling, because RPM is measured directly |
| `INS_HNTCH_FREQ` | ~`80`% of the lowest hover motor frequency (e.g. `80` Hz on 5", `40` Hz on 10"+) | Floor frequency, used at idle or when telemetry drops |
| `INS_HNTCH_BW` | `FREQ / 2` (e.g. `40`) | Wider costs phase; narrower risks missing the peak |
| `INS_HNTCH_ATT` | `40` dB | Default; 30–50 is reasonable |
| `INS_HNTCH_HMNCS` | `3` (1st + 2nd) or `7` (1st–3rd) | Add the 3rd harmonic if the FFT shows it, which is common on 2-blade props |
| `INS_HNTCH_OPTS` | `14` | 2 (per-motor notch) + 4 (update at loop rate) + 8 (all IMUs) |
| `INS_HNTCH_FM_RAT` | `1.0` | Prevents the notch from dropping below `FREQ` at low throttle (4.3+). Lower to 0.7–0.8 if the FFT shows the peak following RPM below the floor. |

**Optional:**
- Add **+1 (double notch)** to `INS_HNTCH_OPTS`, giving `15`, if a single notch leaves residual peak energy. On H7 this is affordable, but it adds phase lag, so check the tune afterward.
- Add **+32 (Treat low throttle as zero)** if your firmware version supports it and you see notch chatter on the ground.

**CPU check:** Per-motor tracking multiplies the filter count:

> filters = motors × harmonics × IMUs × (2 if double notch)

For example, a quad with 3 harmonics, 3 IMUs and double notch runs 72 biquads. Watch `PM.Load` and `PM.NLon` in the logs. Load should stay under about 80%, and NLon should be near zero.

---

## 3. Secondary Notch (Static Frame Resonance)

Use the second notch only if the post-filter FFT shows a fixed-frequency peak that does not move with throttle, such as arm or frame resonance:

| Parameter | Value |
|---|---|
| `INS_HNTC2_ENABLE` | `1` |
| `INS_HNTC2_MODE` | `0` (fixed) |
| `INS_HNTC2_FREQ` | Measured resonance center (e.g. `180`) |
| `INS_HNTC2_BW` | ~`FREQ / 4` (narrow, because the peak is fixed) |
| `INS_HNTC2_ATT` | `30–40` |
| `INS_HNTC2_HMNCS` | `1` |

If no static peak exists, leave `INS_HNTC2_ENABLE = 0`.

---

## 4. Supporting Gyro and Loop Settings

| Parameter | Value | Notes |
|---|---|---|
| `INS_GYRO_FILTER` | `40–80` Hz (5": 60–80; 10"+: 30–50) | Raise only after the notch is proven clean |
| `INS_ACCEL_FILTER` | `10–20` | Default is fine |
| `INS_GYRO_RATE` | `1` (2 kHz) or `2` (4 kHz) on H7 | Higher rates improve filter fidelity at modest CPU cost |
| `SCHED_LOOP_RATE` | `400` (large frames) / `800`–`1000` (5" racers) | Check `PM.Load` after changing |
| `ATC_RAT_*_FLTD` / `FLTT` | ~`INS_GYRO_FILTER / 2` | D-term and target filters |

---

## 5. Example Profiles

**5" quad, 14-pole, ~4S, hover ≈ 18k RPM (≈300 Hz mechanical)**
```
INS_HNTCH_ENABLE 1
INS_HNTCH_MODE   3
INS_HNTCH_REF    1
INS_HNTCH_FREQ   120
INS_HNTCH_BW     60
INS_HNTCH_ATT    40
INS_HNTCH_HMNCS  3
INS_HNTCH_OPTS   14
INS_HNTCH_FM_RAT 1.0
INS_GYRO_FILTER  80
SCHED_LOOP_RATE  800
```

**10–13" quad, 24–28 poles, hover ≈ 5k RPM (≈85 Hz)**
```
INS_HNTCH_FREQ   60
INS_HNTCH_BW     30
INS_HNTCH_HMNCS  7
INS_HNTCH_OPTS   15
INS_GYRO_FILTER  40
SCHED_LOOP_RATE  400
```
For the 10–13" profile, `SERVO_BLH_POLES` must match the actual magnet count.

---

## 6. Verification Procedure

1. **Log raw and filtered gyro data:**
   - `INS_LOG_BAT_MASK = 1` (first IMU)
   - `INS_LOG_BAT_OPT = 4` (pre- and post-filter, sensor-rate) on 4.3+. Use `2` for post-filter only.
   - `LOG_BITMASK`: include IMU_RAW for this flight only.
2. **Fly:** hover for 30–60 s, then do gentle throttle sweeps and a few rolls and pitches.
3. **Analyze** with the ArduPilot Filter Review tool (firmware.ardupilot.org/Tools/FilterReview) or the MAVExplorer FFT:
   - **Pre-filter:** confirm the peaks track the `ESC` RPM ÷ 60 lines and their harmonics.
   - **Post-filter:** motor peaks should be attenuated by at least 20–30 dB. No new peaks should appear.
   - **Estimated phase lag** at about 50 Hz should be minimal. Above roughly 30° will hurt the tune.
4. **Check telemetry quality:** look for gaps in `ESC.RPM`. Dropouts make the notch fall back to `FREQ`.
5. **Iterate (Cycle 12):**
   - If residual motor peaks remain, enable double notch or widen `BW`.
   - If you see excessive phase lag or a sluggish tune, narrow `BW`, drop unneeded harmonics, or remove the double notch.
   - Once the spectrum is clean, raise `INS_GYRO_FILTER` and re-run Autotune.

---

## 7. Common Failure Modes

- **Wrong `SERVO_BLH_POLES`:** the notch sits at the wrong frequency, typically off by a factor of 2 or 1.17.
- **Bidirectional DShot not enabled in ESC firmware:** RPM reads zero, and the notch stays pinned at `FREQ`.
- **`INS_HNTCH_REF` left at a throttle value in mode 3:** unnecessary scaling. Keep it at 1.
- **Notching the wrong IMU:** without the all-IMUs bit (8), an EKF lane switch can leave the active IMU unfiltered.
- **Notch drops too low at idle:** this causes oscillation on landing. Raise `FREQ` or set `FM_RAT` to 1.0.

Send the Filter Review screenshots or the `.bin` metrics from Cycle 10/11 and I can propose specific deltas.