# ArduPilot Harmonic Notch Configuration: STM32H743 with Bidirectional DShot RPM

I don't know what "Cycle 4" refers to in your context. I've treated it as the **fourth iteration of a tune-log-refine loop**, where RPM telemetry already works and you are tightening filters based on log data. If it means something else, such as a specific build or course module, tell me and I'll adjust.

The example values assume a **5–7" multirotor on ArduPilot 4.4–4.5+**. Scaling guidance for other frame sizes follows the example.

---

## 1. RPM Telemetry Foundation (Verify Before Tuning)

| Parameter | Value | Notes |
|---|---|---|
| `MOT_PWM_TYPE` | `6` (DShot600) or `5` (DShot300) | Use DShot300 if telemetry error rates are high. |
| `SERVO_BLH_BDMASK` | e.g. `15` (outputs 1–4) | Enables bidirectional DShot on those outputs. |
| `SERVO_BLH_POLES` | `14` (typical 5" motor) | Count the magnets. A wrong value scales every notch incorrectly. |
| `SERVO_DSHOT_ESC` | `1` (BLHeli32/AM32) or `3` (Bluejay) | Must match your ESC firmware. |
| `SERVO_DSHOT_RATE` | `0`–`2` | A higher rate gives fresher RPM at higher CPU cost, which the H743 handles. |

**Check:** In logs, `ESC[n].RPM` should be smooth with near-zero dropouts. Read `ESC.Err` or the OSD/MAVLink telemetry error rate and keep it below about 1–2%. If RPM drops out, the notch falls back to its minimum frequency and filtering degrades.

---

## 2. Loop Rate and Gyro Rate

The H743 has enough headroom to run fast:

| Parameter | Value |
|---|---|
| `SCHED_LOOP_RATE` | `800` (5") or `400` (large/heavy frames) |
| `INS_GYRO_RATE` | `1` (2 kHz) or `2` (4 kHz) if the IMU supports it |
| `INS_FAST_SAMPLE` | `1` (bit for the primary IMU) |

---

## 3. Harmonic Notch #1: RPM-Driven, Per-Motor

| Parameter | Value | Rationale |
|---|---|---|
| `INS_HNTCH_ENABLE` | `1` | Reboot afterwards to expose the remaining parameters. |
| `INS_HNTCH_MODE` | `3` | ESC telemetry (RPM) tracking. |
| `INS_HNTCH_REF` | `1` | Scaling reference for ESC mode. |
| `INS_HNTCH_FREQ` | `80` Hz (5") | Lowest tracked frequency. Set it just below the hover fundamental seen in logs, roughly 0.7–0.8 × hover frequency. |
| `INS_HNTCH_BW` | `40` Hz | About ½ × FREQ. Per-motor tracking allows narrow notches. |
| `INS_HNTCH_ATT` | `40` dB | Default. Raise toward 50 only if residual peaks remain. |
| `INS_HNTCH_HMNCS` | `3` (1st + 2nd) or `7` (1st–3rd) | Enable the 3rd harmonic if the pre-filter FFT shows it. |
| `INS_HNTCH_OPTS` | `14` = 2 + 4 + 8 | Bit 1 creates a separate notch per motor, bit 2 updates at loop rate, and bit 3 applies to all IMUs. |
| `INS_HNTCH_FM_RAT` (4.5+) | `1.0` | Lower to about 0.7 only if logs show motors spending meaningful time below FREQ, such as during descents or prop-wash. |

**CPU cost:** 4 motors × up to 3 harmonics × 1 notch per harmonic, across all IMUs. The H743 handles this comfortably. Confirm by checking that `PM.Load` stays below about 60%.

---

## 4. Harmonic Notch #2: Static or Throttle Backup

Use the second notch for whichever of these applies:

- **A fixed frame or arm resonance** that does not track RPM:
  - `INS_HNTC2_ENABLE=1`, `MODE=0` (fixed)
  - `FREQ` = the peak frequency
  - `BW` ≈ 0.25–0.5 × FREQ, `ATT=40`, `HMNCS=1`
- **A throttle-based fallback** in case telemetry is unreliable:
  - `MODE=1`, `REF` = `MOT_THST_HOVER` value, `FREQ` ≈ hover fundamental
  - `BW` = FREQ/2, `OPTS=0`

If neither applies, leave it disabled to keep phase lag low.

---

## 5. Low-Pass Filters (Relaxed Because the Notches Do the Heavy Work)

| Parameter | 5" | 7" | 10"+ |
|---|---|---|---|
| `INS_GYRO_FILTER` | 80–100 Hz | 60–80 | 40–50 |
| `INS_ACCEL_FILTER` | 20 | 20 | 10–20 |
| `ATC_RAT_*_FLTD` | 40 | 30 | 20 |
| `ATC_RAT_*_FLTT` / `FLTE` | 0 / 0 (yaw `FLTE` 2) | same | same |

---

## 6. Scaling Guidance by Frame Size

| Frame | Typical hover fundamental | `INS_HNTCH_FREQ` | `INS_HNTCH_BW` |
|---|---|---|---|
| 3" | 200–250 Hz | 150 | 75 |
| 5" | 100–130 Hz | 80 | 40 |
| 7" | 70–90 Hz | 55 | 28 |
| 10" | 50–70 Hz | 40 | 20 |
| 15"+ | 25–40 Hz | 20 | 10 |

---

## 7. Refinement Procedure (the "Cycle 4" Loop)

1. **Log setup:**
   - `INS_LOG_BAT_MASK=1` (first IMU)
   - `INS_LOG_BAT_OPT=4` (captures both pre- and post-filter data)
   - `LOG_BITMASK`: include the IMU bits
2. **Fly:** 30–60 s of hover plus some aggressive throttle and roll inputs.
3. **Analyze:** Use the ArduPilot WebTools *Filter Review* tool or the MAVExplorer FFT. Compare the pre- and post-filter spectra.
4. **Adjust:**
   - Residual peak at the fundamental: check `SERVO_BLH_POLES`, then raise `ATT` or slightly widen `BW`.
   - Residual peak at 2× or 3×: add that harmonic to `HMNCS`.
   - Clean spectrum with sluggish feel: narrow `BW`, raise `INS_GYRO_FILTER`, and drop unneeded harmonics.
   - Broadband noise not tied to RPM: look for a mechanical cause such as props, bearings, or FC mounting, rather than adding filtering.
5. **Re-tune PIDs:** Lower filter lag allows higher D gain. Re-run AutoTune or a manual tune after the filters are final.
6. **Repeat** until the post-filter gyro noise floor is flat, ideally below about −40 dB at the motor peaks.

---

## 8. Safety Notes

- Bench-test with props off first. Confirm that `ESC_RPM` reads correctly for each motor and that the motor order matches.
- Increase `INS_GYRO_FILTER` in small steps of 10–20 Hz. If filtering is too thin, motors can run hot or oscillate.
- After any filter change, check motor temperature after a short flight.

If you share your frame size, prop and motor KV, ArduPilot version, and a pre-filter FFT screenshot or log, I can narrow these to specific numbers.