# ArduPilot Dynamic Harmonic Notch with DShot RPM Feedback on STM32H743: Cycle 3 Refinement

**Assumption:** I'm treating "Cycle 3" as the third tuning iteration. Cycle 1 logged raw gyro noise. Cycle 2 enabled a basic RPM-tracked notch. Cycle 3 refines the notch and then uses the cleaner signal to relax low-pass filtering and raise the D-term. If you meant something else, such as a firmware or hardware revision, let me know.

---

## 1. Prerequisites: Bidirectional DShot RPM Source

| Parameter | Value | Notes |
|---|---|---|
| `MOT_PWM_TYPE` | 6 (DShot600) or 5 (DShot300) | Use DShot300 for large, slow props. |
| `SERVO_BLH_BDMASK` | 15 (outputs 1–4) | Set the bitmask to match your motor outputs. Use 255 for an octa. |
| `SERVO_BLH_POLES` | 14 | Use the motor magnet pole count, typically 12 or 14. It must be correct, or the notch tracks the wrong frequency. |
| `SERVO_BLH_AUTO` | 1 | Maps motors automatically. |
| `SERVO_DSHOT_RATE` | 1–2 | Sets the DShot rate as a multiple of the loop rate. |

**Verification:** Check `ESC[n].RPM` in the logs.
- It should be smooth and non-zero at hover.
- Error rate should be under about 1%. Check `ESCX.Err` or `ESC.Err`, depending on firmware version.

## 2. Gyro Sample and Loop Rate (H743 Headroom)

| Parameter | Value |
|---|---|
| `INS_GYRO_RATE` | 1 (2 kHz) for most builds. Use 2 (4 kHz) on a 5" quad if the IMU supports it. |
| `SCHED_LOOP_RATE` | 400 for 7"+ props. Use 800 for 5" if CPU load (`PM.Load`) stays under about 60%. |
| `INS_FAST_SAMPLE` | Enabled on all IMUs |

## 3. Primary Harmonic Notch (RPM-Tracked)

| Parameter | Cycle 3 Value | Rationale |
|---|---|---|
| `INS_HNTCH_ENABLE` | 1 | Turns the notch on. |
| `INS_HNTCH_MODE` | 3 | Tracks frequency from ESC telemetry. |
| `INS_HNTCH_REF` | 1 | Uses scaling directly. |
| `INS_HNTCH_FREQ` | About 0.8 × hover motor freq (Hz) | Acts as the floor frequency. A 5" quad is typically 80–120 Hz. A 10" quad is about 40–60 Hz. |
| `INS_HNTCH_BW` | About ½ × `INS_HNTCH_FREQ` | Per-motor notches can be narrow. |
| `INS_HNTCH_ATT` | 40 dB | Sets notch depth. |
| `INS_HNTCH_HMNCS` | 3 (1st + 2nd), or 7 to include the 3rd | Add the 3rd harmonic only if the FFT shows it. |
| `INS_HNTCH_OPTS` | 14 = 2 + 4 + 8 | 2 = per-motor notches (multi-source), 4 = update at loop rate, 8 = apply on all IMUs. |
| `INS_HNTCH_FM_RAT` | 0.7–1.0 | Lets the notch follow RPM below the floor during descents. |

**Hover frequency formula:**

`f_hover = RPM_hover / 60`

Because `SERVO_BLH_POLES` is set, ArduPilot already reports `ESC.RPM` as mechanical RPM, so no extra pole-count correction is needed here.

**CPU load:** Per-motor notches across 3 harmonics on a quad give 12 notches per IMU. The H743 handles this, but confirm `PM.Load` stays acceptable.

## 4. Secondary Notch (Static Frame/Structural Resonance, If Present)

Use this only if the Cycle 2 FFT shows a fixed peak that does not move with RPM, such as an arm or gimbal resonance.

| Parameter | Value |
|---|---|
| `INS_HNTC2_ENABLE` | 1 |
| `INS_HNTC2_MODE` | 0 (fixed) |
| `INS_HNTC2_FREQ` | The measured peak, in Hz |
| `INS_HNTC2_BW` | About ¼ to ½ of the frequency |
| `INS_HNTC2_ATT` | 30–40 dB |
| `INS_HNTC2_HMNCS` | 1 |

**Alternative:** Set `INS_HNTC2_MODE = 1` (throttle-based) as a redundancy fallback in case ESC telemetry drops.

## 5. Low-Pass Filters (Relax After the Notch Is Verified)

| Parameter | Cycle 2 Value | Cycle 3 Target |
|---|---|---|
| `INS_GYRO_FILTER` | 40–60 Hz | 80–120 Hz (5"), 50–70 Hz (10"+) |
| `ATC_RAT_RLL/PIT_FLTD` | 20–30 Hz | 40–50 Hz (5"), 20–30 Hz (large) |
| `ATC_RAT_RLL/PIT_FLTT` | Same as FLTD | Same as FLTD |
| `ATC_RAT_YAW_FLTE` | 2 Hz | 2 Hz (keep) |

## 6. Logging for Verification

| Parameter | Value |
|---|---|
| `INS_LOG_BAT_MASK` | 1 (first IMU) |
| `INS_LOG_BAT_OPT` | 4, which logs both pre-filter and post-filter data. Use 2 for post-filter only. |
| `LOG_BITMASK` | Include IMU_FAST |

## 7. Cycle 3 Acceptance Criteria

Run the log through the ArduPilot **Filter Review Tool** (`firmware.ardupilot.org/Tools/FilterReview`).

1. Post-filter motor peaks (1st and 2nd harmonics) should be attenuated by **at least 20 dB**. The residual noise floor should be under about -40 dB.
2. There should be no new peaks introduced by the relaxed `INS_GYRO_FILTER`.
3. The notch center should track `ESC.RPM` through throttle punches and descents with no lag artifacts.
4. Motor outputs should show reduced oscillation, and motor temperatures should stay cool after a 2-minute hover.
5. CPU load (`PM.Load`) should stay under 70% and `PM.NLon` (long loops) should stay low.

## 8. Next Step (Cycle 4)

Once Cycle 3 passes, run **QuickTune** or **AutoTune**. The cleaner gyro signal allows higher D and P gains without hot motors.