# ArduPilot Dynamic Harmonic Notch Configuration
**Target:** STM32H743 flight controller, bidirectional DShot RPM telemetry, ArduCopter 4.4/4.5+

> **About "Cycle 15":** This doesn't match a standard ArduPilot parameter or term. If it refers to your own tuning iteration, the procedure below works as one iteration. If you meant the bidirectional-DShot bitmask value 15 (motors 1–4), that is covered in Step 1.

---

## 1. Bidirectional DShot / RPM Telemetry Setup

| Parameter | Value | Notes |
|---|---|---|
| `MOT_PWM_TYPE` | `6` | DShot600. Use `5` (DShot300) for BLHeli_S/Bluejay at lower loop rates. |
| `SERVO_DSHOT_ESC` | `1` or `3` | `1` = BLHeli32/AM32. `3` = the same with EDT. Use `2`/`4` for BLHeli_S/Bluejay. |
| `SERVO_BLH_BDMASK` | `15` | Bidirectional DShot on outputs 1–4 (bitmask 0b1111). Use `255` for an octo. |
| `SERVO_BLH_POLES` | `14` | Motor magnet count. Common 5" motors (12N14P) use 14. Verify against your motors. |
| `SERVO_BLH_AUTO` | `1` | Auto-maps the BLHeli outputs to the motors. |
| `SERVO_DSHOT_RATE` | `2` | 2× loop rate (800 Hz at 400 Hz loop). Improves RPM sample rate. |

**Check before continuing:** Spin the motors in the Motor Test and confirm that `ESC[n].RPM` is populated in the log. Also check that the ESC telemetry error rate is near 0% in the `EDT2`/`ESC` messages. The outputs must sit on DMA-capable timer groups. Most H743 boards (Matek H743, CubeOrange, Kakute H7) support this on outputs 1–4 or 1–8.

---

## 2. IMU Sampling and Loop Rate

The H743 has enough CPU headroom for fast sampling and per-motor notches.

| Parameter | Value | Notes |
|---|---|---|
| `SCHED_LOOP_RATE` | `400` | Use `800` for sub-5" craft if CPU load (`PM.Load`) stays below about 60%. |
| `INS_FAST_SAMPLE` | `1` (or `3` for dual IMU) | Bitmask for fast sampling per IMU. |
| `INS_GYRO_RATE` | `1` | 2 kHz backend rate. `2` (4 kHz) is feasible on H7. |
| `INS_GYRO_FILTER` | `80`–`100` | Gyro low-pass. Raise it once the notches are effective (about 120 on clean 5" builds). |
| `INS_ACCEL_FILTER` | `20` | |

---

## 3. Primary Harmonic Notch (RPM-tracking)

| Parameter | Value | Notes |
|---|---|---|
| `INS_HNTCH_ENABLE` | `1` | Reboot after setting to expose the other parameters. |
| `INS_HNTCH_MODE` | `3` | ESC telemetry (RPM) tracking. |
| `INS_HNTCH_REF` | `1` | Scaling reference. Must be 1 for RPM mode. |
| `INS_HNTCH_FREQ` | see formula | Minimum tracking frequency. |
| `INS_HNTCH_BW` | `FREQ / 2` | Notch bandwidth in Hz. |
| `INS_HNTCH_ATT` | `40` | Attenuation in dB. |
| `INS_HNTCH_HMNCS` | `3` | 1st and 2nd harmonics. Use `7` to add the 3rd for 2-blade props on rigid frames. Use `1` for 3-blade props if the 2nd harmonic is weak. |
| `INS_HNTCH_OPTS` | `14` | Bit 1 (2) = one notch per motor. Bit 2 (4) = update at loop rate. Bit 3 (8) = apply on all IMUs. |
| `INS_HNTCH_FM_RAT` | `1.0` | Throttle-floor ratio. Lower to `0.7` if the notch sits too high at low throttle. |

### Setting `INS_HNTCH_FREQ`

1. Take a hover log and read the average hover RPM from `ESC[n].RPM`.
2. Calculate the hover fundamental frequency:
 **f_hover = RPM_hover / 60**
3. Set the minimum frequency:
 **INS_HNTCH_FREQ ≈ 0.6–0.8 × f_hover**
 Keep it at or above about 40 Hz so the notch does not reach down into the control bandwidth.

**Worked example (5", 14-pole motors, hover at 9,000 RPM):**
- f_hover = 9000 / 60 = **150 Hz**
- `INS_HNTCH_FREQ` = **100**
- `INS_HNTCH_BW` = **50**

Per-motor notches (bit 1) × 2 harmonics × 4 motors × number of IMUs gives roughly 16–24 active biquads. That is within the H743's capacity at a 400 Hz loop, but verify CPU load.

---

## 4. Secondary Notch (Static Frame Resonance), Optional

Use this if the FFT shows a fixed peak that does not move with throttle, such as frame or arm resonance.

| Parameter | Value |
|---|---|
| `INS_HNTC2_ENABLE` | `1` |
| `INS_HNTC2_MODE` | `0` (fixed) |
| `INS_HNTC2_FREQ` | measured resonance peak (Hz) |
| `INS_HNTC2_BW` | `FREQ / 4` |
| `INS_HNTC2_ATT` | `30`–`40` |
| `INS_HNTC2_HMNCS` | `1` |

---

## 5. Verification Logging

| Parameter | Value | Purpose |
|---|---|---|
| `INS_LOG_BAT_MASK` | `1` | Batch-sample the first IMU. |
| `INS_LOG_BAT_OPT` | `4` | Log both pre- and post-filter data. |
| `LOG_BITMASK` | include IMU_RAW/ESC bits | |

**Procedure:**
1. Hover for about 60 seconds, then do gentle throttle sweeps.
2. Load the `.bin` file into the **ArduPilot Filter Review Tool** (firmware.ardupilot.org/Tools/FilterTool).
3. Compare the pre- and post-filter FFTs. The motor peaks and their harmonics should drop by at least 20 dB.
4. Confirm that the notch center tracks the peaks across the throttle range.

---

## 6. Iteration Checklist

- [ ] RPM telemetry is clean, with ESC error rate below 1%.
- [ ] Notch frequency tracks the 1st and 2nd harmonics in the FFT.
- [ ] No residual peak remains between 80 and 300 Hz post-filter.
- [ ] `PM.Load` is below about 60% and there are no long-loop warnings.
- [ ] After the filters are confirmed, raise `INS_GYRO_FILTER` and run **AutoTune**. Retune because the change in filter phase lag shifts the optimal PIDs.
- [ ] Check motor temperatures after flight. Warm motors mean residual noise is reaching the D-term.

If you share your frame size, prop and blade count, motor KV and pole count, hover RPM (or a log FFT), I can calculate exact values for `INS_HNTCH_FREQ`, `BW`, and `HMNCS`.