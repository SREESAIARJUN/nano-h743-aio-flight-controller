# ArduPilot Harmonic Notch Configuration: STM32H743 with Bidirectional DShot RPM

I don't recognize "Cycle 16" as an ArduPilot term. I've treated it as your own iteration or tuning-cycle label. If it refers to a specific firmware build, frame, or test spec, tell me and I'll adjust.

This configuration targets ArduCopter 4.4 to 4.6 on a quad. Adjust the motor masks for other frame types.

---

## 1. Enable Bidirectional DShot (RPM Source)

| Parameter | Value | Notes |
|---|---|---|
| `MOT_PWM_TYPE` | `6` (DShot600) or `5` (DShot300) | DShot300 is more robust with long ESC leads. |
| `SERVO_BLH_BDMASK` | `15` | Bitmask for outputs 1 to 4. Use `255` for 8 motors. |
| `SERVO_BLH_POLES` | `14` | Count the motor's magnets. Most 22xx/23xx motors have 14 and many small motors have 12. A wrong value gives a wrong notch frequency. |
| `SERVO_BLH_AUTO` | `1` | Maps motor outputs automatically. |
| `SERVO_DSHOT_ESC` | `1` (BLHeli32/AM32) or `3` (BLHeli_S Bluejay) | Match your ESC firmware. |
| `SERVO_DSHOT_RATE` | `0` to `2` | `1` or `2` is fine on the H7 at a 400 to 800 Hz loop. |

**Prerequisites**
- The ESC firmware must support bidirectional DShot. BLHeli32, AM32, and Bluejay all do.
- Use a bdshot-capable firmware target. Most H743 boards support it natively. Some need the `-bdshot` build.

**Check before going further:** after a reboot, spin the motors and confirm `ESC[n].RPM` appears in the logs or the MAVLink inspector. If RPM reads 0, stop here.

---

## 2. Primary Harmonic Notch (RPM-Tracked)

Set `INS_HNTCH_ENABLE = 1`, then reboot so the remaining parameters appear.

| Parameter | Value | Rationale |
|---|---|---|
| `INS_HNTCH_MODE` | `3` | ESC telemetry (RPM) tracking. |
| `INS_HNTCH_REF` | `1` | Scaling is unused in RPM mode. Keep it at 1. |
| `INS_HNTCH_FREQ` | `~80` Hz (5–7" quad); `~100–120` (3" or smaller) | Acts as the floor frequency. Set it about 20% below the lowest RPM frequency seen in flight. |
| `INS_HNTCH_BW` | `FREQ / 2` (e.g., `40`) | Standard ratio. |
| `INS_HNTCH_ATT` | `40` dB | Default and effective. |
| `INS_HNTCH_HMNCS` | `3` (1st and 2nd harmonics) or `7` (1st to 3rd) | Start with 3. Add the 3rd harmonic if the FFT shows it (common with triblades). |
| `INS_HNTCH_OPTS` | `6` | Bit 1 runs one notch per motor. Bit 2 updates at loop rate. Add bit 3 (`+8`, giving `14`) to apply to all IMUs if your board has several. |
| `INS_HNTCH_FM_RAT` | `1.0` | Throttle-off floor ratio. Lower it to about `0.8` if the notch sits too high at idle. |

**CPU load:** with 4 motors × 2 harmonics × 3 axes × the number of IMUs, the H743 has plenty of headroom. Even with 3 harmonics on all IMUs, check `PM.Load` in the logs and keep it below 60%.

---

## 3. Secondary Notch (`INS_HNTC2_*`), Optional

Use this for a fixed frame resonance or a residual peak that does not track RPM:

| Parameter | Value |
|---|---|
| `INS_HNTC2_ENABLE` | `1` |
| `INS_HNTC2_MODE` | `0` (fixed) |
| `INS_HNTC2_FREQ` | Peak frequency from the FFT |
| `INS_HNTC2_BW` | `FREQ / 4` to `FREQ / 2` |
| `INS_HNTC2_ATT` | `40` |
| `INS_HNTC2_HMNCS` | `1` |

Leave it disabled unless the post-filter FFT shows a clear static peak.

---

## 4. Supporting Filters and Loop Rate

| Parameter | Value | Notes |
|---|---|---|
| `SCHED_LOOP_RATE` | `400` (5" and larger), `800` (3" and smaller) | The H743 handles 800 Hz comfortably. |
| `INS_GYRO_FILTER` | `40` → `80`–`100` | Raise it only after the notch is verified. |
| `INS_ACCEL_FILTER` | `20` | Default. |
| `INS_FAST_SAMPLE` | `1` (or a mask of all IMUs) | |
| `ATC_RAT_*_FLTD` / `FLTT` | Per Autotune or the tuning guide | |

---

## 5. Verification Procedure

1. **Set up logging:**
   - `INS_LOG_BAT_MASK = 1`
   - `INS_LOG_BAT_OPT = 4` (logs pre- and post-filter data in one flight on 4.4 and later)
   - `LOG_BITMASK`: include IMU_RAW if needed.
2. **Fly:** hover for about 30 s, then make some throttle sweeps and gentle maneuvers.
3. **Analyze** with the ArduPilot Filter Review Tool (firmware.ardupilot.org → Tools → FilterReview), or with Mission Planner's FFT.
4. **Pass criteria:**
   - Post-filter motor peaks are attenuated by at least 20–30 dB.
   - The notch frequency tracks the `ESC.RPM`-derived frequency (RPM / 60 × harmonic).
   - No new peak appears below `INS_HNTCH_FREQ`.
5. **Raise `INS_GYRO_FILTER`** once the spectrum is clean, then run Autotune.

---

## 6. Quick Parameter Block (5" Quad, BLHeli32, Single IMU)

```
MOT_PWM_TYPE        6
SERVO_BLH_BDMASK    15
SERVO_BLH_POLES     14
SERVO_BLH_AUTO      1
INS_HNTCH_ENABLE    1
INS_HNTCH_MODE      3
INS_HNTCH_REF       1
INS_HNTCH_FREQ      80
INS_HNTCH_BW        40
INS_HNTCH_ATT       40
INS_HNTCH_HMNCS     3
INS_HNTCH_OPTS      6
INS_HNTCH_FM_RAT    1.0
INS_GYRO_FILTER     80
INS_LOG_BAT_MASK    1
INS_LOG_BAT_OPT     4
SCHED_LOOP_RATE     400
```

If you share your frame size, prop and blade count, motor KV and pole count, and ESC firmware, I can narrow the frequency floor and harmonic choice.