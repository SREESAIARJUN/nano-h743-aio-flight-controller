# ArduPilot Dynamic Harmonic Notch Configuration: STM32H743 with Bidirectional DShot RPM

**Assumptions about "Cycle 10":** I've treated it as your 10th tuning iteration. This means the RPM telemetry already works and the goal is a refined, final filter set. If "Cycle 10" means something else, such as a specific airframe or firmware tag, tell me and I'll adjust.

The example numbers below assume a **5–7" multirotor on ArduCopter 4.4/4.5+**. Replace them with values from your own logs.

---

## 1. Prerequisites: RPM Telemetry Path

```
MOT_PWM_TYPE      = 6        # DShot600 (5 = DShot300 if ESC/wiring is marginal)
SERVO_BLH_AUTO    = 1        # BLHeli passthrough/telemetry auto-enable
SERVO_BLH_BDMASK  = 15       # Bidirectional DShot on outputs 1–4 (bitmask; 255 for octo)
SERVO_BLH_POLES   = 14       # Motor magnet poles (count your bell magnets!)
SERVO_DSHOT_ESC   = 1        # BLHeli32/AM32 (3 = BLHeli_S w/ Bluejay)
SERVO_DSHOT_RATE  = 2        # DShot output at 2× loop rate (H7 handles this fine)
```

Verify the setup before trusting the notch. In the `ESC` log messages, the RPM values must be clean and non-zero on all motors, with an error rate under 1%. A wrong `SERVO_BLH_POLES` value scales every notch frequency incorrectly.

---

## 2. Gyro Sampling and Loop Rate (H743 Headroom)

```
SCHED_LOOP_RATE   = 400      # 800 is viable on H743 for small, high-RPM quads
INS_GYRO_RATE     = 1        # 0=1kHz, 1=2kHz, 2=4kHz, 3=8kHz backend rate
INS_FAST_SAMPLE   = 1        # Fast sampling on IMU1 (bitmask)
```

At 4–8 kHz the H743 can comfortably run per-motor notches. Keep the notch centre frequencies below roughly 0.4 × the filter update rate.

---

## 3. Primary Harmonic Notch: ESC-Telemetry Tracked, Per-Motor

```
INS_HNTCH_ENABLE  = 1        # Reboot after setting to expose the rest
INS_HNTCH_MODE    = 3        # ESC telemetry (bidirectional DShot RPM)
INS_HNTCH_REF     = 1.0      # Scaling; must be 1.0 for ESC mode
INS_HNTCH_FREQ    = 80       # Min tracking freq (Hz) – see sizing below
INS_HNTCH_BW      = 40       # ≈ FREQ/2
INS_HNTCH_ATT     = 40       # dB attenuation at centre
INS_HNTCH_HMNCS   = 7        # Harmonics 1,2,3 (bitmask: 1=1st, 2=2nd, 4=3rd, 8=4th…)
INS_HNTCH_FM_RAT  = 1.0      # Min freq ratio at low throttle (lower → 0.7 if it bottoms out)
INS_HNTCH_OPTS    = 6        # bit1 Multi-Source (one notch per motor) + bit2 Update at loop rate
```

### `INS_HNTCH_OPTS` bit guidance
| Bit | Value | Function | Recommendation |
|---|---|---|---|
| 0 | 1 | Double notch | Not needed with per-motor tracking |
| 1 | 2 | Multi-source (per-motor notch) | **Yes**: the biggest win with RPM telemetry |
| 2 | 4 | Update at loop rate | **Yes** on H7 |
| 3 | 8 | Enable on all IMUs | Yes if EKF lane-switching to IMU2/3 matters (adds CPU) |
| 4 | 16 | Triple notch | No (redundant with per-motor) |
| 5 | 32 | Treat low frequency as minimum | Yes if the notch collapses at idle/landed |

A robust set is **`INS_HNTCH_OPTS = 38`** (2 + 4 + 32). Use **46** if you also want the notch on all IMUs.

### Sizing `FREQ` / `BW`
- Find the hover motor fundamental: `f = RPM_hover / 60`.
  - Example: hover at 12,000 RPM gives **200 Hz**.
- Set `INS_HNTCH_FREQ` to about 40–50% of that, **floored at roughly 50 Hz** to stay clear of the control bandwidth. In this example, use 80 Hz.
- Set `INS_HNTCH_BW` to about `FREQ/2`. In this example, use 40 Hz.
- Per-motor tracking is accurate, so a narrow BW is safe. Avoid going wide, because that adds phase lag.

CPU load scales as **motors × harmonics × 3 axes**. For a quad with 3 harmonics, that is 36 biquads, which is trivial on an H743.

---

## 4. Secondary Notch: Static or Frame-Resonance Notch

Use this for frame or arm resonance that does **not** track RPM. You can spot it in the FFT as a fixed peak.

```
INS_HNTC2_ENABLE  = 1
INS_HNTC2_MODE    = 0        # Fixed frequency
INS_HNTC2_FREQ    = 145      # Set to the observed static peak (example)
INS_HNTC2_BW      = 30
INS_HNTC2_ATT     = 30
INS_HNTC2_HMNCS   = 1
INS_HNTC2_OPTS    = 0
```

If there is no static peak, leave `INS_HNTC2_ENABLE = 0`, since every filter adds phase lag. An alternative for prop-blade-pass frequencies is `MODE=3` with `HMNCS` targeting the 2nd/4th harmonics.

---

## 5. Low-Pass Filters (Relax After Notching)

With effective notching you can open up the LPFs to reduce latency:

```
INS_GYRO_FILTER   = 80       # Was ~20–40 default; 80–120 for 5" quads once notched
INS_ACCEL_FILTER  = 10
ATC_RAT_RLL_FLTD  = 40       # D-term LPF; ~INS_GYRO_FILTER/2
ATC_RAT_PIT_FLTD  = 40
ATC_RAT_YAW_FLTE  = 2
```

Raise `INS_GYRO_FILTER` in steps of about 20 Hz. Check motor temperatures and the `RATE`/`PIDR` noise after each step.

---

## 6. Logging for Verification (Batch Sampler)

```
INS_LOG_BAT_MASK  = 1        # IMU1
INS_LOG_BAT_OPT   = 4        # Pre- AND post-filter capture (bit2)
INS_LOG_BAT_CNT   = 1024
INS_LOG_BAT_LGIN  = 20
INS_LOG_BAT_LGCT  = 32
LOG_BITMASK       = (default) + bit for IMU_RAW as needed
```

Disable batch logging after tuning, because it is heavy on SD bandwidth.

---

## 7. Verification Procedure

1. **Hover flight, 60 s, plus a few throttle sweeps.**
2. Open the log in the **ArduPilot Filter Review Tool** (firmware.ardupilot.org/Tools/FilterTool or the WebTools FFT viewer).
3. Compare the pre- and post-filter spectra. Motor peaks and their harmonics should drop by **at least 20 dB** across the throttle range.
4. Check the `RCOU`/`ESC` alignment. The notch centre frequencies (`FTN1`/`FTN2` logs) must overlay the moving peaks.
5. Iterate:
   - If a residual peak sits at the 2× or 4× prop-blade harmonic, add the `HMNCS` bit (8 = 4th).
   - If the notch disappears near idle, set OPTS bit 5 and/or lower `FM_RAT` to 0.7–0.8.
   - If you see excess phase lag (sluggish or oscillatory D-term), reduce `HMNCS` count or narrow `BW`.
6. Re-run **QuickTune or AutoTune** after the filter changes. The PID gains are filter-dependent.

---

## 8. Final "Cycle 10" Parameter Summary (Example 5" Quad)

```
MOT_PWM_TYPE=6  SERVO_BLH_BDMASK=15  SERVO_BLH_POLES=14  SERVO_DSHOT_ESC=1
SCHED_LOOP_RATE=400  INS_GYRO_RATE=1  INS_FAST_SAMPLE=1
INS_HNTCH_ENABLE=1  INS_HNTCH_MODE=3  INS_HNTCH_REF=1  INS_HNTCH_FREQ=80
INS_HNTCH_BW=40  INS_HNTCH_ATT=40  INS_HNTCH_HMNCS=7  INS_HNTCH_FM_RAT=1.0
INS_HNTCH_OPTS=38
INS_HNTC2_ENABLE=0   (enable only if a static resonance is seen)
INS_GYRO_FILTER=80  ATC_RAT_RLL_FLTD=40  ATC_RAT_PIT_FLTD=40
INS_LOG_BAT_MASK=1  INS_LOG_BAT_OPT=4
```

Larger frames (10"+) spin slower, so lower `INS_HNTCH_FREQ` to about 40–50 Hz, `BW` to about 20–25 Hz, and keep `INS_GYRO_FILTER` around 40–60 Hz.

If you share your hover RPM, prop size, motor KV and a pre-filter FFT, I can tighten these to exact values.