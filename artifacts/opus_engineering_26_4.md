# ArduPilot Dynamic Harmonic Notch Configuration
**Target:** STM32H743 flight controller, bidirectional DShot RPM telemetry (ArduCopter 4.4/4.5+)

> **Assumption:** I don't recognize "Cycle 26" as an ArduPilot release or standard term. I've treated it as your internal iteration or test-cycle label. If it refers to a specific firmware branch or airframe, tell me and I'll adjust the values.

---

## 1. Prerequisites: Bidirectional DShot RPM Source

| Parameter | Value | Notes |
|---|---|---|
| `MOT_PWM_TYPE` | `6` (DShot600) or `5` (DShot300) | DShot300 is more tolerant of long or noisy signal leads |
| `SERVO_BLH_BDMASK` | `15` (outputs 1–4), `63` for hex, `255` for octo | Enables bidirectional telemetry per output |
| `SERVO_BLH_POLES` | `14` | Motor **pole** count (magnets), not stator slots. Common values: 12N14P → 14, 9N12P → 12 |
| `SERVO_BLH_AUTO` | `1` | Auto-maps BLHeli outputs to motor outputs |
| `SERVO_DSHOT_ESC` | `1` (BLHeli32/AM32), `2` (BLHeli_S/Bluejay) | Bluejay or BLHeli_S with bidir firmware is required for RPM on BLHeli_S |
| `SERVO_DSHOT_RATE` | `1`–`2` | Multiplier of loop rate |

**Verify before arming:**
- `ESC1..n RPM` appears in the MAVLink inspector or `ESC` log messages.
- Spinning the motors with motor test gives sane RPM values. Hover RPM ÷ 60 × harmonic should match the FFT peaks.
- Check that the DMA allocation on the H743 target has no conflicts. If RPM stays at 0, look for `"DShot: bdshot unavailable"`-style messages.

---

## 2. Loop Rate and Gyro Sampling (H743 Headroom)

| Parameter | Value | Notes |
|---|---|---|
| `SCHED_LOOP_RATE` | `400` (≥7"), `800`–`1000` (3–5") | The H743 handles 1 kHz comfortably |
| `INS_FAST_SAMPLE` | `1` (or `3` for dual IMU) | Fast sampling on primary IMU(s) |
| `INS_GYRO_RATE` | `1` (2 kHz) or `2` (4 kHz) | Raise only if you also raise the loop rate. Check `PM` load stays below ~70% |
| `FSTRATE_ENABLE` | `0` initially | Optional fast-rate thread (4.6+). Validate separately |

---

## 3. Primary Dynamic Harmonic Notch (RPM / ESC Telemetry)

| Parameter | Value | Rationale |
|---|---|---|
| `INS_HNTCH_ENABLE` | `1` | Reboot afterwards to expose the remaining parameters |
| `INS_HNTCH_MODE` | `3` | ESC telemetry (bidirectional DShot RPM) |
| `INS_HNTCH_REF` | `1` | Scale factor of 1.0 for RPM tracking |
| `INS_HNTCH_FREQ` | ~80% of the lowest hover motor fundamental (see table below) | Acts as the lower clamp: the notch never tracks below this frequency |
| `INS_HNTCH_BW` | `INS_HNTCH_FREQ / 2` | Standard starting point |
| `INS_HNTCH_ATT` | `40` dB | Increase to 50 only if FFT shows residual peaks |
| `INS_HNTCH_HMNCS` | `3` (1st + 2nd) | Use `7` (1st–3rd) for props with strong 3rd harmonic (tri-blade) |
| `INS_HNTCH_OPTS` | `14` = 2 + 4 + 8 | See bit breakdown below |
| `INS_HNTCH_FM_RAT` | `1.0` | Throttle-mode floor ratio. Largely irrelevant in mode 3, but keep the default |

**`INS_HNTCH_OPTS` bits:**
- **2 – Multi-source:** one notch set per motor, tracking each ESC's RPM individually. This is the key benefit of RPM feedback.
- **4 – Update at loop rate:** notch centre is updated every loop rather than at 200 Hz.
- **8 – Enable on all IMUs:** filters the secondary IMU, so EKF lane switches stay clean.
- **Optional:** add `1` (double notch) on noisy, stiff frames. On 4.5+, also consider the "use minimum frequency on RPM source failure" bit. Check the bit list in your firmware's parameter docs, since it varies by version.

**CPU budget:** 4 motors × 2 harmonics × 2 IMUs × 3 axes = 48 biquads. This is trivial on an H743. An octo with 3 harmonics and double notch is still fine; confirm in the `PM` log.

### Starting `INS_HNTCH_FREQ` by airframe

| Airframe | Typical hover fundamental | `INS_HNTCH_FREQ` | `INS_HNTCH_BW` |
|---|---|---|---|
| 3" | 180–250 Hz | 140 | 70 |
| 5" | 120–180 Hz | 100 | 50 |
| 7" | 80–110 Hz | 65 | 32 |
| 10" | 50–70 Hz | 40 | 20 |
| 15"+ | 30–45 Hz | 25 | 12 |

Compute your own hover fundamental with:

**f = hover RPM / 60**

---

## 4. Secondary Notch (`INS_HNTC2_*`)

Use the second notch for one of two jobs.

**Option A: Fixed frame or arm resonance**
- Set it as a static notch: `INS_HNTC2_ENABLE=1`, `INS_HNTC2_MODE=0`.
- Set `INS_HNTC2_FREQ` to the non-moving peak seen in the FFT.
- Set `INS_HNTC2_BW` to roughly half of `INS_HNTC2_FREQ`, and `INS_HNTC2_ATT` to `30`–`40`.

**Option B: Throttle-based backup in case RPM telemetry drops**
- `INS_HNTC2_MODE=1`, `INS_HNTC2_REF` set to hover throttle (`MOT_THST_HOVER`).
- `INS_HNTC2_FREQ` and `INS_HNTC2_BW` the same as the primary notch.
- `INS_HNTC2_HMNCS=1`.
- Good for long-range or heavy-lift aircraft where ESC telemetry glitches are a risk.

---

## 5. Low-Pass Filters (Once Notches Are Effective)

| Parameter | Value |
|---|---|
| `INS_GYRO_FILTER` | 3–5": `80`–`110`; 7–10": `40`–`60`; large: `20`–`40` |
| `INS_ACCEL_FILTER` | `10`–`20` |
| `ATC_RAT_*_FLTD` | ≈ `INS_GYRO_FILTER / 2` |
| `ATC_RAT_*_FLTT` | Match `INS_GYRO_FILTER`, or leave at default |

With effective RPM notches you can usually raise the gyro LPF. That reduces phase lag and lets you run higher rate gains.

---

## 6. Logging for Validation

| Parameter | Value |
|---|---|
| `INS_LOG_BAT_MASK` | `1` (first IMU) |
| `INS_LOG_BAT_OPT` | `4` (pre- and post-filter batch sampling) |
| `INS_RAW_LOG_OPT` | Optional raw gyro logging (4.5+) for higher-resolution FFT |
| `LOG_BITMASK` | Include IMU and fast-attitude bits |

### Validation procedure
1. **Baseline flight:** notch enabled but `INS_HNTCH_ATT=0`, or notch disabled. Hover for 60 s plus some gentle manoeuvres.
2. Open the log in the **ArduPilot WebTools → Filter Review / FFT tool**.
   - Confirm the noise peaks track the ESC RPM line.
   - Check for a 2nd harmonic or a frame resonance.
3. **Apply settings** and fly the same profile.
4. **Acceptance criteria:**
   - Post-filter gyro spectrum shows the motor peaks reduced by at least 20–30 dB.
   - Noise floor below about −40 dB across 50–500 Hz.
   - No peak below `INS_HNTCH_FREQ` during low throttle or descent. If there is one, lower `INS_HNTCH_FREQ`.
   - Scheduler load in the `PM` log stays below 70%, with no long loops.
   - Motor temperatures are normal after landing.
5. **Then tune:** raise `INS_GYRO_FILTER` and run QuickTune or AutoTune.

---

## 7. Example "Cycle 26" Parameter Block (5" Quad, 1 kHz)

```
MOT_PWM_TYPE      6
SERVO_BLH_BDMASK  15
SERVO_BLH_POLES   14
SERVO_BLH_AUTO    1
SERVO_DSHOT_ESC   1
SCHED_LOOP_RATE   1000
INS_FAST_SAMPLE   1
INS_GYRO_RATE     1
INS_HNTCH_ENABLE  1
INS_HNTCH_MODE    3
INS_HNTCH_REF     1
INS_HNTCH_FREQ    100
INS_HNTCH_BW      50
INS_HNTCH_ATT     40
INS_HNTCH_HMNCS   3
INS_HNTCH_OPTS    14
INS_HNTC2_ENABLE  0      # enable per Section 4 if FFT shows frame resonance
INS_GYRO_FILTER   90
INS_ACCEL_FILTER  20
ATC_RAT_RLL_FLTD  45
ATC_RAT_PIT_FLTD  45
ATC_RAT_YAW_FLTE  2
INS_LOG_BAT_MASK  1
INS_LOG_BAT_OPT   4
```

---

## 8. Common Pitfalls
- **Wrong `SERVO_BLH_POLES`:** the notch sits at the wrong frequency, typically off by a ratio of 12/14. The FFT shows the peak offset from the RPM line.
- **`INS_HNTCH_FREQ` set too high:** the notch clamps above the real frequency during descents or low throttle, causing oscillation and hot motors on descent.
- **Bidir DShot on a non-capable timer or DMA channel:** RPM reads 0, and the notch silently sits at `INS_HNTCH_FREQ`. Check the `ESC` log messages.
- **Forgetting the reboot:** you must reboot after setting `INS_HNTCH_ENABLE` and `SERVO_BLH_BDMASK`.

If you share the airframe size, motor KV, prop, battery cell count, and a pre-filter FFT log, I can narrow these values down to specific numbers.