# ArduPilot Harmonic Notch Configuration: STM32H743 + Bidirectional DShot RPM (Cycle 2)

I've interpreted "Cycle 2" as the **second tuning iteration**. Cycle 1 established bidirectional DShot telemetry and captured baseline logs. Cycle 2 sets the RPM-tracking notches and then verifies them. The parameters assume **ArduPilot Copter 4.3–4.5** on a quad with 5"–7" props. Where noted, substitute your Cycle 1 log values.

> **Verify against your firmware version.** Option bitmasks and parameter names change between ArduPilot releases. Check each value against the parameter documentation for your exact version before writing it to the vehicle.

---

## 1. Prerequisites: ESC Telemetry Path

| Parameter | Value | Rationale |
|---|---|---|
| `MOT_PWM_TYPE` | `6` (DShot600) | Use `5` (DShot300) for ESCs that cannot handle DShot600 |
| `SERVO_BLH_BDMASK` | `15` (motors 1–4) | Enables bidirectional DShot eRPM per output |
| `SERVO_BLH_POLES` | `14` | Count the motor's magnets (typically 12 for 1407-class, 14 for 22xx) |
| `SERVO_DSHOT_ESC` | `1` (BLHeli32/AM32) or `3` (BLHeli_S w/ Bluejay) | Sets the ESC protocol type |
| `SERVO_DSHOT_RATE` | `0`–`2` | Multiplier relative to the loop rate. `2` is reasonable at 400 Hz |
| `SCHED_LOOP_RATE` | `400` (or `800` for small, aggressive craft) | The H743 has the CPU headroom for 800 Hz |
| `INS_GYRO_RATE` | `1` (2 kHz) or `2` (4 kHz) | Provides fast sampling so the notches act on clean high-rate data |

**Check:** In the ESC telemetry log messages (`ESC[n].RPM`), all motors should report RPM smoothly with **no dropouts** before you proceed.

---

## 2. Primary Harmonic Notch (RPM-Tracked, Per-Motor)

| Parameter | Value | Formulation |
|---|---|---|
| `INS_HNTCH_ENABLE` | `1` | Reboot after setting so the remaining parameters appear |
| `INS_HNTCH_MODE` | `3` | Sets the frequency source to ESC telemetry (bidirectional DShot) |
| `INS_HNTCH_REF` | `1.0` | In ESC mode, the RPM is used directly, so no throttle scaling is applied |
| `INS_HNTCH_FREQ` | **≈ 0.5–0.6 × hover motor frequency** | This is the minimum tracked frequency (the floor). Example: hover 140 Hz → `80` |
| `INS_HNTCH_BW` | **≈ FREQ / 2** | Example: `40` |
| `INS_HNTCH_ATT` | `40` dB | Deep attenuation. Per-motor notches can be narrow, so less phase cost |
| `INS_HNTCH_HMNCS` | `7` (1st + 2nd + 3rd) | Use `3` if Cycle 1 FFT showed no meaningful 3rd harmonic |
| `INS_HNTCH_OPTS` | `6` | Bit 1 = one notch set per motor (multi-source); bit 2 = update at loop rate |
| `INS_HNTCH_FM_RAT` | `1.0` | Throttle-scaling floor ratio. It has little effect in ESC mode, so keep the default |

**Notch count check:** 4 motors × 3 harmonics = 12 notches per IMU. With `EnableOnAllIMUs` (bit 3, giving `OPTS = 14`), this doubles or triples. The H743 handles 2 IMUs × 12 notches at 400 Hz comfortably. After enabling, watch `PM.Load` and the `NLon` field; keep load **below ~60%**.

**Deriving hover frequency from Cycle 1 logs:**
`f_hover (Hz) = ESC.RPM_hover / 60`
Example: 8400 RPM → 140 Hz → `FREQ = 80`, `BW = 40`.

---

## 3. Secondary Notch (Optional, Static/Throttle Backup or Frame Resonance)

Use this only if the Cycle 1 FFT showed a **non-motor peak**, such as frame resonance or a camera mount, or if you want a fallback in case RPM telemetry fails.

| Parameter | Value |
|---|---|
| `INS_HNTC2_ENABLE` | `1` |
| `INS_HNTC2_MODE` | `0` (fixed) for frame resonance, or `1` (throttle) for an RPM-loss fallback |
| `INS_HNTC2_FREQ` | The frequency of the observed peak |
| `INS_HNTC2_BW` | ≈ FREQ / 4 (a narrow fixed notch) |
| `INS_HNTC2_ATT` | `30` |
| `INS_HNTC2_HMNCS` | `1` |

---

## 4. Low-Pass Filters (Relax Once Notches Are Effective)

| Parameter | Cycle 1 (typical) | Cycle 2 target |
|---|---|---|
| `INS_GYRO_FILTER` | 20–40 Hz | **60–90 Hz**, depending on residual noise |
| `INS_ACCEL_FILTER` | 20 | 20 (leave unchanged) |
| `ATC_RAT_RLL/PIT_FLTD` | 10–20 | Up to roughly `INS_GYRO_FILTER / 2` |
| `ATC_RAT_YAW_FLTE` | 2 | 2 (leave unchanged) |

Raise `INS_GYRO_FILTER` in steps of about 10 Hz. Stop when the post-filter spectrum shows rising broadband noise or the motors run warm.

---

## 5. Verification Logging (Cycle 2 Flight)

| Parameter | Value |
|---|---|
| `INS_LOG_BAT_MASK` | `1` (first IMU) |
| `INS_LOG_BAT_OPT` | `4` (pre- and post-filter, sensor rate) |
| `LOG_BITMASK` | Include IMU_FAST if available |

**Procedure:**
1. Hover for 30–60 seconds, then perform moderate throttle sweeps and a few punch-outs.
2. Open the log in the **WebTools FFT viewer** or Mission Planner's *FFT → Filter Review*.
3. Pass criteria:
   - Post-filter spectrum: motor fundamental and harmonics are reduced by **≥ 20 dB**.
   - No residual peak tracks throttle above about −40 dB relative to pre-filter.
   - `PM.Load` stays below about 60% and there are no long-loop warnings.
   - Motor temperatures after landing are lower than in Cycle 1.

---

## 6. Cycle 2 → Cycle 3 Decision Rules

| Observation | Adjustment |
|---|---|
| A residual peak sits at the fundamental at low throttle | Lower `INS_HNTCH_FREQ` by 10–15 Hz |
| Peaks are suppressed but the attitude response is sluggish or shows overshoot from phase lag | Reduce `HMNCS` to `3` or `ATT` to `30` |
| A 2nd harmonic is dominant (common on 2-blade props) | Keep `HMNCS` at `7` or above and confirm bit 1 is set in `OPTS` |
| eRPM dropouts appear in `ESC` logs | Lower to DShot300, check `SERVO_BLH_POLES`, and update the ESC firmware |
| The spectrum is clean up to 200 Hz or more | Raise `INS_GYRO_FILTER`, then retune the rate PIDs (Autotune) |

---

**Example final set** (quad, 140 Hz hover, 14-pole motors):
`MOT_PWM_TYPE=6, SERVO_BLH_BDMASK=15, SERVO_BLH_POLES=14, INS_HNTCH_ENABLE=1, INS_HNTCH_MODE=3, INS_HNTCH_REF=1, INS_HNTCH_FREQ=80, INS_HNTCH_BW=40, INS_HNTCH_ATT=40, INS_HNTCH_HMNCS=7, INS_HNTCH_OPTS=6, INS_GYRO_FILTER=80, INS_LOG_BAT_MASK=1, INS_LOG_BAT_OPT=4`

If you share your Cycle 1 hover RPM, prop and frame size, and an FFT screenshot, I can tighten the specific numbers.