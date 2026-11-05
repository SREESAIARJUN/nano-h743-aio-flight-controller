# ArduPilot Gyro Notch Configuration: STM32H743 + Bidirectional DShot RPM

I've treated "Cycle 27" as your tuning iteration label. The configuration below is a complete baseline. Values marked **⟨tune⟩** must be confirmed against your own logs. Parameter names follow ArduPilot 4.4/4.5+. Always check the bit definitions in your GCS parameter descriptions, because bitmask meanings have shifted slightly between releases.

---

## 1. Prerequisites: RPM Telemetry Source

| Parameter | Value | Notes |
|---|---|---|
| `MOT_PWM_TYPE` | 6 (DShot600) or 5 (DShot300) | DShot300 is often more robust for bidirectional DShot on long wires |
| `SERVO_BLH_BDMASK` | e.g. `15` (outputs 1–4) | Enables bidirectional DShot on the motor outputs |
| `SERVO_BLH_POLES` | 14 ⟨tune⟩ | Count the motor's magnets. 14 is typical for 22xx/23xx motors, 12 for many small motors |
| `SERVO_BLH_AUTO` | 1 | |
| `SERVO_DSHOT_ESC` | ESC type (1=BLHeli32/AM32, 3=BLHeli_S w/ Bluejay) | Firmware must support EDT/bidirectional DShot |

**Verify before continuing:**
- `ESC1_RPM` … `ESC4_RPM` should read sensibly in the Status tab with props off.
- Error rates in the log `EDT2`/`ESC` messages should be below about 1%.
- A wrong pole count scales every notch frequency incorrectly.

---

## 2. Gyro Sampling (H743 Headroom)

The H743 has CPU to spare, so run fast sampling and per-motor notches.

| Parameter | Value | Notes |
|---|---|---|
| `SCHED_LOOP_RATE` | 400 (or 800 for small, agile frames) | Notches update at loop rate |
| `INS_GYRO_RATE` | 1 (2 kHz) or 2 (4 kHz) | Use `INS_FAST_SAMPLE` on older firmware |
| `INS_GYRO_FILTER` | 40 initially → raise to 80–120 after notches are verified | Less static LPF means less phase lag |
| `INS_ACCEL_FILTER` | 20 | |

---

## 3. Harmonic Notch #1: RPM-Tracked (Primary)

| Parameter | Value | Rationale |
|---|---|---|
| `INS_HNTCH_ENABLE` | 1 | Reboot after setting so the rest of the parameters appear |
| `INS_HNTCH_MODE` | **3** | ESC telemetry (RPM) tracking |
| `INS_HNTCH_REF` | 1 | Scaling reference. Must be 1 in ESC mode |
| `INS_HNTCH_FREQ` | ~80% of the lowest motor fundamental at idle/low throttle ⟨tune⟩ | Floor frequency. Typical values: 5" ≈ 80–100 Hz, 7" ≈ 60–80 Hz, 10"+ ≈ 40–50 Hz |
| `INS_HNTCH_BW` | ≈ ½ × `INS_HNTCH_FREQ` | RPM tracking is precise, so this can be narrower than in throttle mode |
| `INS_HNTCH_ATT` | 40 dB | Raise to 50 dB if a residual peak remains |
| `INS_HNTCH_HMNCS` | **3** (1st+2nd), or **7** (1st–3rd) | Pick based on which harmonics appear in your pre-filter FFT |
| `INS_HNTCH_OPTS` | see breakdown below | |
| `INS_HNTCH_FM_RAT` | 1.0 | Lower it (e.g. 0.7) if motors spend time below `FREQ` |

**`INS_HNTCH_OPTS` breakdown:**

| Bit | Value | Function | Use? |
|---|---|---|---|
| 0 | 1 | Double notch | Optional. Unnecessary with per-motor tracking |
| 1 | 2 | **Multi-source (one notch set per motor)** | ✅ |
| 2 | 4 | **Update at loop rate** | ✅ |
| 3 | 8 | **Apply to all IMUs** | ✅ |
| 4 | 16 | Triple notch | No |

**Recommended value: `OPTS = 14`** (2 + 4 + 8).

**CPU load check:** 4 motors × 3 harmonics × 3 IMUs = 36 notch instances. This is fine on an H743. Confirm with `PM` log messages (or `SCHED_OPTIONS` debug) that the loop isn't overrunning.

---

## 4. Harmonic Notch #2: Static / Frame Resonance (Optional)

Use this only if the pre-filter FFT shows a fixed peak that does not move with throttle, such as arm or frame resonance:

| Parameter | Value |
|---|---|
| `INS_HNTC2_ENABLE` | 1 |
| `INS_HNTC2_MODE` | 0 (fixed) |
| `INS_HNTC2_FREQ` | measured resonance peak ⟨tune⟩ |
| `INS_HNTC2_BW` | ~¼ of freq |
| `INS_HNTC2_ATT` | 30–40 dB |
| `INS_HNTC2_HMNCS` | 1 |

**Alternative:** if RPM telemetry is ever unreliable, configure notch #2 as `MODE=1` (throttle-based) with a wider bandwidth as a fallback.

---

## 5. Logging for Verification

| Parameter | Value |
|---|---|
| `LOG_BITMASK` | include IMU_RAW (or set `INS_RAW_LOG_OPT`, 4.5+) |
| `INS_LOG_BAT_MASK` | 1 (first IMU) |
| `INS_LOG_BAT_OPT` | **4** (pre- *and* post-filter batch) |
| `INS_LOG_BAT_LGIN` / `LGCT` | 20 / 2 |

---

## 6. Cycle 27 Procedure

1. **Hover flight, 30–60 s.** Include some throttle sweeps and gentle maneuvers.
2. **Open the log in the ArduPilot Filter Review Tool** (firmware.ardupilot.org/Tools/FilterTool).
3. **Pre-filter FFT:** identify the fundamental and its harmonics, and confirm they track `ESCx_RPM / 60`.
4. **Post-filter FFT:** the motor peaks should be suppressed by ≥ 20–30 dB. If a peak survives:
   - If it is unattenuated, add that harmonic to `HMNCS`.
   - If it sits just outside the notch, check pole count and raise `BW` by 10–20%.
   - If it appears at low throttle, lower `FREQ`.
5. **Raise `INS_GYRO_FILTER`** in steps (60 → 80 → 100) while the post-filter noise floor stays low.
6. **Re-run Autotune** (or re-check PIDs). Lower filter lag usually allows higher D gains.
7. **Check motor temperatures after landing.** Warm motors mean excess noise is still reaching the D term.

---

## Compact Parameter Block (5–7" Quad Baseline)

```
MOT_PWM_TYPE        6
SERVO_BLH_BDMASK    15
SERVO_BLH_POLES     14
SCHED_LOOP_RATE     400
INS_GYRO_RATE       1
INS_GYRO_FILTER     80
INS_HNTCH_ENABLE    1
INS_HNTCH_MODE      3
INS_HNTCH_REF       1
INS_HNTCH_FREQ      80
INS_HNTCH_BW        40
INS_HNTCH_ATT       40
INS_HNTCH_HMNCS     7
INS_HNTCH_OPTS      14
INS_HNTCH_FM_RAT    1.0
INS_LOG_BAT_MASK    1
INS_LOG_BAT_OPT     4
```

If you share your frame size, prop/motor/KV, battery cell count, firmware version, and a pre/post batch-sampler log from the previous cycle, I can narrow the **⟨tune⟩** values precisely.