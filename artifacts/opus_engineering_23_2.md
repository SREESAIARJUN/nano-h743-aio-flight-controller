# ArduPilot Dynamic Harmonic Notch Configuration
**Target:** STM32H743 FC · Bidirectional DShot RPM telemetry · ArduCopter 4.4+

> **Note on "Cycle 23":** I'm treating this as your internal tuning iteration. If it refers to a specific firmware branch, airframe revision, or prior log set, share it and I'll adjust the values.

---

## 1. Prerequisites: Bidirectional DShot RPM Source

The notch is only as good as the RPM signal, so configure this first.

| Parameter | Value | Notes |
|---|---|---|
| `MOT_PWM_TYPE` | `6` (DShot600) | Use `5` (DShot300) for 8–10" craft or noisy ESCs. |
| `SERVO_BLH_BDMASK` | e.g. `15` | Bitmask of motor outputs with bidir DShot. `15` = outputs 1–4, `255` = 1–8. |
| `SERVO_BLH_POLES` | `14` | Motor magnet count. Count them, because an error here mis-scales every notch. |
| `SERVO_BLH_AUTO` | `1` | Maps BLHeli outputs automatically. |
| `SERVO_BLH_TRATE` | `10` | Telemetry log rate (Hz). Does not affect notch update speed. |

**Firmware:** Use the `*-bdshot` target if one exists for your board (e.g. `MatekH743-bdshot`). Many H743 boards ship bdshot in the standard build since 4.3.

**ESCs:** Need bidir-capable firmware (BLHeli_32, Bluejay, AM32).

**Verify before tuning:** Check `ESC[n].RPM` in the log or the Status tab. Values should be smooth, non-zero at idle, and have no dropouts.

---

## 2. Primary Harmonic Notch: Per-Motor ESC Tracking

| Parameter | Value | Rationale |
|---|---|---|
| `INS_HNTCH_ENABLE` | `1` | Reboot afterwards to expose the remaining parameters. |
| `INS_HNTCH_MODE` | `3` | ESC telemetry tracking. |
| `INS_HNTCH_REF` | `1` | Scaling is 1:1 with telemetry RPM in ESC mode. |
| `INS_HNTCH_FREQ` | see §4 | Minimum tracking frequency (Hz). The notch will not go below this. |
| `INS_HNTCH_BW` | `FREQ / 2` | Starting point for −3 dB bandwidth. |
| `INS_HNTCH_ATT` | `40` | Attenuation (dB). Use 30–50. |
| `INS_HNTCH_HMNCS` | `3` (1st+2nd) | Use `7` to add the 3rd harmonic if a 3-blade prop shows it in FFT. |
| `INS_HNTCH_OPTS` | `6` | bit1 = per-motor notches, bit2 = update at loop rate. |
| `INS_HNTCH_FM_RAT` | `1.0` | No low-throttle frequency collapse is needed with RPM data. |

**H743 CPU budget:** Notch count = motors × harmonics × IMUs × (double/triple multiplier).
- Example: 4 motors × 2 harmonics × 2 IMUs = 16 biquads. This is comfortable on H743 at 1–2 kHz.
- Adding bit0 (double notch) or bit3 (all IMUs) multiplies that count.
- Watch `PM.Load` and `PM.NLon` (long loops). Keep load below about 60%.

**Optional `OPTS` bits:**
- `+8` (bit3) applies the notch on all IMUs. Recommended if EKF lane-switching is possible.
- `+1` (bit0) adds a double notch for broader suppression. Only add it if the FFT shows a wide peak.
- Typical H743 production value: `INS_HNTCH_OPTS = 14`.

---

## 3. Secondary Notch: Static Frame/Arm Resonance

Use this for fixed structural modes (arm flex, camera mount) that RPM tracking does not cover.

| Parameter | Value |
|---|---|
| `INS_HNTC2_ENABLE` | `1` |
| `INS_HNTC2_MODE` | `0` (fixed) |
| `INS_HNTC2_FREQ` | Peak from post-filter FFT that doesn't move with throttle |
| `INS_HNTC2_BW` | `FREQ / 4` (narrow) |
| `INS_HNTC2_ATT` | `30` |
| `INS_HNTC2_HMNCS` | `1` |

Alternative: set `INS_HNTC2_MODE = 1` (throttle-based) as a fallback in case RPM telemetry is lost. Pair it with `INS_HNTC2_REF` set to the hover throttle (`MOT_THST_HOVER`).

---

## 4. Frequency Starting Points by Airframe

Compute the hover fundamental with this formula, then set `INS_HNTCH_FREQ` ≈ 0.8 × that value:

> f_hover = hover RPM ÷ 60

| Class | Typical hover f | `INS_HNTCH_FREQ` | `INS_HNTCH_BW` |
|---|---|---|---|
| 3" | 180–250 Hz | 120 | 60 |
| 5" | 120–180 Hz | 80 | 40 |
| 7" | 80–120 Hz | 60 | 30 |
| 10" | 50–80 Hz | 40 | 20 |
| 15"+ | 30–50 Hz | 25 | 12 |

Constraint: keep `FREQ` above roughly 1.5× your rate-loop bandwidth. Otherwise the notch adds phase lag where it hurts handling.

---

## 5. Loop Rate & Low-Pass Filters (H743)

| Parameter | Value | Notes |
|---|---|---|
| `SCHED_LOOP_RATE` | `400` (≤7"), `400` default for large | |
| `INS_GYRO_RATE` | `1` (2 kHz) or `2` (4 kHz) | H743 can handle 4 kHz with good IMUs. |
| `INS_FAST_SAMPLE` | `1` (or `3` for dual IMU) | |
| `FSTRATE_ENABLE` | `1` | Fast-rate thread. Optional; pair with `FSTRATE_DIV` for 1–2 kHz rate loop. |
| `INS_GYRO_FILTER` | 20 → **raise to 40–80** once notch is verified | The main payoff of a good notch. |
| `ATC_RAT_*_FLTD` / `FLTT` | about `INS_GYRO_FILTER / 2` | |

---

## 6. Validation Workflow

1. **Baseline log.** Set:
   - `INS_LOG_BAT_MASK = 1`
   - `INS_LOG_BAT_OPT = 4` (pre- and post-filter batch sampling)
   - `LOG_BITMASK` to include IMU and raw IMU

   Then hover for 60 s and do a few throttle punches.
2. **Analyse.** Open the log in the ArduPilot **Filter Review Tool** (firmware.ardupilot.org/Tools/FilterTool) or the Mission Planner FFT tool.
3. **Confirm.** Pre-filter peaks should track `ESC.RPM`/60 and its multiples. Post-filter, those peaks should be at least 20 dB lower.
4. **Iterate:**
   - If residual peaks remain beside the notch, widen `BW` or add the double-notch bit.
   - If a peak sits at 2× or 3× frequency, add harmonics in `HMNCS`.
   - If a static peak remains, assign it to `INS_HNTC2`.
5. **Raise filters.** Increase `INS_GYRO_FILTER` in steps. Re-check motor temperatures and the D-term noise (`RATE.*Dout`).
6. **Re-tune PIDs.** Run AutoTune, or adjust manually, because the reduced filter lag permits higher D and P.

---

## 7. Quick-Load Template (5", 4 motors, dual IMU)

```
MOT_PWM_TYPE      6
SERVO_BLH_BDMASK  15
SERVO_BLH_POLES   14
SERVO_BLH_AUTO    1
INS_HNTCH_ENABLE  1
INS_HNTCH_MODE    3
INS_HNTCH_REF     1
INS_HNTCH_FREQ    80
INS_HNTCH_BW      40
INS_HNTCH_ATT     40
INS_HNTCH_HMNCS   3
INS_HNTCH_OPTS    14
INS_HNTCH_FM_RAT  1.0
INS_GYRO_FILTER   60
INS_LOG_BAT_MASK  1
INS_LOG_BAT_OPT   4
```

If you send a pre/post-filter batch log, I can derive exact `FREQ`, `BW`, and harmonic values for your airframe.