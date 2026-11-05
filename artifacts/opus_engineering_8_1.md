# ArduPilot Dynamic Harmonic Notch Configuration: STM32H743 + Bidirectional DShot RPM Telemetry

**Assumptions**
- "Cycle 8" isn't a standard ArduPilot term. I've treated it as your 8th tuning iteration, so this is a refinement pass that assumes baseline logs exist.
- Firmware: ArduCopter 4.4 or later.
- Airframe: a 5–7" quad. Scaling notes for other sizes are in section 7.

---

## 1. RPM Source: Bidirectional DShot

| Parameter | Value | Rationale |
|---|---|---|
| `MOT_PWM_TYPE` | `6` (DShot600) | Use `5` (DShot300) for BLHeli_32 on long leads. |
| `SERVO_BLH_BDMASK` | `15` (motors 1–4) | Enables bidirectional DShot (eRPM) on outputs 1–4. Use `255` for 8 motors. |
| `SERVO_BLH_POLES` | `14` | Motor magnet pole count. Count the magnets. A wrong value scales every notch incorrectly. |
| `SERVO_BLH_AUTO` | `1` | Auto-map outputs. |
| `SERVO_DSHOT_RATE` | `1`–`2` | Keep `1` (loop rate) at 1 kHz loop, or `2` at ≤800 Hz. |
| `SERVO_DSHOT_ESC` | `1` (BLHeli32/AM32) or `3` (Bluejay) | Must match ESC firmware. |

**Prerequisites**
- The H743 target must have bidirectional DShot enabled. Standard H7 builds do.
- Motor outputs must be on DMA-capable timer groups.

**Verify before proceeding:** `ESC[n].RPM` in the log tracks throttle smoothly, with no dropouts and no zero spikes in flight.

---

## 2. Gyro Sampling and Loop Rate (H743 Headroom)

| Parameter | Value | Note |
|---|---|---|
| `SCHED_LOOP_RATE` | `1000` | Use `800` for 7"+. |
| `INS_GYRO_RATE` | `1` (2 kHz) | Use `2` (4 kHz) only if the IMU supports it and CPU load stays under 70%. |
| `INS_FAST_SAMPLE` | `1` (or `3` for dual IMU) | Fast sampling on the primary IMU. |

---

## 3. Primary Harmonic Notch (RPM-Tracked, Per-Motor)

| Parameter | Value | Rationale |
|---|---|---|
| `INS_HNTCH_ENABLE` | `1` | Enable. Reboot to reveal the remaining params. |
| `INS_HNTCH_MODE` | `3` | ESC telemetry (RPM) tracking. |
| `INS_HNTCH_REF` | `1` | Scale factor of 1.0 for RPM mode. |
| `INS_HNTCH_FREQ` | `80` (5") / `50` (7") | Floor for the tracked frequency, set just below hover-idle motor frequency. |
| `INS_HNTCH_BW` | `40` (5") / `25` (7") | About ½ of `FREQ`. |
| `INS_HNTCH_ATT` | `40` dB | Use 30–50. Higher values deepen the notch but add phase lag. |
| `INS_HNTCH_HMNCS` | `7` | 1st, 2nd and 3rd harmonics. Use `3` if the 3rd harmonic isn't visible in the FFT. |
| `INS_HNTCH_OPTS` | `6` | Bit 1 sets per-motor notches (multi-source). Bit 2 updates at loop rate. Add bit 3 (`+8` → `14`) if notching all IMUs. |
| `INS_HNTCH_FM_RAT` | `0.8` | Lets the notch track down to 80% of `FREQ` during throttle chops instead of clamping. |

**Notch budget:** 4 motors × 3 harmonics = 12 notches.
- Adding the double-notch option (bit 0) raises this to 24, which an H743 handles at a 1 kHz loop.
- Watch `PM.Load` / `PM.NLon`.
- Use double notch only if the post-filter FFT shows wide residual peaks.

---

## 4. Secondary Notch: Static Frame Resonance or Backup

Use this only if the FFT shows a fixed peak that doesn't move with throttle (frame or arm resonance).

| Parameter | Value |
|---|---|
| `INS_HNTC2_ENABLE` | `1` |
| `INS_HNTC2_MODE` | `0` (fixed) |
| `INS_HNTC2_FREQ` | Measured resonance (e.g. `180`) |
| `INS_HNTC2_BW` | ~¼ of the frequency (e.g. `45`) |
| `INS_HNTC2_ATT` | `30` |
| `INS_HNTC2_HMNCS` | `1` |

Alternative: use it as a throttle-based fallback (`MODE=1`) in case ESC telemetry is lost.

---

## 5. Downstream Low-Pass Filters (Relax After Notching)

With effective notching, the low-pass filters can be opened up to reduce phase lag:

| Parameter | Before | Cycle 8 target |
|---|---|---|
| `INS_GYRO_FILTER` | 40–60 | `80`–`110` (5"), `60`–`80` (7") |
| `INS_ACCEL_FILTER` | 20 | `20` |
| `ATC_RAT_RLL/PIT_FLTD` | — | ≈ ½ of `INS_GYRO_FILTER` |
| `ATC_RAT_RLL/PIT_FLTT` | — | Same as `FLTD` |
| `ATC_RAT_YAW_FLTE` | — | `2` (or `0` for responsive 5") |

---

## 6. Verification Procedure

1. **Logging setup:**
   - `INS_LOG_BAT_MASK = 1` (first IMU).
   - `INS_LOG_BAT_OPT = 4` (pre- and post-filter).
   - Use `INS_LOG_BAT_OPT = 5` to also log at sensor rate.
2. **Test flight:** Hover for 30 s, then do throttle sweeps and punch-outs.
3. **Analysis:** Open the log in the **ArduPilot Filter Review Tool** (web).
   - Post-filter motor peaks should be at least 20–30 dB below pre-filter.
   - Overlay the notch frequency on the spectrogram. It should sit on the motor lines across the whole throttle range.
   - Residual noise should sit below the `INS_GYRO_FILTER` corner.
4. **Health checks:**
   - CPU: `PM.Load` < 60–70%, with near-zero `PM.NLon` (long loops).
   - Motor temperatures are normal after landing.
5. **Retune:** Raise `INS_GYRO_FILTER` in 10 Hz steps, then run AutoTune or manually raise D terms.

---

## 7. Scaling Guide

| Frame | `HNTCH_FREQ` | `HNTCH_BW` | `GYRO_FILTER` | Loop |
|---|---|---|---|---|
| 3" | 120 | 60 | 120–150 | 1000–2000 |
| 5" | 80 | 40 | 80–110 | 1000 |
| 7" | 50 | 25 | 60–80 | 800 |
| 10"+ | 30 | 15 | 40–60 | 400 |

**Most common failure:** a wrong `SERVO_BLH_POLES` value, or ESC telemetry dropouts. Either will misplace every notch, so verify RPM against a tachometer or the motor KV × voltage estimate before trusting the filters.