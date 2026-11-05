# ArduPilot Dynamic Harmonic Notch Configuration: STM32H743 with Bidirectional DShot RPM

I don't know what "Cycle 22" refers to in your context. It could be a test iteration, build cycle, or internal tuning round. If it maps to specific firmware, a frame, or prior results, share those and I'll adjust. The configuration below assumes **ArduCopter 4.4/4.5+** on an H743 flight controller with BLHeli32, AM32 or Bluejay ESCs.

---

## 1. RPM Telemetry Source (Bidirectional DShot)

| Parameter | Value | Notes |
|---|---|---|
| `MOT_PWM_TYPE` | 6 (DShot600) or 5 (DShot300) | Use DShot300 if you see telemetry errors on long ESC leads. |
| `SERVO_BLH_AUTO` | 1 | Auto-maps motor outputs to BLHeli. |
| `SERVO_BLH_BDMASK` | 15 (outputs 1–4) | Bitmask of bidirectional DShot outputs. Use 255 for an octo. |
| `SERVO_BLH_POLES` | 14 | Typical for 22xx/23xx motors. **Count magnets** to confirm. |
| `SERVO_DSHOT_ESC` | 1 (BLHeli32/Bluejay) or 3 (AM32) | Must match the ESC firmware. |
| `SERVO_DSHOT_RATE` | 0–2 | Multiplier of the loop rate. |

**Verification:** check the `ESC[n].RPM` log fields. The motor-pole count must be correct, because a wrong value scales every notch to the wrong frequency.

---

## 2. Gyro Sampling and Loop Rate (H743 Headroom)

| Parameter | Value | Notes |
|---|---|---|
| `INS_GYRO_RATE` | 1 (2 kHz) | The H7 can handle 2 (4 kHz), but check `PM` log load first. |
| `SCHED_LOOP_RATE` | 400 (small craft: 800) | 800 requires a fast IMU backend. |
| `INS_GYRO_FILTER` | 80–120 Hz | The notches carry most of the filtering, so this low-pass can be raised. |
| `INS_ACCEL_FILTER` | 20 Hz | Default. |

---

## 3. Primary Harmonic Notch (RPM-Driven, Per-Motor)

| Parameter | Value | Rationale |
|---|---|---|
| `INS_HNTCH_ENABLE` | 1 | Reboot after setting this to expose the other parameters. |
| `INS_HNTCH_MODE` | 3 | Tracks frequency from ESC telemetry. |
| `INS_HNTCH_REF` | 1.0 | Scaling factor (ESC mode uses RPM directly). |
| `INS_HNTCH_FREQ` | ~0.8 × idle motor frequency | Lower clamp. See the frame-class table below. |
| `INS_HNTCH_BW` | ~0.5 × `FREQ` | Bandwidth. |
| `INS_HNTCH_ATT` | 40 dB | Attenuation depth. |
| `INS_HNTCH_HMNCS` | 7 (1st + 2nd + 3rd) | Use 3 if the 3rd harmonic isn't visible in the FFT. |
| `INS_HNTCH_OPTS` | 14 | Bit 1 = multi-source (one notch set per motor), bit 2 = update at loop rate, bit 3 = all IMUs. |
| `INS_HNTCH_FM_RAT` | 1.0 | Lower values allow tracking below `FREQ` at low throttle. |

**Computational cost:** 4 motors × 3 harmonics × 3 IMUs gives 36 biquad filters. The H743 handles this, but confirm that `PM.Load` stays below ~60%.

**Starting values by frame class:**

| Frame | `INS_HNTCH_FREQ` | `INS_HNTCH_BW` | `INS_GYRO_FILTER` |
|---|---|---|---|
| 3–5" (high KV) | 80–100 | 40–50 | 100–120 |
| 7" | 60–70 | 30–35 | 80–100 |
| 10–15" | 40–50 | 20–25 | 60–80 |
| 18"+ heavy lift | 25–35 | 12–18 | 40–60 |

---

## 4. Secondary Notch (`INS_HNTC2_*`)

Use the second notch bank for whichever of these your logs show:

- **Frame or arm resonance** (a fixed peak that doesn't track RPM):
  - `INS_HNTC2_ENABLE = 1`, `MODE = 0` (static), `FREQ = <peak>`, `BW = peak/4`, `ATT = 30`, `HMNCS = 1`.
- **RPM-telemetry failsafe** (backup if ESC telemetry drops):
  - `MODE = 1` (throttle-based), tuned using `MOT_THST_HOVER` and the hover frequency.
  - Also set `INS_HNTCH_OPTS` bit 5 if your firmware supports "use min freq on RPM failure".

---

## 5. Verification Procedure

1. **Pre-filter capture.** Set `INS_LOG_BAT_MASK = 1` and `INS_LOG_BAT_OPT = 4` (logs pre- and post-filter data in sync). On 4.5+, `INS_RAW_LOG_OPT` provides similar options.
2. **Hover flight.** Hover for 60 seconds, then do gentle throttle sweeps and some moderate maneuvers.
3. **Spectrum analysis.** Load the log into the ArduPilot Filter Review Tool (firmware.ardupilot.org/Tools/FilterReview).
   - Confirm that the 1st, 2nd and 3rd harmonic peaks track the per-motor RPM lines.
   - Post-filter noise floor target: less than about −40 dB relative to the pre-filter peak.
   - Check for residual peaks not covered by the notches. Assign these to `HNTC2` or add harmonics.
4. **Iterate.**
   - Lower `INS_HNTCH_ATT` or `BW` if the filter adds too much phase lag (shown as sluggish rate tracking, or `RATE.*Out` oscillation).
   - Once the notches are clean, raise `INS_GYRO_FILTER` and the D-term filters (`ATC_RAT_RLL_FLTD`, `ATC_RAT_PIT_FLTD`) toward 1.5–2× their defaults.
5. **Retune the PIDs.** Re-run AutoTune after finalizing the filters, because the reduced phase lag changes the optimal gains.

---

## 6. Common Pitfalls

- **Wrong `SERVO_BLH_POLES`:** every notch lands at the wrong frequency.
- **Missing `INS_HNTCH_OPTS` bit 1:** without multi-source, the notches use average RPM, which smears the filtering during yaw or asymmetric loading.
- **`FREQ` set too low:** the notch can sit inside the control bandwidth and destabilize the craft at idle or descent.
- **BDMASK mismatch:** outputs not set to bidirectional silently report RPM = 0. The fallback is the `FREQ` floor.
- **Timer/DMA conflicts on H743:** bidirectional DShot on some outputs can conflict with UART DMA. Check `@SYS/dma.txt` for shared streams.

If you share your frame size, motor KV and pole count, prop size, and a log (or FFT screenshot), I can turn these into exact values for your build.