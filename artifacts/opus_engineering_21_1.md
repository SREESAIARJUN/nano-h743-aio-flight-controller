# ArduPilot Harmonic Notch Configuration: STM32H743 with Bidirectional DShot RPM

*(I'm not sure what "Cycle 21" refers to, so I've treated it as your iteration label. If it implies a specific firmware version, frame, or prior parameter set, share it and I'll adjust.)*

This assumes **ArduCopter 4.4 / 4.5+**, a quad, and BLHeli_32 / AM32 / Bluejay ESCs with bidirectional DShot.

---

## 1. Prerequisite: Bidirectional DShot RPM Telemetry

| Parameter | Value | Notes |
|---|---|---|
| `MOT_PWM_TYPE` | `6` | DShot600. Use `5` (DShot300) if signal integrity is marginal. |
| `SERVO_BLH_BDMASK` | `15` | Bidirectional DShot on outputs 1–4. Use `255` for an octo. |
| `SERVO_BLH_POLES` | `14` | Typical for 5" motors (12N14P). Count the magnets on yours. |
| `SERVO_BLH_AUTO` | `1` | Enables BLHeli passthrough/telemetry on all motor outputs. |
| `SERVO_DSHOT_ESC` | `1`, `2`, or `3` | 1 = BLHeli32/Kiss, 2 = BLHeli_S/Bluejay, 3 = AM32. |
| `SERVO_DSHOT_RATE` | `0` or `1` | 1 = loop-synchronous output. |

**Verify before going further:**
- The `ESC` log messages and the MAVLink ESC status show sane RPM at idle and when throttled.
- There are no persistent RPM dropouts. The H743 needs DMA-capable timers on the motor outputs, so check that the outputs are grouped on DShot-capable timers per the board's hwdef.
- If the RPM reading is wrong by a constant factor, `SERVO_BLH_POLES` is wrong.

---

## 2. Gyro Sampling and Loop Rate

| Parameter | Value | Notes |
|---|---|---|
| `INS_FAST_SAMPLE` | `1` (or `7` for all IMUs) | Fast sampling. |
| `INS_GYRO_RATE` | `1` | 2 kHz backend rate. Use `2` (4 kHz) only if CPU load allows. |
| `SCHED_LOOP_RATE` | `400` | Default. Raise to `800` only on small, fast quads. |
| `INS_GYRO_FILTER` | `40`–`80` Hz | Raise this once the notch is working. Higher means less latency. |

---

## 3. Primary Harmonic Notch (RPM-Tracked)

| Parameter | Value | Rationale |
|---|---|---|
| `INS_HNTCH_ENABLE` | `1` | Reboot after setting, so the remaining params appear. |
| `INS_HNTCH_MODE` | `3` | ESC telemetry (RPM) tracking. |
| `INS_HNTCH_REF` | `1` | Scaling reference. Must be 1.0 for ESC mode. |
| `INS_HNTCH_FREQ` | see sizing below | Minimum tracking frequency. |
| `INS_HNTCH_BW` | `FREQ / 2` | Bandwidth. |
| `INS_HNTCH_ATT` | `40` dB | Attenuation. |
| `INS_HNTCH_HMNCS` | `3` (1st+2nd) or `7` (1st–3rd) | Harmonic bitmask. |
| `INS_HNTCH_OPTS` | `6` | Bit 1 (2) gives a notch per motor; bit 2 (4) updates at loop rate. Add bit 3 (+8 = 14) to run on all IMUs. |
| `INS_HNTCH_FM_RAT` | `0.75` | Lets the notch track below `FREQ` (at 75% of it) during low-throttle descents. |

### Sizing `INS_HNTCH_FREQ`

Hover motor frequency is:

```
f_hover = (hover RPM) / 60
```

`ESC` logs report mechanical RPM, so no pole correction is needed.

**Example:** a 5" quad hovering at about 10,800 RPM gives 180 Hz.
- `INS_HNTCH_FREQ` = 0.8 × 180 ≈ **145 Hz** (roughly the idle/low-throttle RPM).
- `INS_HNTCH_BW` = **72 Hz**.

For larger frames (10"+ with low-kV motors), hover is often 60–90 Hz. Set `FREQ` around 50–60 Hz, and never below about 40 Hz, to avoid phase lag in the control band.

### CPU Budget on the H743

Each notch costs: per-motor notches (4) × harmonics × 3 axes × IMUs.

- 4 motors × 3 harmonics × 3 axes × 2 IMUs is fine on the H743 at a 400 Hz loop with a 2 kHz gyro.
- Check `PM` logs for `NLon` (long loops) and `Load` below about 60%. If load is too high, drop harmonics to `3` first, then drop the extra IMUs.

---

## 4. Secondary Notch (Static or Throttle-Based)

Use this only if the post-filter FFT still shows a peak that doesn't track motor RPM, such as frame or arm resonance or prop-wash.

| Parameter | Value |
|---|---|
| `INS_HNTC2_ENABLE` | `1` |
| `INS_HNTC2_MODE` | `0` (fixed) or `1` (throttle) |
| `INS_HNTC2_FREQ` | measured peak frequency |
| `INS_HNTC2_BW` | `FREQ / 4` (narrow for a fixed resonance) |
| `INS_HNTC2_ATT` | `30`–`40` |
| `INS_HNTC2_HMNCS` | `1` |
| `INS_HNTC2_REF` | `1` (fixed) or hover throttle (throttle mode) |

---

## 5. Verification Workflow

1. **Baseline log.** Disable notches and capture both pre- and post-filter data:
   - On 4.5+: `INS_RAW_LOG_OPT = 9`. Bit 0 logs the primary gyro only; bit 3 logs pre+post filter.
   - On older firmware, use batch sampling instead: `INS_LOG_BAT_MASK = 1`, `INS_LOG_BAT_OPT = 4`.
   - Fly a 60–90 s hover with gentle throttle sweeps.
2. **Analyse.** Use the ArduPilot Filter Review Tool (firmware.ardupilot.org/Tools/WebTools/FilterReview). Identify the motor fundamental and its harmonics, and confirm they follow the `ESC` RPM.
3. **Apply** the parameters above. The Filter Review Tool can simulate the notch on the pre-filter data before you fly.
4. **Re-fly and compare.** Target 20 dB or more of suppression at the fundamental and harmonics, and a post-filter noise floor that is flat and low.
5. **Re-tune.** Raise `INS_GYRO_FILTER`, then re-run QuikTune or AutoTune. The reduced latency usually allows higher PIDs and D-term.
6. **Restore logging.** Set `INS_RAW_LOG_OPT = 0` (or `INS_LOG_BAT_MASK = 0`) for normal flights.

---

## 6. Quick-Reference Parameter Block (5" Quad Example)

```
MOT_PWM_TYPE        6
SERVO_BLH_BDMASK    15
SERVO_BLH_POLES     14
SERVO_BLH_AUTO      1
INS_FAST_SAMPLE     1
INS_GYRO_RATE       1
INS_HNTCH_ENABLE    1
INS_HNTCH_MODE      3
INS_HNTCH_REF       1
INS_HNTCH_FREQ      145
INS_HNTCH_BW        72
INS_HNTCH_ATT       40
INS_HNTCH_HMNCS     7
INS_HNTCH_OPTS      6
INS_HNTCH_FM_RAT    0.75
INS_GYRO_FILTER     60
```

---

## 7. Common Pitfalls

- **Notch frequency stuck at the minimum.** RPM telemetry isn't arriving. Check `SERVO_BLH_BDMASK`, timer/DMA assignment, and ESC firmware support for EDT/bidirectional DShot.
- **High CPU load or long loops.** Too many harmonics or IMUs, or the gyro rate is too high for the loop rate.
- **Notch "chasing" during flips or descents.** Lower `INS_HNTCH_FM_RAT`, or confirm bit 2 (loop-rate update) is set.
- **Wrong scaling.** `SERVO_BLH_POLES` is incorrect.

If you share your frame size, motor kV, prop size, battery cell count, and a pre-filter log, I can compute exact `FREQ`/`BW` values and judge whether the secondary notch is needed.