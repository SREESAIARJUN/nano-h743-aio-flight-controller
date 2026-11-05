# ArduPilot Harmonic Notch Configuration: STM32H743 with Bidirectional DShot RPM Feedback

> **Assumption on "Cycle 12":** I don't know what this refers to. It could be a tuning iteration, a 12N stator, or a 12-pole motor. I've written the plan generically and flagged the one place it matters most: **`SERVO_BLH_POLES`**. If you meant something else, tell me and I'll adjust.

Parameter names and option bits below match ArduPilot 4.4 and 4.5. Check them against the parameter list for your firmware version before writing them.

---

## 1. Prerequisites: Reliable RPM Telemetry

The notch can only be as good as the RPM data driving it, so set this up first.

| Parameter | Value | Notes |
|---|---|---|
| `MOT_PWM_TYPE` | `6` (DShot600) or `5` (DShot300) | Bidirectional DShot needs BLHeli_32, Bluejay or AM32 firmware. |
| `SERVO_BLH_BDMASK` | Bitmask of motor outputs, e.g. `15` for outputs 1–4 | Enables bidirectional DShot on those outputs. |
| `SERVO_BLH_POLES` | **Magnet count**, usually `14` | A 12N14P motor has **14** poles, not 12. Entering 12 skews every notch frequency by about 17%. |
| `SERVO_DSHOT_ESC` | `1` (BLHeli/Bluejay) or `3` (AM32) | Match your ESC firmware. |
| `SERVO_DSHOT_RATE` | `0` or `1` | Higher values tie the DShot rate to the loop rate. |
| `SERVO_BLH_AUTO` | `1` | Optional. |

**Verification:** Arm the vehicle props-off and check that the `ESC[n].RPM` log or GCS values are non-zero, stable, and plausible. Expect roughly 2,000–4,000 RPM at idle on a 5" quad. Also confirm the `ESC` error rate stays below about 1%.

---

## 2. Sampling and Loop Rate (H743 Headroom)

The H743 has enough CPU to run fast sampling, a higher loop rate, and many notches at once.

| Parameter | Value | Notes |
|---|---|---|
| `INS_FAST_SAMPLE` | `1` (first IMU) or `3` (both) | Enables raw high-rate gyro sampling. |
| `INS_GYRO_RATE` | `1` (2 kHz) or `2` (4 kHz) | 4 kHz on small, high-KV craft. |
| `SCHED_LOOP_RATE` | `400` (large) to `800–1000` (5" racer/freestyle) | Notches must stay below Nyquist of the loop rate. |
| `INS_GYRO_FILTER` | `40–80` Hz | Raise this once the notch is effective, to reduce phase lag. |

---

## 3. Primary Dynamic Harmonic Notch (ESC RPM Tracking)

| Parameter | Value | Rationale |
|---|---|---|
| `INS_HNTCH_ENABLE` | `1` | Turns the notch on; reboot afterwards to expose the other parameters. |
| `INS_HNTCH_MODE` | `3` | ESC telemetry tracking. |
| `INS_HNTCH_REF` | `1.0` | Scales the RPM reference. Leave at 1 for ESC mode. |
| `INS_HNTCH_FREQ` | About 80% of the idle rotor frequency (see below) | Acts as the minimum tracked frequency. |
| `INS_HNTCH_BW` | `INS_HNTCH_FREQ / 2` | Standard ratio. |
| `INS_HNTCH_ATT` | `40` dB | Default; 30–50 dB is the usable range. |
| `INS_HNTCH_HMNCS` | `3` (1st + 2nd) or `7` (1st + 2nd + 3rd) | Use 7 for tri-blade props or if FFT shows a 3rd harmonic. |
| `INS_HNTCH_OPTS` | `6` (bit 1 + bit 2) | Bit 1 runs one notch set per motor; bit 2 updates at loop rate. Add bit 3 (`+8` → `14`) to apply to all IMUs. |
| `INS_HNTCH_FM_RAT` | `1.0` | Throttle-floor ratio. ESC mode tracks real RPM, so keep at 1. |

### Setting the Minimum Frequency

Use this formula:

```
f_rotor(Hz) = RPM / 60
INS_HNTCH_FREQ ≈ 0.8 × (idle RPM / 60)
```

Example: an idle of 3,000 RPM gives 50 Hz, so `0.8 × 50 = 40` Hz. With that value, set `INS_HNTCH_BW = 20`.

### CPU Load

The notch count is motors × harmonics × notch multiplicity. For example, 4 motors × 3 harmonics × single notch = 12 notches per axis. The H743 handles this easily. Before flying, check that `PM.Load` stays below 60% and `PM.NLon` (loop overruns) stays low.

---

## 4. Optional Second Notch (Frame Resonance)

Use this only if the post-filter FFT shows a fixed peak that does not move with RPM, such as an arm or frame mode.

| Parameter | Value |
|---|---|
| `INS_HNTC2_ENABLE` | `1` |
| `INS_HNTC2_MODE` | `0` (fixed) |
| `INS_HNTC2_FREQ` | The measured resonance, e.g. `180` |
| `INS_HNTC2_BW` | About `FREQ / 4` (narrower than the primary notch) |
| `INS_HNTC2_ATT` | `30` |
| `INS_HNTC2_HMNCS` | `1` |

---

## 5. Logging and Validation Workflow

1. **Baseline (filters off):** Set `INS_LOG_BAT_MASK = 1` and `INS_LOG_BAT_OPT = 0`. Hover for 30–60 s, then do a few throttle punches.
2. **Configure the notch** from Sections 3 and 4.
3. **Pre/post comparison:** Set `INS_LOG_BAT_OPT = 4` (pre- and post-filter batch) and fly the same profile again.
   - On older firmware, use `2` for post-filter only.
4. **Analyze** in the ArduPilot Filter Review Tool (`firmware.ardupilot.org/Tools/WebTools/FilterReview`):
   - Motor peaks and harmonics should be reduced by at least 20–30 dB post-filter.
   - Remaining broadband noise should sit below about −40 dB.
5. **Tighten:**
   - If the notch is effective, raise `INS_GYRO_FILTER` in 10–20 Hz steps.
   - Then re-run QuikTune or AutoTune.
6. **Disable batch logging** once you're done (`INS_LOG_BAT_MASK = 0`).

---

## 6. Quick Reference: Typical 5" Quad on H743

```
MOT_PWM_TYPE      6
SERVO_BLH_BDMASK  15
SERVO_BLH_POLES   14
INS_FAST_SAMPLE   1
INS_GYRO_RATE     2
SCHED_LOOP_RATE   800
INS_GYRO_FILTER   60
INS_HNTCH_ENABLE  1
INS_HNTCH_MODE    3
INS_HNTCH_REF     1
INS_HNTCH_FREQ    40
INS_HNTCH_BW      20
INS_HNTCH_ATT     40
INS_HNTCH_HMNCS   7
INS_HNTCH_OPTS    6
INS_HNTCH_FM_RAT  1
```

**Larger craft (10"+):**
- Lower `SCHED_LOOP_RATE` to 400.
- Lower `INS_GYRO_RATE` to 1.
- Set `INS_HNTCH_FREQ` to 0.8 × the idle rotor frequency, typically 20–30 Hz.
- Set `INS_HNTCH_HMNCS` to 3.
- Set `INS_GYRO_FILTER` to 20–40 Hz.

If you share what "Cycle 12" means or upload a `.bin` log, I can refine the numbers from your actual spectrum.