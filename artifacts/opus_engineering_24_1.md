# ArduPilot Harmonic Notch Configuration: STM32H743 with Bidirectional DShot RPM Feedback

**About "Cycle 24":** This isn't an ArduPilot term. I've treated it as your internal tuning iteration label. If it refers to something specific, such as a log set or a prior parameter baseline, share it and I'll adjust the values. The parameters below follow ArduPilot 4.4–4.6 naming.

---

## 1. Prerequisites: Bidirectional DShot Telemetry

| Parameter | Value | Notes |
|---|---|---|
| `MOT_PWM_TYPE` | 6 (DShot600) | Use 5 (DShot300) if you see telemetry errors |
| `SERVO_BLH_BDMASK` | e.g. 15 (motors 1–4), 255 for octo | Enables bidirectional DShot on those outputs |
| `SERVO_BLH_POLES` | Motor magnet count (typically 14 for 5" class) | Wrong value scales all notch frequencies incorrectly |
| `SERVO_BLH_AUTO` | 1 | Maps BLHeli outputs to motor functions |
| `SERVO_DSHOT_ESC` | 1 (BLHeli32/AM32) or 3 (BLHeli_S/Bluejay) | Matches ESC firmware type |
| `SERVO_BLH_TRATE` | 10 | Serial telemetry rate; not needed if relying purely on BDShot |

**Before proceeding, verify:**
- `ESC_RPM` (or ESCX) logs show clean, non-zero RPM on all motors.
- The RPM error rate is below about 1%.
- Bench-spinning motors gives plausible RPM values in the MAVLink inspector.

---

## 2. Loop and Gyro Rates

The H743 has enough headroom for per-motor, multi-harmonic notches at full loop rate.

| Parameter | Value | Rationale |
|---|---|---|
| `SCHED_LOOP_RATE` | 400 (default) or 800 for small, agile craft | Use 800 only if `PM` logs show slack |
| `INS_GYRO_RATE` | 1 (2 kHz) or 2 (4 kHz) | 2 kHz is the robust choice; 4 kHz only with an IMU that supports it and low CPU load |
| `INS_FAST_SAMPLE` | 1 (or mask for all IMUs) | Required for high-rate gyro sampling |

---

## 3. Primary Harmonic Notch (RPM-Tracked, Per-Motor)

This notch tracks the motor rotation frequency and its harmonics, which are the dominant noise source.

| Parameter | Value | Formulation |
|---|---|---|
| `INS_HNTCH_ENABLE` | 1 | Reboot after enabling to expose the other parameters |
| `INS_HNTCH_MODE` | 3 | ESC telemetry / bidirectional DShot RPM source |
| `INS_HNTCH_REF` | 1.0 | Scale factor; RPM directly supplies the frequency |
| `INS_HNTCH_FREQ` | ≈ 0.8 × f_hover, typically 40–80 Hz | Acts as the floor frequency (see section 3a) |
| `INS_HNTCH_BW` | ≈ FREQ / 2 | Bandwidth at the floor; scales with frequency |
| `INS_HNTCH_ATT` | 40 dB | Raise to 50 only if the peak residual persists |
| `INS_HNTCH_HMNCS` | 7 (1st + 2nd + 3rd) | Use 3 for clean props; add 8 (4th) for tri-blades if logs show it |
| `INS_HNTCH_OPTS` | 14 | Option bits, see below |
| `INS_HNTCH_FM_RAT` | 1.0 | Lower to 0.7–0.8 to let the notch track below FREQ at low throttle |

**Breakdown of `INS_HNTCH_OPTS = 14`:**
- bit 1 (2): Multi-source, giving one notch set per motor
- bit 2 (4): Update the notch frequency at loop rate
- bit 3 (8): Apply to all IMUs

**Optional additions to OPTS:**
- +32 (bit 5): Hold the notch at minimum frequency if RPM data drops out. Recommended as a safety fallback.
- +1 (bit 0): Double notch, which widens attenuation per harmonic. Usually unnecessary with per-motor tracking and costs CPU.

**Suggested final value:** `INS_HNTCH_OPTS = 46` (2 + 4 + 8 + 32).

### 3a. Deriving f_hover

    f_hover (Hz) = hover eRPM / (pole_pairs × 60) = mechanical hover RPM / 60

Take the hover RPM from the `ESC` logs. For example, if hover is 9,000 RPM:
- f_hover = 150 Hz
- FREQ ≈ 120 Hz
- BW ≈ 60 Hz

### 3b. CPU Budget

Notch count = motors × harmonics × (notches per harmonic) × IMUs.

For a quad with 3 harmonics, single notch, and 2 IMUs, that is 24 filters per axis. This is fine on the H743 at 2 kHz. Check that `PM.Load` stays under about 70% and `PM.NLon` (loop overruns) is near zero.

---

## 4. Secondary Notch (`INS_HNTC2_*`): Frame Resonance

Use this only if the post-filter FFT shows a fixed peak that doesn't move with throttle, such as arm or frame resonance.

| Parameter | Value |
|---|---|
| `INS_HNTC2_ENABLE` | 1 |
| `INS_HNTC2_MODE` | 0 (fixed/static) |
| `INS_HNTC2_FREQ` | Peak frequency from the FFT |
| `INS_HNTC2_BW` | ≈ FREQ / 4 (narrow) |
| `INS_HNTC2_ATT` | 30–40 dB |
| `INS_HNTC2_HMNCS` | 1 |
| `INS_HNTC2_OPTS` | 8 (all IMUs) |

**Alternative:** use `MODE = 4` (in-flight FFT, which requires `FFT_ENABLE = 1`) if the residual peak drifts with battery sag or payload changes.

---

## 5. Low-Pass Filters After Notching

With effective notching you can relax the low-pass filters to reduce phase lag:

| Parameter | Typical Value |
|---|---|
| `INS_GYRO_FILTER` | 80–120 Hz (5"–7" craft); 40–60 Hz for larger frames |
| `ATC_RAT_RLL_FLTD` / `PIT_FLTD` | ≈ INS_GYRO_FILTER / 2 |
| `ATC_RAT_*_FLTT` | Same as INS_GYRO_FILTER, or slightly lower |
| `ATC_RAT_YAW_FLTE` | 2 Hz (default; keep low) |

---

## 6. Verification Workflow

1. **Capture a baseline log** with `INS_LOG_BAT_MASK = 1` and `INS_LOG_BAT_OPT = 4` (pre- and post-filter batch sampling). Do a 30–60 s hover plus some throttle sweeps.
2. **Analyze** with the ArduPilot Filter Review Tool (firmware.ardupilot.org/Tools/FilterTool) or Mission Planner's FFT.
   - Confirm the motor peaks align with the RPM-derived frequency and its harmonics.
   - Confirm the pre-filter peaks are attenuated by at least 20–40 dB post-filter.
3. **Iterate one parameter at a time:**
   - Raise BW if peaks smear past the notch edges.
   - Add harmonics if a peak at 4× remains.
   - Add HNTC2 for static residuals.
4. **Raise INS_GYRO_FILTER** in 20 Hz steps, checking for noise and hot motors.
5. **Re-tune** (manual or AutoTune). The rate PIDs can usually be raised substantially with clean gyro data.
6. **Switch to `INS_LOG_BAT_OPT = 0` or disable batch logging** for normal flights to save CPU and log bandwidth.

---

## 7. Starter Parameter Set (5" Quad, Hover ≈ 150 Hz)

```
MOT_PWM_TYPE        6
SERVO_BLH_BDMASK    15
SERVO_BLH_POLES     14
SERVO_BLH_AUTO      1
INS_GYRO_RATE       1
INS_HNTCH_ENABLE    1
INS_HNTCH_MODE      3
INS_HNTCH_REF       1
INS_HNTCH_FREQ      120
INS_HNTCH_BW        60
INS_HNTCH_ATT       40
INS_HNTCH_HMNCS     7
INS_HNTCH_OPTS      46
INS_HNTCH_FM_RAT    1.0
INS_GYRO_FILTER     100
INS_LOG_BAT_MASK    1
INS_LOG_BAT_OPT     4
```

For a 7"–10" craft, scale FREQ and BW down using the hover RPM formula and drop INS_GYRO_FILTER to 60–80 Hz.

If you share a `.bin` log or your frame/prop/motor details, I can refine FREQ, BW, the harmonic mask, and whether HNTC2 is warranted.