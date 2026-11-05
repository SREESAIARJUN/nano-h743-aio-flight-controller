# ArduPilot Harmonic Notch Configuration: STM32H743 with Bidirectional DShot RPM

*I've read "Cycle 13" as your internal iteration label. If it refers to a specific airframe revision or test log, share the frame size, motor KV, cell count and hover throttle, and I'll turn the placeholders below into exact numbers.*

---

## 1. Prerequisites: Get Clean RPM Telemetry First

The dynamic notch is only as good as its frequency source. Confirm bidirectional DShot works before touching the filter parameters.

| Parameter | Value | Notes |
|---|---|---|
| `MOT_PWM_TYPE` | `6` (DShot600) or `5` (DShot300) | DShot300 is often more robust on long ESC leads. |
| `SERVO_BLH_BDMASK` | e.g. `15` (outputs 1–4) | Bitmask of motor outputs with bidirectional DShot. Octo = `255`. |
| `SERVO_BLH_POLES` | `14` | Motor **magnet** count. Typical 5" motors are 12N14P, so 14. Verify for your motor. |
| `SERVO_DSHOT_ESC` | `1` (BLHeli32/Bluejay) or `3` (AM32) | Must match the ESC firmware. |
| `SERVO_DSHOT_RATE` | `0`–`2` | Higher rates give fresher RPM; `0` is fine at 400 Hz loop. |
| `SERVO_BLH_AUTO` | `1` | Optional; maps BLHeli outputs automatically. |

On the H743, the bidirectional outputs must sit on timers/DMA streams that the board's hwdef supports for BDShot. Most H7 boards support outputs 1–4 or 1–8; check your board's README.

**Verify:** arm on the bench without props (or with props, safely). In the logs or Mission Planner status, `ESC[n].RPM` should be nonzero, smooth, and roughly equal across motors. `esc_rpm_error` / `ESC.Err` should be near 0%. If the error rate exceeds about 1–2%, fix the hardware or DShot rate before continuing.

---

## 2. Primary Harmonic Notch (RPM-Tracking)

| Parameter | Recommended | Rationale |
|---|---|---|
| `INS_HNTCH_ENABLE` | `1` | Reboot after setting so the remaining parameters appear. |
| `INS_HNTCH_MODE` | `3` | ESC telemetry (RPM) tracking. |
| `INS_HNTCH_REF` | `1` | Scale factor. Leave at 1 with ESC mode. |
| `INS_HNTCH_FREQ` | ~0.8 × hover motor frequency (Hz) | Acts as the **minimum** tracking frequency. See the formula below. |
| `INS_HNTCH_BW` | `FREQ / 2` | Standard starting point. |
| `INS_HNTCH_ATT` | `40` | dB attenuation at the notch centre. |
| `INS_HNTCH_HMNCS` | `3` (1st+2nd) or `7` (1st–3rd) | Choose from the FFT/batch log. Props on 5" frames often show strong 2nd/3rd harmonics. |
| `INS_HNTCH_OPTS` | `6` | Bit 1 = per-motor notches; bit 2 = update at loop rate. See below. |
| `INS_HNTCH_FM_RAT` | `1.0` (or `0.7`–`0.8` if minimum is clamping in descent) | Lets the notch go below `FREQ` at low throttle. |

**Choosing `INS_HNTCH_FREQ`:**

```
f_hover (Hz) = hover_RPM / 60
hover_RPM   ≈ KV × V_batt × hover_throttle × ~0.85   (rough estimate; better: read ESC RPM in a hover log)
```

For example, a 5" frame with 1950KV on 6S (22.2 V) at 25% hover gives about 9,200 RPM, which is about 153 Hz. Set `FREQ ≈ 120`, `BW ≈ 60`.

For a 10" frame with 400KV on 6S at 35% hover: about 2,600 RPM, which is about 44 Hz. Set `FREQ ≈ 35`, `BW ≈ 18`.

**`INS_HNTCH_OPTS` bitmask:**

| Bit | Value | Meaning | Recommendation on H743 |
|---|---|---|---|
| 0 | 1 | Double notch | Not needed with per-motor tracking. |
| 1 | 2 | Multi-source: one notch set per motor | **Yes.** The H7 has the CPU headroom. |
| 2 | 4 | Update frequency at loop rate | **Yes.** |
| 3 | 8 | Enable on all IMUs | Yes if you use multiple active IMUs. |
| 4 | 16 | Triple notch | Use only with single-source (non-per-motor). |
| 5 | 32 | Treat low/failed RPM as minimum frequency | Recommended as a failsafe; use `OPTS = 38` if supported by your firmware. |

Notch count per axis is roughly motors × harmonics. With 4 motors and 3 harmonics that is 12, which the H743 handles easily at 2 kHz sampling. For an octo with 3 harmonics, check `PM.Load` / `SCHED_DEBUG` and reduce harmonics if load exceeds about 70%.

---

## 3. Secondary Notch (Static Frame Resonance)

Use this only if the pre-filter FFT shows a **fixed-frequency** peak that does not move with throttle, such as arm resonance or GPS-mast flex.

| Parameter | Value |
|---|---|
| `INS_HNTC2_ENABLE` | `1` |
| `INS_HNTC2_MODE` | `0` (fixed) |
| `INS_HNTC2_FREQ` | Peak frequency from the log |
| `INS_HNTC2_BW` | ~`FREQ / 4` (narrow) |
| `INS_HNTC2_ATT` | `30`–`40` |
| `INS_HNTC2_HMNCS` | `1` |

Alternatively, set `INS_HNTC2_MODE = 4` (in-flight FFT) with `FFT_ENABLE = 1` to track a dynamic secondary source. That costs extra CPU but the H7 can handle it.

---

## 4. Gyro Sampling and Low-Pass Filters (H743-specific)

| Parameter | Recommended | Notes |
|---|---|---|
| `INS_GYRO_RATE` | `1` (2 kHz), or `2` (4 kHz) on high-RPM 5" builds | Fast sampling keeps harmonics below Nyquist. |
| `SCHED_LOOP_RATE` | `400` (default), up to `800`/`1000` on small racers | With 4.6+, consider `FSTRATE_ENABLE = 1` for the fast-rate thread. |
| `INS_GYRO_FILTER` | Start at `40`. After notching is verified: 5" around `60`–`80`, 10"+ around `40`–`60`. | Raise only after the notch is confirmed. |
| `INS_ACCEL_FILTER` | `10`–`20` | Leave conservative. |
| `ATC_RAT_*_FLTD` | ~`INS_GYRO_FILTER / 2` | D-term filter. |
| `ATC_RAT_*_FLTT` | ~`INS_GYRO_FILTER / 2` | Target filter. |

Raising `INS_GYRO_FILTER` is the payoff of good notching: it reduces phase lag, which allows higher PID gains.

---

## 5. Logging Setup for Verification

| Parameter | Value | Purpose |
|---|---|---|
| `INS_LOG_BAT_MASK` | `1` | Log the first IMU. |
| `INS_LOG_BAT_OPT` | `4` (pre- and post-filter at sensor rate), or `2` on older firmware | Lets you compare pre- vs post-filter spectra. |
| `LOG_BITMASK` | Include IMU_RAW and ESC | |

Disable batch logging after tuning, since it uses a lot of log bandwidth.

---

## 6. Tuning Procedure

1. **Baseline.** Hover for about 60 s with the notch **disabled**, logging pre-filter. In Mission Planner use *Ctrl+F → FFT → Batch Sampler* (or the Filter Review tool at firmware.ardupilot.org/Tools/FilterReview). Note the fundamental, the strong harmonics, and any fixed peaks.
2. **Configure** sections 2–4 using the measured hover frequency.
3. **Verify.** Fly a hover plus some throttle sweeps with pre/post logging. The post-filter spectrum should show:
   - Motor peaks reduced by 20 dB or more across the whole throttle range.
   - The notch tracking ESC RPM in descent. If it clamps, lower `FM_RAT` or `FREQ`.
4. **Adjust:**
   - Residual harmonic present: add it to `HMNCS`.
   - Peaks broadened by prop wash or bent props: increase `BW` (up to about `0.75 × FREQ`).
   - Fixed residual peak: add the secondary static notch.
5. **Open up the low-pass.** Raise `INS_GYRO_FILTER` in steps of about 10 Hz. Check motor temperatures and `RATE` / PID noise after each step.
6. **Retune.** Run QuickTune or AutoTune with the new filter set, because the reduced phase lag changes the optimal gains.

---

## 7. Failure Modes to Guard Against

- **RPM dropout:** set `OPTS` bit 5 (if available) so the notch holds at `FREQ` instead of collapsing. Watch `ESC.Err` in logs.
- **Wrong `SERVO_BLH_POLES`:** the notch sits at the wrong multiple of the true frequency. Cross-check against the FFT peak.
- **CPU overload with octo, 3 harmonics, and loop-rate updates:** monitor `PM.Load` and `PM.NLon`. Drop to `HMNCS = 3` if needed.
- **DMA conflicts on H7 when BDShot shares timers with other peripherals:** check the board hwdef or the `@SYS/timers.txt` / `dma.txt` output.

---

### Quick-Start Parameter Block (5" quad, 1950KV, 6S, est. 150 Hz hover)

```
MOT_PWM_TYPE      6
SERVO_BLH_BDMASK  15
SERVO_BLH_POLES   14
SERVO_DSHOT_ESC   1
INS_GYRO_RATE     2
INS_HNTCH_ENABLE  1
INS_HNTCH_MODE    3
INS_HNTCH_REF     1
INS_HNTCH_FREQ    120
INS_HNTCH_BW      60
INS_HNTCH_ATT     40
INS_HNTCH_HMNCS   7
INS_HNTCH_OPTS    6
INS_HNTCH_FM_RAT  1.0
INS_GYRO_FILTER   40    # raise to 60–80 after verification
INS_LOG_BAT_MASK  1
INS_LOG_BAT_OPT   4
```

Share a pre-filter batch log or FFT screenshot from this cycle, and I can refine `FREQ`, `BW`, the harmonics selection, and the low-pass target.