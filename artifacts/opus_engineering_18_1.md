# ArduPilot Harmonic Notch Configuration: STM32H743 with Bidirectional DShot RPM Feedback

**About "Cycle 18":** I'm reading this as your internal iteration or tuning-pass label. If it refers to something specific, such as a prior log set, frame build, or firmware branch, share the FFT and log data and I'll refine these values against it.

**About terminology:** ArduPilot has no separate "dynamic gyro notch" like Betaflight's. Both roles are covered by the two harmonic notch instances:
- `INS_HNTCH_*`: the primary, RPM-tracking notch.
- `INS_HNTC2_*`: the secondary notch, used here for frame or static resonances.

---

## 1. Enable Bidirectional DShot (RPM Source)

| Parameter | Value | Notes |
|---|---|---|
| `MOT_PWM_TYPE` | `6` (DShot600) or `5` (DShot300) | DShot600 is fine on H743. |
| `SERVO_BLH_AUTO` | `1` | Maps BLHeli outputs automatically. |
| `SERVO_BLH_BDMASK` | `15` (outputs 1–4), `63` for a hex, `255` for an octo | Enables bidirectional DShot on those outputs. |
| `SERVO_BLH_POLES` | `14` | Typical for 22xx/23xx motors. **Count the magnets** to confirm. |
| `SERVO_DSHOT_ESC` | Match your ESC firmware | BLHeli32, AM32 or Bluejay. Use the EDT-capable option on 4.5+ if your ESC supports it. |
| `SERVO_DSHOT_RATE` | `0`–`1` | Use the loop-rate multiple. Don't push it unnecessarily. |

**ESC firmware requirement:** BLHeli_S needs Bluejay for bidirectional DShot. BLHeli32 and AM32 support it natively.

**Board requirement:** Bidirectional DShot needs a board whose hwdef has DMA on the timer channels for those outputs. Most H743 boards (Matek, CubeOrange+, Kakute H7, etc.) do, but check your board's docs for which output groups support it.

**Check before going further:** Reboot, arm on the bench without props, and confirm `ESC[n].RPM` in the MAVLink inspector (or the `ESC` log message) reads sensibly on every motor. Also confirm the error rate is ≈0%.

---

## 2. Primary Harmonic Notch: RPM-Tracked

| Parameter | Value | Rationale |
|---|---|---|
| `INS_HNTCH_ENABLE` | `1` | Reboot afterwards so the remaining parameters appear. |
| `INS_HNTCH_MODE` | `3` | ESC telemetry (RPM) tracking. |
| `INS_HNTCH_REF` | `1` | Reference scaling for ESC mode. |
| `INS_HNTCH_OPTS` | `6` | Combines option 2 (one notch set per motor) and option 4 (update every loop). H743 has ample CPU for this. Add `8` (giving `14`) to apply it on all IMUs if your firmware doesn't already. |
| `INS_HNTCH_FREQ` | See the frequency-sizing formula below | Acts as the floor or minimum tracking frequency. |
| `INS_HNTCH_BW` | `FREQ / 2` | Standard starting width. |
| `INS_HNTCH_ATT` | `40` dB | Attenuation depth. |
| `INS_HNTCH_HMNCS` | `3` (1st + 2nd harmonics) | Use `7` to add the 3rd harmonic if the FFT shows it. On a quad with `7`, that's 4 motors × 3 harmonics = 12 notches per IMU, which is fine on H7. |
| `INS_HNTCH_FM_RAT` | `1.0` | Lower it to about `0.7` if the notch needs to follow motors below the floor during descents or at low throttle. |

**Frequency sizing:**

```
f_hover (Hz) = hover_RPM / 60
INS_HNTCH_FREQ ≈ 0.8 × f_hover    (floor below the hover fundamental)
```

Typical floors:

| Frame | Floor |
|---|---|
| 5" | ~80–120 Hz |
| 7" | ~60–80 Hz |
| 10" | ~40–50 Hz |
| 15"+ | ~25–35 Hz |

Get the real hover RPM from a log (`ESC.RPM` at hover) rather than estimating it.

---

## 3. Secondary Notch: Frame / Static Resonance (Optional)

Use this only if a post-filter FFT shows a fixed peak that doesn't track throttle, such as arm flex or a camera mount.

| Parameter | Value |
|---|---|
| `INS_HNTC2_ENABLE` | `1` |
| `INS_HNTC2_MODE` | `0` (fixed frequency) |
| `INS_HNTC2_FREQ` | Peak frequency from the FFT |
| `INS_HNTC2_BW` | ~`FREQ / 4` (narrow) |
| `INS_HNTC2_ATT` | `30–40` dB |
| `INS_HNTC2_HMNCS` | `1` |

---

## 4. Loop Rate and Low-Pass Filters

| Parameter | Value | Notes |
|---|---|---|
| `SCHED_LOOP_RATE` | `400` (large frames), `800`–`1000` (5" class) | H743 handles 1 kHz comfortably. |
| `INS_FAST_SAMPLE` | `1` (or all IMUs) | |
| `INS_GYRO_FILTER` | Raise from `20` to `60`–`100` once the notch is verified | This is the main payoff of RPM notching: less lag and higher achievable PIDs. |
| `ATC_RAT_*_FLTD` / `FLTT` | Re-evaluate after raising the gyro LPF | |

---

## 5. Logging for Verification

Use batch sampling to compare pre- and post-filter data:

```
INS_LOG_BAT_MASK = 1
INS_LOG_BAT_OPT  = 4     (pre + post filter; use 1 for sensor-rate if supported)
LOG_BITMASK      += Fast IMU
```

---

## 6. Procedure

1. **Baseline flight.** Fly with the notch disabled and a conservative gyro LPF (`20`). Hover for 60 s, then do some throttle sweeps.
2. **Analyze.** Open the log in the WebTools Filter Review or the MP FFT tool. Identify the motor fundamental and its harmonics.
3. **Configure.** Apply Sections 1–2 and set `FREQ` from the measured hover data.
4. **Verify with a second flight.** The post-filter FFT should show motor peaks attenuated by ≥20–30 dB across the whole throttle range.
5. **Raise the gyro LPF.** Step `INS_GYRO_FILTER` up in 20 Hz increments. After each step, check motor temperatures and `RATE` noise.
6. **Retune.** Run QuickTune or AutoTune with the new filtering.

---

## 7. Failure Modes to Watch

- **RPM telemetry dropouts.** The notch falls back to the `FREQ` floor. Watch the `ESC` error rate; anything above ~1% suggests a wiring, DMA or pole-count problem.
- **Wrong `SERVO_BLH_POLES`.** The notch tracks at a wrong multiple of the true frequency, which shows up clearly as offset FFT peaks.
- **Hot motors after raising the LPF.** Back the LPF off, or reduce D gain.
- **Notch too wide.** Wide notches add phase lag near the control bandwidth on large, slow frames. Narrow `BW` toward `FREQ/4` for 15"+ craft.

---

If you share your frame size, motor KV and pole count, prop size, battery cell count, and a `.bin` log, I can give you concrete values instead of these ranges.