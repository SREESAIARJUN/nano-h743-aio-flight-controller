# ArduPilot Gyro Filtering on STM32H743 with Bidirectional DShot RPM Feedback

I don't recognize "Cycle 17" as an ArduPilot term. I've treated it as your internal iteration label. If it refers to something specific, such as a firmware version or a tuning stage, say so and I'll adjust.

These settings target ArduCopter 4.4 and later, where ESC-telemetry harmonic notches are mature. Some option bits vary by version, so check the parameter descriptions in your GCS.

---

## 1. Prerequisite: Bidirectional DShot RPM Telemetry

| Parameter | Value | Notes |
|---|---|---|
| `MOT_PWM_TYPE` | `6` (DShot600) or `5` (DShot300) | DShot300 is more robust with long ESC leads. |
| `SERVO_BLH_BDMASK` | `15` | Bitmask of outputs with bidirectional DShot. `15` covers outputs 1–4; use `255` for 1–8. |
| `SERVO_BLH_POLES` | `14` | Count the motor magnets. Typically 14 for 5–7" motors, often 12 for small motors. A wrong value scales every notch frequency incorrectly. |
| `SERVO_BLH_AUTO` | `1` | Optional. Auto-maps BLHeli outputs. |
| `SERVO_DSHOT_ESC` | Per ESC firmware | Use `1` for BLHeli32/AM32. Set to match your firmware. |
| `SERVO_DSHOT_RATE` | `1` or `2` | DShot output rate as a multiple of loop rate. |

**H743 notes:**
- Bidirectional DShot needs DMA-capable timer channels. Most H743 boards support it in standard builds. Older targets may need the `-bdshot` firmware variant.
- Confirm RPM appears per ESC: check `esc1_rpm` through `esc4_rpm` in the MAVLink Inspector, or the ESC messages in logs, with motors spinning.

---

## 2. IMU Sampling and Loop Rate (H743 headroom)

| Parameter | Value | Notes |
|---|---|---|
| `INS_GYRO_RATE` | `1` (2 kHz) or `2` (4 kHz) | `0`=1 kHz, `1`=2 kHz, `2`=4 kHz, `3`=8 kHz. The H743 handles 2–4 kHz comfortably. |
| `INS_FAST_SAMPLE` | `7` or IMU mask | Enables fast sampling on the IMUs. |
| `SCHED_LOOP_RATE` | `400` (5"+ / larger), `800`–`1000` (small/race) | Higher loop rates also let the notch track faster. |
| `INS_GYRO_FILTER` | `80`–`120` Hz | Static low-pass. Raise it after the notch is working; start around 80. |
| `INS_ACCEL_FILTER` | `20` Hz | Default is fine. |

---

## 3. Primary Harmonic Notch: ESC RPM-Tracking (`INS_HNTCH_*`)

| Parameter | Value | Notes |
|---|---|---|
| `INS_HNTCH_ENABLE` | `1` | Reboot afterwards to reveal the remaining parameters. |
| `INS_HNTCH_MODE` | `3` | ESC telemetry (bidirectional DShot RPM). |
| `INS_HNTCH_REF` | `1` | Scaling reference. Must be 1 for ESC-telemetry mode. |
| `INS_HNTCH_FREQ` | ≈ 0.8–0.9 × hover motor frequency | In mode 3 this acts as the lower frequency floor. Examples: 5" ≈ 80 Hz, 7" ≈ 60 Hz, 10"+ ≈ 40–50 Hz. |
| `INS_HNTCH_FM_RAT` | `1.0` (default) | Lets the notch track down to `FREQ × FM_RAT`. Lower it, e.g. to `0.6`, if the motors idle well below `FREQ`. |
| `INS_HNTCH_BW` | ≈ `FREQ / 2` | Bandwidth of the fundamental. Harmonics scale with it. |
| `INS_HNTCH_ATT` | `40` dB | Typical attenuation. |
| `INS_HNTCH_HMNCS` | `7` (1st, 2nd and 3rd harmonics) | Bitmask: 1=1st, 2=2nd, 4=3rd, 8=4th. Many props show strong 1st and 3rd, so `5` is also common. Choose from FFT data. |
| `INS_HNTCH_OPTS` | `2 + 4 + 8 = 14` | Bit 1 (2): one notch per motor. Bit 2 (4): update at loop rate. Bit 3 (8): apply to all IMUs. Add bit 0 (1) for a double notch if residual noise is broad. |

**Why one notch per motor (`OPTS` bit 1):** each motor gets its own notch set, centered on that motor's real RPM. This handles asymmetric loading during yaw and in wind.

**CPU cost:** filters scale as notches × harmonics × motors × IMUs. For example, 4 motors × 3 harmonics × 2 IMUs (× 2 for a double notch) is fine on an H743. After enabling, confirm `PM.Load` / `PM.NLon` stays healthy.

---

## 4. Secondary Notch (`INS_HNTC2_*`)

Use it for noise the RPM notch doesn't cover, such as frame resonance or the blade-pass frequency on large props.

**Option A: fixed frame resonance**

| Parameter | Value |
|---|---|
| `INS_HNTC2_ENABLE` | `1` |
| `INS_HNTC2_MODE` | `0` (fixed) |
| `INS_HNTC2_FREQ` | Resonant peak from FFT, e.g. `180` Hz |
| `INS_HNTC2_BW` | ~`FREQ / 4` |
| `INS_HNTC2_ATT` | `30`–`40` dB |
| `INS_HNTC2_HMNCS` | `1` |

**Option B: in-flight FFT as a backup tracker**
- Set `FFT_ENABLE=1` and `INS_HNTC2_MODE=4`.
- This also gives a fallback if ESC telemetry drops out.

---

## 5. Verification Workflow

1. **Batch log before and after filtering:**
   - `INS_LOG_BAT_MASK = 1` (first IMU)
   - `INS_LOG_BAT_OPT = 4` (pre- and post-filter capture; check the bit meaning on your version)
   - `LOG_BITMASK`: make sure IMU_RAW / FAST_IMU logging is enabled.
2. **Fly** a 60–90 s hover plus some throttle sweeps.
3. **Analyze** with the ArduPilot online Filter Review Tool:
   - Identify the fundamental and harmonic peaks in the pre-filter spectrum.
   - Confirm post-filter peaks are reduced by ≥20–30 dB.
   - Check the estimated phase lag at your crossover frequency.
4. **Iterate:**
   - Adjust `HMNCS`, `BW` and the double-notch bit based on what the spectrum shows.
   - Then raise `INS_GYRO_FILTER` if noise allows.
   - Then rerun AutoTune.
5. **Check RPM integrity in logs:** the ESC RPM and `ESCX` error-rate fields should be near 0% CRC errors. High error rates make the notch jump around.

---

## 6. Starting Template: 5" Quad on an H743

```
MOT_PWM_TYPE      6
SERVO_BLH_BDMASK  15
SERVO_BLH_POLES   14
INS_GYRO_RATE     1
SCHED_LOOP_RATE   800
INS_GYRO_FILTER   90
INS_HNTCH_ENABLE  1
INS_HNTCH_MODE    3
INS_HNTCH_REF     1
INS_HNTCH_FREQ    80
INS_HNTCH_BW      40
INS_HNTCH_ATT     40
INS_HNTCH_HMNCS   7
INS_HNTCH_OPTS    14
INS_HNTCH_FM_RAT  0.8
INS_LOG_BAT_MASK  1
INS_LOG_BAT_OPT   4
```

For a different airframe (prop size, motor KV, or cell count), share it along with a pre-filter FFT, and I can refine `FREQ`, `BW` and `HMNCS` for that specific "Cycle 17" iteration.