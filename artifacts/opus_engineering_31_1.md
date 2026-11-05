# ArduPilot Harmonic Notch Configuration: STM32H743 with Bidirectional DShot RPM Feedback

I don't recognize "Cycle 31" as a standard ArduPilot term. It may be your own iteration label, a firmware build, or a project milestone. If it implies specific constraints, such as a particular firmware version or frame, tell me and I'll adjust. The configuration below targets ArduCopter 4.4–4.6 on H743 boards.

---

## 1. Prerequisites: Bidirectional DShot Telemetry

The ESCs must report eRPM over the signal wire. That requires BLHeli_32, Bluejay, or AM32 firmware.

| Parameter | Value | Purpose |
|---|---|---|
| `MOT_PWM_TYPE` | `6` (DShot600) or `5` (DShot300) | Digital protocol. DShot300 is more robust on long leads. |
| `SERVO_BLH_MASK` | e.g. `15` (outputs 1–4) | Enables BLHeli passthrough/telemetry on the motor outputs |
| `SERVO_BLH_BDMASK` | `15` (match motor outputs) | Enables bidirectional DShot (RPM feedback) |
| `SERVO_BLH_POLES` | `14` (typical 5" motor; count the magnets) | Converts eRPM to mechanical RPM |
| `SERVO_BLH_AUTO` | `1` | Auto-enables BLHeli on multicopter motor outputs |
| `SERVO_DSHOT_RATE` | `0`–`2` | DShot output rate multiple. Keep it at or below the loop rate. |

**Notes:**
- Many H743 targets need the `-bdshot` firmware variant, or a board whose motor outputs sit on DMA-capable timers.
- Bidirectional DShot works per timer group. Keep all motors in BDShot-capable groups.
- **Verify before going further:** In Mission Planner, check `ESC1_RPM` through `ESC4_RPM` in the status tab. Spin the motors with the props off (Motor Test). Every ESC should report sane RPM and an error rate below about 1%.

---

## 2. Loop Rate and Gyro Sampling (H743 Headroom)

| Parameter | Value | Rationale |
|---|---|---|
| `SCHED_LOOP_RATE` | `400` (default). Use `800`–`1000` for small, agile frames. | The H743 has the CPU headroom for higher rates. |
| `INS_FAST_SAMPLE` | `1` (or all IMUs) | Samples the gyro at the backend rate, typically 2–8 kHz |
| `INS_GYRO_RATE` | `0`–`2` | Backend gyro rate multiplier (1/2/4/8 kHz) |
| `INS_GYRO_FILTER` | see table in §4 | Static low-pass filter applied after the notches |

---

## 3. Primary Harmonic Notch: ESC-RPM Tracking (`INS_HNTCH_*`)

### Governing relationships

Motor fundamental frequency:

$$f_1 = \frac{\text{eRPM}}{(P/2)\cdot 60}\ \text{Hz}, \quad P = \text{pole count}$$

Notch centers:

$$f_k = k \cdot f_1, \quad k \in \text{HMNCS harmonics}$$

When the telemetry rate is high, the bandwidth tracks the centre frequency proportionally:

$$BW_{eff} = BW \cdot \frac{f_1}{\text{INS\_HNTCH\_FREQ}}$$

### Parameter set

| Parameter | Value | Explanation |
|---|---|---|
| `INS_HNTCH_ENABLE` | `1` | Enable, then **reboot** to expose the other parameters |
| `INS_HNTCH_MODE` | `3` | ESC telemetry (RPM) tracking |
| `INS_HNTCH_REF` | `1` | Scaling reference. Must be 1 for ESC mode. |
| `INS_HNTCH_FREQ` | ≈ 0.8 × lowest in-flight f₁ (see §4) | Floor frequency. The notch never tracks below this. |
| `INS_HNTCH_BW` | ≈ `INS_HNTCH_FREQ / 2` | -3 dB bandwidth at the floor frequency |
| `INS_HNTCH_ATT` | `40` dB | Notch depth |
| `INS_HNTCH_HMNCS` | `7` (1st + 2nd + 3rd) | Bitmask. Use `3` for 1st + 2nd only, if the FFT shows weak 3rd-harmonic content. |
| `INS_HNTCH_OPTS` | `6` (= 2 + 4) | Bit 1 = one notch per motor (multi-source). Bit 2 = update notch coefficients at loop rate. |
| `INS_HNTCH_FM_RAT` | `1.0` | Throttle-based floor ratio. Only relevant at idle or low throttle. |

**OPTS bit reference:**
- `1`: double notch
- `2`: multi-source (per motor)
- `4`: update at loop rate
- `8`: enable on all IMUs
- `16`: triple notch
- `32`: treat low frequency as minimum

On an H743, setting `OPTS = 6` (adding `+8` if you run multiple IMUs) is affordable:

> 4 motors × 3 harmonics = **12 notches per axis per IMU**

That stays comfortably within the CPU budget at 400–1000 Hz. Avoid stacking double or triple notches on top of per-motor tracking unless an FFT proves you need them, because that multiplies the filter count.

---

## 4. Frame-Size Starting Values

| Frame | Typical hover f₁ | `INS_HNTCH_FREQ` | `INS_HNTCH_BW` | `INS_GYRO_FILTER` | `SCHED_LOOP_RATE` |
|---|---|---|---|---|---|
| 3" | 250–300 Hz | 150 | 75 | 100 | 800–1000 |
| 5" | 150–200 Hz | 100 | 50 | 80–100 | 800 |
| 7" | 100–130 Hz | 70 | 35 | 60–80 | 400–800 |
| 10" | 60–80 Hz | 45 | 22 | 40–50 | 400 |
| 15"+ | 30–50 Hz | 25 | 12 | 20–30 | 400 |

**Rule of thumb:** set `FREQ` below the lowest RPM you will see in flight (idle plus descent). Avoid setting it so low that the notch sits on top of control bandwidth, which is roughly below 1.5 × the rate-loop crossover.

---

## 5. Secondary Notch (`INS_HNTC2_*`): Optional

Use this for non-motor resonances: frame arms, a camera gimbal, or prop-wash humps.

| Parameter | Value |
|---|---|
| `INS_HNTC2_ENABLE` | `1` |
| `INS_HNTC2_MODE` | `0` (fixed) for structural modes, or `4` (in-flight FFT) |
| `INS_HNTC2_FREQ` | The resonant peak identified in the FFT |
| `INS_HNTC2_BW` | 25–50% of `FREQ` |
| `INS_HNTC2_ATT` | `30`–`40` |
| `INS_HNTC2_HMNCS` | `1` |

If you use mode 4 (FFT), also set `FFT_ENABLE = 1`, along with `FFT_MINHZ` and `FFT_MAXHZ` to bracket the target. The H743 handles FFT well.

---

## 6. Validation Procedure

1. **Baseline (pre-filter) log:**
   - Set `INS_LOG_BAT_MASK = 1` and `INS_LOG_BAT_OPT = 4` (pre- and post-filter).
   - Set `LOG_BITMASK` to include IMU raw data. Also set `INS_RAW_LOG_OPT` on 4.5+ if needed.
   - Hover for 60 s or more, with some throttle sweeps.
2. **Analyse:** Open the log in the ArduPilot Filter Review Tool (firmware.ardupilot.org/Tools/FilterReview) or Mission Planner's FFT view.
   - Confirm the motor peaks track `ESCx_RPM / 60`.
   - Confirm the harmonics are present at 2× and 3×.
3. **Apply the filters**, then fly the same profile again.
4. **Acceptance criteria:**
   - Post-filter motor peaks should be attenuated by at least 20–30 dB.
   - Noise floor below about 0.1 (deg/s)²/Hz in the 50–500 Hz band (frame-dependent).
   - Motor temperatures should not rise after tuning.
5. **Tune:** Raise `INS_GYRO_FILTER` in steps of about 10 Hz while the residual noise allows. Then re-run AutoTune or manual PIDs, since the reduced phase lag usually permits higher D gain.

---

## 7. Failure Modes to Guard Against

- **Telemetry dropout:** If ESC RPM is lost, the notch drops to `INS_HNTCH_FREQ`. Keep that floor sensible so the aircraft remains flyable.
- **Wrong pole count:** Every notch lands at the wrong frequency. Check `ESC_RPM` against the FFT peaks.
- **Over-wide notches:** These cause phase lag and sluggish or oscillatory attitude response. Narrow `BW` before you reduce `ATT`.
- **Mixed BDShot timer groups:** Some motors report zero RPM. Remap the outputs or use the `-bdshot` firmware build.

If you share your frame size, motor KV, cell count, prop size, and a pre-filter log FFT, I can turn these starting values into specific numbers.