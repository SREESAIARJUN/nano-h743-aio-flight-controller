# ArduPilot Harmonic Notch Configuration: STM32H743 with Bidirectional DShot RPM

I don't know what "Cycle 19" refers to; it may be a project or iteration label. The configuration below uses ArduPilot 4.4 through 4.5 parameter semantics. Confirm parameter names and option bits against the parameter list for your firmware version.

---

## 1. Prerequisites: Get Clean RPM Telemetry First

The notch can only be as good as the RPM data driving it.

| Parameter | Value | Notes |
|---|---|---|
| `MOT_PWM_TYPE` | `6` (DShot600) | Use `5` (DShot300) on large or slow ESCs. Bidirectional DShot600 is comfortable on H743. |
| `SERVO_BLH_AUTO` | `1` | Auto-maps motor outputs. |
| `SERVO_BLH_BDMASK` | e.g. `15` (outputs 1–4) | Bitmask of outputs with bidirectional DShot. Use `255` for 8 motors. |
| `SERVO_BLH_POLES` | `14` | Motor magnet count (typical for 22xx–28xx motors). Count the magnets, not the stator teeth. |
| `SERVO_DSHOT_ESC` | per ESC (`1` BLHeli32 / `3` AM32 / `2` BLHeli_S-Bluejay) | Affects command handling. |
| `SERVO_DSHOT_RATE` | `0`, or `1` at 800 Hz loop | Output rate tied to loop rate. |

**Required ESC firmware:** BLHeli32 ≥ 32.7, Bluejay, or AM32, with bidirectional (EDT/RPM) support.

**Check before tuning:**
- Spin the motors (props off) and confirm `ESC[n].RPM` in the logs or on the GCS.
- Error rate should be below about 1%.
- The values should look physically plausible:

```
f_fundamental (Hz) = mechanical_RPM / 60
mechanical_RPM     = eRPM / (POLES / 2)
```

If RPM reads about 7× too high or low, `SERVO_BLH_POLES` is wrong.

---

## 2. Sensor and Loop Rate (H743 Headroom)

| Parameter | Value | Rationale |
|---|---|---|
| `INS_GYRO_RATE` | `1` (2 kHz) or `2` (4 kHz) | Gives the notches resolution well above motor harmonics. |
| `SCHED_LOOP_RATE` | `400` (≥7"), `800` (3–5") | H743 handles 800 Hz with per-motor notches. |
| `INS_FAST_SAMPLE` | `1` (or `7` for all IMUs) | Fast sampling on the primary IMU. |
| `INS_GYRO_FILTER` | 5": `80–110`; 7–10": `50–60`; >13": `20–40` | Gyro low-pass. Raise it once notches clean up the noise. |
| `INS_ACCEL_FILTER` | `20` | Default is fine. |

---

## 3. Primary Harmonic Notch: ESC RPM Tracking

| Parameter | Value | Explanation |
|---|---|---|
| `INS_HNTCH_ENABLE` | `1` | Reboot afterward so the rest of the parameters appear. |
| `INS_HNTCH_MODE` | `3` | ESC telemetry (bidirectional DShot) frequency source. |
| `INS_HNTCH_REF` | `1` | Scaling reference; must be 1 for RPM-based modes. |
| `INS_HNTCH_FREQ` | ≈ 0.8 × hover motor frequency, never below ~40 Hz (5") / ~20 Hz (large) | Floor frequency, used when RPM is low or invalid. |
| `INS_HNTCH_BW` | ≈ `INS_HNTCH_FREQ / 2` | Bandwidth at the base frequency; scales with tracked frequency. |
| `INS_HNTCH_ATT` | `40` (dB) | Attenuation depth. Use 30–50. |
| `INS_HNTCH_HMNCS` | `3` (1st+2nd), `7` (1st–3rd) | Harmonics bitmask. 2-blade props: usually `3`. 3-blade or tri-blade: add the 3rd. |
| `INS_HNTCH_FM_RAT` | `1.0` | Minimum-frequency ratio. Lower it (e.g. `0.7`) only if low-throttle tracking is needed. |
| `INS_HNTCH_OPTS` | `6` | Bit 1 = one notch per motor; bit 2 = update at loop rate. Add bit 3 (`+8` → `14`) to apply on all IMUs if the firmware doesn't default it. Add bit 5 TreatLowAsMin (`+32`) if the notch collapses at idle. |

### Option-bit rationale on H743
- **Per-motor tracking (bit 1)** places an independent notch set on each motor's actual RPM. This is far tighter than a single averaged notch, and it costs `motors × harmonics` filters per axis.
- **Loop-rate update (bit 2)** removes lag in fast throttle changes.
- **Double or triple notch (bits 0/4)** is generally not needed with per-motor RPM tracking. Reserve it for throttle or FFT modes where frequency estimation is imprecise.

### Filter count check
The firmware limits the total number of notch filters.

```
filters = motors × harmonics × IMUs × (notches per harmonic)
```

For example, a quad with 1st+2nd harmonics on 2 IMUs: 4 × 2 × 2 = 16. That is fine on H743. An octo with 3 harmonics and 3 IMUs is 72 filters, which may exceed the limit or loop budget. In that case, drop the IMU count or the harmonics.

---

## 4. Secondary Notch (INS_HNTC2_*): Static or Backup

Use this for frame or arm resonance that doesn't track RPM, or as a throttle-based fallback if telemetry drops.

**Option A: fixed frame resonance**

| Parameter | Value |
|---|---|
| `INS_HNTC2_ENABLE` | `1` |
| `INS_HNTC2_MODE` | `0` (fixed) |
| `INS_HNTC2_FREQ` | measured resonance peak |
| `INS_HNTC2_BW` | ~`FREQ/4` to `FREQ/2` |
| `INS_HNTC2_ATT` | `30–40` |
| `INS_HNTC2_HMNCS` | `1` |

**Option B: throttle-based backup**

Set `MODE=1` and `REF=MOT_THST_HOVER`. Base `FREQ`/`BW` on the hover peak.

---

## 5. Worked Examples

Estimate a starting `FREQ` with:

```
f_hover = (KV × V_batt × throttle_hover × ~0.85 loaded) / 60
```

Better, read hover RPM directly from the logs.

| Airframe | Hover RPM | f_hover | `HNTCH_FREQ` | `HNTCH_BW` | `HMNCS` | `GYRO_FILTER` | Loop |
|---|---|---|---|---|---|---|---|
| 5" 6S, 1950 KV | ~10,800 | 180 Hz | 140 | 70 | 3 | 100 | 800 |
| 7" 6S, 1300 KV | ~7,200 | 120 Hz | 95 | 48 | 3 | 70 | 800/400 |
| 10" 6S, 700 KV | ~4,800 | 80 Hz | 60 | 30 | 3 | 50 | 400 |
| 15" 12S, 400 KV | ~3,000 | 50 Hz | 40 | 20 | 7 | 30 | 400 |

---

## 6. Verification Workflow

1. **Logging before any filter changes:**
   - `INS_LOG_BAT_MASK = 1` (first IMU)
   - `INS_LOG_BAT_OPT = 4` (pre- and post-filter capture; use `2` = post-filter only on older firmware)
   - `INS_LOG_BAT_CNT = 1024` (sample batch size)
2. **Hover flight** of 30–60 s, plus a few throttle sweeps.
3. **Analyze** with the ArduPilot Filter Review Tool (web-based) or the MAVExplorer FFT:
   - Pre-filter: identify motor peaks and confirm they line up with `ESC.RPM / 60` and its multiples.
   - Post-filter: peaks should be reduced by roughly 20–40 dB, and the noise floor should be flat.
4. **Iterate:**
   - If residual peaks remain at a harmonic, add it to `HMNCS`.
   - If a peak is too wide, increase `BW` or `ATT`.
   - If you see lag or oscillation at low throttle, raise `FREQ` or enable `TreatLowAsMin`.
5. **Raise `INS_GYRO_FILTER`** progressively once the spectrum is clean. Fewer low-pass phase losses means better tune headroom.
6. **Run Autotune** after filtering is finalized, since notch changes alter the phase margin.

---

## 7. Failure Modes to Guard Against

- **Telemetry dropout:** with per-motor mode, the notch falls back to `INS_HNTCH_FREQ`. Keep that value sensible, and consider the HNTC2 throttle backup.
- **ESC error rate above about 5%:** shorten the signal wires, check for DShot timer or DMA conflicts in the hwdef, or drop to DShot300.
- **Wrong pole count:** every notch lands at the wrong frequency. Always validate this against the FFT.
- **CPU overload:** check `PM.NLon` and `PM.MaxT` in the logs. If the loop is overrunning, reduce harmonics, IMUs, or the loop rate.

If you share your frame size, motor KV, cell count, prop blades, and a pre-filter log, I can narrow these down to exact values.