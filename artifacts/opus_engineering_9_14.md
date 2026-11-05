# Low-Noise IMU Power Supply Architecture: TPS7A4700 (Cycle 9)

> **Note:** I don't have access to the earlier cycles, so this is self-contained. Where it depends on prior decisions (bus voltage, IMU part, isolation class), I state the assumption so you can substitute your Cycle 1–8 values. Confirm all component values against the current TPS7A4700 datasheet (SBVS204) before release.

---

## 1. Design Assumptions

| Parameter | Assumed value | Notes |
|---|---|---|
| Upstream bus | 12 V nominal (9–36 V) | Vehicle or avionics bus with switching noise |
| IMU | Tactical-grade MEMS (ADIS1650x class) | VDD 3.3 V, ~45 mA typical, 3.0–3.6 V allowed |
| IMU I/O | 3.3 V SPI | Shared with host MCU |
| Noise target at IMU VDD | < 10 µVrms (10 Hz–100 kHz), < 5 µV spurious at fSW | Keeps supply-induced bias instability below the IMU noise floor |
| Isolation requirement | Tier A: noise/ground isolation (required). Tier B: galvanic isolation (optional) | |

---

## 2. Architecture Overview

```
 12V BUS
   │
 [Protection]   TVS (SMBJ36A) + reverse-polarity PFET + 2A PTC/eFuse
   │
 [EMI Filter]   CM choke + differential LC (fc ≈ 20 kHz)
   │
 [Pre-Reg]      Sync buck, 12V → 4.5V, fSW ≈ 2.2 MHz, spread-spectrum OFF
   │            (e.g., LMR33630 / TPS62130 / TPS62913)
   │
 [π-Filter]     10µF ─ FB(600Ω@100MHz, low DCR) / 2.2µH ─ 10µF
   │            fc ≈ 34 kHz, ~40 dB at 2.2 MHz
   │
 ┌─────────── Analog Ground Island ─────────────────────────────┐
 │ [TPS7A4700] VIN = 4.5 V → VOUT = 3.3 V (ANY-OUT)              │
 │   │                                                           │
 │   ├── RC/FB split ──► IMU VDD (analog/sensor core)            │
 │   └── FB split ─────► IMU VDDIO (or from separate digital LDO)│
 └───────────────────────────────────────────────────────────────┘
          │ Single-point (star) tie to system ground
          │
 [Optional Tier B] Isolated DC/DC (SN6505B + transformer) feeding the
                   π-filter, plus ISO7741 digital isolator on SPI/DR/RST
```

**Rationale**
- A buck pre-regulator drops most of the voltage efficiently. An LDO straight from 12 V would dissipate (12 − 3.3) × 45 mA ≈ 0.4 W. That is workable, but it heats the board right next to a temperature-sensitive IMU.
- 1.2 V of LDO headroom (4.5 → 3.3 V) keeps the TPS7A4700 well clear of dropout (~307 mV at 1 A, far less at 45 mA). This preserves PSRR, which degrades sharply near dropout.
- The 2.2 MHz switching frequency places buck ripple above the IMU's sensing bandwidth and above typical MEMS resonant frequencies (~20–30 kHz). Spread-spectrum is disabled so the spur stays at one predictable frequency rather than smearing toward baseband.

---

## 3. TPS7A4700 Stage Design

### 3.1 Output voltage: ANY-OUT configuration (no external divider)
VOUT = 1.4 V + Σ(grounded ANY-OUT pins)

| Target | Pins tied to GND | Sum |
|---|---|---|
| **3.3 V** | 1P6V, 200mV, 100mV | 1.4 + 1.6 + 0.2 + 0.1 = 3.3 V |
| 1.8 V (if a separate VDDIO rail is needed) | 400mV | 1.4 + 0.4 = 1.8 V |

Leave all other ANY-OUT pins floating. Using ANY-OUT instead of resistors avoids divider resistor noise and the tempco of external resistors. The internal network is ratiometric and low-noise.

### 3.2 Component values (starting point)

| Ref | Function | Value / type | Notes |
|---|---|---|---|
| CIN | Input bypass | 10 µF X7R, 16 V + 100 nF | Place at the IN pin |
| CNR | Noise-reduction / soft-start | 10 µF X7R (low-leakage) | Sets the reference filter corner and therefore the 10 Hz–1 kHz noise. Leakage here shifts VOUT, so avoid Y5V/Z5U. |
| CBYP | Bypass (feed-forward) | 10 nF C0G | Per the datasheet typical application |
| COUT | Output | 10 µF X7R + 1 µF + 100 nF C0G | Account for ceramic DC-bias derating: keep ≥ 4.7 µF effective |
| SENSE | Remote sense | Kelvin trace to the IMU-side cap | Do not tie at the LDO pin if the trace to the IMU exceeds ~2 cm |
| EN | Enable / UVLO | Divider from VIN, trips at ~4.0 V rising | Prevents the LDO running in dropout during buck startup |
| PG | Power-good | 100 kΩ pull-up to VOUT → MCU GPIO / IMU RST gate | Holds the IMU in reset until the rail is in regulation |

### 3.3 Thermal check
P = (4.5 − 3.3) × 0.05 A ≈ **60 mW**. With θJA ≈ 35 °C/W (VQFN-20 with thermal pad soldered to a ground pour), the temperature rise is about 2 °C. This is negligible, but still place the LDO ≥ 15 mm from the IMU and avoid a direct copper thermal path. Thermal gradients across the IMU drive bias drift more than electrical noise does at this level.

---

## 4. Noise Budget

| Source | Level at source | Attenuation | At IMU VDD |
|---|---|---|---|
| LDO intrinsic noise (10 Hz–100 kHz) | ~4.2 µVrms | n/a | **~4.2 µVrms** |
| Buck ripple @ 2.2 MHz | ~10 mVpp | π-filter ~40 dB + LDO PSRR ~45 dB @ MHz + output FB/caps ~10 dB | **~0.6 µVpp** |
| Bus low-frequency ripple (100 Hz–1 kHz) | ~50 mV | Buck loop ~40 dB + LDO PSRR ~80 dB | **< 0.1 µV** |
| Ground-bounce coupling (layout dependent) | n/a | Mitigated by star ground | Target < 2 µV |
| **Total (RSS)** | | | **≈ 5 µVrms, meeting the < 10 µVrms target** |

Note that LDO PSRR falls at MHz frequencies. The π-filter, not the LDO, does most of the work against switching ripple. Do not remove the π-filter on the assumption that the LDO alone will handle it.

---

## 5. Isolation Strategy

### Tier A: Noise and ground isolation (baseline)
1. **Analog ground island** under the LDO and IMU, joined to system ground at **one point** near the LDO GND pin.
2. **Rail splitting after the LDO:**
   - IMU VDD: 10 Ω + 10 µF RC (fc ≈ 1.6 kHz, ~0.45 V drop at 45 mA is too much). Use a **ferrite bead (≤ 0.1 Ω DCR)** plus 10 µF instead, or 1 Ω + 22 µF. Check the IMU's minimum VDD.
   - VDDIO: separate ferrite bead and 1 µF, so SPI edge currents don't modulate VDD.
3. **SPI series termination:** 33 Ω at the source on SCLK, MOSI, and CS to slow edges and reduce return-current injection into the analog ground.
4. **No digital traces crossing the ground-island split.** SPI enters at the star-point location.

### Tier B: Galvanic isolation (if Cycle requirements include ground-loop immunity or a high-noise host)
- **Power:** SN6505B push-pull driver plus a 1:1.5 transformer (e.g., Würth 750315371), rectified to ~5 V, feeding the π-filter and TPS7A4700. The LDO then also cleans up the transformer ripple (~420 kHz).
- **Signals:** ISO7741 (4-channel, 3+1 directional) or ISO7762F for SPI plus DRDY/RST. Check that the isolator's propagation delay (~11 ns) still meets the SPI timing at your clock rate.
- **Trade-off:** Adds about 100 mW of loss and a new switching source. Use it only if testing shows ground-loop-induced errors.

---

## 6. Layout Rules

1. Order of placement along the power path: buck → π-filter → LDO → split filters → IMU. Keep the path linear with no backtracking.
2. Place the buck in a separate board zone with its hot loop (VIN cap, switch node, GND) minimized. Never route the switch node near the analog island.
3. Put the CNR and CBYP capacitors directly at their pins, with their ground returns to the LDO GND pin, not to the general plane.
4. Use a 4-layer minimum: L2 is a solid ground plane, and the analog island is defined on L1/L3 copper, not by slotting L2. Slots in the reference plane create worse return-path problems than they solve.
5. Attach the thermal pad to a ground pour with vias, and keep that pour within the analog island.
6. Mount the IMU mechanically away from the inductors, both for magnetic coupling and because inductor vibration can couple acoustically into MEMS sensors.

---

## 7. Verification Plan (Cycle 9 Exit Criteria)

| Test | Method | Pass criterion |
|---|---|---|
| Output noise | LNA (×100, e.g., 10 Hz–1 MHz) + spectrum analyzer at the IMU VDD pin | < 10 µVrms (10 Hz–100 kHz) |
| Switching spur | Spectrum analyzer at 2.2 MHz and harmonics | < 5 µV |
| PSRR | Inject ripple on the 12 V bus (Picotest J2120A or equivalent) | Meets the budget in §4 |
| Load transient | 0 → 50 mA step | < 10 mV deviation, no ringing |
| Startup sequencing | Scope VIN, VOUT, PG, IMU RST | IMU released only after PG is high |
| **System-level** | Allan variance of the IMU with the bench linear supply vs. the designed supply | No measurable change in angle random walk or bias instability. **This is the real acceptance test.** |
| Thermal | IR camera at full operating temperature | LDO hotspot does not create a gradient across the IMU |

---

## 8. Risks and Open Items for Cycle 10

- **Ceramic capacitor microphonics:** X7R capacitors on the IMU rail can convert vibration into voltage noise. If vibration testing shows supply-correlated noise, switch to C0G or a polymer tantalum on the final output stage.
- **IMU-specific PSRR:** Obtain the IMU vendor's supply-sensitivity data (e.g., °/hr per mV) to turn the µV budget into a sensor error budget.
- **Tier B decision:** Make the galvanic-isolation call based on the system-level Allan variance results.
- **Cost/area:** The TPS7A4700 is overkill for 45 mA. If board space is tight, the TPS7A20 (lower current, small package, ~7 µVrms) could be evaluated as an alternative. Retaining the TPS7A4700 gives margin for adding a magnetometer or barometer on the same rail.

If you share the actual IMU part number, bus specification, and the outcomes of Cycles 1–8, I can tighten the values and noise budget to match.