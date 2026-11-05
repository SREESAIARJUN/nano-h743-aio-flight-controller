# Low-Noise IMU Power Architecture: TPS7A4700 Isolated Rail (Cycle 16)

> **Context note:** I don't have the outputs of Cycles 1–15, so this assumes a typical baseline: a 12–28 V platform bus, a MEMS IMU in the ADIS1647x / ADIS1650x class (3.3 V, ≤100 mA), and a requirement that switching noise and ground currents from the host not reach the IMU. Swap in your actual IMU and bus values where they differ.

---

## 1. Design Objectives

| Parameter | Target | Rationale |
|---|---|---|
| IMU supply voltage | 3.3 V ±2 % | Typical MEMS IMU VDD |
| Load current | ≤100 mA (IMU) + ≤20 mA (isolator, secondary side) | Includes headroom |
| Output noise (10 Hz–100 kHz) | <10 µVrms | Kept well below the IMU's internal regulator and ADC noise |
| Ripple at IMU pin (>100 kHz) | <100 µVpp | Avoids aliasing into the gyro/accel sense bandwidth |
| PSRR at the pre-regulator switching frequency | >50 dB from the LDO, >90 dB total with filters | Suppresses DC-DC residue |
| Galvanic isolation | Functional (or reinforced, if your system requires it) | Breaks ground loops and host digital return currents |

---

## 2. Top-Level Architecture

```
 Platform Bus (12–28 V)
        │
 [Input protection: TVS + reverse-polarity FET + EMI filter]
        │
 [Isolated DC-DC: SN6505B push-pull + transformer, or isolated module]
        │        ══════ ISOLATION BARRIER ══════
        │  (isolated ground: GND_IMU)
 [Post-rectifier: Schottky bridge + 22 µF bulk]
        │  ≈ 5.0–5.5 V
 [Stage-1 π filter: 10 µF – ferrite bead (≥600 Ω @100 MHz) – 10 µF]
        │  
 [TPS7A4700, ANY-OUT = 3.3 V]  ← primary noise/PSRR stage
        │
 [Stage-2 post filter: low-DCR ferrite + 10 µF ceramic + 100 nF at the IMU pin]
        │
     IMU VDD ──── IMU
        │
 Digital isolator (e.g., ISO7741 / ISO7763) for SPI, DR, and SYNC lines
        │        ══════ ISOLATION BARRIER ══════
     Host MCU / FPGA
```

**Optional pre-regulation:** If the isolated output is loosely regulated (SN6505B open-loop outputs can swing ±10 %), add a pre-LDO (e.g., TPS7A2001 or a second TPS7A4700) set to about 4.5 V. This keeps the TPS7A4700 headroom constant, so its PSRR stays predictable.

---

## 3. TPS7A4700 Configuration

### 3.1 Key Device Characteristics
- Input range: 3–36 V; output range 1.4–20.5 V; up to 1 A
- Output noise: about 4.2 µVrms (10 Hz–100 kHz)
- PSRR: about 82 dB at 100 Hz, and stays above roughly 55 dB toward 1 MHz
- Dropout: about 307 mV at 1 A, and much lower at 100 mA
- ANY-OUT pin-strapping sets the output without external resistors. Resistor-divider noise is therefore eliminated, and the reference noise is not gained up.

### 3.2 ANY-OUT Programming for 3.3 V
The output is V_OUT = 1.4 V + Σ(grounded ANY-OUT pins).

| Pin | Weight | State |
|---|---|---|
| 6P4V2 | 6.4 V | Open |
| 3P2V | 3.2 V | Open |
| 1P6V | 1.6 V | **GND** |
| 800mV | 0.8 V | Open |
| 400mV | 0.4 V | Open |
| 200mV | 0.2 V | **GND** |
| 100mV | 0.1 V | **GND** |

The result is 1.4 + 1.6 + 0.2 + 0.1 = **3.3 V**.

Alternative rails:
- 1.8 V VDDIO: ground the 400 mV pin.
- 5.0 V: ground the 3.2 V and 400 mV pins.

### 3.3 Passive Components

| Node | Value | Type | Notes |
|---|---|---|---|
| C_IN | 10 µF + 100 nF | X7R, 16 V+ | Place within 2 mm of the IN pins |
| C_OUT | 10 µF (≥ 4.7 µF effective after DC bias) | X7R, low ESR | Required for stability |
| C_NR/SS | 10 µF (per the datasheet noise test condition) | X7R or C0G stack | Sets the noise floor and soft-start time; a larger value lowers noise |
| C_BYP (feed-forward) | 10 nF | C0G | Improves noise and PSRR in the 10–100 kHz range |
| EN | Tie to IN via 100 kΩ, or drive from a sequencer | — | Optional RC delay for sequencing |
| SENSE | Kelvin-route to the IMU-side load point | — | Compensates for drop in the trace and post-filter |

*Verify the exact pin names (NR/SS, BYPASS) and recommended values against the current TPS7A47 datasheet revision for your package.*

### 3.4 Operating Point and Thermal Check
- Headroom: V_IN ≈ 4.5 V to V_OUT = 3.3 V gives 1.2 V. That is well above dropout and keeps the device out of the PSRR-degraded region near dropout (keep headroom above about 0.5–1 V).
- Dissipation: (4.5 − 3.3) × 0.12 A ≈ 0.14 W.
- Junction temperature rise: about 6 °C with the VQFN thermal pad soldered (θJA ≈ 39 °C/W), which is negligible.

---

## 4. Noise Budget

| Source | Contribution at the IMU pin | Method |
|---|---|---|
| DC-DC switching residue (~400 kHz, ~50 mVpp) | <1 µVpp | π filter (~40 dB) + LDO (~55 dB) + post filter (~20 dB) |
| TPS7A4700 intrinsic noise | ~4.2 µVrms | Datasheet value |
| Post-filter thermal noise | Negligible | — |
| **Total** | **≈ 5 µVrms** | Well under the 10 µVrms target |

**Impact on the IMU:** MEMS IMUs typically specify a PSRR on the order of 30–60 dB. At 5 µVrms supply noise, the induced bias noise should fall below the device's specified angle random walk and velocity random walk. Confirm this with an Allan-variance test (Section 7).

---

## 5. Isolation Strategy

1. **Power isolation:** The SN6505B transformer driver provides galvanic separation. Use spread-spectrum mode to diffuse switching harmonics, and choose a transformer whose isolation rating matches the requirement (e.g., Würth 760390015-class parts).
2. **Signal isolation:** Use a digital isolator for SPI (SCLK, MOSI, MISO, CS), DR/IRQ, and SYNC.
   - Choose a part with SPI-rate support (≥ 2 MHz) and low propagation skew.
   - The isolator's IMU-side VCC comes from the same clean 3.3 V rail, but through its own ferrite bead and 100 nF capacitor. This keeps the isolator's switching currents out of the IMU decoupling.
3. **Ground scheme:**
   - GND_IMU is a separate copper island.
   - It connects to chassis only at a single point, through an optional RC network (e.g., 1 MΩ ∥ 4.7 nF Y-cap) for ESD and EMI drain.
   - No host ground crosses the isolation gap.

---

## 6. Layout Rules

- Use a 4-layer board in the IMU zone: signal / solid GND_IMU / power / signal.
- Leave an isolation gap of at least 8 mm creepage under the transformer and isolator for reinforced isolation, or per your applicable standard.
- Keep the LDO within 20–30 mm of the IMU, but not directly under it. This limits thermal gradients that cause bias drift.
- Route NR/SS and BYPASS traces short and guarded by ground.
- Use a star return: LDO GND, C_OUT, and IMU GND meet at one via cluster.
- Place no switching nodes, clocks, or high-di/dt traces under the IMU footprint.
- Mount the IMU with mechanical isolation if vibration coupling through the board is an issue. Electrical and mechanical noise are often confused during validation.

---

## 7. Verification Plan

| Test | Method | Pass criteria |
|---|---|---|
| Output noise | LNA (e.g., 40–60 dB gain) + FFT analyzer, 10 Hz–100 kHz, battery-powered reference | <10 µVrms |
| PSRR | Line injector (e.g., Picotest J2120A) + VNA on the LDO input | Matches the budget within 6 dB |
| Switching residue | Scope with 50 Ω coax tip, 20 MHz BW-limit off | <100 µVpp |
| Load transient | 10–100 mA step | <5 mV deviation, no ringing |
| Isolation | Hi-pot per the isolation rating; leakage measurement | Per requirement |
| System-level | IMU static Allan variance, comparing (a) a battery + linear bench supply with (b) this design | ARW/VRW and bias instability within 5 % of the battery baseline |
| Thermal | Thermal camera; IMU bias drift over a temperature sweep | No LDO-induced gradient visible in the bias data |

---

## 8. Proposed Deltas to Carry Forward (Cycle 16 → 17)

1. Replace the resistor-programmed LDO with ANY-OUT pin-strapping to remove feedback-divider noise.
2. Add a pre-regulation stage if the isolated converter is unregulated.
3. Give the digital isolator's IMU-side supply its own filtered branch.
4. Make an Allan-variance A/B comparison the gating acceptance test, rather than rail noise alone.

If you share the specific IMU part, bus voltage, isolation rating, and the Cycle 15 baseline, I can tighten the component values and the noise budget to your actual system.