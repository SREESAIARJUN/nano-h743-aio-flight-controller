# Cycle 11: Low-Noise IMU Power Architecture Using the TPS7A4700

I don't have the earlier cycles, so this design states its assumptions explicitly. Swap in your actual IMU, bus voltage and clock plan where they differ. Datasheet figures below are from memory and should be checked against the current TI datasheet (SBVS204) before release.

---

## 1. Design Assumptions

| Parameter | Assumed value |
|---|---|
| Input bus | 12 V nominal (9–16 V), with automotive or aerospace-style transients |
| IMU | Tactical-grade MEMS, ADIS16495-class: 3.3 V, about 90 mA, with burst-current spikes |
| Other loads on the board | MCU/FPGA, radios and motor drivers, all considered noisy |
| Goals | IMU rail noise below 10 µVrms (10 Hz–100 kHz); no switching spur above 50 µVpk at the IMU pins; IMU ground return isolated from digital and power currents |

---

## 2. Architecture

```
 12V BUS
   │
 [TVS + reverse-pol FET + CM choke + π EMI filter]
   │
 [Sync buck, spread-spectrum OFF or fixed freq ≥2 MHz] ──► 4.5 V "pre-reg" rail
   │                        (shared pre-reg is acceptable; IMU gets its own branch)
   │
   ├──► MCU / digital LDO or buck  (noisy domain)
   │
   └──[FB1: 600 Ω @100 MHz, low-DCR]──[2nd-order LC: 2.2 µH + 22 µF ‖ 0.1 µF, damped]
            │
        ┌───▼──────────────────────────┐
        │  TPS7A4700 (IMU-dedicated)   │  VOUT = 3.3 V via ANY-OUT
        │  IN: 10 µF X7R               │  CNR/SS: 1 µF
        │  OUT: 10–47 µF X7R           │  CBYP (FB–OUT): 10 nF
        └───┬──────────────────────────┘
            │  (Kelvin SENSE at the IMU connector/pins)
        [Local π: 0.1 µF ─ FB2 (low DCR) ─ 10 µF ‖ 0.1 µF ‖ 1 nF]
            │
          IMU VDD  (VDDIO from the same rail, or via a separate 3.3 V LDO if the I/O domain differs)
            │
          AGND_IMU island ──► single star point to system GND near the LDO's GND
```

### Why use two stages instead of an LDO straight off 12 V?

- **Thermal limits.** Running 12 V to 3.3 V at 90 mA dissipates about 0.78 W. That is workable in the QFN with good copper, but it heats the board right next to the IMU, and the IMU's thermal drift is a bigger error source than rail noise.
- **Low dissipation from the pre-regulator.** With a 4.5 V pre-reg, the LDO dissipates (4.5 − 3.3) × 0.09 ≈ 0.11 W, which is negligible.
- **Enough headroom for good rejection.** 1.2 V of headroom sits well above the about 0.3 V dropout at 1 A. At 90 mA dropout is far lower, so PSRR stays near its datasheet curve rather than collapsing near dropout.

---

## 3. TPS7A4700 Configuration

### ANY-OUT programming

VOUT = 1.4 V plus the sum of the bit pins tied to GND. For 3.3 V, an extra 1.9 V is needed:

| Pin | Weight | Connection |
|---|---|---|
| 1P6V | 1.6 V | **GND** |
| 0P2V | 0.2 V | **GND** |
| 0P1V | 0.1 V | **GND** |
| 6P4V2, 6P4V1, 3P2V, 0P8V, 0P4V | — | Float |

Result: 1.4 + 1.6 + 0.2 + 0.1 = **3.3 V**.

**Why ANY-OUT instead of a resistor divider:** the internal resistor network is ratio-matched, so it adds no divider noise and gives better accuracy over temperature. It also removes two external resistors that could pick up coupling.

### Passive components

| Component | Value | Purpose |
|---|---|---|
| C_IN | 10 µF X7R 25 V, plus 0.1 µF | Input stability; this capacitor sees the LC filter's output impedance |
| C_OUT | 22 µF X7R (≥10 µF after DC-bias derating) | Stability and load-transient response for IMU bursts |
| C_NR/SS | 1 µF X7R/C0G-grade, low leakage | Filters the reference noise floor; sets a soft-start of roughly tens of ms |
| C_BYP (FB to OUT) | 10 nF | Feed-forward capacitor; lowers noise and improves PSRR in the 10 kHz–1 MHz band |
| EN | RC from the pre-reg, or an MCU GPIO through 10 kΩ | Sequencing: enable the IMU after its I/O domain is valid, or follow the IMU's power-up spec |

### Expected rail performance

| Parameter | Value |
|---|---|
| Output noise | About 4–5 µVrms (10 Hz–100 kHz) |
| PSRR at 100 Hz | About 80 dB |
| PSRR at 1–2 MHz | Roughly 40–55 dB (check the curve at 1.2 V headroom) |

---

## 4. Noise and Spur Budget at the Buck Frequency (example: 2.2 MHz)

| Stage | Attenuation | Ripple remaining |
|---|---|---|
| Buck output ripple | — | 10 mVpp |
| FB1 + LC (f₀ ≈ 23 kHz, 2nd order) | ≈ −70 dB (practically −50 dB, since capacitor ESL and FB1 saturation limit it) | about 30 µVpp |
| TPS7A4700 PSRR at 2.2 MHz | ≈ −45 dB | below 1 µVpp |
| Local π at the IMU | additional −20 dB | below 0.1 µVpp |

**Takeaway:** the switching spur is eliminated. The residual noise is set by the LDO's intrinsic noise of about 4 µVrms, plus whatever the layout couples in. In practice, layout is the dominant risk.

**Damping the LC filter:** add a series-R/C damping branch (e.g., 1 Ω + 47 µF) across C_IN. This prevents a resonant peak near f₀ from amplifying load-step energy and interacting with the LDO input.

---

## 5. Isolation Strategy

Three levels are available. Pick the one the system needs.

1. **Supply isolation (baseline, implemented above).**
   - The IMU has a dedicated LDO with its own filtered branch.
   - No other load draws from the IMU rail.

2. **Ground isolation (recommended).**
   - Use an AGND_IMU copper island under the IMU and the LDO output network.
   - Tie it to system ground at a single point, located at the LDO GND/thermal pad.
   - Route SPI over the island boundary. Keep the return path continuous underneath the traces; never route signals across a split.
   - Add 22–33 Ω series resistors on SCLK/MOSI/MISO/CS to slow edges and reduce injected current.

3. **Galvanic isolation (only if motor or chassis ground noise is severe).**
   - Use an ISO7741-class digital isolator on SPI/DR/SYNC.
   - Power the IMU domain from an isolated converter (e.g., an SN6505 + transformer feeding the TPS7A4700).
   - The extra post-filtering above becomes mandatory, because isolated converters are noisy.

**Mechanical considerations:** mount the IMU away from inductors to avoid magnetic coupling into the sensor. Treat the buck inductor as a vibration source and a magnetic-field source as well. Use a shielded inductor, oriented away from the IMU, placed at least 20 mm away.

---

## 6. Layout Rules

- **Feedback sensing.** Run the SENSE/FB connection as a Kelvin trace to the IMU-side capacitor. Keep it away from SW nodes and clock lines.
- **Noise-reduction capacitor.** Place C_NR and C_BYP directly at their pins with minimal loop area. This node is high impedance and noise-sensitive.
- **Thermal pad.** Stitch it with at least 9 vias to the ground island.
- **Plane separation.** Keep the buck SW node, its inductor and its input loop on the opposite side of the board, separated by an internal ground plane.
- **Guard ring.** Run a guard pour around the IMU rail, tied to AGND_IMU.
- **Capacitor derating.** Choose ceramic capacitors so that effective capacitance stays at or above 50% of nominal at operating DC bias.

---

## 7. Verification Plan for Cycle 11

| Test | Method | Pass criterion |
|---|---|---|
| Output noise | LNA (e.g., 40–60 dB gain) feeding an FFT analyzer, 10 Hz–100 kHz | ≤ 10 µVrms |
| Switching spur | Spectrum analyzer with a 50 Ω DC-block probe at the IMU pins | No spur > 50 µVpk at f_sw or its harmonics |
| PSRR | Inject 100 Hz–10 MHz with an injection transformer at the pre-reg | Within 6 dB of the datasheet curve at 1.2 V headroom |
| Load transient | Capture during an IMU burst read | ΔV < 5 mV, no ringing beyond 2 cycles |
| System-level | IMU static Allan variance with buck ON vs. bench linear supply | No measurable change in gyro ARW or bias instability |
| Thermal | IR camera at maximum ambient | LDO ΔT < 10 °C; no thermal gradient across the IMU |

---

## 8. Risks and Mitigations

- **LC filter resonance with load steps:** add the damping network and verify by measuring output impedance (Bode 100 or similar).
- **Shrinking headroom at low bus voltage:** the pre-reg holds 4.5 V regardless of bus voltage. Confirm the buck stays in regulation down to 9 V.
- **Ground noise from shared SPI returns:** mitigated by series resistors and the single-point star ground. Escalate to galvanic isolation if Allan-variance tests show contamination.
- **Supply availability:** the TPS7A4701 is the automotive/enhanced-product variant. Check footprint and pin compatibility if the 4700 lead time becomes a problem.

---

**Next-cycle candidates:**
- Spread-spectrum versus fixed-frequency buck trade study for EMI compliance.
- Precision-reference rail for an external ADC on the IMU's analog outputs, if applicable.