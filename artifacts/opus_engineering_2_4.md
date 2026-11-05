# PCB Trace Impedance and Skin-Effect AC Resistance at 48 kHz
### 2 oz Copper Motor Phase Tracks

> **Note on "Cycle 2":** I don't have your Cycle 2 design data (trace widths, lengths, stack-up, phase current). The derivation below is fully parametric. Worked numbers use the assumptions in §0, so substitute your own values where they differ.

---

## 0. Assumptions

| Parameter | Symbol | Value |
|---|---|---|
| Copper weight | 2 oz/ft² | t = 70 µm (nominal) |
| Copper resistivity, 20 °C | ρ₂₀ | 1.72 × 10⁻⁸ Ω·m |
| Temperature coefficient | α | 0.00393 /°C |
| Operating copper temperature | T | 100 °C (hot-trace design case) |
| PWM / switching frequency | f_sw | 48 kHz |
| Trace width (example) | w | 5 mm |
| Trace length (example) | ℓ | 50 mm |
| Dielectric | FR-4 | εr ≈ 4.3 |
| Height to reference plane | h | 0.2 mm (4-layer) **or** 1.5 mm (2-layer) |

---

## 1. DC Resistance

$$R_{DC} = \frac{\rho(T)\,\ell}{w\,t}, \qquad \rho(T) = \rho_{20}\,[1+\alpha(T-20)]$$

The resistivity at the two temperatures is:
- ρ₂₀ = 1.72 × 10⁻⁸ Ω·m
- ρ₁₀₀ = 1.72 × 10⁻⁸ × 1.314 = 2.26 × 10⁻⁸ Ω·m

This gives a sheet resistance for 2 oz copper of:
- R□ = ρ/t = **0.246 mΩ/□ at 20 °C**
- R□ = **0.323 mΩ/□ at 100 °C**

**Example: w = 5 mm, ℓ = 50 mm (10 squares)**
- R_DC = 2.46 mΩ at 20 °C
- R_DC = **3.23 mΩ at 100 °C**

---

## 2. Skin Depth

$$\delta = \sqrt{\frac{\rho}{\pi f \mu_0}}, \qquad \mu_0 = 4\pi\times10^{-7}\ \text{H/m}$$

At 48 kHz:
- At 20 °C: δ = √(1.72×10⁻⁸ / 0.1895) = **0.301 mm**
- At 100 °C: δ = **0.345 mm**

**Key ratio:** Δ = t/δ = 0.070 / 0.301 = **0.23**

The copper is only about a quarter of one skin depth thick, so current fills the full cross-section at 48 kHz.

---

## 3. AC Resistance (Dowell Foil Model)

For a flat conductor of thickness t, the single-layer resistance factor is:

$$F_R = \frac{R_{AC}}{R_{DC}} = \Delta\,\frac{\sinh 2\Delta + \sin 2\Delta}{\cosh 2\Delta - \cos 2\Delta} \;\approx\; 1 + \frac{\Delta^4}{45}\quad(\Delta \ll 1)$$

Phase current contains PWM ripple at f_sw plus its harmonics, so evaluate across the spectrum:

| Frequency | δ (20 °C) | Δ = t/δ | F_R | R_AC (50 mm, 100 °C) |
|---|---|---|---|---|
| 48 kHz (f_sw) | 301 µm | 0.23 | **1.00007** | 3.23 mΩ |
| 96 kHz (2f_sw, center-aligned ripple) | 213 µm | 0.33 | 1.0003 | 3.23 mΩ |
| 240 kHz (5th) | 135 µm | 0.52 | 1.0016 | 3.24 mΩ |
| 1 MHz (edge content) | 66 µm | 1.06 | ≈1.11 | 3.58 mΩ |
| 10 MHz (dv/dt ringing) | 21 µm | 3.4 | ≈3.4 | ~11 mΩ |

**Conclusion:** At 48 kHz, skin effect in 2 oz copper is negligible (<0.01 %). AC resistance only matters above about 1 MHz, where the energy content of the phase current is small.

### Second-order effects to check

1. **Proximity / edge crowding.** Over a solid return plane, current crowds toward the plane-facing surface and the trace edges. With Δ < 0.5 this adds less than about 1–2 % at 48 kHz.

2. **Paralleled layers.** If the phase track is duplicated on multiple layers stitched with vias, use Dowell's multilayer term. Here m is the number of layers carrying the same current:

$$F_R \approx 1 + \frac{(5m^2-1)\Delta^4}{45}$$

   For m = 4 and Δ = 0.23, F_R ≈ 1.0055, which is still negligible.

3. **Via resistance.** Via resistance often dominates over trace R_AC. Each plated via with 0.3 mm drill, 25 µm plating and 1.6 mm length has R ≈ 1.0 mΩ, so use many vias in parallel.

---

## 4. Trace Impedance (Microstrip)

### Is Z₀ relevant at 48 kHz?

At 48 kHz the wavelength is about 3 km, so a 50 mm trace is electrically short. Z₀ itself does not matter for the power path. What matters is the **loop inductance** derived from Z₀, plus Z₀ for high-frequency ringing analysis (tens of MHz).

### Hammerstad–Wheeler model (w/h ≥ 1)

Effective permittivity:

$$\varepsilon_{eff} = \frac{\varepsilon_r+1}{2} + \frac{\varepsilon_r-1}{2}\frac{1}{\sqrt{1+12h/w}}$$

Characteristic impedance:

$$Z_0 = \frac{120\pi}{\sqrt{\varepsilon_{eff}}\left[\frac{w}{h} + 1.393 + 0.667\ln\!\left(\frac{w}{h}+1.444\right)\right]}$$

Per-unit-length inductance and capacitance:

$$L' = \frac{Z_0\sqrt{\varepsilon_{eff}}}{c}, \qquad C' = \frac{\sqrt{\varepsilon_{eff}}}{Z_0\,c}$$

As a sanity check, the parallel-plate approximation for a wide trace is L' ≈ μ₀h/w.

### Results (w = 5 mm)

| Stack-up | w/h | ε_eff | Z₀ | L′ | L (50 mm) | X_L at 48 kHz |
|---|---|---|---|---|---|---|
| 4-layer, h = 0.2 mm | 25 | 4.01 | **6.6 Ω** | 44 nH/m | 2.2 nH | 0.66 mΩ |
| 2-layer, h = 1.5 mm | 3.33 | 3.42 | **35.3 Ω** | 218 nH/m | 10.9 nH | 3.3 mΩ |

### Total phase-track impedance at 48 kHz (50 mm, 100 °C)

$$|Z| = \sqrt{R_{AC}^2 + (\omega L)^2}$$

- **4-layer:** √(3.23² + 0.66²) ≈ **3.3 mΩ**. Resistive-dominated.
- **2-layer:** √(3.23² + 3.3²) ≈ **4.6 mΩ**. Inductance is comparable to resistance, so use a tight return plane.

---

## 5. Power Loss and Design Check

Because F_R ≈ 1 at 48 kHz, the loss can use the total RMS current (fundamental plus ripple):

$$P = I_{RMS}^2\,R_{DC}(T)\,F_R \;\approx\; I_{RMS}^2\,R_{DC}(T)$$

**Example:** With I_RMS = 30 A, P = 900 × 3.23 mΩ = **2.9 W per 50 mm phase track**.

Verify the resulting temperature rise with **IPC-2152** charts, using copper weight, width, board thickness and plane presence. Then iterate T in ρ(T), since resistance and temperature are coupled.

---

## 6. Summary

| Quantity | Formula | Value (5 mm × 50 mm, 2 oz) |
|---|---|---|
| Skin depth at 48 kHz | √(ρ/πfμ₀) | 0.30–0.35 mm |
| t/δ | — | 0.23 |
| R_AC / R_DC | Dowell | ≈ 1.0001 |
| R_DC (100 °C) | ρℓ/wt | 3.23 mΩ |
| Z₀ | Hammerstad–Wheeler | 6.6 Ω (4L) / 35 Ω (2L) |
| Loop L | Z₀√ε_eff / c | 2.2 nH (4L) / 10.9 nH (2L) |

**Design guidance:**
- Widen or parallel the 2 oz layers to reduce DC resistance. Thicker copper (3–4 oz) still has t/δ < 0.5 at 48 kHz, so it is fully effective.
- Minimize h to the return plane to cut inductance and ringing.
- Budget via resistance explicitly.

Send me your Cycle 2 widths, lengths, stack-up and RMS current, and I'll recompute these values for your design.