# PCB Trace Impedance and Skin-Effect AC Resistance at 48 kHz for 2 oz Motor Phase Tracks

I don't have the specific "Cycle 5" geometry (trace widths, lengths, stack-up, phase current), so below is a general formulation with a worked example. The example uses representative values. Substitute your own numbers into the formulas.

---

## 1. Baseline Parameters

| Parameter | Symbol | Value used |
|---|---|---|
| Copper thickness (2 oz/ft²) | t | 70 µm (≈68.6 µm nominal; use ~60 µm if etched and plated thinner) |
| Copper resistivity @ 20 °C | ρ₂₀ | 1.72 × 10⁻⁸ Ω·m |
| Temperature coefficient | α | 0.00393 /°C |
| Permeability | μ₀ | 4π × 10⁻⁷ H/m |
| Frequency (PWM) | f | 48 kHz |
| Example trace width | w | 10 mm |
| Example trace length | L | 50 mm |
| Dielectric height (2-layer FR-4) | h | 1.6 mm |
| Relative permittivity | εr | 4.5 |

---

## 2. Skin Depth

$$\delta = \sqrt{\frac{\rho}{\pi f \mu_0}}$$

- **At 20 °C:** π·48000·4π×10⁻⁷ = 0.1895, so δ = √(1.72×10⁻⁸ / 0.1895) ≈ **0.301 mm (301 µm)**
- **At 100 °C:** ρ = 1.72×10⁻⁸ × (1 + 0.00393 × 80) = 2.26×10⁻⁸ Ω·m, so δ ≈ **0.345 mm**

**Key ratio:** Δ = t/δ = 70 / 301 ≈ **0.23**.

The copper is far thinner than one skin depth, so current penetrates the full thickness at 48 kHz.

---

## 3. DC Resistance

$$R_{DC} = \frac{\rho L}{w\,t} \quad;\quad R_{\square} = \frac{\rho}{t}$$

- **Sheet resistance:** 1.72×10⁻⁸ / 70×10⁻⁶ = **0.246 mΩ/□** at 20 °C, or **0.323 mΩ/□** at 100 °C
- **Example trace** (50 mm / 10 mm = 5 squares):
  - R_DC(20 °C) = 5 × 0.246 ≈ **1.23 mΩ**
  - R_DC(100 °C) ≈ **1.62 mΩ**

---

## 4. AC Resistance (Skin Effect Through Thickness)

Use the Dowell / foil-conductor factor:

$$F_R = \frac{R_{AC}}{R_{DC}} = \Delta\,\frac{\sinh 2\Delta + \sin 2\Delta}{\cosh 2\Delta - \cos 2\Delta}$$

For Δ < 1, this simplifies to:

$$F_R \approx 1 + \frac{4}{45}\Delta^4$$

With Δ = 0.23: Δ⁴ = 0.00293, so **F_R ≈ 1.0003**.

**Result:** R_AC(48 kHz) ≈ R_DC. Thickness skin effect is negligible (<0.1 %) for 2 oz copper at 48 kHz.

### Effects That Do Matter at 48 kHz

1. **Lateral current crowding (width).** Because w ≫ δ, the current in a microstrip over a return plane concentrates toward the edges and the plane-facing surface. For wide traces this can add roughly 5–20 % to R_AC. Quantify it with a 2D field solver (FEMM, Q3D, Sonnet), or apply a 1.1–1.2 margin.
2. **Proximity effect.** This applies between adjacent phase traces, or between a trace and its return path on the opposite layer. Keep phase-to-phase spacing ≥ 2δ (about 0.6 mm), or interleave the return path deliberately.
3. **PWM harmonics.** The phase current ripple contains harmonics at n × 48 kHz:

| Frequency | δ | t/δ | F_R (approx.) |
|---|---|---|---|
| 48 kHz | 301 µm | 0.23 | 1.000 |
| 480 kHz | 95 µm | 0.74 | ~1.03 |
| 1 MHz | 66 µm | 1.06 | ~1.12 |
| 17.5 MHz (20 ns edge, f_knee = 0.35/t_r) | 15.8 µm | 4.4 | ~2.2 |

Because ripple amplitudes are small, harmonic loss is usually negligible. However, higher R at MHz frequencies helps damp switching-node ringing.

### Combined Loss Estimate

$$P = I_{RMS,fund}^2 R_{DC} F_{R}(f_1) + \sum_n I_{n}^2 R_{DC} F_R(n f_1)$$

---

## 5. Characteristic Impedance (Microstrip, Hammerstad–Wheeler, w/h ≥ 1)

**Effective permittivity:**

$$\varepsilon_{eff} = \frac{\varepsilon_r+1}{2} + \frac{\varepsilon_r-1}{2}\left(1+\frac{12h}{w}\right)^{-1/2}$$

With w/h = 6.25: ε_eff = 2.75 + 1.75 × (2.92)^(−0.5) ≈ **3.77**

**Characteristic impedance:**

$$Z_0 = \frac{120\pi}{\sqrt{\varepsilon_{eff}}\left[\frac{w}{h} + 1.393 + 0.667\ln\left(\frac{w}{h}+1.444\right)\right]}$$

Z₀ = 377 / (1.943 × 9.00) ≈ **21.5 Ω**

For thin traces with w/h < 1, use the IPC-2141 form instead:

$$Z_0 = \frac{87}{\sqrt{\varepsilon_r + 1.41}} \ln\left(\frac{5.98h}{0.8w+t}\right)$$

### Interpretation

At 48 kHz the wavelength is:

$$\lambda = \frac{c}{f\sqrt{\varepsilon_{eff}}} \approx 3.2\ \text{km}$$

The trace is therefore electrically short by about 5 orders of magnitude, and Z₀ has no transmission-line significance at the PWM frequency. It only matters for edge-rate content (tens of MHz) when assessing ringing and EMI. At 48 kHz the trace should be modeled as a **lumped R–L** element.

---

## 6. Lumped Inductance and Total Impedance at 48 kHz

**Inductance per unit length:**

$$L' = \frac{Z_0\sqrt{\varepsilon_{eff}}}{c} = \frac{21.5 \times 1.943}{3\times10^8} \approx 0.14\ \text{nH/mm}$$

- **Example 50 mm trace:** L ≈ 7 nH
- **Reactance:** X_L = 2π · 48 000 · 7 nH ≈ **2.1 mΩ**

**Total impedance:**

$$|Z| = \sqrt{R_{AC}^2 + X_L^2} = \sqrt{1.62^2 + 2.1^2} \approx \mathbf{2.65\ m\Omega}\ \ (\text{at }100\,^\circ\text{C})$$

Two observations:

- The inductive reactance at 48 kHz is comparable to or larger than the resistance. For phase balancing and current-sense accuracy, match **trace inductance** across phases, not just resistance.
- To reduce L, place a solid return plane directly beneath (smaller h), or route the phase traces over a dedicated ground pour.

---

## 7. Summary Template (fill in per phase)

| Quantity | Formula | Example value |
|---|---|---|
| δ @ 48 kHz, T_op | √(ρ_T / πfμ₀) | 0.345 mm |
| t/δ | — | 0.20 |
| R_DC @ T_op | ρ_T L / (w t) | 1.62 mΩ |
| F_R (thickness) | 1 + 4Δ⁴/45 | 1.000 |
| F_edge (crowding, solver or margin) | — | 1.1–1.2 |
| R_AC | R_DC · F_R · F_edge | ~1.8–1.95 mΩ |
| Z₀ (for EMI / ringing only) | Hammerstad | 21.5 Ω |
| L_trace | Z₀√ε_eff · L / c | 7 nH |
| \|Z\| @ 48 kHz | √(R² + X²) | ~2.7–2.9 mΩ |
| P_loss | I²_RMS · R_AC | e.g., 30 A_RMS → ~1.7 W |

**Cross-check:** Verify the temperature rise against IPC-2152 using your phase RMS current. For example, 30 A on a 10 mm 2 oz external trace runs roughly 20–30 °C above ambient, depending on copper pour and planes. Iterate ρ_T until the temperature and resistance values are consistent.

If you share the actual Cycle 5 widths, lengths, stack-up, and RMS phase current, I can recompute these numbers precisely.