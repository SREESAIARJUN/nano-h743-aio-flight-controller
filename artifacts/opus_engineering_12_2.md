# PCB Trace Impedance and Skin-Effect AC Resistance at 48 kHz: 2 oz Motor Phase Tracks

I don't have the "Cycle 12" design data (trace widths, lengths, stackup, current levels). The formulation below is general, and the worked example uses placeholder geometry that you should replace with your values.

**Assumed example geometry:**
- **Copper:** 2 oz, t = 70 µm, ρ₂₀ = 1.72×10⁻⁸ Ω·m, α = 0.00393 /°C
- **Trace:** w = 10 mm, ℓ = 50 mm
- **Dielectric:** FR-4, εr = 4.3, h = 1.6 mm to the reference plane (microstrip)
- **Operating temperature:** 20 °C and 100 °C

---

## 1. DC Resistance

$$R_{DC} = \frac{\rho(T)\,\ell}{w\,t}, \qquad \rho(T) = \rho_{20}\,[1+\alpha(T-20)]$$

| T | ρ (Ω·m) | R_DC |
|---|---|---|
| 20 °C | 1.72×10⁻⁸ | **1.23 mΩ** |
| 100 °C | 2.26×10⁻⁸ | **1.62 mΩ** |

---

## 2. Skin Depth at 48 kHz

$$\delta = \sqrt{\frac{\rho}{\pi f \mu_0}}$$

- **At 20 °C:** δ = √(1.72×10⁻⁸ / (π · 48 000 · 4π×10⁻⁷)) ≈ **301 µm**
- **At 100 °C:** δ ≈ **345 µm**

**Key ratio:** Δ = t/δ = 70/301 ≈ **0.23**. The copper is about 4× thinner than one skin depth.

---

## 3. AC Resistance (Dowell, single layer)

$$F_R = \frac{R_{AC}}{R_{DC}} = \Delta\,\frac{\sinh 2\Delta + \sin 2\Delta}{\cosh 2\Delta - \cos 2\Delta} \;\approx\; 1 + \frac{4}{45}\Delta^4 \quad (\Delta \ll 1)$$

$$F_R \approx 1 + 0.089 \times 0.0029 \approx \mathbf{1.0003}$$

**Result:** R_AC(48 kHz) ≈ R_DC. Skin effect through the thickness is negligible.

**Harmonic check.** The PWM ripple is triangular, so it contains harmonics:

| Frequency | δ | Δ | F_R (approx.) |
|---|---|---|---|
| 48 kHz | 301 µm | 0.23 | 1.000 |
| 144 kHz (3rd) | 174 µm | 0.40 | 1.002 |
| 240 kHz (5th) | 135 µm | 0.52 | 1.006 |
| 1 MHz | 66 µm | 1.06 | ≈1.3 |

The ripple-weighted effective F_R stays below about 1.01.

**Edge/proximity crowding.** With a close return plane, current crowds toward the plane-facing surface and the trace edges. This adds a few percent at most at these frequencies. If you need it precisely, use a 2D field solver.

---

## 4. Characteristic Impedance (wide microstrip, Hammerstad–Wheeler)

The usual IPC-2141 formula is invalid for w/h > ~2, so use the wide-strip form.

**Effective permittivity:**

$$\varepsilon_{eff} = \frac{\varepsilon_r+1}{2} + \frac{\varepsilon_r-1}{2}\left(1+\frac{12h}{w}\right)^{-1/2} = 2.65 + 1.65(2.92)^{-1/2} \approx 3.62$$

**Impedance:**

$$Z_0 = \frac{120\pi}{\sqrt{\varepsilon_{eff}}\left[\frac{w}{h}+1.393+0.667\ln\left(\frac{w}{h}+1.444\right)\right]} = \frac{377}{1.902 \times 9.00} \approx \mathbf{22\ \Omega}$$

### Is Z₀ relevant at 48 kHz?

$$\lambda = \frac{c}{f\sqrt{\varepsilon_{eff}}} \approx 3.3\text{ km}$$

The 50 mm trace is electrically tiny, so treat it as a lumped RL element.

| Parameter | Formula | Per metre | 50 mm trace |
|---|---|---|---|
| L | Z₀√ε_eff / c | 140 nH/m | **≈7 nH** |
| C | √ε_eff / (Z₀c) | 288 pF/m | **≈14 pF** |

**Reactance at 48 kHz:** X_L = 2π f L ≈ **2.1 mΩ**.

This is the important finding: the trace's inductive reactance exceeds its resistance (1.2–1.6 mΩ). The phase-path impedance at 48 kHz is

$$|Z| = \sqrt{R^2 + X_L^2} \approx 2.6\text{ mΩ}$$

and loop inductance, not skin effect, is the parameter to minimise. Do this with a tight return path and adjacent planes.

Z₀ only matters for switching edges. For example, t_r = 20 ns gives f_knee ≈ 0.35/t_r ≈ 17.5 MHz. Even then, a 50 mm trace (~0.3 ns delay) is still well under t_r/6.

---

## 5. Loss Estimate

The motor phase current is mostly at the electrical fundamental (DC to a few hundred Hz), with 48 kHz ripple superimposed:

$$P = I_{fund,rms}^2\,R_{DC} + \sum_n I_{ripple,n}^2\,R_{DC}F_{R,n}$$

**Example:** 30 A_rms phase current plus 3 A_rms ripple, at 100 °C:

$$P \approx (900 + 9 \times 1.01) \times 1.62\text{ mΩ} \approx \mathbf{1.47\ W}$$

The ripple contributes about 1 % of this. Size the trace thermally using IPC-2152 on R_DC at operating temperature.

---

## Summary

| Quantity | Value (example) |
|---|---|
| δ @ 48 kHz | 301 µm (20 °C) / 345 µm (100 °C) |
| t/δ (2 oz) | 0.23 |
| R_AC/R_DC | ≈1.000 (≤1.01 including harmonics) |
| R_DC (10 mm × 50 mm) | 1.23 / 1.62 mΩ |
| Z₀ | ≈22 Ω (not governing at 48 kHz) |
| L, X_L @ 48 kHz | 7 nH, 2.1 mΩ, which dominates |

**Design takeaways:**
- **2 oz copper:** skin effect is irrelevant at 48 kHz, so there is no AC derating.
- **Widening and paralleling layers** reduces both R and L.
- **Loop inductance is the dominant term:** minimise it, as it drives switching ringing and EMI.

If you send the actual Cycle 12 trace dimensions, stackup, and current profile, I'll rerun these numbers.