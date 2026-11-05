# PCB Phase-Track Analysis: 2 oz Copper at 48 kHz PWM

I don't have the specific "Cycle 13" design data (track geometry, stack-up, phase current). The formulation below is fully parameterized, with a worked example using typical values that you can replace with your actual numbers.

**Assumed example parameters**

| Parameter | Symbol | Value |
|---|---|---|
| Copper weight | – | 2 oz → t = 70 µm |
| Track width | w | 5.0 mm |
| Track length | ℓ | 50 mm |
| Dielectric height to reference plane | h | 0.20 mm |
| Dielectric constant (FR-4) | εr | 4.4 |
| Copper resistivity at 20 °C | ρ₂₀ | 1.72 × 10⁻⁸ Ω·m |
| Temperature coefficient | α | 0.00393 /°C |
| Operating copper temperature | T | 100 °C |
| Phase current | I_rms | 30 A |

---

## 1. DC Resistance (Baseline)

$$R_{DC} = \frac{\rho_T\,\ell}{w\,t}, \qquad \rho_T = \rho_{20}\left[1+\alpha(T-20)\right]$$

- ρ₁₀₀ = 1.72×10⁻⁸ × (1 + 0.00393×80) = **2.26 × 10⁻⁸ Ω·m**
- R_DC(20 °C) = (1.72×10⁻⁸ × 0.05)/(5×10⁻³ × 70×10⁻⁶) = **2.46 mΩ**
- R_DC(100 °C) = **3.23 mΩ**

---

## 2. Skin Depth

$$\delta = \sqrt{\frac{\rho}{\pi f \mu_0}}$$

| Frequency | δ (20 °C) | δ (100 °C) | t/δ (100 °C) |
|---|---|---|---|
| 48 kHz (fundamental) | 301 µm | 345 µm | 0.20 |
| 480 kHz (10th harmonic) | 95 µm | 109 µm | 0.64 |
| 1 MHz | 66 µm | 76 µm | 0.92 |
| 5 MHz (switching edges) | 29.5 µm | 34 µm | 2.06 |

**Key result:** At 48 kHz the skin depth is about 4–5× the copper thickness. Current penetrates the full 70 µm, so thickness-wise skin effect is negligible at the fundamental.

---

## 3. AC Resistance

For a microstrip over a plane, the return current pulls the signal current toward the plane-facing surface, which is effectively a one-sided field. Using the Dowell-type form with Δ = t/δ:

$$\frac{R_{AC}}{R_{DC}} = F(\Delta) = \Delta\,\frac{\sinh 2\Delta + \sin 2\Delta}{\cosh 2\Delta - \cos 2\Delta}$$

For small Δ this simplifies to:

$$F(\Delta) \approx 1 + \frac{4}{45}\Delta^4$$

For large Δ (Δ ≫ 1), F ≈ Δ.

**Results (100 °C):**

| f | Δ | F | R_AC |
|---|---|---|---|
| 48 kHz | 0.20 | 1.0001 | **3.23 mΩ** |
| 480 kHz | 0.64 | ≈1.015 | 3.28 mΩ |
| 1 MHz | 0.92 | ≈1.06 | 3.42 mΩ |
| 5 MHz | 2.06 | ≈2.1 | ≈6.8 mΩ |

**Width-wise current crowding (edge/proximity effect).** On wide tracks over a plane, HF current concentrates at the track edges and under the track. This is a larger effect than thickness skin effect, and it is not captured by F(Δ). As a rule of thumb, add a factor of about 1.1–1.3 above roughly 1 MHz for w/h > 10. Use a 2D field solver (e.g., FEMM, Q2D, or Saturn PCB) if harmonic losses matter.

**Total loss with harmonics:**

$$P = I_{1,rms}^2 R_{AC}(f_1) + \sum_n I_{n,rms}^2 R_{AC}(f_n)$$

In a motor drive the phase current is mostly the fundamental motor current plus 48 kHz ripple. Harmonic currents are small, so the DC term dominates.

**Example loss:** P ≈ 30² × 3.23 mΩ ≈ **2.9 W per track**. This is high for a 5 mm × 50 mm track. Check the temperature rise with IPC-2152 and consider:
- widening the track,
- paralleling layers with via stitching,
- adding a solder-mask opening plus a tinned or bus-bar overlay.

---

## 4. Characteristic Impedance (Microstrip)

Use the Hammerstad wide-strip form (w/h ≥ 1).

**Effective permittivity:**

$$\varepsilon_{eff} = \frac{\varepsilon_r+1}{2} + \frac{\varepsilon_r-1}{2}\left(1+\frac{12h}{w}\right)^{-1/2}$$

**Impedance:**

$$Z_0 = \frac{120\pi}{\sqrt{\varepsilon_{eff}}\left[\dfrac{w}{h} + 1.393 + 0.667\ln\!\left(\dfrac{w}{h}+1.444\right)\right]}$$

**Worked example (w/h = 25):**
- ε_eff = 2.7 + 1.7 × (1.48)^−0.5 = **4.10**, so √ε_eff = 2.024
- Denominator term = 25 + 1.393 + 0.667 ln(26.44) = 28.58
- **Z₀ ≈ 376.99 / (2.024 × 28.58) ≈ 6.5 Ω**

The thickness correction (w → w + Δw) lowers this by about 1–2%, which is negligible here.

---

## 5. Is Z₀ Relevant at 48 kHz?

**Propagation delay:**

$$t_{pd} = \frac{\sqrt{\varepsilon_{eff}}}{c} = 6.75\text{ ns/m}$$

**Wavelength at 48 kHz:**

$$\lambda = \frac{c}{f\sqrt{\varepsilon_{eff}}} \approx 3.1\text{ km}$$

The trace is electrically tiny at the fundamental.

**Edge-rate check.** Assume the switch-node rise time is t_r ≈ 20 ns, giving a knee frequency of about 17.5 MHz. The critical length is:

$$\ell_{crit} \approx \frac{t_r}{6\,t_{pd}} \approx 0.49\text{ m}$$

**Conclusion:** Treat the phase track as a **lumped R–L element**, not a transmission line. Z₀ matters only for EMI/ringing modelling of the switch-node edges.

---

## 6. Loop Inductance (the Parameter That Actually Matters)

**With a solid reference plane:**

$$L' = \frac{Z_0\sqrt{\varepsilon_{eff}}}{c} = \frac{6.5 \times 2.024}{3\times10^8} \approx 44\text{ nH/m} \;\Rightarrow\; \mathbf{2.2\text{ nH}} \text{ for 50 mm}$$

**Without a plane** (partial self-inductance, ℓ and w + t in cm, result in nH):

$$L = 2\ell\left[\ln\frac{2\ell}{w+t} + 0.5 + 0.2235\frac{w+t}{\ell}\right]$$

This gives L ≈ **35 nH**, about 16× worse. Keep a solid plane under the phase tracks, or route the phase and return as adjacent-layer pairs.

**Impedance at 48 kHz:**

$$X_L = 2\pi f L = 2\pi \times 48\text{k} \times 2.2\text{ nH} = 0.66\text{ mΩ}$$

This is comparable to but smaller than R_AC. The total track impedance is therefore:

$$|Z| = \sqrt{R_{AC}^2 + X_L^2} \approx \mathbf{3.3\text{ mΩ}}$$

---

## Summary

| Quantity | Value (example) | Note |
|---|---|---|
| δ at 48 kHz (100 °C) | 345 µm | About 5× the copper thickness |
| R_AC / R_DC at 48 kHz | 1.0001 | Skin effect negligible |
| R_AC at 48 kHz | 3.23 mΩ | Temperature dominates, not frequency |
| Z₀ | ≈ 6.5 Ω | Irrelevant at 48 kHz; trace is lumped |
| L (over plane) | 2.2 nH | Key parameter for switching/EMI |
| \|Z\| at 48 kHz | ≈ 3.3 mΩ | |
| I²R loss at 30 A | ≈ 2.9 W | Verify thermal rise via IPC-2152 |

**Design takeaway:** For 2 oz copper at 48 kHz, size the tracks on DC resistance plus temperature rise. Control the loop inductance with a solid reference plane, and check HF edge-crowding losses only if switching-edge harmonics carry significant current.

If you share the actual Cycle 13 parameters (track width, length, stack-up, and current), I can rerun the numbers.