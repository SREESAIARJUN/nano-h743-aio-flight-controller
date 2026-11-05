# PCB Motor Phase Trace: Impedance and Skin-Effect AC Resistance at 48 kHz (2 oz Cu)

I don't have your Cycle 18 design data (trace widths, lengths, stack-up, current levels). The formulation below is general, and the worked numbers use stated example values. Substitute your own values where marked.

---

## 1. Inputs and Assumptions

| Parameter | Symbol | Value used |
|---|---|---|
| Copper weight | – | 2 oz/ft² |
| Copper thickness | t | 70 µm (nominal; plated outer layers are often 80–90 µm) |
| Resistivity at 20 °C | ρ₂₀ | 1.72 × 10⁻⁸ Ω·m |
| Temperature coefficient | α | 0.00393 /K |
| Frequency | f | 48 kHz (PWM fundamental) |
| Example trace width | w | 10 mm ← *substitute* |
| Example trace length | ℓ | 50 mm ← *substitute* |
| Dielectric height to plane | h | 0.2 mm, εr = 4.3 ← *substitute* |

**Important framing:** the motor phase current is mostly at the *electrical* frequency, typically DC to a few hundred Hz. The 48 kHz content is the **PWM ripple** riding on that current. Evaluate losses separately for:

- the low-frequency current (use R_DC), and
- the ripple and its harmonics (use R_AC(f)).

---

## 2. Skin Depth

$$\delta = \sqrt{\frac{\rho}{\pi f \mu_0}}$$

- At 20 °C: δ = √(1.72×10⁻⁸ / (π · 48000 · 4π×10⁻⁷)) ≈ **301 µm**
- At 100 °C: ρ = ρ₂₀[1 + α(T − 20)] ≈ 2.26×10⁻⁸ Ω·m, giving δ ≈ **345 µm**

Ratio of thickness to skin depth:

$$\Delta = t/\delta = 70/301 \approx 0.23$$

The copper is much thinner than one skin depth, so **skin effect through the thickness is negligible at 48 kHz.**

---

## 3. DC Resistance

$$R_{DC} = \frac{\rho\,\ell}{w\,t} \qquad \left(R_\square = \rho/t \approx 0.246\ \text{m}\Omega/\square \text{ at 20 °C}\right)$$

For the example trace (ℓ/w = 5 squares):

- R_DC(20 °C) = 5 × 0.246 ≈ **1.23 mΩ**
- R_DC(100 °C) ≈ **1.61 mΩ**

---

## 4. AC Resistance (Foil / Dowell Model)

For a conductor of thickness t:

$$F_R = \frac{R_{AC}}{R_{DC}} = \Delta\,\frac{\sinh 2\Delta + \sin 2\Delta}{\cosh 2\Delta - \cos 2\Delta}\;\approx\;1 + \frac{\Delta^4}{45}\quad(\Delta \lesssim 1)$$

The skin depth at harmonic n is δₙ = δ/√n, so Δₙ = Δ√n.

| Harmonic | f | δ (20 °C) | Δ | F_R |
|---|---|---|---|---|
| 1 | 48 kHz | 301 µm | 0.23 | 1.0001 |
| 3 | 144 kHz | 174 µm | 0.40 | 1.0006 |
| 9 | 432 kHz | 100 µm | 0.70 | ≈1.005 |
| 25 | 1.2 MHz | 60 µm | 1.16 | ≈1.04 |

**Conclusion:** R_AC ≈ R_DC for all harmonics that carry significant ripple energy.

**Caveat — lateral current crowding:** because δ (≈0.3 mm) is much smaller than w (10 mm), current crowds toward the trace edges, or toward the side facing the return path (proximity effect). This happens if there is no solid reference plane directly under the trace, or if the outgoing and return phase traces run side by side. In that case, expect F_R of roughly 1.1–1.5 at the fundamental. Confirm with a 2D field solver (FEMM, Q2D, or Sonnet) if ripple losses matter.

---

## 5. Trace Impedance Z = R + jωL

At 48 kHz the wavelength is λ ≈ 3 km, so the trace is **electrically short**. Treat it as lumped R + L. The characteristic impedance Z₀ is only an intermediate quantity for extracting L.

### a) Trace over a solid plane (microstrip, w/h ≥ 1)

$$\varepsilon_{eff} = \frac{\varepsilon_r + 1}{2} + \frac{\varepsilon_r - 1}{2}\left(1 + \frac{12h}{w}\right)^{-1/2}$$

$$Z_0 = \frac{120\pi}{\sqrt{\varepsilon_{eff}}\left[\frac{w}{h} + 1.393 + 0.667\ln\!\left(\frac{w}{h} + 1.444\right)\right]}$$

$$L' = \frac{Z_0\sqrt{\varepsilon_{eff}}}{c}$$

For the example (w/h = 50):

- ε_eff ≈ 4.13 and Z₀ ≈ **3.4 Ω**
- L' ≈ 23 nH/m, so L ≈ **1.2 nH**
- X_L = 2π f L ≈ **0.35 mΩ** at 48 kHz

### b) Isolated trace with no nearby return (partial self-inductance, units: nH and mm)

$$L \approx 0.2\,\ell\left[\ln\frac{2\ell}{w+t} + 0.5 + 0.2235\frac{w+t}{\ell}\right]$$

For the example: L ≈ **28 nH**, so X_L ≈ **8.6 mΩ** at 48 kHz.

### Result

$$|Z| = \sqrt{R_{AC}^2 + (\omega L)^2}$$

| Layout | Z at 48 kHz |
|---|---|
| Over plane | 1.61 + j0.35 mΩ, so \|Z\| ≈ 1.65 mΩ |
| No return plane | 1.61 + j8.6 mΩ, so \|Z\| ≈ 8.8 mΩ |

**Key insight:** inductance, not skin effect, dominates the AC impedance. Loop area is the design lever. Above roughly 10 MHz, L also governs switching-edge ringing:

$$V = L\,\frac{di}{dt}$$

For example, 28 nH at 1 A/ns gives 28 V of overshoot.

---

## 6. Loss Formulation

$$P_{trace} = I_{rms,LF}^2\,R_{DC}(T) + \sum_{n} I_{rip,n}^2\,R_{DC}(T)\,F_R(n)$$

For triangular ripple with peak-to-peak value ΔI_pp:

$$I_{rip,rms} = \frac{\Delta I_{pp}}{2\sqrt{3}}$$

**Example:** 30 A_rms phase current and 6 A_pp ripple at 100 °C:

- Low-frequency term: 30² × 1.61 mΩ = **1.45 W**
- Ripple term: (1.73 A)² × 1.61 mΩ ≈ **4.8 mW** (negligible)

---

## 7. Verification Checklist

- Size the trace width for temperature rise using IPC-2152, not skin effect. Iterate R(T) until it converges, since resistance rises with temperature.
- Use the actual plated thickness for outer layers.
- Keep the phase and return or DC-link paths tightly coupled to minimize L.
- If parallel layers are stitched with vias, check current sharing. Inductance mismatch, not resistance, sets the AC split between layers.
- Run a 2D solver if traces are side by side or lack a reference plane.

If you share the Cycle 18 values (width, length, stack-up, I_rms, ripple), I can recompute these with your actual numbers.