# PCB Phase-Track Impedance & Skin-Effect AC Resistance at 48 kHz (2 oz Cu)

I don't have your Cycle 29 design data (track widths, lengths, stack-up, current levels), so the formulas below are worked through with **stated example values**. Substitute your own values to get your numbers.

---

## 1. Example Inputs

| Parameter | Symbol | Value used |
|---|---|---|
| Copper weight | — | 2 oz/ft² |
| Copper thickness | t | 70 µm (2.8 mil) |
| Track width | w | 10 mm |
| Track length (one phase run) | ℓ | 50 mm |
| Dielectric height to return plane | h | 1.6 mm (FR-4) |
| Relative permittivity | εr | 4.4 |
| Cu resistivity at 20 °C | ρ₂₀ | 1.72 × 10⁻⁸ Ω·m |
| Temperature coefficient of Cu | α | 0.00393 /°C |
| Operating copper temperature | T | 100 °C |
| PWM / ripple frequency | f | 48 kHz |

**Important context:** In a motor drive, the *phase current fundamental* is the electrical frequency, typically tens of Hz to about 1 kHz. **48 kHz is the PWM ripple component** and its harmonics. The bulk I²R loss is effectively at DC/low frequency. The 48 kHz analysis governs ripple loss and impedance.

---

## 2. Skin Depth

$$
\delta = \sqrt{\frac{\rho}{\pi f \mu_0}}
$$

**At 20 °C:**

$$
\delta = \sqrt{\frac{1.72\times10^{-8}}{\pi \cdot 48\,000 \cdot 4\pi\times10^{-7}}} = \sqrt{\frac{1.72\times10^{-8}}{0.1895}} \approx \mathbf{0.301\ mm}
$$

**At 100 °C**, with ρ₁₀₀ = ρ₂₀[1 + α(80)] = 2.26 × 10⁻⁸ Ω·m:

$$
\delta_{100} \approx \mathbf{0.345\ mm}
$$

**Ratio of thickness to skin depth:**

$$
\frac{t}{\delta} = \frac{0.070}{0.301} \approx 0.23
$$

The copper is about 4× thinner than the skin depth at 48 kHz.

---

## 3. DC Resistance

$$
R_{DC} = \frac{\rho\,\ell}{w\,t}
$$

**At 20 °C:**

$$
R_{DC,20} = \frac{1.72\times10^{-8} \cdot 0.05}{0.010 \cdot 70\times10^{-6}} = \mathbf{1.23\ m\Omega}
$$

**At 100 °C:**

$$
R_{DC,100} = 1.23 \cdot (1 + 0.00393\cdot80) = \mathbf{1.61\ m\Omega}
$$

---

## 4. AC Resistance (Skin Effect)

Use the Dowell 1-D conductor factor, with Δ = t/δ:

$$
F_R = \frac{R_{AC}}{R_{DC}} = \Delta\,\frac{\sinh 2\Delta + \sin 2\Delta}{\cosh 2\Delta - \cos 2\Delta}
$$

For small Δ, this simplifies to F_R ≈ 1 + (4/45)Δ⁴.

$$
F_R \approx 1 + 0.089\,(0.23)^4 \approx \mathbf{1.0003}
$$

$$
R_{AC}(48\text{ kHz}) \approx R_{DC} \approx \mathbf{1.61\ m\Omega\ @\ 100\,°C}
$$

**Conclusion:** Skin effect is negligible at 48 kHz for 2 oz copper. It only becomes significant when δ approaches t, i.e. at about **0.9 MHz** (δ = 70 µm).

**Check of PWM harmonics** (switching edges carry energy well above 48 kHz):

| Frequency | δ (µm) | t/δ | R_AC/R_DC (approx.) |
|---|---|---|---|
| 48 kHz | 301 | 0.23 | 1.00 |
| 480 kHz (10th) | 95 | 0.74 | ~1.03 |
| 1.8 MHz | 49 | 1.43 | ~1.3 |
| 7 MHz (edge knee, t_r ≈ 50 ns) | 25 | 2.8 | ~2.8 |

**Proximity effect:** Over a solid return plane, HF current crowds onto the underside of the track and toward its edges. This adds only a few percent at 48 kHz but grows with frequency. Keep phase and return paths tightly coupled to minimize loop inductance rather than fighting R_AC.

---

## 5. Characteristic Impedance (Microstrip, Wide-Trace Form)

**Effective permittivity:**

$$
\varepsilon_{eff} = \frac{\varepsilon_r+1}{2} + \frac{\varepsilon_r-1}{2}\left(1+\frac{12h}{w}\right)^{-1/2}
= 2.7 + 1.7/1.709 = \mathbf{3.70}
$$

**Impedance** (Hammerstad, valid for w/h > 1; here w/h = 6.25):

$$
Z_0 = \frac{120\pi}{\sqrt{\varepsilon_{eff}}\left[\frac{w}{h} + 1.393 + 0.667\ln\left(\frac{w}{h}+1.444\right)\right]}
= \frac{377}{1.922 \cdot 9.00} \approx \mathbf{21.8\ \Omega}
$$

**Is Z₀ relevant at 48 kHz?**

$$
\lambda = \frac{c}{f\sqrt{\varepsilon_{eff}}} \approx 3.25\ \text{km} \gg 50\ \text{mm}
$$

No. The track is electrically short and behaves as a **lumped R–L**. Z₀ only matters for switching-edge ringing (MHz range).

---

## 6. Lumped Series Impedance at 48 kHz

**Inductance per unit length:**

$$
L' = \frac{Z_0\sqrt{\varepsilon_{eff}}}{c} = \frac{21.8 \cdot 1.922}{3\times10^8} \approx 140\ \text{nH/m}
\;\Rightarrow\; L \approx \mathbf{7\ nH}\ \text{(50 mm)}
$$

**Reactance:**

$$
X_L = 2\pi f L = 2\pi \cdot 48\,000 \cdot 7\times10^{-9} \approx \mathbf{2.1\ m\Omega}
$$

**Total impedance magnitude:**

$$
|Z| = \sqrt{R_{AC}^2 + X_L^2} = \sqrt{1.61^2 + 2.11^2} \approx \mathbf{2.65\ m\Omega}
$$

Inductive reactance already exceeds resistance at 48 kHz. The **R = X_L crossover** is at f = R/(2πL) ≈ 37 kHz.

---

## 7. Loss Calculation

$$
P = I_{rms,LF}^2\,R_{DC,T} \;+\; \sum_n I_{rip,n}^2\,R_{AC}(nf_{sw})
$$

**Example:** 30 A rms phase current and 3 A rms ripple at 48 kHz:

$$
P \approx 30^2(1.61\,\text{m}) + 3^2(1.61\,\text{m}) \approx 1.45 + 0.015 \approx \mathbf{1.46\ W\ per\ phase\ track}
$$

Iterate the copper temperature using IPC-2152 temperature-rise charts, since R_DC depends on T.

---

## 8. Design Takeaways

1. **2 oz copper is thin relative to δ at 48 kHz.** Do not derate for skin effect; size the track for DC/LF I²R and thermal rise (IPC-2152).
2. **Widening or adding copper layers** (parallel layers stitched with via arrays) reduces R almost linearly at this frequency.
3. **Loop inductance matters more than R_AC.** Route phase tracks over a continuous plane and keep the high-di/dt commutation loop (bus capacitor to half-bridge) as small as possible.
4. **Z₀ (~22 Ω here) is only relevant to edge ringing and EMI**, not to 48 kHz conduction.

If you share the actual Cycle 29 widths, lengths, stack-up, and current profile, I can recompute these values for your design.