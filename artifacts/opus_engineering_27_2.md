# PCB Trace Impedance & Skin-Effect AC Resistance at 48 kHz: 2 oz Motor Phase Tracks

**Note:** I don't have access to your project files or the "Cycle 27" design data. The trace geometry below (5 mm wide × 50 mm long, FR-4 εr = 4.4) is an assumed worked example. Substitute your actual layout values into the formulas.

---

## 1. Design Parameters (Assumed)

| Parameter | Symbol | Value |
|---|---|---|
| Copper weight | — | 2 oz/ft² |
| Copper thickness | t | 70 µm (≈68.6 µm nominal) |
| Trace width | w | 5.0 mm |
| Trace length | ℓ | 50 mm |
| Resistivity (20 °C) | ρ₂₀ | 1.68 × 10⁻⁸ Ω·m |
| Temp. coefficient | α | 0.00393 /°C |
| Fundamental frequency | f | 48 kHz |
| Dielectric height to return plane | h | 1.6 mm (2-layer) / 0.2 mm (4-layer) |
| Permittivity | εr | 4.4 |

---

## 2. Skin Depth

$$\delta = \sqrt{\frac{\rho}{\pi f \mu_0}}$$

At 48 kHz, 20 °C:

$$\delta = \sqrt{\frac{1.68\times10^{-8}}{\pi \cdot 48\,000 \cdot 4\pi\times10^{-7}}} = \sqrt{8.87\times10^{-8}} \approx \mathbf{298\ \mu m}$$

At 100 °C, with ρ ≈ 2.21 × 10⁻⁸ Ω·m, δ ≈ **341 µm**.

**Key result:** δ (298 µm) is much larger than t/2 (35 µm). The current penetrates the full copper thickness, so through-thickness skin effect is negligible at the fundamental.

---

## 3. DC Resistance

$$R_{DC} = \frac{\rho\,\ell}{w\,t}, \qquad R_{\square} = \frac{\rho}{t} = \frac{1.68\times10^{-8}}{70\times10^{-6}} = 0.240\ \text{m}\Omega/\square$$

Number of squares = ℓ/w = 50/5 = 10, so:

$$R_{DC,20^\circ C} = 10 \times 0.240 = \mathbf{2.40\ m\Omega}$$

Temperature correction:

$$R_{DC}(T) = R_{20}\,[1 + \alpha (T - 20)]$$

At 80 °C: R = 2.40 × 1.236 = **2.97 mΩ**

---

## 4. AC Resistance (Skin Effect)

Using the Dowell-form factor for a flat conductor, with Δ = t/δ:

$$F_R = \frac{R_{AC}}{R_{DC}} = \Delta\,\frac{\sinh 2\Delta + \sin 2\Delta}{\cosh 2\Delta - \cos 2\Delta}$$

For small Δ, this simplifies to:

$$F_R \approx 1 + \frac{\Delta^4}{45}$$

At 48 kHz, Δ = 70/298 = 0.235:

$$F_R \approx 1 + \frac{0.00305}{45} = \mathbf{1.00007}$$

So R_AC ≈ R_DC = 2.40 mΩ. The skin effect at 48 kHz is negligible (+0.007 %).

### PWM Harmonics: Where Skin Effect Starts to Matter

Phase current ripple and switching edges carry higher-frequency content:

| Frequency | δ (µm) | Δ = t/δ | F_R (approx.) |
|---|---|---|---|
| 48 kHz (1st) | 298 | 0.235 | 1.0001 |
| 144 kHz (3rd) | 172 | 0.41 | 1.0006 |
| 480 kHz (10th) | 94 | 0.74 | 1.007 |
| 1 MHz | 65 | 1.07 | ~1.05 |
| 10 MHz (edges) | 20.6 | 3.4 | ~1.7 (≈ t/2δ for one-sided current) |

A scaling shortcut: δ ≈ 65.2 / √f[MHz] µm.

**Edge crowding and proximity effect.** For wide traces over a plane, return-current coupling pulls HF current toward the plane-facing surface and the trace edges. This can raise effective R by 1.5–3× above ~1 MHz. It does not affect the 48 kHz fundamental.

---

## 5. Characteristic Impedance (Microstrip, Hammerstad)

**Applicability:** At 48 kHz the wavelength is ≈ 3.1 km in FR-4. The trace is electrically short, so Z₀ matters only for inductance extraction and for ringing from switching edges. It is not a matching requirement.

For w/h ≥ 1:

$$\varepsilon_{eff} = \frac{\varepsilon_r+1}{2} + \frac{\varepsilon_r-1}{2}\left(1+\frac{12h}{w}\right)^{-1/2}$$

$$Z_0 = \frac{120\pi}{\sqrt{\varepsilon_{eff}}\left[\frac{w}{h} + 1.393 + 0.667\ln\left(\frac{w}{h}+1.444\right)\right]}$$

### Case A: 2-layer (h = 1.6 mm, w/h = 3.125)

- ε_eff = 2.70 + 1.70 × (4.84)^(-0.5) = **3.47**, so √ε_eff = 1.864
- Denominator = 3.125 + 1.393 + 0.667 ln(4.569) = 5.531
- **Z₀ = 376.99 / (1.864 × 5.531) ≈ 36.6 Ω**

### Case B: 4-layer (h = 0.2 mm, w/h = 25)

- ε_eff ≈ 4.12
- **Z₀ ≈ 5.6 Ω**

---

## 6. Inductance and Total Impedance at 48 kHz

Per-unit-length inductance from Z₀:

$$L' = \frac{Z_0\sqrt{\varepsilon_{eff}}}{c}$$

| Case | L' (nH/mm) | L (50 mm) | X_L at 48 kHz | \|Z\| = √(R² + X²) |
|---|---|---|---|---|
| A: h = 1.6 mm | 0.227 | 11.4 nH | 3.44 mΩ | 4.19 mΩ (R = 2.40 mΩ) |
| B: h = 0.2 mm | 0.038 | 1.9 nH | 0.57 mΩ | 2.47 mΩ |
| No plane (isolated strip)* | — | 35 nH | 10.6 mΩ | 10.9 mΩ |

\*Isolated-strip partial self-inductance (Grover/Ruehli), with ℓ in cm and L in nH:

$$L = 2\ell\left[\ln\frac{2\ell}{w+t} + 0.5 + 0.2235\frac{w+t}{\ell}\right]$$

For ℓ = 5 cm and w + t = 0.507 cm, L ≈ 35 nH.

**Insight:** At 48 kHz the reactance is equal to or larger than the resistance unless a tight return plane is present. Layout (loop area) matters more than skin effect.

---

## 7. Power Loss Calculation

$$P = I_{RMS,fund}^2 \cdot R_{AC}(f_1) + \sum_n I_{n}^2 \cdot R_{AC}(f_n)$$

Example: 30 A_RMS phase current with 10 % ripple at f_sw. At 80 °C:

$$P \approx 30^2 \times 2.97\ \text{m}\Omega \approx \mathbf{2.67\ W}$$

The ripple contribution is about 0.3 % of this, which is negligible.

**Check against IPC-2152:** A 5 mm, 2 oz external trace carrying 30 A gives roughly a 40–50 °C rise. Widen the trace, parallel it on inner layers, or add solder/busbar reinforcement as needed.

---

## 8. Summary Formulas

| Quantity | Formula | Result (example) |
|---|---|---|
| Skin depth | δ = √(ρ/πfμ₀) | 298 µm |
| R_DC | ρℓ/(wt) | 2.40 mΩ (20 °C) |
| R_AC / R_DC | ≈ 1 + (t/δ)⁴/45 | 1.0001 |
| Z₀ (h = 1.6 mm) | Hammerstad | 36.6 Ω |
| L | Z₀√ε_eff·ℓ/c | 11.4 nH |
| \|Z\| at 48 kHz | √(R² + (2πfL)²) | 4.2 mΩ |

## 9. Design Conclusions

1. **Skin effect is not the limiting factor at 48 kHz** for 2 oz copper. Size the traces for DC/thermal loss using IPC-2152.
2. **Inductance dominates the impedance.** Route phase tracks over a solid plane or alongside their return path to minimize loop area.
3. **Skin and proximity effects matter only for edge harmonics above ~1 MHz.** These affect EMI and ringing more than conduction loss.
4. **Moving to 3–4 oz copper** keeps t < δ, so it still yields a near-linear reduction in R_DC.

If you share the actual Cycle 27 trace dimensions, stackup, and current profile, I can rerun these numbers for your design.