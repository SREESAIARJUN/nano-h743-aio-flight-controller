# PCB Phase-Track Analysis at 48 kHz: 2 oz Copper Motor Phases

I don't have the specific geometry or requirements from your "Cycle 26" design package. The worked numbers below use **stated example dimensions** that you can replace with your actual values. The formulas are general.

---

## 1. Baseline Parameters

| Parameter | Symbol | Value |
|---|---|---|
| Copper thickness (2 oz/ft²) | t | 70 µm (nominal, ±10% after plating/etch) |
| Copper resistivity @ 20 °C | ρ₂₀ | 1.72 × 10⁻⁸ Ω·m |
| Temp. coefficient | α | 0.00393 /°C |
| Permeability | μ₀ | 4π × 10⁻⁷ H/m |
| PWM / ripple frequency | f | 48 kHz |
| **Example** track width | w | 5 mm |
| **Example** track length | ℓ | 50 mm |
| **Example** dielectric height to reference plane | h | 0.2 mm |
| FR-4 permittivity | εᵣ | 4.3 |

---

## 2. Skin Depth

$$\delta = \sqrt{\frac{\rho}{\pi f \mu_0}}$$

**At 20 °C, 48 kHz:**

$$\delta = \sqrt{\frac{1.72\times10^{-8}}{\pi \cdot 48\,000 \cdot 4\pi\times10^{-7}}} = \sqrt{\frac{1.72\times10^{-8}}{0.1895}} \approx \mathbf{301\ µm}$$

**At 100 °C** (ρ = ρ₂₀[1 + α·80] ≈ 2.26 × 10⁻⁸ Ω·m): δ ≈ **345 µm**

### Harmonic check
Because δ ∝ 1/√f:

| Frequency | δ (20 °C) | δ vs. t/2 = 35 µm |
|---|---|---|
| 48 kHz (fundamental) | 301 µm | ≫ |
| 144 kHz (3rd) | 174 µm | ≫ |
| 240 kHz (5th) | 135 µm | ≫ |
| 480 kHz (10th) | 95 µm | > |
| ~3.4 MHz | 35 µm | ≈ crossover |

**Conclusion:** δ ≫ t, so current fills the full copper cross-section at 48 kHz and its significant harmonics.

---

## 3. DC Resistance

$$R_{DC} = \frac{\rho\,\ell}{w\,t}$$

**Sheet resistance (2 oz):**
- 20 °C: R□ = ρ/t = 0.246 mΩ/□
- 100 °C: R□ ≈ 0.322 mΩ/□

**Example track (5 mm × 50 mm = 10 squares):**
- 20 °C: R_DC ≈ **2.46 mΩ**
- 100 °C: R_DC ≈ **3.22 mΩ**

---

## 4. AC Resistance (Skin Effect)

### General formulation
For a rectangular conductor, current flows in a shell of depth δ:

$$R_{AC} \approx \frac{\rho\,\ell}{A_{eff}}, \qquad A_{eff} = w\,t - (w - 2\delta)(t - 2\delta) \quad \text{(valid when } \delta < t/2\text{)}$$

When δ ≥ t/2, A_eff = w·t, so R_AC = R_DC.

### Smooth correction for the thin-conductor regime
Using x = t/δ:

$$\frac{R_{AC}}{R_{DC}} \approx \frac{x}{2}\cdot\frac{\sinh x + \sin x}{\cosh x - \cos x}$$

At 48 kHz, x = 70/301 = 0.233, giving:

$$\frac{R_{AC}}{R_{DC}} \approx 1 + \frac{x^4}{180} \approx 1.000016$$

**R_AC ≈ R_DC ≈ 2.46 mΩ (20 °C) / 3.22 mΩ (100 °C).** Skin effect is negligible.

### Effects that matter more
- **Proximity effect:** Over a solid reference plane, the return current crowds under the track and pulls phase current toward the lower face and edges. With δ ≫ t, expect only a few percent increase. Adjacent anti-phase tracks spaced less than about 2w apart can raise this further.
- **Surface roughness** (Hammerstad factor): only relevant when δ approaches R_q (~1–3 µm), so it can be ignored here.

---

## 5. Characteristic Impedance and Inductance

### Is Z₀ relevant at 48 kHz?
The wavelength in FR-4 at 48 kHz is λ ≈ c/(f√ε_eff) ≈ 3.1 km. A 50 mm track is about λ/60,000, so it is **electrically short**. Treat it as lumped R + L. Z₀ matters only for dv/dt edge harmonics (EMI), where f_knee ≈ 0.35/t_r, e.g. 35 MHz for a 10 ns edge.

### Wide microstrip (w/h = 25)
The IPC-2141 formula is invalid at this ratio. Use Hammerstad-Jensen instead.

**Effective permittivity:**

$$\varepsilon_{eff} = \frac{\varepsilon_r+1}{2} + \frac{\varepsilon_r-1}{2}\left(1+\frac{12h}{w}\right)^{-1/2} = 2.65 + 1.65(1.48)^{-1/2} \approx 4.01$$

**Characteristic impedance:**

$$Z_0 = \frac{120\pi}{\sqrt{\varepsilon_{eff}}\left[\frac{w}{h} + 1.393 + 0.667\ln\left(\frac{w}{h}+1.444\right)\right]}$$

$$Z_0 = \frac{377}{2.00 \times (25 + 1.393 + 2.184)} \approx \mathbf{6.6\ \Omega}$$

**Per-unit-length inductance and capacitance:**

$$L' = \frac{Z_0\sqrt{\varepsilon_{eff}}}{c} \approx 44\ \text{nH/m}, \qquad C' = \frac{\sqrt{\varepsilon_{eff}}}{Z_0\,c} \approx 1.0\ \text{nF/m}$$

**Example track (50 mm):**
- L ≈ 2.2 nH, so X_L = 2πfL ≈ **0.66 mΩ** at 48 kHz
- C ≈ 50 pF (the dv/dt coupling path to the plane: i = C·dv/dt, e.g. 50 pF × 10 V/ns = 0.5 A spikes)

**Total impedance at 48 kHz (100 °C):**

$$|Z| = \sqrt{R_{AC}^2 + X_L^2} = \sqrt{3.22^2 + 0.66^2} \approx 3.29\ \text{mΩ}$$

---

## 6. Power Dissipation

$$P = I_{rms,fund}^2 R_{DC} + \sum_n I_{rms,n}^2 R_{AC,n}$$

Since R_AC,n ≈ R_DC here, this reduces to **P ≈ I²_rms,total · R_DC**.

**Example:** 30 A_rms at 100 °C gives P = 900 × 3.22 mΩ ≈ **2.9 W per phase track**. This is substantial. Size the track width using IPC-2152 for your allowable temperature rise.

---

## 7. Design Takeaways

1. Skin effect is not a constraint at 48 kHz on 2 oz copper. Widen the track, add parallel layers, or thicken the copper; no Litz-style splitting is needed.
2. **Use hot-copper resistance (~1.3× R₂₀)** for loss and thermal budgets.
3. Proximity effect and via/connector resistance will likely dominate the AC penalty. Stitch layers with via arrays (each 0.3 mm via ≈ 0.5–1 mΩ).
4. Z₀ (~5–10 Ω for wide power traces) matters only for switching-edge EMI. Minimise the phase-node area over the plane to limit capacitive dv/dt coupling.

If you share the actual Cycle 26 track width, length, stackup, current profile, and temperature limit, I can rerun these numbers for your design.