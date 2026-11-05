# PCB Trace Impedance & Skin-Effect AC Resistance at 48 kHz: 2 oz Motor Phase Tracks

**Note on "Cycle 6":** I don't have your earlier cycles or design files. I've used representative values (**10 mm × 50 mm trace, 0.2 mm dielectric to the return plane, FR-4 with εr = 4.3, 20 A RMS phase current**). Every formula is parameterized, so you can substitute your actual geometry.

---

## 1. Base Parameters

| Parameter | Symbol | Value |
|---|---|---|
| Copper thickness (2 oz) | t | 70 µm (2.76 mil) |
| Resistivity @ 20 °C | ρ₂₀ | 1.72 × 10⁻⁸ Ω·m |
| Temp. coefficient | α | 0.00393 /°C |
| Permeability | μ₀ | 4π × 10⁻⁷ H/m |
| Switching frequency | f | 48 kHz |
| Design operating temp. | T | 85 °C |

Resistivity at operating temperature:

$$\rho_T = \rho_{20}\,[1 + \alpha(T-20)] = 1.72\times10^{-8}(1 + 0.00393\cdot65) = 2.16\times10^{-8}\ \Omega\cdot\text{m}$$

---

## 2. Skin Depth

$$\delta = \sqrt{\frac{\rho}{\pi f \mu_0}}$$

| Condition | δ |
|---|---|
| 48 kHz, 20 °C | **301 µm** |
| 48 kHz, 85 °C | **338 µm** |

**Penetration ratio:** Δ = t/δ = 70/301 ≈ **0.23**

Because the copper is only about a quarter of a skin depth thick, current is essentially uniform through the thickness at the fundamental.

---

## 3. AC Resistance Factor (Dowell 1-D Model)

For a trace over a return plane (one-sided current flow, which is the conservative case):

$$F_R = \frac{R_{AC}}{R_{DC}} = \Delta\,\frac{\sinh 2\Delta + \sin 2\Delta}{\cosh 2\Delta - \cos 2\Delta}$$

For Δ < 1, this simplifies to F_R ≈ 1 + Δ⁴/45.

**At 48 kHz:** F_R ≈ 1 + (0.23)⁴/45 ≈ **1.00006**. The skin effect is negligible.

### PWM Harmonic Check (85 °C)

The fundamental is benign, but switching edges carry higher-frequency content:

| Frequency | δ (µm) | Δ | F_R |
|---|---|---|---|
| 48 kHz (fund.) | 338 | 0.21 | 1.000 |
| 480 kHz (10th) | 107 | 0.65 | ~1.004 |
| 2.4 MHz (50th) | 48 | 1.46 | ~1.35 |
| 10 MHz (edge knee, t_r ≈ 35 ns) | 21 | 3.3 | ~3.3 |

Harmonic energy above 1 MHz is small in phase current, which is filtered by the motor inductance. It mainly affects ringing and EMI, not I²R loss.

---

## 4. DC / AC Resistance

$$R_{DC} = \frac{\rho_T\, L}{w\, t}$$

Example (L = 50 mm, w = 10 mm):

- R_DC @ 20 °C = 1.72e-8 × 0.05 / (0.01 × 70e-6) = **1.23 mΩ**
- R_DC @ 85 °C = **1.54 mΩ**
- R_AC @ 48 kHz ≈ R_DC × 1.0001 ≈ **1.54 mΩ**

**Conduction loss:** P = I²_RMS × R_AC = 20² × 1.54 mΩ ≈ **0.62 W per phase track**

**Width-direction current crowding:** For w ≫ t, the edges carry excess current at high frequency. At 48 kHz with δ ≫ t this adds less than 1–2%. Add a 5–10% margin if you include a harmonic-weighted loss.

---

## 5. Inductance and Impedance

At 48 kHz the wavelength in FR-4 is about 3 km, so the trace is **electrically short**. The lumped L and R matter far more than the characteristic impedance Z₀.

### Partial Inductance

**Isolated trace (no adjacent plane):**

$$L_{nH} = 0.2\,l\left[\ln\frac{2l}{w+t} + 0.5 + 0.2235\frac{w+t}{l}\right]\quad (l,w,t\ \text{in mm})$$

→ L ≈ **28 nH**
→ X_L = 2π(48 kHz)(28 nH) ≈ **8.6 mΩ**, which is more than 5× R_AC.

**Over a solid return plane (h = 0.2 mm):**

$$L \approx \frac{\mu_0\, h\, l}{w}$$

→ L ≈ **1.26 nH**
→ X_L ≈ **0.38 mΩ**

**Key takeaway:** Loop inductance dominates over the skin effect. Route phase tracks with a tight return path or plane.

### Characteristic Impedance (Hammerstad Microstrip, w/h ≥ 1)

This matters for edge ringing and EMI, not for 48 kHz power transfer.

$$\varepsilon_{eff} = \frac{\varepsilon_r+1}{2} + \frac{\varepsilon_r-1}{2}\left(1+\frac{12h}{w}\right)^{-1/2} = 4.13$$

$$Z_0 = \frac{120\pi}{\sqrt{\varepsilon_{eff}}\,\left[\frac{w}{h} + 1.393 + 0.667\ln\left(\frac{w}{h}+1.444\right)\right]} \approx 3.4\ \Omega$$

**Complex series impedance at 48 kHz (over plane):**

$$Z = R_{AC} + j\omega L \approx 1.54 + j0.38\ \text{m}\Omega,\quad |Z| \approx 1.59\ \text{m}\Omega$$

---

## 6. Ampacity Cross-Check (IPC-2221, External Layer)

$$I = 0.048\,\Delta T^{0.44}\,A^{0.725}\quad(A\ \text{in mil}^2)$$

- A = 393.7 mil × 2.76 mil ≈ 1087 mil²
- At ΔT = 20 °C: I ≈ **28 A**, which gives about 40% margin over 20 A RMS.

IPC-2152 with plane and via heat spreading is usually less conservative. Use it for final sizing.

---

## 7. Summary

| Quantity | Result | Design Implication |
|---|---|---|
| δ @ 48 kHz | 301–338 µm ≫ 70 µm | Skin effect negligible |
| R_AC/R_DC | ≈ 1.000 | Size the trace on DC resistance plus thermal limits |
| R_AC (10×50 mm, 85 °C) | 1.54 mΩ | 0.62 W per phase at 20 A |
| L (isolated / over plane) | 28 nH / 1.3 nH | Return-path layout matters most |
| Z₀ | ~3.4 Ω | Relevant only to switching-edge ringing |
| Ampacity | ~28 A @ ΔT = 20 °C | Adequate; verify with IPC-2152 |

**Recommendations:**
- Thicker copper (3–4 oz) still lowers resistance nearly linearly at this frequency, because t ≪ δ.
- Paralleling layers with stitching vias is fully effective at 48 kHz.
- Prioritize minimizing the loop area between phase and DC-link/return paths over any skin-effect mitigation.

If you share the actual Cycle 6 dimensions, current profile, and stack-up, I can recompute these numbers for your design.