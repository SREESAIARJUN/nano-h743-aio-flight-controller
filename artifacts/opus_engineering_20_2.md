# PCB Phase-Track Impedance and Skin-Effect AC Resistance at 48 kHz (2 oz Cu)

I don't have the specific parameters for your "Cycle 20" design, so this uses a representative geometry. Every formula is given symbolically, so you can substitute your actual width, length, stack-up and current.

**Assumed geometry:** outer-layer track, w = 10 mm, ℓ = 50 mm, FR-4 (εr = 4.4), h = 1.6 mm to the return plane, operating at 100 °C.

---

## 1. Material and Geometry Constants

| Parameter | Symbol | Value |
|---|---|---|
| 2 oz copper thickness | t | 70 µm (68.6 µm nominal) |
| Resistivity @ 20 °C | ρ₂₀ | 1.72 × 10⁻⁸ Ω·m |
| Temperature coefficient | α | 0.00393 /°C |
| Resistivity @ 100 °C | ρ₁₀₀ = ρ₂₀[1 + α(T − 20)] | 2.26 × 10⁻⁸ Ω·m |
| Permeability | μ₀ | 4π × 10⁻⁷ H/m |

---

## 2. Skin Depth

$$\delta = \sqrt{\frac{\rho}{\pi f \mu_0}}$$

| f | δ @ 20 °C | δ @ 100 °C |
|---|---|---|
| 48 kHz (PWM fundamental) | 0.301 mm | 0.345 mm |
| 96 kHz (2nd harmonic, typical ripple for centre-aligned PWM) | 0.213 mm | 0.244 mm |
| 240 kHz (5th harmonic) | 0.135 mm | 0.154 mm |

**Key ratio:** Δ = t/δ = 0.070 / 0.345 ≈ **0.20** at 48 kHz.

Because t ≪ 2δ, current fills the copper thickness almost uniformly.

---

## 3. DC Resistance

$$R_{DC} = \frac{\rho\,\ell}{w\,t}$$

- At 20 °C: 1.72e-8 × 0.05 / (0.01 × 70e-6) = **1.23 mΩ**
- At 100 °C: **1.61 mΩ**

---

## 4. AC Resistance (Dowell Formulation)

For a flat conductor carrying current, with m = 1 layer or with proximity to adjacent layers:

$$F_R = \frac{R_{AC}}{R_{DC}} = \Delta\left[\frac{\sinh 2\Delta + \sin 2\Delta}{\cosh 2\Delta - \cos 2\Delta} + \frac{2(m^2-1)}{3}\cdot\frac{\sinh\Delta - \sin\Delta}{\cosh\Delta + \cos\Delta}\right]$$

For small Δ this reduces to:

$$F_R \approx 1 + \frac{(5m^2 - 1)\,\Delta^4}{45}$$

| Case | Δ | F_R | R_AC @ 100 °C |
|---|---|---|---|
| 48 kHz, m = 1 | 0.20 | 1.00007 | 1.61 mΩ |
| 96 kHz, m = 1 | 0.29 | 1.0006 | 1.61 mΩ |
| 48 kHz, m = 2 (paralleled layers, phase over phase) | 0.20 | 1.0006 | 1.61 mΩ |

**Conclusion:** Through-thickness skin effect is negligible for 2 oz copper at 48 kHz.

The effects that do matter are:
- **Lateral edge crowding** in wide tracks (current concentrates at the edges facing the return path). Estimate with 2-D FEM (e.g., FEMM, Q3D). It typically adds 5–20% for w/h > 5.
- **Proximity to adjacent phase tracks** or shunt loops.

---

## 5. Track Impedance and Inductance

At 48 kHz, λ ≈ 6 km, so the track is **electrically short**. Treat it as lumped R + jωL, not as a transmission line. Z₀ is computed below only to derive L.

**Effective permittivity (Hammerstad, w/h ≥ 1):**

$$\varepsilon_{eff} = \frac{\varepsilon_r+1}{2} + \frac{\varepsilon_r-1}{2}\left(1+\frac{12h}{w}\right)^{-1/2}$$

With w/h = 6.25, ε_eff = 3.70.

**Characteristic impedance:**

$$Z_0 = \frac{120\pi}{\sqrt{\varepsilon_{eff}}\left[\frac{w}{h} + 1.393 + 0.667\ln\left(\frac{w}{h}+1.444\right)\right]} \approx 21.8\ \Omega$$

**Inductance:**

$$L' = \frac{Z_0\sqrt{\varepsilon_{eff}}}{c} \approx 140\ \text{nH/m}$$

$$L = L' \cdot \ell \approx 7.0\ \text{nH}$$

**Total impedance:**

| Frequency | R_AC | X_L = 2πfL | \|Z\| |
|---|---|---|---|
| 48 kHz | 1.61 mΩ | 2.11 mΩ | **2.65 mΩ** |
| 96 kHz | 1.61 mΩ | 4.22 mΩ | 4.52 mΩ |
| Switching edge (t_r = 20 ns → f_knee ≈ 0.35/t_r = 17.5 MHz) | ~1.9 mΩ* | 0.77 Ω | 0.77 Ω |

*At 17.5 MHz, δ ≈ 16 µm, so Δ ≈ 4.4 and F_R ≈ Δ (≈ 4.4× on that harmonic only).

**Edge overshoot:** V = L·di/dt. For example, 30 A in 20 ns gives 7 nH × 1.5 GA/s ≈ **10.5 V** across the track. This is why the commutation loop, not the phase track, must be minimised.

---

## 6. Loss Calculation

$$P = I_{fund,rms}^2 R_{DC} + \sum_n I_{n,rms}^2 R_{DC}F_R(n f_{sw})$$

**Example:** 30 A_rms phase current with 3 A_rms 48 kHz ripple:

$$P \approx (30^2 + 3^2)(1.61\ \text{m}\Omega) \approx 1.46\ \text{W per track}$$

Check against **IPC-2152**. For 10 mm / 2 oz external at ~30 A, the temperature rise is roughly 20–30 °C, depending on the copper planes nearby. Iterate ρ(T) until it converges.

---

## 7. Design Rules

1. 2 oz copper is **not skin-limited** below ~1 MHz (t < δ). Don't go to thicker copper expecting AC benefit; only DC/thermal benefit applies.
2. **Widen the track** (or parallel layers with via stitching every ~5 mm) to cut both R and L.
3. **Route the return/adjacent phase tightly coupled** (small h or broadside) to reduce L. Note this increases proximity loss slightly, by (5m² − 1)Δ⁴/45, which is negligible here.
4. If you share your Cycle 20 values (w, ℓ, layer, h, I_rms, ripple spectrum, T_max), I can rerun the numbers exactly.