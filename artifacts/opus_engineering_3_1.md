# PCB Trace Impedance and Skin-Effect AC Resistance at 48 kHz for 2 oz Motor Phase Tracks

I don't have your Cycle 3 geometry or stack-up, so the worked numbers below use the assumed values in Section 0. Substitute your own values and the method carries through unchanged.

---

## 0. Assumed Parameters

| Symbol | Meaning | Value |
|---|---|---|
| t | Copper thickness (2 oz) | 70 µm |
| w | Trace width | 5.0 mm |
| ℓ | Trace length (phase node to connector) | 50 mm |
| h | Dielectric height to reference plane | 0.20 mm |
| εr | FR-4 relative permittivity | 4.4 |
| ρ₂₀ | Copper resistivity at 20 °C | 1.72 × 10⁻⁸ Ω·m |
| α | Copper temperature coefficient | 0.00393 /°C |
| f_sw | PWM frequency | 48 kHz |
| I_rms | Phase current | 30 A |
| T | Operating copper temperature | 100 °C |

---

## 1. DC Resistance

$$R_{DC} = \frac{\rho \, \ell}{w \, t}$$

At 20 °C:

$$R_{DC,20} = \frac{1.72\times10^{-8} \times 0.05}{0.005 \times 70\times10^{-6}} = 2.46\ \text{m}\Omega$$

Temperature correction:

$$R_{DC}(T) = R_{DC,20}\,[1+\alpha(T-20)]$$

At 100 °C this gives 2.46 mΩ × 1.314 = **3.23 mΩ**.

---

## 2. Skin Depth

$$\delta = \sqrt{\frac{\rho}{\pi f \mu_0}}$$

At 48 kHz:

$$\delta_{48k} = \sqrt{\frac{1.72\times10^{-8}}{\pi \cdot 48\,000 \cdot 4\pi\times10^{-7}}} \approx 301\ \mu\text{m}$$

At 100 °C, δ scales by √1.314, giving about 345 µm.

**Key ratio:**

$$\Delta = t/\delta = 70/301 \approx 0.23$$

The copper is far thinner than one skin depth. Current is therefore essentially uniform through the thickness.

---

## 3. AC Resistance at 48 kHz

### 3a. Through-thickness (skin plus proximity to the ground plane)

Use the Dowell one-sided slab model, with m = 1 because the return current flows in the plane beneath:

$$F_R = \frac{R_{AC}}{R_{DC}} = \Delta\,\frac{\sinh 2\Delta + \sin 2\Delta}{\cosh 2\Delta - \cos 2\Delta}$$

For small Δ, this reduces to:

$$F_R \approx 1 + \frac{5m^2-1}{45}\Delta^4$$

With m = 1 and Δ = 0.23:

$$F_R = 1 + 0.0889\,(0.233)^4 \approx 1.0003$$

### 3b. Lateral edge crowding

The trace width (5 mm) is about 17δ, so current concentrates at the edges and under the trace toward the return path. Over a solid plane this typically adds 5–15 %.

Use a conservative design factor of **k_edge ≈ 1.10**. Confirm it with a 2-D field solver (e.g. FEMM or Q2D) if the margin is tight.

$$R_{AC,48k} \approx R_{DC}(T)\cdot F_R \cdot k_{edge} \approx 3.23 \times 1.0003 \times 1.10 \approx \mathbf{3.55\ m\Omega}$$

### 3c. PWM harmonics

δ falls as 1/√f, giving these values:

| Harmonic | f | δ | t/δ | F_R (thickness) |
|---|---|---|---|---|
| 1st | 48 kHz | 301 µm | 0.23 | ≈1.000 |
| 3rd | 144 kHz | 174 µm | 0.40 | ≈1.002 |
| 5th | 240 kHz | 135 µm | 0.52 | ≈1.006 |
| ~1 MHz | 1 MHz | 66 µm | 1.06 | ≈1.10 |
| Switching-edge knee (t_r = 20 ns) | 17.5 MHz | 16 µm | 4.4 | ≈4.4 (≈ t/δ, one-sided conduction) |

Skin effect only becomes significant in the MHz edge content. That region governs damping of ringing, not conduction loss.

---

## 4. Characteristic Impedance

The trace is wide (w/h = 25), which is outside the validity range of the IPC-2141 formula. Use the Hammerstad wide-microstrip form instead.

**Effective permittivity:**

$$\varepsilon_{eff} = \frac{\varepsilon_r+1}{2} + \frac{\varepsilon_r-1}{2}\left(1+\frac{12h}{w}\right)^{-1/2} = 2.7 + 1.7(1.48)^{-1/2} \approx 4.10$$

**Impedance:**

$$Z_0 = \frac{120\pi}{\sqrt{\varepsilon_{eff}}\left[\dfrac{w}{h} + 1.393 + 0.667\ln\!\left(\dfrac{w}{h}+1.444\right)\right]}$$

$$Z_0 = \frac{377}{2.024\,[25 + 1.393 + 2.184]} \approx \mathbf{6.5\ \Omega}$$

### Is Z₀ relevant at 48 kHz?

The wavelength in the board at 48 kHz is:

$$\lambda = \frac{c}{f\sqrt{\varepsilon_{eff}}} \approx 3.1\ \text{km}$$

The trace is electrically short (ℓ ≪ λ/10). Model it as a **lumped R–L–C**. Z₀ is only useful for deriving L′ and C′, and it does matter for edge-rate ringing.

**Per-unit-length parameters derived from Z₀:**

$$L' = \frac{Z_0\sqrt{\varepsilon_{eff}}}{c} \approx 44\ \text{nH/m}$$

$$C' = \frac{\sqrt{\varepsilon_{eff}}}{c\,Z_0} \approx 1.04\ \text{nF/m}$$

**Lumped values for the 50 mm trace:**

- L ≈ 2.2 nH, so X_L at 48 kHz is about 0.66 mΩ. This is smaller than R_AC, so the trace is resistance-dominated at the PWM fundamental.
- C ≈ 52 pF to the plane.
  - At dv/dt = 48 V / 20 ns, the displacement current is i = C·dv/dt ≈ **125 mA** per edge.
  - This is a common-mode/EMI concern. Weigh it against the low-L benefit of the close plane.

---

## 5. Loss and Thermal Check

$$P = I_{rms,fund}^2 R_{DC}(T) + \sum_n I_{n}^2 R_{AC}(f_n)$$

The motor current is dominated by the low-frequency fundamental, and the 48 kHz ripple is typically 10–30 % peak-to-peak. The loss is therefore approximately:

$$P \approx 30^2 \times 3.23\ \text{m}\Omega \approx \mathbf{2.9\ W}$$

The 48 kHz ripple adds less than about 1 % to this.

**Action:** check the temperature rise against IPC-2152 for 5 mm of 2 oz copper at 30 A. This is likely to be marginal on an external layer alone. Options to reduce it:

- Widen the trace.
- Parallel the trace on an inner layer and stitch with vias.
- Add solder or bus-bar reinforcement.

---

## 6. Summary

| Quantity | Value |
|---|---|
| R_DC at 20 °C / 100 °C | 2.46 / 3.23 mΩ |
| δ at 48 kHz | ≈ 301 µm (more than 4× t) |
| R_AC/R_DC at 48 kHz | ≈ 1.10 (edge crowding dominates; thickness term ≈ 1.000) |
| R_AC at 48 kHz, 100 °C | ≈ 3.55 mΩ |
| Z₀ (microstrip, h = 0.2 mm) | ≈ 6.5 Ω (lumped regime at 48 kHz) |
| L / C (50 mm) | 2.2 nH / 52 pF |
| Conduction loss at 30 A | ≈ 2.9 W |

**Conclusion:** at 48 kHz, 2 oz copper is not skin-limited. Size the track on DC resistance and temperature rise. Skin and proximity effects matter only for edge-rate harmonics (MHz and above), where they help damp ringing.

If you provide the actual Cycle 3 width, length, stack-up, and current profile, I'll rerun the numbers.