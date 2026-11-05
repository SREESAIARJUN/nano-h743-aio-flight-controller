# PCB Motor Phase Trace Analysis: Impedance and Skin-Effect AC Resistance at 48 kHz (2 oz Cu)

I don't have the specific data from "Cycle 11" (trace widths, lengths, stack-up, phase current). Below is a complete formulation with a worked example using assumed values, marked **[ASSUMED]**. Substitute your design values.

---

## 1. Input Parameters

| Parameter | Symbol | Value |
|---|---|---|
| Copper weight | – | 2 oz/ft² |
| Copper thickness | t | 70 µm (2.8 mil), nominal finished |
| Resistivity, 20 °C | ρ₂₀ | 1.72 × 10⁻⁸ Ω·m |
| Temp. coefficient | α | 0.00393 /°C |
| Switching frequency | f_sw | 48 kHz |
| Trace width | w | 5.0 mm **[ASSUMED]** |
| Trace length | ℓ | 50 mm **[ASSUMED]** |
| Dielectric height to return plane | h | 0.20 mm **[ASSUMED]** |
| Dielectric constant (FR-4) | εᵣ | 4.3 |
| Operating temperature | T | 100 °C **[ASSUMED]** |
| Phase current (RMS) | I_rms | 30 A **[ASSUMED]** |

Temperature-corrected resistivity:

$$\rho(T) = \rho_{20}\,[1+\alpha(T-20)] \;\Rightarrow\; \rho_{100} = 1.72\times10^{-8}(1.314) = 2.26\times10^{-8}\ \Omega\cdot\text{m}$$

---

## 2. DC Resistance

$$R_{DC} = \frac{\rho\,\ell}{w\,t}$$

Sheet resistance for 2 oz copper:

$$R_\square = \rho/t = 0.246\ \text{m}\Omega/\square \;(20^\circ\text{C}), \quad 0.323\ \text{m}\Omega/\square \;(100^\circ\text{C})$$

For the example, ℓ/w = 10 squares:

$$R_{DC,20} = 2.46\ \text{m}\Omega, \qquad R_{DC,100} = 3.23\ \text{m}\Omega$$

---

## 3. Skin Depth

$$\delta = \sqrt{\frac{\rho}{\pi f \mu_0}}$$

| Frequency | δ at 20 °C | δ at 100 °C | t/δ (100 °C) |
|---|---|---|---|
| 48 kHz (fundamental) | 301 µm | 345 µm | 0.203 |
| 144 kHz (3rd harmonic) | 174 µm | 199 µm | 0.352 |
| 240 kHz (5th harmonic) | 135 µm | 154 µm | 0.454 |
| 480 kHz (10th harmonic) | 95 µm | 109 µm | 0.642 |

**Key result:** t = 70 µm is much smaller than δ at 48 kHz. Skin effect through the copper thickness is negligible.

---

## 4. AC Resistance Factor

### 4a. Through-thickness (Dowell, single layer)

With Δ = t/δ:

$$F_R = \frac{R_{AC}}{R_{DC}} = \Delta\,\frac{\sinh 2\Delta + \sin 2\Delta}{\cosh 2\Delta - \cos 2\Delta} \;\approx\; 1 + \frac{4}{45}\Delta^4 \quad (\Delta \ll 1)$$

| Frequency | F_R |
|---|---|
| 48 kHz | 1.0002 |
| 144 kHz | 1.0014 |
| 240 kHz | 1.0038 |

### 4b. Lateral current crowding (the effect that actually matters)

For thin strips, current redistributes across the **width** over a characteristic length:

$$\lambda = \frac{\delta^2}{t} \approx \frac{(345\ \mu\text{m})^2}{70\ \mu\text{m}} \approx 1.7\ \text{mm at 48 kHz}$$

- **Isolated trace (no adjacent plane):** w = 5 mm ≈ 3λ, so current crowds toward the edges. A typical estimate is F_R ≈ 1.1–1.3.
- **Trace over a solid return plane at h = 0.2 mm (h ≪ w):** the proximity effect pulls current to the face nearest the plane and spreads it evenly across the width. Edge crowding is largely suppressed, giving F_R ≈ 1.02–1.05.
- **Rule:** if w > ~3λ, verify with 2D field simulation (e.g., FEMM, Q2D, or the Ansys/Altium PDN tools).

### 4c. Total AC resistance (example, 100 °C, over a plane)

$$R_{AC,48k} \approx 1.05 \times 3.23 = 3.39\ \text{m}\Omega$$

---

## 5. Characteristic Impedance (Microstrip)

The wide-trace case (w/h = 25 > 1) uses Hammerstad–Jensen.

Effective permittivity:

$$\varepsilon_{eff} = \frac{\varepsilon_r+1}{2} + \frac{\varepsilon_r-1}{2}\frac{1}{\sqrt{1+12h/w}} = 2.65 + \frac{1.65}{1.217} = 4.01$$

Characteristic impedance:

$$Z_0 = \frac{120\pi}{\sqrt{\varepsilon_{eff}}\left[\frac{w}{h} + 1.393 + 0.667\ln\left(\frac{w}{h}+1.444\right)\right]} = \frac{377}{2.00 \times 28.58} \approx 6.6\ \Omega$$

(The IPC-2141 formula 87/√(εᵣ+1.41)·ln(5.98h/(0.8w+t)) is only valid for 0.1 < w/h < 2.0. Do not use it here.)

### Is Z₀ relevant at 48 kHz?

No, not for the 48 kHz content itself. The wavelength is λ = c/(f√ε_eff) ≈ 3.1 km, so the 50 mm trace is electrically tiny. Treat the trace as a lumped R–L element.

Z₀ matters only for the PWM **edges**. The critical length is:

$$\ell_{crit} \approx \frac{t_r \cdot c}{6\sqrt{\varepsilon_{eff}}}$$

For t_r = 20 ns, ℓ_crit ≈ 0.5 m. A 50 mm trace is still lumped, but ringing between the trace L and the device C_oss is the real concern.

---

## 6. Inductance (the parameter that matters)

$$L' = \frac{Z_0\sqrt{\varepsilon_{eff}}}{c} \approx \frac{6.6 \times 2.0}{3\times10^8} = 44\ \text{nH/m}$$

As a check, the parallel-plate approximation gives L' ≈ μ₀h/w = 50 nH/m.

$$L_{trace} \approx 2.2\text{–}2.5\ \text{nH (50 mm)}$$

Reactance at 48 kHz: X_L = 2πfL ≈ 0.7 mΩ, which is comparable to R_AC. The resulting overshoot from L·di/dt at 30 A in 20 ns ≈ 3.75 V.

---

## 7. Power Dissipation

Motor phase current consists of a low-frequency fundamental (the electrical speed) plus a 48 kHz triangular ripple. Split the two:

$$P = I_{rms,LF}^2\,R_{DC} + \sum_n I_{rip,n}^2\,R_{AC}(nf_{sw})$$

For triangular ripple with peak-to-peak ΔI: I_rip,rms = ΔI/(2√3).

**Example** (30 A RMS, ΔI = 6 A p-p):
- LF loss: 30² × 3.23 mΩ = **2.91 W**
- Ripple loss: (1.73)² × 3.39 mΩ = **0.010 W**
- **Total ≈ 2.92 W.** The ripple contribution is about 0.3% of the total.

---

## 8. Conclusions and Checks

1. **At 48 kHz, the 2 oz trace behaves essentially as DC:** t/δ ≈ 0.2 gives F_R ≈ 1.0002 through the thickness.
2. **Width-wise crowding (δ²/t ≈ 1.7 mm) is the only meaningful AC effect.** A solid adjacent return plane minimizes it.
3. **Size the trace by DC/thermal loss.** Use IPC-2152 charts for the temperature rise at I_rms, accounting for internal versus external layers and plane proximity.
4. **Minimize loop inductance, not Z₀.** Use thin dielectric to the return plane and wide traces, and keep the phase/return loop tight.
5. **Paralleling layers works fully** (2 × 2 oz ≈ 2× conductance) because each layer is ≪ δ. Stitch generously with vias, roughly 1–2 A per 0.3 mm via plus a thermal margin.

If you share the actual Cycle 11 values (widths, lengths, stack-up, current profile, ripple), I can rerun the numbers.