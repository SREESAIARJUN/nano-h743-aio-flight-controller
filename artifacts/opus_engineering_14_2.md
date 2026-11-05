# PCB Trace Impedance and Skin-Effect AC Resistance at 48 kHz: 2 oz Motor Phase Tracks

I don't have your Cycle 14 design data (track geometry, stackup, current profile). So this gives the general formulation, then a worked example using assumed values you can replace.

## 1. Assumed Parameters (replace with Cycle 14 values)

| Parameter | Symbol | Assumed value |
|---|---|---|
| Copper weight | – | 2 oz/ft² → **t = 70 µm** (finished; nominal 68.6 µm) |
| Track width | w | 10 mm |
| Track length | ℓ | 50 mm |
| Dielectric height to return plane | h | 0.2 mm (L1→L2 prepreg) |
| Dielectric constant (FR-4) | εr | 4.4 |
| Cu resistivity at 20 °C | ρ₂₀ | 1.72 × 10⁻⁸ Ω·m |
| Temperature coefficient | α | 0.00393 /°C |
| Operating copper temperature | T | 100 °C (worst case) |
| Switching / ripple frequency | f | 48 kHz |
| Phase current | I_rms | 30 A (fundamental) |

## 2. DC Resistance (Baseline)

$$R_{DC}(T) = \frac{\rho_{20}\,[1+\alpha(T-20)]\,\ell}{w\,t}$$

- At 20 °C: R_DC = (1.72e-8 × 0.05)/(0.010 × 70e-6) = **1.23 mΩ**
- At 100 °C: ρ = 2.26 × 10⁻⁸ Ω·m, so R_DC = **1.62 mΩ**

## 3. Skin Depth at 48 kHz

$$\delta = \sqrt{\frac{\rho}{\pi f \mu_0}}, \qquad \mu_0 = 4\pi\times10^{-7}\ \text{H/m}$$

| Condition | δ |
|---|---|
| 20 °C, 48 kHz | **301 µm** |
| 100 °C, 48 kHz | **345 µm** |

**Key ratio:** Δ = t/δ = 70/301 ≈ **0.23** (20 °C) or 0.20 (100 °C).

The copper is about 4–5× thinner than the skin depth. Current is therefore essentially uniform through the thickness at 48 kHz.

## 4. AC Resistance Formulation

### 4a. Thickness-direction skin effect (Dowell, single layer, m = 1)

$$F_R = \frac{R_{AC}}{R_{DC}} = \Delta\,\frac{\sinh 2\Delta + \sin 2\Delta}{\cosh 2\Delta - \cos 2\Delta} \;\approx\; 1 + \frac{4}{45}\Delta^4 \quad (\Delta \ll 1)$$

$$F_R = 1 + 0.0889 \times (0.23)^4 = 1 + 0.00025 \approx \mathbf{1.0003}$$

So R_AC(48 kHz) ≈ 1.62 mΩ at 100 °C. The increase is negligible.

### 4b. PWM harmonics

Ripple current is not sinusoidal, so evaluate each harmonic n separately:

$$\delta_n = \delta_1/\sqrt{n}, \qquad P_{AC} = \sum_n I_n^2\,R_{DC}\,F_R(\Delta\sqrt{n})$$

| Harmonic | f | δ (20 °C) | Δ | F_R |
|---|---|---|---|---|
| 1 | 48 kHz | 301 µm | 0.23 | 1.0003 |
| 3 | 144 kHz | 174 µm | 0.40 | 1.002 |
| 5 | 240 kHz | 135 µm | 0.52 | 1.007 |
| 9 | 432 kHz | 100 µm | 0.70 | 1.02 |

Even at the 9th harmonic, the penalty is about 2%. Thickness skin effect is **not** a design driver for 2 oz copper at 48 kHz.

### 4c. Lateral crowding and proximity effect (the real AC effect)

Over a solid return plane, HF current concentrates on the underside of the track and toward the edges. The width-direction distribution is governed by h/w and inductance, not by δ versus t.

- For w/h = 50 with an adjacent plane, edge crowding adds roughly **2–8%** at the ripple frequency. Confirm with a 2D field solver (e.g., Ansys Q2D, or the FEMM magnetics module).
- Adjacent phase tracks with spacing s < w add proximity loss. Keep s ≥ 2h or interleave return paths.
- Include the **return-plane resistance**, which is often comparable to the trace's own resistance.

**Engineering estimate:**

$$R_{AC,48k} \approx R_{DC}(T)\times(1.03\text{–}1.08) \approx \mathbf{1.67\text{–}1.75\ m\Omega}$$

## 5. Trace Impedance

### 5a. Characteristic impedance (microstrip, Hammerstad; valid for wide traces)

$$\varepsilon_{eff} = \frac{\varepsilon_r+1}{2} + \frac{\varepsilon_r-1}{2}\left(1+\frac{12h}{w}\right)^{-1/2} = 2.7 + 1.7(1.24)^{-1/2} = \mathbf{4.23}$$

$$Z_0 = \frac{120\pi}{\sqrt{\varepsilon_{eff}}\left[\frac{w}{h} + 1.393 + 0.667\ln\left(\frac{w}{h}+1.444\right)\right]} = \frac{377}{2.056 \times 54.02} \approx \mathbf{3.4\ \Omega}$$

**Caveat:** at 48 kHz the wavelength in FR-4 is about 3 km. A 50 mm track is electrically tiny, so Z₀ has no transmission-line significance here. Use the **lumped impedance** in 5c instead. Z₀ only matters for switching-edge ringing (tens of MHz).

### 5b. Inductance

Over a plane:

$$L' = \frac{Z_0\sqrt{\varepsilon_{eff}}}{c} = \frac{3.4 \times 2.056}{3\times10^8} \approx 23\ \text{nH/m}$$

$$L = 23\ \text{nH/m} \times 0.05\ \text{m} \approx \mathbf{1.2\ nH}$$

Without a return plane (partial self-inductance of a flat strip, ℓ and w in cm):

$$L \approx 2\ell\left[\ln\frac{2\ell}{w+t} + 0.5 + 0.2235\frac{w+t}{\ell}\right]\ \text{nH} \approx \mathbf{28\ nH}$$

### 5c. Lumped impedance at 48 kHz

$$Z = R_{AC} + j\omega L$$

| Configuration | R_AC | X_L = 2πfL | \|Z\| |
|---|---|---|---|
| With adjacent plane | 1.70 mΩ | 0.35 mΩ | **1.74 mΩ** |
| No plane (loop return) | 1.70 mΩ | 8.6 mΩ | **8.8 mΩ** |

A plane-referenced layout keeps the track resistive. Without one, inductance dominates, causing voltage overshoot (L·di/dt) and radiated EMI.

## 6. Loss and Thermal Check

$$P = I_{rms,fund}^2 R_{DC} + \sum_n I_{n}^2 R_{AC,n}$$

With 30 A fundamental plus about 3 A rms of 48 kHz ripple:

$$P \approx 30^2(1.62\,\text{m}\Omega) + 3^2(1.72\,\text{m}\Omega) \approx 1.46\ \text{W} + 0.015\ \text{W} \approx \mathbf{1.47\ W}$$

Verify the temperature rise against **IPC-2152** charts for 2 oz, 10 mm, at your board thickness and plane coverage. Iterate T, since R_DC depends on it.

## 7. Summary for Cycle 14

1. **Skin effect at 48 kHz is negligible** for 2 oz copper (t/δ ≈ 0.2, F_R ≈ 1.0003). There is no benefit from going to thicker copper for AC reasons; thickness helps only DC and thermal performance.
2. **The dominant AC penalty is lateral crowding/proximity (≈ 3–8%)** plus return-path resistance. Size for R_DC at hot temperature with about a 10% AC margin.
3. **Inductance, not Z₀, is the relevant impedance.** Route phase tracks over a solid plane, or as tightly coupled go/return pairs, to cut L by about 20×.
4. **Design target:** R_phase ≈ 1.6–1.75 mΩ per 50 mm at 100 °C. Scale linearly with ℓ/w.

If you share the actual Cycle 14 stackup, track dimensions, current waveform, and ambient temperature, I can recompute these with your numbers and the harmonic spectrum of your PWM scheme.