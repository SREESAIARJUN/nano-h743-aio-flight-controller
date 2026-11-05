# PCB Trace Impedance and Skin-Effect AC Resistance at 48 kHz: 2 oz Motor Phase Tracks

I don't have your earlier design cycles, so I don't know the actual trace geometry, stackup or current from Cycle 30. Below, every formula is parameterized, and a reference geometry is worked through numerically. Substitute your own values.

---

## 1. Reference Assumptions

| Parameter | Symbol | Value |
|---|---|---|
| Copper weight | 2 oz/ft² | t = 70 µm (0.070 mm) |
| Trace width | w | 5.0 mm |
| Trace length (driver to connector) | ℓ | 50 mm |
| Dielectric height to return plane | h | 1.6 mm (FR-4, 2-layer) |
| Dielectric constant | εr | 4.3 |
| Copper resistivity, 20 °C | ρ₂₀ | 1.72 × 10⁻⁸ Ω·m |
| Temperature coefficient | α | 0.00393 /°C |
| Operating copper temperature | T | 100 °C (worst case) |
| Frequency of interest | f | 48 kHz (PWM carrier) |

The phase current itself is at the motor electrical frequency, typically less than 1 kHz. **48 kHz is the PWM ripple component.** The analysis below applies to the ripple current and its harmonics (96 kHz, 144 kHz, and so on).

---

## 2. DC Resistance

**Formula:**

$$R_{DC} = \frac{\rho \, \ell}{w \, t}, \qquad \rho(T) = \rho_{20}\,[1 + \alpha (T - 20)]$$

**Values for the reference geometry:**

- Cross-section: A = 5 mm × 0.070 mm = 0.35 mm² (3.5 × 10⁻⁷ m²)
- At 20 °C: R_DC = (1.72e-8 × 0.05) / 3.5e-7 = **2.46 mΩ**
- At 100 °C, the correction factor is 1 + 0.00393 × 80 = 1.314, giving R_DC = **3.23 mΩ**

---

## 3. Skin Depth at 48 kHz

**Formula:**

$$\delta = \sqrt{\frac{\rho}{\pi f \mu_0}}$$

**Values:**

- At 20 °C: δ = √(1.72e-8 / (π × 48 000 × 4π × 10⁻⁷)) = √(9.08 × 10⁻⁸) = **0.301 mm**
- At 100 °C: δ = 0.301 × √1.314 = **0.345 mm**

**Key ratio:**

$$\Delta = \frac{t}{\delta} = \frac{0.070}{0.301} \approx 0.23$$

The 2 oz copper is about 4× thinner than one skin depth, so current is essentially uniform through the thickness.

---

## 4. AC Resistance Factor

Use the Dowell foil-conductor form for a single layer:

$$F_R = \frac{R_{AC}}{R_{DC}} = \Delta \,\frac{\sinh 2\Delta + \sin 2\Delta}{\cosh 2\Delta - \cos 2\Delta} \;\approx\; 1 + \frac{4\Delta^4}{45} \quad (\Delta < 1)$$

**Values by harmonic** (20 °C):

| Harmonic | f | δ | Δ | F_R |
|---|---|---|---|---|
| 1st | 48 kHz | 0.301 mm | 0.233 | **1.0003** |
| 2nd | 96 kHz | 0.213 mm | 0.329 | 1.0010 |
| 3rd | 144 kHz | 0.174 mm | 0.402 | 1.0023 |

**Result:**

- R_AC(48 kHz) ≈ R_DC: 2.46 mΩ at 20 °C, **3.23 mΩ at 100 °C**
- Thickness skin effect is **negligible**.

**Edge crowding across the width.** Current concentrates toward the trace edges, and toward the side facing the return path. At 48 kHz this effect is governed by inductance, not by δ alone. For w/h ≈ 3 it typically adds **5–15 %**. Budget F_R ≈ 1.1 to be conservative, or confirm with a 2D field solver (FEMM, Q2D).

**Proximity effect** becomes significant only if the phase traces run stacked or closely paralleled on adjacent layers with opposing currents.

---

## 5. Characteristic Impedance (Microstrip)

**Formula** (IPC-2141; it is approximate for wide traces, so cross-check with Hammerstad):

$$Z_0 = \frac{87}{\sqrt{\varepsilon_r + 1.41}} \ln\!\left(\frac{5.98\,h}{0.8\,w + t}\right)$$

**Values:**

- Z₀ = (87 / 2.39) × ln(9.568 / 4.07) = 36.4 × 0.855 ≈ **31 Ω**

**Effective permittivity** (Hammerstad):

$$\varepsilon_{eff} = \frac{\varepsilon_r+1}{2} + \frac{\varepsilon_r-1}{2}\left(1+\frac{12h}{w}\right)^{-1/2}$$

- ε_eff = 2.65 + 1.65 × 0.455 ≈ **3.40**

**Relevance check.** Z₀ does not matter for transmission-line behaviour here:

| Quantity | Value | Compared with 50 mm trace |
|---|---|---|
| Wavelength at 48 kHz | λ ≈ 3.4 km | Electrically tiny |
| Knee frequency for t_r = 50 ns edges | 0.35 / t_r ≈ 7 MHz | — |
| Wavelength at 7 MHz | λ ≈ 23 m | Electrically tiny |

The trace is **lumped**. Z₀ is useful only to derive the inductance.

---

## 6. Trace Inductance and Reactance (the Dominant Term)

**Formula:**

$$L' = \frac{Z_0 \sqrt{\varepsilon_{eff}}}{c}, \qquad L = L'\,\ell, \qquad X_L = 2\pi f L$$

**Values:**

- Per-length inductance: L' = 31 × 1.844 / 3 × 10⁸ ≈ **1.9 nH/mm**
- For 50 mm: L ≈ **96 nH**
- Reactance: X_L(48 kHz) = 2π × 48 000 × 96 nH ≈ **29 mΩ**

## 7. Total Ripple-Path Impedance

$$|Z| = \sqrt{R_{AC}^2 + X_L^2} = \sqrt{3.23^2 + 29^2} \approx 29.2\ \text{m}\Omega$$

**The inductive reactance is about 9× the resistance.** At 48 kHz the track behaves inductively, not resistively.

---

## 8. Loss and Thermal Check

**Power loss:**

$$P = I_{RMS,fund}^2 R_{DC} + \sum_n I_{n}^2 R_{AC}(f_n)$$

**Worked example.** Take 30 A_RMS phase current with 15 % peak-to-peak ripple:

1. Triangular ripple RMS: I_ripple,RMS ≈ 4.5 / √12 ≈ 1.3 A
2. Fundamental loss: 30² × 3.23 mΩ = **2.91 W**
3. Ripple loss: 1.3² × 3.23 mΩ ≈ 5.5 mW (negligible)

**Thermal check:**

- 2.9 W in a 5 mm × 50 mm trace is high.
- Per IPC-2152, a 5 mm, 2 oz external trace carrying 30 A gives a ΔT well above 40 °C.
- **Widen to 10 mm or more**, add a parallel layer with via stitching, or use bus bars or solder-built-up tracks.

---

## 9. Design Conclusions

1. **Skin effect at 48 kHz is irrelevant for 2 oz copper** (t/δ ≈ 0.23, F_R ≈ 1.0003). Size the traces by DC/I²R and IPC-2152 thermal limits.
2. **Inductance dominates the ripple-path impedance.** Minimize loop area:
   - Route phase traces over a solid plane, or pair them with their return path.
   - Keep the DC-link capacitor to switch-node loop tight. Its inductance matters far more (ringing at MHz).
3. **Thicker copper (3–4 oz) gives a near-linear DC benefit** up to t ≈ δ (~300 µm) before skin effect starts to limit it.
4. **Validate the edge-crowding factor** with a 2D solver if the loss budget is within 10 %.

If you share the actual Cycle 30 parameters (width, length, stackup, phase current and ripple), I'll rerun the numbers specifically.