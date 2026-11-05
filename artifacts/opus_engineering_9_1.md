# PCB Phase-Track Impedance and AC Resistance at 48 kHz (2 oz Cu)

I don't have the details of your "Cycle 9" design: track geometry, stack-up, current levels or PWM scheme. Below is the full formulation with a worked example using placeholder values. Replace the placeholders with your actual numbers.

**Placeholder parameters (edit these):**

| Symbol | Meaning | Example value |
|---|---|---|
| t | Copper thickness (2 oz) | 68.6 µm (≈ 70 µm finished) |
| w | Track width | 5.0 mm |
| l | Track length | 50 mm |
| h | Dielectric height to the nearest plane | 0.2 mm |
| εr | FR-4 permittivity | 4.4 |
| ρ₂₀ | Copper resistivity at 20 °C | 1.72×10⁻⁸ Ω·m |
| α | Temperature coefficient | 0.00393 /°C |
| f_sw | Switching frequency | 48 kHz |

---

## 1. DC Resistance (Baseline)

**Formulas:**

R_DC(T) = ρ₂₀ · [1 + α(T − 20)] · l / (w · t)

**Example:**

- At 20 °C: R_DC = 1.72e-8 × 0.05 / (0.005 × 68.6e-6) = **2.51 mΩ**
- At 100 °C: the factor is 1.314, so R_DC = **3.30 mΩ**

## 2. Skin Depth at 48 kHz

**Formula:**

δ = √( ρ / (π · f · μ₀) )

**Results:**

| Temperature | ρ (Ω·m) | δ at 48 kHz |
|---|---|---|
| 20 °C | 1.72×10⁻⁸ | **0.301 mm** |
| 100 °C | 2.26×10⁻⁸ | **0.345 mm** |

**Comparison with the copper thickness:** t/δ ≈ 70 µm / 301 µm ≈ 0.23. The copper is about 4× thinner than one skin depth, so current is essentially uniform through the thickness.

## 3. AC Resistance Factor (Dowell / Foil Form)

Use the normalized thickness Δ = t/δ.

**Exact form:**

R_AC / R_DC = Δ · [sinh(2Δ) + sin(2Δ)] / [cosh(2Δ) − cos(2Δ)]

**Small-Δ approximation:**

R_AC / R_DC ≈ 1 + Δ⁴/45

**Result:** With Δ = 0.23, F_R ≈ 1 + 6×10⁻⁵, which is negligible.

**PWM harmonics:** The n-th harmonic has δₙ = δ/√n.

| Harmonic | Frequency | Δₙ | F_R |
|---|---|---|---|
| 1st | 48 kHz | 0.23 | ≈ 1.000 |
| 5th | 240 kHz | 0.52 | ≈ 1.002 |
| 21st | ~1 MHz | 1.05 | ≈ 1.05 |

**Total AC loss:**

P_AC = Σₙ Iₙ²_rms · R_DC · F_R(n)

**Caveats:**
- **Width-direction current crowding (edge effect)** and **proximity effect** from adjacent phases or return planes dominate over skin effect here. Expect roughly 1.05–1.3× for closely spaced parallel phase tracks. Verify with 2D FEM (FEMM, Q3D) if the phases are paralleled or routed over a plane.
- **Centre-aligned PWM:** phase-to-phase ripple appears at 2·f_sw = 96 kHz. Then δ ≈ 0.213 mm, which is still much greater than t, so the conclusions are unchanged.

## 4. "Impedance" at 48 kHz: Use the Lumped Model, Not Z₀

**Why Z₀ doesn't apply:** The wavelength in FR-4 is λ ≈ c / (f·√εeff) ≈ 3 km, so a 50 mm track is electrically tiny. The relevant quantity is the lumped series impedance:

Z = R_AC + jωL

**Inductance, isolated track (no adjacent plane):**

L ≈ 0.2 · l · [ ln(2l/(w+t)) + 0.5 + 0.2235·(w+t)/l ] µH (l in metres)

- Example: L ≈ 0.2 × 0.05 × 3.505 ≈ **35 nH**

**Inductance, wide track over a plane (w ≫ h):**

L ≈ μ₀ · h · l / w

- Example: L ≈ 1.257e-6 × 0.2e-3 × 0.05 / 0.005 ≈ **2.5 nH**

**Reactance at 48 kHz:**

| Configuration | X_L | Compared with R_DC |
|---|---|---|
| Isolated track | **10.6 mΩ** | ≈ 4× R_DC |
| Over plane | **0.75 mΩ** | < R_DC |

**Key finding:** At 48 kHz, inductance dominates the track impedance unless the phase track has a tightly coupled return path. This matters for ringing on switching edges (L·di/dt) more than for I²R loss.

**Reference only (high-frequency edge content):** The microstrip Z₀ for w/h > 1 (Hammerstad) is

- Z₀ = 120π / { √εeff · [w/h + 1.393 + 0.667·ln(w/h + 1.444)] }
- εeff = (εr+1)/2 + (εr−1)/2 · (1 + 12h/w)^(−½)

For the example (w/h = 25), Z₀ ≈ 5–6 Ω. This is only relevant to dV/dt edge ringing in the tens-of-MHz range.

## 5. Loss and Temperature Rise Check

**Total loss:**

P = I²_rms,fund · R_DC(T) + Σₙ Iₙ² · R_DC(T) · F_R(n)

**Temperature rise:** Use IPC-2152 charts (not IPC-2221) for the allowable ΔT at your current, with 2 oz copper and plane/board conditions. Then iterate R_DC(T) with the resulting temperature.

---

## Summary

- **Skin effect:** At 48 kHz, δ ≈ 0.30–0.35 mm, which is much greater than the 70 µm copper, so R_AC ≈ R_DC. Skin effect is negligible.
- **Losses:** These are dominated by DC/fundamental resistance at operating temperature. Watch proximity and edge effects if the phases run close together.
- **Impedance:** Model the track as R + jωL, where L is set mainly by return-path geometry. Z₀ matters only for switching-edge harmonics.

If you share the actual Cycle 9 track widths, lengths, stack-up and phase-current RMS/ripple, I can work through the concrete numbers.