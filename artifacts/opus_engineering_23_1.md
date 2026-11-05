# PCB Motor Phase Trace: Impedance and AC Resistance at 48 kHz (2 oz Cu)

**Note on "Cycle 23":** I don't have your project's Cycle 23 data (track geometry, current, stackup, temperature limits). The method below is complete. The worked numbers use placeholder values, marked **[assumed]**, so you can substitute your own.

---

## 1. Inputs and Assumptions

| Parameter | Symbol | Value |
|---|---|---|
| Copper weight | — | 2 oz/ft² |
| Copper thickness | t | 70 µm (≈2.8 mil; finished thickness is often 60–68 µm, so check with your fab) |
| Trace width | w | 10 mm **[assumed]** |
| Trace length | l | 50 mm **[assumed]** |
| Frequency | f | 48 kHz (PWM fundamental) |
| Resistivity at 20 °C | ρ₂₀ | 1.72 × 10⁻⁸ Ω·m |
| Temperature coefficient | α | 0.00393 /K |
| Operating temperature | T | 100 °C **[assumed]** |
| Phase current | I_rms | 30 A **[assumed]** |
| Dielectric to return plane | h | 0.2 mm **[assumed]** |

---

## 2. Resistivity at Temperature

$$\rho_T = \rho_{20}\,[1+\alpha(T-20)]$$

$$\rho_{100} = 1.72\times10^{-8}\,(1 + 0.00393\times80) = 2.26\times10^{-8}\ \Omega\cdot\text{m}$$

---

## 3. Skin Depth

$$\delta = \sqrt{\frac{\rho}{\pi f \mu_0}}$$

With $\pi f \mu_0 = \pi(48\,000)(4\pi\times10^{-7}) = 0.1895$:

| Condition | δ |
|---|---|
| 20 °C, 48 kHz | **301 µm** |
| 100 °C, 48 kHz | **345 µm** |
| 100 °C, 144 kHz (3rd harmonic) | 199 µm |
| ~1.2 MHz | ≈ 70 µm (equal to t) |

**Key observation:** δ is much greater than t (301 µm vs 70 µm). The current fully penetrates the copper thickness, so classic skin effect is negligible at 48 kHz.

---

## 4. DC Resistance

$$R_{DC} = \frac{\rho\, l}{w\, t}$$

- At 20 °C: $R_{DC} = \dfrac{1.72\times10^{-8}\times0.05}{0.01\times70\times10^{-6}} = 1.23\ \text{m}\Omega$
- At 100 °C: **1.61 mΩ**

---

## 5. AC Resistance (Skin Effect)

For a flat conductor of thickness t, define the normalized thickness Δ = t/δ. The Dowell single-layer factor (no proximity effect) is:

$$F_R = \frac{R_{AC}}{R_{DC}} = \Delta\,\frac{\sinh 2\Delta + \sin 2\Delta}{\cosh 2\Delta - \cos 2\Delta}$$

For small Δ this simplifies to:

$$F_R \approx 1 + \frac{\Delta^4}{45}$$

At 20 °C, Δ = 70/301 = 0.232. Then Δ⁴ = 2.9×10⁻³, so:

$$F_R \approx 1.00007 \quad\Rightarrow\quad R_{AC} \approx R_{DC} = 1.61\ \text{m}\Omega \text{ at } 100\,°\text{C}$$

**Caveats:**
- **Edge and proximity crowding across the width.** Because w is much greater than δ, current can crowd toward trace edges, or toward the adjacent conductor when the return path is close (e.g., a parallel opposite phase or a plane). This typically adds about 5–20%. Quantify it with a 2D field solver (FEMM, Q2D, or Ansys) if margins are tight.
- **Ripple harmonics.** Sum the loss per harmonic, $P = \sum I_n^2 R_{AC}(f_n)$. F_R stays below about 1.05 up to roughly 500 kHz for 2 oz copper.

---

## 6. Trace Impedance

At 48 kHz, the wavelength on FR-4 is about $c/(f\sqrt{\varepsilon_{eff}}) \approx 3$ km. The trace is therefore electrically tiny, so characteristic impedance Z₀ is irrelevant. Treat the trace as a lumped series impedance:

$$Z = R_{AC} + j\omega L$$

**(a) Isolated flat trace (partial self-inductance, Grover/Rosa formula; l and w+t in cm):**

$$L\,[\text{nH}] = 2l\left[\ln\frac{2l}{w+t} + 0.5 + 0.2235\,\frac{w+t}{l}\right]$$

$$L = 2(5)\,[2.296 + 0.5 + 0.045] = 28.4\ \text{nH}$$

$$X_L = 2\pi(48\,000)(28.4\times10^{-9}) = 8.6\ \text{m}\Omega$$

**(b) Trace over a tight return plane (wide-strip approximation):**

$$L' \approx \frac{\mu_0 h}{w} = 25\ \text{nH/m} \quad\Rightarrow\quad L = 1.26\ \text{nH}, \qquad X_L \approx 0.38\ \text{m}\Omega$$

**Interpretation:** With a loose return path, reactance (≈8.6 mΩ) dominates resistance (1.6 mΩ) by about 5×. The motor phase current returns through the other phases, not a plane, so the true loop inductance depends on phase-pair spacing. Route U/V/W pairs tightly, or overlap them on adjacent layers.

$$|Z| = \sqrt{1.61^2 + 8.6^2} \approx 8.75\ \text{m}\Omega \quad \text{(case a, 100 °C)}$$

---

## 7. Loss and Thermal Check

$$P = I_{rms}^2 R_{AC} = 30^2 \times 1.61\times10^{-3} = 1.45\ \text{W}$$

Verify the temperature rise against the IPC-2152 charts. A 10 mm wide 2 oz trace at 30 A gives roughly a 20–30 °C rise on an external layer, depending on the board copper and planes. Iterate T in step 2 until it is self-consistent with the rise.

---

## 8. Summary Formulas

| Quantity | Expression | Example result |
|---|---|---|
| Skin depth | $\delta=\sqrt{\rho/(\pi f\mu_0)}$ | 301–345 µm |
| DC resistance | $R_{DC}=\rho l/(wt)$ | 1.61 mΩ |
| AC factor | $F_R \approx 1+(t/\delta)^4/45$ | ≈1.000 |
| Inductance (isolated) | Grover formula, §6(a) | 28 nH |
| Reactance | $X_L=2\pi fL$ | 8.6 mΩ |
| Loss | $I^2R_{AC}$ | 1.45 W |

**Conclusion:** At 48 kHz, skin effect in 2 oz copper is negligible (R_AC ≈ R_DC). Design effort is better spent on:
1. Minimizing loop inductance through phase-pair routing.
2. Thermal derating per IPC-2152.
3. Checking proximity crowding with a field solver.

If you share the actual Cycle 23 track widths, lengths, current profile, and stackup, I can recompute these values specifically.