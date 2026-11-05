# PCB Motor Phase Trace Analysis: Impedance and AC Resistance at 48 kHz (2 oz Cu)

I don't have your Cycle 16 design data (trace widths, lengths, stackup, current levels). The formulation below is fully parameterized, with a worked example using placeholder values. Substitute your actual numbers.

---

## 1. Base Parameters

| Parameter | Symbol | Value |
|---|---|---|
| Copper thickness (2 oz) | t | 69.6 µm (2.74 mil) |
| Resistivity @ 20 °C | ρ₂₀ | 1.72 × 10⁻⁸ Ω·m |
| Temp. coefficient | α | 0.00393 /°C |
| Permeability | µ₀ | 4π × 10⁻⁷ H/m |
| PWM frequency | f | 48 kHz |

Temperature-corrected resistivity:

$$\rho(T) = \rho_{20}\,[1 + \alpha (T - 20)]$$

At 100 °C: ρ ≈ 2.26 × 10⁻⁸ Ω·m (×1.31).

---

## 2. Skin Depth

$$\delta = \sqrt{\frac{\rho}{\pi f \mu_0}}$$

| Condition | δ at 48 kHz |
|---|---|
| 20 °C | **301 µm** |
| 100 °C | **345 µm** |

The ratio of thickness to skin depth is Δ = t/δ = 69.6/301 ≈ **0.23**.

Because t ≪ 2δ, current penetrates the full copper thickness. Through-thickness skin effect is negligible.

---

## 3. AC Resistance

### 3a. DC resistance

$$R_{DC} = \frac{\rho(T)\, \ell}{w\, t}$$

### 3b. Skin-effect correction (Dowell form, single-layer foil)

$$\frac{R_{AC}}{R_{DC}} = \Delta\,\frac{\sinh 2\Delta + \sin 2\Delta}{\cosh 2\Delta - \cos 2\Delta} \;\approx\; 1 + \frac{\Delta^4}{45} \quad (\Delta < 1)$$

With Δ = 0.23, the factor is ≈ 1 + 6 × 10⁻⁵. **Effectively R_AC = R_DC at 48 kHz.**

### 3c. Effects that actually matter

1. **Edge current crowding (width direction).** The trace width w ≫ δ, so current concentrates at the edges. Typical penalty for an isolated wide strip is 5–20%. Use a field solver (FEMM, Q3D, Sonnet) if precision matters.
2. **Proximity effect.** Adjacent phase traces or an underlying return plane redistribute current. For stacked or parallel phase traces with Δ < 1, the Dowell multi-layer term is ≈ 1 + (5m² − 1)Δ⁴/45, where m is the number of layers. This is still small at 48 kHz.
3. **Harmonics.** The PWM ripple contains 48, 96, 144 kHz components and higher. Evaluate R_AC per harmonic and sum the losses:

$$P_{AC} = \sum_n I_{n,\text{rms}}^2 \, R_{AC}(n f)$$

At 480 kHz (10th harmonic), δ ≈ 95 µm and Δ ≈ 0.73. That still gives only about 1.6% increase, but crowding becomes stronger.

**Loss split:**

$$P = I_{\text{fund,rms}}^2 R_{DC} + \sum_n I_{n}^2 R_{AC}(nf)$$

The motor fundamental (low Hz) sees pure R_DC.

---

## 4. Impedance

### 4a. Characteristic impedance (surface microstrip, Hammerstad, w/h ≥ 1)

$$\varepsilon_{\text{eff}} = \frac{\varepsilon_r+1}{2} + \frac{\varepsilon_r-1}{2}\left(1+\frac{12h}{w}\right)^{-1/2}$$

$$Z_0 = \frac{120\pi}{\sqrt{\varepsilon_{\text{eff}}}\left[\frac{w}{h} + 1.393 + 0.667\ln\left(\frac{w}{h}+1.444\right)\right]}$$

Use w_eff = w + (t/π)[1 + ln(2h/t)] for thickness correction.

Note: at 48 kHz the wavelength is about 3 km, so the trace is electrically tiny and Z₀ is irrelevant to the PWM fundamental. Z₀ only matters for switch-node edge content (tens of MHz) and for EMI/ringing analysis.

### 4b. The relevant 48 kHz impedance is lumped R + jωL

Partial self-inductance of an isolated rectangular trace (ℓ, w, t in cm, result in nH):

$$L \approx 2\ell\left[\ln\frac{2\ell}{w+t} + 0.5 + 0.2235\frac{w+t}{\ell}\right]$$

Trace over a solid return plane at spacing h (w ≫ h):

$$L \approx \frac{\mu_0\, h\, \ell}{w}$$

The total impedance is then:

$$|Z| = \sqrt{R_{AC}^2 + (2\pi f L)^2}$$

---

## 5. Worked Example (placeholder geometry)

Assumed geometry: w = 10 mm, ℓ = 50 mm, 2 oz, FR-4 (εr = 4.3), h = 0.2 mm to plane.

| Quantity | Result |
|---|---|
| R_DC @ 20 °C | 1.24 mΩ |
| R_DC @ 100 °C | 1.62 mΩ |
| R_AC/R_DC (skin, 48 kHz) | ≈ 1.0001 |
| R_AC incl. ~10% edge crowding @ 100 °C | ~1.8 mΩ |
| L (isolated, no plane) | 28.4 nH → X_L = **8.6 mΩ** |
| L (over plane, h = 0.2 mm) | 1.26 nH → X_L = 0.38 mΩ |
| Z₀ (microstrip) | ≈ 3.4 Ω |
| Conduction loss @ 30 A_rms, 100 °C | ≈ 1.6 W |

**Key insight:** for an isolated trace, the 48 kHz reactance (8.6 mΩ) exceeds the resistance by roughly 5×. Layout inductance, not skin effect, dominates the AC impedance. Routing phase traces tightly over or alongside their return path is the high-leverage design choice.

---

## 6. Calculation Script

```python
import numpy as np
mu0, rho20, a = 4e-7*np.pi, 1.72e-8, 0.00393

def phase_trace(w, l, t=69.6e-6, f=48e3, T=100, h=None, er=4.3, nharm=10):
    rho   = rho20*(1 + a*(T-20))
    Rdc   = rho*l/(w*t)
    out = {"Rdc": Rdc}
    for n in range(1, nharm+1):
        d  = np.sqrt(rho/(np.pi*n*f*mu0)); D = t/d
        Fr = D*(np.sinh(2*D)+np.sin(2*D))/(np.cosh(2*D)-np.cos(2*D))
        out[f"Rac_{n*f/1e3:.0f}k"] = Rdc*Fr
    lc, wc = l*100, (w+t)*100
    L = (mu0*h*l/w) if h else 2*lc*(np.log(2*lc/wc)+0.5+0.2235*wc/lc)*1e-9
    out["L"] = L; out["XL_48k"] = 2*np.pi*f*L
    if h:
        ee = (er+1)/2 + (er-1)/2/np.sqrt(1+12*h/w)
        out["Z0"] = 120*np.pi/(np.sqrt(ee)*(w/h+1.393+0.667*np.log(w/h+1.444)))
    return out

print(phase_trace(w=10e-3, l=50e-3, h=0.2e-3))
```

---

## 7. Cross-Checks for Cycle 16

- **Thermal:** size the width by IPC-2152 (not IPC-2221) for the rms current at the allowed ΔT, then verify the loss above.
- **Field-solver validation:** run one FEMM or Q3D case to calibrate the edge-crowding factor for your actual geometry.
- **Bidirectional coupling:** check mutual inductance between the three phases if they are routed in parallel.
- **Vias and transitions:** include via R and L (stitching arrays) in the total phase path.

If you share the Cycle 16 trace geometry, stackup, and phase current waveform (rms and ripple), I can produce the specific numbers.