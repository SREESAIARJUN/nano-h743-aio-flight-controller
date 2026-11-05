# Thermal Model of a 4-in-1 ESC at 80 A Total Burst Current

I don't have the specific "Cycle 26" design files, so this formulation uses an explicit, typical baseline. Every parameter is listed in §0 so you can substitute your actual values and re-run the same equations.

---

## 0. Baseline Assumptions

| Parameter | Symbol | Value |
|---|---|---|
| Board | — | 30.5 × 30.5 mm mount pattern, ~36 × 36 mm outline, 1.6 mm FR-4, 4 layers × 2 oz Cu |
| Bus voltage | V_bus | 22.2 V (6S nominal) |
| Total burst current | I_tot | 80 A, so I = 20 A per channel |
| MOSFETs | — | 24 total (6 per channel), R_ds(on),25 = 1.5 mΩ |
| R_ds(on) temperature coefficient | α_R | 0.006 K⁻¹ |
| Rise + fall time | t_r + t_f | 20 ns |
| PWM frequency | f_sw | 48 kHz |
| Dead time | t_dt | 100 ns |
| Reverse-recovery charge | Q_rr | 40 nC |
| Gate charge | Q_g | 50 nC at 10 V |
| Ambient temperature | T_∞ | 25 °C |
| Convection coefficient, prop-wash | h_c | 80 W/m²K |
| Convection coefficient, still air | h_c | 10 W/m²K |
| Emissivity (solder mask) | ε | 0.9 |
| Board + component mass | m | ~10 g, effective c_p ≈ 700 J/kg·K |

**Commutation scheme:** 6-step commutation with complementary (synchronous) PWM. At any instant two switches carry the phase current.

---

## 1. Loss Physics (Heat Sources)

### 1.1 MOSFET conduction loss (temperature-dependent)
$$
R_{ds}(T)=R_{25}\,[1+\alpha_R (T_j-25)]
$$
$$
P_{cond,ch}=2\,I^2 R_{ds}(T)
$$
- At 25 °C: P = 2·20²·1.5 mΩ = **1.20 W per channel**.
- At ~100 °C this rises to ~1.74 W, which is the positive feedback that drives thermal runaway.

### 1.2 Switching loss (one PWM-ing leg per channel)
$$
P_{sw}=\tfrac12 V_{bus} I (t_r+t_f) f_{sw}=\tfrac12(22.2)(20)(20\text{ ns})(48\text{ kHz})=0.213\text{ W}
$$

### 1.3 Reverse recovery and C_oss loss
$$
P_{rr}=Q_{rr}V_{bus}f_{sw}=0.043\text{ W},\qquad P_{oss}=\tfrac12C_{oss}V^2f_{sw}\approx0.012\text{ W}
$$

### 1.4 Body-diode dead-time conduction (two edges per period)
$$
P_{dt}=2\,V_F\,I\,t_{dt}\,f_{sw}=2(0.8)(20)(100\text{ ns})(48\text{ kHz})=0.154\text{ W}
$$

### 1.5 Gate drive
$$
P_g=Q_gV_{gs}f_{sw}\cdot 2\approx0.048\text{ W}
$$
This loss is mostly deposited in the driver and gate resistors.

### 1.6 Copper and shunt Joule heating
$$
q'''_{Cu}=\rho_{Cu}(T)\,|\mathbf J|^2,\qquad \rho_{Cu}(T)=1.72\times10^{-8}[1+0.0039(T-20)]
$$
- Per-channel traces and shunt: ≈0.2 W.
- Shared 80 A battery bus (≈0.2 mΩ): 80²·0.2 mΩ ≈ **1.28 W**.
- BEC/buck regulator and 4 MCUs: ≈0.3 W.

### 1.7 Total heat generation
$$
P_{tot}(T)=\underbrace{9.0\text{ W}}_{T\text{-independent part}}+\underbrace{4.8\,\alpha_R}_{0.0288\text{ W/K}}(T-25)
$$

| Source | Per channel | ×4 / shared |
|---|---|---|
| Conduction (25 °C) | 1.20 W | 4.80 W |
| Switching + Q_rr + C_oss | 0.27 W | 1.07 W |
| Dead time | 0.15 W | 0.62 W |
| Gate | 0.05 W | 0.19 W |
| Traces/shunt | 0.20 W | 0.80 W |
| Battery bus | — | 1.28 W |
| BEC + MCUs | — | 0.30 W |
| **Total (25 °C)** | | **≈ 9.1 W** |

---

## 2. Governing Conduction Equation

### 2.1 Full 3D anisotropic form
$$
\rho c_p \frac{\partial T}{\partial t}=\nabla\cdot(\mathbf{k}\nabla T)+q'''(\mathbf x,T),\qquad 
\mathbf k=\mathrm{diag}(k_\parallel,k_\parallel,k_\perp)
$$

### 2.2 Effective laminate conductivity
The stack has total copper thickness t_Cu = 4 × 70 µm × 0.70 fill = 0.196 mm, and FR-4 thickness 1.404 mm.

In-plane (parallel paths):
$$
k_\parallel=\frac{t_{Cu}k_{Cu}+t_{FR4}k_{FR4}}{t}=\frac{0.196(390)+1.404(0.3)}{1.6}\approx 48\text{ W/m·K}
$$

Through-plane (series paths):
$$
k_\perp=\frac{t}{t_{Cu}/k_{Cu}+t_{FR4}/k_{FR4}}\approx 0.34\text{ W/m·K}
$$
Under the FET pads, thermal vias raise k_⊥ locally. A via array with copper area fraction φ_v gives approximately k_⊥ ≈ φ_v·k_Cu, which is typically 5–20 W/m·K.

### 2.3 Thin-plate reduction
The Bi