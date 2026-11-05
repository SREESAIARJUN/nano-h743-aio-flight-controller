# Thermal Model of a 4-in-1 ESC at 80 A Total Burst Current

This analysis covers the loss sources, the governing conduction equations, their finite-element discretization, and a numerical steady-state and transient solution.

I don't have a definition of "Cycle 21," so I treat it as a design-iteration label. All parameters below are stated assumptions. Replace them with your BOM and layout values.

---

## 0. Assumed Design Parameters

| Parameter | Symbol | Value |
|---|---|---|
| Battery | V_bus | 6S, 25.2 V max |
| Total battery current (burst) | I_tot | 80 A (20 A per ESC channel) |
| Topology | — | 4 × 3-phase bridges = 24 MOSFETs |
| MOSFET on-resistance | R_ds,25 | 1.2 mΩ, PDFN 5×6 package |
| Temperature coefficient of R_ds | α_R | 0.005 K⁻¹ |
| Switch transition time | t_r + t_f | 20 ns |
| Gate charge | Q_g | 40 nC |
| PWM frequency | f_sw | 48 kHz |
| Dead time | t_dt | 100 ns |
| Body-diode forward voltage | V_f | 0.8 V |
| Board | — | 36 × 36 mm, 1.6 mm, 6 layers × 2 oz Cu, ~70 % copper fill |
| Ambient | T_∞ | 25 °C |
| Duty cycle at burst | D | ≈ 0.9 |
| Commutation | — | 6-step with synchronous rectification |

---

## 1. Heat Generation Physics

The phase current at duty cycle D is

$$I_{ph} \approx \frac{I_{bat}}{D} = \frac{20}{0.9} \approx 22\ \text{A}.$$

### 1.1 Conduction loss
Two FETs conduct at any instant: one high-side and one low-side, or two low-sides during freewheeling.

$$P_{cond}(T) = 2\,I_{ph}^2\,R_{ds,25}\,[1+\alpha_R(T_j-25)]$$

At T_j ≈ 100 °C, R_ds ≈ 1.65 mΩ, giving **P_cond ≈ 1.60 W per ESC**.

### 1.2 Switching loss (hard-switched leg)
$$P_{sw} = \tfrac12 V_{bus} I_{ph}(t_r+t_f) f_{sw} = 0.5\cdot25\cdot22\cdot20\text{ns}\cdot48\text{k} \approx 0.26\ \text{W}$$

### 1.3 Coss, dead-time, and gate losses
- **Coss loss:** $P_{oss} = 2\cdot\tfrac12 C_{oss}V^2 f_{sw} \approx 0.03$ W
- **Dead-time (body diode) loss:** $P_{dt} = V_f I_{ph}\,(2t_{dt}) f_{sw} = 0.8\cdot22\cdot200\text{ns}\cdot48\text{k} \approx 0.17$ W
- **Gate-drive loss:** $P_g = Q_g V_{gs} f_{sw}\times 2 \approx 0.04$ W. This is dissipated mostly in the driver IC.

### 1.4 Passive and copper losses
- **Ceramic capacitor ripple:** $I_{C,rms}=I_{ph}\sqrt{D(1-D)}\approx 6.6$ A. With ESR ≈ 2 mΩ, loss ≈ 0.09 W.
- **Phase trace / Joule loss:** $\sum I^2 R_{trace}$ ≈ 0.3 W per ESC, assuming ~0.6 mΩ effective path.
- **Shared battery bus:** 80 A through ~0.15 mΩ gives 80² · 0.15 mΩ ≈ **0.96 W**.
- **MCU, LDO/buck, and drivers:** ≈ 0.1 W per ESC.

### 1.5 Loss budget

| Source | Per ESC (W) | ×4 / shared (W) |
|---|---|---|
| Conduction | 1.60 | 6.40 |
| Switching + Coss | 0.29 | 1.16 |
| Dead-time diode | 0.17 | 0.68 |
| Gate + logic | 0.14 | 0.56 |
| Capacitors + phase copper | 0.39 | 1.56 |
| Battery bus | — | 0.96 |
| **Total** | | **≈ 11.3 W** |

Conduction loss dominates at ~57 %, and it is temperature-dependent. This makes the thermal problem nonlinear.

---

## 2. Governing Conduction Equations

### 2.1 Heat-conduction PDE
The board volume Ω uses an anisotropic conductivity tensor:

$$\rho c_p\frac{\partial T}{\partial t} = \nabla\cdot(\mathbf{k}\nabla T) + q'''(\mathbf{x},T), \qquad \mathbf{k}=\mathrm{diag}(k_{xy},k_{xy},k_z)$$

At steady state, $\partial T/\partial t = 0$:

$$-\nabla\cdot(\mathbf{k}\nabla T) = q'''(\mathbf{x},T)$$

The volumetric source q''' is the per-device loss divided by the die volume, with conduction loss scaled by $[1+\alpha_R(T-25)]$.

### 2.2 Effective PCB conductivity (layer homogenization)
For layer thicknesses t_i and conductivities k_i, with 0.294 mm effective Cu and 1.306 mm FR-4:

**In-plane (parallel layers):**
$$k_{xy}=\frac{\sum k_i t_i}{\sum t_i}=\frac{0.294(390)+1.306(0.3)}{1.6}\approx 72\ \text{W/m·K}$$

**Through-plane (series layers):**
$$k_z=\frac{\sum t_i}{\sum t_i/k_i}\approx 0.37\ \text{W/m·K}$$

**Under thermal-via arrays:** assume 20 vias, 0.3 mm drill, 25 µm plating, under a 30 mm² pad. The copper area fraction is

$$\phi = \frac{20\,\pi d\,t_{pl}}{A_{pad}} \approx 0.016,$$

so locally

$$k_z^{via} \approx \phi k_{Cu} + (1-\phi)k_z \approx 6.5\ \text{W/m·K}.$$

That is a ~17× improvement in vertical conduction. Model these regions as separate material zones.

### 2.3 Boundary conditions
**Top and bottom surfaces (Robin condition)** combine convection and radiation:

$$-\mathbf{n}\cdot\mathbf{k}\nabla T = h_c(T-T_\infty)+\varepsilon\sigma(T^4-T_\infty^4)$$

Linearized radiation coefficient: $h_r\approx 4\varepsilon\sigma\bar T^3 \approx 8.8$ W/m²K at ε = 0.9, T̄ ≈ 350 K.

Convection coefficient by regime:

| Regime | h_c (W/m²K) |
|---|---|
| Stack, still air | ~10 |
| Prop-wash in flight | 40–100 |

**Mounting points:** four M3 standoffs act as conductive sinks to the frame. Model them as $-\mathbf{n}\cdot\mathbf{k}\nabla T = (T-T_{frame})/(R_{bolt}A_{bolt})$, or as adiabatic if nylon.

**Wire pads:** heavy battery and motor leads can be modeled as fin sinks:

$$G_{wire} = \sqrt{hPkA}\tanh(mL), \qquad m=\sqrt{\frac{hP}{kA}}.$$

---

## 3. Finite-Element Formulation

### 3.1 Weak form
Find T ∈ H¹(Ω) such that, for all test functions v:

$$\int_\Omega \nabla v\cdot\mathbf{k}\nabla T\,d\Omega + \int_{\Gamma_R} h\,vT\,d\Gamma = \int_\Omega v\,q'''\,d\Om