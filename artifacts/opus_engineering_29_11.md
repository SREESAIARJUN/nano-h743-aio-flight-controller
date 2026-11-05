# Thermal Model of a 4-in-1 ESC at 80 A Total Burst Current

I don't have the specific design or data behind "Cycle 29." This formulation therefore uses a representative 30.5 × 30.5 mm-mount 4-in-1 ESC: a 6S supply, 24 MOSFETs, and a 6-layer, 2 oz copper PCB. Each parameter is shown explicitly so you can substitute your measured values.

---

## 1. Assumptions

| Parameter | Symbol | Value |
|---|---|---|
| Bus voltage (6S, full charge) | V_bus | 25.2 V |
| Total current, split evenly across 4 channels | I_tot / I_ch | 80 A / 20 A |
| PWM frequency | f_sw | 48 kHz |
| MOSFET R_DS(on) at 25 °C | R₀ | 1.5 mΩ |
| R_DS(on) temperature coefficient | α_R | ≈ 0.005 K⁻¹ |
| Rise + fall time | t_r + t_f | 40 ns |
| Dead time | t_dt | 200 ns |
| Body-diode forward voltage | V_f | 0.8 V |
| Gate charge / gate drive | Q_g / V_gs | 50 nC / 10 V |
| Board size | — | 40 × 40 × 1.6 mm, 6 layers × 70 µm Cu |
| Ambient temperature | T_∞ | 25 °C |

---

## 2. Heat-Generation Physics (Source Terms)

### 2.1 MOSFET losses per channel

The model assumes six-step or trapezoidal commutation with complementary PWM. At any instant, one high-side and one low-side FET conduct.

**Conduction loss.** R_DS(on) rises with temperature, which makes this term temperature-dependent:

$$P_{cond}(T) = 2\,I_{ch}^2\,R_0\,[1+\alpha_R (T_j - 25)]$$

At 25 °C: 2·(20)²·1.5 mΩ = **1.20 W**.

**Switching loss.** Only one leg is PWM-switching at a time:

$$P_{sw} = \tfrac12 V_{bus} I_{ch}(t_r+t_f) f_{sw} = 0.5\cdot25.2\cdot20\cdot40\text{ns}\cdot48\text{kHz} = \mathbf{0.48\ W}$$

**Dead-time diode conduction:**

$$P_{dt} = V_f\,I_{ch}\,(2t_{dt})\,f_{sw} = 0.8\cdot20\cdot400\text{ns}\cdot48\text{kHz} = \mathbf{0.31\ W}$$

**Gate drive** (dissipated mostly in the driver and gate resistors):

$$P_g = Q_g V_{gs} f_{sw} \approx \mathbf{0.02\ W}$$

**Per-channel total:**
- About 2.0 W at 25 °C.
- About 2.5 W at a junction temperature near 100 °C.

### 2.2 Board-level sources

- **Copper Joule heating.** Battery pads, bus planes and phase pours carry 80 A. Assuming an effective resistance of about 0.3 mΩ:
  $$P_{Cu} = I_{tot}^2 R_{Cu}[1+\alpha_{Cu}\Delta T] \approx 1.9\text{ W}\quad (\alpha_{Cu}=0.0039\ \text{K}^{-1})$$
- **MCUs, LDO/buck regulators and gate drivers:** about 0.3 W.

**Total dissipation:**

$$P_{tot} \approx 4(2.0\text{–}2.5) + 1.9 + 0.3 \approx \mathbf{10\text{–}12\ W}$$

Copper loss is about 15–20% of the total and is often underestimated.

### 2.3 Volumetric source definition

Each component's loss is spread over its footprint volume:

$$q'''(\mathbf{x},T) = \sum_k \frac{P_k(T)}{V_k}\,\chi_k(\mathbf{x}) \;+\; J^2(\mathbf{x})\,\rho_{Cu}(T)$$

- χ_k is the indicator function of the k-th component's footprint.
- J is the current density, obtained from a coupled DC electrical solve of ∇·(σ∇φ) = 0 over the copper.

---

## 3. Conductive Heat Transfer Formulation

### 3.1 Governing PDE (anisotropic, nonlinear)

$$\rho c_p \frac{\partial T}{\partial t} = \nabla\cdot(\mathbf{K}\nabla T) + q'''(\mathbf{x},T), \qquad \mathbf{K} = \mathrm{diag}(k_{xy},k_{xy},k_z)$$

At steady state, the time derivative vanishes:

$$\nabla\cdot(\mathbf{K}\nabla T) + q'''(T)=0$$

### 3.2 Effective PCB conductivity (layer homogenization)

Assume 70% copper coverage (φ), 6 × 70 µm copper layers and 1.18 mm of FR-4.

**In-plane** (layers act in parallel):

$$k_{xy} = \frac{\sum k_i t_i}{\sum t_i} = \frac{385(0.42)(0.7) + 0.3(1.18)}{1.6} \approx \mathbf{71\ W/m\cdot K}$$

**Through-plane** (layers act in series):

$$k_z = \frac{\sum t_i}{\sum t_i/k_i} \approx \mathbf{0.4\ W/m\cdot K}\ \text{(no vias)}$$

Thermal-via arrays under the FETs raise k_z locally to about 5–30 W/m·K:

$$k_{z,via} \approx k_{Cu}\,\frac{N\pi(r_o^2-r_i^2)}{A_{cell}} + k_{FR4}$$

### 3.3 Boundary conditions

**Convection plus radiation** on the top and bottom surfaces:

$$-\mathbf{K}\nabla T\cdot\mathbf{n} = h_{eff}(T - T_\infty), \qquad h_{eff} = h_c + h_r$$

The radiation term is linearized:

$$h_r = 4\varepsilon\sigma T_m^3 \approx 4(0.9)(5.67\times10^{-8})(330)^3 \approx 7\ \text{W/m}^2\text{K}$$

Typical convection coefficients:

| Condition | h_c |
|---|---|
| Bench, no airflow | ≈ 10 W/m²K |
| In-frame prop wash | ≈ 40–100 W/m²K |
| Duct / direct downwash | > 100 W/m²K |

**Conduction into the frame** through the mounting standoffs is modeled as a Robin condition on the hole walls:

$$-\mathbf{K}\nabla T\cdot\mathbf{n} = \frac{T - T_{frame}}{R''_{mount}}$$

---

## 4. Finite-Element Discretization

### 4.1 Weak (Galerkin) form

Find T ∈ H¹(Ω) such that, for all test functions w:

$$\int_\Omega \nabla w\cdot\mathbf{K}\nabla T\,d\Omega + \int_{\Gamma_h} w\,h_{eff}T\,d\Gamma = \int_\Omega w\,q'''\,d\Omega + \int_{\Gamma_h} w\,h_{eff}T_\infty\,d\Gamma$$

### 4.2 Element matrices

Use the interpolation T ≈ **N**ᵀ**T**_e and define **B** = ∇**N**.

| Term | Expression |
|---|---|
| Conductance | $\mathbf{K}_e = \int_{\Omega_e}\mathbf{B}^T\mathbf{K}\,\mathbf{B}\,d\Omega$ |
| Convection | $\mathbf{H}_e = \int_{\Gamma_e} h_{eff}\,\mathbf{N}\mathbf{N}^T d\Gamma$ |
| Source load | $\mathbf{f}_{q,e} = \int_{\Omega_e}\mathbf{N}\,q'''\,d\Omega$ |
| Ambient load | $\mathbf{f}_{h,e} = \int_{\Gamma_e} h_{eff}T_\infty\mathbf{N}\,d\Gamma$ |
| Capacitance (transient) | $\mathbf{C}_e = \int_{\Omega_e}\rho c_p\mathbf{N}\mathbf{N}^T d\Omega$ |

For a linear triangle in a 2.5D shell model, with b_i, c_i the shape-function gradient coefficients and A_e the element area:

$$\mathbf{K}_e = \frac{k_{xy}t}{4A_e}\left[b_ib_j + c_ic_j\right]$$

$$\mathbf{H}_e = \frac{(h_{top}+h_{bot})A_e}{12}\begin{bmatrix}2&1&1\\1&2&1\\1&1&2\end{bmatrix}$$

### 4.3 Assembled system

**Steady state:**

$$\big[\mathbf{K}+\mathbf{H}\big]\mathbf{T} = \mathbf{f}_q(\mathbf{T}) + \mathbf{f}_h$$

**Transient** (backward Euler):

$$\left[\tfrac{1}{\Delta t}\mathbf{C} + \mathbf{K} + \mathbf{H}\right]\mathbf{T}^{n+1} = \tfrac{1}{\Delta t}\mathbf{C}\mathbf{T}^n + \mathbf{f}^{n+1}$$

### 4.4 Nonlinear coupling (Picard iteration)

1. Initialize T⁽⁰⁾ = T_∞.
2. Update R_DS(on)(T⁽ᵐ⁾) and ρ_Cu(T⁽ᵐ⁾), then recompute q'''.
3. Solve the linear system for T⁽ᵐ⁺¹⁾.
4. Repeat until ‖T⁽ᵐ⁺¹⁾ − T⁽ᵐ⁾‖_∞ < 0.01 K. This typically takes 4–8 iterations.

Junction temperature is recovered from each FET's local board node:

$$T_j = T_{pad} + P_{FET}(R_{\theta jc} + R_{\theta,solder})$$

---

## 5. Steady-State Temperature Distribution

### 5.1 Global (lumped) solution

The cooled area is both board faces: A = 2(0.04)² = 3.2 × 10⁻³ m².

**Linear estimate:**

$$\Delta T_{board} = \frac{P_{tot}}{h_{eff}A}$$

**Including the temperature feedback.** Losses grow with temperature as P(T) = P₀ + β ΔT. Here:
- P₀ ≈ 10 W
- β ≈