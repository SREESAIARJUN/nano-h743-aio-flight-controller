# Thermal Model of a 4-in-1 ESC at 80 A Total Burst (Cycle 1 Baseline)

This is the first-iteration model. It sets up the loss model, the governing PDE, the finite-element (FE) discretization, and a steady-state and transient estimate. Every number rests on the assumptions in §0. Replace them with your BOM and layout values in Cycle 2.

---

## 0. Baseline Assumptions

| Parameter | Symbol | Value |
|---|---|---|
| Board size | L × W × t | 36 × 36 × 1.6 mm (30.5 mm mounting) |
| Stack-up | — | 6 layers, 2 oz Cu (70 µm), ~60 % copper fill, FR-4 |
| Battery | V_bus | 6S, 25.2 V max |
| Burst current | I_bat | 80 A total = 20 A per motor |
| Duty at burst | D | 0.8 |
| MOSFETs | — | 24 × 40 V; R_ds,on(25 °C) = 1.5 mΩ; t_r = t_f = 20 ns; Q_g = 50 nC; Q_rr = 50 nC; V_f = 0.8 V |
| PWM / dead time | f_sw, t_dt | 48 kHz, 200 ns |
| Current shunt | R_sh | 0.25 mΩ (single, on total current) |
| Ambient | T_∞ | 25 °C |
| Prop-wash air speed | u | ~10 m/s |

---

## 1. Heat-Generation Physics (Source Term q‴)

### 1.1 Phase current
With 6-step commutation, two legs carry current at any instant:

$$I_{ph} \approx \frac{I_{bat,motor}}{D} = \frac{20}{0.8} = 25\ \text{A}$$

### 1.2 MOSFET losses (per motor)

**Temperature-dependent on-resistance.** This dependence is what makes the problem nonlinear:

$$R_{ds}(T_j) = R_{25}\,[1+\alpha(T_j-25)],\quad \alpha \approx 0.0045\ \text{K}^{-1}$$

At T_j ≈ 100 °C, R_ds ≈ 2.2 mΩ.

**Conduction** (synchronous rectification, so one FET conducts per active leg):
$$P_{cond} = 2\,I_{ph}^2 R_{ds}(T) = 2(25^2)(0.0022) = 2.75\ \text{W}$$

**Switching** (one PWM'd leg):
$$P_{sw} = \tfrac12 V_{bus} I_{ph}(t_r+t_f) f_{sw} = \tfrac12(25.2)(25)(40\text{ns})(48\text{k}) = 0.60\ \text{W}$$

**Dead-time body-diode conduction:**
$$P_{dt} = V_f I_{ph}\,(2t_{dt}) f_{sw} = 0.8(25)(400\text{ns})(48\text{k}) = 0.38\ \text{W}$$

**Reverse recovery and gate drive:**
$$P_{rr} = Q_{rr}V_{bus}f_{sw} \approx 0.06\ \text{W},\qquad P_g = Q_g V_{gs} f_{sw}\cdot 2 \approx 0.05\ \text{W}$$

**Per-motor total:** ≈ 3.85 W.
**All four motors:** ≈ 15.4 W.

### 1.3 Board-level losses

| Source | Expression | Loss |
|---|---|---|
| Power-plane copper (R_eff ≈ 0.3 mΩ) | I_bat² R_Cu (with ρ_Cu(T) = ρ₀[1+0.00393(T−20)]) | ≈ 1.9 W |
| Shunt | 80² × 0.25 mΩ | 1.6 W |
| MCU, gate drivers, LDO/BEC | — | ≈ 0.5 W |

### 1.4 Total dissipation

$$\boxed{P_{tot} \approx 19.4\ \text{W at burst}}$$

### 1.5 Volumetric source term
Losses are applied as volumetric sources over each component footprint Ω_i:

$$q'''(\mathbf{x},T) = \sum_i \frac{P_i(T)}{V_i}\,\chi_{\Omega_i}(\mathbf{x})$$

---

## 2. Governing Conductive Heat-Transfer Equations

### 2.1 Transient energy equation in anisotropic media

$$\rho c_p \frac{\partial T}{\partial t} = \nabla\cdot(\mathbf{k}\,\nabla T) + q'''(\mathbf{x},T)$$

At steady state:

$$\nabla\cdot(\mathbf{k}\nabla T) + q'''(T) = 0$$

### 2.2 Effective PCB conductivity tensor
The conductivity tensor is

$$\mathbf{k} = \text{diag}(k_{xy},k_{xy},k_z)$$

**In-plane (layers in parallel):**
$$k_{xy} = \frac{\sum k_i t_i}{\sum t_i} = \frac{385(6\times0.07\times0.6) + 0.3(1.348)}{1.6} \approx 61\ \text{W/m·K}$$

**Through-plane (layers in series):**
$$k_z = \frac{\sum t_i}{\sum t_i/k_i} \approx 0.35\ \text{W/m·K}$$

Under FET pads with a thermal-via array, use a local effective value:

$$k_z^{via} = \phi_{via}\,k_{Cu} + (1-\phi_{via})k_{FR4} \approx 5\text{–}20\ \text{W/m·K}$$

### 2.3 Boundary conditions

**Convection on the top and bottom faces (Robin condition):**
$$-\mathbf{n}\cdot\mathbf{k}\nabla T = h_{c}(T-T_\infty) + \varepsilon\sigma(T^4-T_\infty^4)$$

**Convection coefficient** from a laminar flat plate in prop wash:
- Reynolds number: Re = uL/ν = 10(0.036)/1.6×10⁻⁵ ≈ 22 500
- Nusselt number: Nu = 0.664 Re^{1/2} Pr^{1/3} ≈ 88
- Result: h_c = Nu·k_air/L ≈ **64 W/m²K**

**Linearized radiation:**
$$h_r = \varepsilon\sigma(T^2+T_\infty^2)(T+T_\infty) \approx 7\ \text{W/m²K}\ (\varepsilon = 0.9,\ T\approx 120^\circ\text{C})$$

**Mounting standoffs (Dirichlet-like or contact condition):**
$$q = (T-T_{frame})/R_{contact}$$

**Edges:** adiabatic, $\mathbf{n}\cdot\nabla T = 0$.

---

## 3. Finite-Element Formulation

### 3.1 Weak (Galerkin) form
Find T ∈ H¹(Ω) such that, for all test functions w:

$$\int_\Omega \nabla w\cdot\mathbf{k}\nabla T\,d\Omega + \int_{\Gamma_c} w\,h\,T\,d\Gamma = \int_\Omega w\,q'''\,d\Omega + \int_{\Gamma_c} w\,h\,T_\infty\,d\Gamma$$

### 3.2 Discretization
Approximate the field as T ≈ **N**ᵀ**T**ₑ, using 8-node hexahedra (or layered shell elements). Define B = ∇N.

**Element conduction matrix:**
$$\mathbf{K}_e = \int_{\Omega_e}\mathbf{B}^T\mathbf{k}\,\mathbf{B}\,d\Omega$$

**Element convection matrix:**
$$\mathbf{H}_e = \int_{\Gamma_e} h\,\mathbf{N}\mathbf{N}^T d\Gamma$$

**Element load vector:**
$$\mathbf{f}_e = \int_{\Omega_e}\mathbf{N}q'''d\Omega + \int_{\Gamma_e}hT_\infty\mathbf{N}\,d\Gamma$$

**Element capacitance matrix:**
$$\mathbf{C}_e = \int_{\Omega_e}\rho c_p\mathbf{N}\mathbf{N}^Td\Omega$$

### 3.3 Assembled system

**Steady state:**
$$[\mathbf{K}+\mathbf{H}(T)]\,\mathbf{T} = \mathbf{f}(T)$$

**Transient** (θ-method; θ = 1 is backward Euler):
$$\left[\tfrac{\mathbf{C}}{\Delta t}+\theta(\mathbf{K}+\mathbf{H})\right]\mathbf{T}^{n+1} = \left[\tfrac{\mathbf{C}}{\Delta t}-(1-\theta)(\mathbf{K}+\mathbf{H})\right]\mathbf{T}^n + \mathbf{f}$$

### 3.4 Nonlinear solution (Newton–Raphson)
Two terms depend on temperature: q‴ through R_ds(T) and ρ_Cu(T), and h_r through radiation.

**Residual:**
$$\mathbf{R}(\mathbf{T}) = (\mathbf{K}+\mathbf{H})\mathbf{T}-\mathbf{f}$$

**Jacobian:**
$$\mathbf{J} = \mathbf{K}+\mathbf{H}+4\varepsilon\sigma T^3\mathbf{M}_\Gamma - \frac{\partial\mathbf{f}}{\partial\mathbf{T}}$$

where

$$\frac{\partial f}{\partial T}\Big|_{FET} = \frac{2I_{ph}^2R_{25}\alpha}{V_{FET}}$$

**Iteration:** solve J ΔT = −R until ‖ΔT‖ < 0