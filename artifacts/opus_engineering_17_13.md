# Thermal Model of a 4-in-1 ESC at 80 A Total Burst Current

**Note on "Cycle 17":** I don't have your project's Cycle 17 specification. The model below is fully parameterized, and the numbers use the stated assumptions. Replace them with your datasheet and layout values.

---

## 1. Baseline Assumptions

| Parameter | Symbol | Value |
|---|---|---|
| Board | — | 30.5 × 30.5 mm, 6-layer, 1.6 mm thick, 4 × 2 oz (70 µm) Cu planes |
| Supply | V_bus | 6S, 25.2 V maximum |
| Current | I_bus | 80 A total, so 20 A per motor channel |
| Phase current at burst (duty D ≈ 0.9) | I_ph | ≈ I_bus,ch / D ≈ 22 A |
| MOSFETs | — | 24 × DFN3.3; R_DS(on),25 = 1.5 mΩ, α_R ≈ 0.005 K⁻¹ |
| Switching | — | f_sw = 48 kHz, t_r + t_f = 30 ns, dead time t_dt = 200 ns, body-diode V_f = 0.8 V |
| Ambient | T_∞ | 25 °C |
| Cooling | h | Prop-wash convection 50 W/m²K, plus linearized radiation ≈ 7 W/m²K |

---

## 2. Heat-Source Physics (Loss Model)

### 2.1 MOSFET conduction (temperature-dependent)

In 6-step or FOC commutation, one high-side and one low-side device carry I_ph at any instant (synchronous rectification). Per channel:

$$P_{cond}(T) = 2\,I_{ph}^2\,R_{DS,25}\,[1+\alpha_R(T-25)]$$

At 25 °C: 2 · 22² · 1.5 mΩ = **1.45 W per channel**.

### 2.2 Switching losses (hard-switched half-bridge)

$$P_{sw} = \tfrac12 V_{bus} I_{ph}(t_r+t_f)f_{sw} + Q_{rr}V_{bus}f_{sw} + \tfrac12 C_{oss}V_{bus}^2 f_{sw}$$

- First term: 0.5 · 25.2 · 22 · 30 ns · 48 kHz ≈ **0.40 W**
- Q_rr and C_oss terms: about 0.15 W
- Total: **P_sw ≈ 0.55 W per channel**

### 2.3 Dead-time body-diode conduction

$$P_{dt} = V_f\, I_{ph}\cdot 2t_{dt} f_{sw}$$

0.8 · 22 · 400 ns · 48 kHz ≈ **0.34 W per channel**

### 2.4 Copper and shunt Joule heating (distributed sources)

$$q'''_{Cu} = \frac{|\mathbf J|^2}{\sigma_{Cu}(T)}, \qquad \sigma_{Cu}(T)=\frac{\sigma_0}{1+0.0039(T-25)}$$

- Bus copper (≈ 0.3 mΩ effective): 80² · 0.3 mΩ ≈ **1.9 W**
- Shunts (4 × 0.5 mΩ): 4 · 20² · 0.5 mΩ ≈ **0.8 W**
- Gate drive and LDO: about 0.1 W, neglected

### 2.5 Total power budget

$$P_{tot}(T) = \underbrace{4(0.55+0.34)+1.9+0.8}_{P_c\,\approx\,6.3\text{ W}} \;+\; \underbrace{5.8\,[1+\alpha_R(T-25)]}_{\text{conduction}}$$

At 25 °C, P_tot ≈ 12.1 W. At 100 °C, P_tot ≈ 14.3 W.

---

## 3. Governing Conduction Equation

### 3.1 Strong form

Anisotropic, nonlinear, steady state:

$$\nabla\cdot\big(\mathbf K(\mathbf x)\,\nabla T\big) + q'''(\mathbf x,T) = 0 \quad \text{in } \Omega$$

$$\mathbf K = \mathrm{diag}(k_\parallel, k_\parallel, k_\perp)$$

### 3.2 Effective PCB conductivities (laminate homogenization)

Copper thickness t_Cu = 0.28 mm; FR-4 thickness t_FR4 = 1.32 mm.

**In-plane (parallel rule):**
$$k_\parallel=\frac{k_{Cu}t_{Cu}+k_{FR4}t_{FR4}}{t} = \frac{385(0.28)+0.3(1.32)}{1.6}\approx 67.6\ \text{W/mK}$$

**Through-plane (series rule):**
$$k_\perp=\frac{t}{t_{Cu}/k_{Cu}+t_{FR4}/k_{FR4}}\approx 0.36\ \text{W/mK}$$

**Under thermal-via arrays beneath the FETs** (via fill fraction φ_v):
$$k_\perp^{via}\approx \phi_v k_{Cu} + (1-\phi_v)k_\perp$$

With 0.3 mm vias at 0.8 mm pitch, φ_v ≈ 0.08 for plated barrels, giving k_⊥^via ≈ 15–30 W/mK.

### 3.3 Boundary conditions

**Top and bottom surfaces (Robin):**
$$-\mathbf n\cdot\mathbf K\nabla T = h_{eff}(T-T_\infty)$$

$$h_{eff}=h_{conv}+h_{rad}, \qquad h_{rad}\approx 4\varepsilon\sigma \bar T^3\approx 7\ \text{W/m}^2\text{K}$$

**Solder pads to battery and motor wires (Robin, conduction into the leads):**
$$-\mathbf n\cdot\mathbf K\nabla T = \frac{T-T_\infty}{R_{wire}A_{pad}}, \qquad R_{wire}=\frac{1}{\sqrt{hPkA}\,\tanh(mL)}$$

This is the fin model of the wire.

**Board edges:** treated as adiabatic.

**MOSFET junctions (sub-model):**
$$T_j = T_{pad} + P_{FET}\,R_{\theta jc}, \qquad R_{\theta jc}\approx 1.5\ \text{K/W}$$

---

## 4. Finite-Element Formulation

### 4.1 Weak (Galerkin) form

Find T ∈ H¹(Ω) such that, for all test functions w:

$$\int_\Omega \nabla w\cdot\mathbf K\nabla T\,d\Omega + \int_{\Gamma_R} h_{eff}\,wT\,d\Gamma = \int_\Omega w\,q'''\,d\Omega + \int_{\Gamma_R} h_{eff}\,wT_\infty\,d\Gamma$$

### 4.2 Discretization

Approximate the field as T ≈ **N**·**T**_e. Use hexahedral elements through the laminate (one element per Cu/FR-4 layer), or 2.5D layered shells.

**Conductance matrix:**
$$\mathbf K_e=\int_{\Omega_e}\mathbf B^\top\mathbf K\,\mathbf B\,d\Omega, \qquad \mathbf B=\nabla\mathbf N$$

**Convection matrix:**
$$\mathbf H_e=\int_{\Gamma_e}h_{eff}\mathbf N^\top\mathbf N\,d\Gamma$$

**Load vector:**
$$\mathbf f_e=\int_{\Omega_e}q'''\mathbf N^\top d\Omega+\int_{\Gamma_e}h_{eff}T_\infty\mathbf N^\top d\Gamma$$

**Assembled system:**
$$\big[\mathbf K+\mathbf H\big]\mathbf T=\mathbf f(\mathbf T)$$

For linear triangles, the in-plane shell element reduces to:
$$\mathbf K_e=\frac{k_\parallel t}{4A}\left(\mathbf b\mathbf b^\top+\mathbf c\mathbf c^\top\right)$$

### 4.3 Electro-thermal coupling

Use Picard or Newton iteration:

1. Set T⁽⁰⁾ = T_∞.
2. Update R_DS(T⁽ⁿ⁾) and σ_Cu(T⁽ⁿ⁾).
3. Optionally, solve the current field from ∇·(σ∇φ) = 0 on the copper layers to obtain **J**.
4. Assemble **f**(T⁽ⁿ⁾) and solve for T⁽ⁿ⁺¹⁾.
5. Repeat until ‖T⁽ⁿ⁺¹⁾ − T⁽ⁿ⁾‖∞ < 0.01 K.

**Newton Jacobian:**
$$\mathbf J = \mathbf K+\mathbf H-\partial\mathbf f/\partial\mathbf T$$

where ∂P_FET/∂T = P_cond,25 α_R. The iteration diverges, indicating thermal runaway, if the spectral radius of (**K**+**H**)⁻¹ ∂**f**/∂**T** is 1 or greater.

### 4.4 Mesh guidance

- Element size of 0.25 mm or less under the FET pads and shunts.
- 1 mm elsewhere.
- Roughly 60k–150k DOF for a 3D layered model.

---

## 5. Steady-State Solution: Analytical Check of the FE Result

### 5.1 Fin parameter: is the board isothermal in-plane?

$$m=\sqrt{\frac{2h_{eff}}{k_\parallel t}}=\sqr