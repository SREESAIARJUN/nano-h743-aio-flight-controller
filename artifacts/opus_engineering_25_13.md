# Thermal Model of a 4-in-1 ESC at 80 A Total Burst Current

I don't have the earlier cycles of this design, so the parameters below are typical assumptions for a 30.5 × 30.5 mm 4-in-1 ESC. Replace them with your values. The equations stay valid and the numbers rescale directly.

## 0. Baseline Assumptions

| Parameter | Value |
|---|---|
| Board | 30.5 × 30.5 × 1.6 mm FR-4, 6 layers × 2 oz Cu (70 µm) |
| Supply | 6S, V_bus = 25.2 V |
| Current | 80 A total, so **20 A per channel** (burst) |
| MOSFETs | 24 total (6 per channel, 3 half-bridges) |
| R_ds(on) | 1.5 mΩ at 25 °C, α ≈ 0.005 /K |
| PWM | f_sw = 48 kHz, t_r + t_f ≈ 40 ns, dead time 200 ns |
| Gate / diode | Q_g ≈ 50 nC at 10 V; V_f ≈ 0.8 V; Q_rr ≈ 50 nC |
| Cooling | Prop wash ≈ 10 m/s, T_∞ = 25 °C, ε ≈ 0.9 (solder mask) |

---

## 1. Heat-Generation Physics (Source Term q)

In 6-step commutation two phases conduct, so the current flows through one high-side and one low-side FET in series.

**Temperature-dependent on-resistance**

$$R_{ds}(T)=R_{25}\,[1+\alpha (T_j-25)]$$

**Per-channel loss components**

| Mechanism | Expression | Value (T_j ≈ 100 °C, R_ds = 2.25 mΩ) |
|---|---|---|
| Conduction | $I^2 \cdot 2R_{ds}(T)$ | 1.80 W |
| Switching | $\tfrac12 V I (t_r+t_f) f_{sw}$ | 0.48 W |
| Dead-time diode | $V_f I \cdot 2t_{dt} f_{sw}$ | 0.31 W |
| Reverse recovery | $Q_{rr} V f_{sw}$ | 0.06 W |
| Gate drive | $Q_g V_{gs} f_{sw}\cdot 2$ | 0.05 W |
| Copper (≈0.5 mΩ path) | $I^2 R_{Cu}$ | 0.20 W |
| **Per channel** | | **≈ 2.9 W** |

**Board total:** 4 × 2.9 W plus about 0.3 W for the MCUs and LDO gives **P ≈ 12 W**.

**Copper Joule heating (optional coupled field)**

$$\nabla\cdot(\sigma(T)\nabla V)=0,\qquad q_{Cu}=\sigma(T)\,|\nabla V|^2,\qquad \sigma(T)=\frac{\sigma_0}{1+0.0039(T-20)}$$

---

## 2. Governing Conduction Equation

**Steady-state, anisotropic, nonlinear:**

$$\nabla\cdot\big(\mathbf{k}\,\nabla T\big)+q(\mathbf{x},T)=0\quad\text{in }\Omega$$

**Effective layered-PCB conductivity tensor**

The copper volume fraction is φ = 6·70/1600 = 0.26.

- In-plane (parallel rule):
$$k_{xy}= \phi k_{Cu}+(1-\phi)k_{FR4}=0.26(385)+0.74(0.3)\approx 101\ \text{W/m·K}$$
- Through-plane (series rule):
$$k_z = \Big[\tfrac{\phi}{k_{Cu}}+\tfrac{1-\phi}{k_{FR4}}\Big]^{-1}\approx 0.41\ \text{W/m·K}$$
- Under thermal-via arrays, k_z rises to roughly 5–30 W/m·K, depending on via density.

**Boundary conditions on the top and bottom surfaces (Robin plus radiation):**

$$-\mathbf{k}\nabla T\cdot\mathbf{n}=h\,(T-T_\infty)+\varepsilon\sigma_{SB}(T^4-T_\infty^4)$$

Board edges are taken as approximately adiabatic. Mounting standoffs are modeled as a lumped conductance G_s(T − T_frame).

**Convection coefficient (flat-plate correlation in prop wash)**

$$Re=\frac{UL}{\nu}=\frac{10(0.03)}{1.6\times10^{-5}}\approx 1.9\times10^4$$

$$h=0.664\,\frac{k_{air}}{L}Re^{1/2}Pr^{1/3}\approx 70\ \text{W/m}^2\text{K}$$

**Linearized radiation coefficient**

$$h_{rad}\approx 4\varepsilon\sigma T_m^3\approx 7\ \text{W/m}^2\text{K}$$

This gives h_eff ≈ 77 W/m²K.

---

## 3. Finite-Element Formulation

**Weak form.** Find T ∈ H¹ such that, for all test functions w:

$$\int_\Omega \nabla w\cdot\mathbf{k}\nabla T\,d\Omega+\int_{\Gamma_c} w\,h_{eff}T\,d\Gamma=\int_\Omega wq\,d\Omega+\int_{\Gamma_c} w\,h_{eff}T_\infty\,d\Gamma$$

**Galerkin discretization.** Use the expansion T ≈ Σ N_i T_i with the gradient matrix B = ∇N:

$$\big[\mathbf{K}_c+\mathbf{H}\big]\mathbf{T}=\mathbf{F}_q+\mathbf{F}_h$$

$$\mathbf{K}_c^e=\int_{\Omega_e}\mathbf{B}^T\mathbf{k}\mathbf{B}\,d\Omega,\quad
\mathbf{H}^e=\int_{\Gamma_e}h_{eff}\mathbf{N}^T\mathbf{N}\,d\Gamma,\quad
\mathbf{F}_q^e=\int_{\Omega_e}\mathbf{N}^Tq\,d\Omega,\quad
\mathbf{F}_h^e=\int_{\Gamma_e}h_{eff}T_\infty\mathbf{N}^T d\Gamma$$

**Nonlinear solution.** The loss q depends on T through R_ds(T), and radiation scales as T⁴. Solve with Picard iteration or Newton–Raphson:

$$\big[\mathbf{K}_c+\mathbf{H}(T^{(n)})-\mathbf{J}_q(T^{(n)})\big]\Delta\mathbf{T}=-\mathbf{R}(T^{(n)})$$

Here $J_q=\partial q/\partial T = 2I^2R_{25}\alpha$ per FET.

Electro-thermal runaway occurs if $\det[\mathbf{K}_c+\mathbf{H}-\mathbf{J}_q]\to 0$. This is the stability criterion.

**Meshing guidance**

- Use hexahedral elements per copper layer, or layered shell elements.
- Refine to ≤ 0.25 mm under MOSFET pads.
- Model each MOSFET as a block with R_θjc ≈ 1 K/W to its drain pad.

**Reduced 2-D shell model (fin equation) for quick checks:**

$$k_{xy}t\,\nabla^2_{xy}T-2h_{eff}(T-T_\infty)+q''(x,y)=0,\qquad m=\sqrt{\frac{2h_{eff}}{k_{xy}t}}$$

---

## 4. Steady-State Temperature Distribution

**Spreading parameter**

$$m=\sqrt{\frac{2(77)}{101\times0.0016}}\approx 31\ \text{m}^{-1}$$

For a 15 mm quadrant, mL ≈ 0.46 < 1. The board is therefore **nearly isothermal**, with only about a 10% in-plane gradient.

**Mean board temperature (lumped)**

Components increase the wetted area to about 1.5 × 2 × (0.0305)² ≈ 2.8 × 10⁻³ m².

$$\Delta T_{board}=\frac{P}{h_{eff}A}=\frac{12}{77\times2.8\times10^{-3}}\approx 56\ \text{K}\;\Rightarrow\;\bar T_{board}\approx 81\,^\circ\text{C}$$

**Local hotspot (FET cluster)**

Radial spreading resistance from the pad (r₁ = 3 mm) to the cluster boundary (r₂ = 7.5 mm):

$$R_{sp}=\frac{\ln(r_2/r_1)}{2\pi k_{xy}t}\approx 0.9\ \text{K/W}$$

$$T_j\approx \bar T_{board}+\underbrace{\sim 4\text{–}6}_{\text{spread/gradient}}+\underbrace{P_{FET}R_{\theta jc}}_{\approx1}\approx 87\text{–}88\,^\circ\text{C}$$

Re-iterating R_ds at this lower T_j trims the losses slightly. The solution converges to **T_j ≈ 85 °C** and **board ≈ 79 °C**.

**Qualitative field**

- Four hotspots sit at the FET clusters near the board corners.
- A saddle-shaped cooler region (about 70–75 °C) lies at the center under the MCUs.
- The thick bus/ground planes flatten the gradients between clusters.

---

## 5. Burst Transient (Important for "Burst" Ratings)

Steady state is not reached within a short burst.

$$C_{th}\approx m c_p\approx 6\ \text{g}\times0.8\ \text{J/g·K}\approx 4.8\ \text{J/K},\qquad \tau=\frac{C_{th}}{h_{eff}A}\approx 22\ \text{s}$$

$$T(t)=T_\infty+\Delta T_{ss}\big(1-e^{-t/\tau}\big)$$

| Burst duration | Board rise |
|---|---|
| 5 s | ≈ 11 K |
| 10 s | ≈