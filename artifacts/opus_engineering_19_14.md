# Thermal Model of a 4-in-1 ESC at 80 A Total Burst Current

I don't have the specifics behind "Cycle 19", so I've used representative parameters for a 6S, 30.5 mm-mount 4-in-1 ESC. Each one is stated explicitly so you can substitute your measured values.

---

## 0. Assumptions

| Parameter | Symbol | Value |
|---|---|---|
| Board size | L×W×t | 37 × 37 × 1.6 mm |
| Stack-up | — | 4 layers, 2 oz Cu (70 µm each), FR-4 core |
| Battery voltage (6S, full charge) | V_bus | 25.2 V |
| Total battery current (burst) | I_tot | 80 A, so 20 A per channel |
| Duty cycle at burst | D | 0.8 |
| Phase current | I_ph ≈ I_b/D | 25 A |
| PWM frequency | f_sw | 48 kHz |
| MOSFETs | — | 24 total (6 per channel), R_ds(on) = 1.5 mΩ at 25 °C |
| Rise + fall time | t_r + t_f | 20 ns |
| Dead time | t_dt | 200 ns |
| Body-diode forward voltage | V_f | 0.8 V |
| Ambient temperature | T∞ | 25 °C |
| Convection coefficient | h | 60 W/m²K with prop-wash (≈10 static) |

---

## 1. Loss (Heat Source) Physics

With 6-step commutation, at any instant two half-bridges carry I_ph. Synchronous rectification means effectively two FETs are always in the current path.

**Conduction loss.** R_ds(on) rises with temperature:

$$R_{ds}(T_j)=R_{ds,25}\,[1+\alpha(T_j-25)],\quad \alpha\approx0.005\,\text{K}^{-1}$$

At T_j ≈ 125 °C, R_ds ≈ 2.25 mΩ, so:

$$P_{cond}=2\,I_{ph}^2R_{ds}=2(25)^2(2.25\text{ mΩ})\approx2.81\text{ W/channel}$$

**Switching loss:**

$$P_{sw}=\tfrac12V_{bus}I_{ph}(t_r+t_f)f_{sw}+\tfrac12Q_{oss}V_{bus}f_{sw}\approx0.30+0.05=0.35\text{ W}$$

**Dead-time (body-diode) loss:**

$$P_{dt}=2V_fI_{ph}t_{dt}f_{sw}=2(0.8)(25)(200\text{ ns})(48\text{ kHz})\approx0.38\text{ W}$$

**Gate-drive loss:**

$$P_g=Q_gV_{gs}f_{sw}\times N_{sw}\approx0.1\text{ W}$$

**Per-channel total:** P_ch ≈ 3.65 W, so the four bridges dissipate about 14.6 W.

**Board-level losses:**

- Copper planes, assuming an effective path resistance of 0.3 mΩ: 80² × 0.3 mΩ ≈ 1.9 W.
- Current-sense shunts at 0.5 mΩ each: 4 × 20² × 0.5 mΩ ≈ 0.8 W.
- MCU, LDO and gate drivers: about 0.3 W.

**Total:** Q_tot ≈ 17.6 W.

Averaged over commutation, each FET dissipates about 3.65/6 ≈ 0.6 W.

---

## 2. Governing Conduction Equation

The general transient, anisotropic heat equation is:

$$\rho c_p\frac{\partial T}{\partial t}=\nabla\cdot(\mathbf{k}\nabla T)+\dot q(\mathbf{x},T)$$

The volumetric heat source includes Joule heating in the copper:

$$\dot q=\sum_i P_i(T)\,\delta_{\Omega_i}/V_i+J^2\rho_e(T)\big|_{Cu}$$

### Effective orthotropic PCB conductivity

The copper fraction through the thickness is t_Cu = 0.28 mm of 1.6 mm. Conductivity is a parallel (arithmetic) average in-plane and a series (harmonic) average through the thickness:

$$k_{xy}=\frac{\sum k_it_i}{t}=\frac{385(0.28)+0.3(1.32)}{1.6}\approx68\ \text{W/mK}$$

$$k_z=\frac{t}{\sum t_i/k_i}=\frac{1.6}{0.28/385+1.32/0.3}\approx0.36\ \text{W/mK}$$

The board spreads heat well laterally but poorly through its thickness. Thermal vias under the FET pads locally raise k_z by 10–50×.

### Boundary conditions

Convection plus linearized radiation on the top and bottom faces (Robin condition):

$$-\mathbf{k}\nabla T\cdot\mathbf{n}=h_{eff}(T-T_\infty)$$

$$h_{eff}=h_c+h_r,\qquad h_r=4\varepsilon\sigma T_m^3\approx6\ \text{W/m}^2\text{K}\ (\varepsilon=0.9)$$

Mounting standoffs can be modelled as a conductance G_s(T − T_frame), or neglected for nylon standoffs.

---

## 3. Finite-Element Formulation

**Weak form.** Find T ∈ H¹ such that for all test functions w:

$$\int_\Omega\rho c_p w\dot T\,d\Omega+\int_\Omega\nabla w\cdot\mathbf{k}\nabla T\,d\Omega+\int_{\Gamma_h}h\,wT\,d\Gamma=\int_\Omega w\dot q\,d\Omega+\int_{\Gamma_h}h\,wT_\infty d\Gamma$$

**Galerkin discretization.** Substituting T = N Tₑ gives:

$$\mathbf{C}\dot{\mathbf{T}}+(\mathbf{K}_k+\mathbf{K}_h)\mathbf{T}=\mathbf{F}_q+\mathbf{F}_h$$

with the element matrices:

$$\mathbf{K}_k^e=\int_{\Omega_e}\mathbf{B}^T\mathbf{k}\mathbf{B}\,d\Omega,\quad \mathbf{K}_h^e=\int_{\Gamma_e}h\mathbf{N}^T\mathbf{N}\,d\Gamma,\quad \mathbf{C}^e=\int_{\Omega_e}\rho c_p\mathbf{N}^T\mathbf{N}\,d\Omega$$

$$\mathbf{F}_q^e=\int_{\Omega_e}\mathbf{N}^T\dot q\,d\Omega,\qquad \mathbf{F}_h^e=\int_{\Gamma_e}hT_\infty\mathbf{N}^T d\Gamma$$

**Mesh options:**

- **(a) 2-D thin plate**, using linear triangles. The z-direction is collapsed, so both faces' convection becomes a sink term:

$$k_{xy}t\nabla^2T+q''-2h_{eff}(T-T_\infty)=0$$

- **(b) Full 3-D**, using hex or tet elements per layer. Each FET is modelled as a junction node connected through R_θjc ≈ 1 K/W to its pad.

**Nonlinearity (R_ds depends on T).** Solve with Picard iteration:

$$\mathbf{K}\mathbf{T}^{(n+1)}=\mathbf{F}(\mathbf{T}^{(n)}),\qquad P_i^{(n)}=P_{i,0}[1+\alpha(T_i^{(n)}-25)]$$

Iterate until ‖ΔT‖ < 0.01 K.

**Transient integration.** Use the θ-method (Crank–Nicolson, θ = ½):

$$\left(\tfrac{\mathbf{C}}{\Delta t}+\theta\mathbf{K}\right)\mathbf{T}^{n+1}=\left(\tfrac{\mathbf{C}}{\Delta t}-(1-\theta)\mathbf{K}\right)\mathbf{T}^n+\mathbf{F}$$

---

## 4. Steady-State Temperature Distribution

### 4a. Global (lumped) rise

Wetted area is two faces: A = 2(0.037)² = 2.74×10⁻³ m².

$$\Delta T_{board}=\frac{Q_{tot}}{h_{eff}A}=\frac{17.6}{66(2.74\times10^{-3})}\approx97\ \text{K}\ \Rightarrow\ T_{board}\approx122\ ^\circ\text{C}$$

### 4b. Local spreading (fin) solution

The thin-plate equation has a characteristic length set by the fin parameter:

$$m=\sqrt{\frac{2h_{eff}}{k_{xy}t}}=\sqrt{\frac{132}{68(1.6\times10^{-3})}}\approx35\ \text{m}^{-1},\qquad 1/m\approx29\text{ mm}$$

The decay length is comparable to the board size, so the board is quasi-isothermal, with moderate peaks at the FET clusters.

The Green's function for a point source in an infinite plate is:

$$\theta(r)=T-T_\infty=\frac{P}{2\pi k_{xy}t}K_0(mr)$$

By superposition over the 24 FETs plus distributed sources:

$$T(x,y)=T_\infty+\sum_{i=1}^{24}\frac{P_i}{2\pi k_{xy}t}K_0\!\left(m\,|\mathbf{r}-\mathbf{r}_i|\right)+\theta_{dist}(x,y)$$

Use image sources to enforce the finite board edges.

**Self-heating at a FET pad.** With an effective source radius r₀ ≈ 1.