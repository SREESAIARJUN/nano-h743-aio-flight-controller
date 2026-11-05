# Thermal Model of a 4-in-1 ESC at 80 A Total Burst Current (Cycle 31)

This model covers loss generation, the conduction PDE and its finite-element discretization, a first-pass steady-state solution, and the transient behavior that matters for an 80 A *burst*.

---

## 0. Assumed Configuration

Replace these with your datasheet and stack-up values. Every equation below is parametric.

| Parameter | Symbol | Value |
|---|---|---|
| Board size | L × W | 30.5 × 30.5 mm |
| Bus voltage (6S nominal) | V_bus | 22.2 V |
| Total burst current | I_tot | 80 A, so 20 A per channel |
| Duty at full throttle | D | ≈ 0.95, giving phase current I_ph ≈ I_dc/D ≈ 21 A |
| MOSFETs | | 24 total (4 channels × 3 half-bridges × 2) |
| On-resistance | R_ds,on(25 °C) | 1.5 mΩ |
| R_ds temperature coefficient | α | ≈ 0.004 K⁻¹ |
| Rise / fall time | t_r, t_f | 20 ns / 20 ns |
| Dead time | t_dt | 200 ns |
| Switching frequency | f_sw | 48 kHz |
| Body-diode forward voltage | V_f | 0.8 V |
| Reverse-recovery charge | Q_rr | 50 nC |
| Gate charge | Q_g | 30 nC |
| Gate drive voltage | V_gs | 10 V |
| Stack-up | | 6 layers, 1.6 mm, total copper 0.35 mm, FR-4 1.25 mm |
| Ambient air temperature | T_∞ | 25 °C |
| Prop-wash air velocity | | ≈ 5 m/s |

---

## 1. Heat-Generation Physics (Source Terms q)

### 1.1 Temperature-dependent on-resistance

$$R_{ds}(T_j) = R_{ds,25}\,[1+\alpha(T_j-25)]$$

### 1.2 Per-channel MOSFET losses (6-step commutation with synchronous rectification)

In 6-step commutation, one high-side and one low-side FET conduct at any instant.

**Conduction loss**
$$P_{cond} = 2\,I_{ph}^2\,R_{ds}(T_j)$$

**Switching loss** (hard-switched leg)
$$P_{sw} = \tfrac12 V_{bus} I_{ph}(t_r+t_f) f_{sw}$$

**Dead-time body-diode loss**
$$P_{dt} = 2\,V_f\, I_{ph}\, t_{dt}\, f_{sw}$$

**Reverse-recovery loss**
$$P_{rr} = Q_{rr} V_{bus} f_{sw}$$

**Gate-drive loss** (dissipated mostly in the driver and gate resistors)
$$P_g = n_{sw} Q_g V_{gs} f_{sw}$$

Evaluated at an initial guess of T_j = 100 °C, which gives R_ds = 2.1 mΩ:

| Loss term | Per channel |
|---|---|
| P_cond = 2·21²·0.0021 | 1.85 W |
| P_sw = 0.5·22.2·21·40 ns·48 kHz | 0.45 W |
| P_dt | 0.32 W |
| P_rr | 0.05 W |
| P_g | 0.03 W |
| **Total** | **≈ 2.70 W** |

### 1.3 Copper (Joule) losses

The volumetric source in the traces and planes is

$$q_{Cu} = \frac{|\mathbf J|^2}{\sigma_{Cu}(T)}, \qquad \sigma_{Cu}(T)=\frac{\sigma_{0}}{1+0.00393(T-20)}$$

- **Bus planes:** 80 A through an effective ≈ 0.3 mΩ gives ≈ 1.9 W.
- **Phase traces:** 4 × (21² × 0.5 mΩ) gives ≈ 0.9 W.

For an accurate distribution of this loss, solve the electrical problem ∇·(σ∇φ) = 0 first. Then map |J|² onto the thermal mesh, which makes this an electro-thermal coupling.

### 1.4 Logic

The MCUs, LDOs/BEC and current-sense amplifiers contribute ≈ 0.3 W.

### 1.5 Total heat load

$$P_{tot} \approx 4(2.70) + 1.9 + 0.9 + 0.3 \approx \mathbf{13.9\ W}\quad(\text{before thermal iteration})$$

---

## 2. Governing Conduction Equations

### 2.1 Strong form (steady state)

$$\nabla\cdot\big(\mathbf{k}(T)\,\nabla T\big) + q(\mathbf{x},T) = 0 \quad \text{in } \Omega$$

### 2.2 Anisotropic effective PCB conductivity

The layered stack-up gives a diagonal conductivity tensor:

$$\mathbf{k} = \mathrm{diag}(k_{xy},k_{xy},k_z)$$

**In-plane (parallel layers):**
$$k_{xy} = \frac{\sum k_i t_i}{\sum t_i} = \frac{385(0.35)+0.3(1.25)}{1.6} \approx 84.5\ \mathrm{W/m\,K}$$

**Through-plane (series layers):**
$$k_z = \frac{\sum t_i}{\sum t_i/k_i} = \frac{1.6}{0.35/385 + 1.25/0.3} \approx 0.38\ \mathrm{W/m\,K}$$

**Thermal-via regions** (via fill fraction φ_v, copper plating):
$$k_z^{via} \approx \phi_v k_{Cu} + (1-\phi_v)k_z$$

For example, φ_v = 0.08 gives k_z^via ≈ 31 W/m·K. These regions must be modeled as separate material zones under each FET pad.

### 2.3 Boundary conditions on Γ (top, bottom and edges)

$$-\mathbf{n}\cdot\mathbf{k}\nabla T = h_{conv}(T-T_\infty) + \varepsilon\sigma_{SB}(T^4 - T_\infty^4)$$

**Forced convection from prop wash** (laminar flat plate):

$$Re = \frac{UL}{\nu}=\frac{5(0.03)}{1.6\times10^{-5}}\approx 9{,}400$$

$$Nu = 0.664\,Re^{1/2}Pr^{1/3}\approx 57$$

$$h_{conv}=\frac{Nu\,k_{air}}{L}\approx 51\ \mathrm{W/m^2K}$$

**Linearized radiation:**

$$h_{rad}=\varepsilon\sigma_{SB}(T+T_\infty)(T^2+T_\infty^2)\approx 7\ \mathrm{W/m^2K}\quad(\varepsilon=0.9,\ T\approx 120^\circ\mathrm C)$$

**Mounting:** where the board is clamped through stack screws or a frame, use a contact conductance h_c (typically 500–2000 W/m²K) to the frame temperature.

### 2.4 Reduced 2-D "fin" form

Because the board is thin, through-thickness averaging gives a useful reduced model:

$$k_{xy}t\,\nabla_{xy}^2 T - 2h(T - T_\infty) + q''(x,y) = 0$$

The characteristic spreading length is

$$\frac{1}{m}=\sqrt{\frac{k_{xy}t}{2h}}=\sqrt{\frac{0.135}{116}}\approx 34\ \mathrm{mm}$$

Since 1/m is greater than the 30.5 mm board size, the board is **nearly isothermal at the macro scale**. Peak temperatures are therefore governed by **local spreading under each FET**, not by gradients across the whole board.

---

## 3. Finite-Element Formulation

### 3.1 Weak form

Multiply by a test function w and integrate by parts. Find T such that, for all w:

$$\int_\Omega \nabla w\cdot\mathbf{k}\nabla T\,d\Omega + \int_\Gamma w\,h\,T\,d\Gamma = \int_\Omega w\,q\,d\Omega + \int_\Gamma w\,h\,T_\infty\,d\Gamma$$

Here h = h_conv + h_rad.

### 3.2 Galerkin discretization

Approximate T ≈ Σ N_j T_j and use the same shape functions as test functions. This gives the linear system

$$\mathbf{K}\mathbf{T}=\mathbf{F}$$

**Conduction stiffness matrix:**
$$K_{ij} = \sum_e\left[\int_{\Omega_e}(\nabla N_i)^T\mathbf{k}\,\nabla N_j\,d\Omega + \int_{\Gamma_e} h\,N_iN_j\,d\Gamma\right]$$

**Load vector:**
$$F_i = \sum_e\left[\int_{\Omega_e}N_i\,q\,d\Omega + \int_{\Gamma_e}h\,T_\infty N_i\,d\Gamma\right]$$

**Element choice and meshing:**
- Use quadratic hexahedra (or prisms), layered to match the stack-up, with at least one element per copper layer.
- Ref