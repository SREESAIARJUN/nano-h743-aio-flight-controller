# Thermal Model: 4-in-1 ESC at 80 A Total Burst (Cycle 23)

I don't have the parameters from earlier cycles, so this cycle uses the explicit baseline below. Substitute your actual values and the equations carry through unchanged.

## 0. Baseline Assumptions

| Parameter | Value |
|---|---|
| Board | 30.5 × 30.5 mm (36 × 36 mm outline), 6-layer, 2 oz Cu, 1.6 mm FR-4 |
| Supply | 6S Li-ion: 22.2 V nominal, 25.2 V max |
| Current | 80 A total burst = 20 A per channel (4 channels) |
| Topology | 3 half-bridges per channel (24 MOSFETs total), 6-step trapezoidal drive, synchronous rectification |
| MOSFET | 40 V, R_DS(on,25°C) = 1.5 mΩ, PDFN 3.3×3.3, R_θJC = 1.5 K/W |
| Switching frequency | 48 kHz |
| Switching times | t_r + t_f = 20 ns, dead time 200 ns, V_SD = 0.8 V |
| Cooling | Ambient 35 °C; prop-wash velocity ≈ 8 m/s over a stacked board |

---

## 1. Heat-Generation Physics (Electro-Thermal Source Terms)

### 1.1 Motor phase current
With duty cycle D ≈ 0.95 at burst, the motor phase current is:

$$I_m = \frac{I_{bat,ch}}{D} \approx \frac{20}{0.95} \approx 21\ \text{A}$$

### 1.2 Temperature-dependent on-resistance

$$R_{DS}(T) = R_{25}\,[1 + \alpha_R (T_j - 25)], \qquad \alpha_R \approx 0.006\ \text{K}^{-1}$$

### 1.3 Conduction loss per channel
In 6-step drive, two switch positions always carry I_m: the PWM leg (high side or its synchronous low side) and the return low side.

$$P_{cond} = 2\,I_m^2\,R_{DS}(T_j)$$

At T_j ≈ 105 °C, R_DS = 2.25 mΩ, which gives **P_cond ≈ 1.98 W**.

### 1.4 Switching loss

$$P_{sw} = \tfrac12 V_{bus} I_m (t_r + t_f) f_{sw} = \tfrac12(25)(21)(20\text{ns})(48\text{k}) \approx 0.25\ \text{W}$$

### 1.5 Dead-time body-diode conduction

$$P_{dt} = V_{SD}\,I_m\,(2t_{dead})\,f_{sw} \approx 0.32\ \text{W}$$

### 1.6 Output-capacitance and gate-drive losses

$$P_{oss} = \tfrac12 C_{oss}V^2 f_{sw} \approx 0.02\ \text{W}$$

$$P_g = Q_g V_{GS} f_{sw} \approx 0.02\ \text{W}$$

### 1.7 Copper Joule loss
Copper resistivity rises with temperature:

$$\rho_{Cu}(T) = \rho_0[1 + 0.00393(T - 20)]$$

The volumetric source in the copper is:

$$q_{Cu}''' = \frac{|\mathbf J|^2}{\sigma(T)}, \qquad \mathbf J = -\sigma \nabla\phi, \qquad \nabla\cdot(\sigma\nabla\phi) = 0$$

The main bus carries 80 A at roughly 0.3 mΩ, which gives **≈ 1.9 W**. Phase traces add about 0.3 W.

### 1.8 Loss budget

| Source | Per channel | Total |
|---|---|---|
| MOSFET loss | ≈ 2.6 W | 10.4 W |
| Copper (bus + phase traces) | — | ≈ 2.2 W |
| Capacitor ESR, MCU, regulator | — | ≈ 0.4 W |
| **Total burst dissipation** | | **P_tot ≈ 13 W** |

---

## 2. Governing Conduction Equation

### 2.1 Field equation
Transient anisotropic conduction:

$$\rho c_p \frac{\partial T}{\partial t} = \nabla\cdot(\mathbf K\nabla T) + q'''(T, \mathbf x)$$

At steady state:

$$\nabla\cdot(\mathbf K\nabla T) + q''' = 0$$

### 2.2 Homogenized PCB conductivity
Use a copper fill fraction of about 0.7 across 6 × 70 µm layers, giving an effective copper thickness t_Cu ≈ 0.294 mm.

In-plane (layers act in parallel):

$$k_{\parallel} = \frac{\sum k_i t_i}{\sum t_i} = \frac{385(0.294) + 0.3(1.306)}{1.6} \approx 71\ \text{W/m·K}$$

Through-plane (layers act in series):

$$k_\perp = \frac{\sum t_i}{\sum t_i/k_i} \approx 0.37\ \text{W/m·K}$$

Thermal vias under the FET pads raise the local k_⊥ to about 5–15 W/m·K, depending on via density.

The conductivity tensor is therefore:

$$\mathbf K = \mathrm{diag}(k_\parallel,\, k_\parallel,\, k_\perp)$$

### 2.3 Boundary conditions

**Convection** on the top and bottom faces (Robin condition):

$$-\mathbf n\cdot\mathbf K\nabla T = h_c (T - T_\infty)$$

The convection coefficient comes from the laminar flat-plate correlation. With Re = vL/ν = 8(0.03)/1.6×10⁻⁵ ≈ 15 000:

$$\text{Nu} = 0.664\,\text{Re}^{1/2}\,\text{Pr}^{1/3} \approx 72$$

$$h_c \approx \frac{72\,(0.026)}{0.03} \approx 63\ \text{W/m}^2\text{K}$$

Derate this to about 50 W/m²K to account for stack shadowing.

**Radiation**, linearized:

$$h_r = 4\varepsilon\sigma \bar T^3 \approx 4(0.9)(5.67\times10^{-8})(390)^3 \approx 12\ \text{W/m}^2\text{K}$$

**Mounting standoffs and wires** are represented as lumped conductances G_k to the frame, at about 0.05 K⁻¹·W each, on the bus wires.

---

## 3. Finite-Element Formulation

### 3.1 Weak form (Galerkin)
Find T ∈ H¹(Ω) such that for every test function w:

$$\int_\Omega \nabla w\cdot\mathbf K\nabla T\,d\Omega + \int_{\Gamma_R} w\,(h_c + h_r)\,T\,d\Gamma = \int_\Omega w\,q'''\,d\Omega + \int_{\Gamma_R} w\,(h_c + h_r)\,T_\infty\,d\Gamma$$

### 3.2 Discretization
Use the interpolation T ≈ **N**ᵉ**T**ᵉ, with gradient matrix **B** = ∇**N**.

The element conductance matrix (conduction plus face film) is:

$$\mathbf K^e = \int_{\Omega_e} \mathbf B^T \mathbf K\,\mathbf B\,d\Omega + \int_{\Gamma_e} (h_c + h_r)\,\mathbf N^T\mathbf N\,d\Gamma$$

The element load vector is:

$$\mathbf f^e = \int_{\Omega_e} \mathbf N^T q'''\,d\Omega + \int_{\Gamma_e} (h_c + h_r)\,T_\infty\,\mathbf N^T d\Gamma$$

Assembling all elements gives:

$$\mathbf K(\mathbf T)\,\mathbf T = \mathbf f(\mathbf T)$$

### 3.3 Recommended element choices
- **Board:** 3D hex8/hex20 elements with one element per Cu/FR-4 layer. Alternatively, use a 2.5D layered shell with k_∥ in-plane and k_⊥ through-thickness.
- **MOSFETs:** model each as a volumetric source block. Attach it to the pad through R_θJC, and to the solder through a 25 µm layer at k = 50 W/m·K.
- **Mesh:** h ≤ 0.25 mm near the FETs, graded to 1 mm elsewhere.

### 3.4 Nonlinear solution (Picard iteration)
Iterate on the temperature-dependent losses and radiation:

$$\mathbf K(\mathbf T^{(n)})\,\mathbf T^{(n+1)} = \mathbf f(\mathbf T^{(n)})$$

At each iteration, update q''' with P_cond(T_j) and ρ_Cu(T), and update h_r(T). Stop when ‖ΔT‖_∞ < 0.