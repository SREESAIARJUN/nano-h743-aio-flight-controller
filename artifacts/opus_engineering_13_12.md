# Thermal Model of a 4-in-1 ESC at 80 A Total Burst Current

I don't have the "Cycle 13" specification, so this analysis uses the assumptions in Section 0. Replace them with your actual values. The equations hold regardless, and the numbers scale as described.

---

## 0. Baseline Assumptions

| Parameter | Value |
|---|---|
| Supply | 6S Li-ion, V_bus = 22.2 V nominal |
| Current | 80 A total, 20 A per channel (4 channels) |
| Topology | 3-phase bridge per channel (24 MOSFETs total), synchronous complementary PWM |
| MOSFET R_DS(on) | 1.5 mΩ at 25 °C, temperature coefficient α ≈ 0.0045 /K |
| MOSFET switching | t_r + t_f = 40 ns, Q_g = 50 nC, Q_rr = 50 nC, V_f(body) = 0.8 V |
| PWM | f_sw = 48 kHz, dead time t_dt = 300 ns |
| PCB | 40 × 40 mm, 1.6 mm thick, 6 layers of 2 oz copper (70 µm), ~60 % copper fill |
| Cooling | Forced convection in prop wash, h ≈ 60 W/m²K; ambient T∞ = 25 °C; emissivity ε ≈ 0.9 |

---

## 1. Dissipation Physics (Heat Sources)

### 1.1 MOSFET conduction loss
In 6-step (or FOC near peak), two devices carry the phase current at any instant: one high-side and one low-side, or its synchronous complement.

$$
P_{cond,ch} = I_{ph}^2 \cdot 2R_{DS}(T_j),\qquad R_{DS}(T_j)=R_{25}\left[1+\alpha (T_j-25)\right]
$$

At T_j ≈ 105 °C, R_DS ≈ 2.2 mΩ:
$$
P_{cond}=20^2\cdot 2\cdot 0.0022 = 1.76\ \text{W/channel}
$$

### 1.2 Switching loss (one commutating leg)
$$
P_{sw}=\tfrac12 V_{bus} I_{ph}(t_r+t_f) f_{sw} = 0.5\cdot22.2\cdot20\cdot40\text{ ns}\cdot48\text{ kHz}=0.43\ \text{W}
$$

### 1.3 Dead-time body-diode loss
$$
P_{dt}=V_f I_{ph}\,(2t_{dt}) f_{sw}=0.8\cdot20\cdot600\text{ ns}\cdot48\text{ kHz}=0.46\ \text{W}
$$

### 1.4 Reverse recovery and gate charge
$$
P_{rr}=Q_{rr}V_{bus}f_{sw}\approx0.05\ \text{W},\qquad P_g = 2Q_gV_{GS}f_{sw}\approx0.05\ \text{W (dissipated in driver)}
$$

**Per channel:** P_ch ≈ 2.75 W. Of this, roughly 1.2 W lands in the hottest (switching high-side) FET.

### 1.5 Board-level losses

| Source | Expression | Value |
|---|---|---|
| Bus copper (80 A, R ≈ 0.3 mΩ) | I²R | ≈ 1.9 W |
| Phase traces (4 × 20 A, ~0.5 mΩ each) | 4·I²R | ≈ 0.8 W |
| Bulk capacitor ESR ripple | I_rms² · ESR, with I_rms ≈ I·√(D(1−D)) | ≈ 0.5 W |
| MCU, LDO, gate drivers | — | ≈ 0.3 W |

### 1.6 Total
$$
P_{tot}=4(2.75)+1.9+0.8+0.5+0.3\approx \mathbf{14.5\ W}
$$
This is about 0.8 % of the ~1.78 kW throughput.

**Volumetric source term for the FEM model:** q(x) = P_i / V_i inside each component footprint. Joule heating in copper is q = J²/σ_Cu(T), which can be solved by coupling to a DC current-flow field, ∇·(σ∇φ) = 0.

---

## 2. Conductive Heat Transfer Formulation

### 2.1 Governing equation (anisotropic laminate)
$$
\rho c_p\frac{\partial T}{\partial t} = \nabla\cdot(\mathbf{K}\nabla T) + q(\mathbf{x},T),\qquad 
\mathbf{K}=\mathrm{diag}(k_{xy},k_{xy},k_z)
$$

### 2.2 Effective PCB conductivities
Effective copper thickness is t_Cu = 6 × 70 µm × 0.6 = 0.252 mm, and the FR-4 thickness is 1.348 mm.

**In-plane (layers act in parallel):**
$$
k_{xy}=\frac{\sum k_i t_i}{t}=\frac{385(0.252)+0.3(1.348)}{1.6}\approx 61\ \text{W/mK}
$$

**Through-plane (layers act in series):**
$$
k_z=\frac{t}{\sum t_i/k_i}=\frac{1.6}{0.252/385+1.348/0.3}\approx 0.36\ \text{W/mK}
$$

**Under FET pads with thermal-via arrays:**
$$
k_{z,via}\approx k_{Cu}\,\frac{N\pi(r_o^2-r_i^2)}{A_{pad}} + k_{FR4} \approx 5\text{–}20\ \text{W/mK}
$$
Model this as a separate material region.

### 2.3 Boundary conditions (Robin, top and bottom surfaces)
$$
-\mathbf{n}\cdot\mathbf{K}\nabla T = h(T-T_\infty)+\varepsilon\sigma(T^4-T_\infty^4)
$$

Linearized radiation coefficient:
$$
h_r = 4\varepsilon\sigma T_m^3\approx 6.5\ \text{W/m}^2\text{K}\quad (T_m\approx340\text{ K})
$$
Use h_eff = h + h_r. Additional boundary terms:
- **Mounting standoffs:** Dirichlet or contact-conductance condition at the four holes, h_c ≈ 1000–5000 W/m²K over the screw area.
- **Motor wires:** treat as fins with $\dot Q = \sqrt{hPkA_c}\,(T_b-T_\infty)\tanh(mL)$.

### 2.4 Weak (Galerkin) form
Find T ∈ H¹(Ω) such that for all test functions v:
$$
\int_\Omega \nabla v\cdot\mathbf{K}\nabla T\,d\Omega + \int_{\Gamma_R} h_{eff}\,vT\,d\Gamma
=\int_\Omega vq\,d\Omega+\int_{\Gamma_R} h_{eff}\,vT_\infty\,d\Gamma
$$

### 2.5 Discretization
With T ≈ Σ N_j T_j:
$$
\big(\mathbf{K}_c+\mathbf{H}\big)\mathbf{T}=\mathbf{F}_q+\mathbf{F}_h
$$

where:
- Conduction matrix: $K_{c,ij}=\int_{\Omega_e}\nabla N_i\cdot\mathbf{K}\nabla N_j\,d\Omega$
- Convection matrix: $H_{ij}=\int_{\Gamma_e}h_{eff}N_iN_j\,d\Gamma$
- Source vector: $F_{q,i}=\int N_i q\,d\Omega$
- Convection load: $F_{h,i}=\int h_{eff}N_iT_\infty\,d\Gamma$

For a linear tetrahedron, $\mathbf{K}_c^e = V_e\,\mathbf{B}^T\mathbf{K}\mathbf{B}$ with constant gradient matrix **B**.

**Transient form (for the burst):**
$$
\mathbf{C}\dot{\mathbf{T}}+(\mathbf{K}_c+\mathbf{H})\mathbf{T}=\mathbf{F}
$$
Integrate with backward Euler or Crank–Nicolson.

### 2.6 Electro-thermal nonlinearity
Because q depends on R_DS(T), solve with Picard iteration:
$$
\mathbf{T}^{(k+1)} = (\mathbf{K}_c+\mathbf{H})^{-1}\mathbf{F}\big(\mathbf{T}^{(k)}\big)
$$
Iterate until $\|\Delta T\|_\infty<0.1$ K.

**Thermal-runaway stability criterion:**
$$
\frac{\partial P}{\partial T}=\alpha\,P_{cond} < \frac{1}{R_{th}}
$$
Here, 0.0045 × 7 W ≈ 0.03 W/K, which is far below hA ≈ 0.19 W/K, so the design is stable.

---

## 3. Steady-State Solution (Analytical Cross-Check of the FEM Model)

### 3.1 Global energy balance (mean board rise)
$$
\bar{\Delta T}=\frac{P_{tot}}{h_{eff}A_s}=\frac{14.5}{66.5\cdot(2\cdot0.0016)}\approx \mathbf{68\ K}
$$
With h alone (no radiation), this would be ~76 K.

### 3.2 Spreading: 2.5D fin-plate model
$$
k_{xy}t\,\nab