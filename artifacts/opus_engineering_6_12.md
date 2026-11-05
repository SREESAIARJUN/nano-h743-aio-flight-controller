# Thermal Model of a 4-in-1 ESC at 80 A Total Burst (Cycle 6)

## 0. Assumptions

I don't have the earlier cycles' parameters, so these are representative values. Substitute your own and the structure of the model still holds.

| Parameter | Symbol | Value |
|---|---|---|
| Battery | V_bus | 6S, 22.2 V nominal |
| Total burst current | I_tot | 80 A, so 20 A per motor channel |
| MOSFETs | — | 24 total (4 × 3 half-bridges), R_DS(on),25 = 1.5 mΩ |
| R_DS temperature coefficient | α | 0.005 K⁻¹ |
| Switching | f_sw, t_r + t_f | 48 kHz, 40 ns total |
| Dead time | t_dt | 200 ns, body diode V_f = 0.8 V |
| Gate charge, recovery charge | Q_g, Q_rr | 30 nC, 50 nC |
| PCB | — | 36 × 36 × 1.6 mm, 6 layers of 2 oz Cu (6 × 70 µm) |
| Cooling | h_conv | 80 W/m²K (prop-wash forced convection) |
| Ambient | T∞ | 30 °C |

---

## 1. Heat-Source (Dissipation) Physics

The motors are driven with six-step commutation and synchronous rectification. At any instant, two FET channels per motor carry the phase current I_ph.

**1.1 Conduction loss (temperature-dependent)**

$$
P_{cond} = I_{ph}^2 \cdot 2R_{DS,25}\,[1+\alpha(T_j-25)]
$$

**1.2 Switching loss** (hard-switched edges)

$$
P_{sw} = \tfrac12 V_{bus} I_{ph}(t_r+t_f) f_{sw}
$$

**1.3 Dead-time, reverse-recovery, and gate-drive losses**

$$
P_{dt} = 2V_f I_{ph} t_{dt} f_{sw},\qquad
P_{rr} = Q_{rr}V_{bus}f_{sw},\qquad
P_g = 2Q_g V_{GS} f_{sw}
$$

**1.4 Copper and capacitor losses**

$$
P_{Cu} = I_{tot}^2 R_{plane},\qquad
P_{cap} = I_{rip,rms}^2\,ESR,\quad I_{rip,rms}\approx I_{tot}\sqrt{D(1-D)}
$$

**1.5 Loss budget**

The per-motor values below use T_j ≈ 100 °C, which gives a resistance factor of 1.375.

| Loss term | Per motor | × 4 motors |
|---|---|---|
| Conduction: 400 × 3.0 mΩ × 1.375 | 1.65 W | 6.6 W |
| Switching: 0.5 × 22.2 × 20 × 40 ns × 48 kHz | 0.43 W | 1.7 W |
| Dead time | 0.31 W | 1.2 W |
| Reverse recovery | 0.05 W | 0.2 W |
| Gate drive | 0.03 W | 0.1 W |
| Copper planes (R ≈ 0.3 mΩ) | — | 1.9 W |
| Capacitor ESR | — | ~0.5 W |
| **Total P_tot** | | **≈ 12.2 W** |

The volumetric source term is q(x) = Σ P_k / V_k over each component footprint (FET dies, bus planes, capacitors). It is zero elsewhere.

---

## 2. Governing Conduction Equations

**2.1 3-D steady heat equation (anisotropic laminate)**

$$
\nabla\cdot(\mathbf{k}\nabla T) + q(\mathbf{x},T) = 0,\qquad
\mathbf{k}=\mathrm{diag}(k_\parallel,k_\parallel,k_\perp)
$$

**2.2 Effective laminate conductivities**

With t_Cu = 0.42 mm and t_FR4 = 1.18 mm:

- In-plane (layers in parallel):
$$k_\parallel=\frac{\sum k_i t_i}{\sum t_i}=\frac{398(0.42)+0.3(1.18)}{1.6}\approx 105\ \mathrm{W/m\,K}$$
- Through-plane (layers in series):
$$k_\perp=\frac{\sum t_i}{\sum t_i/k_i}\approx 0.41\ \mathrm{W/m\,K}$$

Thermal vias under the FETs raise k_⊥ locally to about 5–20 W/m·K.

**2.3 Boundary conditions (top and bottom faces)**

$$
-\mathbf{k}\nabla T\cdot\mathbf{n} = h_{conv}(T-T_\infty) + \varepsilon\sigma(T^4-T_\infty^4)
$$

Linearize the radiation term as h_rad = 4εσT̄³. With ε = 0.9 and T̄ ≈ 350 K, h_rad ≈ 8.8 W/m²K, so h_eff ≈ 89 W/m²K.

**2.4 2.5-D shell reduction**

Because the board is thin, the problem reduces to an in-plane equation:

$$
k_\parallel t\,\nabla^2 T - 2h_{eff}(T-T_\infty) + q''(x,y)=0
$$

The characteristic spreading length is:

$$
m=\sqrt{\frac{2h_{eff}}{k_\parallel t}}=\sqrt{\frac{178}{0.168}}\approx 32.6\ \mathrm{m^{-1}},\qquad \frac{1}{m}\approx 31\ \mathrm{mm}
$$

This length is comparable to the board size, so the board spreads heat well. The remaining gradients concentrate at the FET clusters.

---

## 3. Finite-Element Formulation

**3.1 Weak form.** Find T ∈ H¹ such that, for all test functions w:

$$
\int_\Omega \nabla w\cdot\mathbf{k}\nabla T\,d\Omega + \int_{\Gamma} h\,wT\,d\Gamma = \int_\Omega wq\,d\Omega + \int_\Gamma h\,wT_\infty\,d\Gamma
$$

**3.2 Galerkin discretization.** Use T ≈ N Tₑ with shape functions N (hex8 for 3-D, quad4 for shell) and B = ∇N.

$$
\mathbf{K}_e=\int_{\Omega_e}\mathbf{B}^T\mathbf{k}\mathbf{B}\,d\Omega + \int_{\Gamma_e}h_{eff}\mathbf{N}^T\mathbf{N}\,d\Gamma
$$

$$
\mathbf{f}_e=\int_{\Omega_e}\mathbf{N}^Tq\,d\Omega+\int_{\Gamma_e}h_{eff}T_\infty\mathbf{N}^T\,d\Gamma
$$

Integrate with 2×2×2 Gauss quadrature.

**3.3 Assembly.** Assemble K T = f, where K is symmetric positive-definite. Solve with Cholesky or preconditioned conjugate gradient.

**3.4 Nonlinear iteration (Picard).** Two quantities depend on temperature: R_DS(T) and h_rad(T).

```
T = T_inf everywhere
repeat:
    q   = losses(R_DS(T_j))      # update heat sources
    h   = h_conv + 4*eps*sigma*T^3
    assemble K(h), f(q, h)
    T_new = solve(K, f)
until max|T_new - T| < 0.01 K
```

**3.5 Thermal-runaway criterion.** A stable solution exists only if:

$$
\frac{\partial P}{\partial T} = \alpha P_{cond,25} < G_{th} = h_{eff}A
$$

Here αP_cond,25 ≈ 0.005 × 4.8 = 0.024 W/K. G_th ≈ 0.23 W/K, so the design is stable with about a 10× margin.

---

## 4. Steady-State Solution

**4.1 Mean board rise (lumped estimate)**

$$
A = 2(0.036)^2 = 2.59\times10^{-3}\ \mathrm{m^2},\qquad
\Delta\bar T = \frac{P_{tot}}{h_{eff}A}=\frac{12.2}{0.231}\approx 53\ \mathrm{K}
$$

This gives T̄_board ≈ 83 °C.

**4.2 Hotspot superposition (single FET cluster)**

Model each cluster as a disk source of radius a ≈ 4 mm carrying P_c ≈ 2.4 W. For r ≤ a:

$$
\Delta T_{spread} = \frac{P_c}{2\pi k_\parallel t}\left[1-ma\,K_1(ma)\right]\frac{1}{(ma)^2}\cdot 2 \approx 6\text{–}9\ \mathrm{K}
$$

The junction-to-board resistance adds R_θ,jb ≈ 3 K/W, or about 2.5 K for a 0.8 W device.

**4.3 Predicted continuous (steady-state) temperature map**

```
        Motor 1 FETs          Motor 2 FETs
   ┌─────────────────────────────────────┐
   │  ███ 95°C                ███ 95°C   │
   │      88      84    84      88       │
   │  ~85      MCU/caps 80        ~85    │  T∞ = 30 °C
   │      88      84    84      88       │
   │  ███ 96°C  (bus-in edge)  ███ 96°C  │
   └─────────────────────────────────────┘
        Motor 3 FETs          Motor 4 FETs
```

| Location | Temperature |
|---|---|
| Junction (peak) | ≈ 100 °C (consistent with the assumed T_j, so the iteration has converged) |
| Board edges/center | ≈ 80–85 °C |
| Margin to 150 °C T_j,max | ~50 K |

---

## 5. Burst Versus Steady State

Board plus components have C_th ≈ 9 J/K, so:

$$
\tau = \frac{C_{th}}{G_{th}} \approx 40\ \mathrm{s},\qquad
T(t)=T_\infty+\Delta T_{ss}\,(1-e^{-t/\tau})
$$

- **Typical 5 s burst:** board rises only about 6 K, to ≈ 36 °C. FET junctions, with τ_j ~ 10–50 ms, track their local ~15 K rise above the board almost instantly.
- **Sustained 80 A:** converges to the ~100 °C junction state from §4.
- **Low prop wash** (h ≈ 15 W/m²K, e.g. hover in still air): ΔT ≈ 12.2 / 0.062 ≈ 196 K, so the board would not survive steady 80 A. Current limiting or a time-based derate is required.

To validate the model, compare FEM node temperatures against thermocouple or IR readings at the FET clusters. Then tune h_conv as the main uncertain parameter.