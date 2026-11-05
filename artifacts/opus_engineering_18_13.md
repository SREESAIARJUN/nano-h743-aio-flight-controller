# Thermal Model of a 4-in-1 ESC at 80 A Total Burst Current

**Scope.** I don't have the specifics of "Cycle 18," so this uses a representative 30.5 × 30.5 mm 4-in-1 ESC. Any parameter can be swapped for your actual values.

## 0. Baseline Assumptions

| Parameter | Value |
|---|---|
| Supply | 6S Li-ion, V_bus = 25.2 V |
| Current | 80 A total, so I_ch = 20 A per channel |
| Power stage | 6 MOSFETs per channel (24 total), PDFN 3.3×3.3 |
| R_DS(on) | 1.2 mΩ at 25 °C, α_R = 0.005 K⁻¹ |
| Switching | f_sw = 48 kHz, t_r = t_f = 20 ns, t_dt = 100 ns, V_f = 0.8 V |
| Gate drive | Q_g = 50 nC, V_g = 10 V |
| PCB | 36 × 36 mm, 1.6 mm, 8 layers × 2 oz Cu (70 µm each) |
| Shunts | 0.5 mΩ per channel |
| Bus copper | ≈ 0.2 mΩ, carries the full 80 A |
| Ambient | T_∞ = 30 °C |
| Convection | h = 80 W/m²K (prop-wash forced convection); 10 W/m²K in still air |

---

## 1. Heat-Generation Physics

### 1.1 MOSFET losses (per channel)

In 6-step commutation with synchronous rectification, one high-side and one low-side device effectively carry I_ch at all times.

**Conduction loss** (with temperature-dependent resistance):

$$
P_{cond} = 2\,I_{ch}^2\,R_{DS}(T_j), \qquad R_{DS}(T_j) = R_{25}\left[1 + \alpha_R (T_j - 25)\right]
$$

**Switching loss** (hard-switched, one switching leg per PWM period):

$$
P_{sw} = \tfrac12 V_{bus} I_{ch}(t_r + t_f) f_{sw} = \tfrac12(25.2)(20)(40\times10^{-9})(48\times10^3) = 0.48\ \text{W}
$$

**Dead-time body-diode loss:**

$$
P_{dt} = 2\,V_f\,I_{ch}\,t_{dt}\,f_{sw} = 0.15\ \text{W}
$$

**Gate drive loss** (dissipated mostly in the driver and gate resistors):

$$
P_g \approx Q_g V_g f_{sw}\times N_{sw} \approx 0.05\ \text{W}
$$

### 1.2 Joule losses in copper and shunts

$$
P_{shunt} = I_{ch}^2 R_{sh} = 0.20\ \text{W/ch}, \qquad P_{trace} \approx 0.20\ \text{W/ch}, \qquad P_{bus} = (80)^2(0.2\,\text{m}\Omega) = 1.28\ \text{W}
$$

### 1.3 Loss budget (first pass, T_j ≈ 100 °C, so R_DS ≈ 1.65 mΩ)

| Source | Per channel | ×4 / board |
|---|---|---|
| Conduction | 1.32 W | 5.28 W |
| Switching | 0.48 W | 1.94 W |
| Dead-time | 0.15 W | 0.61 W |
| Gate | 0.05 W | 0.20 W |
| Shunt + traces | 0.40 W | 1.60 W |
| Bus copper | — | 1.28 W |
| MCU / regulators | — | 0.30 W |
| **Total** | **≈ 2.4 W** | **Q_tot ≈ 11.2 W** |

The volumetric source term for the FEM is each loss divided by its component volume:

$$
\dot q'''_i = \frac{P_i}{V_i}
$$

---

## 2. Governing Conduction Equation

### 2.1 Strong form (anisotropic, nonlinear source)

$$
\rho c_p \frac{\partial T}{\partial t} = \nabla\cdot\left(\mathbf{k}\,\nabla T\right) + \dot q'''(\mathbf{x}, T) \quad \text{in } \Omega
$$

At steady state the time derivative vanishes:

$$
\nabla\cdot(\mathbf{k}\nabla T) + \dot q'''(T) = 0
$$

### 2.2 Orthotropic PCB conductivity

The copper volume fraction is φ = (8 × 70 µm)/1.6 mm = 0.35.

**In-plane** (parallel rule):

$$
k_{xy} = \varphi k_{Cu} + (1-\varphi)k_{FR4} = 0.35(390) + 0.65(0.3) \approx 137\ \text{W/mK}
$$

**Through-plane** (series rule):

$$
k_z = \left[\frac{\varphi}{k_{Cu}} + \frac{1-\varphi}{k_{FR4}}\right]^{-1} \approx 0.46\ \text{W/mK}
$$

Under MOSFET pads with a via array of area fraction φ_v (plated copper):

$$
k_z^{via} \approx \varphi_v k_{Cu} + (1-\varphi_v)k_z \quad (\varphi_v = 0.05 \Rightarrow k_z^{via} \approx 20\ \text{W/mK})
$$

The resulting conductivity tensor is

$$
\mathbf{k} = \text{diag}(k_{xy},\,k_{xy},\,k_z)
$$

### 2.3 Boundary conditions

**Top and bottom surfaces** (convection plus linearized radiation):

$$
-\mathbf{n}\cdot\mathbf{k}\nabla T = h_{eff}(T - T_\infty), \qquad h_{eff} = h + 4\varepsilon\sigma \bar T^3 \approx 80 + 6 = 86\ \text{W/m}^2\text{K}
$$

**Edges:** adiabatic. Their area is negligible (4 × 36 × 1.6 mm²).

**Mounting holes:** Robin condition with standoff conductance G_s ≈ 0.05 W/K each, or adiabatic if nylon standoffs are used.

**Solder-pad wires:** treat as Robin fins,

$$
G_{wire} = \sqrt{hPkA}\,\tanh(mL)
$$

---

## 3. Finite-Element Formulation

### 3.1 Weak (Galerkin) form

Find T ∈ H¹(Ω) such that for all test functions w:

$$
\int_\Omega \nabla w\cdot\mathbf{k}\nabla T\,d\Omega + \int_{\Gamma_R} h_{eff}\,wT\,d\Gamma = \int_\Omega w\,\dot q'''\,d\Omega + \int_{\Gamma_R} h_{eff}\,wT_\infty\,d\Gamma
$$

### 3.2 Discretization

Approximate T by nodal shape functions, T ≈ **N T**ₑ with **B** = ∇**N** (8-node hexahedra for the layer stack, or 4-node tets).

**Element conductivity matrix:**

$$
\mathbf{K}_e = \int_{\Omega_e}\mathbf{B}^T\mathbf{k}\,\mathbf{B}\,d\Omega
$$

**Convection matrix:**

$$
\mathbf{H}_e = \int_{\Gamma_e} h_{eff}\,\mathbf{N}^T\mathbf{N}\,d\Gamma
$$

**Source vector:**

$$
\mathbf{f}_e = \int_{\Omega_e}\mathbf{N}^T\dot q'''\,d\Omega + \int_{\Gamma_e} h_{eff}T_\infty\mathbf{N}^T\,d\Gamma
$$

**Assembled system:**

$$
\left(\mathbf{K} + \mathbf{H}\right)\mathbf{T} = \mathbf{f}(\mathbf{T})
$$

For the transient case, add the capacitance matrix and step with backward Euler:

$$
\mathbf{C}_e = \int \rho c_p \mathbf{N}^T\mathbf{N}\,d\Omega
$$

$$
\left(\tfrac{1}{\Delta t}\mathbf{C} + \mathbf{K} + \mathbf{H}\right)\mathbf{T}^{n+1} = \tfrac{1}{\Delta t}\mathbf{C}\mathbf{T}^n + \mathbf{f}^{n+1}
$$

### 3.3 Electro-thermal coupling (Picard iteration)

Because R_DS(T_j) feeds back into the source term, iterate:

1. Guess **T**⁽⁰⁾ = T_∞ + 50 K.
2. Compute R_DS(T_j⁽ᵏ⁾), then P_cond, then **f**⁽ᵏ⁾.
3. Solve (**K** + **H**) **T**⁽ᵏ⁺¹⁾ = **f**⁽ᵏ⁾.
4. Stop when ‖**T**⁽ᵏ⁺¹⁾ − **T**⁽ᵏ⁾‖∞ < 0.1 K. Convergence typically takes 3–5 iterations.

The loop is a contraction, and stability is guaranteed, if:

$$
\frac{\partial P}{\partial T}\,R_{th,ja} = 2I^2R_{25}\alpha_R \cdot R_{th} < 1
$$

Here that gives 0.0048 W/K × 4.8 K/W ≈ 0.02 per channel, which is far from thermal runaway.

### 3.4 Minimal 2-D reference solver

This lumps the board into a plate with surface convection:

```python
import numpy as np, scipy.sparse as sp, scipy.sparse.linalg as spl
L, n = 0.036, 73; dx = L/(n-1)
kt  = 137*1.6e-3          # in-plane sheet conductance k*t  [W/K]
h2  = 2*86                # two convecting faces [W/m^2K]
Tinf= 30.0
q = np.zeros((n,n))
def src(T):                # MOSFET clusters at 4 corners + bus centre
    q[:] = 0
    for cx,cy in [(9,9),(9,27),(27,9),(27,27)]:
        Tj = T[int(cx/36*(n-1)),int(cy/36*(n-1))]
        Rds = 1.2e-3*(1+0.005*(Tj-25))
        P = 2*20**2*Rds + 0.48+0.15+0.05+0.40
        i,j = int(cx/36*(n-1)), int(cy/36*(n-1))
        q[i-3:i+4,j-3:j+4] += P/(49*dx*dx)
    q[n//2-4:n//2+5, :] += (1.28+0.3)/(9*n*dx*dx)
    return q.ravel()
N=n*n; I=np.arange(N).reshape(n,n)
A=sp.lil_matrix((N,N))
for i in range(n):
    for j in range(n):
        p=I[i,j]; A[p,p]=h2*dx*dx
        for ii,jj in [(i+1,j),(i-1,j),(i,j+1),(i,j-1)]:
            if 0<=ii<n and 0<=jj<n:
                A[p,p]+=kt; A[p,I[ii,jj]]-=kt
A=A.tocsr(); T=np.full(N,80.0)
for _ in range(8):
    T=spl.spsolve(A, src(T.reshape(n,n))*dx*dx + h2*dx*dx*Tinf)
print(T.min(), T.max())
```

---

## 4. Steady-State