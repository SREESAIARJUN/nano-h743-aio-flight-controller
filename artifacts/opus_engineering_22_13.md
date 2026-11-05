# 3-Axis Gimbal on ESP32-S3: Mathematical Foundations of FOC (Cycle 22)

**Scope:** This covers the Clarke and Park transforms, the dq machine model, SVPWM duty-cycle synthesis, and field weakening. Each result is derived, not just stated. The final sections map the math onto ESP32-S3 peripherals and give a verification plan.

---

## 0. Plant Assumptions and Notation

Gimbal motors (e.g., GBM2804, GM3506, iPower GM4108) are **surface-mounted PMSMs (SPMSM)** with characteristics that shape every design choice below:

| Parameter | Typical value | Consequence |
|---|---|---|
| Phase resistance $R_s$ | 5–15 Ω | Resistive drop dominates; it must stay in the voltage-limit model |
| Inductance $L_d \approx L_q = L_s$ | 1–4 mH | Small saliency, so MTPA reduces to $i_d=0$ |
| Pole pairs $p$ | 7–11 | High electrical frequency per mechanical rev |
| PM flux linkage $\psi_m$ | 5–20 mWb | Characteristic current $\psi_m/L_s \gg I_{max}$, which limits field weakening (§4) |
| Supply $V_{dc}$ | 12–16.8 V (3S/4S) | |

**Notation:**
- Phase quantities: $x_a, x_b, x_c$
- Stationary frame: $x_\alpha, x_\beta$
- Rotor frame: $x_d, x_q$
- Electrical angle and speed: $\theta = p\,\theta_m$, $\omega = d\theta/dt$
- Rotation operator: $a = e^{j2\pi/3}$

---

## 1. Clarke Transform (abc → αβ0)

### 1.1 Space-vector definition

Define the complex space vector, with a scaling constant $K$ to be chosen:

$$
\vec{x}_s = K\left(x_a + a\,x_b + a^2 x_c\right),\qquad a = -\tfrac12 + j\tfrac{\sqrt3}{2},\quad a^2 = -\tfrac12 - j\tfrac{\sqrt3}{2}
$$

Separating real and imaginary parts:

$$
x_\alpha = K\left(x_a - \tfrac12 x_b - \tfrac12 x_c\right),\qquad
x_\beta = K\,\tfrac{\sqrt3}{2}\left(x_b - x_c\right)
$$

Add the zero-sequence component $x_0 = K_0(x_a+x_b+x_c)$ to obtain the full matrix:

$$
\begin{bmatrix}x_\alpha\\x_\beta\\x_0\end{bmatrix}
=
\underbrace{\begin{bmatrix}
K & -K/2 & -K/2\\
0 & K\sqrt3/2 & -K\sqrt3/2\\
K_0 & K_0 & K_0
\end{bmatrix}}_{\mathbf{C}}
\begin{bmatrix}x_a\\x_b\\x_c\end{bmatrix}
$$

### 1.2 Choosing $K$: two invariance conditions

**Amplitude-invariant ($K = 2/3$).**
Apply a balanced set $x_a = X\cos\phi$, $x_b = X\cos(\phi - 2\pi/3)$, $x_c = X\cos(\phi+2\pi/3)$. Then:

$$
x_a + a x_b + a^2 x_c = \tfrac{3}{2}X e^{j\phi}
\;\Rightarrow\; |\vec x_s| = \tfrac{3}{2}KX
$$

So $K = 2/3$ gives $|\vec x_s| = X$: the vector magnitude equals the phase peak. The cost is that power is no longer invariant:

$$
P = v_a i_a + v_b i_b + v_c i_c = \tfrac32\left(v_\alpha i_\alpha + v_\beta i_\beta\right)
$$

**Power-invariant ($K = \sqrt{2/3}$, $K_0 = 1/\sqrt3$).**
Here $\mathbf C$ is orthonormal ($\mathbf C^{T}\mathbf C = \mathbf I$). Therefore:

$$
P = \mathbf v_{abc}^T \mathbf i_{abc} = \mathbf v_{\alpha\beta0}^T\mathbf C\,\mathbf C^{T}\,\mathbf i_{\alpha\beta0} = v_\alpha i_\alpha + v_\beta i_\beta + v_0 i_0
$$

**Design decision: amplitude-invariant.** PI gains, current limits and telemetry then read directly in phase-peak amperes. The 3/2 factor appears in the torque equation and is carried explicitly.

### 1.3 Reduced two-sensor Clarke

For a star connection with floating neutral, $i_a + i_b + i_c = 0$. Substitute $i_c = -i_a - i_b$:

$$
i_\alpha = \tfrac23\left(i_a + \tfrac12 i_a + \tfrac12 i_b - \tfrac12 i_b\right) = i_a
$$

$$
i_\beta = \tfrac23\cdot\tfrac{\sqrt3}{2}\left(i_b + i_a + i_b\right) = \frac{i_a + 2 i_b}{\sqrt3}
$$

This needs one multiply and two adds, so only two current shunts are required.

**Robustness variant (three shunts).** Discard the phase whose low-side shunt has the shortest sampling window, i.e., the phase with the largest duty. Reconstruct it from the other two before applying the formula above.

### 1.4 Inverse Clarke (amplitude-invariant, $x_0 = 0$)

$