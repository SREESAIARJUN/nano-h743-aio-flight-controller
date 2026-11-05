# 3-Axis Gimbal on ESP32-S3: FOC Mathematical Foundations (Cycle 5)

**Scope:** Rigorous derivation of the Clarke and Park transforms, the PMSM dq model, SVPWM duty-cycle equations, and field-weakening control, followed by ESP32-S3 implementation mapping.

---

## 0. Notation and Conventions

| Symbol | Meaning |
|---|---|
| $x_a, x_b, x_c$ | Phase quantities (voltage or current) |
| $\vec{x}_s = x_\alpha + jx_\beta$ | Stationary-frame space vector |
| $\vec{x}_r = x_d + jx_q$ | Rotor-frame (synchronous) space vector |
| $\theta_e = p\,\theta_m$ | Electrical angle, with $p$ = pole pairs |
| $\omega_e = d\theta_e/dt$ | Electrical angular velocity |
| $R_s, L_d, L_q$ | Phase resistance and dq inductances |
| $\psi_m$ | PM flux linkage (peak, per phase) |
| $V_{dc}$ | DC bus voltage |
| $T_s$ | PWM period |
| $a = e^{j2\pi/3}$ | 120° rotation operator |

**Convention:** amplitude-invariant transforms ($k = 2/3$). A balanced sinusoid of peak $I$ maps to a vector of magnitude $I$. This is why the torque equation carries a $3/2$ factor.

---

## 1. Clarke Transform (abc → αβ)

### 1.1 Space-Vector Definition

The three windings are spatially displaced by 0°, 120°, and 240°. The space vector is the weighted spatial sum:

$$
\vec{x}_s = k\left(x_a + a\,x_b + a^2 x_c\right), \quad a = -\tfrac{1}{2} + j\tfrac{\sqrt{3}}{2}, \quad a^2 = -\tfrac{1}{2} - j\tfrac{\sqrt{3}}{2}
$$

Separating real and imaginary parts:

$$
x_\alpha = k\left(x_a - \tfrac{1}{2}x_b - \tfrac{1}{2}x_c\right), \qquad
x_\beta = k\,\tfrac{\sqrt{3}}{2}\left(x_b - x_c\right)
$$

### 1.2 Choosing $k$: Amplitude Invariance

Take a balanced set $x_a = X\cos\phi$, $x_b = X\cos(\phi - 2\pi/3)$, $x_c = X\cos(\phi + 2\pi/3)$.

Write each cosine as $\cos\gamma = \tfrac{1}{2}(e^{j\gamma} + e^{-j\gamma})$:

$$
x_a + a x_b + a^2 x_c = \tfrac{X}{2}\Big[e^{j\phi}(1 + a\,a^{-1} + a^2 a^{-2}) + e^{-j\phi}(1 + a\cdot a + a^2\cdot a^2)\Big]
$$

The first bracket is $1 + 1 + 1 = 3$. The second bracket is $1 + a^2 + a^4 = 1 + a^2 + a = 0$. Therefore:

$$
x_a + a x_b + a^2 x_c = \tfrac{3}{2}X e^{j\phi}
$$

Setting $|\vec{x}_s| = X$ requires $\boxed{k = 2/3}$.

The alternative $k = \sqrt{2/3}$ gives a *power-invariant* (orthonormal) transform. We reject it because per-unit current limits then no longer equal phase peak amps, which complicates protection logic.

### 1.3 Full Matrix with Zero Sequence

$$
\begin{bmatrix} x_\alpha \\ x_\beta \\ x_0 \end{bmatrix}
= \frac{2}{3}
\begin{bmatrix}
1 & -\tfrac12 & -\tfrac12 \\
0 & \tfrac{\sqrt3}{2} & -\tfrac{\sqrt3}{2} \\
\tfrac12 & \tfrac12 & \tfrac12
\end{bmatrix}
\begin{bmatrix} x_a \\ x_b \\ x_c \end{bmatrix}
$$

### 1.4 Two-Sensor Reduction

For a wye winding with an isolated neutral, KCL gives $i_a + i_b + i_c = 0$. Substitute $i_c = -i_a - i_b$:

$$
i_\alpha = \tfrac{2}{3}\left(i_a + \tfrac12 i_a + \tfrac12 i_b - \tfrac12 i_b\right) = i_a
$$

$$
i_\beta = \tfrac{2}{3}\cdot\tfrac{\sqrt3}{2}\left(i_b + i_a + i_b\right) = \frac{i_a + 2i_b}{\sqrt3}
$$

$$
\boxed{i_\alpha = i_a, \qquad i_\beta = \frac{1}{\sqrt3}\left(i_a + 2 i_b\right)}
$$

This costs 1 multiply and 2 adds. Two shunts per motor saves 3 ADC channels across the gimbal, which matters on the S3 (see §6).

**Robustness option:** with three shunts, also compute $i_0 = (i_a + i_b + i_c)/3$. A non-zero $i_0$ indicates ADC offset drift or a sampling fault, so it serves as a free diagnostic.

### 1.5 Inverse Clarke

$$
x_a = x_\alpha, \qquad
x_b = -\tfrac12 x_\alpha + \tfrac{\sqrt3}{2}x_\beta, \qquad
x_c = -\tfrac12 x_\alpha - \tfrac{\sqrt3}{2}x_\beta
$$

---

## 2. Park Transform (αβ → dq)

### 2.1 Derivation

The rotor frame rotates at $\theta_e$. A vector expressed in rotor coordinates is the stationary vector de-rotated by $\theta_e$:

$$
\vec{x}_r = \vec{x}_s\, e^{-j\theta_e}
$$

Expanding $(x_\alpha + jx_\beta)(\cos\theta_e - j\sin\theta_e)$:

$$
\boxed{
\begin{aligned}
x_d &= \phantom{-}x_\alpha\cos\theta_e + x_\beta\sin\theta_e \\
x_q &= -x_\alpha\sin\theta_e + x_\beta\cos\theta_e
\end{aligned}}
$$

The inverse Park transform is $\vec{x}_s = \vec{x}_r e^{j\theta_e}$:

$$
x_\alpha = x_d\cos\theta_e - x_q\sin\theta_e, \qquad
x_\beta = x_d\sin\theta_e + x_q\cos\theta_e
$$

### 2.2 Why dq: The Rotating-Frame Voltage Equation

The stator voltage equation in the stationary frame is:

$$
\vec{v}_s = R_s\vec{i}_s + \frac{d\vec{\psi}_s}{dt}
$$

Substitute $\vec{x}_s = \vec{x}_r e^{j\theta_e}$ and apply the product rule:

$$
\frac{d}{dt}\left(\vec{\psi}_r e^{j\theta_e}\right) = \left(\dot{\vec{\psi}}_r + j\omega_e\vec{\psi}_r\right)e^{j\theta_e}
$$

Cancelling $e^{j\theta_e}$ gives:

$$
\vec{v}_r = R_s\vec{i}_r + \frac{d\vec{\psi}_r}{dt} + j\omega_e\vec{\psi}_r
$$

The term $j\omega_e\vec{\psi}_r$ is the **speed voltage** (back-EMF plus cross-coupling). It arises purely from the frame rotation.

The rotor-frame flux linkages are:

$$
\psi_d = L_d i_d + \psi_m, \qquad \psi_q = L_q i_q
$$

Separating real and imaginary parts:

$$
\boxed{
\begin{aligned}
v_d &= R_s i_d + L_d\frac{di_d}{dt} - \omega_e L_q i_q \\
v_q &= R_s i_q + L_q\frac{di_q}{dt} + \omega_e L_d i_d + \omega_e\psi