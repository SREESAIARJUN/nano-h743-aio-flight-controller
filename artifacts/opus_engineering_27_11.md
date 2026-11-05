# 3-Axis Gimbal FOC Core: Mathematical Derivations and ESP32-S3 Implementation

**Platform:** ESP32-S3 with three gimbal-class PMSMs (outer-rotor, high pole count, high winding resistance)
**Scope:** Clarke and Park transforms, SVPWM duty synthesis, field-weakening, and discrete-time implementation

---

## 0. System Context and Conventions

| Symbol | Meaning |
|---|---|
| $x_a, x_b, x_c$ | Phase quantities (voltage or current) |
| $\theta_e = p\,\theta_m$ | Electrical angle, where $p$ is the number of pole pairs |
| $\omega = \dot\theta_e$ | Electrical angular velocity (rad/s) |
| $R_s, L_d, L_q$ | Phase resistance and dq inductances |
| $\psi_m$ | PM flux linkage, peak per phase (Wb, per electrical rad) |
| $V_{dc}$ | DC bus voltage |
| $T_s$ | PWM period |

All transforms below use the **amplitude-invariant** convention. Under it, a balanced sinusoid of peak $X$ maps to a space vector of magnitude $X$. This keeps current limits, sensor scaling and voltage limits in physical "peak phase" units, which removes a class of scaling bugs.

---

## 1. Clarke Transform (abc → αβ)

### 1.1 Space-vector definition

Define the unit rotation $a = e^{j2\pi/3}$. The space vector is

$$
\vec{x} = x_\alpha + j x_\beta = K\left(x_a + a\,x_b + a^2 x_c\right)
$$

Choose $K$ so that a balanced set maps to a vector of the same amplitude:

$$
x_a = X\cos\theta,\quad x_b = X\cos(\theta - \tfrac{2\pi}{3}),\quad x_c = X\cos(\theta + \tfrac{2\pi}{3})
$$

Write $\cos\phi = \tfrac12(e^{j\phi}+e^{-j\phi})$ and expand:

$$
\sum_{k=0}^{2} \cos\!\left(\theta - \tfrac{2\pi k}{3}\right) e^{j2\pi k/3}
= \tfrac12 \sum_{k} \left[e^{j\theta} + e^{-j\theta} e^{j4\pi k/3}\right]
= \tfrac{3}{2}e^{j\theta} + \tfrac12 e^{-j\theta}\underbrace{\sum_k e^{j4\pi k/3}}_{=0}
$$

The bracketed sum is zero because it adds the three cube roots of unity. Therefore $\vec{x} = K\cdot\tfrac{3}{2}X e^{j\theta}$, and amplitude invariance requires $K = \tfrac{2}{3}$.

### 1.2 Matrix form

Take real and imaginary parts, using $a = -\tfrac12 + j\tfrac{\sqrt3}{2}$ and $a^2 = -\tfrac12 - j\tfrac{\sqrt3}{2}$:

$$
\begin{bmatrix} x_\alpha \\ x_\beta \\ x_0 \end{bmatrix}
= \frac{2}{3}
\begin{bmatrix}
1 & -\tfrac12 & -\tfrac12 \\
0 & \tfrac{\sqrt3}{2} & -\tfrac{\sqrt3}{2} \\
\tfrac12 & \tfrac12 & \