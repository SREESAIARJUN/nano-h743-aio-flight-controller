# 3-Axis Gimbal FOC on ESP32-S3: Mathematical Foundations (Cycle 6)

**Scope:** Clarke and Park transforms, the dq machine model, SVPWM dwell-time and duty-cycle equations, overmodulation, and field-weakening (FW). Each derivation ends with an implementation-ready form for ESP32-S3.

---

## 0. Conventions and System Context

| Symbol | Meaning |
|---|---|
| $x_a, x_b, x_c$ | Phase quantities (voltage, current, or flux) |
| $\theta_e = p\,\theta_m$ | Electrical angle. $p$ is pole pairs; $\theta_m$ is the encoder angle. |
| $\omega = \dot\theta_e$ | Electrical angular velocity |
| $V_{dc}$ | DC bus voltage (3S Li-ion: 9.9–12.6 V) |
| $R_s, L_d, L_q, \psi_m$ | Phase resistance, dq inductances, PM flux linkage (peak, per phase) |
| $T_s$ | PWM period |

**Motor class assumption.** Gimbal motors (e.g., the 2804/4108 class) are **surface-mounted PM (SPM)** machines. That gives $L_d \approx L_q = L$. They also have **high $R_s$** (≈ 5–15 Ω) and **low $L$** (≈ 1–3 mH). This matters greatly in §4.

---

## 1. The Clarke Transform

### 1.1 Space-vector definition
Place the three phase axes in the complex plane at $0, \tfrac{2\pi}{3}, \tfrac{4\pi}{3}$. Let $a = e^{j2\pi/3}$. The space vector is

$$
\vec{x} = k\left(x_a + a\,x_b + a^2 x_c\right).
$$

### 1.2 Choosing $k$: amplitude-invariant form
Take a balanced set:

$$
x_a = X\cos\theta,\quad x_b = X\cos(\theta - \tfrac{2\pi}{3}),\quad x_c = X\cos(\theta - \tfrac{4\pi}{3}).
$$

Write $\cos\phi = \tfrac12(e^{j\phi}+e^{-j\phi})$. Then

$$
\sum_{n=0}^{2} \cos\!\left(\theta - \tfrac{2\pi n}{3}\right)e^{j2\pi n/3}
= \tfrac12\sum_{n}\left(e^{j\theta} + e^{-j\theta}e^{j4\pi n/3}\right)
= \tfrac{3}{2}e^{j\theta} + \tfrac12 e^{-j\theta}\underbrace{\sum_n e^{j4\pi n/3}}_{=0}.
$$

So $\vec{x} = k\cdot\tfrac32 X e^{j\theta}$.

Choosing **$k = 2/3$** makes $|\vec{x}| = X$. The vector length equals the phase peak. This is the convention used throughout, because current limits and ADC scaling map directly.

### 1.3 Real and imaginary parts
Use $a = -\tfrac12 + j\tfrac{\sqrt3}{2}$ and $a^2 = -\tfrac12 - j\tfrac{\sqrt3}{2}$:

$$
\boxed{\;x_\alpha = \tfrac23\left(x_a - \tfrac12 x_b - \tfrac12 x_c\right),\qquad
x_\beta = \tfrac{1}{\sqrt3}\left(x_b - x_c\right)\;}
$$

Add the zero-sequence row $x_0 = \tfrac13(x_a+x_b+x_c)$ to get the full invertible matrix:

$$
\begin{bmatrix}x_\alpha\\x_\beta\\x_0\end{bmatrix}
=\frac23\begin{bmatrix}1&-\tfrac12&-\tfrac12\\0&\tfrac{\sqrt3}{2}&-\tfrac{\sqrt3}{2}\\ \tfrac12&\tfrac12&\tfrac12\end{bmatrix}
\begin{bmatrix}x_a\\x_b\\x_c\end{bmatrix}.
$$

### 1.4 Two-sensor reduction
For a wye-connected motor with no neutral return, $i_a + i_b + i_c = 0$. Substitute $i_c = -i_a - i_b$:

$$
i_\alpha = i_a,\qquad i_\beta = \frac{i_