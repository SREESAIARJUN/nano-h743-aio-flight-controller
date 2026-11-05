# 3-Axis Gimbal on ESP32-S3: FOC Transform, SVPWM and Field-Weakening Derivations (Cycle 10)

**Document scope:** This document derives the field-oriented control (FOC) mathematics from first principles, specialized for gimbal-class SPMSMs (high $R$, high pole count, low speed). It then maps the results onto ESP32-S3 peripherals.

**Notation:**
- $p$ is the number of pole pairs.
- $\theta_e = p\,\theta_m$ is the electrical angle.
- $\omega = \dot\theta_e$ is the electrical speed.
- $V_{dc}$ is the bus voltage.
- $T_s$ is the PWM period.

---

## 0. Plant Assumptions (Gimbal-Specific)

| Parameter | Typical gimbal BLDC (e.g. 2804/4108 class) | Consequence |
|---|---|---|
| $R_s$ | 5–15 Ω | Ohmic drop dominates. $R$ **cannot** be neglected in voltage limits. |
| $L_d \approx L_q = L$ | 1–4 mH | Surface-mount PMSM, so reluctance torque is ≈ 0. |
| $p$ | 7–11 | Electrical frequency is 7–11× mechanical. Angle-latency error is amplified. |
| $\psi_m$ | 3–8 mWb | The characteristic current $\psi_m/L$ is ≈ 1–4 A, comparable to $I_{max}$. |
| $\omega_{m}$ (operating) | mostly < 10 rad/s, slews up to ~60 rad/s | Field weakening matters only for fast slews and low-battery conditions. |

---

## 1. Clarke Transform

### 1.1 Space-vector definition

Let the three phase quantities $x_a, x_b, x_c$ be spatially displaced by $0, \tfrac{2\pi}{3}, \tfrac{4\pi}{3}$. With $\mathbf{a} = e^{j2\pi/3}$, define the space vector

$$
\vec{x} = k\left(x_a + \mathbf{a}\,x_b + \mathbf{a}^2 x_c\right) = x_\alpha + j x_\beta .
$$

### 1.2 Choice of $k$ (amplitude invariance)

Take a balanced sinusoidal set $x_n = X\cos(\theta - n\tfrac{2\pi}{3})$ for $n = 0, 1, 2$. Write each cosine as $\tfrac12(e^{j\phi}+e^{-j\phi})$:

$$
\sum_{n=0}^{2} X\cos\!\left(\theta - \tfrac{2\pi n}{3}\right)e^{j\frac{2\pi n}{3}}
= \frac{X}{2}\Big[\underbrace{\textstyle\sum_n e^{j\theta}}_{3e^{j\theta}} + \underbrace{\textstyle\sum_n e^{-j\theta}e^{j\frac{4\pi n}{3}}}_{0}\Big] = \tfrac{3}{2}X e^{j\theta}.
$$

The second sum vanishes because the three terms are the cube roots of unity.

- Choosing $k = \tfrac{2}{3}$ gives $\vec{x} = Xe^{j\theta}$. The vector magnitude equals the phase peak (**amplitude-invariant**). This is the convention adopted.
- Choosing $k = \sqrt{2/3}$ gives the power-invariant form (orthonormal matrix).

### 1.3 Matrix form

Using $\mathbf a = -\tfrac12 + j\tfrac{\sqrt3}{2}$ and $\mathbf a^2 = -\tfrac12 - j\tfrac{\sqrt3}{2}$:

$$
\begin{bmatrix} x_\alpha \\ x_\beta \\ x_0\end{bmatrix}
= \frac{2}{3}\begin{bmatrix} 1 & -\tfrac12 & -\tfrac12 \\ 0 & \tfrac{\sqrt3}{2} & -\tfrac{\sqrt3}{2} \\ \tfrac12 & \tfrac12 & \tfrac12 \end{bmatrix}
\begin{bmatrix} x_a \\ x_b \\ x_c\end{bmatrix}.
$$

**Reduced two-sensor form.** For a star-connected motor with isolated neutral, $i_a + i_b + i_c = 0$. Substituting $i_c = -i_a - i_b$:

$$
i_\alpha = \tfrac{2}{3}\left(i_a - \tfrac12 i_b - \tfrac12(-i_a - i_b)\right) = i_a,
\qquad
i_\beta = \tfrac{2}{3}\cdot\tfrac{\sqrt3}{2}(i_b - i_c) = \frac{i_a + 2 i_b}{\sqrt3}.
$$

This form requires only two current measurements: 1 multiply and 1 add on the S3.

### 1.4 Inverse Clarke

Assuming zero-sequence $x_0 = 0$:

$$
x_a = x_\alpha,\qquad
x_b = -\tfrac12 x_\alpha + \tfrac{\sqrt3}{2}x_\beta,\qquad
x_c = -\tfrac12 x_\alpha - \tfrac{\sqrt3}{2}x_\beta .
$$

### 1.5 Power scaling (needed for torque)

$$
P = v_a i_a + v_b i_b + v_c