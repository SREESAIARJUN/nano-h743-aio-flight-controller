# 3-Axis Gimbal on ESP32-S3: FOC Mathematical Core (Cycle 7)

**Document scope:** Derivations for the Clarke and Park transforms, SVPWM duty-cycle synthesis and field weakening, with implementation notes for the ESP32-S3 MCPWM peripheral.

**Target plant:** Three surface-mount PMSM gimbal motors, typically 7–14 pole pairs, phase resistance of roughly 5–15 Ω, and inductance of roughly 1–5 mH.

---

## 0. Conventions

| Symbol | Meaning |
|---|---|
| $a, b, c$ | Phase quantities, star-connected, isolated neutral |
| $\alpha,\beta$ | Stationary orthogonal frame, with α aligned to phase *a* |
| $d,q$ | Rotor frame, with d aligned to the rotor magnet flux $\psi_m$ |
| $\theta_e = p\,\theta_m$ | Electrical angle, where $p$ is the number of pole pairs |
| $\omega_e = \dot\theta_e$ | Electrical angular speed |
| $V_{dc}$ | DC bus voltage |
| $a_{op} = e^{j2\pi/3}$ | Rotation operator |

Unless stated otherwise, all transforms are **amplitude-invariant**. This is the industry convention, and it means a dq current magnitude equals the phase current peak.

---

## 1. Clarke Transform (abc → αβ)

### 1.1 Space-vector definition

Define the complex space vector:

$$
\vec{x}_s = k\left(x_a + a_{op}\,x_b + a_{op}^2\,x_c\right) = x_\alpha + j x_\beta
$$

### 1.2 Choosing the scaling constant *k*

Take a balanced sinusoidal set:

$$
x_a = X\cos\theta,\quad x_b = X\cos(\theta - \tfrac{2\pi}{3}),\quad x_c = X\cos(\theta + \tfrac{2\pi}{3})
$$

Expand each cosine with Euler's formula, $\cos\phi = \tfrac12(e^{j\phi}+e^{-j\phi})$:

$$
x_a + a_{op}x_b + a_{op}^2 x_c = \frac{X}{2}\Big[\underbrace{e^{j\theta}(1+1+1)}_{3e^{j\theta}} + e^{-j\theta}\underbrace{(1 + e^{j4\pi/3} + e^{j8\pi/3})}_{=\,0}\Big] = \frac{3}{2}X e^{j\theta}
$$

The choice of $k$ follows from what we want preserved:

- **Amplitude-invariant:** choose $k = 2/3$, which gives $\vec{x}_s = X e^{j\theta}$.
- **Power-invariant:** choose $k = \sqrt{2/3}$. The transform matrix is then orthonormal.

### 1.3 Real-valued form

Separate real and imaginary parts, using $a_{op} = -\tfrac12 + j\tfrac{\sqrt3}{2}$:

$$
\begin{bmatrix}x_\alpha\\x_\beta\\x_0\end{bmatrix}
=\frac{2}{3}
\begin{bmatrix}
1 & -\tfrac12 & -\tfrac12\\
0 & \tfrac{\sqrt3}{2} & -\tfrac{\sqrt3}{2}\\
\tfrac12 & \tfrac12 & \tfrac12
\end{bmatrix}
\begin{bmatrix}x_a\\x_b\\x_c\end{bmatrix}
$$

### 1.4 Reduced two-sensor form (used in firmware)

With an isolated neutral, $i_a + i_b + i_c = 0$, so $x_0 = 0$. Substituting $i_c = -i_a - i_b$:

$$
i_\alpha = \tfrac23\left(i_a - \tfrac12 i_b - \tfrac12 i_c\right) = i_a
$$

$$
i_\beta = \tfrac{2}{3}\cdot\tfrac{\sqrt3}{2}(i_b - i_c) = \frac{i_a + 2 i_b}{\sqrt3}
$$

This form needs only two current shunts.

### 1.5 Inverse Clarke

$$
x_a = x_\alpha,\qquad
x_b = -\tfrac12 x_\alpha + \tfrac{\sqrt3}{2}x_\beta,\qquad
x_c = -\tfrac12 x_\alpha - \tfrac{\sqrt3}{2}x_\beta
$$

### 1.6 Power bookkeeping

The amplitude-invariant scaling does not conserve power numerically, so a factor of 3/2 appears:

$$
p = v_a i_a + v_b i_b + v_c i_c = \tfrac{3}{2}\left(v_\alpha i_\alpha + v_\beta i_\beta\right)
$$

This factor reappears in the torque equation in §2.4.

---

## 2. Park Transform (αβ → dq)

### 2.1 Definition

Rotate the stationary vector by $-\theta_e$:

$$
\vec{x}_{dq} = \vec{x}_{\alpha\beta}\,e^{-j\theta_e}
$$

In matrix form:

$$
\begin{bmatrix}x_d\\x_q\end{bmatrix}=
\begin{bmatrix}\cos\theta_e & \sin\theta_e\\ -\sin\theta_e & \cos\theta_e\end{bmatrix}
\begin{bmatrix}x_\alpha\\x_\beta\end{bmatrix}
$$

The inverse uses the transpose, because the matrix is orthonormal:

$$
x_\alpha = x_d\cos\theta_e - x_q\sin\theta_e,\qquad
x_\beta = x_d\sin\theta_e + x_q\cos\theta_e
$