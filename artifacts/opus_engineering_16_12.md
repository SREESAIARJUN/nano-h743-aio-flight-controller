# 3-Axis Gimbal on ESP32-S3: FOC Mathematical Foundations

**Design Document, Cycle 16: Clarke/Park, SVPWM, and Field-Weakening Derivations**

---

## 0. Scope and Conventions

| Symbol | Meaning |
|---|---|
| $x_a, x_b, x_c$ | Phase quantities (voltage or current) |
| $\vec{x}_s = x_\alpha + jx_\beta$ | Stationary-frame space vector |
| $\vec{x}_r = x_d + jx_q$ | Rotor-frame space vector |
| $\theta_e = p\,\theta_m$ | Electrical angle ($p$ = pole pairs) |
| $\omega_e = d\theta_e/dt$ | Electrical angular velocity |
| $\psi_m$ | PM flux linkage, phase peak [Wb] |
| $V_{dc}$ | DC bus voltage |
| $T_s$ | PWM period |

**Transform convention.** This document uses the **amplitude-invariant** transform. The magnitude of the space vector equals the phase peak, which suits PI tuning and current-limit logic. The power-invariant variant is derived as well, for completeness.

**Machine assumptions.** Three-phase, wye-connected with an isolated neutral. Sinusoidal back-EMF. No saturation, iron loss, or cross-coupling saturation.

---

## 1. Clarke Transform

### 1.1 Space Vector Definition

Define the spatial operator $a = e^{j2\pi/3}$. The three windings are displaced by $0, 2\pi/3, 4\pi/3$. The space vector is:

$$
\vec{x}_s = k\left(x_a + a\,x_b + a^2 x_c\right)
$$

Expand $a = -\tfrac12 + j\tfrac{\sqrt3}{2}$ and $a^2 = -\tfrac12 - j\tfrac{\sqrt3}{2}$:

$$
x_\alpha = k\left(x_a - \tfrac12 x_b - \tfrac12 x_c\right), \qquad
x_\beta = k\,\tfrac{\sqrt3}{2}\left(x_b - x_c\right)
$$

### 1.2 Choosing $k$ (Amplitude Invariance)

Take a balanced set:

$$x_a = X\cos\phi, \quad x_b = X\cos(\phi - 2\pi/3), \quad x_c = X\cos(\phi + 2\pi/3)$$

Write each cosine as $\tfrac12(e^{j\cdot} + e^{-j\cdot})$:

$$
x_a + a x_b + a^2 x_c = \tfrac{X}{2}\left[e^{j\phi}(1 + 1 + 1) + e^{-j\phi}(1 + a^2 + a^4)\right] = \tfrac{3X}{2}e^{j\phi}
$$

The second bracket vanishes because $1 + a^2 + a^4 = 1 + a^2 + a = 0$.

Requiring $|\vec{x}_s| = X$ gives $k = 2/3$:

$$
\boxed{\begin{bmatrix}x_\alpha\\x_\beta\end{bmatrix} = \frac23\begin{bmatrix}1 & -\tfrac12 & -\tfrac12\\[2pt] 0 & \tfrac{\sqrt3}{2} & -\tfrac{\sqrt3}{2}\end{bmatrix}\begin{bmatrix}x_a\\x_b\\x_c\end{bmatrix}}
$$

### 1.3 Two-Sensor Reduction

With an isolated neutral, $i_a + i_b + i_c = 0$. Substitute $i_c = -i_a - i_b$:

$$
i_\alpha = \tfrac23\left(i_a + \tfrac12 i_a + \tfrac12 i_b - \tfrac12 i_b\right) = i_a
$$

$$
i_\beta = \tfrac23\cdot\tfrac{\sqrt3}{2}(i_b + i_a + i_b) = \frac{i_a + 2i_b}{\sqrt3}
$$

$$
\boxed{i_\alpha = i_a,\qquad i_\beta = \tfrac{1}{\sqrt3}(i_a + 2 i_b)}
$$

This is one multiply and two adds. It is the form used in the ISR.

### 1.4 Power Scaling

The amplitude-invariant transform is **not** power-invariant. For balanced sinusoidal quantities:

$$
p = v_a i_a + v_b i_b + v_c i_c = \tfrac32\,\mathrm{Re}\{\vec v_s \vec i_s^{*}\} = \tfrac32(v_\alpha i_\alpha + v_\beta i_\beta)
$$

With $k = \sqrt{2/3}$ the matrix becomes orthonormal, and $p = v_\alpha i_\alpha + v_\beta i_\beta$. The factor $\tfrac32$ therefore appears in every torque and power expression below.

### 1.5 Inverse Clarke

Use the zero-sequence-free inverse:

$$
\boxed{x_a = x_\alpha,\quad x_b = -\tfrac12 x_\alpha + \tfrac{\sqrt3}{2}x_\beta,\quad x_c = -\tfrac12 x_\alpha - \tfrac{\sqrt3}{2}x_\beta}
$$

---

## 2. Park Transform

### 2.1 Definition

Rotate into a frame aligned with the rotor flux (d-axis on the magnet north pole):

$$
\vec{x}_r = \vec{x}_s\,e^{-j\theta_e}
$$

$$
\boxed{\begin{aligned}
x_d &= \;\;\,x_\alpha\cos\theta_e + x_\beta\sin\theta_e\\
x_q &= -x_