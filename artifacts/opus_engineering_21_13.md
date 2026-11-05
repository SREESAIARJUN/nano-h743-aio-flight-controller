# 3-Axis Brushless Gimbal on ESP32-S3: FOC Mathematical Foundations (Cycle 21)

**Document:** GIM-S3-ARCH-021 · **Scope:** current-loop math, modulation, flux control, and the ESP32-S3 mapping
**Conventions:** amplitude-invariant transforms, motor convention, electrical angle $\theta_e = p\,\theta_m$, $p$ = pole pairs.

---

## 0. Design Context and Key Assumptions

| Parameter | Baseline (per axis) | Note |
|---|---|---|
| Motor | 2204/2804-class gimbal BLDC, 14P → $p = 7$ | Surface PM, so $L_d \approx L_q$ |
| $R_s$ (phase) | 5–12 Ω | High R by design (thin wire) |
| $L_s$ | 1–3 mH | |
| $\lambda_{pm}$ | ≈ 3 mWb (derived in §6.6) | |
| $V_{dc}$ | 12.6 V (3S) | |
| PWM | 20 kHz center-aligned | Above the audible band |
| Current loop | 10–20 kHz | |
| Angle sensor | AS5048A/MT6701 (14-bit SPI/SSI) | |

**Architectural caveat:** gimbal motors operate far below base speed. §6 shows that field weakening is almost never needed for these motors. The real voltage limit is the resistive drop. I derive field weakening rigorously anyway, because the same voltage-limit machinery protects the design during battery sag and fast recentering. I also quantify when it matters.

---

## 1. Space Vector Foundation

Define the complex operator $a = e^{j2\pi/3}$. For any three-phase quantity $x_a, x_b, x_c$ (current, voltage, or flux linkage), the space vector is:

$$
\vec{x} = k\left(x_a + a\,x_b + a^2 x_c\right)
$$

**Choice of $k$.** Take a balanced set: $x_a = X\cos\phi$, $x_b = X\cos(\phi - 2\pi/3)$, $x_c = X\cos(\phi + 2\pi/3)$. Write each cosine with Euler's formula, $\cos\psi = \tfrac12(e^{j\psi}+e^{-j\psi})$:

$$
x_a + a x_b + a^2 x_c = \tfrac{X}{2}\Big[e^{j\phi}(1 + 1 + 1) + e^{-j\phi}(1 + a^2 + a^4)\Big]
$$

Since $a^4 = a$ and $1 + a + a^2 = 0$, the negative-sequence term vanishes:

$$
x_a + a x_b + a^2 x_c = \tfrac{3}{2}X e^{j\phi}
$$

The two standard scalings follow:

- **$k = 2/3$ (amplitude-invariant):** $|\vec{x}| = X$. The vector magnitude equals the phase peak.
- **$k = \sqrt{2/3}$ (power-invariant):** the transform matrix is orthonormal.

**Selected:** amplitude-invariant. Current limits, ADC scaling, and the voltage hexagon all map directly to phase-peak quantities. The cost is a $3/2$ factor in power and torque (§3.3).

---

## 2. Clarke Transform

### 2.1 Full form

Take the real and imaginary parts of $\vec{x}$ with $k = 2/3$, using $a = -\tfrac12 + j\tfrac{\sqrt3}{2}$ and $a^2 = -\tfrac12 - j\tfrac{\sqrt3}{2}$:

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

The third row (zero sequence, $x_0 = \tfrac13\sum x$) makes the matrix invertible.

### 2.2 Reduced form for a wye motor without neutral connection

Kirchhoff's current law gives $i_a + i_b + i_c = 0$. Substitute $i_c = -i_a - i_b$:

$$
i_\alpha = \tfrac23\left(i_a - \tfrac12 i_b + \tfrac12 i_a + \tfrac12 i_b\right) = i_a
$$

$$
i_\beta = \tfrac23\cdot\tfrac{\sqrt3}{2}(i_b - i_c) = \tfrac{1}{\sqrt3}(i_b - i_c) = \frac{i_a + 2 i_b}{\sqrt3}
$$

**Implementation consequence:** only two phase currents need sensing. If three shunts are fitted, use the two phases with the widest low-side on-time in the current sector. Equivalently, reconstruct the phase with the shortest low-side window from the other two. This removes the minimum-pulse sampling problem at high modulation.

### 2.3 Inverse Clarke (needed for min-max SVPWM)

$$
x_a = x_\alpha,\qquad
x_b = -\tfrac12 x_\alpha + \tfrac{\sqrt3}{2}x_\beta,\qquad
x_c = -\tfrac12 x_\alpha - \tfrac{\sqrt3}{2}x_\beta
$$

---

## 3. Park Transform and the PMSM dq Model

### 3.1 Derivation

Let the rotor flux axis sit at $\theta_e$. A vector in the rotor frame is the stationary vector rotated by $-\theta_e$:

$$
\vec{x}^{\,r} = \vec{x}^{\,s} e^{-j\theta_e}
\;\Rightarrow\;
\begin{bmatrix} x_d \\ x_q \end{bmatrix} =
\begin{bmatrix} \cos\theta_e & \sin\theta_e \\ -\sin\theta_e & \cos\theta_e \end{b