# 3-Axis Gimbal Controller, ESP32-S3 (Cycle 26)
## Technical Design Note: FOC Transforms, SVPWM, and Field Weakening

---

## 0. Scope and Notation

| Symbol | Meaning |
|---|---|
| $i_a, i_b, i_c$ | Phase currents (A) |
| $v_a, v_b, v_c$ | Phase voltages w.r.t. motor neutral (V) |
| $\theta_e = p\,\theta_m$ | Electrical angle, with $p$ pole pairs |
| $\omega_e = d\theta_e/dt$ | Electrical angular velocity (rad/s) |
| $R_s, L_d, L_q$ | Phase resistance and d/q inductances |
| $\psi_m$ | Permanent-magnet flux linkage, peak per phase (Wb) |
| $V_{dc}$ | DC bus voltage |
| $T_s$ | PWM period |
| $a = e^{j2\pi/3}$ | 120° rotation operator |

Gimbal motors are surface-mounted PMSMs (SPMSM) with high resistance and many poles. They are typically 12N14P, so $p=7$. For these motors $L_d \approx L_q = L$. I derive the general salient case first and then specialize.

---

## 1. Clarke Transform (abc → αβ)

### 1.1 Space-Vector Definition

Place three unit vectors along the magnetic axes of the windings at $0$, $2\pi/3$, and $4\pi/3$. The stator current space vector is the scaled sum of the phase contributions:

$$
\vec{i}_s = K\left(i_a + a\,i_b + a^2 i_c\right), \qquad a = -\tfrac12 + j\tfrac{\sqrt3}{2},\quad a^2 = -\tfrac12 - j\tfrac{\sqrt3}{2}
$$

Separating real and imaginary parts:

$$
i_\alpha = K\left(i_a - \tfrac12 i_b - \tfrac12 i_c\right), \qquad
i_\beta = K\,\tfrac{\sqrt3}{2}\left(i_b - i_c\right)
$$

### 1.2 Choosing $K$

Apply a balanced sinusoidal set: $i_a = I\cos\theta$, $i_b = I\cos(\theta - 2\pi/3)$, $i_c = I\cos(\theta + 2\pi/3)$. Writing $\cos x = \tfrac12(e^{jx}+e^{-jx})$:

$$
i_a + a i_b + a^2 i_c = \frac{I}{2}\Big[e^{j\theta}(1 + a\cdot a^{-1} + a^2\cdot a^{-2}) + e^{-j\theta}(1 + a^2 + a^4)\Big]
$$

The first bracket equals 3. Because $a^4 = a$, the second bracket is $1 + a + a^2 = 0$. Therefore

$$
i_a + a i_b + a^2 i_c = \tfrac32 I\, e^{j\theta}
$$

This gives two standard choices:

- **Amplitude-invariant**, $K = 2/3$. Then $|\vec i_s| = I$, so the vector magnitude equals the phase peak. This is the form used in this design because it maps directly to datasheet peak currents. The cost is that power acquires a factor of 3/2:
  $$P = \tfrac32\left(v_\alpha i_\alpha + v_\beta i_\beta\right)$$
- **Power-invariant**, $K = \sqrt{2/3}$. The transform is orthonormal, so $P = v_\alpha i_\alpha + v_\beta i_\beta$.

### 1.3 Full Matrix with Zero Sequence

$$
\begin{bmatrix} i_\alpha \\ i_\beta \\ i_0 \end{bmatrix}
= \frac23
\begin{bmatrix}
1 & -\tfrac12 & -\tfrac12 \\
0 & \tfrac{\sqrt3}{2} & -\tfrac{\sqrt3}{2} \\
\tfrac12 & \tfrac12 & \tfrac12
\end{bmatrix}
\begin{bmatrix} i_a \\ i_b \\ i_c \end{bmatrix}
$$

For a wye-connected motor with an isolated neutral, KCL gives $i_a + i_b + i_c = 0$, so $i_0 \equiv 0$. Substituting $i_c = -i_a - i_b$ yields the **two-sensor form** used in firmware:

$$
\boxed{\,i_\alpha = i_a, \qquad i_\beta = \frac{i_a + 2 i_b}{\sqrt3}\,}
$$

Proof for $i_\beta$: $\tfrac23\cdot\tfrac{\sqrt3}{2}(i_b - i_c) = \tfrac{1}{\sqrt3}(i_b + i_a + i_b)$.

### 1.4 Inverse Clarke

$$
v_a = v_\alpha, \qquad
v_b = -\tfrac12 v_\alpha + \tfrac{\sqrt3}{2} v_\beta, \qquad
v_c = -\tfrac12 v_\alpha - \tfrac{\sqrt3}{2} v_\beta
$$

---

## 2. Park Transform (αβ → dq)

### 2.1 Derivation

Define a frame rotating synchronously with the rotor flux, with the d-axis aligned to the magnet north pole at angle $\theta_e$. A vector expressed in the rotating frame is the stationary vector rotated by $-\theta_e$:

$$
\vec i_{dq} = \vec i_{\alpha\beta}\, e^{-j\theta_e}
$$

Expanding $(i_\alpha + j i_\beta)(\cos\theta_e - j\sin\theta_e)$:

$$
\boxed{
\begin{aligned}
i_d &= \;\;\, i_\alpha\cos\theta_e + i_\beta\sin\theta_e \\
i_q &= -i_\alpha\sin\theta_e + i_\beta\cos\theta_e
\end{aligned}}
$$

The inverse is $\vec v_{\alpha\beta} = \vec v_{dq}\,e^{+j\theta_e}$:

$$
\boxed{
\begin{aligned}
v_\alpha &= v_d\cos\theta_e - v_q\sin\theta_e \\
v_\beta &= v_d\sin\theta_e + v_q\cos\theta_e
\end{aligned}}
$$

The rotation matrix is orthonormal, so Park itself preserves magnitude and power. Any 3/2 factor comes only from the Clarke scaling.

**Check.** With $i_\alpha = I\cos\theta$, $i_\beta = I\sin\theta$, and $\theta_e = \theta - \pi/2$, the result is $i_d = 0$ and $i_q = I$. A steady sinusoid becomes DC, which is the whole point: PI controllers then track DC references with zero steady-state error.

### 2.2 Machine Model in dq (Derivation of Coupling Terms)

Start from the stationary-frame stator equation:

$$
\vec v_{\alpha\beta} = R_s \vec i_{\alpha\beta} + \frac{d\vec\psi_{\alpha\beta}}{dt}
$$

Substitute $\vec\psi_{\alpha\beta} = \vec\psi_{dq}e^{j\theta_e}$ and apply the product rule:

$$
\frac{d\vec\psi_{\alpha\beta}}{dt} = \left(\frac{d\vec\psi_{dq}}{dt} + j\omega_e\vec\psi_{dq}\right)e^{j\theta_e}
$$

Multiply through by $e^{-j\theta_e}$:

$$
\vec v_{dq} = R_s\vec i_{dq} + \frac{d\vec\psi_{dq}}{dt} + j\omega_e\vec\psi_{dq}
$$

Use the flux linkages $\psi_d = L_d i_d + \psi_m$ and $\psi_q = L_q i_q$. Since $j(\psi_d + j\psi_q) = -\psi_q + j\psi_d$:

$$
\boxed{
\begin{aligned}
v_d &= R