# 3-Axis Gimbal Motor Control on ESP32-S3: Design Document, Cycle 9

## FOC Transform Mathematics, SVPWM, and Field-Weakening

---

## 0. Scope, Conventions, and System Context

| Item | Choice | Rationale |
|---|---|---|
| Motor class | Surface PMSM (gimbal BLDC, hollow shaft, 7–11 pole pairs) | $L_d \approx L_q$ and high $R_s$ (2–15 Ω) |
| Transform scaling | **Amplitude-invariant** ($K = 2/3$) | dq magnitudes equal physical phase peaks, so current limits map directly |
| Angle reference | $\theta_e = p\,\theta_m - \theta_{\text{offset}}$, d-axis aligned with rotor flux | Standard rotor-flux orientation |
| Phase sum | $i_a + i_b + i_c = 0$ (floating star) | Two-shunt sensing is sufficient |
| Bus | 3S–4S Li-ion, $V_{dc} = 9.0$ to $16.8$ V | Typical gimbal supply |
| PWM | Center-aligned (up/down), 20–25 kHz | Inaudible. Gives a symmetric sampling instant. |

**Notation.** Space vectors are written as complex numbers. The rotation operator is $a = e^{j2\pi/3}$. $\omega_e = d\theta_e/dt$. $\lambda_{pm}$ is the peak PM flux linkage per phase.

---

## 1. Clarke Transform (abc → αβ)

### 1.1 Space-vector definition

The three phase windings are spatially displaced by $0$, $2\pi/3$, and $4\pi/3$. Any set of phase quantities $x_a, x_b, x_c$ defines a single complex space vector:

$$
\vec{x}_s = K\left(x_a + a\,x_b + a^2 x_c\right), \qquad a = e^{j2\pi/3}
$$

Expand the rotation operators:

$$
a = -\tfrac12 + j\tfrac{\sqrt3}{2}, \qquad a^2 = -\tfrac12 - j\tfrac{\sqrt3}{2}
$$

Separate real and imaginary parts:

$$
x_\alpha = K\left(x_a - \tfrac12 x_b - \tfrac12 x_c\right), \qquad
x_\beta = K\cdot\tfrac{\sqrt3}{2}\left(x_b - x_c\right)
$$

### 1.2 Determining $K$

Consider a balanced set:

$$
x_a = X\cos\phi, \quad x_b = X\cos(\phi - 2\pi/3), \quad x_c = X\cos(\phi + 2\pi/3)
$$

Use $\cos u = \tfrac12(e^{ju} + e^{-ju})$:

$$
x_a + a x_b + a^2 x_c = \tfrac{X}{2}\Big[e^{j\phi}(1 + a\cdot a^{-1} + a^2\cdot a^{-2}) + e^{-j\phi}(1 + a\cdot a + a^2\cdot a^2)\Big]
$$

The first bracket equals 3. The second equals $1 + a^2 + a^4 = 1 + a^2 + a = 0$. Therefore:

$$
\vec{x}_s = K\cdot\tfrac{3}{2}X e^{j\phi}
$$

This leaves two choices for $K$:

- **Amplitude-invariant:** $K = 2/3$ gives $|\vec x_s| = X$. This is the choice adopted here.
- **Power-invariant:** $K = \sqrt{2/3}$ makes the transform orthonormal, so $p = \vec v\cdot\vec i$ holds with no scale factor.

The cost of amplitude invariance is a factor of 3/2 in the power expression. Section 1.4 derives it.

### 1.3 Full matrix with zero-sequence

$$
\begin{bmatrix} x_\alpha \\ x_\beta \\ x_0 \end{bmatrix}
= \frac23
\begin{bmatrix}
1 & -\tfrac12 & -\tfrac12\\
0 & \tfrac{\sqrt3}{2} & -\tfrac{\sqrt3}{2}\\
\tfrac12 & \tfrac12 & \tfrac12
\end{bmatrix}
\begin{bmatrix} x_a \\ x_b \\ x_c\end{bmatrix}
$$

Its inverse:

$$
\begin{bmatrix} x_a \\ x_b \\ x_c\end{bmatrix}
=
\begin{bmatrix}
1 & 0 & 1\\
-\tfrac12 & \tfrac{\sqrt3}{2} & 1\\
-\tfrac12 & -\tfrac{\sqrt3}{2} & 1
\end{bmatrix}
\begin{bmatrix} x_\alpha \\ x_\beta \\ x_0 \end{bmatrix}
$$

### 1.4 Two-shunt reduction

With a floating star, $i_c = -(i_a + i_b)$ and $i_0 = 0$. Substitute:

$$
i_\alpha = \tfrac23\left(i_a + \tfrac12 i_a + \tfrac12 i_a + \tfrac12 i_b - \tfrac12 i_b\right) = i_a
$$

$$
i_\beta = \tfrac23\cdot\tfrac{\sqrt3}{2}\left(i_b + i_a + i_b\right) = \frac{i_a + 2 i_b}{\sqrt3}
$$

**Implementation form (2 multiplies, 1 add):**

$$
\boxed{\,i_\alpha = i_a, \qquad i_\beta = \tfrac{1}{\sqrt3}\,i_a + \tfrac{2}{\sqrt3}\,i_b\,}
$$

**Power scaling.** Let $T$ be the 2×3 amplitude-invariant matrix. Then $T\,T^{\mathsf T} = \tfrac23 I$. With zero-sequence absent, the phase vector is recovered as $\mathbf{x}_{abc} = \tfrac32 T^{\mathsf T}\mathbf{x}_{\alpha\beta}$. Therefore:

$$
p = \mathbf v_{abc}^{\mathsf T}\mathbf i_{abc} = \tfrac94\,\mathbf v_{\alpha\beta}^{\mathsf T}(T T^{\mathsf T})\,\mathbf i_{\alpha\beta} = \tfrac32\left(v_\alpha i_\alpha + v_\beta i_\beta\right)
$$

---

## 2. Park Transform (αβ → dq)

### 2.1 Derivation

Define a frame rotating at $\theta_e$. A vector expressed in the stationary frame is re-expressed in the rotating frame by multiplying by $e^{-j\theta_e}$:

$$
x_d + j x_q = (x_\alpha + j x_\beta)\,e^{-j\theta_e}
$$

Expanding gives:

$$
\boxed{
\begin{bmatrix}x_d\\x_q\end{bmatrix}
=
\begin{bmatrix}\cos\theta_e & \sin\theta_e\\ -\sin\theta_e & \cos\theta_e\end{bmatrix}
\begin{bmatrix}x_\alpha\\x_\beta\end{bmatrix}}
$$

$$
\boxed{
\begin{bmatrix}x_\alpha\\x_\beta\end{bmatrix}
=
\begin{bmatrix}\cos\theta_e & -\sin\theta_e\\ \sin\theta_e & \cos\theta_e\end{bmatrix}
\begin{bmatrix}x_d\\x_q\end{bmatrix}}
$$

The Park matrix is orthogonal, so its inverse is its transpose. The 3/2 power factor therefore carries through unchanged:

$$
p = \tfrac32\left(v_d i_d + v_q i_q\right)
$$

**Why the quantities become DC.** A balanced current $\vec i_s = I e^{j(\omega_e t + \gamma)}$ with $\theta_e = \omega_e t$ transforms to $i_d + j i_q = I e^{j\gamma}$, which is constant. This lets ordinary PI controllers track sinusoidal currents with zero steady-state error.

### 2.2 Machine model in dq

Stator voltage equation in the stationary frame:

$$
\vec v_s = R_s\vec i_s + \frac{d\vec\psi_s}{dt}
$$

Substitute $\vec\psi_s = \vec\psi_{dq}\,e^{j\theta_e}$:

$$
\frac{d\vec\psi_s}{dt} = \left(\dot{\vec\psi}_{dq} + j\omega_e\vec\psi_{dq}\right)e^{j\theta_e}
$$

Multiply the whole equation by $e^{-j\theta_e}$. The flux linkages are $\psi_d = L_d i_d + \lambda_{pm}$ and $\psi_q = L_q i_q$. Separating real and imaginary parts:

$$
\boxed{
\begin{aligned}
v_d &= R_s i_d + L_d\frac{di_d}{dt} - \omega_e L_q i_q\\
v_q &= R_s i_q + L_q\frac{di_q}{dt} + \omega_e L_d i_d + \omega_e\lambda_{pm}
\end{aligned}}
$$

### 2.3 Torque

Electrical power crossing the air gap equals the speed-dependent terms multiplied by current, scaled by $\tfrac32$:

$$
P_{em} = \tfrac32\,\omega_e\left(\psi_d i_q - \psi_q i_d\right)
$$

Mechanical power is $T_e\,\omega_m$ with $\omega_m = \omega_e/p$. Therefore:

$$
\boxed{T_e = \tfrac32\,p\left[\lambda_{pm} i_q + (L_d - L_q)\,i_d i_q\right]}
$$

For a gimbal SPM, $L_d \approx L_q$, so $T_e = \tfrac32 p\lambda_{pm} i_q$. Torque is linear in $i_q$, and $i_d$ produces no torque. This fixes the MTPA (maximum torque per ampere) policy as $i_d^* = 0$ below base speed.

### 2.4 Decoupling feed-forward

The cross-coupling terms are cancelled by feed-forward