# 3-Axis Gimbal on ESP32-S3: FOC Mathematical Foundations
**Design Cycle 30 · Motor Control Core Specification**

---

## 0. System Context and Modeling Assumptions

Before deriving anything, we fix the model so every equation has a defined scope.

| Item | Assumption / Choice | Consequence |
|---|---|---|
| Motor | 3-phase PMSM (gimbal BLDC, e.g. 2804/4108 class), Y-connected, isolated neutral | $i_a+i_b+i_c=0$, so no zero-sequence current |
| Rotor | Surface-mounted magnets (SPMSM), $L_d \approx L_q = L_s$ | Reluctance torque ≈ 0; MTPA reduces to $i_d=0$ |
| Back-EMF | Sinusoidal (gimbal motors are wound for this) | Park transform yields DC quantities in steady state |
| Magnetics | Linear (no saturation), iron loss neglected | Constant $L_s$ and $\lambda_m$ |
| Drivers | 3-PWM half-bridge ICs with internal dead-time (DRV8313 / TMC6300 class) | 9 PWM channels total, not 18 |
| Position | 14-bit magnetic encoder per axis (MT6701 / AS5048A over SPI) | $\theta_e = p\,\theta_m - \theta_{offset}$ |
| MCU | ESP32-S3 @ 240 MHz, single-precision FPU, 2 × MCPWM units | Float math is acceptable; trig is replaced with LUT + interpolation |

**Notation.** $p$ is pole pairs, $\theta_e$ the electrical angle, $\omega = d\theta_e/dt$ the electrical angular velocity, $\lambda_m$ the PM flux linkage (Wb), $R_s$ the phase resistance, $V_{dc}$ the bus voltage, and $T_s$ the PWM period.

---

## 1. Clarke Transform (abc → αβ)

### 1.1 Geometric derivation

Each phase winding produces an MMF along its own magnetic axis. The three axes are spaced at $0$, $2\pi/3$ and $4\pi/3$. Define the **space vector** as the complex sum of the phase quantities placed on their axes:

$$
\vec{x} = k\left(x_a + x_b\,e^{j2\pi/3} + x_c\,e^{j4\pi/3}\right) = x_\alpha + j\,x_\beta
$$

Expanding with $e^{j2\pi/3} = -\tfrac12 + j\tfrac{\sqrt3}{2}$ and $e^{j4\pi/3} = -\tfrac12 - j\tfrac{\sqrt3}{2}$:

$$
x_\alpha = k\left(x_a - \tfrac12 x_b - \tfrac12 x_c\right), \qquad
x_\beta = k\,\tfrac{\sqrt3}{2}\left(x_b - x_c\right)
$$

Adding the zero-sequence component $x_0 = k_0(x_a+x_b+x_c)$ makes the map invertible:

$$
\begin{bmatrix} x_\alpha \\ x_\beta \\ x_0 \end{bmatrix}
= k\begin{bmatrix}
1 & -\tfrac12 & -\tfrac12 \\
0 & \tfrac{\sqrt3}{2} & -\tfrac{\sqrt3}{2} \\
\tfrac{k_0}{k} & \tfrac{k_0}{k} & \tfrac{k_0}{k}
\end{bmatrix}
\begin{bmatrix} x_a \\ x_b \\ x_c \end{bmatrix}
$$

### 1.2 Choosing the scale factor $k$

**Amplitude-invariant form ($k = 2/3$).** Take balanced phases $x_a = X\cos\phi$, $x_b = X\cos(\phi - 2\pi/3)$, $x_c = X\cos(\phi+2\pi/3)$. Using the identity $\sum_{n=0}^{2}\cos(\phi - 2\pi n/3)\,e^{j2\pi n/3} = \tfrac32 e^{j\phi}$:

$$
\vec{x} = k\cdot\tfrac32 X e^{j\phi} \quad\Rightarrow\quad |\vec x| = X \text{ iff } k = \tfrac23
$$

The peak phase current therefore equals $|\vec i|$, which keeps current limits intuitive.

**Power-invariant form ($k=\sqrt{2/3}$, $k_0 = 1/\sqrt3$).** Here the matrix is orthonormal, so $T^{-1}=T^{T}$ and $p = v_\alpha i_\alpha + v_\beta i_\beta + v_0 i_0$ with no correction factor.

**Design decision.** Use **amplitude-invariant** scaling. The price is that power and torque carry a factor of $3/2$:

$$
P = \tfrac32\left(v_\alpha i_\alpha + v_\beta i_\beta\right) + 3v_0 i_0
$$

### 1.3 Reduced form for the isolated neutral

With $i_c = -i_a - i_b$:

$$
\boxed{\;i_\alpha = i_a,\qquad i_\beta = \frac{i_a + 2i_b}{\sqrt3}\;}
$$

*Proof:* $\tfrac23\left(i_a - \tfrac12 i_b - \tfrac12(-i_a-i_b)\right) = \tfrac23\cdot\tfrac32 i_a = i_a$, and $\tfrac23\cdot\tfrac{\sqrt3}{2}\left(i_b + i_a + i_b\right) = \tfrac{i_a+2i_b}{\sqrt3}$. ∎

This form needs only two current sensors per motor, which saves ESP32-S3 ADC channels: 6 instead of 9.

### 1.4 Inverse Clarke

$$
x_a = x_\alpha + x_0,\quad
x_b = -\tfrac12 x_\alpha + \tfrac{\sqrt3}{2}x_\beta + x_0,\quad
x_c = -\tfrac12 x_\alpha - \tfrac{\sqrt3}{2}x_\beta + x_0
$$

Note that $x_0$ is a free parameter on the voltage side. **SVPWM is precisely a particular choice of $v_0$** (Section 3.5).

---

## 2. Park Transform (αβ → dq)

### 2.1 Definition

Rotate the stationary frame by $-\theta_e$ so that the d-axis aligns with the rotor magnet flux:

$$
\vec{x}_{dq} = \vec{x}_{\alpha\beta}\,e^{-j\theta_e}
\quad\Longleftrightarrow\quad
\begin{bmatrix} x_d\\ x_q\end{bmatrix} =
\underbrace{\begin{bmatrix}\cos\theta_e & \sin\theta_e\\ -\sin\theta_e & \cos\theta_e\end{bmatrix}}_{R(-\theta_e)}
\begin{bmatrix}x_\alpha\\ x_\beta\end{bmatrix}
$$

The inverse is $R(\theta_e) = R(-\theta_e)^T$:

$$
x_\alpha = x_d\cos\theta_e - x_q\sin\theta_e,\qquad x_\beta = x_d\sin\theta_e + x_q\cos\theta_e
$$

### 2.2 Stator voltage equation in dq: where cross-coupling comes from

In the stationary frame (complex notation):

$$
\vec v_{s} = R_s\,\vec i_{s} + \frac{d\vec\psi_s}{dt},\qquad
\vec\psi_s = L_s\vec i_s + \lambda_m e^{j\theta_e}
$$

Substitute $\vec x_s = \vec x_{dq}e^{j\theta_e}$ and apply the product rule:

$$
\frac{d}{dt}\left(\vec\psi_{dq}e^{j\theta_e}\right) = \left(\frac{d\vec\psi_{dq}}{dt} + j\omega\,\vec\psi_{dq}\right)e^{j\theta_e}
$$

Cancel $e^{j\theta_e}$:

$$
\vec v_{dq} = R_s\vec i_{dq} + \frac{d\vec\psi_{dq}}{dt} + j\omega\vec\psi_{dq},\qquad \vec\psi_{dq} = L_d i_d + j L_q i_q + \lambda_m
$$

Separating real and imaginary parts gives the general salient model:

$$
\boxed{
\begin{aligned}
v_d &= R_s i_d + L_d\frac{di_d}{dt} - \omega L_q i_q\\
v_q &= R_s i_q + L_q\frac{di_q}{dt} + \omega L_d i_d + \omega\lambda_m
\end{aligned}}
$$

The term $j\omega\vec\psi$ is the "speed voltage" created by the rotating frame. It produces:
- the **cross-coupling** terms $\mp\omega L i$, and
- the **back-EMF** $\omega\lambda_m$, which appears only on the q-axis.

### 2.3 Torque equation

Power balance (amplitude-invariant, so the $3/2$ factor applies) gives electromagnetic power $P_e = \tfrac32\omega(\psi_d i_q - \psi_q i_d)$. Dividing by mechanical speed $\omega/p$:

$$
\boxed{T_e = \tfrac32 p\left[\lambda_m i_q + (L_d - L_q)\,i_d i_q\right]}
\;\xrightarrow{\text{SPMSM}}\;
T_e = \underbrace{\tfrac32 p\lambda_m}_{K_t}\, i_q
$$

For the SPMSM, $i_d$ produces no torque. This is the theoretical basis for both $i_d^* = 0$ control below base speed and field weakening above it.

### 2.4 Decoupled current controller

Because the plant contains cross-coupling and back-EMF, the PI outputs are augmented with feed-forward terms:

$$
v_d^* = \text{PI}_d(i_d^*-i_d) - \hat\omega\hat L_q i_q,\qquad
v_q^* = \text{PI}_q(i_q^*-i_q) + \hat\omega\hat L_d i_d + \hat\omega\hat\lambda_m
$$

With perfect estimates, each axis reduces to $\frac{1}{L s + R_s}$. Pole-zero cancellation then gives the gains:

$$
K_p = \omega_{bw}L_s,\qquad K_i = \omega_{bw}R_s
$$

This yields the closed loop $\frac{\omega_{bw}}{s+\omega_{bw}}$. Choose $\omega_{bw} \le 2\pi f_{loop}/10$.

**Gimbal-specific note.** Gimbal motors have high $R_s$ (5–15 Ω) and electrical time constants $\tau = L/R$ of roughly 0.1–0.5 ms. Many designs run **voltage-mode F