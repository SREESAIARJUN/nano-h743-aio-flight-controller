# 3-Axis Gimbal on ESP32-S3, Cycle 3: FOC Mathematical Foundations

**Scope:** This document derives the Clarke and Park transforms, the dq machine model, SVPWM duty-cycle equations, and field weakening (FW) from first principles, then maps each result to the ESP32-S3 implementation.

**Target plant:** A gimbal-class surface-mounted PMSM (SPM). This means:

- high pole count,
- high phase resistance,
- $L_d \approx L_q$.

**Key consequence:** The resistive drop on a gimbal motor is comparable to the back-EMF. For that reason, the field-weakening derivation retains $R_s$ rather than using the usual high-speed textbook approximation.

---

## 0. Conventions and Reference Frames

**Phase quantities.** For any phase quantity $x \in \{v, i, \psi\}$:

$$
\mathbf{x}_{abc} = \begin{bmatrix} x_a & x_b & x_c \end{bmatrix}^T
$$

**Magnetic axes.** Phase axes are spatially displaced by $120^\circ$ electrical:

- $\hat{a} = e^{j0}$
- $\hat{b} = e^{j2\pi/3}$
- $\hat{c} = e^{j4\pi/3}$

**Angles and speeds.**

- Electrical angle: $\theta_e = p\,\theta_m$, where $p$ is the number of pole pairs. Gimbal motors typically have 7, 11 or 14.
- Electrical speed: $\omega_e = \dot\theta_e$.

**Rotor frame.** The **d-axis** is aligned with the rotor permanent-magnet flux. The **q-axis** leads it by $90^\circ$.

**Space-vector definition.** The space vector is defined with a scaling constant $k$, which is fixed in §1:

$$
\vec{x}_s = k\left(x_a + a\,x_b + a^2 x_c\right), \qquad a = e^{j2\pi/3}
$$

---

## 1. Clarke Transform ($abc \rightarrow \alpha\beta\gamma$)

### 1.1 Derivation

Expand $a = -\tfrac12 + j\tfrac{\sqrt3}{2}$ and $a^2 = -\tfrac12 - j\tfrac{\sqrt3}{2}$:

$$
\vec{x}_s = k\left[\left(x_a - \tfrac12 x_b - \tfrac12 x_c\right) + j\tfrac{\sqrt3}{2}\left(x_b - x_c\right)\right] \equiv x_\alpha + j x_\beta
$$

The map from 3 dimensions to 2 has a null space spanned by $[1,1,1]^T$, the zero-sequence. To keep the transform invertible, a third row is appended that captures the zero-sequence component $x_\gamma$:

$$
\begin{bmatrix} x_\alpha \\ x_\beta \\ x_\gamma \end{bmatrix}
= k
\begin{bmatrix}
1 & -\tfrac12 & -\tfrac12 \\
0 & \tfrac{\sqrt3}{2} & -\tfrac{\sqrt3}{2} \\
\kappa & \kappa & \kappa
\end{bmatrix}
\begin{bmatrix} x_a \\ x_b \\ x_c \end{bmatrix}
$$

### 1.2 Choosing $k$: amplitude-invariant vs. power-invariant

Consider a balanced set:

$$
x_a = X\cos\phi, \qquad x_b = X\cos\!\left(\phi - \tfrac{2\pi}{3}\right), \qquad x_c = X\cos\!\left(\phi + \tfrac{2\pi}{3}\right)
$$

Using $\sum_{n} \cos\!\left(\phi - \tfrac{2\pi n}{3}\right)e^{j2\pi n/3} = \tfrac32 e^{j\phi}$:

$$
\vec{x}_s = k \cdot \tfrac32 X e^{j\phi}
$$

The two standard choices of $k$ are compared below.

| Choice | $k$ | Property | Power expression |
|---|---|---|---|
| **Amplitude-invariant** (used here) | $\tfrac23$ | $\lvert\vec{x}_s\rvert = X$ (phase peak) | $P = \tfrac32(v_\alpha i_\alpha + v_\beta i_\beta)$ |
| Power-invariant | $\sqrt{2/3}$ | Orthonormal matrix, $T^{-1} = T^T$ | $P = v_\alpha i_\alpha + v_\beta i_\beta$ |

**Proof of the power factor in the amplitude-invariant case.** Let $T_0$ be the bracketed matrix with $\kappa = 1/\sqrt2$. Then $T_0 T_0^T = \tfrac32 I$. Setting $T = \tfrac23 T_0$ gives:

$$
T^{-T}T^{-1} = \left(\tfrac32\right)^2 \cdot \tfrac23\, I = \tfrac32\, I
$$

Therefore:

$$
P = \mathbf{v}_{abc}^T \mathbf{i}_{abc} = \tfrac32\left(v_\alpha i_\alpha + v_\beta i_\beta\right) + 3\,v_\gamma i_\gamma
$$

With $\kappa = \tfrac12$, the zero-sequence becomes $x_\gamma = \tfrac13(x_a + x_b + x_c)$.

**Design decision.** We use the amplitude-invariant form, $k = \tfrac23$. With this choice, $|i_{dq}|$ reads directly in phase-peak amps. This keeps current limits, ADC scaling, and the voltage hexagon in physical units.

### 1.3 Reduced forms for implementation

The motor is star-connected with a floating neutral, so:

$$
i_a + i_b + i_c = 0 \;\Rightarrow\; i_\gamma = 0
$$

Substituting $i_c = -i_a - i_b$:

$$
\boxed{\,i_\alpha = i_a, \qquad i_\beta = \frac{i_a + 2 i_b}{\sqrt3}\,}
$$

This requires two multiply-accumulate operations and two shunts.

**Three-shunt variant.** With three shunts, use the redundant form

$$
i_\alpha = \tfrac23 i_a - \tfrac13(i_b + i_c), \qquad i_\beta = \tfrac{1}{\sqrt3}(i_b - i_c)
$$

and monitor $\epsilon = i_a + i_b + i_c$ as an ADC and offset health check. Set a fault threshold of roughly 5% of $I_{max}$.

### 1.4 Inverse Clarke (needed for SVPWM)

Applying $T^{-1}$ with $x_\gamma = 0$:

$$
\boxed{\,x_a = x_\alpha, \qquad x_b = -\tfrac12 x_\alpha + \tfrac{\sqrt3}{2}x_\beta, \qquad x_c = -\tfrac12 x_\alpha - \tfrac{\sqrt3}{2}x_\beta\,}
$$

---

## 2. Park Transform ($\alpha\beta \rightarrow dq$)

### 2.1 Derivation

A vector expressed in the stationary frame is projected onto a frame rotating at $\theta_e$. This is multiplication by $e^{-j\theta_e}$:

$$
x_d + j x_q = \left(x_\alpha + j x_\beta\right)e^{-j\theta_e}
$$

In matrix form, with the rotation $R(\theta)$:

$$
\boxed{\begin{bmatrix} x_d \\ x_q \end{bmatrix} =
\begin{bmatrix} \cos\theta_e & \sin\theta_e \\ -\sin\theta_e & \cos\theta_e \end{bmatrix}
\begin{bmatrix} x_\alpha \\ x_\beta \end{bmatrix}}
\qquad
\boxed{\begin{bmatrix} x_\alpha \\ x_\beta \end{bmatrix} =
\begin{bmatrix} \cos\theta_e & -\sin\theta_e \\ \sin\theta_e & \cos\theta_e \end{bmatrix}
\begin{bmatrix} x_d \\ x_q \end{bmatrix}}
$$

Because $R$ is orthonormal, Park is power-neutral. The total power factor of $\tfrac32$ therefore carries through unchanged into the dq frame.

### 2.2 Why it matters: the dq machine model

**Stator equation in the stationary frame:**

$$
\vec v_s = R_s \vec i_s + \frac{d\vec\psi_s}{dt}
$$

**Moving to the rotor frame.** Substitute $\vec x_s = \vec x_r e^{j\theta_e}$. The derivative of a rotating vector picks up a cross term:

$$
\frac{d}{dt}\left(\vec\psi_r e^{j\theta_e}\right) = \left(\dot{\vec\psi}_r + j\omega_e\vec\psi_r\right)e^{j\theta_e}
$$

Multiplying through by $e^{-j\theta_e}$ gives:

$$
\vec v_r = R_s\vec i_r + \dot{\vec\psi}_r + j\omega_e\vec\psi_r
$$

**Flux linkages:**

$$
\psi_d = L_d i_d + \lambda_{m}, \qquad \psi_q = L_q i_q
$$

Here $\lambda_m$ is the PM flux linkage, in V·s/rad (electrical), as a phase-peak value.

**Voltage equations.** Separating real and imaginary parts:

$$
\boxed{\begin{aligned}
v_d &= R_s i_d + L_d \frac{di_d}{dt} - \omega_e L_q i_q \\
v_q &= R_s i_q + L_q \frac{di_q}{dt} + \omega_e L_d i_d + \omega_e\lambda_m
\end{aligned}}
$$

**Torque by power balance.** The air-gap power is the rotational term:

$$
P_{em} = \tfrac32\,\mathrm{Re}\{j\omega_e\vec\psi_r\,\vec i_r^{\,*}\} = \tfrac32\,\omega_e\left(\psi_d i_q - \psi_q i_d\right)
$$

Since $T_e = P_{em}/\omega_m = pP_{em}/\omega_e$:

$$
\boxed{T_e = \tfrac32\,p\left[\lambda_m i_q + (L_d - L_q)\,i_d i_q\right] \;\xrightarrow{\text{SPM}}\; \tfrac32\,p\,\lambda_m i_q}
$$

**Consequences for the gimbal:**

- Torque is linear in $i_q$. Torque ripple therefore maps directly to $i_q$ ripple.
- The stabilization loop commands $i_q$, which makes the torque constant $K_t = \tfrac32 p\lambda_m$ in N·m per phase-peak amp.
- For an SPM, $i_d$ produces no torque. It is used only for field weakening and is otherwise held at 0. Section 3.4 proves that $i_d = 0$ is MTPA for an SPM.

### 2.3 Current-controller design from the model

**Decoupling.** Add feedforward terms that cancel the cross-coupling:

$$
v_d^{ff} = -\omega_e L_q i_q, \qquad v_q^{ff} = \omega_e(L_d i_d + \lambda_m)
$$

**Resulting plant.** Each axis then reduces to $G(s) = 1/(L s + R_s)$.

**PI tuning.** A PI controller $K_p + K_i/s$ with pole-zero cancellation, $K_i/K_p = R_s/L$, gives:

$$
K_p = \omega_c L, \qquad K_i = \omega_c R_s \quad\Rightarrow\quad \frac{i}{i^*} = \frac{\omega_c}{s + \omega_c}
$$

**Bandwidth choice.** Choose $\omega_c \le 2\pi f_{pwm}/20$.

- At $f_{pwm} = 20$ kHz, this gives about $2\pi \times 1$ kHz.
- This is well above the IMU attitude loop, which runs at roughly 50–150 Hz. The cascade separation is therefore at least 7×.

**Anti-windup.** Clamp the integrator using back-calculation from the saturated output of the voltage limiter (§4.6).

---

## 3. Space Vector PWM

### 3.1 Inverter switching states

Each leg $x \in \{a, b, c\}$ has a state $S_x \in \{0, 1\}$, with pole voltage $S_x V_{dc}$ referenced to the DC negative rail.

**Effect of the floating neutral.** The neutral voltage is $v_n = \tfrac{V_{dc}}{3}\sum S_x$. This common-mode term lies along $[1,1,1]$, so the Clarke transform annihilates it. Applying Clarke with $k = \tfrac23$:

$$
\vec V_k = \tfrac23 V_{d