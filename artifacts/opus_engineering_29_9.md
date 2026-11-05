# 3-Axis Gimbal on ESP32-S3: FOC Mathematical Foundations

**Cycle 29 Design Document: Motor Control Core**

---

## 0. System Context and Design Constraints

Before deriving anything, here are the physical facts that shape the math.

| Parameter | Typical gimbal BLDC (e.g., GBM2804, GM3506) | Consequence |
|---|---|---|
| Pole pairs *p* | 7–12 | Electrical angle θₑ = p·θₘ amplifies encoder error |
| Phase resistance Rₛ | 5–15 Ω (high-turn winding) | **R cannot be neglected** in voltage equations |
| Inductance L | 1–5 mH, Ld ≈ Lq (surface PM) | SPM: MTPA reduces to i_d = 0 |
| Operating speed | Mostly < 5 rad/s mech (stabilization) | Field weakening is only relevant during fast slew and return-to-home |
| Bus voltage | 12–16.8 V (3S–4S) | |

**ESP32-S3 resource mapping:**

- **MCPWM:** 2 units × 3 timers × 3 operators × 2 generators. With a 3-PWM driver (DRV8313 / TMC6300-style), each phase needs one generator, so **one unit drives two motors** and three axes fit within both units.
- **Encoders:** Use SPI absolute encoders (MT6701 SSI or AS5048A) at ≥ 10 kHz. Avoid the AS5600 over I²C: its latency is too high once multiplied by pole count.
- **Core partitioning:** Core 1 runs the FOC ISR (all 3 axes, timer-synchronized). Core 0 runs IMU fusion, attitude control and comms.
- **FPU:** Single-precision hardware float. All derivations below are implemented in `float`, not Q-format.

---

## 1. The Space Vector Foundation

### 1.1 Definition

A balanced three-phase machine has windings displaced by 120°. Define the rotation operator

$$
a = e^{j2\pi/3} = -\tfrac{1}{2} + j\tfrac{\sqrt{3}}{2}, \qquad 1 + a + a^2 = 0
$$

The **stator current space vector** is

$$
\vec{i}_s = k\left(i_a + a\,i_b + a^2 i_c\right)
$$

where *k* is a scaling constant chosen next.

### 1.2 Choosing *k*: amplitude- vs. power-invariance

Let the phase currents be balanced sinusoids with amplitude Î:

$$
i_a = \hat I\cos\theta,\quad i_b = \hat I\cos(\theta - \tfrac{2\pi}{3}),\quad i_c = \hat I\cos(\theta + \tfrac{2\pi}{3})
$$

Write each cosine via Euler's identity, $\cos x = \tfrac12(e^{jx}+e^{-jx})$:

$$
i_a + a i_b + a^2 i_c = \frac{\hat I}{2}\Big[\underbrace{(e^{j\theta} + a\,a^{-1}e^{j\theta} + a^2 a e^{j\theta})}_{3e^{j\theta}} + \underbrace{(e^{-j\theta} + a\cdot a\, e^{-j\theta} + a^2 a^{-1} e^{-j\theta})}_{e^{-j\theta}(1+a^2+a)=0}\Big]
$$

$$
\Rightarrow\; i_a + a i_b + a^2 i_c = \tfrac{3}{2}\hat I e^{j\theta}
$$

- **Amplitude-invariant:** k = 2/3 gives |i⃗ₛ| = Î. Phase peak equals vector magnitude.
- **Power-invariant:** k = √(2/3) gives P = vₐiₐ+v_bi_b+v_ci_c = Re{v⃗ₛ i⃗ₛ*}.

**Design decision:** Use **amplitude-invariant** (k = 2/3). Current limits and sensor full-scale then map directly, at the cost of a 3/2 factor in power and torque equations.

---

## 2. Clarke Transform (abc → αβ)

### 2.1 Derivation

Decompose $\vec{i}_s = i_\alpha + j i_\beta$ with k = 2/3:

$$
i_\alpha = \tfrac{2}{3}\left(i_a - \tfrac12 i_b - \tfrac12 i_c\right),\qquad
i_\beta = \tfrac{2}{3}\left(\tfrac{\sqrt3}{2} i_b - \tfrac{\sqrt3}{2} i_c\right)
$$

Adding a zero-sequence row $i_0 = \tfrac13(i_a+i_b+i_c)$ makes the transform invertible:

$$
\begin{bmatrix} i_\alpha \\ i_\beta \\ i_0 \end{bmatrix}
= \frac{2}{3}
\begin{bmatrix}
1 & -\tfrac12 & -\tfrac12 \\
0 & \tfrac{\sqrt3}{2} & -\tfrac{\sqrt3}{2} \\
\tfrac12 & \tfrac12 & \tfrac12
\end{bmatrix}
\begin{bmatrix} i_a \\ i_b \\ i_c \end{bmatrix}
$$

### 2.2 Reduced form (isolated neutral)

A star-connected motor with floating neutral forces $i_a + i_b + i_c = 0$, so $i_0 \equiv 0$. Substituting $i_c = -i_a - i_b$:

$$
i_\alpha = \tfrac23\left(i_a + \tfrac12 i_a\right) = i_a
$$

$$
i_\beta = \tfrac{2}{3}\cdot\tfrac{\sqrt3}{2}(i_b - i_c) = \tfrac{1}{\sqrt3}(i_b + i_a + i_b) = \frac{i_a + 2 i_b}{\sqrt3}
$$

Only two current sensors are therefore required. This matters on the S3, where ADC channels are limited across three axes.

### 2.3 Inverse Clarke

$$
v_a = v_\alpha,\qquad v_b = -\tfrac12 v_\alpha + \tfrac{\sqrt3}{2}v_\beta,\qquad v_c = -\tfrac12 v_\alpha - \tfrac{\sqrt3}{2}v_\beta
$$

---

## 3. Park Transform (αβ → dq)

### 3.1 Derivation

The αβ frame is stationary, so in steady state the signals are AC at frequency ωₑ. Define a frame rotating with the rotor flux at angle θₑ, with d aligned to the magnet's north pole. A vector expressed in the rotating frame is the stationary vector rotated backward:

$$
\vec{i}_{dq} = \vec{i}_{\alpha\beta}\, e^{-j\theta_e}
$$

Expand $(i_\alpha + j i_\beta)(\cos\theta_e - j\sin\theta_e)$:

$$
\boxed{\;
\begin{bmatrix} i_d \\ i_q \end{bmatrix} =
\begin{bmatrix} \cos\theta_e & \sin\theta_e \\ -\sin\theta_e & \cos\theta_e \end{bmatrix}
\begin{bmatrix} i_\alpha \\ i_\beta \end{bmatrix}\;}
$$

Inverse Park, $\vec v_{\alpha\beta} = \vec v_{dq} e^{+j\theta_e}$:

$$
v_\alpha = v_d\cos\theta_e - v_q\sin\theta_e,\qquad v_\beta = v_d\sin\theta_e + v_q\cos\theta_e
$$

**Sanity check.** Substituting $i_\alpha = \hat I\cos(\theta_e+\gamma)$ and $i_\beta = \hat I\sin(\theta_e+\gamma)$ gives $i_d = \hat I\cos\gamma$ and $i_q = \hat I\sin\gamma$. These are DC quantities, which is why PI controllers achieve zero steady-state error in dq.

### 3.2 Machine model in the dq frame (the motivation for Park)

The stator voltage equation in the stationary frame is

$$
\vec v_{\alpha\beta} = R_s \vec i_{\alpha\beta} + \frac{d\vec\psi_{\alpha\beta}}{dt}
$$

Substitute $\vec x_{\alpha\beta} = \vec x_{dq}e^{j\theta_e}$ and apply the product rule:

$$
\frac{d}{dt}\left(\vec\psi_{dq}e^{j\theta_e}\right) = \left(\frac{d\vec\psi_{dq}}{dt} + j\omega_e\vec\psi_{dq}\right)e^{j\theta_e}
$$

Cancelling $e^{j\theta_e}$:

$$
\vec v_{dq} = R_s\vec i_{dq} + \frac{d\vec\psi_{dq}}{dt} + j\omega_e\vec\psi_{dq}
$$

The PMSM flux linkages are $\psi_d = L_d i_d + \psi_m$ and $\psi_q = L_q i_q$. Separating real and imaginary parts gives:

$$
\boxed{
\begin{aligned}
v_d &= R_s i_d + L_d\frac{di_d}{dt} - \omega_e L_q i_q \\
v_q &= R_s i_q + L_q\frac{di_q}{dt} + \omega_e L_d i_d + \omega_e\psi_m
\end{aligned}}
$$

The term $j\omega_e\vec\psi$ produces the **cross-coupling** terms ($\mp\omega_e L i$) and the **back-EMF** ($\omega_e\psi_m$).

### 3.3 Torque equation

Electrical input power with amplitude-invariant scaling is $P = \tfrac32(v_d i_d + v_q i_q)$. The speed-dependent part that converts to mechanical power is $\tfrac32\omega_e(\psi_d i_q - \psi_q i_d)$. Using $\omega_e = p\,\omega_m$:

$$
\boxed{T_e = \tfrac32 p\left[\psi_m i_q + (L_d - L_q) i_d i_q\right]}
$$

For a gimbal SPM motor with $L_d = L_q$, this reduces to $T_e = \tfrac32 p\psi_m\, i_q = K_t i_q$. Torque is linear in $i_q$, and $i_d$ produces no torque. **MTPA is therefore $i_d^* = 0$** below base speed.

### 3.4 Current controller design (pole-zero cancellation)

Treat the cross-coupling as a disturbance and cancel it with feedforward. Each axis then becomes a first-order plant:

$$
G(s) = \frac{1}{L s + R}
$$

Choose a PI controller $C(s) = K_p + K_i/s = K_p\,\frac{s + K_i/K_p}{s}$ and set $K_i/K_p = R/L