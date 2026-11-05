# 3-Axis Gimbal Motor Control on ESP32-S3: FOC Mathematical Foundations (Design Revision 20)

**Scope:** This document derives the math behind each stage of the FOC pipeline. For every result it also states the numerical form used on the ESP32-S3 and the design decision that follows.

**Pipeline covered:**
- Clarke transform
- Park transform and the PMSM dq model
- Torque equation
- Current-loop design
- SVPWM duty-cycle equations
- MTPA and field weakening

**Before deploying:** Every numeric value in this document (motor parameters, timings, peripheral counts) is a planning figure. Verify it against your actual motor datasheet, driver datasheet, the ESP-IDF version you build with, and bench measurements.

---

## 0. System Context and Constraints

| Item | Choice / Constraint | Consequence for the math |
|---|---|---|
| MCU | ESP32-S3, dual Xtensa LX7 at 240 MHz, single-precision FPU | Use `float` throughout. Avoid `double`, which is emulated in software. |
| Motors | 3× gimbal BLDC (2804/4108 class): R ≈ 5–15 Ω, L ≈ 1–4 mH, p ≈ 7–11 pole pairs | R is large, so ωL is comparable to R. Do not drop R in the steady-state analysis (§6). |
| Drivers | 3-PWM half-bridge drivers with integrated dead time (DRV8313-class) | 9 PWM outputs are needed. Both MCPWM units are used, generators A and B per operator. |
| Feedback | 14-bit magnetic encoders (MT6701/AS5048A over SSI/SPI); low-side or inline current sense | θ_e = p·θ_m. Quantization of θ_e is 2π·p/16384. |
| PWM | 20–25 kHz, center-aligned (up-down count) | Center-aligned sampling gives ripple-free average current (§5.6). |
| Loop partitioning | Core 1: current/FOC ISR in IRAM. Core 0: IMU fusion, attitude, comms | Gives a deterministic FOC timing budget (§7). |

**Peripheral note:**
- Each S3 MCPWM unit has 3 operators. Each operator has 2 generators and binds to one timer.
- Two units therefore give 12 independent edges, enough for 9 phase signals.
- Synchronize all timers through the sync input so that every motor's PWM center, and therefore its ADC trigger, is phase-coherent.
- Confirm the operator/timer binding rules against your ESP-IDF MCPWM driver version.

---

## 1. The Space Vector and the Clarke Transform

### 1.1 Definition

Let the three stator windings be displaced spatially by 2π/3. Define the rotation operator

$$
a = e^{j2\pi/3} = -\tfrac12 + j\tfrac{\sqrt3}{2},\qquad a^2 = e^{-j2\pi/3} = -\tfrac12 - j\tfrac{\sqrt3}{2},\qquad 1+a+a^2=0 .
$$

For any phase quantity x ∈ {v, i, ψ}, the **space vector** is

$$
\vec{x}_s = K\left(x_a + a\,x_b + a^2 x_c\right) = x_\alpha + j x_\beta .
$$

### 1.2 Component Expansion

Take real and imaginary parts:

$$
x_\alpha = K\left(x_a - \tfrac12 x_b - \tfrac12 x_c\right),\qquad
x_\beta = K\,\tfrac{\sqrt3}{2}\left(x_b - x_c\right).
$$

Add a zero-sequence component $x_0 = K_0 (x_a+x_b+x_c)$ to make the map invertible:

$$
\begin{bmatrix}x_\alpha\\x_\beta\\x_0\end{bmatrix}
= K
\begin{bmatrix}
1 & -\tfrac12 & -\tfrac12\\
0 & \tfrac{\sqrt3}{2} & -\tfrac{\sqrt3}{2}\\
\tfrac{K_0}{K} & \tfrac{K_0}{K} & \tfrac{K_0}{K}
\end{bmatrix}
\begin{bmatrix}x_a\\x_b\\x_c\end{bmatrix}.
$$

### 1.3 Choice of K: Amplitude- vs Power-Invariant

Apply a balanced set $x_a = X\cos\phi,\; x_b = X\cos(\phi - 2\pi/3),\; x_c = X\cos(\phi+2\pi/3)$. Write each cosine as $\tfrac12(e^{j\cdot}+e^{-j\cdot})$:

$$
x_a + a x_b + a^2 x_c = \tfrac{X}{2}\Big[e^{j\phi}(1+1+1) + e^{-j\phi}(1 + a\cdot a + a^2\cdot a^2)\Big]
= \tfrac{3X}{2}e^{j\phi},
$$

The second bracket vanishes because $1 + a^2 + a^4 = 1 + a^2 + a = 0$.

- **Amplitude-invariant, K = 2/3:** $|\vec x_s| = X$. Peak phase quantities map 1:1. **Adopted here**, so current limits, encoder-referenced ψ_m and ADC scaling stay intuitive.
- **Power-invariant, K = √(2/3):** the matrix becomes orthonormal and $p = \vec v\cdot\vec i$ directly.

With K = 2/3, instantaneous power is

$$
p(t) = v_a i_a + v_b i_b + v_c i_c = \tfrac32\left(v_\alpha i_\alpha + v_\beta i_\beta\right) + 3 v_0 i_0 .
$$

**Proof sketch:** the inverse of the amplitude-invariant matrix is $\tfrac32 \times$ its transpose on the αβ rows. So $\mathbf{v}^T\mathbf{i} = \mathbf{v}_{\alpha\beta}^T (C^{-T}C^{-1})\mathbf{i}_{\alpha\beta}$, and $C^{-T}C^{-1} = \tfrac32 I_2$ on the αβ subspace.

The 3/2 factor reappears in the torque equation (§2.4).

### 1.4 Reduced Clarke for Isolated Neutral

A wye winding with a floating neutral forces $i_a + i_b + i_c = 0$. Substitute $i_c = -i_a - i_b$:

$$
i_\alpha = \tfrac23\left(i_a + \tfrac12 i_a + \tfrac12 i_b - \tfrac12 i_b\right) = i_a,
\qquad
i_\beta = \tfrac23\cdot\tfrac{\sqrt3}{2}(i_b + i_a + i_b) = \frac{i_a + 2 i_b}{\sqrt3}.
$$

$$
\boxed{\,i_\alpha = i_a,\qquad i_\beta = \tfrac{1}{\sqrt3}\,(i_a + 2i_b)\,}
$$

**Implementation:** only two current channels are needed. If all three are sampled, use the sum $i_a+i_b+i_c$ as a live fault/offset diagnostic. It should stay within a few ADC LSB of zero.

### 1.5 Inverse Clarke (Zero Sequence Set to 0)

$$
x_a = x_\alpha,\qquad
x_b = -\tfrac12 