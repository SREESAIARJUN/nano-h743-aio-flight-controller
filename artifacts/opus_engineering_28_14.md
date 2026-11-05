# 3-Axis Gimbal on ESP32-S3: FOC Mathematical Foundation
**Clarke/Park Transforms, SVPWM, and Field Weakening (Cycle 28)**

---

## 0. System Context and Design Assumptions

These constraints shape the math, so they come first.

| Parameter | Value / Assumption |
|---|---|
| MCU | ESP32-S3, dual-core LX7 @ 240 MHz, single-precision FPU |
| PWM | 2× MCPWM units × 3 operators × 2 generators = 12 outputs |
| Drivers | 3-PWM + EN topology (e.g., DRV8313 class), 3 motors × 3 PWM = 9 outputs |
| Motor (illustrative) | Gimbal SPM: p = 7, R = 10 Ω, L = 2.5 mH, ψ_m = 6 mWb (peak, per phase) |
| V_dc | 12 V (3S LiPo, nominal) |
| f_PWM | 25 kHz, center-aligned (up-down count) |

**Note on the topology:** complementary-output drivers would need 6 PWM lines per motor, 18 in total. That exceeds the 12 available outputs, so 3-PWM drivers with integrated dead-time are the correct choice. The motor parameters above are representative, not taken from a specific datasheet. Substitute measured values. Section 4.6 shows these numbers drive a critical conclusion.

---

## 1. Clarke Transform

### 1.1 Space-Vector Definition

Let a = e^{j2π/3}. For any three-phase quantity (x_a, x_b, x_c), define the space vector:

$$
\vec{x}_s = K\left(x_a + a\,x_b + a^2 x_c\right)
$$

**Derivation of K (amplitude-invariant form).** Take a balanced set x_a = X cos θ, x_b = X cos(θ − 2π/3), x_c = X cos(θ + 2π/3). Using cos φ = ½(e^{jφ} + e^{−jφ}):

- x_a = (X/2)(e^{jθ} + e^{−jθ})
- a·x_b = (X/2)(e^{jθ} + a²e^{−jθ})
- a²·x_c = (X/2)(e^{jθ} + a e^{−jθ})

Summing:

$$
x_a + a x_b + a^2 x_c = \tfrac{3X}{2}e^{j\theta} + \tfrac{X}{2}e^{-j\theta}\underbrace{(1 + a + a^2)}_{=0} = \tfrac{3X}{2}e^{j\theta}
$$

Choosing **K = 2/3** gives $\vec{x}_s = X e^{j\theta}$. The vector magnitude then equals the phase pe