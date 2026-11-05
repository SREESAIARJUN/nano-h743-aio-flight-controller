# 3-Axis Gimbal on ESP32-S3, Cycle 15: FOC Mathematical Foundations

**Scope:** Clarke and Park transforms, the PMSM dq model, SVPWM duty-cycle derivation, and field-weakening. Each derivation ends with the equation that ships in firmware.

**Conventions**
- $\theta_e = p\,\theta_m$ is the electrical angle, with $p$ the number of pole pairs.
- $\omega = \dot\theta_e$ is the electrical speed.
- $\psi_m$ is the PM flux linkage amplitude in Wb.
- Machine phases are star-connected with an isolated neutral, so $x_a+x_b+x_c = 0$.

---

## 1. Clarke Transform (abc → αβ)

### 1.1 Space-vector definition

Let $a = e^{j2\pi/3}$. Define the complex space vector as

$$
\vec{x} = x_\alpha + j x_\beta = k\left(x_a + a\,x_b + a^2 x_c\right)
$$

**Choosing k.** Take a balanced sinusoidal set: $x_a = X\cos\theta$, $x_b = X\cos(\theta - 2\pi/3)$, $x_c = X\cos(\theta + 2\pi/3)$.

Writing $\cos\phi = \tfrac12(e^{j\phi}+e^{-j\phi})$ gives

$$
x_a + a x_b + a^2 x_c = \tfrac{X}{2}\Big[e^{j\theta}(1+1+1) + e^{-j\theta}(1 + a^2 + a^4)\Big] = \tfrac{3}{2}X e^{j\theta},
$$

since $1 + a^2 + a^4 = 1 + a^2 + a = 0$. This leaves two standard choices:

| Choice | k | Property |
|---|---|---|
| Amplitude-invariant | $2/3$ | $\lvert\vec x\rvert = X$ (phase peak) |
| Power-invariant | $\sqrt{2/3}$ | $p = \vec v\cdot\vec i$ directly |

**We use amplitude-invariant**, so that $\lvert i_{dq