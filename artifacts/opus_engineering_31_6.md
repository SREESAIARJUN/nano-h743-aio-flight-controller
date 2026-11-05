# Autonomous Gimbal Tracking Controller: PID with Velocity Feedforward and Acceleration Limiting (Cycle 31)

## 1. System Overview

```
 Camera ──► Detector ──► Pixel→Angle ──► Target State Estimator ──┐
   ▲                                     (α-β / Kalman, latency    │
   │                                      compensation)            │
   │                                            │ θ̂_t, ω̂_t         │
   │                                            ▼                  │
   │        ┌──────────── Error e = θ̂_t − θ_g ◄─────────────────────┤
   │        ▼                                                      │
   │   PID (filtered D, anti-windup) ──(+)── Feedforward ω̂_t − ω_base
   │                                    │
   │                          Rate limit → Accel limit → (Jerk limit)
   │                                    │
   └──────── Gimbal motor rate loop ◄───┘   (inner loop, IMU-stabilized)
```

The outer loop is the **visual position loop**, running at camera rate (30–60 Hz). It outputs an **angular-rate command** to the inner, IMU-stabilized gimbal rate loop, which runs at 500 Hz–2 kHz. Each axis (pan ψ and tilt φ) gets its own instance of the controller.

---

## 2. Measurement Model: Pixels to Angular Error

The camera intrinsics are (fₓ, f_y, cₓ, c_y). The detection centroid is (u, v).

$$
\varepsilon_\psi = \arctan\!\left(\frac{u - c_x}{f_x}\right), \qquad
\varepsilon_\phi = -\arctan\!\left(\frac{v - c_y}{f_y}\right)
$$

**Pan/tilt coupling correction.** Near the tilt extremes, a given image-plane error requires a larger pan motion:

$$
\varepsilon_\psi^{\text{cmd}} = \frac{\varepsilon_\psi}{\max(\cos\phi_g,\ \cos\phi_{\text{lim}})}, \qquad \phi_{\text{lim}} \approx 80^\circ
$$

**Target absolute angle** (world/stabilized frame), measured at image timestamp t_img:

$$
\theta_t^{\text{meas}} = \theta_g(t_{\text{img}}) + \varepsilon
$$

Use the gimbal encoder/IMU angle **interpolated to the image timestamp**, not the current angle. This step alone removes most latency-induced oscillation.

---

## 3. Target State Estimation (Feedforward Source)

A constant-velocity Kalman filter is applied per axis, with state x = [θ_t, ω_t]ᵀ:

$$
\mathbf{x}_{k|k-1} = \begin{bmatrix}1 & T\\0 & 1\end{bmatrix}\mathbf{x}_{k-1}, \qquad
Q = q\begin{bmatrix}T^3/3 & T^2/2\\ T^2/2 & T\end{bmatrix}, \qquad
H = [1\ \ 0],\ R = \sigma_{\text{pix}}^2/f^2
$$

**Lightweight α-β alternative:**

$$
r = \theta_t^{\text{meas}} - (\hat\theta + \hat\omega T), \quad
\hat\theta \leftarrow \hat\theta + \hat\omega T + \alpha r, \quad
\hat\omega \leftarrow \hat\omega + \tfrac{\beta}{T} r
$$

Typical values are α ≈ 0.5 and β ≈ 0.1. Lower values give smoother output with more lag.

**Latency compensation.** The total pipeline latency τ (exposure, inference, and transport) is typically 50–120 ms. Predict the target forward by τ:

$$
\theta_t^{\text{pred}} = \hat\theta + \hat\omega\,\tau
$$

---

## 4. Control Law

### Error

$$
e = \operatorname{wrap}_{\pi}\!\left(\theta_t^{\text{pred}} - \theta_g\right)
$$

A soft deadband removes jitter when the target is centered:

$$
e_{db} = \operatorname{sign}(e)\,\max(|e| - \delta,\ 0)\cdot\frac{|e|}{|e|+\delta}
$$

Use δ ≈ 0.2–0.5°. This form is smooth with no step, unlike a hard deadband.

### Velocity feedforward

$$
\omega_{ff} = K_{ff}\,(\hat\omega_t - \omega_{\text{base}}), \qquad K_{ff}\in[0.8,\ 1.0]
$$

Here ω_base is the vehicle body rate projected onto the axis (from the IMU). If the inner loop is already world-stabilized, set ω_base = 0.

### PID with filtered derivative and derivative-on-measurement

$$
\omega_{fb} = K_p e_{db} + K_i \!\int\! e_{db}\,dt - K_d\,\dot{\theta}_{g,f}
$$

The derivative term acts on the gimbal angle, not the error. This avoids derivative kick when a new target is acquired. Discretize the derivative with a first-order filter (N ≈ 5–20):

$$
D_k = \frac{T_f}{T_f + T}D_{k-1} - \frac{K_d}{T_f + T}\left(\theta_{g,k} - \theta_{g,k-1}\right), \quad T_f = K_d/(K_p N)
$$

### Raw command

$$
\omega_{\text{raw}} = \omega_{ff} + \omega_{fb}
$$

---

## 5. Smoothing Stage: Rate, Acceleration, and Jerk Limits

**Rate saturation:**

$$
\omega_{\text{sat}} = \operatorname{clip}(\omega_{\text{raw}},\ -\omega_{\max},\ \omega_{\max})
$$

**Acceleration limit (slew):**

$$
\omega_{\text{cmd},k} = \omega_{\text{cmd},k-1} + \operatorname{clip}\!\left(\omega_{\text{sat}} - \omega_{\text{cmd},k-1},\ -a_{\max}T,\ a_{\max}T\right)
$$

**Optional jerk limit** (gives an S-curve for cinematic smoothness):

$$
a_k = a_{k-1} + \operatorname{clip}\!\left(\tfrac{\omega_{\text{sat}}-\omega_{k-1}}{T} - a_{k-1},\ -j_{\max}T,\ j_{\max}T\right),\quad a_k\in[-a_{\max},a_{\max}]
$$

**Stopping-distance-aware braking.** This prevents overshoot when approaching a mechanical limit θ_lim:

$$
\omega_{\max}^{\text{eff}} = \min\!\left(\omega_{\max},\ \sqrt{2\,a_{\max}\,|\theta_{\lim} - \theta_g|}\right)
$$

### Anti-windup (back-calculation)

$$
I_k = I_{k-1} + \left[K_i\,e_{db} + K_{aw}\,(\omega_{\text{cmd}} - \omega_{\text{raw}})\right]T, \qquad K_{aw}\approx 1/T_i
$$

Also freeze integration while the target is lost or while |e| exceeds the large-error threshold (conditional integration).

---

## 6. Target-Loss Behavior (State Machine)

| State | Condition | Action |
|---|---|---|
| **TRACK** | Detection confidence > c_hi | Full control law |
| **COAST** | Lost for < t_coast (≈0.5–1 s) | Run estimator predict only; ω_cmd = ω̂_t, decaying via ω̂ ← λω̂ (λ≈0.95); freeze I |
| **HOLD** | Lost for > t_coast | Decelerate to 0 at a_max; hold stabilized attitude |
| **REACQUIRE** | New detection | Reset the estimator to the new measurement; bumpless transfer: set I so ω_raw = current ω_cmd |

Use hysteresis on confidence (c_hi / c_lo) to avoid chatter between states.

---

## 7. Reference Implementation (per axis)

```python
import math

class GimbalAxisTracker:
    def __init__(self, Kp=3.0, Ki=0.4, Kd=0.15, Kff=0.9, N=10,
                 w_max=math.radians(120), a_max=math.radians(300),
                 j_max=math.radians(3000), deadband=math.radians(0.3),
                 alpha=0.5, beta=0.1, latency=0.08, Kaw=None):
        self.Kp, self.Ki, self.Kd, self.Kff, self.N = Kp, Ki, Kd, Kff, N
        self.w_max, self.a_max, self.j_max = w_max, a_max, j_max
        self.db, self.alpha, self.beta, self.tau = deadband, alpha, beta, latency
        self.Kaw = Kaw if Kaw is not None else (Ki / Kp if Kp else 0.0)
        self.th_hat = None; self.w_hat = 0.0
        self.I = 0.0; self.D = 0.0; self.prev_thg = None
        self.w_cmd = 0.0; self.a_cmd = 0.0

    @staticmethod
    def wrap(x): return (x + math.pi) % (2 * math.pi) - math.pi

    def soft_deadband(self, e):
        m = max(abs(e) - self.db, 0.0)
        return math.copysign(m, e) * abs(e) / (abs(e) + self.db + 1e-9)

    def estimate(self, meas, T):
        if self.th_hat is None:
            self.th_hat, self.w_hat = meas, 0.0; return
        pred = self.th_hat + self.w_hat * T
        r = self.wrap(meas - pred)
        self.th_hat = pred + self.alpha * r
        self.w_hat += self.beta / T * r

    def update(self, T, th_g, th_g_at_img=None, eps=None,
               w_base=0.0, theta_lim=None, tracking=True):
        # 1. Estimator
        if tracking and eps is not None:
            self.estimate(th_g_at_img + eps, T)
        elif self.th_hat is not None:                 # COAST
            self.th_hat += self.w_hat * T
            self.w_hat *= 0.95
        if self.th_hat is None:
            return self._limit(0.0, T, th_g, theta_lim)

        # 2. Error with latency compensation
        e = self.soft_deadband(self.wrap(self.th_hat + self.w_hat * self.tau - th_g))

        # 3. Feedforward + PID (D on measurement, filtered)
        w_ff = self.Kff * (self.w_hat - w_base)
        if self.prev_thg is not None and self.Kd > 0:
            Tf = self.Kd / (self.Kp * self.N)
            self.D = (Tf * self.D - self.Kd * (th_g - self.prev_thg)) / (Tf + T)
        self.prev_thg = th_g
        w_raw = w_ff + self.Kp * e + self.I + self.D

        # 4. Limits
        w_cmd = self._limit(w_raw, T, th_g, theta_lim)

        # 5. Anti-windup (back-calculation, conditional)
        if tracking:
            self.I += (self.Ki * e + self.Kaw * (w_cmd - w_raw)) * T
        return w_cmd

    def _limit(self, w_target, T, th_g, theta_lim):
        w_max = self.w_max
        if theta_lim is not None:
            w_max = min(w_max, math.sqrt(2 * self.a_max * abs(theta_lim - th_g)))
        w_target = max(-w_max, min(w_max, w_target))
        # jerk-limited acceleration
        a_des = (w_target - self.w_cmd) / T
        da = max(-self.j_max * T, min(self.j_max * T, a_des - self.a_cmd))
        self.a_cmd = max(-self.a_max, min(self.a_max, self.a_cmd + da))
        # prevent overshoot of the target rate
        step = self.a_cmd * T
        if abs(step) > abs(w_target - self.w_cmd):
            step = w_target - self.w_cmd
            self.a_cmd = step / T
        self.w_cmd += step
        return self.w_cmd
```

---

## 8. Tuning Procedure

1. **Inner loop first.** Verify that the gimbal rate loop has a bandwidth ω_bw,in at least 5× the outer-loop crossover.
2. **Set K_ff = 0 and K_i = K_d = 0.** Raise K_p until a slight overshoot appears on step targets, then back off 30%. Rule of thumb: crossover ω_c ≈ K_p ≲ 1/(3τ). For example, τ = 80 ms gives K_p ≲ 4 s⁻¹.
3. **Enable feedforward.** Ramp K_ff from 0.5 toward 1.0 on a constant-velocity target until tracking lag is about 0 without leading.
4. **Add K_i** (≈ K_p/8) only to remove steady bias, such as from a moving base or calibration offsets.
5. **Add K_d** only if residual overshoot remains. Keep N small to avoid amplifying pixel noise.
6. **Choose limits by aesthetic profile:**

| Profile | ω_max | a_max | j_max | δ |
|---|---|---|---|---|
| Cinematic | 30–60 °/s | 60–120 °/s² | 300–600 °/s³ | 0.5° |
| Sports/fast | 120–200 °/s | 300–600 °/s² | 3000+ °/s³ | 0.2° |

---

## 9. Stability and Performance Notes

- **Effective loop delay.** The delay is τ plus T/2 from sampling. The phase margin drops by ω_c·τ (in radians), so latency compensation and timestamp-matched gimbal angles are essential.
- **Acceleration limiting is a nonlinearity.** Under large errors the loop behaves like a bang-bang profile. Back-calculation anti-windup keeps recovery overshoot-free.
- **Steady-state behavior with K_ff ≈ 1.**
  - Ramp targets: zero lag error without needing a large K_i.
  - Accelerating targets: residual error ≈ a_t / (K_p K_i).
- **Noise path.** The estimator's β sets how much pixel jitter reaches ω_ff. Reduce β before reducing K_p if the image looks "nervous."

**Cycle 31 deliverable summary:** timestamp-aligned angular error → α-β/Kalman target-rate estimate with latency prediction → soft-deadband PID (filtered D-on-measurement, back-calculation anti-windup) + base-compensated velocity feedforward → rate, acceleration, and jerk limiting with stopping-distance awareness → TRACK/COAST/HOLD/REACQUIRE supervisor with bumpless transfer.