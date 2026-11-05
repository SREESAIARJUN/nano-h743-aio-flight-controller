# Autonomous Gimbal Tracking Controller: PID + Velocity Feedforward + Acceleration Limiting (Cycle 16)

## 1. Problem Setup

**Goal:** Keep a visually detected target centered in the camera frame with smooth, cinematic motion. That means no jitter at rest, no overshoot on target acceleration, and bounded angular acceleration and jerk.

**Per-axis decoupling:** Pan (yaw, ψ) and tilt (pitch, θ) are controlled independently with identical structure. Roll is stabilized separately by the IMU loop. The derivation below is for one axis; subscripts are dropped where unambiguous.

**Architecture (cascade):**

```
 Vision (10–60 Hz) ──► Target Estimator ──► Outer Tracking Loop ──► Rate Shaper ──► Inner Rate Loop (≥500 Hz) ──► Motor
                       (angle + velocity)   (PID + FF)              (vel/acc/jerk     (gimbal IMU / encoder)
                                                                     limiting)
```

This document formulates the outer loop and rate shaper. The inner rate loop is assumed to track the commanded rate with bandwidth at least 5× the outer loop.

---

## 2. Measurement Model: Pixel Error → Angular Error

Given a detection centroid (u, v), principal point (c_x, c_y), and focal lengths in pixels (f_x, f_y):

$$
e_\psi = \arctan\!\left(\frac{u - c_x}{f_x}\right), \qquad
e_\theta = -\arctan\!\left(\frac{v - c_y}{f_y}\right)
$$

With zoom lenses, f_x and f_y must be updated from the zoom state. Otherwise the loop gain scales with zoom and the loop goes unstable at telephoto.

**Absolute target bearing** in the world-stabilized frame (gimbal angle q at the image timestamp):

$$
\phi_{\text{meas}}[k] = q(t_{\text{img}}) + e[k]
$$

Use the gimbal angle **at the image capture time**, not the current one. This requires buffering gimbal state history and indexing it by camera timestamp.

---

## 3. Target State Estimator (Feedforward Source)

A constant-velocity Kalman filter runs on the absolute bearing φ, with state **x** = [φ, ω_t]ᵀ (target angle and angular rate).

**Predict** (at the control rate, Δt):

$$
\mathbf{x}^- = F\mathbf{x}, \quad
F = \begin{bmatrix}1 & \Delta t\\ 0 & 1\end{bmatrix}, \quad
P^- = FPF^\top + Q
$$

$$
Q = \sigma_a^2 \begin{bmatrix}\Delta t^4/4 & \Delta t^3/2\\ \Delta t^3/2 & \Delta t^2\end{bmatrix}
$$

**Update** (when a vision frame arrives, H = [1, 0]):

$$
K = P^-H^\top (HP^-H^\top + R)^{-1}, \quad
\mathbf{x} = \mathbf{x}^- + K(\phi_{\text{meas}} - H\mathbf{x}^-)
$$

**Tuning knobs:**
- σ_a: target maneuverability. Larger values respond faster but produce noisier feedforward.
- R: detection noise, roughly (pixel jitter / f)².

**Outlier gating:** Reject a measurement if the Mahalanobis distance d² = ν²/S > 9 (3σ). After N consecutive rejections, re-initialize.

**Latency compensation:** Propagate the estimate forward by the total pipeline latency τ (capture → inference → command):

$$
\hat\phi_t = \phi + \omega_t\,\tau, \qquad \hat\omega_t = \omega_t
$$

---

## 4. Outer Tracking Control Law

**Tracking error** (world frame, wrapped to [−π, π]):

$$
e[k] = \text{wrap}(\hat\phi_t[k] - q[k])
$$

**Deadband with smooth blend.** This prevents hunting from detection jitter without creating a hard discontinuity. With deadband half-width e_db:

$$
\tilde e =
\begin{cases}
0 & |e| < e_{db}\\[2pt]
\text{sign}(e)\,\dfrac{(|e|-e_{db})^2}{2e_{db}} & e_{db} \le |e| < 2e_{db}\\[6pt]
e - \text{sign}(e)\,\tfrac{3}{2}e_{db} & |e| \ge 2e_{db}
\end{cases}
$$

This function is C¹-continuous.

**Rate command (pre-shaping):**

$$
\omega_{\text{cmd}}^{\text{raw}} =
\underbrace{k_{ff}\,\hat\omega_t}_{\text{feedforward}}
+ \underbrace{K_p\,\tilde e}_{P}
+ \underbrace{K_i\,I[k]}_{I}
+ \underbrace{K_d\,D_f[k]}_{D}
$$

### Integral Term (with anti-windup)

$$
I[k] = I[k-1] + \tilde e[k]\,\Delta t \;+\; \frac{\Delta t}{T_t}\big(\omega_{\text{cmd}}[k-1] - \omega_{\text{cmd}}^{\text{raw}}[k-1]\big)
$$

- This is back-calculation anti-windup with tracking time constant T_t ≈ √(T_i T_d), or about 0.5 T_i if there is no D term.
- Freeze I when the target is lost or confidence drops below threshold.
- Clamp |I| ≤ I_max.

### Derivative Term (on measurement, filtered)

The derivative is taken on the error excluding target motion (already handled by feedforward), which equals the negative gimbal rate:

$$
D_f[k] = \alpha_d D_f[k-1] + (1-\alpha_d)\,(-\omega_g[k]) + \ldots
$$

In practice, use **D on the error-rate residual**:

$$
\dot e_{\text{res}} = \hat\omega_t - \omega_g
$$

Then low-pass filter it with α_d = e^{−Δt/τ_d}, where τ_d ≈ T_d/N and N ∈ [5, 20]. Because feedforward already supplies ω̂_t, K_d is usually small or zero for gimbals.

### Feedforward Gain

Use k_ff ∈ [0.8, 1.0]. Values below 1 trade steady-state lag (which the integral removes) for robustness to velocity-estimate noise. Fade k_ff with estimator confidence:

$$
k_{ff} = k_{ff,0}\cdot \text{clip}\!\left(1 - \frac{\sqrt{P_{22}}}{\sigma_{\omega,\max}},\,0,\,1\right)
$$

---

## 5. Rate Shaper: Velocity, Acceleration, and Jerk Limiting

Raw commands are passed through a **second-order rate limiter** to guarantee smooth motion.

### Velocity Saturation

$$
\omega^\star = \text{clip}(\omega_{\text{cmd}}^{\text{raw}}, -\omega_{\max}, \omega_{\max})
$$

### Jerk-Limited Acceleration Tracking

The shaper maintains its own state (ω_s, a_s), the shaped rate and acceleration. The desired acceleration to reach ω* is:

$$
a^\star = \text{clip}\!\left(\frac{\omega^\star - \omega_s}{T_s},\; -a_{\max},\; a_{\max}\right)
$$

T_s is the shaping time constant, roughly 2–4 Δt.

**Braking-aware correction.** To reach ω* without overshoot under a jerk limit j_max, the acceleration must be able to fall to zero in time. The current acceleration is safe only if:

$$
|\omega^\star - \omega_s| \ge \frac{a_s^2}{2 j_{\max}}
$$

If this condition fails and the shaper is still accelerating toward ω*, set a* = 0 to begin ramping down.

**Jerk limiting:**

$$
a_s[k] = a_s[k-1] + \text{clip}(a^\star - a_s[k-1],\; -j_{\max}\Delta t,\; j_{\max}\Delta t)
$$

**Integration:**

$$
\omega_s[k] = \text{clip}(\omega_s[k-1] + a_s[k]\,\Delta t,\; -\omega_{\max},\; \omega_{\max})
$$

### Output

The shaped command sent to the inner loop is ω_cmd[k] = ω_s[k]. This value also feeds back into the anti-windup term in §4.

### Soft Mechanical Limits (tilt axis, and pan if not continuous)

Ensure the gimbal can stop before reaching hard stop q_lim:

$$
\omega_{\max}^{\text{eff}} = \min\!\left(\omega_{\max},\; \sqrt{2 a_{\max}\,\max(0,\, q_{\lim} - q)}\right)
$$

Apply this in the direction of the limit.

---

## 6. Target Loss Behavior

| State | Condition | Behavior |
|---|---|---|
| **TRACK** | Confidence ≥ c_on | Full control law |
| **COAST** | Lost for < T_coast (≈ 0.5–1.5 s) | Predict with the KF. Use k_ff only, with no P, I frozen, and FF decaying as e^{−t/τ_c} |
| **HOLD** | Lost for ≥ T_coast | Decay ω* → 0 through the shaper, which guarantees a smooth stop |
| **REACQUIRE** | Detection with confidence ≥ c_on | Re-init the KF at the measurement, keep the shaper state (no rate jump), and ramp K_p from 0 to nominal over ~0.3 s |

Hysteresis is applied with c_on > c_off.

---

## 7. Reference Implementation (Python, per axis)

```python
import math
from dataclasses import dataclass

def wrap(a): return (a + math.pi) % (2 * math.pi) - math.pi
def clip(x, lo, hi): return max(lo, min(hi, x))

@dataclass
class AxisParams:
    Kp: float = 4.0        # 1/s
    Ki: float = 0.8        # 1/s^2
    Kd: float = 0.0
    kff0: float = 0.95
    Tt: float = 0.6
    I_max: float = 0.3
    e_db: float = math.radians(0.3)
    w_max: float = math.radians(120)
    a_max: float = math.radians(300)
    j_max: float = math.radians(2000)
    Ts: float = 0.02
    sigma_a: float = math.radians(200)
    R: float = math.radians(0.15)**2
    latency: float = 0.08
    sigma_w_max: float = math.radians(30)

class GimbalAxisTracker:
    def __init__(self, p: AxisParams, dt: float):
        self.p, self.dt = p, dt
        self.x = [0.0, 0.0]                      # [phi, omega_t]
        self.P = [[1.0, 0.0], [0.0, 1.0]]
        self.I = 0.0
        self.ws = 0.0; self.as_ = 0.0            # shaper state
        self.last_raw = 0.0; self.last_cmd = 0.0
        self.kp_ramp = 1.0

    # ---------- Kalman filter ----------
    def kf_predict(self):
        dt, q = self.dt, self.p.sigma_a**2
        phi, w = self.x
        self.x = [phi + w * dt, w]
        P = self.P
        P00 = P[0][0] + dt*(P[1][0] + P[0][1]) + dt*dt*P[1][1] + q*dt**4/4
        P01 = P[0][1] + dt*P[1][1] + q*dt**3/2
        P11 = P[1][1] + q*dt*dt
        self.P = [[P00, P01], [P01, P11]]

    def kf_update(self, phi_meas) -> bool:
        nu = wrap(phi_meas - self.x[0])
        S = self.P[0][0] + self.p.R
        if nu * nu / S > 9.0:
            return False                          # gated outlier
        K0, K1 = self.P[0][0] / S, self.P[1][0] / S
        self.x = [self.x[0] + K0*nu, self.x[1] + K1*nu]
        P = self.P
        self.P = [[(1-K0)*P[0][0], (1-K0)*P[0][1]],
                  [P[1][0] - K1*P[0][0], P[1][1] - K1*P[0][1]]]
        return True

    # ---------- deadband ----------
    def deadband(self, e):
        d, a = self.p.e_db, abs(e)
        if a < d:
            return 0.0
        if a < 2 * d:
            return math.copysign((a - d)**2 / (2 * d), e)
        return e - math.copysign(1.5 * d, e)

    # ---------- shaper ----------
    def shape(self, w_star, w_lim):
        p, dt = self.p, self.dt
        w_star = clip(w_star, -w_lim, w_lim)
        dw = w_star - self.ws
        a_star = clip(dw / p.Ts, -p.a_max, p.a_max)
        # braking check: can we bring accel to zero before reaching w_star?
        if self.as_ * dw > 0 and abs(dw) < self.as_**2 / (2 * p.j_max):
            a_star = 0.0
        self.as_ += clip(a_star - self.as_, -p.j_max*dt, p.j_max*dt)
        self.ws = clip(self.ws + self.as_*dt, -w_lim, w_lim)
        return self.ws

    # ---------- main step ----------
    def step(self, q, w_gimbal, meas_phi=None, tracking=True, q_lim=None):
        p, dt = self.p, self.dt
        self.kf_predict()
        if meas_phi is not None:
            self.kf_update(meas_phi)

        phi_hat = self.x[0] + self.x[1] * p.latency
        w_hat = self.x[1]

        conf = clip(1 - math.sqrt(self.P[1][1]) / p.sigma_w_max, 0, 1)
        kff = p.kff0 * conf

        if tracking:
            e = self.deadband(wrap(phi_hat - q))
            # back-calculation anti-windup
            self.I += e*dt + (dt/p.Tt) * (self.last_cmd - self.last_raw)
            self.I = clip(self.I, -p.I_max, p.I_max)
            d_term = p.Kd * (w_hat - w_gimbal)
            raw = kff*w_hat + self.kp_ramp*p.Kp*e + p.Ki*self.I + d_term
        else:
            raw = kff * w_hat                     # coast: FF only, I frozen

        w_lim = p.w_max
        if q_lim is not None:                     # soft stop toward +limit
            w_lim = min(w_lim, math.sqrt(2*p.a_max*max(0.0, q_lim - q)))

        cmd = self.shape(raw, w_lim)
        self.last_raw, self.last_cmd = raw, cmd
        return cmd                                # rate command to inner loop
```

---

## 8. Tuning Procedure

1. **Inner loop first.** Verify rate-loop bandwidth ω_bw,in (typically 15–40 Hz) with a step-response test.
2. **Set the outer bandwidth** to ω_c ≈ ω_bw,in / 5 and below about 1/(3τ) for latency τ. Then K_p ≈ ω_c. For example, with τ = 80 ms, K_p ≲ 4 s⁻¹.
3. **Integral:** K_i ≈ K_p² / 10 to 1/5. Check phase margin ≥ 45°, since the latency term e^{−sτ} costs ω_c·τ radians.
4. **Feedforward:** Track a target moving at constant angular rate. Increase k_ff until steady lag