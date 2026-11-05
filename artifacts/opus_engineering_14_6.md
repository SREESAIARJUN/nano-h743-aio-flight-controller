# Autonomous Gimbal Tracking Controller: PID with Velocity Feedforward and Acceleration Limiting

## 1. System Architecture

```
 Camera ──► Detector ──► Pixel→Angle ──► LOS Estimator ──► Outer Tracking Loop ──► Accel/Rate ──► Inner Rate Loop ──► Motors
 (30–60 Hz)  (bbox)       (intrinsics)   (α-β / KF, +τ)    (PID + Feedforward)     Limiter       (IMU, ~1 kHz)
                                              ▲                                                        │
                                              └──────────── gimbal encoder angles θ_g ─────────────────┘
```

- The **outer loop** (this design) outputs angular-rate commands per axis (pan ψ, tilt θ).
- The **inner loop** is a fast rate stabilizer that uses the gyro to reject base motion.
- The axes are treated as decoupled. The derivation is given for one axis; tilt is identical with its own parameters.

---

## 2. Measurement Model

### Pixel to angular error

The target centroid is at (u, v). The intrinsics are f_x, f_y, c_x, c_y.

$$
e_\psi^{m} = \arctan\!\left(\frac{u - c_x}{f_x}\right), \qquad
e_\theta^{m} = -\arctan\!\left(\frac{v - c_y}{f_y}\right)
$$

### Line-of-sight (LOS) angle in the gimbal-base frame

The image was captured at time t_c = t_k − τ, where τ is the total latency (exposure, inference and transport). The gimbal angle must be taken at that capture time:

$$
\lambda^{m}_k = \theta_g(t_k - \tau) + e^{m}_k
$$

Keep a short ring buffer of timestamped encoder angles so θ_g can be interpolated at t_c.

---

## 3. LOS State Estimator (α-β filter)

The state is the LOS angle λ and the LOS rate λ̇. The sample interval is T_s, which may vary.

Predict:
$$
\hat\lambda^- = \hat\lambda_{k-1} + T_s\,\hat{\dot\lambda}_{k-1}, \qquad
\hat{\dot\lambda}^- = \hat{\dot\lambda}_{k-1}
$$

Update, with residual r = λ^m_k − λ̂⁻:
$$
\hat\lambda_k = \hat\lambda^- + \alpha r, \qquad
\hat{\dot\lambda}_k = \hat{\dot\lambda}^- + \frac{\beta}{T_s} r
$$

Typical gains are α ≈ 0.5–0.85 and β ≈ 0.05–0.3. Equivalently, choose a tracking index and use β = α²/(2−α). A constant-velocity Kalman filter can replace this when measurement noise varies, for example by scaling R with detection confidence.

### Latency-compensated error

Use the LOS estimate projected to the current time:
$$
e_k = \underbrace{\hat\lambda_k + \hat{\dot\lambda}_k\,\tau}_{\text{LOS now}} - \theta_g(t_k)
$$

Then apply a smooth deadband to suppress jitter near center (e₀ ≈ 0.1–0.3°):
$$
e_k \leftarrow e_k\left(1 - e^{-(e_k/e_0)^2}\right)
$$

---

## 4. Control Law

### 4.1 Reference rate (PID + feedforward)

$$
\omega_{\text{ref},k} =
\underbrace{k_{ff}\,\hat{\dot\lambda}_k}_{\text{velocity FF}}
+ K_p e_k
+ I_k
+ K_d\, \dot e_{f,k}
$$

- **Feedforward** (k_ff ≈ 0.8–1.0) carries most of the tracking effort. The PID terms then only correct residual error, which is what makes the motion smooth.
- **Derivative** is low-passed (time constant T_f):
$$
\dot e_{f,k} = \frac{T_f}{T_f + T_s}\dot e_{f,k-1} + \frac{1}{T_f+T_s}(e_k - e_{k-1})
$$
  K_d is often near zero because the feedforward already supplies rate anticipation.

### 4.2 Braking-aware rate cap

This cap guarantees the gimbal can decelerate to zero error without overshoot:
$$
\omega_{\text{brake}} = \sqrt{2\,a_{\max}\,|e_k|} + |\hat{\dot\lambda}_k|, \qquad
\omega_{\text{ref}} \leftarrow \operatorname{clip}(\omega_{\text{ref}}, \pm\min(\omega_{\max}, \omega_{\text{brake}}))
$$

### 4.3 Acceleration (slew) limiting

$$
\Delta\omega = \operatorname{clip}\!\left(\omega_{\text{ref},k} - \omega_{\text{cmd},k-1},\; -a_{\max}T_s,\; a_{\max}T_s\right)
$$
$$
\omega_{\text{cmd},k} = \operatorname{clip}(\omega_{\text{cmd},k-1} + \Delta\omega,\; \pm\omega_{\max})
$$

**Optional jerk limit** for cinematic smoothness: limit the change in commanded acceleration by j_max·T_s per step before integrating it into ω_cmd.

### 4.4 Anti-windup (back-calculation)

$$
I_{k+1} = I_k + T_s\left[K_i e_k + K_{aw}(\omega_{\text{cmd},k} - \omega_{\text{ref},k})\right]
$$

Use K_aw ≈ 1/T_i to 1/√(T_iT_d), and also clamp |I| ≤ I_max. When saturated, the integrator bleeds off instead of winding up.

---

## 5. Tuning Rules

The effective delay is τ_eff = τ + T_s/2 + τ_inner. Set the outer-loop crossover:
$$
\omega_c \le \frac{1}{3\,\tau_{\text{eff}}}
$$

For example, τ_eff = 80 ms gives ω_c ≈ 4 rad/s.

| Parameter | Starting value | Notes |
|---|---|---|
| K_p | ω_c | units 1/s (rate per unit angle) |
| K_i | ω_c² / 4 to ω_c²/10 | removes steady bias |
| K_d | 0 to 0.05 | only if FF is unavailable |
| T_f | 2–3·T_s | derivative filter |
| k_ff | 0.9 | raise toward 1.0 if there is steady lag on moving targets |
| a_max | 2–6 rad/s² (cinematic), 20+ (aggressive) | lower values give smoother motion |
| ω_max | motor/inner-loop limit | |
| e₀ | 0.1–0.3° | deadband softness |

**Procedure:**
1. Set k_ff = 0 and tune K_p on a static target for fast response without overshoot.
2. Add K_i until steady error is zero.
3. Enable feedforward and increase k_ff on a constant-velocity target until lag disappears.
4. Reduce a_max until the motion looks smooth.

---

## 6. Target-Loss Handling

| State | Condition | Behavior |
|---|---|---|
| TRACK | detection valid | full law above |
| COAST | lost < t_coast (≈0.5–1 s) | propagate λ̂ with λ̇̂; ω_ref = k_ff λ̇̂ decaying as e^(−t/τ_c); freeze I |
| HOLD/SEARCH | lost > t_coast | ramp ω_cmd → 0 under a_max; optional search pattern |
| REACQUIRE | new detection | reset α-β state to the measurement; keep ω_cmd (bumpless) |

---

## 7. Reference Implementation (Python, per axis)

```python
import math
from dataclasses import dataclass

def clip(x, lo, hi): return max(lo, min(hi, x))

@dataclass
class AxisParams:
    kp: float = 4.0; ki: float = 2.0; kd: float = 0.0; tf: float = 0.05
    kff: float = 0.9; kaw: float = 1.0; i_max: float = 0.5
    w_max: float = 3.0; a_max: float = 4.0; j_max: float = 40.0
    alpha: float = 0.7; beta: float = 0.2
    e0: float = math.radians(0.2)
    t_coast: float = 0.8; tau_c: float = 0.4

class GimbalAxisTracker:
    def __init__(self, p: AxisParams):
        self.p = p
        self.lam = None; self.lam_dot = 0.0
        self.I = 0.0; self.e_prev = 0.0; self.de_f = 0.0
        self.w_cmd = 0.0; self.a_cmd = 0.0
        self.t_lost = 0.0

    def _estimate(self, lam_meas, Ts):
        if self.lam is None:                     # (re)acquisition
            self.lam, self.lam_dot = lam_meas, 0.0
            return
        lam_pred = self.lam + Ts * self.lam_dot
        r = lam_meas - lam_pred
        self.lam = lam_pred + self.p.alpha * r
        self.lam_dot += (self.p.beta / Ts) * r

    def update(self, Ts, theta_now, theta_at_capture=None, err_meas=None, latency=0.0):
        """err_meas: angular error from image (rad) or None if no detection."""
        p = self.p
        if err_meas is not None:
            self.t_lost = 0.0
            self._estimate(theta_at_capture + err_meas, Ts)
            ff_gain, integrate = 1.0, True
        else:
            self.t_lost += Ts
            if self.lam is None or self.t_lost > p.t_coast:
                self.lam = None                  # HOLD: decelerate to zero
                return self._limit(0.0, Ts)
            self.lam += Ts * self.lam_dot        # COAST
            ff_gain, integrate = math.exp(-self.t_lost / p.tau_c), False

        # latency-compensated error with smooth deadband
        e = (self.lam + self.lam_dot * latency) - theta_now
        e *= 1.0 - math.exp(-(e / p.e0) ** 2)

        # filtered derivative
        self.de_f = (p.tf * self.de_f + (e - self.e_prev)) / (p.tf + Ts)
        self.e_prev = e

        # PID + velocity feedforward
        w_ref = (p.kff * ff_gain * self.lam_dot
                 + p.kp * e + self.I + p.kd * self.de_f)

        # braking-aware cap
        w_brake = math.sqrt(2 * p.a_max * abs(e)) + abs(self.lam_dot)
        w_cap = min(p.w_max, w_brake)
        w_ref = clip(w_ref, -w_cap, w_cap)

        w_out = self._limit(w_ref, Ts)

        # back-calculation anti-windup
        if integrate:
            self.I += Ts * (p.ki * e + p.kaw * (w_out - w_ref))
            self.I = clip(self.I, -p.i_max, p.i_max)
        return w_out

    def _limit(self, w_ref, Ts):
        p = self.p
        a_des = clip((w_ref - self.w_cmd) / Ts, -p.a_max, p.a_max)
        self.a_cmd += clip(a_des - self.a_cmd, -p.j_max * Ts, p.j_max * Ts)  # jerk limit
        self.w_cmd = clip(self.w_cmd + self.a_cmd * Ts, -p.w_max, p.w_max)
        return self.w_cmd
```

**Usage per frame:**
```python
e_pan = math.atan2(u - cx, fx)
w_pan = pan.update(Ts, enc_pan_now, enc_pan_at(t_capture), e_pan, latency=tau)
send_rate_command(pan=w_pan, tilt=w_tilt)   # to inner gyro-stabilized loop
```

---

## 8. Validation Checklist

- **Step test** (static target offset 10°): verify no overshoot and a trapezoidal velocity profile capped by a_max.
- **Ramp test** (constant-velocity target): steady-state error should approach 0. Residual lag indicates k_ff or τ is underestimated.
- **Sinusoidal target**: check phase lag at the expected target frequencies against ω_c.
- **Dropout test** (occlude for 0.5 s, then 2 s): verify smooth coast, decay and bumpless reacquire.
- **Base disturbance** (shake the platform): the inner loop should reject it, and the outer loop should show minimal error spikes.
- **Monitoring**: log e, λ̇̂, ω_ref, ω_cmd and I. Watch for integrator saturation and limiter chatter.

---

## 9. Possible Next Refinements

- An adaptive a_max scaled by target-size or zoom level (smaller FOV calls for gentler motion).
- A constant-acceleration Kalman filter for maneuvering targets.
- Coupled pan/tilt magnitude limiting (limit the ‖ω‖ vector) for natural diagonal moves.
- Zoom-dependent gains: K_p scales with FOV.