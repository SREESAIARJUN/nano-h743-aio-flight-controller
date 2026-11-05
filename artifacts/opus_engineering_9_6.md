# Autonomous Gimbal Tracking Controller: PID + Velocity Feedforward + Acceleration Limiting (Cycle 9)

This controller drives a 2-axis (pan/tilt) gimbal in **rate mode** to keep a visually detected target centered with smooth, jerk-free motion. Each axis is controlled independently with the same structure. Where they differ, yaw is ψ and pitch is θ.

---

## 1. Signal Chain Overview

```
Camera ─► Detector ─► Pixel error ─► Angular error e(k) ─┐
                                                          ├─► PID ─┐
Gimbal encoders/IMU ─► Gimbal angle q(k) ─┐               │        │
                                          ▼               │        ▼
                        Target angle estimator (KF) ─► ω_ff ──► Σ ─► Rate limit ─► Accel/Jerk limit ─► Gimbal rate cmd
                                                                      ▲                    │
                                                                      └── anti-windup ◄────┘
```

---

## 2. Measurement Model

### 2.1 Pixel to angular error

The camera has intrinsics f_x, f_y (focal lengths in pixels) and principal point (c_x, c_y). For a detected target centroid (u, v):

$$
e_\psi = \arctan\!\left(\frac{u - c_x}{f_x}\right), \qquad
e_\theta = -\arctan\!\left(\frac{v - c_y}{f_y}\right)
$$

The arctan form stays accurate at wide FOV. For narrow FOV, the small-angle form e ≈ (u − c_x)/f_x is fine.

### 2.2 Latency compensation

The image was captured at time t_img = t_now − τ, where τ is the exposure, inference, and transport delay. Reference the error to the gimbal pose **at capture time**, not the current pose:

$$
\phi_{\text{tgt}}(t_{img}) = q(t_{img}) + e(t_{img})
$$

Here q(t_img) is interpolated from a buffered history of gimbal angles. This is the single most important fix for oscillation in visual servoing.

---

## 3. Target State Estimator (Feedforward Source)

Estimate the target's **inertial angular position and rate** per axis with a constant-velocity Kalman filter. Use constant-acceleration for agile targets.

**State:** x = [φ, φ̇]ᵀ

**Prediction:**

$$
x_{k|k-1} = \begin{bmatrix}1 & \Delta t\\ 0 & 1\end{bmatrix} x_{k-1}, \quad
Q = \sigma_a^2 \begin{bmatrix}\Delta t^4/4 & \Delta t^3/2\\ \Delta t^3/2 & \Delta t^2\end{bmatrix}
$$

**Update** (with measurement z = φ_tgt, H = [1 0], R = σ_px² / f²):

$$
K = P H^\top (H P H^\top + R)^{-1}, \quad x \leftarrow x + K(z - Hx)
$$

**Outputs:**
- Feedforward rate: ω_ff = φ̂̇, propagated forward by τ.
- Predicted target angle at the current time: φ̂_now = φ̂ + φ̂̇ · τ.

**Fallback (lighter):** an α-β filter with α ≈ 0.5 and β ≈ 0.1.

---

## 4. Control Law

### 4.1 Tracking error (current time, latency-compensated)

$$
e(k) = \hat\phi_{\text{now}}(k) - q(k)
$$

### 4.2 Error shaping (smoothness near center)

A soft deadband prevents hunting on detector jitter without creating a discontinuity:

$$
\tilde e =
\begin{cases}
0, & |e| < d\\
\operatorname{sign}(e)\,\dfrac{(|e|-d)^2}{2w}, & d \le |e| < d+w\\
e - \operatorname{sign}(e)\,(d + w/2), & |e| \ge d+w
\end{cases}
$$

Typical values: d ≈ 0.5–1 px equivalent, w ≈ 2d.

### 4.3 PID with filtered derivative on measurement

**Proportional:**

$$
P = K_p\,\tilde e
$$

**Integral** (with back-calculation anti-windup):

$$
I_k = I_{k-1} + K_i\,\tilde e\,\Delta t + K_{aw}\,(\omega_{out,k-1} - \omega_{raw,k-1})\,\Delta t
$$

**Derivative** (on the error rate, first-order low-pass filtered):

$$
D_k = \frac{T_f}{T_f + \Delta t} D_{k-1} + \frac{K_d}{T_f + \Delta t}\left(\tilde e_k - \tilde e_{k-1}\right)
$$

The time constant T_f ≈ 2–4 × the vision frame period.

> **Note:** Because the feedforward term supplies the target velocity, the derivative term only needs to damp residual error dynamics. K_d is usually small or zero.

### 4.4 Velocity feedforward

$$
\omega_{ff} = K_{ff}\,\hat{\dot\phi}\cdot g(\text{conf})
$$

- K_ff ∈ [0.8, 1.0]. Start at 0.9 to avoid overshoot from estimator lag.
- g(conf) ∈ [0, 1] scales feedforward by tracker confidence or covariance, e.g. g = clip(1 − P₂₂/P_max, 0, 1).

### 4.5 Raw command

$$
\omega_{raw} = \omega_{ff} + P + I_k + D_k
$$

---

## 5. Rate, Acceleration, and Jerk Limiting

The limits are applied in order.

**1. Rate saturation:**

$$
\omega_{sat} = \text{clip}(\omega_{raw}, -\omega_{max}, \omega_{max})
$$

**2. Acceleration limit (slew):**

$$
\omega_{out,k} = \omega_{out,k-1} + \text{clip}\left(\omega_{sat} - \omega_{out,k-1},\; -a_{max}\Delta t,\; a_{max}\Delta t\right)
$$

**3. Jerk limit (optional, Cycle 9 addition for cinematic smoothness):** track the commanded acceleration a_k and limit its change:

$$
a_k = a_{k-1} + \text{clip}\left(\frac{\omega_{sat}-\omega_{out,k-1}}{\Delta t} - a_{k-1},\; -j_{max}\Delta t,\; j_{max}\Delta t\right),\quad a_k \in [-a_{max}, a_{max}]
$$

$$
\omega_{out,k} = \omega_{out,k-1} + a_k \Delta t
$$

**Braking-aware acceleration limit.** To avoid overshoot when approaching zero error with high velocity, cap the rate at the value that can still stop within the remaining error:

$$
|\omega_{rel}| \le \sqrt{2\,a_{max}\,|e|}
$$

Here ω_rel = ω_out − ω_ff is the rate relative to the target.

**Anti-windup:** the difference (ω_out − ω_raw) feeds back into the integrator (Section 4.3), so it never winds up while limits are active.

---

## 6. Target Loss and Reacquisition

| State | Condition | Behavior |
|---|---|---|
| TRACK | detection, conf ≥ c_min | Full law |
| COAST | lost for < T_coast (≈0.5–1 s) | KF predicts; ω = ω_ff · e^(−t/τ_c); I frozen |
| HOLD/SEARCH | lost for > T_coast | Decelerate at a_max to 0, or run a search pattern |
| REACQUIRE | detection returns | Reset D; bumpless transfer: I ← ω_out − ω_ff − P |

**Gating:** reject detections whose innovation exceeds a Mahalanobis threshold (χ²₁ > 9). This prevents snapping to distractors.

---

## 7. Reference Implementation (Python)

```python
import math
import numpy as np

class AxisTracker:
    def __init__(self, Kp=3.0, Ki=0.4, Kd=0.05, Kff=0.9, Kaw=2.0,
                 w_max=math.radians(120), a_max=math.radians(300),
                 j_max=math.radians(3000), Tf=0.08, dead=math.radians(0.05),
                 sigma_a=math.radians(60), sigma_meas=math.radians(0.1),
                 T_coast=0.8, tau_c=0.4):
        self.Kp, self.Ki, self.Kd, self.Kff, self.Kaw = Kp, Ki, Kd, Kff, Kaw
        self.w_max, self.a_max, self.j_max = w_max, a_max, j_max
        self.Tf, self.d, self.wz = Tf, dead, 2 * dead
        self.sa2, self.R = sigma_a**2, sigma_meas**2
        self.T_coast, self.tau_c = T_coast, tau_c
        self.x = np.zeros(2)
        self.P = np.diag([1.0, 1.0])
        self.I = self.D = self.e_prev = 0.0
        self.w_out = self.a = self.w_raw_prev = 0.0
        self.lost_t = 0.0
        self.init = False

    # ---------- Estimator ----------
    def _predict(self, dt):
        F = np.array([[1, dt], [0, 1]])
        Q = self.sa2 * np.array([[dt**4/4, dt**3/2], [dt**3/2, dt**2]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def _update(self, z):
        H = np.array([1.0, 0.0])
        S = H @ self.P @ H + self.R
        y = z - H @ self.x
        if y * y / S > 9.0:            # innovation gate
            return False
        K = self.P @ H / S
        self.x += K * y
        self.P = (np.eye(2) - np.outer(K, H)) @ self.P
        return True

    # ---------- Error shaping ----------
    def _shape(self, e):
        a, s = abs(e), math.copysign(1, e)
        if a < self.d:
            return 0.0
        if a < self.d + self.wz:
            return s * (a - self.d)**2 / (2 * self.wz)
        return e - s * (self.d + self.wz / 2)

    # ---------- Main step ----------
    def step(self, dt, q_now, meas=None, q_at_capture=None, latency=0.0, conf=1.0):
        """q_now: current gimbal angle; meas: angular image error at capture time."""
        self._predict(dt)
        valid = False
        if meas is not None:
            z = q_at_capture + meas
            if not self.init:
                self.x[:] = [z, 0.0]
                self.init = True
                valid = True
            else:
                valid = self._update(z)

        if valid:
            self.lost_t = 0.0
        else:
            self.lost_t += dt

        phi_now = self.x[0] + self.x[1] * latency
        e = self._shape(phi_now - q_now)

        g = max(0.0, min(1.0, 1 - self.P[1, 1] / math.radians(50)**2)) * conf
        w_ff = self.Kff * self.x[1] * g

        if self.lost_t == 0.0:                       # TRACK
            P = self.Kp * e
            self.D = (self.Tf * self.D + self.Kd * (e - self.e_prev)) / (self.Tf + dt)
            self.I += self.Ki * e * dt + self.Kaw * (self.w_out - self.w_raw_prev) * dt
            w_raw = w_ff + P + self.I + self.D
        elif self.lost_t < self.T_coast:             # COAST
            w_raw = w_ff * math.exp(-self.lost_t / self.tau_c)
        else:                                        # HOLD
            w_raw = 0.0
        self.e_prev = e

        # Braking-aware cap (relative to target motion)
        w_brake = math.sqrt(2 * self.a_max * abs(e)) if e != 0 else 0.0
        w_rel = max(-w_brake - 1e-6, min(w_brake + 1e-6, w_raw - w_ff))
        w_sat = max(-self.w_max, min(self.w_max, w_ff + w_rel))

        # Jerk-limited acceleration
        a_des = (w_sat - self.w_out) / dt
        da = max(-self.j_max * dt, min(self.j_max * dt, a_des - self.a))
        self.a = max(-self.a_max, min(self.a_max, self.a + da))
        # Avoid overshooting w_sat in the final step
        if abs(self.a * dt) > abs(w_sat - self.w_out):
            self.a = (w_sat - self.w_out) / dt
        self.w_out += self.a * dt

        self.w_raw_prev = w_raw
        return self.w_out
```

**Usage:** run one `AxisTracker` per axis at the gimbal loop rate (100–500 Hz). Pass `meas` only on frames that have a new detection.

---

## 8. Tuning Procedure

1. **Identify latency τ** by step-moving the gimbal and timing the image response. Set the history buffer accordingly.
2. **Feedforward off, PID only:** raise K_p until slight overshoot, then back off 30%. The bandwidth bound is roughly ω_c < 1/(3τ_total).
3. **Add K_i** at about K_p/10 to K_p/5 to remove steady-state lag on slow drift.
4. **Enable feedforward.** Tune the KF σ_a so the estimate follows target maneuvers without noisy ω_ff. Ramp K_ff from 0.7 toward 0.95 while watching for overshoot on target reversals.
5. **Set limits from mechanics:** a_max ≈ 70% of the motor torque-limited acceleration. Set j_max for the desired cinematic feel: lower values give smoother motion but more lag.
6. **Add K_d** only if residual oscillation remains after feedforward.

**Performance metrics:** RMS pixel error, peak error on target reversal, command jerk RMS, and time-to-recenter after reacquisition.

---

## 9. Cycle 9 Refinements (Summary)

- Latency-compensated error using the gimbal pose at capture time plus KF forward prediction.
- Confidence/covariance-weighted feedforward, which prevents noisy ω_ff at track start.
- C¹-continuous soft deadband instead of a hard deadzone.
- Braking-aware rate cap, √(2·a_max·|e|), which eliminates overshoot on recentering.
- Jerk-limited S-curve acceleration for cinematic smoothness.
- Back-calculation anti-windup tied to the final limited output.
- Explicit TRACK / COAST / HOLD / REACQUIRE state machine with bumpless transfer and innovation gating.