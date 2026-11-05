# Autonomous Gimbal Tracking Controller: PID with Velocity Feedforward and Acceleration Limiting (Cycle 26)

## 1. System Architecture

```
 Camera ──► Detector/Tracker ──► Pixel→Angle ──► Target State Estimator ──┐
                                                                          │ θ̂_t, ω̂_t
 IMU (base rates) ───────────────────────────────────────────────────┐   │
                                                                     ▼   ▼
                            ┌──────────── Outer Loop (this design) ──────────┐
                            │ e = θ̂_t − θ_g                                  │
                            │ ω_cmd = ω_ff + PID(e)                          │
                            │ → deadband → vel limit → accel/jerk limit      │
                            └──────────────────────┬─────────────────────────┘
                                                   ▼
                              Inner Rate Loop (motor driver / FOC) ──► Gimbal
```

The controller is formulated per axis (yaw ψ, pitch θ). Roll is normally stabilized to the horizon separately.

The outer loop commands angular **rate**. The inner rate loop, typically running at 1 to 8 kHz on the motor controller, is assumed to be fast and well tuned.

---

## 2. Measurement Model: Pixel Error to Angular Error

Inputs:
- Target centroid: (u, v)
- Principal point: (c_x, c_y)
- Focal lengths in pixels: f_x, f_y

The angular offsets of the target from the optical axis are:

$$
\delta_\psi = \arctan\!\left(\frac{u - c_x}{f_x}\right), \qquad
\delta_\theta = -\arctan\!\left(\frac{v - c_y}{f_y}\right)
$$

The target line-of-sight (LOS) angle, measured at the image timestamp t_k, is:

$$
z_k = \theta_g(t_k) + \delta(t_k)
$$

Important: θ_g(t_k) must be the gimbal encoder or attitude angle **interpolated to the exposure timestamp**, not the current angle. Otherwise image latency (typically 30 to 120 ms) appears as phantom target motion.

---

## 3. Target State Estimation (for Feedforward)

Use a constant-velocity Kalman filter per axis, with state x = [θ_t, ω_t]ᵀ.

**Predict**

$$
F = \begin{bmatrix}1 & \Delta t\\ 0 & 1\end{bmatrix},\quad
Q = q\begin{bmatrix}\Delta t^3/3 & \Delta t^2/2\\ \Delta t^2/2 & \Delta t\end{bmatrix}
$$

$$
\hat x^- = F\hat x, \qquad P^- = FPF^\top + Q
$$

**Update** (with H = [1 0] and R = σ²_pix / f²):

$$
K = P^-H^\top (HP^-H^\top + R)^{-1}, \qquad
\hat x = \hat x^- + K(z_k - H\hat x^-), \qquad
P = (I - KH)P^-
$$

**Latency compensation.** Propagate the estimate forward by the measured pipeline delay τ:

$$
\hat\theta_t(t) = \hat\theta_t(t_k) + \hat\omega_t\,(t - t_k)
$$

**Innovation gating.** Reject a measurement if the normalized innovation squared, (z − Hx̂⁻)² / S, exceeds about 9 (3σ). This prevents detector glitches from producing rate spikes.

**Tuning the process noise q.** q sets the trade-off between feedforward smoothness and responsiveness to target maneuvers:
- Smaller q gives smoother feedforward but more lag on maneuvers.
- Typical values are 0.5 to 5 (rad/s²)² for people or vehicles at moderate range.

---

## 4. Control Law

### 4.1 Tracking Error

$$
e(t) = \mathrm{wrap}_{\pi}\big(\hat\theta_t(t) - \theta_g(t)\big)
$$

### 4.2 Velocity Feedforward with Base-Motion Compensation

The feedforward term combines the estimated target LOS rate with the base (vehicle) rate:

$$
\omega_{ff} = k_{ff}\,\hat\omega_t \;-\; \omega_{base,\perp}
$$

- ω_base,⊥ is the vehicle IMU body rate projected onto the gimbal axis. Subtract it only if the inner loop runs in the **encoder frame**. If the inner loop is already inertially stabilized, omit this term.
- The feedforward gain k_ff lies in the range 0.8 to 1.0. Setting it to 1 removes steady-state lag for constant-velocity targets. Values slightly below 1 add robustness when the estimate is noisy.

### 4.3 PID Feedback

$$
\omega_{fb} = K_p\,e + K_i\!\int e\,dt - K_d\,\dot\theta_{g,f}
$$

**Derivative on measurement.** The derivative acts on the gimbal rate, not on the error. This avoids derivative kick when the target jumps or re-acquires. Filter it with a first-order low-pass:

$$
\dot\theta_{g,f}[k] = \alpha\,\dot\theta_{g,f}[k-1] + (1-\alpha)\,\dot\theta_g[k],
\qquad \alpha = \frac{\tau_d}{\tau_d + \Delta t}
$$

Because the inner loop already provides rate damping, K_d is often small or zero.

**Error deadband.** Apply a smooth deadband so pixel jitter does not cause visible micro-motion:

$$
e_{db} = \begin{cases} 0 & |e| < e_0 \\ \mathrm{sign}(e)\,(|e| - e_0) & \text{otherwise}\end{cases}
$$

Choose e_0 at roughly 0.5 to 2 pixels' worth of angle. Use e_db for the P and I terms. The feedforward path continues to provide motion within the deadband.

### 4.4 Raw Command

$$
\omega_{raw} = \omega_{ff} + \omega_{fb}
$$

---

## 5. Smoothness Shaping: Velocity, Acceleration, and Jerk Limiting

### 5.1 Velocity Saturation

$$
\omega_{sat} = \mathrm{clip}(\omega_{raw}, -\omega_{max}, \omega_{max})
$$

### 5.2 Acceleration (Slew-Rate) Limiting

$$
\omega_{cmd}[k] = \omega_{cmd}[k-1] + \mathrm{clip}\big(\omega_{sat} - \omega_{cmd}[k-1],\; -a_{max}\Delta t,\; a_{max}\Delta t\big)
$$

### 5.3 Optional Jerk Limit (for Cinematic Smoothness)

Track the commanded acceleration a_cmd and limit how fast it can change:

$$
a_{des} = \frac{\omega_{sat} - \omega_{cmd}[k-1]}{\Delta t},\quad
a_{cmd}[k] = a_{cmd}[k-1] + \mathrm{clip}(a_{des} - a_{cmd}[k-1], \pm j_{max}\Delta t)
$$

$$
a_{cmd}[k] \leftarrow \mathrm{clip}(a_{cmd}[k], \pm a_{max}),\qquad
\omega_{cmd}[k] = \omega_{cmd}[k-1] + a_{cmd}[k]\Delta t
$$

A pure jerk-limited integrator can overshoot the velocity target. To brake in time, cap the acceleration magnitude so the controller can still stop at the target:

$$
|a_{cmd}| \le \sqrt{2\,j_{max}\,|\omega_{sat} - \omega_{cmd}|}
$$

### 5.4 Anti-Windup (Back-Calculation)

When the limiters are active, the integral term would otherwise keep growing. Back-calculation bleeds off the difference between the raw and delivered command:

$$
I[k] = I[k-1] + \Delta t\left(K_i\,e_{db} + \frac{1}{T_t}\big(\omega_{cmd} - \omega_{raw}\big)\right),
\qquad T_t \approx \sqrt{T_i T_d}\ \text{ or }\ 0.5\,T_i
$$

Also clamp the integral: |I| ≤ I_max (for example, 20% of ω_max).

---

## 6. Mode Logic (Target Loss and Re-acquisition)

| State | Condition | Behavior |
|---|---|---|
| **TRACK** | Valid measurement within the gate | Full law (Sections 4 and 5) |
| **COAST** | No measurement for less than T_coast (about 0.5 to 1.5 s) | Feedback off and integral frozen. Feedforward decays: ω_ff ← ω̂_t·e^(−t/τ_c). KF predicts only and its covariance grows |
| **SEARCH/HOLD** | Lost for longer than T_coast | Decelerate to zero at a_max, hold, or run a search pattern |
| **REACQUIRE** | New detection | Reset the KF velocity to 0 (or to the last estimate if close). Ramp feedback gains in over about 200 ms. The slew limiter ensures no snap |

---

## 7. Reference Implementation (Python, Per Axis)

```python
import math
from dataclasses import dataclass

def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

def clip(x, lo, hi):
    return max(lo, min(hi, x))

@dataclass
class AxisParams:
    kp: float = 3.0        # 1/s
    ki: float = 0.4        # 1/s^2
    kd: float = 0.0        # s
    kff: float = 0.95
    tau_d: float = 0.02    # derivative LPF time constant (s)
    deadband: float = 0.0008   # rad (~1 px at f=1200)
    w_max: float = 3.0     # rad/s
    a_max: float = 6.0     # rad/s^2
    j_max: float = 40.0    # rad/s^3 (set to None to disable)
    i_max: float = 0.6     # rad/s
    t_track: float = 1.0   # anti-windup back-calculation time constant (s)
    q: float = 2.0         # KF process noise
    r: float = 1e-6        # KF measurement noise (rad^2)
    coast_time: float = 1.0
    coast_tau: float = 0.4

class TargetKF:
    def __init__(self, q, r):
        self.x = [0.0, 0.0]
        self.P = [[1.0, 0.0], [0.0, 1.0]]
        self.q, self.r = q, r
        self.init = False

    def predict(self, dt):
        th, w = self.x
        self.x = [th + w * dt, w]
        P = self.P
        q = self.q
        p00 = P[0][0] + dt * (P[1][0] + P[0][1]) + dt * dt * P[1][1] + q * dt**3 / 3
        p01 = P[0][1] + dt * P[1][1] + q * dt**2 / 2
        p11 = P[1][1] + q * dt
        self.P = [[p00, p01], [p01, p11]]

    def update(self, z, gate=9.0):
        if not self.init:
            self.x = [z, 0.0]
            self.P = [[self.r, 0], [0, 1.0]]
            self.init = True
            return True
        y = wrap_pi(z - self.x[0])
        S = self.P[0][0] + self.r
        if y * y / S > gate:
            return False                      # reject outlier
        k0, k1 = self.P[0][0] / S, self.P[1][0] / S
        self.x = [self.x[0] + k0 * y, self.x[1] + k1 * y]
        P = self.P
        self.P = [[(1 - k0) * P[0][0], (1 - k0) * P[0][1]],
                  [P[1][0] - k1 * P[0][0], P[1][1] - k1 * P[0][1]]]
        return True

class GimbalAxisController:
    def __init__(self, p: AxisParams):
        self.p = p
        self.kf = TargetKF(p.q, p.r)
        self.I = 0.0
        self.w_cmd = 0.0
        self.a_cmd = 0.0
        self.dterm_rate = 0.0
        self.t_since_meas = 1e9

    def on_measurement(self, gimbal_angle_at_exposure, pixel_angle_offset, age):
        """age = now - exposure time; KF assumed already predicted to 'now'."""
        z = gimbal_angle_at_exposure + pixel_angle_offset
        # Bring the measurement forward to "now" using the estimated velocity
        z_now = z + self.kf.x[1] * age
        if self.kf.update(z_now):
            self.t_since_meas = 0.0

    def step(self, dt, gimbal_angle, gimbal_rate, base_rate_on_axis=0.0):
        p = self.p
        self.kf.predict(dt)
        self.t_since_meas += dt

        tracking = self.t_since_meas < p.coast_time and self.kf.init
        fresh = self.t_since_meas < 0.15

        # --- feedforward ---
        if tracking:
            decay = 1.0 if fresh else math.exp(-(self.t_since_meas - 0.15) / p.coast_tau)
            w_ff = p.kff * self.kf.x[1] * decay - base_rate_on_axis
        else:
            w_ff = -base_rate_on_axis

        # --- feedback ---
        if tracking and fresh:
            e = wrap_pi(self.kf.x[0] - gimbal_angle)
            e_db = 0.0 if abs(e) < p.deadband else math.copysign(abs(e) - p.deadband, e)
            alpha = p.tau_d / (p.tau_d + dt)
            self.dterm_rate = alpha * self.dterm_rate + (1 - alpha) * gimbal_rate
            w_fb = p.kp * e_db + self.I - p.kd * self.dterm_rate
        else:
            e_db, w_fb = 0.0, 0.0          # freeze integral while coasting/lost

        w_raw = w_ff + w_fb

        # --- shaping: velocity -> jerk -> acceleration ---
        w_sat = clip(w_raw, -p.w_max, p.w_max)
        dv = w_sat - self.w_cmd
        if p.j_max:
            a_des = dv / dt
            a_brake = math.sqrt(2 * p.j_max * abs(dv))      # stop without overshoot
            a_des = clip(a_des, -a_brake, a_brake)
            self.a_cmd += clip(a_des - self.a_cmd, -p.j_max * dt, p.j_max * dt)
            self.a_cmd = clip(self.a_cmd, -p.a_max, p.a_max)
            step = self.a_cmd * dt
            # never step past the target velocity
            step = clip(step, min(0, dv), max(0, dv))
        else:
            step = clip(dv, -p.a_max * dt, p.a_max * dt)
        self.w_cmd += step

        # --- anti-windup (back-calculation) ---
        if tracking and fresh:
            self.I += dt * (p.ki * e_db + (self.w_cmd - w_raw) / p.t_track)
            self.I = clip(self.I, -p.i_max, p.i_max)

        return self.w_cmd   # send to inner rate loop
```

Run `step()` at a fixed outer-loop rate of 200 to 500 Hz. Call `on_measurement()` asynchronously whenever a detection arrives.

---

## 8. Tuning Procedure

1. **Inner rate loop first.**