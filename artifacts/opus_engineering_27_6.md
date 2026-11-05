# Gimbal Visual-Tracking Controller: PID + Velocity Feedforward + Acceleration Limiting (Cycle 27)

## 1. System Architecture

```
 Camera (30–60 Hz)       Estimator (per frame)        Control loop (200–500 Hz)          Gimbal
┌───────────────┐   ┌──────────────────────────┐   ┌───────────────────────────────┐   ┌────────────┐
│ Detector/     │──▶│ Pixel → angle            │──▶│ Latency-compensated error     │──▶│ Inner rate │
│ tracker bbox  │   │ Absolute bearing θ_t     │   │ PID + velocity feedforward    │   │ loop (FOC/ │
│ (u,v,t_img)   │   │ Kalman: θ̂_t, ω̂_t, α̂_t     │   │ Braking-curve velocity limit  │   │ ESC + IMU) │
└───────────────┘   └──────────────────────────┘   │ Accel (+ jerk) slew limit     │   └────────────┘
                                                   │ Anti-windup                   │
                                                   └───────────────────────────────┘
```

The outer loop outputs an **angular rate command** ω_cmd per axis (pan/yaw, tilt/pitch). It assumes an IMU-stabilized inner rate loop, which is standard for brushless gimbals. The two axes are treated independently. The equations below are written for one axis.

---

## 2. Measurement Model

### 2.1 Pixel to angular error

Convert pixels to angles using the camera intrinsics (f_x, f_y, c_x, c_y). Use the target centroid (u, v):

$$
e_\psi = \arctan\!\left(\frac{u - c_x}{f_x}\right), \qquad
e_\theta = -\arctan\!\left(\frac{v - c_y}{f_y}\right)
$$

For wide-FOV lenses, first undistort (u, v). For a large tilt, compute the full 3D ray **r** = K⁻¹[u, v, 1]ᵀ and rotate it into the gimbal frame. This avoids pan/tilt cross-coupling.

### 2.2 Absolute (world-frame) target bearing

Use the gimbal angle **at the image timestamp** t_img, not the current angle:

$$
z_k = \theta_g(t_{img}) + e_k
$$

This decouples target motion from gimbal motion. The estimator then sees the true target trajectory, rather than an error that the controller itself is changing.

---

## 3. Target State Estimator (Constant-Acceleration Kalman Filter)

The state is **x** = [θ_t, ω_t, α_t]ᵀ, with sample interval Δt = t_img,k − t_img,k−1.

$$
F = \begin{bmatrix}1 & \Delta t & \tfrac{1}{2}\Delta t^2\\ 0 & 1 & \Delta t\\ 0&0&1\end{bmatrix},\quad
H = [1\;0\;0],\quad
Q = q\begin{bmatrix}\tfrac{\Delta t^5}{20}&\tfrac{\Delta t^4}{8}&\tfrac{\Delta t^3}{6}\\ \tfrac{\Delta t^4}{8}&\tfrac{\Delta t^3}{3}&\tfrac{\Delta t^2}{2}\\ \tfrac{\Delta t^3}{6}&\tfrac{\Delta t^2}{2}&\Delta t\end{bmatrix},\quad
R = \sigma_z^2
$$

Run the standard predict/update cycle on each frame. Between frames, the control loop **propagates the state forward** without an update.

Tuning knobs:
- **q** (jerk spectral density) sets agility versus smoothness. Lower q gives smoother feedforward.
- **σ_z** ≈ detector jitter in radians, i.e. pixel jitter divided by f.

**Gating:** reject a measurement if its normalized innovation squared exceeds χ²₁(0.99) ≈ 6.63. This handles detector glitches and ID switches.

---

## 4. Control Law

### 4.1 Latency-compensated error

Total delay τ = (now − t_img) + τ_actuator. Predict the target ahead by τ:

$$
\hat\theta_t^{+} = \hat\theta_t + \hat\omega_t\,\tau + \tfrac12\hat\alpha_t\,\tau^2,
\qquad
e = \hat\theta_t^{+} - \theta_g(\text{now})
$$

Then apply a smooth deadband, which suppresses jitter without creating a discontinuity:

$$
e_{db} = \operatorname{sign}(e)\,\max(|e| - \delta,\,0)
$$

### 4.2 Velocity (and acceleration) feedforward

$$
\omega_{ff} = k_{ff}\left(\hat\omega_t + \hat\alpha_t\,\tau\right) - \omega_{base}^{\perp}
$$

- **k_ff** ∈ [0.8, 1.0]. Start at 0.9 and increase toward 1 once the estimator is trusted.
- **ω_base^⊥** is the vehicle body rate projected onto the axis. Include it only if the inner loop does *not* already stabilize in the world frame.
- **Feedforward fading:** use k_ff · c, where the confidence c = exp(−P_ωω / σ²_ref) ∈ (0, 1]. Feedforward then fades automatically when velocity uncertainty is high, such as after reacquisition.

### 4.3 PID (discrete, sample time T_s)

$$
\begin{aligned}
P_k &= K_p\, e_{db,k}\\[2pt]
D_k &= \frac{T_f}{T_f + T_s}\,D_{k-1} + \frac{K_d}{T_f + T_s}\,(e_{k} - e_{k-1}) \quad\text{(filtered, } T_f = K_d/(K_pN)\text{)}\\[2pt]
\omega_{raw,k} &= \omega_{ff,k} + P_k + I_{k-1} + D_k
\end{aligned}
$$

Because feedforward carries the target velocity, the PID only corrects the **residual** error. This allows low gains and therefore smooth motion.

### 4.4 Braking-curve velocity limit (overshoot prevention)

The commanded *closing* rate must be one the gimbal can stop from within the remaining error under a_max:

$$
\omega_{close,max} = \sqrt{2\,a_{max}\,|e|}
$$

$$
\omega_{rel} = \omega_{raw} - \omega_{ff}, \qquad
\omega_{rel} \leftarrow \operatorname{clamp}\big(\omega_{rel},\,-\omega_{close,max},\,\omega_{close,max}\big)
$$

$$
\omega_{lim} = \operatorname{clamp}\big(\omega_{ff} + \omega_{rel},\,-\omega_{max},\,\omega_{max}\big)
$$

### 4.5 Acceleration limiting (and optional jerk limiting)

Accel-only slew limit:

$$
\omega_{cmd,k} = \omega_{cmd,k-1} + \operatorname{clamp}\big(\omega_{lim} - \omega_{cmd,k-1},\,-a_{max}T_s,\,a_{max}T_s\big)
$$

For cinematic smoothness, add a jerk limit:

$$
a_{des} = \frac{\omega_{lim}-\omega_{cmd,k-1}}{T_s},\quad
a_k = a_{k-1} + \operatorname{clamp}(a_{des}-a_{k-1},\,\pm j_{max}T_s),\quad
a_k \leftarrow \operatorname{clamp}(a_k,\pm a_{max})
$$

$$
\omega_{cmd,k} = \omega_{cmd,k-1} + a_k T_s
$$

### 4.6 Anti-windup (back-calculation)

$$
I_k = I_{k-1} + T_s\Big[K_i\, e_{db,k} + K_{aw}\big(\omega_{cmd,k} - \omega_{raw,k}\big)\Big],
\qquad K_{aw} \approx \frac{1}{\sqrt{T_i T_d}}\ \text{or}\ \frac{K_i}{K_p}
$$

Also clamp |I| ≤ I_max, and freeze integration while the target is lost.

---

## 5. Reference Implementation (Python, one axis)

```python
import numpy as np

class AxisTracker:
    """Gimbal-axis visual tracking: KF target estimate + PID + velocity FF + accel/jerk limiting."""

    def __init__(self, Ts=0.005, Kp=4.0, Ki=0.8, Kd=0.15, N=10.0, kff=0.95,
                 w_max=np.radians(180), a_max=np.radians(400), j_max=np.radians(4000),
                 deadband=np.radians(0.15), q=np.radians(50)**2, sigma_z=np.radians(0.1),
                 tau_act=0.015, I_max=np.radians(30)):
        self.Ts, self.Kp, self.Ki, self.Kd, self.N, self.kff = Ts, Kp, Ki, Kd, N, kff
        self.w_max, self.a_max, self.j_max, self.db = w_max, a_max, j_max, deadband
        self.q, self.R, self.tau_act, self.I_max = q, sigma_z**2, tau_act, I_max
        self.Kaw = Ki / Kp
        self.x = np.zeros(3); self.P = np.diag([1e-2, 1.0, 10.0])
        self.t_x = None                         # time the state x refers to
        self.I = self.D = self.e_prev = 0.0
        self.w_cmd = self.a_cmd = 0.0
        self.valid = False; self.t_last_meas = -np.inf

    # ---------- estimator ----------
    @staticmethod
    def _FQ(dt, q):
        F = np.array([[1, dt, 0.5*dt*dt], [0, 1, dt], [0, 0, 1]])
        Q = q*np.array([[dt**5/20, dt**4/8, dt**3/6],
                        [dt**4/8,  dt**3/3, dt**2/2],
                        [dt**3/6,  dt**2/2, dt]])
        return F, Q

    def measure(self, pixel_err_rad, gimbal_angle_at_img, t_img):
        z = gimbal_angle_at_img + pixel_err_rad
        if not self.valid:                      # (re)acquisition
            self.x[:] = [z, 0.0, 0.0]; self.P = np.diag([self.R, 1.0, 10.0])
            self.t_x, self.valid = t_img, True; self.t_last_meas = t_img
            return
        F, Q = self._FQ(max(t_img - self.t_x, 1e-4), self.q)
        x, P = F @ self.x, F @ self.P @ F.T + Q
        S = P[0, 0] + self.R; y = z - x[0]
        if y*y/S > 6.63:                        # chi-square gate, 99%
            return
        K = P[:, 0] / S
        self.x = x + K*y
        self.P = (np.eye(3) - np.outer(K, [1, 0, 0])) @ P
        self.t_x = self.t_last_meas = t_img

    # ---------- controller ----------
    def update(self, gimbal_angle_now, t_now, w_base=0.0, lost_timeout=0.5):
        Ts = self.Ts
        if not self.valid or t_now - self.t_last_meas > lost_timeout:
            self.valid = False                  # lost: decay smoothly to rest, freeze I
            target_w, e = 0.0, 0.0
            w_ff = 0.0
        else:
            tau = (t_now - self.t_x) + self.tau_act
            th, w, a = self.x
            th_pred = th + w*tau + 0.5*a*tau*tau
            e = np.arctan2(np.sin(th_pred - gimbal_angle_now),
                           np.cos(th_pred - gimbal_angle_now))   # wrap
            conf = np.exp(-self.P[1, 1] / np.radians(20)**2)
            w_ff = self.kff*conf*(w + a*tau) - w_base

        e_db = np.sign(e)*max(abs(e) - self.db, 0.0)

        # PID (filtered derivative on error)
        P = self.Kp*e_db
        Tf = self.Kd/(self.Kp*self.N) if self.Kd > 0 else 0.0
        self.D = (Tf*self.D + self.Kd*(e_db - self.e_prev)/1.0) / (Tf + Ts) * 1.0 if self.Kd > 0 else 0.0
        self.e_prev = e_db
        w_raw = w_ff + P + self.I + self.D

        # braking-curve limit on closing rate, then absolute rate limit
        w_close = np.sqrt(2*self.a_max*abs(e))
        w_rel = np.clip(w_raw - w_ff, -w_close, w_close)
        w_lim = np.clip(w_ff + w_rel, -self.w_max, self.w_max)

        # jerk-limited acceleration limiting
        a_des = (w_lim - self.w_cmd)/Ts
        self.a_cmd += np.clip(a_des - self.a_cmd, -self.j_max*Ts, self.j_max*Ts)
        self.a_cmd = np.clip(self.a_cmd, -self.a_max, self.a_max)
        # avoid jerk-limit overshoot of target rate
        w_next = self.w_cmd + self.a_cmd*Ts
        if (w_next - w_lim)*(self.w_cmd - w_lim) < 0:
            w_next, self.a_cmd = w_lim, 0.0
        self.w_cmd = w_next

        # back-calculation anti-windup
        if self.valid:
            self.I += Ts*(self.Ki*e_db + self.Kaw*(self.w_cmd - w_raw))
            self.I = np.clip(self.I, -self.I_max, self.I_max)
        return self.w_cmd          # rad/s → inner rate loop
```

**Usage:**
- Call `measure()` on every detection. Pass the gimbal enco