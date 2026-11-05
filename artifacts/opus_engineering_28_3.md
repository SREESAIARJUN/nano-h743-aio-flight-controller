# Autonomous Gimbal Tracking Controller: PID + Velocity Feedforward + Acceleration Limiting (Cycle 28)

The controller is applied independently per axis (pan/yaw and tilt/pitch), with optional roll stabilization. It outputs a **rate command** to the gimbal's inner rate loop, which is the standard architecture for brushless gimbals.

---

## 1. Measurement Model: Pixel Error to Angular Error

For a pinhole camera with image width $W$, height $H$, and horizontal FOV $\text{HFOV}$, the focal length in pixels is:

$$f_x = \frac{W/2}{\tan(\text{HFOV}/2)}, \qquad f_y = \frac{H/2}{\tan(\text{VFOV}/2)}$$

For a detected target centroid $(u, v)$ with principal point $(c_x, c_y)$, the angular errors are:

$$e_\psi = \arctan\!\left(\frac{u - c_x}{f_x}\right), \qquad e_\theta = -\arctan\!\left(\frac{v - c_y}{f_y}\right)$$

Using `atan` rather than a linear pixel scale keeps the gain consistent across the frame and across zoom levels. When zoom changes, update $f_x$ and $f_y$ every frame.

**Inertial target bearing.** This decouples target motion from gimbal motion:

$$\psi_T[k] = \psi_G(t_{\text{img}}) + e_\psi[k]$$

Here $\psi_G(t_{\text{img}})$ is the gimbal encoder or IMU angle **at the image capture timestamp**, not at processing time. This is essential for latency correctness.

---

## 2. Target State Estimation

Track the target bearing per axis with a constant-velocity Kalman filter, using state $\mathbf{x} = [\psi_T, \ \dot\psi_T]^\top$.

**Predict:**

$$\mathbf{x}^- = F\mathbf{x}, \quad F = \begin{bmatrix}1 & \Delta t\\ 0 & 1\end{bmatrix}, \quad P^- = FPF^\top + Q$$

$$Q = q\begin{bmatrix}\Delta t^3/3 & \Delta t^2/2\\ \Delta t^2/2 & \Delta t\end{bmatrix}$$

**Update:** use $z = \psi_T[k]$ with $H = [1\ 0]$ and measurement noise $R = \sigma_{\text{det}}^2 / f_x^2$, where $\sigma_{\text{det}}$ is the detector jitter in pixels.

**Latency compensation.** Propagate the estimate forward by the total pipeline latency $\tau$ (capture → inference → command):

$$\hat\psi_T(t_{\text{now}}) = \hat\psi_T + \hat{\dot\psi}_T\,\tau$$

**Tuning:**
- Increasing $q$ gives faster response to maneuvers but noisier velocity.
- Decreasing $q$ gives smoother tracking but more lag.

---

## 3. Control Law

Define the predicted tracking error:

$$e[k] = \hat\psi_T(t_{\text{now}}) - \psi_G[k] \quad (\text{wrapped to } [-\pi, \pi])$$

The rate command combines feedforward and PID terms:

$$\boxed{\omega_{\text{raw}} = \underbrace{K_{ff}\,\hat{\dot\psi}_T}_{\text{feedforward}} + K_p\,e + K_i\!\int e\,dt + K_d\,\dot e_f}$$

**Derivative on filtered error.** Use a first-order low-pass with cutoff $N$ (rad/s):

$$\dot e_f[k] = \frac{N\,(e[k]-e[k-1]) + \dot e_f[k-1]}{1 + N\Delta t}$$

Because the feedforward term already supplies the target velocity, $K_d$ is often small or zero. Feedforward does most of the "follow" work; the PID corrects only residual error.

**Smooth deadband.** This prevents micro-jitter when the target is near center:

$$e_{db} = \operatorname{sign}(e)\,\max(0, |e| - \delta) \quad \text{or the soft form} \quad e\left(1 - e^{-(e/\delta)^2}\right)$$

Apply the deadband to the P and I paths only, never to the feedforward.

---

## 4. Velocity and Acceleration Limiting (Rate Shaping)

The limits are applied in order.

**1. Velocity clamp:**

$$\omega_{\text{sat}} = \operatorname{clip}(\omega_{\text{raw}}, -\omega_{\max}, \omega_{\max})$$

**2. Acceleration limit (slew rate):**

$$\omega_{\text{cmd}}[k] = \omega_{\text{cmd}}[k-1] + \operatorname{clip}\!\big(\omega_{\text{sat}} - \omega_{\text{cmd}}[k-1],\ -a_{\max}\Delta t,\ a_{\max}\Delta t\big)$$

**3. Optional jerk limit**, for cinematic smoothness: apply the same slew structure to acceleration $a[k]$ with bound $j_{\max}$, then integrate.

**Anti-windup via back-calculation.** This keeps the integrator consistent when any limiter is active:

$$I[k] = I[k-1] + \Big(K_i\,e[k] + K_{aw}\big(\omega_{\text{cmd}}[k] - \omega_{\text{raw}}[k]\big)\Big)\Delta t, \qquad K_{aw} \approx \frac{1}{\sqrt{T_i T_d}} \text{ or } \frac{K_i}{K_p}$$

**Braking-aware approach.** This avoids overshoot caused by the acceleration limit. Cap the P-driven speed so the gimbal can still stop in time:

$$|\omega_{\text{fb}}| \le \sqrt{2\,a_{\max}\,|e|}$$

This is the time-optimal braking curve. It produces a smooth, overshoot-free arrival, effectively a nonlinear P near large errors.

---

## 5. Target-Loss Behavior (Coast and Recover)

| State | Condition | Action |
|---|---|---|
| **TRACK** | Detection confidence ≥ threshold | Full control law |
| **COAST** | Lost < $T_c$ (≈0.5–1 s) | KF predict only; $\omega = K_{ff}\hat{\dot\psi}_T e^{-t/\tau_c}$; freeze integrator |
| **HOLD/SEARCH** | Lost > $T_c$ | Decelerate to 0 at $a_{\max}$, then hold or run a search pattern |
| **REACQUIRE** | New detection | Gate by Mahalanobis distance; ramp $K_{ff}$ from 0 to 1 over ~0.3 s to avoid a snap |

---

## 6. Reference Implementation (Python)

```python
import math

class AxisTracker:
    def __init__(self, kp=4.0, ki=0.5, kd=0.05, kff=1.0, kaw=2.0,
                 w_max=math.radians(180), a_max=math.radians(400),
                 deadband=math.radians(0.3), d_cutoff=30.0,
                 q=math.radians(50)**2, r=math.radians(0.2)**2, latency=0.08):
        self.kp, self.ki, self.kd, self.kff, self.kaw = kp, ki, kd, kff, kaw
        self.w_max, self.a_max, self.db, self.N = w_max, a_max, deadband, d_cutoff
        self.q, self.r, self.tau = q, r, latency
        # KF state
        self.x = [0.0, 0.0]; self.P = [[1, 0], [0, 1]]; self.init = False
        # PID state
        self.I = 0.0; self.e_prev = 0.0; self.de_f = 0.0; self.w_cmd = 0.0

    @staticmethod
    def wrap(a): return (a + math.pi) % (2 * math.pi) - math.pi

    def kf_predict(self, dt):
        x, P, q = self.x, self.P, self.q
        x[0] += x[1] * dt
        p00 = P[0][0] + dt*(P[1][0]+P[0][1]) + dt*dt*P[1][1] + q*dt**3/3
        p01 = P[0][1] + dt*P[1][1] + q*dt**2/2
        p11 = P[1][1] + q*dt
        self.P = [[p00, p01], [p01, p11]]

    def kf_update(self, z):
        if not self.init:
            self.x = [z, 0.0]; self.init = True; return
        P = self.P
        y = self.wrap(z - self.x[0]); S = P[0][0] + self.r
        k0, k1 = P[0][0]/S, P[1][0]/S
        self.x[0] += k0*y; self.x[1] += k1*y
        self.P = [[(1-k0)*P[0][0], (1-k0)*P[0][1]],
                  [P[1][0]-k1*P[0][0], P[1][1]-k1*P[0][1]]]

    def step(self, dt, gimbal_angle, meas_bearing=None):
        """meas_bearing = gimbal_angle_at_capture + atan(pixel_err/f), or None if lost."""
        self.kf_predict(dt)
        if meas_bearing is not None:
            self.kf_update(meas_bearing)
        tgt = self.x[0] + self.x[1]*self.tau          # latency compensation
        e = self.wrap(tgt - gimbal_angle)

        e_db = math.copysign(max(0.0, abs(e) - self.db), e)
        self.de_f = (self.N*(e - self.e_prev) + self.de_f) / (1 + self.N*dt)
        self.e_prev = e

        fb = self.kp*e_db + self.I + self.kd*self.de_f
        fb_cap = math.sqrt(2*self.a_max*abs(e))       # braking-curve cap
        fb = max(-fb_cap, min(fb_cap, fb))
        w_raw = self.kff*self.x[1] + fb

        w_sat = max(-self.w_max, min(self.w_max, w_raw))
        dw = max(-self.a_max*dt, min(self.a_max*dt, w_sat - self.w_cmd))
        self.w_cmd += dw

        if meas_bearing is not None:                  # freeze I while coasting
            self.I += (self.ki*e_db + self.kaw*(self.w_cmd - w_raw)) * dt
        return self.w_cmd
```

**Usage per frame/tick:**

```python
bearing = enc_at_capture + math.atan((u - cx) / fx)
w_pan = pan.step(dt, enc_now, bearing)
```

Do the same for tilt using $v$, $f_y$, and a sign flip.

---

## 7. Tuning Procedure

1. **Inner rate loop first.** Verify that the gimbal tracks rate commands with bandwidth ≥ 5× the outer loop.
2. **Measure latency $\tau$.** Flash an LED or move the gimbal with a step and timestamp when the pixels respond.
3. **Feedforward.** Set $K_p = K_i = K_d = 0$ and $K_{ff} = 1$. Track a constant-velocity target; the residual should be a constant offset.
4. **$K_p$.** Raise it until the step response is crisp. Outer bandwidth should be about $K_p$ rad/s, kept ≤ 1/(3τ). For τ = 80 ms, that means $K_p \lesssim 4$.
5. **$K_i$.** Set $K_i \approx K_p/10$ to remove steady-state bias from friction, cable drag, or imbalance.
6. **$K_d$.** Add only if overshoot remains; keep $N$ below the detector noise frequency.
7. **Limits.**
   - $a_{\max}$: start at about 50–70% of motor capability. Lower it for cinematic footage (e.g., 60–120°/s²); raise it for sports or drone tracking.
   - $\omega_{\max}$: set from motion-blur tolerance, roughly $\omega_{\max} \approx \text{blur}_{px}\cdot \text{IFOV} / t_{\text{exposure}}$.
8. **Kalman $q$.** Adjust while watching $\hat{\dot\psi}_T$: it should be smooth but respond within ~0.2 s to real maneuvers.

---

## 8. Stability Notes

- **Delay margin.** With the outer loop roughly an integrator times $K_p$, the phase margin is about $90^\circ - K_p\tau\cdot 57.3^\circ$. Keep $K_p\tau < 0.5$ for ≥60° margin.
- **Feedforward and stability.** The feedforward path does not affect closed-loop stability, since it acts on the estimated target state rather than the loop error. However, an over-estimated $\hat{\dot\psi}_T$ (high $q$) causes visible overshoot, which is the main tradeoff between smoothness and lag.
- **Gimbal lock.** Near ±90° tilt, pan authority scales by $1/\cos\theta$. Compute errors in a world-frame unit vector and convert to joint rates with the gimbal Jacobian, or clamp tilt to about ±80°.