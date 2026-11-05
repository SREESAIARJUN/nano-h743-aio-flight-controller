# 3-Axis Rigid-Body Spacecraft / Gimbal Physics Simulator (Cycle 10)

I wasn't sure what "Cycle 10" refers to in your project, so I treated it as a revision label. The code below is a complete, self-contained simulator. It requires only `numpy`. `matplotlib` is optional and used only for plots.

## What it models

| Subsystem | Model |
|---|---|
| **Rigid body** | Full 3-axis Euler equations with a non-diagonal inertia tensor and gyroscopic coupling (ω × H). Attitude uses scalar-first quaternions with RK4 integration. |
| **Actuators** | 4 reaction wheels in a pyramid layout. Each wheel is a brushless DC motor with back-EMF, a bus voltage limit, a current limit, viscous and Coulomb friction, and a speed limit. These produce a realistic torque-speed curve. |
| **Gyro** | Angle random walk, rate random walk, Gauss-Markov bias instability, temperature drift, scale-factor error, axis misalignment, quantization, and saturation. |
| **Estimator** | Gyro-propagated attitude with an optional low-rate star tracker that corrects attitude and estimates bias. Turning it off lets you watch pure gyro drift. |
| **Controller** | Quaternion PD controller. Torque is allocated to the wheels with a pseudo-inverse, then turned into motor voltages by inverting the motor model. |
| **Disturbances** | Constant torque plus an orbital-period sinusoidal torque. |

```python
#!/usr/bin/env python3
"""
rigid3d_sim.py  --  Cycle 10
3-axis rigid-body physics simulator with reaction-wheel motor torque curves
and gyro drift modeling.

Conventions
-----------
* Quaternion q = [qw, qx, qy, qz], scalar first, rotates BODY -> INERTIAL.
* omega is the body angular rate expressed in the body frame [rad/s].
* Wheel momentum h_i = Jw * (Omega_i + a_i . omega) is the absolute axial momentum.
* The body inertia J excludes the wheels' axial inertia
  (standard formulation: H = J w + A h).

Usage
-----
    python rigid3d_sim.py                     # run with plots
    python rigid3d_sim.py --no-startracker    # pure gyro propagation -> watch drift
    python rigid3d_sim.py --duration 900 --csv out.csv --no-plot
"""

import argparse
import math
import sys
import numpy as np

# ----------------------------------------------------------------------------
# Quaternion / rotation utilities
# ----------------------------------------------------------------------------

def quat_mul(p, q):
    pw, px, py, pz = p
    qw, qx, qy, qz = q
    return np.array([
        pw*qw - px*qx - py*qy - pz*qz,
        pw*qx + px*qw + py*qz - pz*qy,
        pw*qy - px*qz + py*qw + pz*qx,
        pw*qz + px*qy - py*qx + pz*qw,
    ])

def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])

def quat_norm(q):
    n = np.linalg.norm(q)
    return q / n if n > 0 else np.array([1.0, 0, 0, 0])

def quat_from_axis_angle(axis, angle):
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    s = math.sin(angle / 2.0)
    return np.array([math.cos(angle / 2.0), *(axis * s)])

def quat_from_rotvec(rv):
    ang = np.linalg.norm(rv)
    if ang < 1e-12:
        return quat_norm(np.array([1.0, *(0.5 * rv)]))
    return quat_from_axis_angle(rv / ang, ang)

def quat_to_rotvec(q):
    """Small/large-angle safe conversion to rotation vector (shortest path)."""
    q = quat_norm(q)
    if q[0] < 0:
        q = -q
    v = q[1:]
    s = np.linalg.norm(v)
    if s < 1e-12:
        return 2.0 * v
    ang = 2.0 * math.atan2(s, q[0])
    return v / s * ang

def quat_to_dcm(q):
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ])

def quat_to_euler_deg(q):
    """ZYX (yaw-pitch-roll) Euler angles in degrees."""
    w, x, y, z = q
    roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return np.degrees([roll, pitch, yaw])

def angle_between_deg(q1, q2):
    return math.degrees(np.linalg.norm(quat_to_rotvec(quat_mul(quat_conj(q1), q2))))

# ----------------------------------------------------------------------------
# Motor / reaction wheel array with torque-speed curves
# ----------------------------------------------------------------------------

class ReactionWheelArray:
    """
    N brushless DC reaction wheels, vectorized.

    Electrical model (quasi-static, L/R << dt):
        i     = clip((V - Ke*Omega) / R, -Imax, Imax)
        tau_e = Kt * i
    Friction:
        tau_f = b*Omega + c*tanh(Omega/omega_c)       (viscous + smoothed Coulomb)
    Speed limit: the driver cuts drive torque that would exceed |Omega| > Omega_max.

    The resulting max-torque envelope versus speed is the classic DC-motor
    torque-speed curve: flat (current-limited) up to the corner speed, then
    linearly falling (back-EMF limited) to the no-load speed Vbus/Ke.
    """

    def __init__(self, axes, Jw=2.0e-3, Kt=0.03, R=1.5, Vbus=24.0, Imax=2.0,
                 b_visc=2.0e-6, c_coul=4.0e-4, omega_c=0.5, omega_max_rpm=6000.0):
        self.A = np.array(axes, float).T          # 3 x N, columns = spin axes
        self.A /= np.linalg.norm(self.A, axis=0)
        self.N = self.A.shape[1]
        self.A_pinv = np.linalg.pinv(self.A)      # N x 3
        self.Jw = Jw
        self.Kt = Kt
        self.Ke = Kt                              # SI units: Ke [V/(rad/s)] == Kt [Nm/A]
        self.R = R
        self.Vbus = Vbus
        self.Imax = Imax
        self.b = b_visc
        self.c = c_coul
        self.wc = omega_c
        self.Omega_max = omega_max_rpm * 2 * math.pi / 60.0

    def friction(self, Omega):
        return self.b * Omega + self.c * np.tanh(Omega / self.wc)

    def electrical_torque(self, V, Omega):
        i = np.clip((V - self.Ke * Omega) / self.R, -self.Imax, self.Imax)
        tau = self.Kt * i
        # Driver speed limiting: no further acceleration past Omega_max
        over = (np.abs(Omega) >= self.Omega_max) & (np.sign(tau) == np.sign(Omega))
        tau = np.where(over, 0.0, tau)
        i = np.where(over, 0.0, i)
        return tau, i

    def voltage_for_torque(self, tau_cmd, Omega, friction_ff=True):
        """Invert the motor model: voltage needed to produce the commanded net wheel torque."""
        tau_e = tau_cmd + (self.friction(Omega) if friction_ff else 0.0)
        i = np.clip(tau_e / self.Kt, -self.Imax, self.Imax)
        V = i * self.R + self.Ke * Omega
        return np.clip(V, -self.Vbus, self.Vbus)

    def torque_speed_curve(self, n=400):
        """Max motoring torque available at full +Vbus versus speed (for plotting)."""
        Om = np.linspace(0, self.Vbus / self.Ke * 1.05, n)
        tau, _ = self.electrical_torque(np.full(n, self.Vbus), Om)
        return Om, tau, tau - self.friction(Om)

# ----------------------------------------------------------------------------
# Gyro model with drift
# ----------------------------------------------------------------------------

DEG = math.p