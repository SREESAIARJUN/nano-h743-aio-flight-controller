# 3-Axis Rigid Body Physics Simulator: Motor Torque Curves and Gyro Drift

I'm not sure what "Cycle 28" refers to. It may be a revision number or a project-specific iteration. Below is a complete, self-contained simulator. If Cycle 28 has particular parameters or requirements, share them and I'll adjust the defaults.

The simulator models:

- **Rigid body dynamics**: full 3-axis Euler equations with gyroscopic coupling, quaternion kinematics, and an RK4 integrator.
- **Actuators**: three reaction wheels driven by DC motors. Each motor has back-EMF, current and voltage saturation, Coulomb and viscous friction, and an optional tabulated torque-speed envelope.
- **Gyro errors**: angle random walk (ARW), rate random walk (RRW), Gauss-Markov bias instability, temperature-dependent bias, scale factor, axis misalignment, quantization and saturation.
- **Attitude estimation**: gyro propagation plus an optional star-tracker correction with bias estimation. You can disable the star tracker to observe pure drift.
- **Control**: a quaternion PD controller with a motor voltage inversion step.
- **Diagnostics**: an angular momentum conservation check, CSV export and plots.

Requirements: Python 3.9+ and `numpy`. `matplotlib` is optional and only used for plots.

```python
#!/usr/bin/env python3
"""
rigidbody3d_sim.py  —  3-axis rigid body simulator (Cycle 28)

Models:
  * Rigid body rotational dynamics (Euler eqns + reaction wheel momentum)
  * DC-motor-driven reaction wheels with torque-speed curves
  * MEMS/FOG-style gyro with drift (ARW, RRW, bias instability, temperature)
  * Gyro-propagated attitude estimator with optional star-tracker updates
  * Quaternion PD attitude controller

Conventions:
  * Quaternions are scalar-first [w, x, y, z], Hamilton product,
    rotating BODY -> INERTIAL.
  * Angular velocity w is expressed in the BODY frame (rad/s).
  * Wheel speeds are RELATIVE to the body (rad/s).

Usage:
  python rigidbody3d_sim.py                  # default 120 s slew + hold
  python rigidbody3d_sim.py --no-startracker # observe pure gyro drift
  python rigidbody3d_sim.py --tend 600 --csv out.csv --no-plot
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np


# =============================================================================
# Quaternion utilities
# =============================================================================
def q_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def q_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def q_norm(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    return q / n if n > 0 else np.array([1.0, 0.0, 0.0, 0.0])


def q_from_axis_angle(axis, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    s = math.sin(angle / 2.0)
    return np.array([math.cos(angle / 2.0), *(axis * s)])


def q_from_rotvec(rv: np.ndarray) -> np.ndarray:
    ang = np.linalg.norm(rv)
    if ang < 1e-12:
        return q_norm(np.array([1.0, *(0.5 * rv)]))
    return q_from_axis_angle(rv / ang, ang)


def q_to_rotvec(q: np.ndarray) -> np.ndarray:
    q = q_norm(q)
    if q[0] < 0:
        q = -q  # shortest rotation
    v = q[1:]
    s = np.linalg.norm(v)
    if s < 1e-12:
        return 2.0 * v
    ang = 2.0 * math.atan2(s, q[0])
    return v / s * ang


def q_to_euler_deg(q: np.ndarray) -> np.ndarray:
    """ZYX (yaw-pitch-roll) Euler angles in degrees, returned as [roll, pitch, yaw]."""
    w, x, y, z = q_norm(q)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.degrees([roll, pitch, yaw])


def q_angle_deg(q: np.ndarray) -> float:
    """Total rotation angle of a quaternion (deg)."""
    return float(np.degrees(np.linalg.norm(q_to_rotvec(q))))


# =============================================================================
# Motor / reaction wheel model
# =============================================================================
@dataclass
class DCMotorParams:
    kt: float = 0.025            # torque constant  [N*m/A]
    ke: float = 0.025            # back-EMF constant [V*s/rad]
    R: float = 1.2               # winding resistance [ohm]
    v_max: float = 24.0          # supply voltage limit [V]
    i_max: float = 4.0           # current limit [A]
    coulomb: float = 2.0e-4      # Coulomb friction [N*m]
    viscous: float = 1.0e-6      # viscous friction [N*m*s/rad]
    friction_eps: float = 0.5    # tanh smoothing for Coulomb friction [rad/s]
    wheel_J: float = 2.0e-3      # wheel spin inertia [kg*m^2]
    max_speed: float = 6000 * 2 * math.pi / 60  # hard limit [rad/s]
    # Optional torque-speed envelope (|speed| rad/s -> max |torque| N*m).
    # If provided, motor torque is additionally clipped to this curve.
    curve_speed: Optional[np.ndarray] = None
    curve_torque: Optional[np.ndarray] = None


class ReactionWheelMotor:
    """
    Brushed/BLDC motor in quasi-static electrical approximation
    (L/R time constant << control period):

        i      = clip((V - ke*w) / R, -i_max, i_max)
        tau_em = kt * i
        tau    = tau_em - coulomb*tanh(w/eps) - viscous*w
        tau    = clip(tau, -curve(|w|), +curve(|w|))     (if curve given)

    The natural (unlimited) torque-speed line is
        tau = kt*(V_max - ke*w)/R
    i.e. stall torque kt*Vmax/R falling to zero at no-load speed Vmax/ke.
    """

    def __init__(self, p: DCMotorParams):
        self.p = p
        self.voltage = 0.0
        self.last_current = 0.0
        self.last_torque = 0.0

    def torque(self, speed: float, voltage: Optional[float] = None) -> float:
        p = self.p
        v = self.voltage if voltage is None else voltage
        v = float(np.clip(v, -p.v_max, p.v_max))
        i = (v - p.ke * speed) / p.R
        i = float(np.clip(i, -p.i_max, p.i_max))
        tau = p.kt * i
        tau -= p.coulomb * math.tanh(speed / p.friction_eps) + p.viscous * speed
        if p.curve_speed is not None and p.curve_torque is not None:
            lim = float(np.interp(abs(speed), p.curve_speed, p.curve_torque))
            tau = float(np.clip(tau, -lim, lim))
        # Speed hard stop: do not accelerate beyond max speed
        if abs(speed) >= p.max_speed and np.sign(tau) == np.sign(speed):
            tau = 0.0
        self.last_current = i
        self.last_torque = tau
        return tau

    def voltage_for_torque(self, tau_des: float, speed: float) -> float:
        """Invert motor model (incl. friction feed-forward) to get voltage."""
        p = self.p
        tau_fric = p.coulomb * math.tanh(speed / p.friction_eps) + p.viscous * speed
        i_des = (tau_des + tau_fric) / p.kt
        i_des = float(np.clip(i_des, -p.i_max, p.i_max))
        v = i_des * p.R + p.ke * speed
        return float(np.clip(v, -p.v_max, p.v_max))

    def torque_speed_curve(self, n: int = 200):
        """Return (speed, max_torque) envelope at full voltage for plotting."""
        p = self.p
        w = np.linspace(0, p.v_max / p.ke, n)
        t = np.array([self.torque(wi, p.v_max) for wi in w])
        return w, t


# =============================================================================
# Gyro model with drift
# =============================================================================
@dataclass
class GyroParams:
    rate_hz: float = 100.0
    arw: float = math.radians(0.15) / 60.0          # rad/sqrt(s)  (0.15 deg/sqrt(hr))
    rrw: float = math.radians(0.5) / 3600.0 / 60.0  # rad/s/sqrt(s)
    bias_instability: float = math.radians(3.0) / 3600.0  # rad/s (3