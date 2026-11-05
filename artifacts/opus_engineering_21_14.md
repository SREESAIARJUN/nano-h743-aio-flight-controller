# 3D Rigid-Body Physics Simulator: 3-Axis Dynamics, Motor Torque Curves, and Gyro Drift

I don't know what "Cycle 21" refers to in your project, so I treated it as a version label. This is a complete, self-contained simulator. The only required dependency is `numpy`. `matplotlib` is optional and only used for plots.

## What It Models

| Subsystem | Model |
|---|---|
| **Rigid body** | Full 6-DOF motion: quaternion attitude, a full inertia tensor, gyroscopic coupling (Euler's equations with internal wheel momentum), and translation under force. Integrated with RK4. |
| **Motors** | 3 reaction wheels driven by DC/BLDC motors with a back-EMF voltage limit, a current limit, and viscous plus Coulomb friction. This produces a realistic torque–speed curve: flat at the current limit, then falling linearly with speed. |
| **Gyro** | Scale-factor error, axis misalignment, constant bias, angle random walk (ARW), rate random walk (RRW), Gauss-Markov bias instability, temperature-dependent bias, and quantization. Sampled at a discrete rate. |
| **GNC** | Attitude is propagated from gyro data only, so drift is visible. An optional star tracker periodically corrects attitude and estimates gyro bias. A PD controller handles slews and adds gyroscopic feed-forward. |
| **Diagnostics** | Total system angular momentum in the inertial frame, used to check the integrator. Also: true vs. estimated attitude error, wheel saturation, and power draw. |

## Source Code: `rigidsim.py`

```python
#!/usr/bin/env python3
"""
rigidsim.py — 3D rigid-body physics simulator (Cycle 21)

Models:
  * 6-DOF rigid body: quaternion attitude, full inertia tensor, Euler's
    equations with internal reaction-wheel momentum, translational motion.
  * Reaction-wheel motors with torque-speed curves (back-EMF voltage limit,
    current limit, viscous + Coulomb friction).
  * MEMS/FOG-style gyro error model: bias, scale factor, misalignment,
    ARW, RRW, Gauss-Markov bias instability, temperature sensitivity,
    quantization.
  * Gyro-propagated attitude estimator (drifts) with optional star-tracker
    corrections, and a PD attitude controller.

Conventions:
  * Quaternions are scalar-first [w, x, y, z], mapping body -> inertial.
  * Angular velocity omega is expressed in the body frame (rad/s).
  * SI units throughout.

Usage:
  python rigidsim.py                      # default scenario, with plots
  python rigidsim.py --duration 600 --no-startracker
  python rigidsim.py --csv out.csv --no-plot
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass, field

import numpy as np


# =============================================================================
# Quaternion / rotation utilities
# =============================================================================

def qmul(p: np.ndarray, q: np.ndarray) -> np.ndarray:
    """Hamilton product p ⊗ q."""
    pw, px, py, pz = p
    qw, qx, qy, qz = q
    return np.array([
        pw * qw - px * qx - py * qy - pz * qz,
        pw * qx + px * qw + py * qz - pz * qy,
        pw * qy - px * qz + py * qw + pz * qx,
        pw * qz + px * qy - py * qx + pz * qw,
    ])


def qconj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def qnormalize(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    return q / n if n > 0 else np.array([1.0, 0.0, 0.0, 0.0])


def q_from_axis_angle(axis, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n < 1e-12 or abs(angle) < 1e-15:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = axis / n
    s = math.sin(angle / 2.0)
    return np.array([math.cos(angle / 2.0), *(axis * s)])


def q_from_rotvec(rv: np.ndarray) -> np.ndarray:
    ang = np.linalg.norm(rv)
    return q_from_axis_angle(rv, ang) if ang > 1e-15 else np.array([1.0, 0, 0, 0])


def q_to_rotvec(q: np.ndarray) -> np.ndarray:
    """Rotation vector (axis * angle). Uses the shortest-path hemisphere."""
    q = qnormalize(q)
    if q[0] < 0:
        q = -q
    v = q[1:]
    s = np.linalg.norm(v)
    if s < 1e-12:
        return 2.0 * v  # small-angle limit
    angle = 2.0 * math.atan2(s, q[0])
    return v / s * angle


def q_to_dcm(q: np.ndarray) -> np.ndarray:
    """Direction cosine matrix body -> inertial."""
    w, x, y, z = qnormalize(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ])


def q_to_euler_deg(q: np.ndarray) -> np.ndarray:
    """ZYX (yaw-pitch-roll) Euler angles in degrees, returned as [roll, pitch, yaw]."""
    w, x, y, z = qnormalize(q)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.degrees([roll, pitch, yaw])


def quat_derivative(q: np.ndarray, omega_body: np.ndarray) -> np.ndarray:
    return 0.5 * qmul(q, np.array([0.0, *omega_body]))


# =============================================================================
# Motor model with torque-speed curve
# =============================================================================

@dataclass
class MotorParams:
    Kt: float = 0.025          # torque constant [N·m/A]
    Ke: float = 0.025          # back-EMF constant [V·s/rad] (== Kt in SI)
    R: float = 1.2             # winding resistance [Ω]
    V_bus: float = 24.0        # supply voltage [V]
    I_max: float = 2.0         # driver current limit [A]
    c_visc: float = 2.0e-6     # viscous friction [N·m·s/rad]
    c_coul: float = 1.0e-4     # Coulomb friction [N·m]
    omega_eps: float = 0.5     # friction smoothing speed [rad/s]
    J_rotor: float = 2.0e-3    # wheel + rotor inertia about spin axis [kg·m²]


class Motor:
    """
    Current-mode-driven brushless DC motor.

    The deliverable current is bounded by
      * the driver current limit:  |i| <= I_max
      * the bus voltage vs back-EMF:  (-V - Ke·Ω)/R <= i <= (V - Ke·Ω)/R

    Together these produce the classic torque-speed curve: constant torque
    up to the corner speed, then a linear decline to zero at no-load speed.
    """

    def __init__(self, p: MotorParams):
        self.p = p

    def current_bounds(self, omega: float) -> tuple[float, float]:
        p = self.p
        lo_v = (-p.V_bus - p.Ke * omega) / p.R
        hi_v = (p.V_bus - p.Ke * omega) / p.R
        return lo_v, hi_v

    def friction(self, omega: float) -> float:
        p = self.p
        return p.c_visc * omega + p.c_coul * math.tanh(omega / p.omega_eps)

    def evaluate(self, torque_cmd: float,