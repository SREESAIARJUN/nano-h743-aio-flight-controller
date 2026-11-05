# 3-Axis Rigid-Body Attitude Simulator: Reaction-Wheel Motors and Gyro Drift

I don't know what "Cycle 29" refers to in your project, so I used it as a version tag. If it names a specific spec or set of requirements, share them and I'll adapt the code.

This is a single-file simulator. It needs NumPy, and Matplotlib only if you want plots. It models:

- **Rigid-body dynamics:** full Euler equations with a non-diagonal inertia tensor, gyroscopic coupling and stored wheel momentum.
- **Kinematics:** quaternion integration with RK4 and renormalization.
- **Motor torque curves:** DC/BLDC model with back-EMF, current limit, voltage saturation, viscous friction and Coulomb friction. Torque falls off with wheel speed.
- **Gyro errors:** constant bias, bias random walk, angle random walk, scale factor, misalignment, temperature sensitivity, quantization and sample rate.
- **Attitude estimation:** gyro integration with periodic star-tracker corrections and online bias estimation. You can disable the corrections to see raw drift.
- **Control:** quaternion PD controller with gyroscopic feedforward, plus a pseudo-inverse allocation over a 4-wheel pyramid.

```python
#!/usr/bin/env python3
"""
attitude_sim.py  --  3-axis rigid-body attitude simulator (Cycle 29)

Features
  * Rigid-body rotational dynamics (Euler equations + reaction-wheel momentum)
  * Quaternion kinematics, RK4 integration
  * DC/BLDC motor model with speed-dependent torque envelope
  * Gyro error model: bias, bias random walk, ARW, scale factor,
    misalignment, temperature drift, quantization
  * Gyro-propagated attitude estimator with optional star-tracker
    correction and bias estimation
  * Quaternion PD controller + wheel torque allocation

Conventions
  * Quaternions are scalar-first [w, x, y, z]
  * q maps body-frame vectors to inertial frame: v_I = R(q) v_B
  * Angular velocity w is body rate relative to inertial, in body frame

Usage
  python attitude_sim.py                    # run default scenario
  python attitude_sim.py --no-startracker   # pure gyro dead-reckoning (drift)
  python attitude_sim.py --plot --csv out.csv
"""

from __future__ import annotations
import argparse
import csv
import math
from dataclasses import dataclass, field

import numpy as np


# =============================================================================
# Quaternion utilities
# =============================================================================
def quat_mul(p: np.ndarray, q: np.ndarray) -> np.ndarray:
    pw, px, py, pz = p
    qw, qx, qy, qz = q
    return np.array([
        pw*qw - px*qx - py*qy - pz*qz,
        pw*qx + px*qw + py*qz - pz*qy,
        pw*qy - px*qz + py*qw + pz*qx,
        pw*qz + px*qy - py*qx + pz*qw,
    ])


def quat_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_normalize(q: np.ndarray) -> np.ndarray:
    return q / np.linalg.norm(q)


def quat_from_axis_angle(axis, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    s = math.sin(angle / 2.0)
    return np.array([math.cos(angle / 2.0), *(axis * s)])


def quat_exp(rotvec: np.ndarray) -> np.ndarray:
    """Quaternion for a rotation vector (axis * angle)."""
    angle = np.linalg.norm(rotvec)
    if angle < 1e-12:
        return quat_normalize(np.array([1.0, *(0.5 * rotvec)]))
    return quat_from_axis_angle(rotvec / angle, angle)


def quat_to_dcm(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ])


def quat_angle_between(q1: np.ndarray, q2: np.ndarray) -> float:
    """Smallest rotation angle (rad) between two attitudes."""
    d = abs(float(np.dot(q1, q2)))
    return 2.0 * math.acos(min(1.0, d))


def quat_to_euler321(q: np.ndarray) -> np.ndarray:
    """Yaw-pitch-roll (Z-Y-X) Euler angles, returned as [roll, pitch, yaw] (rad)."""
    w, x, y, z = q
    roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return np.array([roll, pitch, yaw])


def skew(v: np.ndarray) -> np.ndarray:
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])


# =============================================================================
# Motor model (brushed DC / BLDC equivalent)
# =============================================================================
@dataclass
class MotorParams:
    kt: float = 0.03          # torque constant  [N*m/A]
    ke: float = 0.03          # back-EMF const   [V*s/rad] (== kt in SI)
    R: float = 2.0            # winding resistance [ohm]
    v_max: float = 12.0       # bus voltage limit [V]
    i_max: float = 2.0        # current limit [A]
    b_visc: float = 2e-6      # viscous friction [N*m*s/rad]
    tau_coulomb: float = 1e-4 # Coulomb friction [N*m]
    coulomb_eps: float = 0.5  # smoothing speed for friction tanh [rad/s]


class Motor:
    """
    Steady electrical model (inductance neglected; L/R << control period):
        i   = clip((V - ke*omega)/R, +-i_max)
        tau = kt*i - b*omega - tau_c*tanh(omega/eps)
    The speed-dependent torque envelope (torque curve) emerges from
    back-EMF + voltage saturation + current limit.
    """

    def __init__(self, p: MotorParams):
        self.p = p

    def current(self, v_cmd: float, omega: float) -> float:
        p = self.p
        v = float(np.clip(v_cmd, -p.v_max, p.v_max))
        i = (v - p.ke * omega) / p.R
        return float(np.clip(i, -p.i_max, p.i_max))

    def friction(self, omega: float) -> float:
        p = self.p
        return p.b_visc * omega + p.tau_coulomb * math.tanh(omega / p.coulomb_eps)

    def shaft_torque(self, v_cmd: float, omega: float) -> float:
        return self.p.kt * self.current(v_cmd, omega) - self.friction(omega)

    def voltage_for_torque(self, tau_cmd: float, omega: float) -> float:
        """Inverse model with friction compensation; saturates at v_max."""
        p = self.p
        tau_em = tau_cmd + self.friction(omega)
        i = float(np.clip(tau_em / p.kt, -p.i_max, p.i_max))
        v = i * p.R + p.ke * omega
        return float(np.clip(v, -p.v_max, p.v_max))

    def torque_envelope(self, omega: float) -> tuple[float, float]:
        """(max positive, max negative) shaft torque available at speed omega."""
        return (self.shaft_torque(+self.p.v_max, omega),
                self.shaft_torque(-self.p.v_max, omega))

    def electrical_power(self, v_cmd: float, omega: float) -> float:
        v = float(np.clip(v_cmd, -self.p.v_max, self.p.v_max))
        return v * self.current(v_cmd, omega)

    @property
    def no_load_speed(self) -> float:
        return self.p.v_max / self.p.ke


# =============================================================================
# Gyro model
# =============================================================================
@dataclass
class GyroParams:
    bias0: np.ndarray = field(default_factory=lambda: np.deg2rad([0.02, -0.015, 0.01]))  # rad/s
    sigma_arw: float = np.deg2rad(0.15) / 60.0          # angle random walk [rad/sqrt(s)]  (0.15 deg/sqrt(h))
    sigma_rrw: float = np.deg2rad(0.5) / 3600.0 / 30.0  # rate random walk  [rad/s/sqrt(s)]
    scale_factor: np.ndarray = field(default_factory=lambda: np.array([300e-6, -200e-6, 150e-6]))
    misalign: np.ndarray = field(default_factory=lambda: np.deg2rad(np.array([
        [0.0, 0.05, -0.03],
        [-0.02, 0.0, 0.04],
        [0.03, -0.01, 0.0]])))
    temp_coeff: np.ndarray = field(default_factory=lambda: np.deg2rad([0.002, 0.003, -0.002]))  # r