# 3D Rigid-Body Attitude Simulator

This single-file Python simulator models:

- **Rigid-body mechanics** in 3D: 6-DOF translation and rotation, quaternion attitude, full inertia tensor, and gyroscopic coupling.
- **Three reaction-wheel motors**: brushless DC electrical model with back-EMF, a torque-speed curve, current and voltage limits, cogging ripple, and Coulomb plus viscous friction.
- **Gyro errors**: angle random walk, rate random walk, Gauss-Markov bias instability, temperature-dependent bias, scale factor error, axis misalignment and quantization.
- **Flight software loop**: gyro-integrating attitude estimator with optional star-tracker corrections and bias estimation, plus a quaternion PD attitude controller.

It depends only on `numpy`. `matplotlib` is optional and only needed for plots.

## Usage

```
python rigid_body_sim.py --tend 120 --plot
python rigid_body_sim.py --no-startracker --plot   # watch gyro drift grow unchecked
python rigid_body_sim.py --selftest                 # conservation checks
python rigid_body_sim.py --csv out.csv
```

## Source code (`rigid_body_sim.py`)

```python
#!/usr/bin/env python3
"""
rigid_body_sim.py -- 3-axis rigid body simulator with reaction-wheel motors
and a realistic gyro error model.  Version tag: Cycle 27.

Conventions
-----------
* Quaternions are scalar-first [w, x, y, z] and rotate body -> inertial.
* omega is the body angular rate relative to inertial, expressed in body axes.
* The three reaction wheels are aligned with the body x, y, z axes.
* h is the total wheel angular momentum: h_i = J_w * (omega_i + Omega_i),
  where Omega_i is the wheel speed relative to the body.

Rotational dynamics (exact for axis-aligned wheels)
---------------------------------------------------
    I_b * domega/dt = tau_ext - omega x (I_b*omega + h) - dh/dt
    dh/dt           = tau_motor - tau_friction
    dOmega/dt       = (dh/dt)/J_w - domega/dt

I_b is the body inertia including the wheels' transverse inertia but
excluding the wheel spin-axis inertia, which is carried in h.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional

import numpy as np


# =============================================================================
# Quaternion / rotation utilities
# =============================================================================

def quat_mul(q: np.ndarray, r: np.ndarray) -> np.ndarray:
    """Hamilton product q ⊗ r (scalar-first)."""
    w0, x0, y0, z0 = q
    w1, x1, y1, z1 = r
    return np.array([
        w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1,
        w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,
        w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1,
        w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1,
    ])


def quat_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_normalize(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    return q / n if n > 0 else np.array([1.0, 0.0, 0.0, 0.0])


def quat_from_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n < 1e-15 or abs(angle) < 1e-15:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = axis / n
    s = math.sin(angle / 2.0)
    return np.array([math.cos(angle / 2.0), *(axis * s)])


def quat_from_rotvec(rv: np.ndarray) -> np.ndarray:
    ang = np.linalg.norm(rv)
    return quat_from_axis_angle(rv, ang) if ang > 1e-15 else np.array([1.0, 0, 0, 0])


def quat_to_dcm(q: np.ndarray) -> np.ndarray:
    """DCM rotating body vectors into the inertial frame."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ])


def quat_to_euler(q: np.ndarray) -> np.ndarray:
    """ZYX Euler angles (roll, pitch, yaw) in radians."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sp)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


def quat_angle_between(q1: np.ndarray, q2: np.ndarray) -> float:
    """Smallest rotation angle (rad) between two attitudes."""
    d = abs(float(np.dot(quat_normalize(q1), quat_normalize(q2))))
    return 2.0 * math.acos(min(1.0, d))


def skew(v: np.ndarray) -> np.ndarray:
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


# =============================================================================
# Motor model (brushless DC reaction-wheel drive)
# =============================================================================

@dataclass
class MotorParams:
    Kt: float = 0.025         # torque constant          [N*m/A]
    Ke: float = 0.025         # back-EMF constant        [V*s/rad] (= Kt in SI)
    R: float = 1.2            # winding resistance       [ohm]
    L: float = 0.8e-3         # winding inductance       [H]
    V_bus: float = 28.0       # bus voltage              [V]
    I_max: float = 2.0        # driver current limit     [A]
    J_w: float = 2.0e-3       # wheel spin inertia       [kg*m^2]
    coulomb: float = 2.0e-4   # Coulomb friction         [N*m]
    viscous: float = 1.0e-6   # viscous friction         [N*m*s/rad]
    stribeck_eps: float = 0.5  # smoothing speed for tanh(Coulomb) [rad/s]
    ripple_amp: float = 1.5e-4  # cogging/ripple amplitude [N*m]
    ripple_harmonic: int = 12   # ripple cycles per revolution
    Kp_current: float = 40.0    # driver current-loop gain [V/A]

    @property
    def no_load_speed(self) -> float:
        return self.V_bus / self.Ke

    @property
    def stall_torque(self) -> float:
        return self.Kt * min(self.V_bus / self.R, self.I_max)


class MotorDriver:
    """Current-mode driver. The commanded torque becomes a current request,
    which a voltage-limited P + feedforward loop tracks. The torque-speed
    curve emerges from the V_bus saturation against the back-EMF."""

    def __init__(self, p: MotorParams):
        self.p = p

    def voltage_command(self, tau_cmd: np.ndarray, i_meas: np.ndarray,
                        Omega: np.ndarray) -> np.ndarray:
        p = self.p
        i_cmd = np.clip(tau_cmd / p.Kt, -p.I_max, p.I_max)
        v = p.R * i_cmd + p.Ke * Omega + p.Kp_current * (i_cmd - i_meas)
        return np.clip(v, -p.V_bus, p.V_bus)

    def friction(self, Omega: np.ndarray) -> np.ndarray:
        p = self.p
        return p.coulomb * np.tanh(Omega / p.stribeck_eps) + p.viscous * Omega

    def ripple(self, theta: np.ndarray, i: np.ndarray) -> np.ndarray:
        p = self.p
        # Cogging and commutation ripple, scaled mildly with load current.
        return p.ripple_amp * np.sin(p.ripple_harmonic * theta) * (
            1.0 + 0.5 * np.abs(i) / p.I_max)

    def steady_state_torque_curve(self, speeds: np.ndarray) -> np.ndarray:
        """Maximum motoring torque vs. speed (static curve, for plotting)."""
        p = self.p
        i_avail = np.clip((p.V_bus - p.Ke * np.abs(speeds)) / p.R, 0.0, p.I_max)
        return p.Kt * i_avail - (p.coulomb + p.viscous * np.abs(speeds))


# =============================================================================
# Gyro model
# =============================================================================

@dataclass
class GyroParams:
    rate_hz: float = 200.0
    arw: float = math.radians(0.15) / 60.0         # angle random walk [rad/sqrt(s)]
    rrw: float = math.radians(2.0) / 3600.0 / 60.0  # rate random walk  [rad/s/sqrt(s)]
    bias_instability: float = math.radians(3.0) / 3600.0  # GM sigma [rad/s]
    bias_corr_time: float = 300.0                  # GM correlation time [s]
    turn_on_bias: float = math.radians(10.0) / 3600.0  # 1-sigma per axis [rad/s]
    temp_coeff: float = math.radians(0.5) / 3600.0     # bias slope [rad/s per degC]
    temp_ref: float = 25.0                         # [degC]
    scale_factor_ppm: float = 300.0                # 1-sigma per axis
    misalignment_rad: float = 1.0e-3               # 1-sigma per off-diag