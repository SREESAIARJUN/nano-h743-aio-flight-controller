# 3D Rigid-Body Physics Simulator: Cycle 1

This is a single-file, runnable simulator. It requires only `numpy`; `matplotlib` is optional and used only for plotting.

**What it models**

| Subsystem | Model |
|---|---|
| **Rigid body** | Full 3-axis Euler equations with gyroscopic coupling. Quaternion attitude (scalar-first, body→inertial). RK4 integration. |
| **Actuators** | N reaction wheels, either a 3-axis orthogonal or a 4-wheel pyramid layout. Wheel momentum is coupled into the body dynamics. |
| **Motor torque curves** | Current-controlled DC/BLDC motor model. The available torque is limited by the current limit and by the bus voltage (back-EMF headroom). It also includes Coulomb and viscous friction. An optional tabular torque-speed curve overrides the physics-based curve. |
| **Gyro** | Constant turn-on bias, Gauss-Markov bias instability, rate random walk, angle random walk, scale-factor error, axis misalignment, temperature-dependent bias, saturation, and quantization. |
| **GNC loop** | Attitude is propagated from gyro output (so drift corrupts the estimate). Quaternion PD controller with gyroscopic compensation. Torque allocation by pseudo-inverse. |
| **Validation** | A built-in self-test checks inertial angular-momentum conservation and the motor curve limits. |

---

```python
#!/usr/bin/env python3
"""
rigidsim.py  --  3D Rigid-Body Physics Simulator (Cycle 1)
=========================================================

Models:
  * 3-axis rigid-body rotational mechanics (Euler's equations + quaternion kinematics)
  * Reaction-wheel actuators driven by DC/BLDC motors with realistic torque-speed curves
  * MEMS/FOG-style gyro error model (bias, drift, ARW, RRW, scale factor, misalignment,
    temperature sensitivity, quantization, saturation)
  * Closed-loop attitude control using a gyro-propagated attitude estimate

Conventions:
  * Quaternion q = [w, x, y, z] (scalar first), rotates BODY -> INERTIAL
  * Angular velocity w is body-frame, rad/s
  * SI units internally

Usage:
  python rigidsim.py                      # default slew + hold scenario
  python rigidsim.py --plot               # with plots (requires matplotlib)
  python rigidsim.py --feedback truth     # control on true attitude (no drift impact)
  python rigidsim.py --wheels pyramid     # 4-wheel pyramid configuration
  python rigidsim.py --selftest           # physics validation tests
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass, field
from typing import Callable, Optional, Sequence

import numpy as np

DEG = math.pi / 180.0
RAD2DEG = 180.0 / math.pi


# =============================================================================
# Quaternion / rotation utilities
# =============================================================================
def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product a (x) b."""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_normalize(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    if n < 1e-15:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n


def quat_from_axis_angle(axis: Sequence[float], angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    s = math.sin(angle / 2.0)
    return np.array([math.cos(angle / 2.0), *(axis * s)])


def quat_exp_rotvec(rv: np.ndarray) -> np.ndarray:
    """Quaternion for a rotation vector rv (rad)."""
    ang = np.linalg.norm(rv)
    if ang < 1e-12:
        return quat_normalize(np.array([1.0, *(0.5 * rv)]))
    return quat_from_axis_angle(rv / ang, ang)


def quat_to_dcm(q: np.ndarray) -> np.ndarray:
    """DCM rotating body vectors into the inertial frame."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def quat_angle_between(q1: np.ndarray, q2: np.ndarray) -> float:
    """Smallest rotation angle (rad) between two attitudes."""
    d = abs(float(np.dot(q1, q2)))
    return 2.0 * math.acos(min(1.0, d))


def quat_to_euler_zyx(q: np.ndarray) -> np.ndarray:
    """Returns roll, pitch, yaw (rad), aerospace ZYX sequence."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


# =============================================================================
# Motor model with torque-speed curve
# =============================================================================
@dataclass
class MotorParams:
    kt: float = 0.025            # Torque constant  [N*m/A]   (ke = kt in SI)
    resistance: float = 1.2      # Winding resistance [ohm]
    v_bus: float = 24.0          # Supply voltage [V]
    i_max: float = 2.0           # Driver current limit [A]
    coulomb_friction: float = 2e-4   # [N*m]
    viscous_friction: float = 2e-7   # [N*m/(rad/s)]
    friction_smoothing: float = 0.5  # [rad/s] tanh smoothing near zero speed
    # Optional tabular curve: (speed rad/s, max torque N*m). Overrides the
    # physics-based envelope when provided (symmetric in speed sign).
    table_speed: Optional[Sequence[float]] = None
    table_torque: Optional[Sequence[float]] = None

    @property
    def ke(self) -> float:
        return self.kt

    @property
    def no_load_speed(self) -> float:
        return self.v_bus / self.ke

    @property
