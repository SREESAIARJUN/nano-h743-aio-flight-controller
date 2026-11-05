# 3-Axis Rigid-Body Attitude Simulator (Python, Cycle 14)

This is a complete, self-contained simulator in one file. It needs only `numpy`. `matplotlib` is optional and used for plots.

I don't have the earlier cycles of your project, so this is written as a standalone, complete build rather than a diff. If you have an existing interface from Cycle 13, let me know and I can adapt the class names and APIs to match it.

## Features

- **Rigid-body mechanics**
  - Full inertia tensor, including products of inertia.
  - Euler's equations with the gyroscopic coupling term ω × H.
  - Quaternion kinematics.
  - Translational dynamics.
  - RK4 integration.
- **Reaction-wheel actuators**
  - Arbitrary wheel axes, with momentum exchange between wheels and body.
- **BLDC/DC motor torque curves**
  - Back-EMF-limited torque–speed envelope and a current limit.
  - Electrical lag.
  - Viscous and Coulomb friction.
  - Power draw.
- **Gyro error model**
  - Constant bias, angle random walk (ARW) and rate random walk (RRW).
  - Gauss-Markov bias instability.
  - Thermal drift.
  - Scale factor and axis misalignment.
  - Quantization, saturation and discrete sampling.
- **Estimation and control**
  - Gyro-integrated attitude estimator, with an optional star-tracker complementary filter that also estimates bias.
  - Quaternion PD controller.
- **Tooling**
  - CSV logging and plots.
  - Torque-curve export.
  - A conservation-law self-test.

```python
#!/usr/bin/env python3
"""
rigidsim.py  --  3-Axis Rigid Body Physics Simulator  (Cycle 14)
==================================================================

Models
------
  * 6-DOF rigid body: translational + rotational dynamics, full inertia tensor,
    Euler's equations with gyroscopic coupling, quaternion kinematics.
  * Reaction wheel assembly (N wheels, arbitrary spin axes) with momentum exchange.
  * BLDC/DC motor model: back-EMF limited torque-speed curve, current limit,
    electrical time constant, viscous + Coulomb friction, power draw.
  * MEMS/FOG-style gyro: constant bias, angle random walk (ARW), rate random
    walk (RRW), Gauss-Markov bias instability, thermal drift, scale factor,
    misalignment, quantization, saturation, discrete sampling.
  * Attitude estimator: pure gyro integration (shows drift) or complementary
    filter with star-tracker updates and online bias estimation.
  * Quaternion PD attitude controller with gyroscopic feed-forward.

Conventions
-----------
  Quaternion q = [w, x, y, z], scalar first, maps BODY -> INERTIAL.
  Angular velocity omega is expressed in the BODY frame (rad/s).
  Euler angles are ZYX (yaw, pitch, roll).

Usage
-----
  python rigidsim.py                      # default slew scenario, plots if available
  python rigidsim.py --no-startracker     # observe raw gyro drift in the estimate
  python rigidsim.py --selftest           # torque-free conservation checks
  python rigidsim.py --torque-curve       # print/plot the motor torque-speed curve
  python rigidsim.py --csv out.csv --duration 300 --seed 3
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass, field

import numpy as np

# ----------------------------------------------------------------------------
# Unit helpers
# ----------------------------------------------------------------------------
D2R = math.pi / 180.0
R2D = 180.0 / math.pi
DEG_PER_HR = D2R / 3600.0                 # deg/hr          -> rad/s
DEG_PER_SQRT_HR = D2R / 60.0              # deg/sqrt(hr)    -> rad/sqrt(s)
DEG_PER_HR_PER_SQRT_HR = D2R / 3600.0 / 60.0   # deg/hr/sqrt(hr) -> rad/s/sqrt(s)
RPM = 2.0 * math.pi / 60.0


# ----------------------------------------------------------------------------
# Quaternion math
# ----------------------------------------------------------------------------
def qmul(p: np.ndarray, q: np.ndarray) -> np.ndarray:
    pw, pv = p[0], p[1:]
    qw, qv = q[0], q[1:]
    w = pw * qw - pv @ qv
    v = pw * qv + qw * pv + np.cross(pv, qv)
    return np.array([w, *v])


def qconj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def qnormalize(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    return q / n if n > 0 else np.array([1.0, 0, 0, 0])


def qfrom_rotvec(rv: np.ndarray) -> np.ndarray:
    ang = np.linalg.norm(rv)
    if ang < 1e-12:
        return qnormalize(np.array([1.0, *(0.5 * rv)]))
    ax = rv / ang
    return np.array([math.cos(ang / 2), *(math.sin(ang / 2) * ax)])


def qto_dcm(q: np.ndarray) -> np.ndarray:
    """Rotation matrix BODY -> INERTIAL."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ])


def qfrom_euler(yaw: float, pitch: float, roll: float) -> np.ndarray:
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def qto_euler(q: np.ndarray) -> np.ndarray:
    """Returns (yaw, pitch, roll) in rad, ZYX."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sp)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z