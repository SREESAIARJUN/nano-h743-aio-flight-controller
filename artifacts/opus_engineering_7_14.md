# 3-Axis Rigid-Body Attitude Simulator (Python)

I don't have context for "Cycle 7" from an earlier conversation, so this is a complete, self-contained version. It covers all three requested areas:

- **Rigid-body dynamics:** Full 3-axis motion of a spacecraft with three reaction wheels, including the cross-coupling between axes.
- **Motor torque curves:** A brushless DC motor model with back-EMF, a current limit, bus-voltage saturation and friction.
- **Gyro drift:** Random noise, slowly wandering bias, temperature sensitivity, scale and alignment errors, and quantization.

It also includes a sensor-fusion estimator, a PD controller, RK4 integration, a self-test, CSV export and plotting.

**Requirements:** Python 3.9+ and `numpy`. `matplotlib` is optional and only needed for `--plot`.

```python
#!/usr/bin/env python3
"""
sim3d.py — 3-axis rigid-body attitude dynamics simulator

Models
------
* Rigid body:   Euler's equations with internal wheel momentum (gyroscopic coupling)
                I·ω̇ = τ_ext − ω×(I·ω + h_w) − A·ḣ_w_spin
* Kinematics:   unit quaternion (scalar-first, body→inertial), q̇ = ½ q ⊗ [0, ω]
* Actuators:    3 reaction wheels driven by BLDC/DC motors
                i = clip((V − Ke·Ω)/R, ±i_max),  τ = Kt·i
                friction = Coulomb·tanh(Ω/Ω_s) + viscous·Ω
                → produces a realistic torque–speed curve (current-limited plateau,
                  back-EMF roll-off to zero torque at no-load speed)
* Gyro:         ω_m = (I + S + M)·ω + b0 + b_rw + b_gm + k_T·(T − T0) + ARW noise
                quantized and range-limited
* Star tracker: noisy absolute attitude at low rate (can be disabled to show drift)
* Estimator:    Mahony-style complementary filter with gyro-bias estimation
* Controller:   quaternion PD + gyroscopic feed-forward, torque-saturated
* Integrator:   RK4, zero-order-hold motor voltage, quaternion renormalization

Usage
-----
    python sim3d.py                    # default 600 s scenario, summary printed
    python sim3d.py --no-star          # gyro-only: watch attitude drift grow
    python sim3d.py --plot             # matplotlib plots
    python sim3d.py --csv out.csv      # log to CSV
    python sim3d.py --curve            # print motor torque–speed curve
    python sim3d.py --selftest         # conservation-law checks
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass, field

import numpy as np

DEG = math.pi / 180.0
ARCSEC = DEG / 3600.0


# ═══════════════════════════════════════════════════════════════════════════
# Quaternion utilities  (scalar-first [w, x, y, z], body → inertial)
# ═══════════════════════════════════════════════════════════════════════════
def q_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def q_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def q_normalize(q: np.ndarray) -> np.ndarray:
    return q / np.linalg.norm(q)


def q_exp(rv: np.ndarray) -> np.ndarray:
    """Rotation vector → quaternion."""
    a = np.linalg.norm(rv)
    if a < 1e-12:
        return q_normalize(np.concatenate(([1.0], 0.5 * rv)))
    return np.concatenate(([math.cos(a / 2)], math.sin(a / 2) * rv / a))


def q_log(q: np.ndarray) -> np.ndarray:
    """Quaternion → rotation vector (shortest path)."""
    if q[0] < 0:
        q = -q
    v = q[1:]
    s = np.linalg.norm(v)
    if s < 1e-12:
        return 2.0 * v
    return 2.0 * math.atan2(s, q[0]) * v / s


def q_from_axis_angle(axis, angle: float) -> np.ndarray:
    axis = np.asarray(axis, float)