# 3-Axis Rigid-Body Attitude Simulator (Cycle 16)

I don't have the code from earlier cycles, so this is a **complete, self-contained** program. It runs with NumPy alone. Matplotlib is optional and is only used for plots.

## What it models

| Subsystem | Model |
|---|---|
| **Rigid body** | Full Euler equations with gyroscopic coupling to the wheel momentum: `I·ω̇ = τ_ext − A·τ_w − ω × (I·ω + A·h_w)`. Uses quaternion kinematics and fixed-step RK4. |
| **Actuators** | 3 orthogonal reaction wheels or a 4-wheel pyramid, with pseudo-inverse torque allocation. |
| **Motor torque curves** | Brushless DC motor: back-EMF, bus-voltage headroom, current limit, electrical lag (L/R), viscous and Coulomb friction, and an overspeed cutoff. Produces the classic torque-speed curve. |
| **Gyro** | Angle random walk, bias random walk (drift), initial turn-on bias, temperature-dependent bias, scale-factor error, axis misalignment, saturation and quantization. |
| **Estimation** | Gyro-integrated attitude, plus an optional star tracker that corrects the attitude and estimates the bias. Turning it off shows the gyro drift directly. |
| **Environment** | Gravity-gradient torque plus a residual and orbital-periodic disturbance. |
| **Verification** | `--selftest` checks conservation of inertial angular momentum and kinetic energy in torque-free motion. |

## Source: `rigidsim.py`

```python
#!/usr/bin/env python3
"""
rigidsim.py -- 3-axis rigid-body attitude dynamics simulator (Cycle 16)

Models:
  * Rigid-body rotational dynamics with reaction-wheel momentum coupling
  * Quaternion kinematics (scalar-first, body->inertial), RK4 integration
  * BLDC reaction-wheel motors with realistic torque-speed curves
  * MEMS/FOG-style gyro error model (ARW, bias drift, temp, SF, misalignment)
  * Gyro-propagated attitude estimator with optional star-tracker correction
  * PD attitude controller with slew schedule

Usage:
  python rigidsim.py                      # default 600 s run, prints summary
  python rigidsim.py --plot               # with matplotlib plots
  python rigidsim.py --no-startracker     # observe pure gyro drift
  python rigidsim.py --wheels 4 --plot    # pyramid wheel configuration
  python rigidsim.py --selftest           # conservation-law verification
  python rigidsim.py --curve              # print motor torque-speed curve
  python rigidsim.py --csv out.csv        # dump telemetry

Units: SI throughout (rad, rad/s, N*m, kg*m^2, A, V, s) unless noted.
"""
from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass, field

import numpy as np

DEG = math.pi / 180.0
ARCSEC = DEG / 3600.0
HOUR = 3600.0


# ============================================================================
# Quaternion utilities  (q = [w, x, y, z], rotates body -> inertial)
# ============================================================================
def qmul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x