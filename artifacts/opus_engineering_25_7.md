# 3-Axis Rigid Body Attitude Simulator with Motor Torque Curves and Gyro Drift

This is a self-contained Python program (≈500 lines). It needs only `numpy`, and `matplotlib` if you want plots. I wasn't sure what "Cycle 25" refers to, so the code is tagged `CYCLE = 25` in the header and output. Change that if it means something specific in your project.

## What it models

| Subsystem | Model |
|---|---|
| **Rigid body** | Full inertia tensor (including products of inertia), quaternion kinematics, Euler's equations with gyroscopic coupling to the stored wheel momentum, RK4 integration |
| **Actuators** | 3 (or N) reaction wheels driven by brushless DC motors with these effects: <br>• Torque constant and back-EMF <br>• Winding resistance and supply voltage limit, which together give the torque-speed curve <br>• Current limit <br>• First-order current-loop lag <br>• Viscous and Coulomb friction <br>• Speed saturation |
| **Gyro** | Angle random walk (ARW), bias instability (Gauss-Markov process), rate random walk, turn-on bias, scale-factor error, axis misalignment, temperature drift, range saturation, quantization, and finite sample rate |
| **Navigation and control** | Attitude estimated by integrating the gyro (so it drifts), an optional star tracker that resets the drift, a quaternion PD controller, wheel torque allocation, and environmental disturbance torques |
| **Diagnostics** | Inertial angular momentum conservation check, Allan deviation of the gyro, and plots of the torque-speed curve with the operating points |

```python
#!/usr/bin/env python3
"""
attitude_sim.py  --  3-Axis Rigid Body Physics Simulator   (Cycle 25)

Models:
  * 3-axis rigid-body rotational dynamics (quaternion + Euler equations, RK4)
  * Reaction-wheel actuators driven by DC motors with realistic torque-speed curves
  * MEMS/FOG-style gyro with bias drift, noise, scale factor, misalignment, temperature
  * Gyro-integrated attitude estimator (drifts), optional star-tracker resets
  * Quaternion PD attitude controller

Usage:
  python attitude_sim.py                       # default slew + hold, with plots
  python attitude_sim.py --tend 600 --no-star  # watch pure gyro drift
  python attitude_sim.py --no-plot --seed 3
  python attitude_sim.py --test                # run physics self-checks
"""

from __future__ import annotations
import argparse
import math
from dataclasses import dataclass, field
import numpy as np

CYCLE = 25

# =============================================================================
# Quaternion utilities   (scalar-first: q = [w, x, y, z], body->inertial)
# =============================================================================

def q_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ])

def q_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])

def q_norm(q: np.ndarray) -> np.ndarray:
    return q / np.linalg.norm(q)

def q_from_axis_angle(axis, angle: float) -> np.ndarray:
    axis = np.asarray(axis, float)
    n = np.linalg.norm(axis)
    if n < 1e-15:
        return np.array([1.0, 0, 0, 0])
    axis = axis / n
    s = math.sin(angle / 2)
    return np.array([math.cos(angle / 2), *(axis * s)])

def q_from_rotvec(rv: np.ndarray) -> np.ndarray:
    ang = np.linalg.norm(rv)
    return q_from_axis_angle(rv, ang) if ang > 1e-15 else np.array([1.0, 0, 0, 0])

def q_to_dcm(q: np.ndarray) -> np.ndarray:
    """Rotation matrix body->inertial."""
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ])

def q_angle_between(a: np.ndarray, b: np.ndarray) -> float:
    """Smallest rotation angle (rad) between two attitudes."""
    d = abs(float(np.dot(a, b)))
    return 2.0 * math.acos(min(1.0, d))

def q_to_euler_321(q: np.ndarray) -> np.ndarray:
    """Roll, pitch, yaw (rad) for display."""
    w, x, y, z = q
    roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return np.array([roll, pitch, yaw])

def q_kinematics(q: np.ndarray, w: np.ndarray) -> np.ndarray:
    """q_dot = 0.5 * q ⊗ [0, w]   (w in body frame)."""
    return 0.5 * q_mul(q, np.array([0.0, *w]))

def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


# =============================================================================
# DC motor + reaction wheel  (torque-speed curve)
# =============================================================================

@dataclass
class MotorParams:
    Kt: float = 0.030          # N·m/A torque constant
    Ke: float = 0.030          # V·s/rad back-EMF constant (= Kt in SI)
    R: float = 1.2             # ohm winding resistance
    V_bus: float = 24.0        # V supply
    I_max: float = 3.0         # A current limit (driver)
    tau_elec: float = 0.005    # s current-loop time constant
    b_visc: float = 2.0e-6     # N·m·s/rad viscous friction
    c_coul: float = 1.5e-3     # N·m Coulomb friction
    w_coul_smooth: float = 0.5 # rad/s smoothing width for sign()

    def available_current(self, omega: float) -> tuple[float, float]:
        """Current range achievable given bus voltage and back-EMF."""
        i_hi = (self.V_bus - self.Ke * omega) / self.R
        i_lo = (-self.V_bus - self.Ke * omega) / self.R
        return max(-self.I_max, i_lo), min(self.I_max, i_hi)

    def max_torque(self, omega: float) -> tuple[float, float]:
        """Electromagnetic torque envelope (min, max) at shaft speed omega."""
        lo, hi = self.available_current(omega)
        return self.Kt * lo, self.Kt * hi

    def friction(self, omega: float) -> float:
        return self.b_visc * omega + self.c_coul * math.tanh(omega / self.w_coul_smooth)

    @property
    def no_load_speed(self) -> float:
        return self.V_bus / self.Ke

    @property
    def stall_torque(self) -> float:
        return self.Kt * min(self.I_max, self.V_bus / self.R)


@dataclass
class ReactionWheel:
    axis: np.ndarray               # unit spin axis in body frame
    J: float = 2.0e-3              # kg·m² rotor inertia
    motor: MotorParams = field(default_factory=MotorParams)
    current: float = 0.0           # A actual (state of current loop)
    i_cmd: float = 0.0             # A commanded

    def __post_init__(self):
        self.axis = np.asarray(self.axis, float)
        self.axis /= np.linalg.norm(self.axis)

    def electromagnetic_torque(self, omega: float, current: float) -> float:
        lo, hi = self.motor.available_current(omega)
        return self.motor.Kt * min(max(current, lo), hi)

    def net_torque(self, omega: float, current: float) -> float:
        """Torque accelerating the rotor (motor minus friction)."""
        return self.electromagnetic_torque(omega, current) - self.motor.friction(omega)


# =============================================================================
# Gyro model with drift
# =============================================================================

@dataclass
class GyroParams:
    rate_hz: float = 200.0
    arw: float = math.radians(0.15) / 60.0          # rad/√s   (0.15 °/√h)
    bias_instability: float = math.radians(3.0) / 3600.0  # rad/s (3 °/h)
    bias_corr_time: float = 300.0                   # s Gauss-Markov τ
    rrw: float = math.radians(0.5) / 3600.0 / 60.0  # rad/s/√s rate random walk
    turn_on_bias_sigma: float = math.radians(10.0) / 3600.0  # rad/s
    scale_factor_ppm: float = 500.0                 # 1σ
    misalignment_rad: float = 1e-3                  # 1σ
    temp_coeff: float = math.radians(0.5) / 3600.0  # rad/s per °C
    temp_ref: float = 25.0
    range_rad_s: float = math.radians(300.0)
    lsb_rad_s: float = math.radians(0.004)          # quantization


class Gyro:
    def __init__(self, p: GyroParams, rng: np.random.Generator):
        self.p = p
        self.rng = rng
        self.dt = 1.0 / p.rate_hz
        sf = 1.0 + p.scale_factor_ppm * 1e-6 * rng.standard_normal(3)
        mis = p.misalignment_rad * rng.standard_normal((3, 3))
        np.fill_diagonal(mis, 0.0)
        self.M = np.diag(sf) + mis                  # scale + misalignment
        self.b0 = p.turn_on_bias_sigma * rng.standard_normal(3)
        self.b_gm = np.zeros(3)
        self.b_rw = np.zeros(3)

    @property
    def bias(self) -> np.ndarray:
        return self.b0 + self.b_gm + self.b_rw

    def measure(self, w_true: np.ndarray, temp_c: float) -> np.ndarray:
        p, dt, r = self.p, self.dt, self.rng
        # rate random walk
        self.b_rw += p.rrw * math.sqrt(dt) * r.standard_normal(3)
        # bias instability as first-order Gauss-Markov
        a = math.exp(-dt / p.bias_corr_time)
        self.b_g