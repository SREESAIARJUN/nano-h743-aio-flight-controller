# 3-Axis Rigid Body Physics Simulator (Cycle 4)

I don't have context on what "Cycle 4" refers to in your project, so this is a complete, self-contained simulator built from scratch. If you have earlier cycle code or specific requirements, share them and I'll adapt this to match.

## What it models

- **Rigid body:** full 3D rotational dynamics using Euler's equations with an arbitrary inertia tensor. Attitude is a quaternion, and the state is integrated with RK4.
- **Actuators:** three reaction wheels driven by DC motors, with:
  - a torque–speed curve (back-EMF voltage limit)
  - a current limit
  - Coulomb and viscous friction
  - wheel speed saturation
- **Gyro:** a MEMS/FOG-style error model with:
  - angle random walk
  - bias random walk
  - Gauss-Markov bias instability
  - temperature-dependent bias
  - scale factor and misalignment errors
  - quantization and saturation
- **Estimation and control:** attitude is propagated from the gyro, with optional star-tracker correction. A PD controller with gyroscopic compensation drives the wheels.
- **Validation:** a momentum-conservation self-test, plus CSV and plot output.

## Requirements

```bash
pip install numpy matplotlib   # matplotlib optional (plots)
```

## Source: `rigid_sim.py`

```python
#!/usr/bin/env python3
"""
rigid_sim.py - 3-axis rigid body simulator with reaction-wheel motor
torque curves and gyro drift modeling.

Conventions
-----------
* Quaternion q = [w, x, y, z], scalar-first, rotates body -> inertial.
* Angular velocity w is the body rate expressed in body frame [rad/s].
* Wheel momentum h is expressed in body frame [N*m*s]; wheel i spins about body axis i.
* Body dynamics:  I*dw/dt = -w x (I*w + h) - tau_m + tau_ext
  Wheel dynamics: dh/dt   =  tau_m
"""

from __future__ import annotations
import argparse
import csv
from dataclasses import dataclass, field
import numpy as np

# ----------------------------------------------------------------------------
# Quaternion utilities
# ----------------------------------------------------------------------------

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
    axis = axis / np.linalg.norm(axis)
    s = np.sin(angle / 2.0)
    return np.array([np.cos(angle / 2.0), *(axis * s)])

def q_to_dcm(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ])

def q_dot(q: np.ndarray, w: np.ndarray) -> np.ndarray:
    return 0.5 * q_mul(q, np.array([0.0, *w]))

def q_exp_step(q: np.ndarray, w: np.ndarray, dt: float) -> np.ndarray:
    """Exact quaternion propagation for constant rate over dt."""
    rate = np.linalg.norm(w)
    if rate * dt < 1e-12:
        dq = np.array([1.0, *(0.5 * w * dt)])
    else:
        dq = q_from_axis_angle(w / rate, rate * dt)
    return q_norm(q_mul(q, dq))

def q_angle_between(a: np.ndarray, b: np.ndarray) -> float:
    """Smallest rotation angle [rad] between two attitudes."""
    d = abs(np.dot(a, b))
    return 2.0 * np.arccos(np.clip(d, -1.0, 1.0))

# ----------------------------------------------------------------------------
# Motor model (DC brushless-equivalent driving a reaction wheel)
# ----------------------------------------------------------------------------

@dataclass
class MotorParams:
    Kt: float = 0.03           # torque constant [N*m/A]
    Ke: float = 0.03           # back-EMF constant [V*s/rad]
    R: float = 1.2             # winding resistance [ohm]
    V_supply: float = 24.0     # bus voltage [V]
    I_max: float = 3.0         # driver current limit [A]
    tau_coulomb: float = 2e-4  # Coulomb friction [N*m]
    b_viscous: float = 2e-7    # viscous friction [N*m*s/rad]
    J_wheel: float = 2e-3      # wheel rotor inertia [kg*m^2]
    omega_max: float = 650.0   # hard speed limit (~6200 rpm) [rad/s]

class Motor:
    """DC motor torque-speed model with voltage/current limits and friction."""

    def __init__(self, p: MotorParams):
        self.p = p

    @property
    def stall_torque(self) -> float:
        return self.p.Kt * min(self.p.I_max, self.p.V_supply / self.p.R)

    @property
    def no_load_speed(self) -> float:
        return self.p.V_supply / self.p.Ke

    def current_limits(self, omega: float) -> tuple[float, float]:
        """Achievable current range given back-EMF at wheel speed omega."""
        p = self.p
        i_hi = (p.V_supply - p.Ke * omega) / p.R
        i_lo = (-p.V_supply - p.Ke * omega) / p.R
        return max(i_lo, -p.I_max), min(i_hi, p.I_max)

    def friction(self, omega: float) -> float:
        # tanh smooths the Coulomb discontinuity at zero speed
        return self.p.tau_coulomb * np.tanh(omega / 0.5) + self.p.b_viscous * omega

    def torque(self, tau_cmd: float, omega: float) -> float:
        """Net torque delivered to wheel rotor for a commanded torque."""
        p = self.p
        i_cmd = tau_cmd / p.Kt
        i_lo, i_hi = self.current_limits(omega)
        i = float(np.clip(i_cmd, i_lo, i_hi))
        tau = p.Kt * i - self.friction(omega)
        # Speed limiter: block torque that would push past omega_max
        if (omega >= p.omega_max and tau > 0) or (omega <= -p.omega_max and tau < 0):
            tau = -self.friction(omega)
        return tau

    def torque_curve(self, n: int = 11):
        """Max positive torque vs. speed (the motor's torque-speed envelope)."""
        speeds = np.linspace(0, self.p.omega_max, n)
        return speeds, np.array([self.torque(1e9, w) for w in speeds])

# ----------------------------------------------------------------------------
# Gyro error model
# ----------------------------------------------------------------------------

@dataclass
class GyroParams:
    arw: float = np.deg2rad(0.15) / 60.0          # angle random walk [rad/sqrt(s)]
    brw: float = np.deg2rad(0.5) / 3600.0 / 30.0  # bias random walk [rad/s/sqrt(s)]
    bias_instab: float = np.deg2rad(3.0) / 3600.0 # Gauss-Markov sigma [rad/s]
    bias_tau: float = 300.0                        # GM correlation time [s]
    bias_init: np.ndarray = field(
        default_factory=lambda: np.deg2rad([10.0, -6.0, 4.0]) / 3600.0)
    temp_coeff: float = np.deg2rad(0.5) / 3600.0  # bias per deg C [rad/s/C]
    temp_ref: float = 25.0
    scale_factor: np.ndarray = field(
        default_factory=lambda: np.array([300e-6, -200e-6, 150e-6]))
    misalign: float = 1e-3                         # rad, random small-angle
    quant: float = np.deg2rad(0.001)               # LSB [rad/s]
    sat: float = np.deg2rad(300.0)                 # range [rad/s]

class Gyro:
    def __init__(self, p: GyroParams, rng: np.random.Generator):
        self.p = p
        self.rng = rng
        self.bias_rw = p.bias_init.copy()
        self.bias_gm = np.zeros(3)
        m = rng.normal(0, p.misalign, 6)
        M = np.array([[0, m[0], m[1]],
                      [m[2], 0, m[3]],
                      [m[4], m[5], 0]])
        self.C = np.eye(3) + np.diag(p.scale_factor) + M

    def total_bias(self, temp_c: float) -> np.ndarray:
        return self.bias_rw + self.bias_gm + self.p.temp_coeff * (temp_c - self.p.temp_ref)

    def measure(self, w_true: np.ndarray, dt: float, temp_c: float) -> np.ndarray:
        p, rng = self.p, self.rng
        # Bias random walk
        self.bias_rw += p.brw * np.sqrt(dt) * rng.standard_normal(3)
        # First-order Gauss-Markov (exact discretization)
        phi = np.exp(-dt / p.bias_tau)
        self.bias_gm = phi * self.bias_gm + p.bias_instab * np.sqrt(1 - phi**2) * rng.standard_normal(3)
        # White rate noise (ARW)
        noise = p.arw / np.sqrt(dt) * rng.standard_normal(3)
        meas = self.C @ w_true + self.total_bias(temp_c) + noise
        meas = np.clip(meas, -p.sat, p.sat)
        return np.round(meas / p.quant) * p.quant

# ----------------------------------------------------------------------------
# Rigid body + reaction wheels
# ----------------------------------------------------------------------------

class Spacecraft:
    def __init__(self, inertia: np.ndarray, motors: list[Motor],
                 q0: np.ndarray, w0: np.ndarray):
        self.I = np.asarray(inertia, float)
        self.I_inv = np.linalg.inv(self.I)
        self.motors = motors
        self.q = q_norm(np.asarray(q0, float))
        self.w = np.asarray(w0, float).copy()
        self.h = np.zeros(3)  # wheel momentum (absolute, about each axis)

    def wheel_rel_speed(self, w: np.ndarray, h: np.ndarray) -> np.ndarray:
        J = np.array([m.p.J_wheel for m in self.motors])
        return h / J - w

    def derivs(self, q, w, h, tau_m, tau_ext):
        wdot = self.I_inv @ (-np.cross(w, self.I @ w + h) - tau_m + tau_ext)
        return q_dot(q, w), wdot, tau_m

    def step(self, dt: float, tau_m_cmd: np.ndarray, tau_ext_fn, t: float):
        """RK4 step. Motor torque recomputed at each stage (speed-dependent)."""
        def f(q, w, h, tt):
            om = self.wheel_rel_speed(w, h)
            tau_m = np.array([m.torque(tau_m_cmd[i], om[i])
                              for i, m in enumerate(self.motors)])
            return self.derivs(q, w, h, tau_m, tau_ext_fn(tt, q))

        q, w, h = self.q, self.w, self.h
        k1 = f(q, w, h, t)
        k2 = f(q + 0.5*dt*k1[0], w + 0.5*dt*k1[1], h + 0.5*dt*k1[2], t + 0.5*dt)
        k3 = f(q + 0.5*dt*k2[0], w + 0.5*dt*k2[1], h + 0.5*dt*k2[2], t + 0.5*dt)
        k4 = f(q + dt*k3[0], w + dt*k3[1], h + dt*k3[2], t + dt)
        self.q = q