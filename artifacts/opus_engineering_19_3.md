# 3-Axis Rigid Body Physics Simulator in Python

I don't have context for earlier "cycles," so this is a complete, self-contained simulator. It models:

- **Rigid body dynamics:** full inertia tensor, Euler's equations, gyroscopic coupling, and quaternion attitude.
- **Three reaction-wheel motors:** a DC motor electrical model that produces a realistic torque-speed curve, plus back-EMF, current and voltage limits, Coulomb and viscous friction, and wheel saturation.
- **A 3-axis MEMS gyro:** bias random walk (drift), angle random walk noise, temperature-dependent bias, scale-factor error, axis misalignment, quantization and a rate range limit.
- **A closed-loop attitude controller:** a quaternion PD controller fed by the noisy gyro, with a simple star-tracker bias estimator.
- **An RK4 integrator** with logging and plots, plus built-in physics self-tests.

Requirements: `numpy` (required) and `matplotlib` (optional, for plots).

```python
#!/usr/bin/env python3
"""
rigidsim.py - 3D rigid body simulator with reaction-wheel motors and gyro drift.

Usage:
    python rigidsim.py                 # run default slew scenario + plots
    python rigidsim.py --no-plot       # text output only
    python rigidsim.py --test          # run physics self-tests
    python rigidsim.py --duration 120 --dt 0.005 --seed 7
"""
from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field

import numpy as np

# ---------------------------------------------------------------------------
# Quaternion utilities (scalar-first: q = [w, x, y, z], body->inertial)
# ---------------------------------------------------------------------------

def q_normalize(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    return q / n if n > 0 else np.array([1.0, 0.0, 0.0, 0.0])


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


def q_from_axis_angle(axis, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    s = math.sin(angle / 2.0)
    return np.array([math.cos(angle / 2.0), *(axis * s)])


def q_to_dcm(q: np.ndarray) -> np.ndarray:
    """Rotation matrix mapping body vectors into the inertial frame."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def q_to_euler(q: np.ndarray) -> np.ndarray:
    """ZYX Euler angles (roll, pitch, yaw) in radians."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


def q_error_angle(q_a: np.ndarray, q_b: np.ndarray) -> float:
    """Smallest rotation angle between two attitudes (rad)."""
    d = abs(float(np.dot(q_a, q_b)))
    return 2.0 * math.acos(min(1.0, d))


def q_integrate(q: np.ndarray, omega: np.ndarray, dt: float) -> np.ndarray:
    """Exact quaternion propagation for constant body rate over dt."""
    rate = np.linalg.norm(omega)
    if rate < 1e-12:
        dq = np.array([1.0, *(0.5 * omega * dt)])
    else:
        dq = q_from_axis_angle(omega / rate, rate * dt)
    return q_normalize(q_mul(q, dq))


def skew(v: np.ndarray) -> np.ndarray:
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


# ---------------------------------------------------------------------------
# Motor model: brushless DC reaction wheel with torque-speed curve
# ---------------------------------------------------------------------------

@dataclass
class MotorParams:
    kt: float = 0.025             # torque constant [N*m/A]
    ke: float = 0.025             # back-EMF constant [V*s/rad] (= kt in SI)
    resistance: float = 1.2       # winding resistance [ohm]
    v_supply: float = 12.0        # bus voltage [V]
    i_max: float = 3.0            # current limit [A]
    wheel_inertia: float = 2.0e-3  # rotor + flywheel inertia [kg*m^2]
    coulomb_friction: float = 1.0e-4  # [N*m]
    viscous_friction: float = 2.0e-6  # [N*m*s/rad]

    @property
    def no_load_speed(self) -> float:
        return self.v_supply / self.ke

    @property
    def stall_torque(self) -> float:
        return self.kt * min(self.i_max, self.v_supply / self.resistance)


class ReactionWheelMotor:
    """
    Electrical model (inductance neglected, since the electrical time constant
    is much shorter than the step size):
        i   = (V - ke*Omega) / R,  |i| <= i_max,  |V| <= v_supply
        tau = kt*i - friction(Omega)
    The voltage and current limits produce the classic torque-speed curve:
    flat current-limited torque at low speed, then a linear decline to zero
    at no-load speed.
    """

    def __init__(self, params: MotorParams, axis: np.ndarray):
        self.p = params
        self.axis = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
        self.speed = 0.0          # wheel speed relative to body [rad/s]
        self.current = 0.0
        self.voltage = 0.0
        self.torque_cmd = 0.0

    def available_torque(self, speed: float, direction: float) -> float:
        """Maximum electromagnetic torque in a given direction at a given speed."""
        p = self.p
        v = math.copysign(p.v_supply, direction)
        i = (v - p.ke * speed) / p.resistance
        i = max(-p.i_max, min(p.i_max, i))
        return p.kt * i

    def friction(self, speed: float) -> float:
        p = self.p
        if abs(speed) < 1e-6:
            return 0.0
        return math.copysign(p.coulomb_friction, speed) + p.viscous_friction * speed

    def electromagnetic_torque(self, speed: float) -> float:
        """Map the commanded torque to the achievable torque via the motor driver."""
        p = self.p
        i_req = self.torque_cmd / p.kt
        v_req = i_req * p.resistance + p.ke * speed
        v = max(-p.v_supply, min(p.v_supply, v_req))
        i = (v - p.ke * speed) / p.resistance
        i = max(-p.i_max, min(p.i_max, i))
        self.voltage, self.current = v, i
        return p.kt * i

    def net_torque(self, speed: float) -> float:
        """Torque accelerating the wheel (motor torque minus friction)."""
        return self.electromagnetic_torque(speed) - self.friction(speed)

    def torque_speed_curve(self, n: int = 50):
        speeds = np.linspace(0.0, self.p.no_load_speed, n)
        torques = np.array([self.available_torque(s, +1.0) for s in speeds])
        return speeds, torques


# ---------------------------------------------------------------------------
# Gyro model: bias drift, noise, temperature, scale factor, misalignment
# ---------------------------------------------------------------------------

@dataclass
class GyroParams:
    arw: float = math.radians(0.15) / 60.0      # angle random walk [rad/sqrt(s)]
    rrw: float = math.radians(10.0) / 3600.0 / 30.0  # rate random walk [rad/s/sqrt(s)]
    bias_init_sigma: float = math.radians(20.0) / 3600.0  # turn-on bias [rad/s]
    bias_tc: float = 300.0                       # bias Gauss-Markov time constant [s]
    temp_coeff: float = math.radians(1.0) / 3600.0  # bias per deg C [rad/s/C]
    scale_factor_ppm: float = 500.0              # 1-sigma scale-factor error
    misalignment_sigma: float = 1.0e-3           # [rad]
    range_limit: float = math.radians(300.0)     # saturation [rad/s]
    lsb: float = math.radians(0.0076)            # quantization [rad/s]
    sample_rate: float = 200.0                   # [Hz]


class Gyro:
    def __init__(self, params: GyroParams, rng: np.random.Generator):
        self.p = params
        self.rng = rng
        self.bias = rng.normal(0.0, params.bias_init_sigma, 3)
        self.markov = np.zeros(3)  # first-order Gauss-Markov bias instability
        sf = rng.normal(0.0, params.scale_factor_ppm * 1e-6, 3)
        mis = rng.normal(0.0, params.misalignment_sigma, (3, 3))
        np.fill_diagonal(mis, 0.0)
        self.M = np.eye(3) + np.diag(sf) + mis   # scale-factor/misalignment matrix
        self.temperature = 25.0
        self.t_next = 0.0
        self.last = np.zeros(3)

    def total_bias(self) -> np.ndarray:
        return self.bias + self.markov + self.p.temp_coeff * (self.temperature - 25.0)

    def propagate(self, dt: float):
        p = self.p
        # rate random walk (unbounded drift)
        self.bias += p.rrw * math.sqrt(dt) * self.rng.normal(size=3)
        # Gauss-Markov bias instability
        a = math.exp(-dt / p.bias_tc)
        sigma_ss = p.bias_init_sigma * 0.25
        self.markov = a * self.markov + sigma_ss * math.sqrt(1 - a * a) * self.rng.normal(size=3)

    def measure(self, t: float, omega_true: np.ndarray) -> np.ndarray:
        """Sample-and-hold measurement at the gyro's own sample rate."""
        if t + 1e-12 >= self.t_next:
            p = self.p
            dt_s = 1.0 / p.sample_rate
            self.propagate(dt_s)
            white = p.arw / math.sqrt(dt_s) * self.rng.normal(size=3)
            meas = self.M @ omega_true + self.total_bias() + white
            meas = np.clip(meas, -p.range_limit, p.range_limit)
            self.last = np.round(meas / p.lsb) * p.lsb
            self.t_next += dt_s
        return self.last


# ---------------------------------------------------------------------------
# Rigid body with embedded reaction wheels
# ---------------------------------------------------------------------------

@dataclass
class BodyState:
    q: np.ndarray = field(default_factory=lambda: np.array([1.0, 0, 0, 0]))
    omega: np.ndarray = field(default_factory=lambda: np.zeros(3))  # body rate [rad/s]
    wheel_speeds: np.ndarray = field(default_factory=lambda: np.zeros(3))


class RigidBody:
    """
    Equations of motion (body frame), with wheel momentum h = A @ (Jw * Omega):
        I_b * w_dot = tau_ext - w x (I_b*w + h) - A @ tau_wheel
        Omega_dot   = tau_wheel / Jw - (A^T w_dot)
    where I_b includes the wheel masses but excludes their spin inertia
    about their axes.
    """

    def __init__(self, inertia: np.ndarray, motors: list[ReactionWheelMotor]):
        self.I = np.asarray(inertia, dtype=float)
        self.motors = motors
        self.A = np.column_stack([m.axis for m in motors])        # 3 x N
        self.Jw = np.array([m.p.wheel_inertia for m in motors])
        # Effective inertia including wheel spin-axis inertia, for the
        # coupled solve with absolute wheel momentum.
        self.I_total = self.I + self.A @ np.diag(self.Jw) @ self.A.T
        self.I_total_inv = np.linalg.inv(self.I_total)
        self.state = BodyState()
        self.ext_torque_fn = lambda t, s: np.zeros(3)

    def wheel_momentum(self, omega, speeds) -> np.ndarray:
        # absolute wheel spin momentum about each wheel axis, in body frame
        return self.A @ (self.Jw * (speeds + self.A.T @ omega))

    def angular_momentum_body(self, omega, speeds) -> np.ndarray:
        return self.I @ omega + self.wheel_momentum(omega, speeds)

    def derivatives(self, t, omega, speeds):
        tau_w = np.array([m.net_torque(s) for m, s in zip(self.motors, speeds)])
        tau_ext = self.ext_torque_fn(t, self.state)
        H = self.angular_momentum_body(omega, speeds)
        # Motor torques are internal: they cancel in total momentum.
        # Body: I_total*w_dot + A*Jw*Omega_dot = tau_ext - w x H
        # Wheel: Jw*(Omega_dot + a^T w_dot) = tau_w
        rhs = tau_ext - np.cross(omega, H) - self.A @ tau_w
        omega_dot = np.linalg.solve(self.I, rhs)
        speeds_dot = tau_w / self.Jw - self.A.T @ omega_dot
        return omega_dot, speeds_dot

    def step(self, t: float, dt: float):
        """RK4 for rates and wheel speeds, exact quaternion update per stage."""
        s = self.state
        w0, n0 = s.omega, s.wheel_speeds
        k1w, k1n = self.derivatives(t, w0, n0)
        k2w, k2n = self.derivatives(t + dt / 2, w0 + dt / 2 *