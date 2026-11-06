#!/usr/bin/env python3
"""
Nano H743 AIO - DPS310 (U4) foam chamber: geometry, acoustic / pneumatic filter,
water, light, thermal and fit check (Revision 12.0)

Single source of geometry: cad/baro_foam_chamber.scad.  Its top-level
"name = expr;" assignments and assert()s are parsed and evaluated here with a
small safe expression evaluator (OpenSCAD subset: + - * / %, comparisons,
&& || !, ?:, sqrt pow atan atan2 sin cos tan abs max min floor ceil round),
so no dimension is duplicated in this file.  Python 3 + numpy.
Writes analysis/baro_acoustic_report.json, exits 1 if any gate fails.

Sections
  1. SCAD parse + evaluation, SCAD asserts re-evaluated in Python, and a
     cross-check against OpenSCAD's own ECHO output (openscad -o x.echo).
  2. Render part = cap / foam_block / air / vent_air to stl/ with OpenSCAD
     (skip with --no-render): CGAL 'Simple: yes' and 'Volumes: 2' (exactly one
     solid), STL connectivity, vent_air spans cavity face -> outer face, STL
     volumes against the analytic values.
  3. Driver / ArduPilot sampling: DPS310 PM_RATE / PM_PRC from
     firmware/baro_dps310/dps310_robust.h (Config defaults) and the measurement
     time table in dps310_robust.cpp; hwdef.dat BARO line; defaults.parm
     overrides (none expected).
  4. Lumped acoustic model (two-node ladder):
        p_out --[R_v + jwM_v]-- node1 (C1: air in the foam pores + vent share)
              --[R_f]-- node2 (C2: air gap around U4 below the foam) = sensor
     with an optional leak branch outside -> node2 (bond-line pinhole).
     Corner sweep: vent_a +/- vent_tol, foam_sigma x0.5..x2, T -20..85 C,
     p_amb 70..105 kPa, adiabatic / isothermal, inertance profile factor 1..4/3.
     Gates: no resonant gain, prop blade-pass band residual after the DPS310
     oversampling comb, quasi-static lag tau and altitude error at 30 / 10 m/s.
     Bond-leak sensitivity -> leak spec and a leak-down acceptance test.
  5. Water ingress (hydrophobic capillary entry pressure, dynamic pressure,
     closed-volume penetration depth, drain slope).
  6. Light shielding (line of sight through the labyrinth, foam coverage, cap
     opacity requirement).
  7. Thermal (cap foot / resin vs Q11 and board temperature from
     analysis/thermal_report.json vs resin HDT), foam compression retention,
     stack height, board edge / grommet / courtyard fit.

Formula sources
  [SL]  Shah & London, "Laminar Flow Forced Convection in Ducts" (1978): square
        duct fully developed laminar f*Re = 56.91  ->  dp = 28.45 mu L Q / a^4.
  [SA]  Sampson (1891) / Happel & Brenner: creeping flow through a circular
        orifice in a thin wall dp = 3 mu Q / r^3 (half per side used as the
        resistive end correction of each port).
  [KI]  Kinsler, Frey et al., "Fundamentals of Acoustics", 4th ed. s10.8-10.9:
        acoustic compliance C = V / (rho c^2) = V / (gamma p) (adiabatic),
        V / p (isothermal); inertance M = rho L' / S with flanged end
        correction 0.85 r per end; Helmholtz w0 = 1/sqrt(M C).
  [BE]  Beranek, "Acoustics" (1954) ch.5: lumped series R-M-C, zeta =
        (R/2) sqrt(C/M); Poiseuille kinetic-energy factor 4/3 on M at low
        Womersley number, -> 1 (plug flow) at high Womersley number.
  [AL]  Allard & Atalla, "Propagation of Sound in Porous Media" 2nd ed. ch.2:
        static airflow resistivity sigma, layer resistance R = sigma t / A;
        isothermal behaviour of the pore air when the thermal boundary layer
        delta_t = sqrt(2 alpha / w) exceeds the pore size.
  [SU]  Sutherland viscosity law mu = 1.716e-5 (T/273.15)^1.5 (273.15+110.4)/(T+110.4).
  [YL]  Young-Laplace: entry pressure of a non-wetting (theta > 90 deg) square
        channel dp = gamma_w |cos theta| * perimeter / area = 4 gamma_w |cos theta| / a.
  [DS]  Infineon DPS310 datasheet v1.1, Table 16 (measurement time vs PM_PRC):
        t = 2.0 ms + 1.6 ms x N  -> sub-conversion period 1.6 ms.
  [EKF] ArduPilot EKF3 EK3_HGT_DELAY default 60 ms (height measurement delay
        budget the filter already assumes).

Usage: python3 analysis/baro_acoustic_analysis.py [--no-render]
"""
import argparse
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
SCAD = os.path.join(REPO, "cad", "baro_foam_chamber.scad")
STL_DIR = os.path.join(REPO, "stl")
DRV_H = os.path.join(REPO, "firmware", "baro_dps310", "dps310_robust.h")
DRV_CPP = os.path.join(REPO, "firmware", "baro_dps310", "dps310_robust.cpp")
HWDEF = os.path.join(REPO, "ardupilot", "hwdef.dat")
PARM = os.path.join(REPO, "ardupilot", "defaults.parm")
THERMAL = os.path.join(HERE, "thermal_report.json")

MM = 1e-3
G0 = 9.80665
R_AIR = 287.05          # J/(kg K), dry air
GAMMA = 1.4
P_STD = 101325.0

# ----------------------------------------------------------------------------
# Assumptions (all copied into the report).  Geometry and materials that are
# part of the design live in the .scad; these are environment / model inputs.
# ----------------------------------------------------------------------------
A = {
    "env": {
        "T_C": [-20.0, 25.0, 85.0],          # board / air temperature corners (nominal 25)
        "p_amb_Pa": [70000.0, 101325.0, 105000.0],   # 3000 m AMSL .. high-pressure sea level
        "p_sealevel_band_Pa": [95000.0, 105000.0],   # where the lag-error limit is applied
        "note": "C ~ 1/p, so low p maximises lag; high p minimises attenuation",
    },
    "air": {"alpha_thermal_m2_s": 2.2e-5, "note": "thermal diffusivity of air at 25 C"},
    "foam_sigma_factor": [0.5, 1.0, 2.0],    # task sweep x0.5..x2
    "inertance_profile": [1.0, 4.0 / 3.0],   # plug .. Poiseuille kinetic-energy factor [BE]
    "prop": {
        "band_Hz": [100.0, 800.0],
        "band_note": "5-inch class blade-pass B*rpm/60: 2-blade 3000 rpm (idle) = 100 Hz .. "
                     "3-blade 16000 rpm = 800 Hz",
        "check_to_Hz": 2000.0,               # also cover the 2nd oversampling comb lobe
        "spl_dB": 114.0,                      # tonal SPL at the FC, near field of the props
        "spl_note": "~95 dB at 1 m for a 5-inch quad at high throttle, +20 dB spherical spreading "
                    "to ~0.1 m from the disc edge; taken flat over the band (conservative)",
    },
    "residual_frac_of_gate_margin": 0.25,    # alias residual <= 25 % of the driver gate_margin_pa
    "climb_mps": [30.0, 10.0],               # 30 m/s as in test_dps310_robust.cpp T10
    "lag_err_max_m_at_30mps": 0.5,           # applied over p_sealevel_band
    "tau_max_s": 0.020,                      # vs 31.25 ms sample period and EK3_HGT_DELAY 60 ms
    "gain_max": 1.02,                        # no resonant amplification
    "leak": {"len_note": "pinhole through the bond bead across the wall foot (length = wall_t)",
             "spec_factor": 0.5,             # spec = half of the diameter that halves attenuation
             "slot_w_mm": 1.0},
    "water": {
        "gamma_N_m": 0.072, "rho_w": 1000.0,
        "v_forward_mps": 30.0, "v_propwash_mps": 25.0,
        "dive_m": 150.0,                      # port wetted during a 150 m dive (pessimistic)
        "max_penetration_frac": 0.5,
    },
    "foam": {"cut_tol_mm": 0.1, "compression_set": 0.5, "sigma_comp_exponent": 2.0,
             "max_comp": 0.25,
             "note": "die-cut tolerance +-0.1 mm on each dimension; 50 % compression set "
                     "(hot, long term); sigma ~ sigma0/(1-eps)^2 per compressed axis [AL]"},
    "thermal": {"hdt_margin_C": 5.0, "scenarios_prefix": "R",
                "dps310_max_C": 85.0, "foam_pu_cont_C": 100.0},
}

GATES = []
WARNS = []


def gate(name, ok, detail):
    GATES.append({"gate": name, "pass": bool(ok), "detail": detail})
    print("  [%s] %s: %s" % ("PASS" if ok else "FAIL", name, detail))
    return ok


def warn(name, detail):
    WARNS.append({"warn": name, "detail": detail})
    print("  [WARN] %s: %s" % (name, detail))


# ----------------------------------------------------------------------------
# 1. SCAD parsing and safe evaluation
# ----------------------------------------------------------------------------
def scad_statements(text):
    """Top-level statements (brace depth 0), comments stripped, string-aware."""
    out, buf, i, n, depth = [], [], 0, len(text), 0
    while i < n:
        c = text[i]
        if c == '"':
            j = i + 1
            while j < n and text[j] != '"':
                j += 2 if text[j] == "\\" else 1
            if depth == 0:
                buf.append(text[i:j + 1])
            i = j + 1
            continue
        if text.startswith("//", i):
            i = text.find("\n", i)
            i = n if i < 0 else i
            continue
        if text.startswith("/*", i):
            i = text.find("*/", i) + 2
            continue
        if c == "{":
            depth += 1
            if depth == 1:
                buf = []          # drop "module x(...)" header
        elif c == "}":
            depth -= 1
        elif depth == 0:
            if c == ";":
                st = "".join(buf).strip()
                if st:
                    out.append(st)
                buf = []
            else:
                buf.append(c)
        i += 1
    return out


TOK = re.compile(r"\s*(?:(\d+\.\d*(?:[eE][-+]?\d+)?|\.\d+(?:[eE][-+]?\d+)?|\d+(?:[eE][-+]?\d+)?)"
                 r"|(\"(?:[^\"\\]|\\.)*\")|([A-Za-z_$][A-Za-z_0-9$]*)|(==|!=|<=|>=|&&|\|\||[-+*/%<>!?:(),\[\]]))")


def tokenize(s):
    toks, pos = [], 0
    s = s.strip()
    while pos < len(s):
        m = TOK.match(s, pos)
        if not m or m.end() == pos:
            raise ValueError("bad token at %r" % s[pos:pos + 20])
        num, st, ident, op = m.groups()
        if num is not None:
            toks.append(("num", float(num)))
        elif st is not None:
            toks.append(("str", st[1:-1]))
        elif ident is not None:
            toks.append(("id", ident))
        else:
            toks.append(("op", op))
        pos = m.end()
        while pos < len(s) and s[pos].isspace():
            pos += 1
    return toks


def _deg(f):
    return lambda x: f(math.radians(x))


FUNCS = {
    "sqrt": math.sqrt, "pow": math.pow, "abs": abs, "floor": math.floor, "ceil": math.ceil,
    "round": lambda x: math.floor(x + 0.5), "max": max, "min": min,
    "atan": lambda x: math.degrees(math.atan(x)), "atan2": lambda y, x: math.degrees(math.atan2(y, x)),
    "sin": _deg(math.sin), "cos": _deg(math.cos), "tan": _deg(math.tan),
}


class Expr:
    """Recursive-descent evaluator for the OpenSCAD expression subset."""

    def __init__(self, toks, env):
        self.t, self.i, self.env = toks, 0, env

    def peek(self):
        return self.t[self.i] if self.i < len(self.t) else (None, None)

    def take(self, op=None):
        tk = self.peek()
        if op is not None and tk != ("op", op):
            raise ValueError("expected %r got %r" % (op, tk))
        self.i += 1
        return tk

    def parse(self):
        v = self.ternary()
        if self.i != len(self.t):
            raise ValueError("trailing tokens %r" % (self.t[self.i:],))
        return v

    def ternary(self):
        c = self.lor()
        if self.peek() == ("op", "?"):
            self.take("?")
            a = self.ternary()
            self.take(":")
            b = self.ternary()
            return a if c else b
        return c

    def _binary(self, sub, ops):
        v = sub()
        while self.peek()[0] == "op" and self.peek()[1] in ops:
            op = self.take()[1]
            w = sub()
            v = ops[op](v, w)
        return v

    def lor(self):
        return self._binary(self.land, {"||": lambda a, b: bool(a) or bool(b)})

    def land(self):
        return self._binary(self.eq, {"&&": lambda a, b: bool(a) and bool(b)})

    def eq(self):
        return self._binary(self.rel, {"==": lambda a, b: a == b, "!=": lambda a, b: a != b})

    def rel(self):
        return self._binary(self.add, {"<": lambda a, b: a < b, ">": lambda a, b: a > b,
                                       "<=": lambda a, b: a <= b, ">=": lambda a, b: a >= b})

    def add(self):
        return self._binary(self.mul, {"+": lambda a, b: a + b, "-": lambda a, b: a - b})

    def mul(self):
        return self._binary(self.unary, {"*": lambda a, b: a * b, "/": lambda a, b: a / b,
                                         "%": lambda a, b: math.fmod(a, b)})

    def unary(self):
        tk = self.peek()
        if tk == ("op", "-"):
            self.take()
            return -self.unary()
        if tk == ("op", "+"):
            self.take()
            return self.unary()
        if tk == ("op", "!"):
            self.take()
            return not self.unary()
        return self.primary()

    def primary(self):
        kind, val = self.take()
        if kind in ("num", "str"):
            return val
        if kind == "op" and val == "(":
            v = self.ternary()
            self.take(")")
            return v
        if kind == "id":
            if val in ("true", "false"):
                return val == "true"
            if self.peek() == ("op", "("):
                self.take("(")
                args = []
                if self.peek() != ("op", ")"):
                    args.append(self.ternary())
                    while self.peek() == ("op", ","):
                        self.take(",")
                        args.append(self.ternary())
                self.take(")")
                if val not in FUNCS:
                    raise ValueError("function %s not allowed" % val)
                return FUNCS[val](*args)
            if val not in self.env:
                raise KeyError("undefined %s" % val)
            return self.env[val]
        raise ValueError("unexpected %r" % ((kind, val),))


def split_args(s):
    """Split 'a, b' at top-level commas (no parentheses / strings)."""
    out, depth, cur, ins = [], 0, [], False
    for c in s:
        if c == '"':
            ins = not ins
        if not ins:
            if c in "([":
                depth += 1
            elif c in ")]":
                depth -= 1
            elif c == "," and depth == 0:
                out.append("".join(cur))
                cur = []
                continue
        cur.append(c)
    out.append("".join(cur))
    return out


def parse_scad(path):
    text = open(path).read()
    env, order, asserts = {}, [], []
    asg = re.compile(r"^([A-Za-z_$][A-Za-z_0-9$]*)\s*=\s*(.+)$", re.S)
    for st in scad_statements(text):
        if st.startswith(("module", "function", "if", "else", "echo")):
            continue
        if st.startswith("assert"):
            inner = st[st.index("(") + 1: st.rindex(")")]
            parts = split_args(inner)
            asserts.append((parts[0].strip(), parts[1].strip().strip('"') if len(parts) > 1 else ""))
            continue
        m = asg.match(st)
        if m:
            name, expr = m.group(1), " ".join(m.group(2).split())
            env[name] = Expr(tokenize(expr), env).parse()
            order.append((name, expr))
    return env, order, asserts, text


def openscad_bin():
    return shutil.which("openscad")


def openscad_echo(scad):
    exe = openscad_bin()
    if not exe:
        return None, "openscad not found"
    with tempfile.TemporaryDirectory() as td:
        out = os.path.join(td, "x.echo")
        r = subprocess.run([exe, "-o", out, scad], capture_output=True, text=True, timeout=300)
        txt = open(out).read() if os.path.exists(out) else ""
        return txt + r.stderr, None


def parse_echo(txt):
    m = re.search(r'ECHO: "BAROCAP (.*?)"', txt)
    if not m:
        return None
    vals = {}
    for k, v in re.findall(r"(\w+)=(\S+)", m.group(1)):
        vals[k] = [float(x) for x in v.split("x")]
    return vals


# ----------------------------------------------------------------------------
# 2. Rendering and mesh checks
# ----------------------------------------------------------------------------
def render(part, out):
    exe = openscad_bin()
    r = subprocess.run([exe, "-o", out, "-D", 'part="%s"' % part, SCAD],
                       capture_output=True, text=True, timeout=900)
    log = r.stdout + r.stderr
    simple = re.search(r"Simple:\s+(\w+)", log)
    vols = re.search(r"Volumes:\s+(\d+)", log)
    return {"rc": r.returncode, "simple": simple.group(1) if simple else None,
            "volumes": int(vols.group(1)) if vols else None,
            "warnings": [ln for ln in log.splitlines() if "WARNING" in ln or "ERROR" in ln]}


def read_stl(path):
    data = open(path, "rb").read()
    tris = []
    if data[:5] == b"solid" and b"facet" in data[:512]:
        v = [tuple(float(x) for x in ln.split()[1:4]) for ln in data.decode().splitlines()
             if ln.strip().startswith("vertex")]
        tris = [v[i:i + 3] for i in range(0, len(v), 3)]
    else:
        import struct
        n = struct.unpack("<I", data[80:84])[0]
        for k in range(n):
            f = struct.unpack("<12f", data[84 + 50 * k: 84 + 50 * k + 48])
            tris.append([tuple(f[3:6]), tuple(f[6:9]), tuple(f[9:12])])
    return np.array(tris, dtype=float)


def mesh_stats(tris):
    """Volume (divergence theorem), bbox, edge-manifoldness and vertex-connected shells."""
    v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]
    vol = float(np.einsum("ij,ij->i", v0, np.cross(v1, v2)).sum() / 6.0)
    pts = tris.reshape(-1, 3)
    key = {}
    idx = np.array([key.setdefault(tuple(np.round(p, 5)), len(key)) for p in pts]).reshape(-1, 3)
    parent = list(range(len(key)))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    edges = {}
    for a, b, c in idx:
        for u, w in ((a, b), (b, c), (c, a)):
            ru, rw = find(u), find(w)
            if ru != rw:
                parent[ru] = rw
            e = (min(u, w), max(u, w))
            edges[e] = edges.get(e, 0) + 1
    shells = len({find(i) for i in range(len(key))})
    bad_edges = sum(1 for c in edges.values() if c != 2)
    comp_of = np.array([find(i) for i in idx[:, 0]])
    return {"volume_mm3": vol, "bbox_min": pts.min(axis=0).tolist(), "bbox_max": pts.max(axis=0).tolist(),
            "triangles": int(len(tris)), "shells": shells, "non2_edges": bad_edges,
            "_pts": pts, "_comp_tri": comp_of}


# ----------------------------------------------------------------------------
# 3. Firmware / ArduPilot sampling
# ----------------------------------------------------------------------------
def driver_sampling():
    h = open(DRV_H).read()
    cpp = open(DRV_CPP).read()

    def cfg(name, typ=float):
        m = re.search(r"\b%s\s*=\s*([-0-9.]+)f?\s*[;,]" % name, h)
        return typ(m.group(1))
    pm_rate, pm_prc = cfg("pm_rate", int), cfg("pm_prc", int)
    tmp_rate, tmp_prc = cfg("tmp_rate", int), cfg("tmp_prc", int)
    gate_margin = cfg("gate_margin_pa")
    max_vz = cfg("max_vz_mps")
    tm = re.search(r"measurement_time_us\(uint8_t prc\)\s*\{[^}]*?\{([^}]*)\}", cpp, re.S)
    t_meas = [int(x) for x in tm.group(1).split(",")]
    n_os = 1 << pm_prc
    t_sub = (t_meas[pm_prc] - t_meas[0]) / (n_os - 1) * 1e-6 if n_os > 1 else t_meas[0] * 1e-6
    hw = open(HWDEF).read()
    baro_line = re.search(r"^BARO\s+(.*)$", hw, re.M)
    parm = open(PARM).read() if os.path.exists(PARM) else ""
    parm_baro = [ln for ln in parm.splitlines() if re.match(r"\s*(BARO|EK3_HGT_DELAY|EK3_ALT_M_NSE)", ln)]
    return {"pm_rate_code": pm_rate, "pm_prc_code": pm_prc, "tmp_rate_code": tmp_rate, "tmp_prc_code": tmp_prc,
            "fs_Hz": float(1 << pm_rate), "oversampling": n_os, "t_meas_s": t_meas[pm_prc] * 1e-6,
            "t_sub_s": t_sub, "comb_lobe_Hz": 1.0 / t_sub, "gate_margin_pa": gate_margin,
            "max_vz_mps": max_vz, "hwdef_baro": baro_line.group(1).strip() if baro_line else None,
            "defaults_parm_baro_lines": parm_baro,
            "budget_ms_per_s": ((1 << pm_rate) * t_meas[pm_prc] + (1 << tmp_rate) * t_meas[tmp_prc]) / 1e3}


def os_envelope(f, n, t_sub):
    """Upper envelope of the N-point boxcar (point-sample oversampling, no credit for integration
    inside a sub-conversion): |D_N| <= min(1, 1/(N |sin(pi f Ts)|)); main lobes at k/Ts."""
    s = np.abs(np.sin(np.pi * f * t_sub))
    with np.errstate(divide="ignore"):
        env = np.where(s > 0, 1.0 / (n * s), 1.0)
    return np.minimum(1.0, env)


# ----------------------------------------------------------------------------
# 4. Acoustic model
# ----------------------------------------------------------------------------
def mu_air(T_C):
    T = T_C + 273.15
    return 1.716e-5 * (T / 273.15) ** 1.5 * (273.15 + 110.4) / (T + 110.4)     # [SU]


def geometry_volumes(g, stl_air_mm3=None):
    """Air volumes (mm^3).  The air part = cavity minus U4, passives and ledges."""
    cav_xy = (g["cav_x1"] - g["cav_x0"]) * (g["cav_y1"] - g["cav_y0"])
    u4 = g["u4_w"] * g["u4_l"] * g["u4_h"]
    pas = g["n_pass"] * g["pass_w"] * g["pass_l"] * g["pass_h"]
    ledges = 2 * (g["cav_x1"] - g["cav_x0"]) * g["ledge_w"] * g["ledge_h"]
    air_analytic = cav_xy * g["cav_h"] - u4 - pas - ledges
    air = stl_air_mm3 if stl_air_mm3 else air_analytic
    below = cav_xy * g["foam_z0"] - u4 - pas - ledges          # node 2: gap region (ledges sit below foam_z0)
    above = air - below                                         # foam region
    free = ((g["cav_x1"] - g["cav_x0"] + 2 * g["foam_fit"]) * (g["cav_y1"] - g["cav_y0"] + 2 * g["foam_fit"])
            * (g["foam_t"] + g["foam_fit_z"]))
    solid = free * g["foam_density"] / g["foam_solid_density"]
    vent = g["vent_a"] ** 2 * g["vent_len"]
    return {"air_analytic_mm3": air_analytic, "air_used_mm3": air, "gap_mm3": below,
            "foam_region_mm3": above, "foam_solid_mm3": solid, "foam_pore_air_mm3": above - solid,
            "vent_mm3": vent, "porosity": 1 - g["foam_density"] / g["foam_solid_density"]}


def foam_compression(g):
    ex = 2 * g["foam_fit"] / (g["cav_x1"] - g["cav_x0"] + 2 * g["foam_fit"])
    ey = 2 * g["foam_fit"] / (g["cav_y1"] - g["cav_y0"] + 2 * g["foam_fit"])
    ez = g["foam_fit_z"] / (g["foam_t"] + g["foam_fit_z"])
    k = A["foam"]["sigma_comp_exponent"]
    fac = 1.0 / ((1 - ex) * (1 - ey) * (1 - ez)) ** (k / 3.0)
    return {"eps_x": ex, "eps_y": ey, "eps_z": ez, "sigma_factor": fac}


def elements(g, vols, a_mm, T_C, p, iso, sig_f, prof, vent_share):
    """Lumped elements, SI.  Returns dict."""
    mu = mu_air(T_C)
    rho = p / (R_AIR * (T_C + 273.15))
    a = a_mm * MM
    L = g["vent_len"] * MM
    S = a * a
    r_eq = a / math.sqrt(math.pi)                         # equal-area radius for the end terms
    R_v = 28.45 * mu * L / a ** 4 + 2 * 1.5 * mu / r_eq ** 3      # [SL] + [SA] both ports
    M_v = prof * rho * L / S + 2 * rho * 0.85 * r_eq / S          # [BE] + [KI] flanged ends
    kappa = 1.0 if iso else GAMMA
    C1 = (vols["foam_pore_air_mm3"] + (vent_share * vols["vent_mm3"])) * MM ** 3 / (kappa * p)   # [KI]
    C2 = vols["gap_mm3"] * MM ** 3 / (kappa * p)
    comp = foam_compression(g)
    sigma = g["foam_sigma"] * sig_f * comp["sigma_factor"]
    t_path = (g["vent_z_in"] - g["foam_z0"]) * MM          # inner port height -> foam underside
    A_f = (g["cav_x1"] - g["cav_x0"]) * (g["cav_y1"] - g["cav_y0"]) * MM ** 2
    R_f = sigma * t_path / A_f                            # [AL]
    return {"R_v": R_v, "M_v": M_v, "C1": C1, "C2": C2, "R_f": R_f, "mu": mu, "rho": rho}


def H_of(f, e, leak=None):
    """Transfer p_sensor / p_out.  Nodal analysis, optional leak branch outside -> node 2."""
    w = 2 * np.pi * np.asarray(f, dtype=float)
    Zv = e["R_v"] + 1j * w * e["M_v"]
    Yv = 1.0 / Zv
    Yf = 1.0 / e["R_f"]
    Y1 = 1j * w * e["C1"]
    Y2 = 1j * w * e["C2"]
    Yl = 0.0
    if leak is not None:
        Yl = 1.0 / (leak["R"] + 1j * w * leak["M"])
    # [Yv+Yf+Y1, -Yf; -Yf, Yf+Y2+Yl] [p1;p2] = [Yv; Yl]
    a11, a12, a22 = Yv + Yf + Y1, -Yf, Yf + Y2 + Yl
    det = a11 * a22 - a12 * a12
    p2 = (a11 * Yl - a12 * Yv) / det
    return p2


def tau_dc(e, leak=None):
    """Ramp lag = first moment of the impulse response = -d(phase)/dw at w -> 0.
    No leak: R_v (C1 + C2) + R_f C2."""
    if leak is None:
        return e["R_v"] * (e["C1"] + e["C2"]) + e["R_f"] * e["C2"]
    f = np.array([1e-3])
    h = H_of(f, e, leak)
    return float(-np.angle(h)[0] / (2 * np.pi * f[0]))


def helmholtz(e):
    C = e["C1"] + e["C2"]
    w0 = 1.0 / math.sqrt(e["M_v"] * C)
    zeta = 0.5 * (e["R_v"] + e["R_f"]) * math.sqrt(C / e["M_v"])
    return w0 / (2 * math.pi), zeta


def leak_element(d_m, L_m, e):
    r = d_m / 2
    R = 128 * e["mu"] * L_m / (math.pi * d_m ** 4) + 3 * e["mu"] / r ** 3    # Poiseuille + [SA]
    M = e["rho"] * (L_m + 2 * 0.85 * r) / (math.pi * r * r)
    return {"R": R, "M": M}


# ----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-render", action="store_true", help="skip OpenSCAD STL rendering")
    args = ap.parse_args()
    rep = {"revision": "12.0", "assumptions": A}

    # ------------------------------------------------------------------ 1
    print("== 1. SCAD parse / evaluate ==")
    g, order, asserts, scad_text = parse_scad(SCAD)
    nnum = sum(1 for _, v in g.items() if isinstance(v, (int, float)) and not isinstance(v, bool))
    gate("P1 SCAD top-level assignments parsed and evaluated", len(order) >= 60 and "vent_len" in g,
         "%d assignments (%d numeric), part=%r" % (len(order), nnum, g.get("part")))
    bad = []
    for cond, msg in asserts:
        try:
            ok = bool(Expr(tokenize(" ".join(cond.split())), g).parse())
        except Exception as ex:     # noqa: BLE001
            ok, msg = False, "%s (eval error %s)" % (msg, ex)
        if not ok:
            bad.append(msg)
    gate("P2 SCAD asserts re-evaluated in Python", not bad and len(asserts) >= 20,
         "%d asserts, failing: %s" % (len(asserts), bad or "none"))
    echo_txt, why = openscad_echo(SCAD)
    echo = parse_echo(echo_txt) if echo_txt else None
    if echo:
        pairs = {"cav": [g["cav_x1"] - g["cav_x0"], g["cav_y1"] - g["cav_y0"], g["cav_h"]],
                 "foam_t": [g["foam_t"]], "vent_len": [g["vent_len"]], "vent_pitch": [g["vent_pitch"]],
                 "vent_slope_deg": [g["vent_slope_deg"]]}
        worst = max(abs(a - b) / max(abs(b), 1e-9) for k, v in pairs.items() for a, b in zip(echo[k], v))
        err_lines = [ln for ln in echo_txt.splitlines() if "ERROR" in ln or "Assertion" in ln]
        gate("P3 Python evaluation == OpenSCAD ECHO (cav, foam_t, vent_len, pitch, slope)",
             worst < 2e-5 and not err_lines,
             "max rel diff %.1e (OpenSCAD prints 6 significant digits); vent_len %.4f mm; openscad errors: %s"
             % (worst, g["vent_len"], err_lines or "none"))
    else:
        print("  [SKIP] P3 ECHO cross-check: %s" % (why or "no ECHO line"))
    rep["scad"] = {k: v for k, v in g.items() if isinstance(v, (int, float, str))}
    rep["echo"] = echo

    # ------------------------------------------------------------------ 2
    print("== 2. render + mesh checks ==")
    stl_air = None
    mesh = {}
    if args.no_render or not openscad_bin():
        print("  [SKIP] rendering (%s)" % ("--no-render" if args.no_render else "openscad not found"))
    else:
        os.makedirs(STL_DIR, exist_ok=True)
        for part in ("cap", "foam_block", "air", "vent_air"):
            out = os.path.join(STL_DIR, "baro_%s.stl" % part)
            r = render(part, out)
            st = mesh_stats(read_stl(out))
            r.update({k: v for k, v in st.items() if not k.startswith("_")})
            r["file"] = os.path.relpath(out, REPO)
            mesh[part] = (r, st)
            print("    %-10s CGAL simple=%s volumes=%s  STL tris %5d shells %d non-2-edges %d  V %.3f mm^3"
                  % (part, r["simple"], r["volumes"], st["triangles"], st["shells"], st["non2_edges"],
                     st["volume_mm3"]))
        for part in ("cap", "foam_block", "air", "vent_air"):
            r, st = mesh[part]
            # a lone primitive (foam_block) bypasses CGAL, so OpenSCAD prints no Simple/Volumes lines;
            # the STL edge/shell check then carries the manifold proof.
            cgal_ok = (r["simple"] == "yes" and r["volumes"] == 2) or (r["simple"] is None and part == "foam_block")
            gate("R1 %s renders manifold, one solid" % part,
                 r["rc"] == 0 and cgal_ok and st["non2_edges"] == 0 and st["shells"] == 1 and not r["warnings"],
                 "rc %d, Simple: %s, CGAL Volumes: %s (outer + 1 solid), %d non-2 edges, %d STL shells%s"
                 % (r["rc"], r["simple"], r["volumes"], st["non2_edges"], st["shells"],
                    ", warnings %s" % r["warnings"] if r["warnings"] else ""))
        r, st = mesh["vent_air"]
        x_in = g["cav_x1"]
        x_out = g["cap_w"] / 2
        pts = st["_pts"]
        gate("R2 vent_air is ONE connected volume", st["shells"] == 1 and r["volumes"] == 2,
             "%d vertex-connected STL shell(s), CGAL volumes %d" % (st["shells"], r["volumes"]))
        y_in_ok = pts[pts[:, 0] <= x_in + 1e-6]
        y_out_ok = pts[pts[:, 0] >= x_out - 1e-6]
        zin = (y_in_ok[:, 2].min(), y_in_ok[:, 2].max()) if len(y_in_ok) else (0, 0)
        zout = (y_out_ok[:, 2].min(), y_out_ok[:, 2].max()) if len(y_out_ok) else (0, 0)
        gate("R3 vent_air pierces the cavity face and the outer face",
             st["bbox_min"][0] < x_in and st["bbox_max"][0] > x_out and len(y_in_ok) and len(y_out_ok)
             and abs(0.5 * sum(zin) - g["vent_z_in"]) < 1e-3 and abs(0.5 * sum(zout) - g["vent_z_out"]) < 1e-3,
             "x %.3f..%.3f vs faces %.3f / %.3f; inner opening z %.3f..%.3f (vent_z_in %.2f), "
             "outer opening z %.3f..%.3f (vent_z_out %.2f)"
             % (st["bbox_min"][0], st["bbox_max"][0], x_in, x_out, zin[0], zin[1], g["vent_z_in"],
                zout[0], zout[1], g["vent_z_out"]))
        vv = st["volume_mm3"]
        v_line = g["vent_a"] ** 2 * g["vent_len"]
        gate("R4 vent_air volume ~ a^2 * vent_len (centreline length is right)",
             abs(vv / v_line - 1) < 0.08,
             "STL %.3f mm^3 vs a^2 L %.3f mm^3 (%+.1f %%; joints/stub overhang)"
             % (vv, v_line, 100 * (vv / v_line - 1)))
        stl_air = mesh["air"][1]["volume_mm3"]
        va = geometry_volumes(g)["air_analytic_mm3"]
        gate("R5 air STL volume == analytic cavity air", abs(stl_air / va - 1) < 0.01,
             "STL %.3f vs %.3f mm^3" % (stl_air, va))
        rc, stc = mesh["cap"]
        cap_bb_ok = (abs(stc["bbox_max"][2] - g["cap_h"]) < 1e-3 and abs(stc["bbox_min"][0] + g["cap_w"] / 2) < 1e-3
                     and abs(stc["bbox_max"][1] - g["cap_l"] / 2) < 1e-3)
        gate("R6 cap STL bbox == cap_w x cap_l x cap_h", cap_bb_ok,
             "bbox %s .. %s" % (np.round(stc["bbox_min"], 3).tolist(), np.round(stc["bbox_max"], 3).tolist()))
        fb = mesh["foam_block"][1]
        exp_fb = ((g["cav_x1"] - g["cav_x0"] + 2 * g["foam_fit"]) * (g["cav_y1"] - g["cav_y0"] + 2 * g["foam_fit"])
                  * (g["foam_t"] + g["foam_fit_z"]))
        gate("R7 foam_block cut template volume", abs(fb["volume_mm3"] / exp_fb - 1) < 1e-3,
             "%.3f vs %.3f mm^3" % (fb["volume_mm3"], exp_fb))
        rep["render"] = {p: mesh[p][0] for p in mesh}

    # ------------------------------------------------------------------ 3
    print("== 3. DPS310 sampling (driver / ArduPilot) ==")
    ds = driver_sampling()
    print("    PM_RATE %d -> %.0f Hz, PM_PRC %d -> %dx, t_meas %.1f ms, sub-conversion %.2f ms -> comb lobes at "
          "k x %.0f Hz; hwdef 'BARO %s'; defaults.parm baro/EKF height lines: %s"
          % (ds["pm_rate_code"], ds["fs_Hz"], ds["pm_prc_code"], ds["oversampling"], ds["t_meas_s"] * 1e3,
             ds["t_sub_s"] * 1e3, ds["comb_lobe_Hz"], ds["hwdef_baro"], ds["defaults_parm_baro_lines"] or "none"))
    gate("S1 sampling plan read from the driver and consistent",
             ds["hwdef_baro"] is not None and "DPS310" in ds["hwdef_baro"] and ds["budget_ms_per_s"] < 1000
             and ds["t_meas_s"] < 1.0 / ds["fs_Hz"],
             "%.0f Hz x %dx, background budget %.0f ms/s, t_meas %.1f ms < period %.2f ms"
             % (ds["fs_Hz"], ds["oversampling"], ds["budget_ms_per_s"], ds["t_meas_s"] * 1e3, 1e3 / ds["fs_Hz"]))
    rep["sampling"] = ds

    # ------------------------------------------------------------------ 4
    print("== 4. acoustic / pneumatic model ==")
    vols = geometry_volumes(g, stl_air)
    comp = foam_compression(g)
    print("    air %.2f mm^3 (gap %.2f, foam region %.2f, foam solid %.2f, porosity %.3f), vent %.2f mm^3"
          % (vols["air_used_mm3"], vols["gap_mm3"], vols["foam_region_mm3"], vols["foam_solid_mm3"],
             vols["porosity"], vols["vent_mm3"]))
    a0, tol = g["vent_a"], g["vent_tol"]
    f = np.unique(np.concatenate([np.logspace(0, math.log10(A["prop"]["check_to_Hz"]), 1500),
                                  np.arange(1, A["prop"]["check_to_Hz"], 1.0)]))
    nom = elements(g, vols, a0, 25.0, P_STD, False, 1.0, 4.0 / 3.0, 0.0)
    nom_iso = elements(g, vols, a0, 25.0, P_STD, True, 1.0, 4.0 / 3.0, 0.5)
    Hn = np.abs(H_of(f, nom))
    fc = float(f[np.argmax(Hn <= 1 / math.sqrt(2))])
    f0, zeta = helmholtz(nom)
    tau_nom_ad, tau_nom_iso = tau_dc(nom), tau_dc(nom_iso)
    att = {int(x): float(-20 * np.log10(np.abs(H_of(np.array([x]), nom))[0]))
           for x in (16, 32, 100, 200, 400, 625, 800)}
    print("    nominal 25 C 101.3 kPa: R_vent %.3e Pa s/m^3 (Poiseuille %.1f %%), M_vent %.3e kg/m^4, R_foam %.3e, "
          "C1 %.3e C2 %.3e m^3/Pa (adiabatic)"
          % (nom["R_v"], 100 * (1 - 3 * nom["mu"] / (a0 * MM / math.sqrt(math.pi)) ** 3 / nom["R_v"]),
             nom["M_v"], nom["R_f"], nom["C1"], nom["C2"]))
    print("    tau %.2f ms (adiabatic) / %.2f ms (isothermal + half vent volume), fc %.1f Hz, Helmholtz f0 %.0f Hz "
          "zeta %.2f" % (tau_nom_ad * 1e3, tau_nom_iso * 1e3, fc, f0, zeta))
    print("    attenuation nominal: " + ", ".join("%d Hz %.1f dB" % kv for kv in att.items()))
    alpha = A["air"]["alpha_thermal_m2_s"]
    pore = 25.4 / g["foam_ppi"]
    dth = {fx: 1e3 * math.sqrt(2 * alpha / (2 * math.pi * fx)) for fx in (10, 100, 625)}
    print("    thermal boundary layer %s mm vs pore %.2f mm -> isothermal below ~100 Hz, "
          "adiabatic bound used for attenuation, isothermal bound for lag" % (
              {k: round(v, 3) for k, v in dth.items()}, pore))

    # corner sweep
    corners = []
    for a_mm in (a0 - tol, a0, a0 + tol):
        for sf in A["foam_sigma_factor"]:
            for T in A["env"]["T_C"]:
                for p in A["env"]["p_amb_Pa"]:
                    for iso in (False, True):
                        for prof in A["inertance_profile"]:
                            e_att = elements(g, vols, a_mm, T, p, iso, sf, prof, 0.0)
                            e_lag = elements(g, vols, a_mm, T, p, iso, sf, prof, 0.5)
                            corners.append(((a_mm, sf, T, p, iso, prof), e_att, e_lag))
    band = (f >= A["prop"]["band_Hz"][0]) & (f <= A["prop"]["check_to_Hz"])
    P_in = 20e-6 * 10 ** (A["prop"]["spl_dB"] / 20) * math.sqrt(2)
    G_os = os_envelope(f, ds["oversampling"], ds["t_sub_s"])
    res_lim = A["residual_frac_of_gate_margin"] * ds["gate_margin_pa"]
    worst = {"gain": (0, None), "res": (0, None, None), "tau": (0, None), "lag_sl": (0, None),
             "att": {}, "f0z": (1e9, None, None)}
    for key, e_att, e_lag in corners:
        H = np.abs(H_of(f, e_att))
        gmax = float(H.max())
        if gmax > worst["gain"][0]:
            worst["gain"] = (gmax, key)
        res = P_in * H[band] * G_os[band]
        k = int(np.argmax(res))
        if res[k] > worst["res"][0]:
            worst["res"] = (float(res[k]), key, float(f[band][k]))
        for fx in (100, 200, 400, 625, 800):
            h = float(np.abs(H_of(np.array([fx]), e_att))[0])
            if h > worst["att"].get(fx, (0, None))[0]:
                worst["att"][fx] = (h, key)
        t = tau_dc(e_lag)
        if t > worst["tau"][0]:
            worst["tau"] = (t, key)
        if A["env"]["p_sealevel_band_Pa"][0] <= key[3] <= A["env"]["p_sealevel_band_Pa"][1] and t > worst["lag_sl"][0]:
            worst["lag_sl"] = (t, key)
        f0c, zc = helmholtz(e_att)
        if zc < worst["f0z"][0]:
            worst["f0z"] = (zc, f0c, key)

    def ck(key):
        return "a=%.2f mm sigma x%.1f T=%.0f C p=%.0f kPa %s M-profile %.2f" % (
            key[0], key[1], key[2], key[3] / 1e3, "isothermal" if key[4] else "adiabatic", key[5])
    gate("A1 no resonant amplification |H| <= %.2f, all corners" % A["gain_max"], worst["gain"][0] <= A["gain_max"],
         "max |H| %.4f (%s); min zeta %.2f at f0 %.0f Hz" % (worst["gain"][0], ck(worst["gain"][1]),
                                                             worst["f0z"][0], worst["f0z"][1]))
    gate("A2 prop band %g..%g Hz residual after chamber x DPS310 %dx comb <= %.1f Pa, all corners"
         % (A["prop"]["band_Hz"][0], A["prop"]["check_to_Hz"], ds["oversampling"], res_lim),
         worst["res"][0] <= res_lim,
         "worst %.2f Pa at %.0f Hz (%s); input %.1f dB SPL = %.1f Pa peak; limit = %.0f %% of driver "
         "gate_margin_pa %.0f Pa" % (worst["res"][0], worst["res"][2], ck(worst["res"][1]), A["prop"]["spl_dB"],
                                     P_in, 100 * A["residual_frac_of_gate_margin"], ds["gate_margin_pa"]))
    att_worst = {fx: -20 * math.log10(v[0]) for fx, v in worst["att"].items()}
    print("    worst-corner attenuation: " + ", ".join("%d Hz %.1f dB" % (k, v) for k, v in att_worst.items()))
    tw, kw = worst["tau"]
    gate("A3 quasi-static lag tau <= %.0f ms, all corners (incl. 3000 m, 85 C, iso)" % (A["tau_max_s"] * 1e3),
         tw <= A["tau_max_s"], "worst tau %.2f ms (%s) vs period %.2f ms, EK3_HGT_DELAY 60 ms"
         % (tw * 1e3, ck(kw), 1e3 / ds["fs_Hz"]))
    ts, ks = worst["lag_sl"]
    lag30, lag10 = A["climb_mps"][0] * ts, A["climb_mps"][1] * ts
    gate("A4 altitude lag error at 30 m/s <= %.1f m (sea-level band)" % A["lag_err_max_m_at_30mps"],
         lag30 <= A["lag_err_max_m_at_30mps"],
         "%.3f m at 30 m/s, %.3f m at 10 m/s (tau %.2f ms, %s); at 3000 m worst %.3f m at 30 m/s; nominal %.3f m"
         % (lag30, lag10, ts * 1e3, ck(ks), A["climb_mps"][0] * tw, A["climb_mps"][0] * tau_nom_iso))
    gate("A5 chamber sits between EKF-scale lag and blade-pass band (fc > 2 x fs, f0 > fc)",
         fc > 2 * ds["fs_Hz"] and f0 > fc and fc < A["prop"]["band_Hz"][0],
         "fc %.1f Hz, fs %.0f Hz, f0 %.0f Hz, prop band from %.0f Hz" % (fc, ds["fs_Hz"], f0, A["prop"]["band_Hz"][0]))

    # bond leak
    Lk = g["wall_t"] * MM
    fl = ds["comb_lobe_Hz"]
    h0 = float(np.abs(H_of(np.array([fl]), nom))[0])

    def h_leak(d, e):
        return float(np.abs(H_of(np.array([fl]), e, leak_element(d, Lk, e)))[0])
    lo, hi = 1e-6, 1e-3
    for _ in range(80):
        mid = math.sqrt(lo * hi)
        lo, hi = (mid, hi) if h_leak(mid, nom) < 2 * h0 else (lo, mid)
    d_half = hi
    # largest leak that still keeps A2 at the worst residual corner
    kr = worst["res"][1]
    e_r = [c[1] for c in corners if c[0] == kr][0]
    fb_ = f[band]
    Gb = G_os[band]

    def res_leak(d):   # worst residual over the whole band at the worst-residual corner
        return float((P_in * np.abs(H_of(fb_, e_r, leak_element(d, Lk, e_r))) * Gb).max())
    lo2, hi2 = 1e-6, 1e-3
    for _ in range(80):
        mid = math.sqrt(lo2 * hi2)
        lo2, hi2 = (mid, hi2) if res_leak(mid) <= res_lim else (lo2, mid)
    no_chamber = float((P_in * Gb).max())
    gate("A2b the chamber is load-bearing (same residual without it fails)", no_chamber > res_lim,
         "bare sensor residual %.1f Pa at the %.0f Hz comb lobe vs limit %.1f Pa" % (no_chamber, fl, res_lim))
    d_a2 = lo2 if worst["res"][0] <= res_lim else 0.0
    d_spec = min(A["leak"]["spec_factor"] * d_half, d_a2) if d_a2 > 0 else A["leak"]["spec_factor"] * d_half
    lk_spec = leak_element(d_spec, Lk, nom_iso)
    C_tot_iso = nom_iso["C1"] + nom_iso["C2"]
    tau_leakdown = lk_spec["R"] * C_tot_iso
    # equivalent bond slot (width w, height h, length wall_t): R = 12 mu L / (w h^3)
    w_s = A["leak"]["slot_w_mm"] * MM
    h_slot = (12 * nom["mu"] * Lk / (w_s * lk_spec["R"])) ** (1 / 3.0)
    h_spec = float(np.abs(H_of(np.array([fl]), nom, leak_element(d_spec, Lk, nom)))[0])
    gate("A6 bond-leak spec derived (leak at spec keeps A2 and costs < 1 dB at the comb lobe)",
         d_a2 > 0 and 20 * math.log10(h_spec / h0) < 1.0,
         "pinhole halving attenuation at %.0f Hz: d = %.1f um; A2 limit at worst corner: d = %.1f um; "
         "SPEC d_eq <= %.1f um (%.2f dB at spec) == bond slot h <= %.2f um over %.0f mm; acceptance: outer port "
         "sealed, leak-down tau >= %.3f s" % (fl, d_half * 1e6, d_a2 * 1e6, d_spec * 1e6,
                                             20 * math.log10(h_spec / h0), h_slot * 1e6, A["leak"]["slot_w_mm"],
                                             tau_leakdown))
    rep["acoustic"] = {
        "volumes_mm3": vols, "foam_compression": comp,
        "nominal": {"R_vent": nom["R_v"], "M_vent": nom["M_v"], "R_foam": nom["R_f"], "C1": nom["C1"],
                    "C2": nom["C2"], "tau_adiabatic_ms": tau_nom_ad * 1e3, "tau_isothermal_ms": tau_nom_iso * 1e3,
                    "fc_Hz": fc, "f0_Hz": f0, "zeta": zeta, "attenuation_dB": att},
        "thermal_boundary_layer_mm": dth, "pore_mm": pore,
        "worst": {"max_gain": worst["gain"][0], "max_gain_corner": ck(worst["gain"][1]),
                  "residual_Pa": worst["res"][0], "residual_f_Hz": worst["res"][2],
                  "residual_corner": ck(worst["res"][1]), "residual_limit_Pa": res_lim,
                  "input_peak_Pa": P_in, "attenuation_dB": att_worst,
                  "tau_ms": tw * 1e3, "tau_corner": ck(kw),
                  "tau_sealevel_ms": ts * 1e3, "lag_30mps_m": lag30, "lag_10mps_m": lag10,
                  "lag_30mps_3000m_m": A["climb_mps"][0] * tw, "min_zeta": worst["f0z"][0]},
        "leak": {"d_half_um": d_half * 1e6, "d_A2_um": d_a2 * 1e6, "d_spec_um": d_spec * 1e6,
                 "slot_h_spec_um": h_slot * 1e6, "slot_w_mm": A["leak"]["slot_w_mm"],
                 "leakdown_tau_min_s": tau_leakdown},
        "corners": len(corners),
    }

    # ------------------------------------------------------------------ 5
    print("== 5. water ingress ==")
    W = A["water"]
    th = math.radians(g["coat_theta"])
    p_cap = 4 * W["gamma_N_m"] * abs(math.cos(th)) / (g["vent_a"] * MM)          # [YL]
    q_fwd = 0.5 * 1.225 * W["v_forward_mps"] ** 2
    q_pw = 0.5 * 1.225 * W["v_propwash_mps"] ** 2
    dp_dive = 1.225 * G0 * W["dive_m"]
    dp_drive = max(q_fwd, q_pw) + dp_dive - p_cap
    V_tot = (vols["foam_pore_air_mm3"] + vols["gap_mm3"] + vols["vent_mm3"]) * MM ** 3
    dV = V_tot * max(dp_drive, 0) / P_STD                                         # isothermal Boyle
    pen = dV / (g["vent_a"] * MM) ** 2 / MM
    print("    entry pressure %.0f Pa (theta %.0f deg) vs dynamic pressure %.0f Pa (30 m/s) / %.0f Pa (prop wash): "
          "a wetted port CAN be pushed in, the trapped air limits how far" % (p_cap, g["coat_theta"], q_fwd, q_pw))
    gate("W1 hydrophobic channel (theta > 90 deg -> capillary force expels water)", g["coat_theta"] > 90 and p_cap > 0,
         "theta %.0f deg, Young-Laplace entry pressure %.0f Pa" % (g["coat_theta"], p_cap))
    gate("W2 closed-volume penetration <= %.0f %% of the labyrinth" % (100 * W["max_penetration_frac"]),
         pen <= W["max_penetration_frac"] * g["vent_len"],
         "drive %.0f Pa (max dyn. pressure + %.0f m dive - entry), air %.1f mm^3 -> %.2f mm^3 -> %.1f mm of %.1f mm"
         % (dp_drive, W["dive_m"], V_tot / MM ** 3, dV / MM ** 3, pen, g["vent_len"]))
    gate("W3 labyrinth descends monotonically to the outer port (self-draining)",
         g["vent_slope_deg"] >= 5 and g["vent_z_out"] < g["vent_z_in"] and g["vent_pitch"] > 0,
         "every lane %.2f deg down, inner %.2f mm -> outer %.2f mm (lowest point)"
         % (g["vent_slope_deg"], g["vent_z_in"], g["vent_z_out"]))
    rep["water"] = {"p_entry_Pa": p_cap, "q_forward_Pa": q_fwd, "q_propwash_Pa": q_pw, "dp_dive_Pa": dp_dive,
                    "penetration_mm": pen, "vent_len_mm": g["vent_len"]}

    # ------------------------------------------------------------------ 6
    print("== 6. light ==")
    dy = 2 * g["vent_y_end"]
    dz = g["vent_z_in"] - g["vent_z_out"]
    turns = 2 + g["vent_n_cross"] * 2
    gate("L1 no line of sight through the labyrinth", dy > g["vent_a"] and dz > g["vent_a"] and turns >= 3,
         "ports offset %.2f mm in Y and %.2f mm in Z (channel %.2f mm), %d right-angle turns"
         % (dy, dz, g["vent_a"], turns))
    gate("L2 opaque foam covers the full cavity plan with interference (no edge gap)",
         g["foam_fit"] > A["foam"]["cut_tol_mm"] / 2 and g["vent_z_in"] - g["vent_a"] / 2 >= g["foam_z0"],
         "foam oversize %.2f mm/side > cut tol %.2f mm/side; inner port opens into the foam (z >= %.2f mm)"
         % (g["foam_fit"], A["foam"]["cut_tol_mm"] / 2, g["foam_z0"]))
    gap_view = math.degrees(math.atan(g["air_gap"] / max(g["u4_x0"] - g["cav_x0"], 1e-3)))
    opaque_note = bool(re.search(r"must be OPAQUE", scad_text))
    gate("L3 cap opacity requirement recorded in the SCAD material notes", opaque_note,
         "U4 lid port sees the side walls up to %.0f deg elevation through the %.1f mm gap -> walls must be opaque"
         % (gap_view, g["air_gap"]))

    # ------------------------------------------------------------------ 7
    print("== 7. thermal / mechanical fit ==")
    tr = json.load(open(THERMAL))
    sc = {k: v for k, v in tr["scenarios"].items() if k.startswith(A["thermal"]["scenarios_prefix"])}
    t_site = max(v["t_site_C"] for v in sc.values())
    t_board = max(v["t_board_C"] for v in sc.values())
    tj = max(v["tj_hs_C"] for v in sc.values())
    k_site = max(sc, key=lambda k: sc[k]["t_site_C"])
    gate("T1 cap foot (board at the FET site) <= HDT - %.0f C" % A["thermal"]["hdt_margin_C"],
         t_site + A["thermal"]["hdt_margin_C"] <= g["resin_hdt"],
         "max t_site %.1f C (%s) vs HDT %.0f C" % (t_site, k_site, g["resin_hdt"]))
    gap_q11 = (g["q11_cy_k"] - g["q11_l"] / 2) - g["cap_y1_k"]
    gate("T2 Q11 junction bound <= HDT (cap never hotter than the nearest FET junction)", tj <= g["resin_hdt"],
         "max Tj(HS) %.1f C over %s, Q11 courtyard %.2f mm from the cap" % (tj, sorted(sc), gap_q11))
    if t_board > A["thermal"]["dps310_max_C"]:
        warn("DPS310 operating range", "board reaches %.1f C at the rating scenarios > DPS310 %.0f C max: baro "
             "site temperature is a board-level open item (not fixed by the cap)" % (t_board, A["thermal"]["dps310_max_C"]))
    if t_site > A["thermal"]["foam_pu_cont_C"]:
        warn("PU foam temperature", "foam bottom face may reach %.1f C (burst) vs ~%.0f C continuous for polyether "
             "PU; consider black melamine or silicone open-cell foam with the same sigma"
             % (t_site, A["thermal"]["foam_pu_cont_C"]))
    F = A["foam"]
    side_min = g["foam_fit"] - F["cut_tol_mm"] / 2
    z_min = g["foam_fit_z"] - F["cut_tol_mm"]
    gate("M1 foam retention after %.0f %% compression set and cut tolerance" % (100 * F["compression_set"]),
         side_min * (1 - F["compression_set"]) > 0 and z_min * (1 - F["compression_set"]) > 0,
         "residual interference %.3f mm/side (X/Y), %.3f mm (Z)"
         % (side_min * (1 - F["compression_set"]), z_min * (1 - F["compression_set"])))
    gate("M2 foam compression <= %.0f %% and sigma rise within the x2 sweep" % (100 * F["max_comp"]),
         max(comp["eps_x"], comp["eps_y"], comp["eps_z"]) <= F["max_comp"] and comp["sigma_factor"] <= 2.0,
         "eps x %.1f %% y %.1f %% z %.1f %%, sigma x%.2f"
         % (100 * comp["eps_x"], 100 * comp["eps_y"], 100 * comp["eps_z"], comp["sigma_factor"]))
    gate("M3 stack height", g["cap_h"] + g["stack_clear_z"] <= g["stack_free_h"],
         "cap %.2f + %.2f mm <= free %.2f mm" % (g["cap_h"], g["stack_clear_z"], g["stack_free_h"]))
    hole = (g["board_size"] - g["mount_pitch"]) / 2
    hx, hy = g["board_size"] - hole, hole
    dx = max(g["cap_x0_k"] - hx, 0, hx - g["cap_x1_k"])
    dyy = max(g["cap_y0_k"] - hy, 0, hy - g["cap_y1_k"])
    dist = math.hypot(dx, dyy)
    gate("M4 cap clear of the NE grommet keep-out", dist >= g["grommet_keepout_r"] + g["stack_clear"],
         "nearest footprint point %.3f mm from hole (%.2f, %.2f) >= %.2f + %.2f"
         % (dist, hx, hy, g["grommet_keepout_r"], g["stack_clear"]))
    edge = min(g["board_size"] - g["cap_x1_k"], g["cap_y0_k"])
    gq4 = max(g["cap_x0_k"] - (g["q4_cx_k"] + g["q4_w"] / 2), g["cap_y0_k"] - (g["q4_cy_k"] + g["q4_l"] / 2))
    gate("M5 board edge and Q4/Q11 courtyards", edge >= 0.3 and gq4 >= g["court_clear"] and gap_q11 >= g["court_clear"],
         "edge %.2f mm, Q4 gap %.2f mm, Q11 gap %.2f mm (min %.2f)" % (edge, gq4, gap_q11, g["court_clear"]))
    gate("M6 sensor clearances (U4 / passives / foam)",
         g["air_gap"] >= 0.5 and g["u4_x1"] + 0.25 + g["pass_w"] <= g["cav_x1"] - 0.2 and g["foam_t"] >= 3.0,
         "air gap %.2f mm, passives-to-vent-wall %.2f mm, foam %.2f mm"
         % (g["air_gap"], g["cav_x1"] - (g["u4_x1"] + 0.25 + g["pass_w"]), g["foam_t"]))
    rep["thermal"] = {"t_site_max_C": t_site, "t_board_max_C": t_board, "tj_hs_max_C": tj, "scenarios": sorted(sc)}

    nfail = sum(1 for x in GATES if not x["pass"])
    rep.update({"gates": GATES, "warnings": WARNS, "result": "PASS" if nfail == 0 else "FAIL"})
    out = os.path.join(HERE, "baro_acoustic_report.json")
    with open(out, "w") as fh:
        json.dump(rep, fh, indent=1, default=str)
    print("\nwrote %s" % out)
    print("RESULT: %s (%d gates, %d failed)" % (rep["result"], len(GATES), nfail))
    return 1 if nfail else 0


if __name__ == "__main__":
    sys.exit(main())
