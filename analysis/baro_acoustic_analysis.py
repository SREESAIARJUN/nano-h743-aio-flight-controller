#!/usr/bin/env python3
"""
Nano H743 AIO - DPS310 (U4) foam chamber: geometry, acoustic / pneumatic filter,
water, light, thermal and fit check (Revision 12.1)

Single source of geometry: cad/baro_foam_chamber.scad.  Its top-level
"name = expr;" assignments and assert()s are parsed and evaluated here with a
small safe expression evaluator (OpenSCAD subset: + - * / %, comparisons,
&& || !, ?:, sqrt pow atan atan2 sin cos tan abs max min floor ceil round),
so no dimension, material value or design limit is duplicated in this file.
Python 3 + numpy.  Writes analysis/baro_acoustic_report.json, exits 1 if any
gate fails.

Signal path (Rev 12.1):  outside -> ePTFE vent membrane (east face seat) ->
plenum -> single-plane serpentine labyrinth (light baffle / puncture guard)
-> foam region (black melamine) -> air gap -> DPS310 lid port.

Sections
  1. SCAD parse + evaluation, SCAD asserts re-evaluated in Python, and a
     cross-check against OpenSCAD's own ECHO output (openscad -o x.echo).
  2. Render part = cap / foam_block / membrane / air / vent_air to stl/ with
     OpenSCAD (skip with --no-render): CGAL 'Simple: yes', 'Volumes: 2', STL
     edge-manifoldness / shells, vent_air spans cavity face -> plenum floor,
     STL volumes against the analytic values.
  3. FLIGHT driver sampling.  hwdef 'BARO DPS310 I2C:0:0x76' selects stock
     ArduPilot AP_Baro_DPS310 (AP_Baro_DPS280.cpp).  With --ardupilot DIR or
     $ARDUPILOT_ROOT the PCONF / TCONF / CFG_REG writes, the poll rate, the
     driver temperature limit and the EKF3 EK3_ALT_M_NSE / EK3_HGT_DELAY
     defaults are parsed from that tree; otherwise the documented upstream
     constants are used and S1 is reported as SKIP.  The host-test reference
     driver firmware/baro_dps310/dps310_robust.h is NOT in the flight build and
     is only evaluated as an informational secondary check.
  4. Acoustic / pneumatic model:
        p_out --[Z_mem = R_mem || (C_d, M_d) diaphragm]-- n0 (C_plenum)
              --[vent two-port]-- n1 (C1: foam pore air) --[R_foam]-- n2 (C2: gap) = sensor
     optional bond-leak branch outside -> n2.  Vent two-port:
       * TL  : lossy transmission line, Stinson low-reduced-frequency model for
               a rectangular duct (viscous + thermal, single-series closed form),
               + port end impedances  -> used for all gates
       * LUMP: R + jwM (+ vent compliance share) lumped model -> reported for
               comparison (validity check at the 1250 / 1875 Hz comb lobes).
     Corner sweep: vent_a +/- vent_tol, foam_sigma x0.5..x2, membrane
     resistance mem_r_lo..mem_r_hi, T -20 / 25 / 85 C / max site temperature from
     analysis/thermal_report.json, p_amb 70..105 kPa, adiabatic / isothermal
     chamber air.  Gates: no resonant gain; prop blade-pass residual after the
     DPS310 oversampling comb vs a fraction of the EKF3 baro noise
     (EK3_ALT_M_NSE, converted with the corner's own air density); lag tau and
     altitude error; bond-leak spec and a feasible leak-down acceptance test.
  5. Water: membrane water-entry pressure vs dynamic pressure + dive + thermal
     contraction suction of the trapped air (hot board -> cold water) with
     margin; dry-cooling equilibration through the membrane; membrane
     deflection vs plenum depth; the labyrinth is NOT self-draining (Bond
     number, contact-angle-hysteresis pinning) - stated, not claimed.
  6. Light shielding.
  7. Thermal (cap resin, foam and membrane vs max site temperature), foam
     compression retention, stack height, board edge / grommet / courtyards.

Formula sources
  [SL]  Shah & London (1978): square duct laminar f*Re = 56.91 -> dp = 28.45 mu L Q / a^4.
  [ST]  Stinson, "The propagation of plane sound waves in narrow and wide
        circular tubes, and generalization to uniform tubes of arbitrary
        cross-sectional shape", JASA 89 (1991) 550; rectangular duct LRF
        solution (single sine series in y, closed form in z).
  [SA]  Sampson (1891): creeping flow through a thin-wall orifice dp = 3 mu Q / r^3.
  [KI]  Kinsler, Frey et al. 4th ed. s10.8-10.9: C = V/(gamma p) adiabatic, V/p
        isothermal; inertance rho L'/S, flanged end correction 0.85 r.
  [BE]  Beranek, "Acoustics" (1954) ch.5: lumped R-M-C, Poiseuille factor 4/3 on M.
  [AL]  Allard & Atalla 2nd ed. ch.2: static flow resistivity, R = sigma t / A.
  [SU]  Sutherland viscosity law; air conductivity k = 0.0241 (T/273.15)^0.81.
  [YL]  Young-Laplace entry pressure of a square channel 4 gamma |cos th_a| / a;
        hysteresis pinning 4 gamma |cos th_r - cos th_a| / a.
  [TI]  Timoshenko & Woinowsky-Krieger: clamped strip w = p y^2 (b-y)^2 / (24 D),
        w_max = p b^4 / (384 D), D = E t^3 / (12 (1 - nu^2)).
  [DS]  Infineon DPS310 datasheet v1.1 Table 16: t_meas = 2.0 ms + 1.6 ms x N
        (sub-conversion 1.6 ms); background mode requires
        PM_RATE t_P + TMP_RATE t_T < 1 s.
  [AP]  ArduPilot libraries/AP_Baro/AP_Baro_DPS280.cpp (set_config_registers,
        64 Hz poll), libraries/AP_NavEKF3/AP_NavEKF3.cpp (ALT_M_NSE_DEFAULT 2.0 m
        copter, HGT_DELAY 60 ms).
  [GO]  Gore automotive-electronics adhesive vents datasheet (2025-09), LM9xx
        high-airflow ePTFE/PET laminate: 65.5 l/h/cm^2 @ 70 mbar, WEP >= 600 mbar.

Usage: python3 analysis/baro_acoustic_analysis.py [--no-render] [--ardupilot DIR]
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
REF_H = os.path.join(REPO, "firmware", "baro_dps310", "dps310_robust.h")
HWDEF = os.path.join(REPO, "ardupilot", "hwdef.dat")
PARM = os.path.join(REPO, "ardupilot", "defaults.parm")
THERMAL = os.path.join(HERE, "thermal_report.json")

MM = 1e-3
G0 = 9.80665
R_AIR = 287.05          # J/(kg K), dry air
GAMMA = 1.4
CP_AIR = 1005.0         # J/(kg K)
P_STD = 101325.0

# ----------------------------------------------------------------------------
# Assumptions (all copied into the report).  Geometry, materials and design
# limits live in the .scad; these are environment / model / requirement inputs.
# ----------------------------------------------------------------------------
A = {
    "env": {
        "T_C_fixed": [-20.0, 25.0, 85.0],    # + max t_site from thermal_report.json (added at run time)
        "p_amb_Pa": [70000.0, 101325.0, 105000.0],   # 3000 m AMSL .. high-pressure sea level
        "p_sealevel_band_Pa": [95000.0, 105000.0],   # where the lag-error limit is applied
        "note": "C ~ 1/p, so low p maximises lag; high p minimises attenuation",
    },
    "prop": {
        "band_Hz": [100.0, 800.0],
        "band_note": "5-inch class blade-pass B*rpm/60: 2-blade 3000 rpm (idle) = 100 Hz .. "
                     "3-blade 16000 rpm = 800 Hz",
        "check_to_Hz": 2000.0,               # covers the 625 / 1250 / 1875 Hz comb lobes
        "spl_1m_dB": 95.0,                   # 5-inch quad at high throttle, 1 m
        "spreading_dB": 20.0,                # 1/r from 1 m to ~0.1 m from the disc edge
        "spl_note": "95 dB at 1 m + 20 dB spherical spreading to ~0.1 m = 115 dB, taken flat over "
                    "the band (conservative)",
    },
    "residual_frac_of_ekf_noise": 0.10,
    "residual_note": "tonal alias amplitude <= 10 % of EK3_ALT_M_NSE (1-sigma baro noise the EKF3 "
                     "assumes); a sinusoid of that amplitude adds (0.1/sqrt2)^2 = 0.5 % to the assumed "
                     "variance, so EKF weighting and innovation gating are unaffected.  Converted to Pa "
                     "with each corner's own air density (dp = rho g dh).",
    "climb_mps": [30.0, 10.0],
    "lag_err_max_m_at_30mps": 0.5,           # applied over p_sealevel_band
    "tau_max_s": 0.020,                      # vs 31.25 ms sample period and EK3_HGT_DELAY 60 ms
    "gain_max": 1.02,                        # no resonant amplification
    "leak": {"len_note": "pinhole through the bond bead across the wall foot (length = wall_t)",
             "spec_factor": 0.5,             # spec = half of the diameter that doubles |H| (-6 dB)
             "slot_w_mm": 1.0,
             "test_volume_mm3": 10000.0,     # external leak-down fixture volume (10 ml)
             "test_dp_Pa": 1000.0,           # fixture over-pressure (bond load 0.03 N, sensor-safe)
             "blank_factor": 10.0},          # fixture blank tau must exceed the spec by this factor
    "water": {
        "gamma_N_m": 0.072, "rho_w": 1000.0,
        "v_forward_mps": 30.0, "v_propwash_mps": 25.0,
        "dive_m": 150.0,                      # ambient rise during a 150 m dive with a wetted membrane
        "T_water_C": 0.0,                     # coldest liquid water (rain / immersion)
        "immersion_m": 1.0,                   # IP67-style 1 m immersion (secondary case)
        "wep_margin": 1.5, "wep_margin_immersion": 1.25,
        "cool_rate_K_s": 10.0,                # dry cooling rate bound (prop wash on a hot board)
        "deflect_frac_max": 0.5,              # membrane centre deflection <= half the plenum depth
    },
    "foam": {"cut_tol_mm": 0.1, "compression_set": 0.5, "sigma_comp_exponent": 2.0,
             "max_comp": 0.25,
             "note": "die-cut tolerance +-0.1 mm on each dimension; 50 % compression set "
                     "(hot, long term); sigma ~ sigma0/(1-eps)^2 per compressed axis [AL]"},
    "thermal": {"hdt_margin_C": 5.0, "mat_margin_C": 10.0, "scenarios_prefix": "R",
                "dps310_max_C": 85.0},
    "upstream_ardupilot": {
        "note": "documented ArduPilot master constants used when no tree is given (S1 SKIP)",
        "PCONF": 0x54, "TCONF": 0x54, "CFG_REG": 0x0C, "poll_Hz": 64, "temp_limit_C": 120.0,
        "ALT_M_NSE": 2.0, "HGT_DELAY_ms": 60,
    },
    "foam_sigma_factor": [0.5, 1.0, 2.0],    # foam flow resistivity sweep x0.5..x2
    "tl_modes": 21,                          # odd sine modes 1..41 in the Stinson series
}

GATES = []
WARNS = []
SKIPS = []


def gate(name, ok, detail):
    GATES.append({"gate": name, "pass": bool(ok), "detail": detail})
    print("  [%s] %s: %s" % ("PASS" if ok else "FAIL", name, detail))
    return ok


def warn(name, detail):
    WARNS.append({"warn": name, "detail": detail})
    print("  [WARN] %s: %s" % (name, detail))


def skip(name, detail):
    SKIPS.append({"skip": name, "detail": detail})
    print("  [SKIP] %s: %s" % (name, detail))


def info(name, detail):
    print("  [INFO] %s: %s" % (name, detail))


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
# 3. Flight driver (ArduPilot) and reference-driver sampling
# ----------------------------------------------------------------------------
def t_meas_s(prc):
    """[DS] Table 16: measurement time for 2^prc oversampling."""
    return (2.0 + 1.6 * (1 << prc)) * 1e-3


def _num(s):
    return int(s, 16) if s.lower().startswith("0x") else int(s)


def hwdef_baro():
    hw = open(HWDEF).read()
    m = re.search(r"^BARO\s+(\S+)\s+(.*)$", hw, re.M)
    return (m.group(1), m.group(2).strip()) if m else (None, None)


def parm_overrides():
    out = {}
    if os.path.exists(PARM):
        for ln in open(PARM).read().splitlines():
            m = re.match(r"\s*(BARO\w*|EK3_HGT_DELAY|EK3_ALT_M_NSE)\s*[ ,=]\s*([-0-9.eE]+)", ln)
            if m:
                out[m.group(1)] = float(m.group(2))
    return out


def ardupilot_sampling(root):
    """Parse the flight driver and EKF3 defaults from an ArduPilot tree.  Returns (dict, why_not)."""
    drv_name, drv_bus = hwdef_baro()
    up = A["upstream_ardupilot"]
    d = {"hwdef_driver": drv_name, "hwdef_bus": drv_bus, "source": None}
    if root:
        cpp = os.path.join(root, "libraries", "AP_Baro", "AP_Baro_DPS280.cpp")
        hdr = os.path.join(root, "libraries", "AP_Baro", "AP_Baro_DPS280.h")
        ekf = os.path.join(root, "libraries", "AP_NavEKF3", "AP_NavEKF3.cpp")
        if not all(os.path.exists(p) for p in (cpp, hdr, ekf)):
            return None, "ArduPilot tree %s lacks AP_Baro_DPS280.cpp/.h or AP_NavEKF3.cpp" % root
        c, h, e = open(cpp).read(), open(hdr).read(), open(ekf).read()
        body = re.search(r"AP_Baro_DPS280::set_config_registers\s*\(void\)\s*\{(.*?)\n\}", c, re.S)
        if not body:
            return None, "set_config_registers() not found"
        b = body.group(1)

        def reg(name):
            m = re.search(r"write_register\(\s*DPS280_REG_%s\s*,\s*(0x[0-9A-Fa-f]+|\d+)" % name, b)
            return _num(m.group(1)) if m else None
        d.update({"PCONF": reg("PCONF"), "TCONF": reg("TCONF"), "CFG_REG": reg("CREG"), "MCONF": reg("MCONF")})
        m = re.search(r"register_periodic_callback\(\s*\(\s*1000\s*/\s*(\d+)\s*\)", c)
        d["poll_Hz"] = int(m.group(1)) if m else None
        m = re.search(r"#define\s+TEMPERATURE_LIMIT_C\s+([0-9.]+)", c)
        d["temp_limit_C"] = float(m.group(1)) if m else None
        m = re.search(r"scaling_16\s*=\s*1\.0f\s*/\s*(\d+)", c)
        d["kP_scaling"] = int(m.group(1)) if m else None
        d["class_found"] = bool(drv_name and re.search(r"class\s+AP_Baro_%s\b" % re.escape(drv_name), h))
        nse = [float(x) for x in re.findall(r"#define\s+ALT_M_NSE_DEFAULT\s+([0-9.]+)f?", e)]
        d["ALT_M_NSE_defaults"] = nse
        d["ALT_M_NSE"] = min(nse) if nse else None
        m = re.search(r'AP_GROUPINFO\(\s*"HGT_DELAY"\s*,\s*\d+\s*,\s*NavEKF3\s*,\s*\w+\s*,\s*(\d+)\s*\)', e)
        d["HGT_DELAY_ms"] = int(m.group(1)) if m else None
        d["source"] = os.path.relpath(cpp, root) + " @ " + root
        if None in (d["PCONF"], d["TCONF"], d["poll_Hz"], d["ALT_M_NSE"], d["HGT_DELAY_ms"]):
            return None, "could not parse all fields: %s" % d
    else:
        d.update({k: up[k] for k in ("PCONF", "TCONF", "CFG_REG", "poll_Hz", "temp_limit_C", "ALT_M_NSE",
                                     "HGT_DELAY_ms")})
        d.update({"MCONF": 0x07, "kP_scaling": 253952, "class_found": None, "ALT_M_NSE_defaults": [2.0, 2.0, 3.0, 2.0],
                  "source": "documented upstream constants (no ArduPilot tree given)"})
    po = parm_overrides()
    d["defaults_parm_overrides"] = po
    if "EK3_ALT_M_NSE" in po:
        d["ALT_M_NSE"] = po["EK3_ALT_M_NSE"]
    if "EK3_HGT_DELAY" in po:
        d["HGT_DELAY_ms"] = int(po["EK3_HGT_DELAY"])
    pm_rate, pm_prc = (d["PCONF"] >> 4) & 7, d["PCONF"] & 0xF
    tmp_rate, tmp_prc = (d["TCONF"] >> 4) & 7, d["TCONF"] & 0xF
    n_os = 1 << pm_prc
    t_p, t_t = t_meas_s(pm_prc), t_meas_s(tmp_prc)
    budget = (1 << pm_rate) * t_p + (1 << tmp_rate) * t_t
    fs_cfg = float(1 << pm_rate)
    # if over-subscribed the sensor cannot run faster than back-to-back P/T conversions
    fs_eff = min(fs_cfg, 1.0 / (t_p + t_t * (1 << tmp_rate) / (1 << pm_rate))) if budget > 1.0 else fs_cfg
    d.update({"pm_rate_code": pm_rate, "pm_prc_code": pm_prc, "tmp_rate_code": tmp_rate, "tmp_prc_code": tmp_prc,
              "fs_Hz": fs_cfg, "fs_eff_Hz": fs_eff, "oversampling": n_os, "t_meas_s": t_p, "t_meas_T_s": t_t,
              "t_sub_s": 1.6e-3, "comb_lobe_Hz": 1.0 / 1.6e-3, "budget_ms_per_s": budget * 1e3,
              "kP_expected": {3: 7864320, 4: 253952, 5: 516096, 6: 1040384}.get(pm_prc),
              "shift_required": pm_prc > 3, "p_shift_set": bool((d.get("CFG_REG") or 0) & 0x04)})
    return d, None


def reference_sampling():
    """Secondary, informational: firmware/baro_dps310/dps310_robust.h (host-test reference, NOT flown)."""
    if not os.path.exists(REF_H):
        return None
    h = open(REF_H).read()

    def cfg(name, typ=float):
        m = re.search(r"\b%s\s*=\s*([-0-9.]+)f?\s*[;,]" % name, h)
        return typ(m.group(1)) if m else None
    r = {k: cfg(k, int) for k in ("pm_rate", "pm_prc", "tmp_rate", "tmp_prc")}
    r["gate_margin_pa"] = cfg("gate_margin_pa")
    if None in r.values():
        return None
    r["oversampling"] = 1 << r["pm_prc"]
    r["budget_ms_per_s"] = ((1 << r["pm_rate"]) * t_meas_s(r["pm_prc"]) + (1 << r["tmp_rate"]) * t_meas_s(r["tmp_prc"])) * 1e3
    return r


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


def k_air(T_C):
    return 0.0241 * ((T_C + 273.15) / 273.15) ** 0.81


def geometry_volumes(g, stl_air_mm3=None):
    """Air volumes (mm^3).  The air part = cavity minus U4, passives and ledges."""
    cav_xy = (g["cav_x1"] - g["cav_x0"]) * (g["cav_y1"] - g["cav_y0"])
    u4 = g["u4_w"] * g["u4_l"] * g["u4_h"]
    pas = g["n_pass"] * g["pass_w"] * g["pass_l"] * g["pass_h"]
    ledges = (cav_xy - g["sep_area"]) * g["ledge_h"]
    septum = g["sep_w"] * g["sep_l"] * g["mem_t"]
    air_analytic = cav_xy * g["cav_h"] - u4 - pas - ledges - septum
    air = stl_air_mm3 if stl_air_mm3 else air_analytic
    below = cav_xy * g["sep_z0"] - u4 - pas                    # node 2: sensor gap below the septum
    above = air - below                                         # node 1: ledge-ring opening + foam region
    free = ((g["cav_x1"] - g["cav_x0"] + 2 * g["foam_fit"]) * (g["cav_y1"] - g["cav_y0"] + 2 * g["foam_fit"])
            * (g["foam_t"] + g["foam_fit_z"]))
    solid = free * g["foam_density"] / g["foam_solid_density"]
    vent = g["vent_a"] ** 2 * g["vent_len"]
    plenum = g["mem_area"] * g["mem_plenum_d"]
    return {"air_analytic_mm3": air_analytic, "air_used_mm3": air, "gap_mm3": below,
            "foam_region_mm3": above, "foam_solid_mm3": solid, "foam_pore_air_mm3": above - solid,
            "vent_mm3": vent, "plenum_mm3": plenum, "porosity": 1 - g["foam_density"] / g["foam_solid_density"]}


def foam_compression(g):
    ex = 2 * g["foam_fit"] / (g["cav_x1"] - g["cav_x0"] + 2 * g["foam_fit"])
    ey = 2 * g["foam_fit"] / (g["cav_y1"] - g["cav_y0"] + 2 * g["foam_fit"])
    ez = g["foam_fit_z"] / (g["foam_t"] + g["foam_fit_z"])
    k = A["foam"]["sigma_comp_exponent"]
    fac = 1.0 / ((1 - ex) * (1 - ey) * (1 - ez)) ** (k / 3.0)
    return {"eps_x": ex, "eps_y": ey, "eps_z": ez, "sigma_factor": fac}


def membrane_mech(g, pw=None, ph=None):
    """Clamped strip (short side b of the active area, length Lm) [TI]."""
    pw, ph = (g["mem_pw"], g["mem_ph"]) if pw is None else (pw, ph)
    b = min(pw, ph) * MM
    Lm = max(pw, ph) * MM
    D = g["mem_e_eff"] * (g["mem_t"] * MM) ** 3 / (12 * (1 - g["mem_nu"] ** 2))
    C_d = b ** 5 * Lm / (720 * D)                     # volume per pressure, m^3/Pa
    M_d = (900.0 / 630.0) * g["mem_rho_s"] / (b * Lm)  # kg/m^4 for the y^2 (b-y)^2 mode shape
    return {"b_m": b, "L_m": Lm, "D_Nm": D, "C_d": C_d, "M_d": M_d,
            "w_per_Pa_m": b ** 4 / (384 * D)}


def elements(g, vols, a_mm, T_C, p, iso, sig_f, mem_f):
    """Lumped / distributed element values, SI."""
    mu = mu_air(T_C)
    rho = p / (R_AIR * (T_C + 273.15))
    a = a_mm * MM
    L = g["vent_len"] * MM
    S = a * a
    r_eq = a / math.sqrt(math.pi)                         # equal-area radius for the end terms
    kappa = 1.0 if iso else GAMMA
    comp = foam_compression(g)
    sigma = g["foam_sigma"] * sig_f * comp["sigma_factor"]
    t_path = (g["vent_z_in"] - g["foam_z0"]) * MM          # inner port height -> foam underside
    A_f = (g["cav_x1"] - g["cav_x0"]) * (g["cav_y1"] - g["cav_y0"]) * MM ** 2
    mm = membrane_mech(g)
    ms = membrane_mech(g, g["sep_pw"], g["sep_pl"])
    visc = mu / mu_air(g["mem_t_ref"])
    return {
        "mu": mu, "rho": rho, "T_C": T_C, "p": p, "a": a, "L": L, "S": S, "r_eq": r_eq,
        "alpha_th": k_air(T_C) / (rho * CP_AIR),
        "R_v": 28.45 * mu * L / a ** 4 + 2 * 1.5 * mu / r_eq ** 3,            # [SL] + [SA] both ports
        "M_v": (4.0 / 3.0) * rho * L / S + 2 * rho * 0.85 * r_eq / S,          # [BE] + [KI]
        "Ze_R": 1.5 * mu / r_eq ** 3, "Ze_M": rho * 0.85 * r_eq / S,           # per port end
        "C_vent": vols["vent_mm3"] * MM ** 3 / (kappa * p),
        "R_m": g["mem_r"] * mem_f * visc / (g["mem_area"] * MM ** 2),
        "R_s": g["mem_r"] * mem_f * visc / (g["sep_area"] * MM ** 2),
        "C_ds": ms["C_d"], "M_ds": ms["M_d"],
        "C_d": mm["C_d"], "M_d": mm["M_d"],
        "C_p": vols["plenum_mm3"] * MM ** 3 / (kappa * p),
        "C1": vols["foam_pore_air_mm3"] * MM ** 3 / (kappa * p),               # [KI]
        "C2": vols["gap_mm3"] * MM ** 3 / (kappa * p),
        "R_f": sigma * t_path / A_f,                                           # [AL]
    }


ODD = None


def stinson_tl(w, e):
    """Series impedance Z' and shunt admittance Y' per metre of a square duct [ST]."""
    global ODD
    if ODD is None or len(ODD) != A["tl_modes"]:
        ODD = 2 * np.arange(A["tl_modes"]) + 1.0
    m = ODD[None, :]
    a, mu, rho, al = e["a"], e["mu"], e["rho"], e["alpha_th"]
    jw = 1j * np.asarray(w, dtype=float)[:, None]
    beta2 = (m * np.pi / a) ** 2
    cm = 8.0 / (m * m * np.pi ** 2)

    def shape(s2):
        s = np.sqrt(s2)
        x = s * a / 2
        return 1.0 - np.tanh(x) / x
    s2v = beta2 + jw * rho / mu
    Gv = (cm / (mu * s2v) * shape(s2v)).sum(axis=1)                 # <v> / (-dp/dx)
    s2t = beta2 + jw / al
    F = (cm * jw / (al * s2t) * shape(s2t)).sum(axis=1)             # 0 isothermal .. 1 adiabatic
    Zs = 1.0 / (e["S"] * Gv)
    Ys = jw[:, 0] * e["S"] / (GAMMA * e["p"]) * (GAMMA - (GAMMA - 1.0) * F)
    return Zs, Ys


def vent_y(w, e, model):
    """Two-port admittance (Y11, Y12, Y22) of the vent incl. port end impedances, plus lumped
    shunt compliances (n0, n1) for the lumped model."""
    w = np.asarray(w, dtype=float)
    if model == "tl":
        Zs, Ys = stinson_tl(w, e)
        gam = np.sqrt(Zs * Ys)
        Zc = np.sqrt(Zs / Ys)
        gl = gam * e["L"]
        Aa = np.cosh(gl)
        B = Zc * np.sinh(gl)
        C = np.sinh(gl) / Zc
        Ze = e["Ze_R"] + 1j * w * e["Ze_M"]
        # [1 Ze; 0 1] [A B; C A] [1 Ze; 0 1]
        A2 = Aa + Ze * C
        B2 = Aa * Ze + B + Ze * (C * Ze + Aa)
        D2 = C * Ze + Aa
        return D2 / B2, -1.0 / B2, A2 / B2
    Z = e["R_v"] + 1j * w * e["M_v"]
    return 1.0 / Z, -1.0 / Z, 1.0 / Z


def H_of(f, e, model="tl", leak=None, vent_share=0.0):
    """p_sensor / p_out by nodal analysis on n0 (plenum), n1 (foam), n2 (sensor gap)."""
    w = 2 * np.pi * np.atleast_1d(np.asarray(f, dtype=float))
    jw = 1j * w
    Ym = 1.0 / e["R_m"] + 1.0 / (1.0 / (jw * e["C_d"]) + jw * e["M_d"])
    Y11, Y12, Y22 = vent_y(w, e, model)
    cv = vent_share * e["C_vent"] / 2 if model == "lump" else 0.0
    Yp = jw * (e["C_p"] + cv)
    Y1 = jw * (e["C1"] + cv)
    Y2 = jw * e["C2"]
    Zs = 1.0 / (1.0 / e["R_s"] + 1.0 / (1.0 / (jw * e["C_ds"]) + jw * e["M_ds"]))
    Yf = 1.0 / (e["R_f"] + Zs)
    Yl = np.zeros_like(jw) if leak is None else 1.0 / (leak["R"] + jw * leak["M"])
    n = len(w)
    M = np.zeros((n, 3, 3), dtype=complex)
    M[:, 0, 0] = Ym + Yp + Y11
    M[:, 0, 1] = Y12
    M[:, 1, 0] = Y12
    M[:, 1, 1] = Y22 + Y1 + Yf
    M[:, 1, 2] = -Yf
    M[:, 2, 1] = -Yf
    M[:, 2, 2] = Yf + Y2 + Yl
    rhs = np.zeros((n, 3, 1), dtype=complex)
    rhs[:, 0, 0] = Ym
    rhs[:, 2, 0] = Yl
    return np.linalg.solve(M, rhs)[:, 2, 0]


def tau_of(e, model="tl", leak=None, vent_share=0.5):
    """Ramp lag = -d(phase)/dw at w -> 0 (first moment of the impulse response)."""
    f0 = 1e-2
    h = H_of(np.array([f0]), e, model, leak, vent_share)
    return float(-np.angle(h)[0] / (2 * np.pi * f0))


def r_dc_series(e):
    """DC flow resistance outside -> sensor (membrane + vent + foam + septum)."""
    return e["R_m"] + e["R_v"] + e["R_f"] + e["R_s"]


def leak_element(d_m, L_m, e):
    r = d_m / 2
    R = 128 * e["mu"] * L_m / (math.pi * d_m ** 4) + 3 * e["mu"] / r ** 3    # Poiseuille + [SA]
    M = e["rho"] * (L_m + 2 * 0.85 * r) / (math.pi * r * r)
    return {"R": R, "M": M}


# ----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-render", action="store_true", help="skip OpenSCAD STL rendering")
    ap.add_argument("--ardupilot", default=os.environ.get("ARDUPILOT_ROOT"),
                    help="ArduPilot source tree (default $ARDUPILOT_ROOT); flight driver is parsed from it")
    args = ap.parse_args()
    rep = {"revision": "12.1", "assumptions": A}

    # ------------------------------------------------------------------ 1
    print("== 1. SCAD parse / evaluate ==")
    g, order, asserts, scad_text = parse_scad(SCAD)
    nnum = sum(1 for _, v in g.items() if isinstance(v, (int, float)) and not isinstance(v, bool))
    gate("P1 SCAD top-level assignments parsed and evaluated", len(order) >= 100 and "vent_len" in g and "mem_r" in g,
         "%d assignments (%d numeric), part=%r" % (len(order), nnum, g.get("part")))
    bad = []
    for cond, msg in asserts:
        try:
            ok = bool(Expr(tokenize(" ".join(cond.split())), g).parse())
        except Exception as ex:     # noqa: BLE001
            ok, msg = False, "%s (eval error %s)" % (msg, ex)
        if not ok:
            bad.append(msg)
    gate("P2 SCAD asserts re-evaluated in Python", not bad and len(asserts) >= 30,
         "%d asserts, failing: %s" % (len(asserts), bad or "none"))
    echo_txt, why = openscad_echo(SCAD)
    echo = parse_echo(echo_txt) if echo_txt else None
    if echo:
        pairs = {"cav": [g["cav_x1"] - g["cav_x0"], g["cav_y1"] - g["cav_y0"], g["cav_h"]],
                 "foam_t": [g["foam_t"]], "vent_len": [g["vent_len"]], "vent_z_out": [g["vent_z_out"]],
                 "mem_area": [g["mem_area"]], "mem_r": [g["mem_r"]], "sep_area": [g["sep_area"]]}
        worst = max(abs(a - b) / max(abs(b), 1e-9) for k, v in pairs.items() for a, b in zip(echo[k], v))
        err_lines = [ln for ln in echo_txt.splitlines() if "ERROR" in ln or "Assertion" in ln]
        gate("P3 Python evaluation == OpenSCAD ECHO (cav, foam_t, vent_len, vent_z_out, mem_area, mem_r)",
             worst < 2e-5 and not err_lines,
             "max rel diff %.1e (OpenSCAD prints 6 significant digits); vent_len %.4f mm; openscad errors: %s"
             % (worst, g["vent_len"], err_lines or "none"))
    else:
        skip("P3 ECHO cross-check", why or "no ECHO line")
    rep["scad"] = {k: v for k, v in g.items() if isinstance(v, (int, float, str))}
    rep["echo"] = echo

    # ------------------------------------------------------------------ 2
    print("== 2. render + mesh checks ==")
    stl_air = None
    mesh = {}
    parts = ("cap", "foam_block", "membrane", "septum", "air", "vent_air")
    if args.no_render or not openscad_bin():
        skip("rendering", "--no-render" if args.no_render else "openscad not found")
    else:
        os.makedirs(STL_DIR, exist_ok=True)
        for part in parts:
            out = os.path.join(STL_DIR, "baro_%s.stl" % part)
            r = render(part, out)
            st = mesh_stats(read_stl(out))
            r.update({k: v for k, v in st.items() if not k.startswith("_")})
            r["file"] = os.path.relpath(out, REPO)
            mesh[part] = (r, st)
            print("    %-10s CGAL simple=%s volumes=%s  STL tris %5d shells %d non-2-edges %d  V %.3f mm^3"
                  % (part, r["simple"], r["volumes"], st["triangles"], st["shells"], st["non2_edges"],
                     st["volume_mm3"]))
        for part in parts:
            r, st = mesh[part]
            # a lone primitive (foam_block, membrane) bypasses CGAL, so OpenSCAD prints no Simple/Volumes
            # lines; the STL edge/shell check then carries the manifold proof.
            cgal_ok = (r["simple"] == "yes" and r["volumes"] == 2) or (r["simple"] is None
                                                                       and part in ("foam_block", "membrane", "septum"))
            gate("R1 %s renders manifold, one solid" % part,
                 r["rc"] == 0 and cgal_ok and st["non2_edges"] == 0 and st["shells"] == 1 and not r["warnings"],
                 "rc %d, Simple: %s, CGAL Volumes: %s (outer + 1 solid), %d non-2 edges, %d STL shells%s"
                 % (r["rc"], r["simple"], r["volumes"], st["non2_edges"], st["shells"],
                    ", warnings %s" % r["warnings"] if r["warnings"] else ""))
        r, st = mesh["vent_air"]
        x_in = g["cav_x1"]
        x_out = g["mem_x_floor"]
        pts = st["_pts"]
        gate("R2 vent_air is ONE connected volume", st["shells"] == 1 and r["volumes"] == 2,
             "%d vertex-connected STL shell(s), CGAL volumes %d" % (st["shells"], r["volumes"]))
        p_in = pts[pts[:, 0] <= x_in + 1e-6]
        p_out = pts[pts[:, 0] >= x_out - 1e-6]
        zin = (p_in[:, 2].min(), p_in[:, 2].max()) if len(p_in) else (0, 0)
        zout = (p_out[:, 2].min(), p_out[:, 2].max()) if len(p_out) else (0, 0)
        yout = (p_out[:, 1].min(), p_out[:, 1].max()) if len(p_out) else (9, 9)
        in_plenum = (len(p_out) and abs(yout[0]) <= g["mem_pw"] / 2 and abs(yout[1]) <= g["mem_pw"] / 2
                     and zout[0] >= g["mem_zc"] - g["mem_ph"] / 2 and zout[1] <= g["mem_zc"] + g["mem_ph"] / 2)
        gate("R3 vent_air pierces the cavity face and opens inside the membrane plenum",
             st["bbox_min"][0] < x_in and st["bbox_max"][0] > x_out and len(p_in) and in_plenum
             and abs(0.5 * sum(zin) - g["vent_z_in"]) < 1e-3 and abs(0.5 * sum(zout) - g["vent_z_out"]) < 1e-3,
             "x %.3f..%.3f vs cavity face %.3f / plenum floor %.3f; inner opening z %.3f..%.3f (vent_z_in %.2f), "
             "outer opening z %.3f..%.3f y %.3f..%.3f inside plenum %.1f x %.1f mm"
             % (st["bbox_min"][0], st["bbox_max"][0], x_in, x_out, zin[0], zin[1], g["vent_z_in"],
                zout[0], zout[1], yout[0], yout[1], g["mem_pw"], g["mem_ph"]))
        vv = st["volume_mm3"]
        v_line = g["vent_a"] ** 2 * g["vent_len"]
        gate("R4 vent_air volume ~ a^2 * vent_len (centreline length is right)",
             abs(vv / v_line - 1) < 0.08,
             "STL %.3f mm^3 vs a^2 L %.3f mm^3 (%+.1f %%; end caps / stub overlap)"
             % (vv, v_line, 100 * (vv / v_line - 1)))
        stl_air = mesh["air"][1]["volume_mm3"]
        va = geometry_volumes(g)["air_analytic_mm3"]
        gate("R5 air STL volume == analytic cavity air", abs(stl_air / va - 1) < 0.01,
             "STL %.3f vs %.3f mm^3" % (stl_air, va))
        rc, stc = mesh["cap"]
        cap_bb_ok = (abs(stc["bbox_max"][2] - g["cap_h"]) < 1e-3 and abs(stc["bbox_min"][0] + g["cap_w"] / 2) < 1e-3
                     and abs(stc["bbox_max"][1] - g["cap_l"] / 2) < 1e-3 and abs(stc["bbox_max"][0] - g["cap_w"] / 2) < 1e-3)
        gate("R6 cap STL bbox == cap_w x cap_l x cap_h (membrane seat recessed, nothing proud)", cap_bb_ok,
             "bbox %s .. %s" % (np.round(stc["bbox_min"], 3).tolist(), np.round(stc["bbox_max"], 3).tolist()))
        fb = mesh["foam_block"][1]
        exp_fb = ((g["cav_x1"] - g["cav_x0"] + 2 * g["foam_fit"]) * (g["cav_y1"] - g["cav_y0"] + 2 * g["foam_fit"])
                  * (g["foam_t"] + g["foam_fit_z"]))
        mb = mesh["membrane"][1]
        exp_mb = g["mem_w"] * g["mem_h"] * g["mem_t"]
        sb = mesh["septum"][1]
        exp_sb = g["sep_w"] * g["sep_l"] * g["mem_t"]
        gate("R7 foam_block / membrane / septum cut templates", abs(fb["volume_mm3"] / exp_fb - 1) < 1e-3
             and abs(mb["volume_mm3"] / exp_mb - 1) < 1e-3 and mb["bbox_max"][0] <= g["cap_w"] / 2
             and abs(sb["volume_mm3"] / exp_sb - 1) < 1e-3 and sb["bbox_min"][2] - g["u4_h"] >= g["air_gap_min"] - 1e-9,
             "foam %.3f vs %.3f mm^3; membrane %.3f vs %.3f mm^3, outer surface x %.3f <= face %.3f; septum %.3f vs "
             "%.3f mm^3, %.2f mm above the U4 lid"
             % (fb["volume_mm3"], exp_fb, mb["volume_mm3"], exp_mb, mb["bbox_max"][0], g["cap_w"] / 2,
                sb["volume_mm3"], exp_sb, sb["bbox_min"][2] - g["u4_h"]))
        rep["render"] = {p: mesh[p][0] for p in mesh}

    # thermal inputs are needed for the temperature corners
    tr = json.load(open(THERMAL))
    sc = {k: v for k, v in tr["scenarios"].items() if k.startswith(A["thermal"]["scenarios_prefix"])}
    t_site = max(v["t_site_C"] for v in sc.values())
    t_board = max(v["t_board_C"] for v in sc.values())
    tj = max(v["tj_hs_C"] for v in sc.values())
    k_site = max(sc, key=lambda k: sc[k]["t_site_C"])
    T_hot = math.ceil(t_site)
    T_corners = sorted(set(A["env"]["T_C_fixed"] + [float(T_hot)]))

    # ------------------------------------------------------------------ 3
    print("== 3. DPS310 sampling: FLIGHT driver (ArduPilot) ==")
    ds, why = ardupilot_sampling(args.ardupilot)
    if ds is None:
        print("  [WARN] ArduPilot tree unusable (%s); falling back to documented upstream constants" % why)
        ds, _ = ardupilot_sampling(None)
        why = "tree unusable: " + why
    print("    hwdef 'BARO %s %s' -> AP_Baro_%s (%s)" % (ds["hwdef_driver"], ds["hwdef_bus"], ds["hwdef_driver"],
                                                     ds["source"]))
    print("    PCONF 0x%02X -> PM_RATE %d (%.0f Hz) PM_PRC %d (%dx, %.1f ms); TCONF 0x%02X -> TMP_RATE %d TMP_PRC %d "
          "(%.1f ms); CFG_REG 0x%02X (P_SHIFT %s); poll %d Hz; budget %.0f ms/s; EK3_ALT_M_NSE %.1f m, "
          "EK3_HGT_DELAY %d ms; driver temp limit %s C; defaults.parm overrides %s"
          % (ds["PCONF"], ds["pm_rate_code"], ds["fs_Hz"], ds["pm_prc_code"], ds["oversampling"], ds["t_meas_s"] * 1e3,
             ds["TCONF"], ds["tmp_rate_code"], ds["tmp_prc_code"], ds["t_meas_T_s"] * 1e3, ds["CFG_REG"] or 0,
             ds["p_shift_set"], ds["poll_Hz"], ds["budget_ms_per_s"], ds["ALT_M_NSE"], ds["HGT_DELAY_ms"],
             ds["temp_limit_C"], ds["defaults_parm_overrides"] or "none"))
    s1_ok = (ds["hwdef_driver"] == "DPS310" and ds["class_found"] and ds["MCONF"] == 0x07
             and ds["shift_required"] == ds["p_shift_set"] and ds["kP_scaling"] == ds["kP_expected"]
             and ds["poll_Hz"] >= 2 * ds["fs_Hz"])
    s1_detail = ("AP_Baro_%s class present, continuous P+T (MCONF 0x%02X), %.0f Hz x %dx (sub-conversion %.1f ms -> "
                 "comb lobes k x %.0f Hz), P_SHIFT consistent, kP %s matches %dx, poll %d Hz >= 2 x %.0f Hz"
                 % (ds["hwdef_driver"], ds["MCONF"], ds["fs_Hz"], ds["oversampling"], ds["t_sub_s"] * 1e3,
                    ds["comb_lobe_Hz"], ds["kP_scaling"], ds["oversampling"], ds["poll_Hz"], ds["fs_Hz"]))
    if args.ardupilot and not why:
        gate("S1 flight sampling plan parsed from the ArduPilot tree and self-consistent", s1_ok, s1_detail)
    else:
        skip("S1 flight sampling plan from an ArduPilot tree",
             "%s; using documented upstream AP_Baro_DPS280.cpp constants (PCONF/TCONF 0x54, 64 Hz poll, "
             "ALT_M_NSE 2.0 m) - pass --ardupilot DIR or set ARDUPILOT_ROOT to verify"
             % (why or "no tree given"))
    if ds["budget_ms_per_s"] > 1000:
        warn("DPS310 background budget (flight driver)",
             "stock ArduPilot requests P %.0f Hz x %.1f ms + T %.0f Hz x %.1f ms = %.0f ms/s > 1000 ms/s [DS]; the "
             "sensor serialises conversions -> effective pressure rate ~%.1f Hz.  Not a chamber issue; the "
             "analysis uses the configured %.0f Hz for the fc > 2 fs check (conservative) and the 16x comb "
             "(unchanged).  Upstream driver change would be needed to fix."
             % (ds["fs_Hz"], ds["t_meas_s"] * 1e3, float(1 << ds["tmp_rate_code"]), ds["t_meas_T_s"] * 1e3,
                ds["budget_ms_per_s"], ds["fs_eff_Hz"], ds["fs_Hz"]))
    if ds["temp_limit_C"] is not None and t_board > ds["temp_limit_C"] - 10:
        warn("driver temperature limit", "board %.1f C vs AP_Baro_DPS280 TEMPERATURE_LIMIT_C %.0f C (driver resets "
             "the sensor above it)" % (t_board, ds["temp_limit_C"]))
    ref = reference_sampling()
    if ref:
        info("reference driver (host-test, NOT in the flight build)",
             "dps310_robust.h plan PM_RATE %d PM_PRC %d (%dx) TMP_RATE %d TMP_PRC %d, budget %.0f ms/s, "
             "gate_margin_pa %.1f Pa; same pressure oversampling as flight: %s"
             % (ref["pm_rate"], ref["pm_prc"], ref["oversampling"], ref["tmp_rate"], ref["tmp_prc"],
                ref["budget_ms_per_s"], ref["gate_margin_pa"], ref["oversampling"] == ds["oversampling"]))
    rep["sampling"] = {"flight": ds, "flight_source_ok": bool(args.ardupilot and not why), "reference_info": ref}

    # ------------------------------------------------------------------ 4
    print("== 4. acoustic / pneumatic model ==")
    vols = geometry_volumes(g, stl_air)
    comp = foam_compression(g)
    mm_ = membrane_mech(g)
    print("    air %.2f mm^3 (gap %.2f, foam region %.2f, foam solid %.2f, porosity %.3f), vent %.2f mm^3, "
          "plenum %.2f mm^3" % (vols["air_used_mm3"], vols["gap_mm3"], vols["foam_region_mm3"],
                                vols["foam_solid_mm3"], vols["porosity"], vols["vent_mm3"], vols["plenum_mm3"]))
    print("    membrane: %.1f l/h/cm^2 @ %.0f mbar -> r = %.0f Pa s/m (specific airflow resistance, %.0f C), "
          "active %.1f x %.1f = %.1f mm^2; diaphragm C_d %.2e m^3/Pa, M_d %.2e kg/m^4"
          % (g["mem_airflow"], g["mem_dp_ref"] / 100, g["mem_r"], g["mem_t_ref"], g["mem_pw"], g["mem_ph"],
             g["mem_area"], mm_["C_d"], mm_["M_d"]))
    a0, tol = g["vent_a"], g["vent_tol"]
    f = np.unique(np.concatenate([np.logspace(-1, math.log10(A["prop"]["check_to_Hz"]), 1200),
                                  np.arange(1, A["prop"]["check_to_Hz"] + 1, 1.0)]))
    nom = elements(g, vols, a0, 25.0, P_STD, False, 1.0, 1.0)
    nom_iso = elements(g, vols, a0, 25.0, P_STD, True, 1.0, 1.0)
    # TL implementation check: DC series resistance == Shah & London, and mode convergence
    Zs0, _ = stinson_tl(np.array([1e-3]), nom)
    r_sl = 28.45 * nom["mu"] / (nom["a"] ** 4)
    nm = A["tl_modes"]
    A["tl_modes"] = 2 * nm
    Zs_hi, Ys_hi = stinson_tl(np.array([1875 * 2 * np.pi]), nom)
    A["tl_modes"] = nm
    Zs_lo, Ys_lo = stinson_tl(np.array([1875 * 2 * np.pi]), nom)
    conv = max(abs(Zs_hi[0] / Zs_lo[0] - 1), abs(Ys_hi[0] / Ys_lo[0] - 1))
    gate("A0 TL model check: Stinson DC resistance == Shah-London 28.45 mu/a^4, %d-mode series converged" % nm,
         abs(Zs0[0].real / r_sl - 1) < 2e-3 and conv < 1e-3,
         "Re Z'(0)/R'_SL = %.5f; doubling the modes changes Z', Y' at 1875 Hz by %.1e"
         % (Zs0[0].real / r_sl, conv))

    Hn = np.abs(H_of(f, nom))
    fc = float(f[np.argmax(Hn <= 1 / math.sqrt(2))])
    tau_nom_ad, tau_nom_iso = tau_of(nom), tau_of(nom_iso)
    f0_lump = 1 / (2 * math.pi * math.sqrt(nom["M_v"] * (nom["C1"] + nom["C2"])))
    fx_list = (16, 32, 100, 200, 400, 625, 800, 1250, 1875)
    att = {x: float(-20 * np.log10(abs(H_of([x], nom)[0]))) for x in fx_list}
    att_l = {x: float(-20 * np.log10(abs(H_of([x], nom, "lump")[0]))) for x in fx_list}
    rdc = r_dc_series(nom)
    print("    nominal 25 C 101.3 kPa: R_mem %.3e (%.0f %% of DC path), R_vent %.3e (%.0f %%), R_foam %.3e, "
          "R_septum %.3e (%.0f %%) Pa s/m^3; C_p %.2e C1 %.2e C2 %.2e m^3/Pa (adiabatic); septum C_d %.2e"
          % (nom["R_m"], 100 * nom["R_m"] / rdc, nom["R_v"], 100 * nom["R_v"] / rdc, nom["R_f"], nom["R_s"],
             100 * nom["R_s"] / rdc, nom["C_p"], nom["C1"], nom["C2"], nom["C_ds"]))
    print("    TL: tau %.2f ms (adiabatic) / %.2f ms (isothermal), fc %.1f Hz; lumped tau %.2f / %.2f ms; "
          "lumped vent Helmholtz %.0f Hz" % (tau_nom_ad * 1e3, tau_nom_iso * 1e3, fc,
                                             tau_of(nom, "lump") * 1e3, tau_of(nom_iso, "lump") * 1e3, f0_lump))
    print("    attenuation nominal  TL  : " + ", ".join("%d Hz %.1f dB" % kv for kv in att.items()))
    print("    attenuation nominal  LUMP: " + ", ".join("%d Hz %.1f dB" % kv for kv in att_l.items()))
    pore = g["foam_pore"]
    alpha = nom["alpha_th"]
    dth = {fx: 1e3 * math.sqrt(2 * alpha / (2 * math.pi * fx)) for fx in (10, 100, 625)}
    print("    thermal boundary layer %s mm vs cell %.2f mm / gap %.2f mm -> chamber air bracketed "
          "adiabatic (attenuation) .. isothermal (lag)" % ({k: round(v, 3) for k, v in dth.items()}, pore,
                                                           g["air_gap"]))

    # corner sweep (TL for gates, lumped alongside for comparison)
    alt_nse = ds["ALT_M_NSE"]
    frac = A["residual_frac_of_ekf_noise"]
    band = (f >= A["prop"]["band_Hz"][0]) & (f <= A["prop"]["check_to_Hz"])
    spl = A["prop"]["spl_1m_dB"] + A["prop"]["spreading_dB"]
    P_in = 20e-6 * 10 ** (spl / 20) * math.sqrt(2)
    G_os = os_envelope(f, ds["oversampling"], ds["t_sub_s"])
    fb_, Gb = f[band], G_os[band]
    corners = []
    worst = {"gain": (0, None), "ratio": (0, None, None, None, None), "tau": (0, None), "lag_sl": (0, None),
             "att": {}, "lump_gain": 0.0, "lump_ratio": 0.0, "lump_tau": 0.0, "dlump_dB": {}}
    fcmp = np.array([100.0, 625.0, 1250.0, 1875.0])
    for a_mm in (a0 - tol, a0, a0 + tol):
        for sf in A["foam_sigma_factor"]:
            for mf in (g["mem_r_lo"], 1.0, g["mem_r_hi"]):
                for T in T_corners:
                    for p in A["env"]["p_amb_Pa"]:
                        for iso in (False, True):
                            e = elements(g, vols, a_mm, T, p, iso, sf, mf)
                            key = (a_mm, sf, mf, T, p, iso)
                            corners.append((key, e))
                            Hc = np.abs(H_of(f, e))
                            lim = frac * alt_nse * e["rho"] * G0
                            res = P_in * Hc[band] * Gb
                            k = int(np.argmax(res))
                            if Hc.max() > worst["gain"][0]:
                                worst["gain"] = (float(Hc.max()), key)
                            if res[k] / lim > worst["ratio"][0]:
                                worst["ratio"] = (float(res[k] / lim), key, float(res[k]), float(lim), float(fb_[k]))
                            for fx in (100, 200, 400, 625, 800, 1250, 1875):
                                h = float(Hc[np.argmin(abs(f - fx))])
                                if h > worst["att"].get(fx, (0, None))[0]:
                                    worst["att"][fx] = (h, key)
                            t = tau_of(e) if iso else None
                            if iso and t > worst["tau"][0]:
                                worst["tau"] = (t, key)
                            if iso and A["env"]["p_sealevel_band_Pa"][0] <= p <= A["env"]["p_sealevel_band_Pa"][1] \
                                    and t > worst["lag_sl"][0]:
                                worst["lag_sl"] = (t, key)
                            # lumped comparison (same corner)
                            Hl = np.abs(H_of(f, e, "lump"))
                            worst["lump_gain"] = max(worst["lump_gain"], float(Hl.max()))
                            worst["lump_ratio"] = max(worst["lump_ratio"], float((P_in * Hl[band] * Gb).max() / lim))
                            if iso:
                                worst["lump_tau"] = max(worst["lump_tau"], tau_of(e, "lump"))
                            ht = np.abs(H_of(fcmp, e))
                            hl = np.abs(H_of(fcmp, e, "lump"))
                            for fx, d in zip(fcmp, 20 * np.log10(hl / ht)):
                                if abs(d) > abs(worst["dlump_dB"].get(int(fx), 0.0)):
                                    worst["dlump_dB"][int(fx)] = float(d)

    def ck(key):
        return "a=%.2f mm foam x%.1f mem x%.1f T=%.0f C p=%.0f kPa %s" % (
            key[0], key[1], key[2], key[3], key[4] / 1e3, "isothermal" if key[5] else "adiabatic")
    gate("A1 no resonant amplification |H| <= %.2f, all %d corners (TL)" % (A["gain_max"], len(corners)),
         worst["gain"][0] <= A["gain_max"],
         "max |H| %.4f (%s); lumped model max |H| %.4f" % (worst["gain"][0], ck(worst["gain"][1]), worst["lump_gain"]))
    wr = worst["ratio"]
    res_lim_desc = ("%.0f %% of EK3_ALT_M_NSE %.1f m = %.2f m x rho g at each corner" % (100 * frac, alt_nse,
                                                                                          frac * alt_nse))
    gate("A2 prop band %g..%g Hz alias residual after chamber x DPS310 %dx comb <= %s, all corners (TL)"
         % (A["prop"]["band_Hz"][0], A["prop"]["check_to_Hz"], ds["oversampling"], res_lim_desc),
         wr[0] <= 1.0,
         "worst %.2f Pa vs limit %.2f Pa (%.0f %% of limit) at %.0f Hz (%s); input %.0f dB SPL = %.1f Pa peak; "
         "lumped model worst %.0f %% of limit" % (wr[2], wr[3], 100 * wr[0], wr[4], ck(wr[1]), spl, P_in,
                                                  100 * worst["lump_ratio"]))
    att_worst = {fx: -20 * math.log10(v[0]) for fx, v in worst["att"].items()}
    print("    worst-corner attenuation (TL): " + ", ".join("%d Hz %.1f dB" % (k, v) for k, v in att_worst.items()))
    tw, kw = worst["tau"]
    gate("A3 quasi-static lag tau <= %.0f ms, all corners (incl. 3000 m, %d C, iso, mem x%.1f) (TL)"
         % (A["tau_max_s"] * 1e3, T_hot, g["mem_r_hi"]),
         tw <= A["tau_max_s"], "worst tau %.2f ms (%s) vs period %.2f ms, EK3_HGT_DELAY %d ms; lumped worst %.2f ms"
         % (tw * 1e3, ck(kw), 1e3 / ds["fs_Hz"], ds["HGT_DELAY_ms"], worst["lump_tau"] * 1e3))
    ts, ks = worst["lag_sl"]
    lag30, lag10 = A["climb_mps"][0] * ts, A["climb_mps"][1] * ts
    gate("A4 altitude lag error at 30 m/s <= %.1f m (sea-level band)" % A["lag_err_max_m_at_30mps"],
         lag30 <= A["lag_err_max_m_at_30mps"],
         "%.3f m at 30 m/s, %.3f m at 10 m/s (tau %.2f ms, %s); at 3000 m worst %.3f m at 30 m/s; nominal %.3f m"
         % (lag30, lag10, ts * 1e3, ck(ks), A["climb_mps"][0] * tw, A["climb_mps"][0] * tau_nom_iso))
    gate("A5 chamber sits between EKF-scale lag and blade-pass band (2 x fs < fc < band start)",
         2 * ds["fs_Hz"] < fc < A["prop"]["band_Hz"][0],
         "fc %.1f Hz (nominal TL), fs %.0f Hz configured (%.1f Hz effective), prop band from %.0f Hz"
         % (fc, ds["fs_Hz"], ds["fs_eff_Hz"], A["prop"]["band_Hz"][0]))
    gate("A7 TL vs lumped vent model compared (both reported; TL used for the gates)",
         abs(tau_of(nom_iso, "lump") / tau_nom_iso - 1) < 0.05,
         "DC lag agrees within %.1f %%; worst lumped-minus-TL |H| over corners: %s (lumped R-M-C is "
         "%s at the upper comb lobes); vent length %.1f mm = %.3f wavelength at 1875 Hz"
         % (100 * abs(tau_of(nom_iso, "lump") / tau_nom_iso - 1),
            ", ".join("%d Hz %+.1f dB" % kv for kv in sorted(worst["dlump_dB"].items())),
            "optimistic" if worst["dlump_dB"].get(1875, 0) < 0 else "conservative",
            g["vent_len"], g["vent_len"] * MM * 1875 / 346.0))

    # bond leak (TL model)
    Lk = g["wall_t"] * MM
    fl = ds["comb_lobe_Hz"]
    h0 = abs(H_of([fl], nom)[0])

    def h_leak(d, e):
        return abs(H_of([fl], e, leak=leak_element(d, Lk, e))[0])
    lo, hi = 1e-6, 1e-3
    for _ in range(80):
        mid = math.sqrt(lo * hi)
        lo, hi = (mid, hi) if h_leak(mid, nom) < 2 * h0 else (lo, mid)
    d_x2 = hi
    kr = wr[1]
    e_r = [c[1] for c in corners if c[0] == kr][0]
    lim_r = wr[3]

    def res_leak(d):   # worst residual over the band at the worst-ratio corner
        return float((P_in * np.abs(H_of(fb_, e_r, leak=leak_element(d, Lk, e_r))) * Gb).max())
    lo2, hi2 = 1e-6, 1e-3
    for _ in range(80):
        mid = math.sqrt(lo2 * hi2)
        lo2, hi2 = (mid, hi2) if res_leak(mid) <= lim_r else (lo2, mid)
    no_chamber = float((P_in * Gb).max())
    gate("A2b the chamber is load-bearing (same residual without it fails)", no_chamber > lim_r,
         "bare sensor residual %.1f Pa at the %.0f Hz comb lobe vs limit %.2f Pa" % (no_chamber, fl, lim_r))
    d_a2 = lo2 if wr[0] <= 1.0 else 0.0
    d_spec = min(A["leak"]["spec_factor"] * d_x2, d_a2) if d_a2 > 0 else A["leak"]["spec_factor"] * d_x2
    lk_spec = leak_element(d_spec, Lk, nom_iso)
    C_tot_iso = nom_iso["C1"] + nom_iso["C2"]
    tau_leak_chamber = lk_spec["R"] * C_tot_iso
    w_s = A["leak"]["slot_w_mm"] * MM
    h_slot = (12 * nom["mu"] * Lk / (w_s * lk_spec["R"])) ** (1 / 3.0)
    h_spec = abs(H_of([fl], nom, leak=leak_element(d_spec, Lk, nom))[0])
    # feasible acceptance test: before the membrane is fitted, a gasketed fixture with V_test seals on the
    # membrane seat; V_test + chamber decay through (labyrinth + bond leak) to atmosphere, read by an
    # external transducer.  C_test >> C_chamber stretches tau to seconds.
    V_t = A["leak"]["test_volume_mm3"] * MM ** 3
    C_t = V_t / P_STD
    tau_test = (lk_spec["R"] + nom_iso["R_v"] + nom_iso["R_f"] + nom_iso["R_s"]) * (C_t + C_tot_iso)
    dp_t = A["leak"]["test_dp_Pa"]
    decay_10s = dp_t * (1 - math.exp(-10.0 / tau_test))
    gate("A6 bond-leak spec derived (leak at spec keeps A2 and costs < 1 dB at the comb lobe)",
         d_a2 > 0 and 20 * math.log10(h_spec / h0) < 1.0,
         "pinhole that doubles |H| (-6 dB, attenuation lost) at %.0f Hz: d = %.1f um; A2 limit at worst corner: "
         "d = %.1f um; SPEC d_eq <= %.1f um (%.2f dB at spec) == bond slot h <= %.2f um over %.0f mm "
         "(chamber-only leak-down tau %.3f s - too fast for the 32 Hz DPS310 to verify)"
         % (fl, d_x2 * 1e6, d_a2 * 1e6, d_spec * 1e6, 20 * math.log10(h_spec / h0), h_slot * 1e6,
            A["leak"]["slot_w_mm"], tau_leak_chamber))
    print("    leak-down acceptance test: before fitting the membrane, seal a %.0f ml fixture (O-ring on the seat "
          "face, external 0.1 Pa-resolution differential transducer) over the plenum, raise +%.0f Pa, isolate, "
          "log 10 s.  At the spec leak tau_test = %.1f s (decay %.0f Pa in 10 s); ACCEPT if fitted tau >= %.1f s; "
          "fixture blank (sealed dummy cap) must show tau >= %.0f s."
          % (A["leak"]["test_volume_mm3"] / 1000, dp_t, tau_test, decay_10s, tau_test,
             A["leak"]["blank_factor"] * tau_test))
    rep["acoustic"] = {
        "volumes_mm3": vols, "foam_compression": comp, "membrane_mech": mm_,
        "nominal": {"R_mem": nom["R_m"], "R_septum": nom["R_s"], "R_vent": nom["R_v"], "M_vent": nom["M_v"], "R_foam": nom["R_f"],
                    "C_plenum": nom["C_p"], "C1": nom["C1"], "C2": nom["C2"], "C_d": nom["C_d"],
                    "tau_adiabatic_ms": tau_nom_ad * 1e3, "tau_isothermal_ms": tau_nom_iso * 1e3,
                    "fc_Hz": fc, "attenuation_dB_TL": att, "attenuation_dB_lumped": att_l,
                    "lumped_vent_helmholtz_Hz": f0_lump},
        "thermal_boundary_layer_mm": dth, "pore_mm": pore, "T_corners_C": T_corners,
        "worst": {"max_gain": worst["gain"][0], "max_gain_corner": ck(worst["gain"][1]),
                  "residual_Pa": wr[2], "residual_limit_Pa": wr[3], "residual_ratio": wr[0],
                  "residual_f_Hz": wr[4], "residual_corner": ck(wr[1]), "input_peak_Pa": P_in, "spl_dB": spl,
                  "attenuation_dB": att_worst, "tau_ms": tw * 1e3, "tau_corner": ck(kw),
                  "tau_sealevel_ms": ts * 1e3, "lag_30mps_m": lag30, "lag_10mps_m": lag10,
                  "lag_30mps_3000m_m": A["climb_mps"][0] * tw,
                  "lumped": {"max_gain": worst["lump_gain"], "residual_ratio": worst["lump_ratio"],
                             "tau_ms": worst["lump_tau"] * 1e3, "lumped_minus_TL_dB": worst["dlump_dB"]}},
        "leak": {"d_x2_um": d_x2 * 1e6, "d_A2_um": d_a2 * 1e6, "d_spec_um": d_spec * 1e6,
                 "slot_h_spec_um": h_slot * 1e6, "slot_w_mm": A["leak"]["slot_w_mm"],
                 "chamber_leakdown_tau_s": tau_leak_chamber, "test_tau_min_s": tau_test,
                 "test_volume_ml": A["leak"]["test_volume_mm3"] / 1000, "test_dp_Pa": dp_t,
                 "test_decay_10s_Pa": decay_10s},
        "corners": len(corners),
    }

    # ------------------------------------------------------------------ 5
    print("== 5. water ingress ==")
    W = A["water"]
    th_a, th_r = math.radians(g["coat_theta_adv"]), math.radians(g["coat_theta_rec"])
    a_m = g["vent_a"] * MM
    q_fwd = 0.5 * 1.225 * W["v_forward_mps"] ** 2
    q_pw = 0.5 * 1.225 * W["v_propwash_mps"] ** 2
    q_max = max(q_fwd, q_pw)
    dp_dive = 1.225 * G0 * W["dive_m"]
    p_hi = max(A["env"]["p_amb_Pa"])
    Th, Tw = T_hot + 273.15, W["T_water_C"] + 273.15
    dp_contr = p_hi * (1 - Tw / Th)                         # sealed (wetted membrane) air cooled hot -> water
    load = q_max + dp_dive + dp_contr
    dp_imm = W["rho_w"] * G0 * W["immersion_m"]
    load_imm = dp_imm + dp_dive + dp_contr
    wep = g["mem_wep_min"]
    gate("W1 membrane WEP >= %.1f x (dynamic pressure + dive + thermal-contraction suction)" % W["wep_margin"],
         wep >= W["wep_margin"] * load,
         "WEP %.0f kPa vs %.1f kPa = q %.2f (%.0f m/s) + dive %.2f (%.0f m) + contraction %.1f kPa (air sealed by a "
         "wetted membrane cooled %d C -> %.0f C at %.0f kPa: -%.0f %% volume) -> margin x%.2f"
         % (wep / 1e3, load / 1e3, q_max / 1e3, W["v_forward_mps"], dp_dive / 1e3, W["dive_m"], dp_contr / 1e3,
            T_hot, W["T_water_C"], p_hi / 1e3, 100 * (1 - Tw / Th), wep / load))
    gate("W1b membrane WEP >= %.2f x (%.0f m immersion + dive + contraction)" % (W["wep_margin_immersion"],
                                                                               W["immersion_m"]),
         wep >= W["wep_margin_immersion"] * load_imm,
         "%.1f kPa load -> margin x%.2f" % (load_imm / 1e3, wep / load_imm))
    # dry cooling: contraction simply draws air in through the membrane
    e_cool = elements(g, vols, a0 - tol, T_hot, min(A["env"]["p_amb_Pa"]), True, 2.0, g["mem_r_hi"])
    V_all = (vols["foam_pore_air_mm3"] + vols["gap_mm3"] + vols["vent_mm3"] + vols["plenum_mm3"]) * MM ** 3
    dVdt = V_all * W["cool_rate_K_s"] / Th
    dp_cool = r_dc_series(e_cool) * dVdt
    tau_eq = tau_of(e_cool)
    v_contr = V_all * (1 - Tw / Th) / MM ** 3
    t_cool = (Th - Tw) / W["cool_rate_K_s"]
    gate("W2 dry cooling: contraction equilibrates through the membrane (no suction build-up)",
         tau_eq < 0.01 * t_cool and dp_cool < 0.01 * wep,
         "contraction %.1f mm^3 (%.0f %% of %.1f mm^3) is drawn in through R_dc %.2e Pa s/m^3: equilibration "
         "tau %.1f ms (worst corner) << %.0f s cooling time; at %.0f K/s the steady suction is %.2f Pa (<< WEP)"
         % (v_contr, 100 * (1 - Tw / Th), V_all / MM ** 3, r_dc_series(e_cool), tau_eq * 1e3, t_cool,
            W["cool_rate_K_s"], dp_cool))
    w_wep = mm_["w_per_Pa_m"] * wep / MM
    gate("W3 membrane deflection at WEP <= %.0f %% of the plenum depth (active area never blinded)"
         % (100 * W["deflect_frac_max"]), w_wep <= W["deflect_frac_max"] * g["mem_plenum_d"],
         "clamped strip b = %.1f mm, D = %.2e N m (E %.0f MPa low bound): w_max %.3f mm at %.0f kPa vs plenum "
         "%.2f mm" % (mm_["b_m"] / MM, mm_["D_Nm"], g["mem_e_eff"] / 1e6, w_wep, wep / 1e3, g["mem_plenum_d"]))
    covered = (abs(g["vent_y_out"]) + g["vent_a"] / 2 <= g["mem_pw"] / 2
               and g["mem_zc"] - g["mem_ph"] / 2 <= g["vent_z_out"] - g["vent_a"] / 2
               and g["vent_z_out"] + g["vent_a"] / 2 <= g["mem_zc"] + g["mem_ph"] / 2
               and g["mem_w"] / 2 <= g["cap_l"] / 2 - g["corner_r"] and g["mem_seat_d"] > g["mem_t"])
    gate("W4 membrane is the only water path: outer port opens inside the sealed plenum, membrane recessed "
         "within the flat east face", covered,
         "port y %.2f z %.2f inside plenum %.1f x %.1f mm; membrane %.1f x %.1f mm (ring %.1f mm) within flat face "
         "%.1f x %.1f mm, %.2f mm below the face"
         % (g["vent_y_out"], g["vent_z_out"], g["mem_pw"], g["mem_ph"], g["mem_w"], g["mem_h"], g["mem_ring"],
            g["cap_l"] - 2 * g["corner_r"], g["cap_h"], g["mem_seat_d"] - g["mem_t"]))
    p_entry = 4 * W["gamma_N_m"] * abs(math.cos(th_a)) / a_m                      # [YL]
    p_pin = 4 * W["gamma_N_m"] * abs(math.cos(th_r) - math.cos(th_a)) / a_m
    bond = W["rho_w"] * G0 * a_m ** 2 / W["gamma_N_m"]
    head = W["rho_w"] * G0 * (g["vent_z_in"] - g["vent_z_out"] + g["vent_a"]) * MM
    warn("labyrinth is NOT self-draining",
         "Bond number %.3f; contact-angle hysteresis (th_a %.0f / th_r %.0f deg) pins a slug with %.0f Pa vs at "
         "most %.0f Pa gravity head over the full %.1f mm height -> water that gets past a breached membrane "
         "stays and blocks the vent (secondary hydrophobic entry pressure only %.0f Pa).  Breached membrane = "
         "replace cap; the membrane (W1/W4) is the water barrier."
         % (bond, g["coat_theta_adv"], g["coat_theta_rec"], p_pin, head, g["vent_z_in"] - g["vent_z_out"], p_entry))
    rep["water"] = {"wep_Pa": wep, "q_max_Pa": q_max, "dp_dive_Pa": dp_dive, "dp_contraction_Pa": dp_contr,
                    "load_Pa": load, "margin": wep / load, "load_immersion_Pa": load_imm,
                    "margin_immersion": wep / load_imm, "dry_cool_tau_ms": tau_eq * 1e3,
                    "dry_cool_suction_Pa": dp_cool, "contraction_mm3": v_contr,
                    "membrane_deflection_at_wep_mm": w_wep, "labyrinth": {
                        "self_draining": False, "bond_number": bond, "pinning_Pa": p_pin,
                        "gravity_head_Pa": head, "entry_pressure_Pa": p_entry}}

    # ------------------------------------------------------------------ 6
    print("== 6. light ==")
    dy = abs(g["vent_y_end"] - g["vent_y_out"])
    dz = g["vent_z_in"] - g["vent_z_out"]
    turns = 2 + 2 * (g["vent_lanes"] - 1)
    gate("L1 no line of sight through the labyrinth (membrane is translucent)",
         dy > g["vent_a"] and dz > g["vent_a"] and turns >= 6,
         "ports offset %.2f mm in Y and %.2f mm in Z (channel %.2f mm), %d right-angle turns"
         % (dy, dz, g["vent_a"], turns))
    gate("L2 opaque foam covers the full cavity plan with interference (no edge gap)",
         g["foam_fit"] > A["foam"]["cut_tol_mm"] / 2 and g["vent_z_in"] - g["vent_a"] / 2 >= g["foam_z0"],
         "foam oversize %.2f mm/side > cut tol %.2f mm/side; inner port opens into the foam (z >= %.2f mm)"
         % (g["foam_fit"], A["foam"]["cut_tol_mm"] / 2, g["foam_z0"]))
    gap_view = math.degrees(math.atan(g["air_gap"] / max(g["u4_x0"] - g["cav_x0"], 1e-3)))
    opaque_note = bool(re.search(r"must be OPAQUE", scad_text)) and bool(re.search(r"black-dyed", scad_text))
    gate("L3 cap and foam opacity requirements recorded in the SCAD material notes", opaque_note,
         "U4 lid port sees the side walls up to %.0f deg elevation through the %.1f mm gap -> walls must be opaque"
         % (gap_view, g["air_gap"]))

    # ------------------------------------------------------------------ 7
    print("== 7. thermal / mechanical fit ==")
    gate("T1 cap foot (board at the FET site) <= HDT - %.0f C" % A["thermal"]["hdt_margin_C"],
         t_site + A["thermal"]["hdt_margin_C"] <= g["resin_hdt"],
         "max t_site %.1f C (%s) vs HDT %.0f C" % (t_site, k_site, g["resin_hdt"]))
    gap_q11 = (g["q11_cy_k"] - g["q11_l"] / 2) - g["cap_y1_k"]
    gate("T2 Q11 junction bound <= HDT (cap never hotter than the nearest FET junction)", tj <= g["resin_hdt"],
         "max Tj(HS) %.1f C over %s, Q11 courtyard %.2f mm from the cap" % (tj, sorted(sc), gap_q11))
    mm_c = A["thermal"]["mat_margin_C"]
    gate("T3 foam (melamine) and membrane laminate rated above the site temperature + %.0f C" % mm_c,
         t_site + mm_c <= g["foam_t_max"] and t_site + mm_c <= g["mem_t_max"],
         "t_site %.1f C vs foam %.0f C, membrane %.0f C (PU foam ~100 C would fail -> melamine)"
         % (t_site, g["foam_t_max"], g["mem_t_max"]))
    if t_board > A["thermal"]["dps310_max_C"]:
        warn("DPS310 operating range", "board reaches %.1f C at the rating scenarios > DPS310 %.0f C max: baro "
             "site temperature is a board-level open item (not fixed by the cap); acoustic corners run to %d C"
             % (t_board, A["thermal"]["dps310_max_C"], T_hot))
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
    gate("M5 board edge and Q4/Q11 courtyards",
         edge >= g["edge_clear_min"] and gq4 >= g["court_clear"] and gap_q11 >= g["court_clear"],
         "edge %.2f mm (min %.2f), Q4 gap %.2f mm, Q11 gap %.2f mm (min %.2f)"
         % (edge, g["edge_clear_min"], gq4, gap_q11, g["court_clear"]))
    pass_clear = g["cav_x1"] - (g["u4_x1"] + g["pass_gap"] + g["pass_w"])
    gate("M6 sensor clearances (U4 / passives / foam)",
         g["air_gap"] >= g["air_gap_min"] and pass_clear >= g["pass_wall_clear"] and g["foam_t"] >= g["foam_t_min"],
         "air gap %.2f mm (min %.2f), passives-to-vent-wall %.2f mm (min %.2f), foam %.2f mm (min %.2f)"
         % (g["air_gap"], g["air_gap_min"], pass_clear, g["pass_wall_clear"], g["foam_t"], g["foam_t_min"]))
    rep["thermal"] = {"t_site_max_C": t_site, "t_board_max_C": t_board, "tj_hs_max_C": tj, "scenarios": sorted(sc)}

    nfail = sum(1 for x in GATES if not x["pass"])
    rep.update({"gates": GATES, "warnings": WARNS, "skips": SKIPS, "result": "PASS" if nfail == 0 else "FAIL"})
    out = os.path.join(HERE, "baro_acoustic_report.json")
    with open(out, "w") as fh:
        json.dump(rep, fh, indent=1, default=str)
    print("\nwrote %s" % out)
    print("RESULT: %s (%d gates, %d failed, %d skipped, %d warnings)" % (rep["result"], len(GATES), nfail,
                                                                       len(SKIPS), len(WARNS)))
    return 1 if nfail else 0


if __name__ == "__main__":
    sys.exit(main())
