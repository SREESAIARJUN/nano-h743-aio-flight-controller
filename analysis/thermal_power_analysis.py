#!/usr/bin/env python3
"""
Nano H743 AIO - 4-in-1 ESC power-stage loss & thermal analysis (Revision 8.0, gate drive updated in Revision 9.0)
============================================================================
Replaces the Rev <=7 script, which multiplied the TOTAL board loss by one
theta_JA (so 4 ESCs were treated as one junction), used 1.4 mOhm (the 4.5 V
RDS(on)) at every gate voltage, and labelled a 218 degC result "certified safe".

Model (all inputs below are cited or flagged as assumptions):
  * Losses per FET for 6-step commutation with complementary (synchronous)
    PWM at duty D: in each 120 deg window the PWM phase HS conducts D, its LS
    conducts (1-D), and the sink-phase LS conducts 100 %.  Over a full
    electrical cycle: P_cond,HS = I^2 R D/3 ; P_cond,LS = I^2 R (2-D)/3.
    HS adds switching (gate-charge based transition times), Coss and Qrr
    loss; LS adds body-diode conduction during dead time.
    RDS(on)(Tj) is iterated to convergence.
  * Thermal network, geometry READ BACK from kicad/nano-h743-aio.kicad_pcb:
      junction -> case (Rth(ch-c) 0.88 K/W max) -> SAC305 joint ->
        HS: 4x4 POFV barrel array to the nearest VBAT plane (In2 top / In3
            bottom) in parallel with the 1080 prepreg to the In1/In4 GND plane
        LS: drain tab + L1/L6 phase pour -> 1080 prepreg -> In1/In4 GND plane
            (no drain vias possible: see gen_power_stage_pcb.py)
      -> radial spreading in the 4 x 2 oz planes from the half-bridge site
      -> board-to-air, both faces, fin efficiency from the plane k*t.
  * Board transient for the 35 A burst: lumped C of copper + FR4 + FETs.

Usage: python3 analysis/thermal_power_analysis.py [--board PCB] [--no-plot]
Writes analysis/thermal_report.json and analysis/thermal_dissipation_curve.png
Exit code 1 if any MUST-PASS scenario fails.
"""
import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, "tools"))
PCB = os.path.join(REPO, "kicad", "nano-h743-aio.kicad_pcb")

# ---------------------------------------------------------------- device data
# Toshiba TPHR8504PL datasheet Rev.5.0 (2019-10-24)  [Rev 9: label corrected]
FET = {
    "rds_max_10v": 0.85e-3,       # ID=50 A, Ta=25 C
    "rds_max_4v5": 1.40e-3,
    "rth_jc": 0.88,               # K/W max
    "tch_max": 175.0,
    "qg_10v": 103e-9, "qgd": 12.4e-9, "qsw": 23e-9, "qoss": 85.4e-9, "qrr": 93e-9,
    "rg_int_max": 1.1,
    "vf_diode": 0.80,             # body diode at ~20 A (datasheet VDSF -1.2 V max at 150 A)
    "exposed_pad_mm2": 4.25 * 3.5,
    "mass_g": 0.087,
}
# ASSUMPTIONS (flagged in the report):
ASSUME = {
    "rds_tempco_per_K": 0.0072,   # Rev 9: datasheet Fig. 8.9 (10 V) 0.70 -> ~1.33 mOhm, 25 -> 150 C (was 0.0056)
    "v_plateau": 3.3,             # Rev 9: datasheet Fig. 8.10 / 8.1 at 20-50 A (was 3.0)
    # Rev 9: gate drive taken from am32_esc/gate_drive_report.json (see load_gate_drive()):
    #   r_drv_source/sink  FD6288Q slow corner from VOH/VOL max @ 100 mA (9.0 / 4.5 ohm)
    #   rg_on / rg_off     selected network (15 ohm on, Schottky bypass off)
    #   dead_time_s        worst-case body-diode conduction per edge (outgoing channel off -> incoming on)
    #   switching energy   e_on + e_off + Miller shoot-through vs (I, Vbus) from the same simulator
    #                      (Rev 9 review: the Qsw/I_plateau formula was optimistic on turn-off);
    #                      switching_times() is kept only for the reported t_on/t_off
    "duty_at_rated": 0.90,
    "solder_bond_um": 50.0, "solder_void_frac": 0.30, "k_sac305": 58.0,
    "via_plating_um": 18.0,       # IPC-6012 class 2 minimum average hole-wall Cu
    "k_cu": 385.0, "k_fr4_z": 0.30, "rho_cu_20": 1.72e-8, "rho_tempco": 0.0039,
    "plane_squares": 0.75,        # battery pad -> 6 half-bridge sites, VBAT and GND each
    "shunt_ohm": 0.5e-3,          # README current shunt (on-board, pending layout)
    "p_misc_w": 0.6,              # H743 @480 MHz + regulators + IMUs
    "h_flight": 50.0,             # W/m2K prop wash, conservative end of 50-100
    "h_bench": 12.0,              # W/m2K still air + radiation
    "t_amb": 35.0,
    "ls_pour_mm2": 55.0,          # effective L1/L6 pour area coupling an LS drain to In1/In4
    "cvol_cu": 3.45, "cvol_fr4": 2.0,  # J/(cm3 K)
    # Soldered leads = convecting pin fins WITH internal I^2R generation (same h as board).
    # HARNESS REQUIREMENT for the 4 x 20 A rating: battery leads 12 AWG silicone
    # (3.31 mm2, OD 3.6 mm, carry I_bat), motor leads 20 AWG (0.518 mm2, OD 1.5 mm,
    # carry I_motor*sqrt(2/3) rms for 6-step 120 deg conduction).
    # (name, count, Cu area m2, OD m, current kind)
    "wires": [("battery_12AWG", 2, 3.31e-6, 3.6e-3, "bat"), ("motor_20AWG", 12, 0.518e-6, 1.5e-3, "motor")],
    "wires_alt_14AWG_22AWG": [("battery_14AWG", 2, 2.08e-6, 2.6e-3, "bat"), ("motor_22AWG", 12, 0.326e-6, 1.2e-3, "motor")],
    "vdrv_qg_slope_F": 7.37e-9,   # Qg(V) above 10 V ~ Qg(10 V) + Ciss*(V-10) (gate charge curve linear region)
    "am32_margin_K": 5.0,         # burst must end >= 5 K below the AM32 limits.temperature (115 C)
}
# Published per-motor ratings (all 4 ESCs loaded, D=0.9, 24 kHz, Tamb 35 C, 12 AWG
# battery / 20 AWG motor leads).  Derived from this model in Rev 8.0; the gate checks
# the published numbers, it does not tune the model to fit them.
# Rev 9.0 derating (was 17 / 22 / 20 / 28 A): the Rev 8 loss model used 60 ns of body-diode
# conduction per edge; with the FD6288Q's fixed 100-300 ns interlock plus AM32 DEAD_TIME 34
# (283 ns) the worst-case window is ~0.7 us (am32_esc/gate_drive_report.json), and the
# datasheet RDS(on) tempco / Miller plateau corrections add conduction loss.
RATING = {"cont_conservative_A": 16.0, "burst10s_conservative_A": 21.0,   # h = 50 W/m2K
          "cont_typical_A": 20.0, "burst10s_typical_A": 25.0}             # h = 80 W/m2K
GATE_REPORT = os.path.join(REPO, "am32_esc", "gate_drive_report.json")


def load_gate_drive(path=GATE_REPORT):
    """Rev 9: single source of truth for driver R, gate network and dead time."""
    with open(path) as f:
        g = json.load(f)
    if g.get("status") != "PASS":
        raise SystemExit(f"{path} status {g.get('status')}: run am32_esc/gate_drive_design.py first")
    net = g["selected_gate_network"]
    r_off = net["r_off_ohm"]
    ASSUME.update({
        "r_drv_source": g["driver"]["r_src"]["slow"],
        "r_drv_sink": g["driver"]["r_snk"]["slow"],
        "rg_on": net["rg_on_ohm"],
        # Schottky bypass: ~0.3 V knee + ~0.5 ohm dynamic + R_off; at the 3.3 V plateau the
        # bypass carries the discharge, so model it as R_off + 0.5 ohm || Rg_on
        "rg_off": net["rg_on_ohm"] if r_off is None else
                  1.0 / (1.0 / net["rg_on_ohm"] + 1.0 / (r_off + 0.5)),
        "dead_time_s": g["dead_time"]["body_diode_per_edge_ns"]["max"] * 1e-9,
        "am32_dead_time_code": g["dead_time"]["DEAD_TIME"],
        "cgs_ext_F": net["cgs_ext_nF"] * 1e-9,
    })
    ESW.clear()
    ESW.update(g["switching_energy_table"])
    return g


ESW = {}


def _interp(xs, ys, x):
    if x <= xs[0]:
        return ys[0] * x / xs[0]                  # energies -> 0 with current
    for x0, x1, y0, y1 in zip(xs, xs[1:], ys, ys[1:]):
        if x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return ys[-2] + (ys[-1] - ys[-2]) * (x - xs[-2]) / (xs[-1] - xs[-2])


def switching_energy(i, vbat):
    """Rev 9: HS turn-on + turn-off + Miller shoot-through energy (J) from the calibrated
    gate-charge simulator in am32_esc/gate_drive_design.py (slow driver corner, plateau at the
    load current), interpolated in current and linearly in bus voltage."""
    rows = sorted(ESW["rows"], key=lambda r: r["vbus"])
    per_v = []
    for r in rows:
        tot = [a + b + c for a, b, c in zip(r["e_on_uJ"], r["e_off_uJ"], r["e_st_uJ"])]
        per_v.append((r["vbus"], _interp(ESW["i_A"], tot, i)))
    (v0, e0), (v1, e1) = per_v[0], per_v[-1]
    return 1e-6 * (e0 + (e1 - e0) * (vbat - v0) / (v1 - v0))


LIMITS = {"tj_cont": 125.0, "tj_burst": 150.0, "t_board_cont": 105.0}
AM32_CUTOFF = 115.0


# ------------------------------------------------------- geometry read-back
def read_board(path):
    sys.path.insert(0, os.path.join(REPO, "tools"))
    import kicad_sexpr as ks
    b = ks.load(path)
    g = {}
    st = ks.stackup(b)
    g["copper"] = {l["name"]: l["thickness"] for l in st if l["type"] == "copper"}
    g["dielectrics"] = [l["thickness"] for l in st if l["type"] in ("prepreg", "core")]
    edge = [r for r in ks.kids(b, "gr_rect") if ks.val(r, "layer") == "Edge.Cuts"][0]
    (x0, y0), (x1, y1) = [tuple(float(v) for v in ks.kid(edge, k)[1:3]) for k in ("start", "end")]
    g["board_w"], g["board_h"] = x1 - x0, y1 - y0
    hs, ls = [], []
    mounts = 0
    for fp in ks.footprints(b):
        th = [p["drill"] for p in fp["pads"] if p["type"] == "thru_hole"]
        if "ThermalVias" in fp["lib"]:
            hs.append(th)
        elif "SourceVias" in fp["lib"]:
            ls.append(th)
        mounts += sum(1 for p in fp["pads"] if p["type"] == "np_thru_hole")
    g["hs_via_arrays"], g["ls_via_arrays"] = hs, ls
    g["ls_row_vias"] = len(ks.kids(b, "via"))
    g["mount_holes"] = mounts
    return g


# --------------------------------------------------------------- loss model
def rds(tj, vgs_class):
    base = FET["rds_max_10v"] if vgs_class == "10V" else FET["rds_max_4v5"]
    return base * (1 + ASSUME["rds_tempco_per_K"] * (tj - 25.0))


def switching_times(v_drive):
    i_on = (v_drive - ASSUME["v_plateau"]) / (ASSUME["rg_on"] + FET["rg_int_max"] + ASSUME["r_drv_source"])
    i_off = ASSUME["v_plateau"] / (ASSUME["rg_off"] + FET["rg_int_max"] + ASSUME["r_drv_sink"])
    return FET["qsw"] / i_on, FET["qsw"] / i_off


def fet_losses(i, vbat, fsw, duty, tj_hs, tj_ls, vgs_class, v_drive):
    t_on, t_off = switching_times(v_drive)
    hs = {"cond": i * i * rds(tj_hs, vgs_class) * duty / 3.0,
          "sw": switching_energy(i, vbat) * fsw / 3.0,
          "coss_qrr": (0.5 * FET["qoss"] + FET["qrr"]) * vbat * fsw / 3.0}
    ls = {"cond": i * i * rds(tj_ls, vgs_class) * (2.0 - duty) / 3.0,
          "deadtime": FET["vf_diode"] * i * 2 * ASSUME["dead_time_s"] * fsw / 3.0}
    return hs, ls, (t_on, t_off)


# ------------------------------------------------------------ thermal model
def r_via_barrel(length_m, drill_m, n):
    t = ASSUME["via_plating_um"] * 1e-6
    a = math.pi * (drill_m * t - t * t)
    return length_m / (ASSUME["k_cu"] * a) / n


def network(g):
    cu = g["copper"]
    d = g["dielectrics"]
    inner_kt = ASSUME["k_cu"] * sum(cu[n] for n in ("In1.Cu", "In2.Cu", "In3.Cu", "In4.Cu")) * 1e-3
    outer_kt = ASSUME["k_cu"] * (cu["F.Cu"] + cu["B.Cu"]) * 1e-3 * 0.5   # ~50 % outer fill
    kt = inner_kt + outer_kt
    a_board = g["board_w"] * g["board_h"] * 1e-6 - g["mount_holes"] * math.pi * (1.6e-3) ** 2
    r_sold = (ASSUME["solder_bond_um"] * 1e-6) / (ASSUME["k_sac305"] * FET["exposed_pad_mm2"] * 1e-6
                                                   * (1 - ASSUME["solder_void_frac"]))
    # depth from an outer drain pad to the nearest VBAT plane (In2 from top, In3 from bottom)
    depth_vbat = (d[0] + cu["In1.Cu"] + d[1] + 0.5 * cu["In2.Cu"]) * 1e-3
    n_hs = len(g["hs_via_arrays"][0])
    drill = g["hs_via_arrays"][0][0] * 1e-3
    r_vias = r_via_barrel(depth_vbat, drill, n_hs)
    land_mm2 = 4.7 * 4.8
    r_pp_hs = (d[0] * 1e-3) / (ASSUME["k_fr4_z"] * land_mm2 * 1e-6)
    r_hs_local = 1.0 / (1.0 / r_vias + 1.0 / r_pp_hs)
    r_pp_ls = (d[0] * 1e-3) / (ASSUME["k_fr4_z"] * ASSUME["ls_pour_mm2"] * 1e-6)
    # radial spreading: half-bridge site (2 lands, 10.1 x 7 mm) -> its 1/6 share of the board
    r1 = math.sqrt(10.1 * 7.0 / math.pi) * 1e-3
    r2 = math.sqrt(a_board / 6.0 / math.pi)
    r_spread = math.log(r2 / r1) / (2 * math.pi * kt)
    return {"kt_w_per_k": kt, "a_board_m2": a_board, "r_solder": r_sold, "r_vias_hs": r_vias,
            "n_hs_vias": n_hs, "via_depth_mm": depth_vbat * 1e3, "r_prepreg_hs": r_pp_hs,
            "r_hs_local": r_hs_local, "r_prepreg_ls": r_pp_ls, "r_spread_site": r_spread}


def rho_cu(t):
    return ASSUME["rho_cu_20"] * (1 + ASSUME["rho_tempco"] * (t - 20.0))


def wire_terms(h, i_motor, i_bat, t_wire, wires=None, wire_scale=None):
    """Infinite pin fin with uniform generation q' = I^2 rho/A:
         q_from_board = g (T_b - T_amb - theta),  g = sqrt(h P k A),  theta = q'/(h P).
    Returns (sum g, sum g*theta, detail)."""
    g_sum = gt_sum = 0.0
    det = {}
    for name, n, a, od, kind in (wires or ASSUME["wires"]):
        cur = i_bat if kind == "bat" else i_motor * math.sqrt(2.0 / 3.0)
        cur *= (wire_scale or {}).get(name, 1.0)
        hp = h * math.pi * od
        g = math.sqrt(hp * ASSUME["k_cu"] * a)
        theta = cur * cur * rho_cu(t_wire) / a / hp
        g_sum += n * g
        gt_sum += n * g * theta
        det[name] = {"I_A": cur, "g_W_K_each": g, "self_heating_rise_K": theta}
    return g_sum, gt_sum, det


def g_plate(net, h):
    # square plate both faces; fin efficiency of the copper-loaded laminate
    m = math.sqrt(2 * h / net["kt_w_per_k"])
    half = math.sqrt(net["a_board_m2"]) / 2
    eta = math.tanh(m * half) / (m * half)
    return h * 2 * net["a_board_m2"] * eta, eta


def heat_capacity(g):
    cu = g["copper"]
    a_cm2 = g["board_w"] * g["board_h"] / 100.0
    cu_cm = (sum(cu[n] for n in ("In1.Cu", "In2.Cu", "In3.Cu", "In4.Cu")) + 0.5 * (cu["F.Cu"] + cu["B.Cu"])) / 10
    fr4_cm = sum(g["dielectrics"]) / 10
    return (a_cm2 * cu_cm * ASSUME["cvol_cu"] + a_cm2 * fr4_cm * ASSUME["cvol_fr4"]
            + 24 * FET["mass_g"] * 0.6)


def qg_at(v):
    return FET["qg_10v"] + ASSUME["vdrv_qg_slope_F"] * max(0.0, v - 10.0)


def bridge_resistance(t):
    """HS-source -> LS-drain/motor-pad phase bridge in 1 oz outer copper, per face,
    from the generator geometry (top: full-depth bridge, bottom: motor-strip neck)."""
    import gen_power_stage_pcb as gen
    cu = 0.035e-3
    length = ((gen.LAND_HALF_W + gen.G1 + gen.LAND_HALF_W) - (gen.MOTOR_PAD[0] / 2) + 0.6) * 1e-3
    w_top = (gen.V_C - gen.DRAIN_BODY_Y1 - 0.5 - 0.4) * 1e-3
    w_bot = (gen.LAND_EDGE_V - 0.2 - 0.4) * 1e-3
    r = lambda w: rho_cu(t) * length / (w * cu)  # noqa: E731
    return r(w_top), r(w_bot), length, w_top, w_bot


def solve(g, net, i, vbat, fsw, h, vgs_class, v_drive, duty=None, t_board_override=None, wires=None,
          wire_scale=None):
    duty = ASSUME["duty_at_rated"] if duty is None else duty
    gp, eta = g_plate(net, h)
    tj_hs = tj_ls = 80.0
    t_board = 80.0
    i_bat = 4 * i * duty
    for _ in range(200):
        hs, ls, tsw = fet_losses(i, vbat, fsw, duty, tj_hs, tj_ls, vgs_class, v_drive)
        p_hs, p_ls = sum(hs.values()), sum(ls.values())
        p_fets = 12 * (p_hs + p_ls)                       # 4 ESC x 3 phases
        rho = rho_cu(t_board)
        t_vb = (g["copper"]["In2.Cu"] + g["copper"]["In3.Cu"]) * 1e-3
        t_gn = (g["copper"]["In1.Cu"] + g["copper"]["In4.Cu"]) * 1e-3
        p_planes = i_bat ** 2 * rho * ASSUME["plane_squares"] * (1 / t_vb + 1 / t_gn)
        p_shunt = i_bat ** 2 * ASSUME["shunt_ohm"]
        # gate drive: 2 FETs switch per ESC (PWM HS + complementary LS), energy Qg*V
        # dissipated in driver + Rg on this board
        p_gate = 4 * 2 * (qg_at(v_drive) + ASSUME.get("cgs_ext_F", 0.0) * v_drive) * v_drive * fsw
        rbt, rbb, *_ = bridge_resistance(t_board)
        p_bridge = 6 * i * i * duty / 3.0 * (rbt + rbb)   # 6 phases per face, HS conduction only
        p_total = p_fets + p_planes + p_shunt + p_gate + p_bridge + ASSUME["p_misc_w"]
        gw, gwt, wdet = wire_terms(h, i, i_bat, t_board, wires, wire_scale)
        g_tot = gp + gw
        prev_b = t_board
        if t_board_override is None:
            t_board = ASSUME["t_amb"] + (p_total + gwt) / g_tot
        else:
            t_board = t_board_override
        # each half-bridge site carries top + bottom half-bridges (2 HS + 2 LS)
        t_site = t_board + 2 * (p_hs + p_ls) * net["r_spread_site"]
        r_jc = FET["rth_jc"] + net["r_solder"]
        prev = tj_hs
        tj_hs = t_site + p_hs * (r_jc + net["r_hs_local"])
        tj_ls = t_site + p_ls * (r_jc + net["r_prepreg_ls"])
        if abs(tj_hs - prev) < 1e-5 and abs(t_board - prev_b) < 1e-5:
            break
    return {"I_motor_A": i, "vbat_V": vbat, "fsw_kHz": fsw / 1e3, "h_W_m2K": h, "duty": duty,
            "vgs_class": vgs_class, "t_on_ns": tsw[0] * 1e9, "t_off_ns": tsw[1] * 1e9,
            "p_hs_W": {k: round(v, 4) for k, v in hs.items()}, "p_ls_W": {k: round(v, 4) for k, v in ls.items()},
            "p_fet_hs_W": p_hs, "p_fet_ls_W": p_ls, "p_fets_total_W": p_fets, "p_planes_W": p_planes,
            "p_shunt_W": p_shunt, "p_gate_drive_W": p_gate, "p_phase_bridge_W": p_bridge,
            "p_misc_W": ASSUME["p_misc_w"], "p_board_total_W": p_total,
            "g_plate_W_K": gp, "fin_eff": eta, "g_wires_W_K": gw, "wire_self_heating_term_W": gwt,
            "wires": wdet, "g_total_W_K": g_tot, "r_board_air_K_W": 1.0 / g_tot,
            "t_board_C": t_board, "t_site_C": t_site,
            "tj_hs_C": tj_hs, "tj_ls_C": tj_ls, "rds_hs_mohm": rds(tj_hs, vgs_class) * 1e3}


def burst(g, net, base, i_burst, seconds, vbat, fsw, h, vgs_class, v_drive):
    """Board: lumped C, exponential from the continuous operating point toward the
    sustained-burst equilibrium.  Leads: their own first-order lag
    tau_w = rho c A / (h P) (~10-20 s), so the wire self-heating term at the end
    of the burst is theta_cont + (1 - exp(-t/tau_w)) * (theta_burst - theta_cont)."""
    c = heat_capacity(g)
    i_bat_b, i_bat_c = 4 * i_burst * ASSUME["duty_at_rated"], 4 * base["I_motor_A"] * ASSUME["duty_at_rated"]
    wires = []
    for name, n, a, od, kind in ASSUME["wires"]:
        tau_w = ASSUME["cvol_cu"] * 1e6 * a / (h * math.pi * od)
        f = 1 - math.exp(-seconds / tau_w)
        ib = i_bat_b if kind == "bat" else i_burst
        ic = i_bat_c if kind == "bat" else base["I_motor_A"]
        i_eff = math.sqrt(ic * ic + f * (ib * ib - ic * ic))      # theta ~ I^2
        scale = i_eff / (ib if kind == "bat" else i_burst)
        wires.append((name, n, a, od, kind, scale, tau_w))
    eff = [(nm, n, a, od, k) for nm, n, a, od, k, _, _ in wires]
    # equivalent wire currents: emulate by scaling via a custom wire list per kind
    ss = solve(g, net, i_burst, vbat, fsw, h, vgs_class, v_drive,
               wires=[(nm, n, a / 1.0, od, k) for nm, n, a, od, k in eff], wire_scale={w[0]: w[5] for w in wires})
    tau = c / ss["g_total_W_K"]
    t_b = ss["t_board_C"] + (base["t_board_C"] - ss["t_board_C"]) * math.exp(-seconds / tau)
    r = solve(g, net, i_burst, vbat, fsw, h, vgs_class, v_drive, t_board_override=t_b,
              wire_scale={w[0]: w[5] for w in wires})
    r.update({"burst_s": seconds, "c_board_J_K": c, "tau_board_s": tau,
              "tau_wires_s": {w[0]: w[6] for w in wires},
              "t_board_equilibrium_if_sustained_C": ss["t_board_C"]})
    return r


def max_current(g, net, vbat, fsw, h, vgs_class, v_drive):
    lo, hi = 1.0, 80.0
    for _ in range(50):
        mid = (lo + hi) / 2
        r = solve(g, net, mid, vbat, fsw, h, vgs_class, v_drive)
        ok = (r["tj_hs_C"] <= LIMITS["tj_cont"] and r["tj_ls_C"] <= LIMITS["tj_cont"]
              and r["t_board_C"] <= LIMITS["t_board_cont"])
        lo, hi = (mid, hi) if ok else (lo, mid)
    return lo


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--board", default=PCB)
    ap.add_argument("--no-plot", action="store_true")
    ap.add_argument("--out", default=os.path.join(HERE, "thermal_report.json"))
    a = ap.parse_args()

    load_gate_drive()
    g = read_board(a.board)
    net = network(g)
    V4S, V2S = 16.8, 8.4
    FS48, FS24 = 48e3, 24e3
    hf, hb = ASSUME["h_flight"], ASSUME["h_bench"]

    H_TYP = 80.0
    R = RATING
    sc = {}
    # --- published ratings (MUST pass) ---------------------------------------------
    sc["R1_4S_cont_conservative"] = solve(g, net, R["cont_conservative_A"], V4S, FS24, hf, "10V", 16.8)
    sc["R2_2S_cont_conservative"] = solve(g, net, R["cont_conservative_A"], V2S, FS24, hf, "4.5V", 7.6)
    sc["R3_4S_burst10s_conservative"] = burst(g, net, sc["R1_4S_cont_conservative"], R["burst10s_conservative_A"],
                                              10, V4S, FS24, hf, "10V", 16.8)
    sc["R4_4S_cont_typical"] = solve(g, net, R["cont_typical_A"], V4S, FS24, H_TYP, "10V", 16.8)
    sc["R5_4S_burst10s_typical"] = burst(g, net, sc["R4_4S_cont_typical"], R["burst10s_typical_A"],
                                         10, V4S, FS24, H_TYP, "10V", 16.8)
    # --- informative ---------------------------------------------------------------
    sc["I1_legacy_4S_20A_conservative"] = solve(g, net, 20, V4S, FS24, hf, "10V", 16.8)
    sc["I2_4S_cont_typical_48kHz"] = solve(g, net, R["cont_typical_A"], V4S, FS48, H_TYP, "10V", 16.8)
    sc["I3_legacy_35A_10s_burst_typical"] = burst(g, net, sc["R4_4S_cont_typical"], 35, 10, V4S, FS24, H_TYP,
                                                  "10V", 16.8)
    sc["I4_4S_20A_static_bench"] = solve(g, net, 20, V4S, FS24, hb, "10V", 16.8)
    must = {k for k in sc if k.startswith("R")}
    verdict = {}
    for k, r in sc.items():
        bur = "burst" in k
        lim = LIMITS["tj_burst"] if bur else LIMITS["tj_cont"]
        tb_lim = (AM32_CUTOFF - ASSUME["am32_margin_K"]) if bur else LIMITS["t_board_cont"]
        ok = max(r["tj_hs_C"], r["tj_ls_C"]) <= lim and r["t_board_C"] <= tb_lim
        verdict[k] = {"pass": ok, "must_pass": k in must, "tj_limit_C": lim, "t_board_limit_C": tb_lim,
                      "am32_cutoff_trips": r["t_board_C"] >= AM32_CUTOFF}

    def burst_seconds(base, i_b, h):
        lo, hi = 0.0, 120.0
        for _ in range(40):
            mid = (lo + hi) / 2
            rr = burst(g, net, base, i_b, mid, V4S, FS24, h, "10V", 16.8)
            ok = (max(rr["tj_hs_C"], rr["tj_ls_C"]) <= LIMITS["tj_burst"]
                  and rr["t_board_C"] <= AM32_CUTOFF - ASSUME["am32_margin_K"])
            lo, hi = (mid, hi) if ok else (lo, mid)
        return lo
    imax = {"conservative_h50_4S_24kHz_A": max_current(g, net, V4S, FS24, hf, "10V", 16.8),
            "conservative_h50_2S_24kHz_A": max_current(g, net, V2S, FS24, hf, "4.5V", 7.6),
            "typical_h80_4S_24kHz_A": max_current(g, net, V4S, FS24, H_TYP, "10V", 16.8),
            "typical_h80_4S_48kHz_A": max_current(g, net, V4S, FS48, H_TYP, "10V", 16.8),
            "static_bench_h12_4S_24kHz_A": max_current(g, net, V4S, FS24, hb, "10V", 16.8),
            "allowed_35A_burst_s_from_typical_cont": burst_seconds(sc["R4_4S_cont_typical"], 35, H_TYP),
            "allowed_35A_burst_s_from_conservative_cont": burst_seconds(sc["R1_4S_cont_conservative"], 35, hf)}
    fails = [k for k, v in verdict.items() if v["must_pass"] and not v["pass"]]
    # ablation: what the 16-via POFV array buys the HS junction (35 A burst point)
    b = sc["I3_legacy_35A_10s_burst_typical"]
    rjc = FET["rth_jc"] + net["r_solder"]
    ablation = {"p_hs_W": b["p_fet_hs_W"],
                "hs_rise_above_site_with_vias_K": b["p_fet_hs_W"] * (rjc + net["r_hs_local"]),
                "hs_rise_above_site_prepreg_only_K": b["p_fet_hs_W"] * (rjc + net["r_prepreg_hs"]),
                "ls_rise_above_site_K": b["p_fet_ls_W"] * (rjc + net["r_prepreg_ls"])}
    ablation["via_array_benefit_K"] = (ablation["hs_rise_above_site_prepreg_only_K"]
                                       - ablation["hs_rise_above_site_with_vias_K"])

    report = {
        "revision": "9.0",
        "board_geometry_readback": {k: g[k] for k in ("board_w", "board_h", "copper", "dielectrics",
                                                     "ls_row_vias", "mount_holes")},
        "hs_thermal_arrays": {"count": len(g["hs_via_arrays"]), "vias_each": [len(v) for v in g["hs_via_arrays"]],
                              "drill_mm": sorted({d for v in g["hs_via_arrays"] for d in v})},
        "ls_source_arrays": {"count": len(g["ls_via_arrays"]), "vias_each": [len(v) for v in g["ls_via_arrays"]]},
        "thermal_network": net,
        "assumptions": ASSUME, "limits_C": LIMITS, "am32_temperature_cutoff_C": AM32_CUTOFF,
        "published_rating": RATING,
        "scenarios": sc, "verdict": verdict, "max_continuous_current_per_motor": imax,
        "via_array_ablation_35A": ablation,
        "must_pass_failures": fails,
        "status": "PASS" if not fails else "FAIL",
    }
    with open(a.out, "w") as f:
        json.dump(report, f, indent=2, default=float)
        f.write("\n")

    print(f"[THERMAL] board {g['board_w']}x{g['board_h']} mm, kt={net['kt_w_per_k']:.3f} W/K, "
          f"HS vias {net['n_hs_vias']}x -> R_vias {net['r_vias_hs']:.2f} K/W, HS local {net['r_hs_local']:.2f}, "
          f"LS prepreg {net['r_prepreg_ls']:.2f}, spread {net['r_spread_site']:.2f} K/W")
    for k, r in sc.items():
        v = verdict[k]
        print(f"  {k:32s} P_board {r['p_board_total_W']:6.2f} W  T_board {r['t_board_C']:6.1f}  "
              f"Tj HS {r['tj_hs_C']:6.1f}  LS {r['tj_ls_C']:6.1f}  "
              f"[{'PASS' if v['pass'] else 'FAIL'}{' (must)' if v['must_pass'] else ''}]"
              f"{'  AM32 cutoff trips' if v['am32_cutoff_trips'] else ''}")
    print("  35 A burst HS rise above site: with vias %.1f K, prepreg only %.1f K; LS %.1f K" % (
        ablation["hs_rise_above_site_with_vias_K"], ablation["hs_rise_above_site_prepreg_only_K"],
        ablation["ls_rise_above_site_K"]))
    print("  max continuous per motor:", {k: round(v, 1) for k, v in imax.items()})
    print("[THERMAL]", report["status"])

    if not a.no_plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        cur = [1 + k * 0.5 for k in range(69)]
        fig, ax = plt.subplots(figsize=(10, 5.5))
        for (fsw, h, lab, st) in [(FS24, hf, "conservative 50 W/m2K, 24 kHz", "g-"),
                                  (FS24, H_TYP, "typical 80 W/m2K, 24 kHz", "c-"),
                                  (FS48, H_TYP, "typical 80 W/m2K, 48 kHz", "b-"),
                                  (FS24, hb, "static bench 12 W/m2K, 24 kHz", "r--")]:
            rs = [solve(g, net, c, V4S, fsw, h, "10V", 16.8) for c in cur]
            ax.plot(cur, [max(r["tj_hs_C"], r["tj_ls_C"]) for r in rs], st, lw=2, label=f"Tj max, {lab}")
            ax.plot(cur, [r["t_board_C"] for r in rs], st, lw=1, alpha=0.5, label=f"board, {lab}")
        ax.axhline(LIMITS["tj_cont"], color="k", ls=":", label="Tj continuous design limit 125 C")
        ax.axhline(AM32_CUTOFF, color="m", ls=":", label="AM32 limits.temperature 115 C")
        ax.axvline(RATING["cont_conservative_A"], color="g", ls="-.", lw=1, label="rated cont. (conservative)")
        ax.axvline(RATING["cont_typical_A"], color="c", ls="-.", lw=1, label="rated cont. (typical)")
        ax.set_xlabel("Motor current per ESC, all 4 loaded (A), 4S, D=0.9, 12 AWG / 20 AWG leads")
        ax.set_ylabel("Temperature (C), Tamb 35 C")
        ax.set_ylim(30, 200)
        ax.set_title("Rev 8.0 power stage: 6L 2oz-inner, 16x POFV HS arrays (geometry read from KiCad board)")
        ax.grid(True, ls="--", alpha=0.5)
        ax.legend(fontsize=7, loc="upper left")
        fig.tight_layout()
        fig.savefig(os.path.join(HERE, "thermal_dissipation_curve.png"), dpi=120)
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
