#!/usr/bin/env python3
"""
Nano H743 AIO - 4-in-1 ESC power-stage loss & thermal analysis (Revision 8.0)
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
PCB = os.path.join(REPO, "kicad", "nano-h743-aio.kicad_pcb")

# ---------------------------------------------------------------- device data
# Toshiba TPHR8504PL datasheet Rev.5.0.A (2026-04-14)
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
    "rds_tempco_per_K": 0.0056,   # normalised RDS(on) 1.70 at 150 C (U-MOS IX-H typical curve shape)
    "v_plateau": 3.0,             # Miller plateau at 10-35 A (VGS(th) max 2.4 V + I/gfs)
    "r_drv_source": 6.0,          # FD6288Q output resistance, source (ohm)  - confirm
    "r_drv_sink": 4.0,            # FD6288Q output resistance, sink (ohm)    - confirm
    "rg_ext": 4.7,                # external gate resistor (schematic TBD)   - design requirement
    "dead_time_s": 60e-9,         # am32_esc/am32_target.h DEAD_TIME_NS
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
}
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
    rg = ASSUME["rg_ext"] + FET["rg_int_max"]
    i_on = (v_drive - ASSUME["v_plateau"]) / (rg + ASSUME["r_drv_source"])
    i_off = ASSUME["v_plateau"] / (rg + ASSUME["r_drv_sink"])
    return FET["qsw"] / i_on, FET["qsw"] / i_off


def fet_losses(i, vbat, fsw, duty, tj_hs, tj_ls, vgs_class, v_drive):
    t_on, t_off = switching_times(v_drive)
    hs = {"cond": i * i * rds(tj_hs, vgs_class) * duty / 3.0,
          "sw": 0.5 * vbat * i * (t_on + t_off) * fsw / 3.0,
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


def r_board_air(net, h):
    # square plate both faces; fin efficiency of the copper-loaded laminate
    m = math.sqrt(2 * h / net["kt_w_per_k"])
    half = math.sqrt(net["a_board_m2"]) / 2
    eta = math.tanh(m * half) / (m * half)
    return 1.0 / (h * 2 * net["a_board_m2"] * eta), eta


def heat_capacity(g):
    cu = g["copper"]
    a_cm2 = g["board_w"] * g["board_h"] / 100.0
    cu_cm = (sum(cu[n] for n in ("In1.Cu", "In2.Cu", "In3.Cu", "In4.Cu")) + 0.5 * (cu["F.Cu"] + cu["B.Cu"])) / 10
    fr4_cm = sum(g["dielectrics"]) / 10
    return (a_cm2 * cu_cm * ASSUME["cvol_cu"] + a_cm2 * fr4_cm * ASSUME["cvol_fr4"]
            + 24 * FET["mass_g"] * 0.6)


def solve(g, net, i, vbat, fsw, h, vgs_class, v_drive, duty=None, t_board_override=None):
    duty = ASSUME["duty_at_rated"] if duty is None else duty
    rba, eta = r_board_air(net, h)
    tj_hs = tj_ls = 80.0
    for _ in range(60):
        hs, ls, tsw = fet_losses(i, vbat, fsw, duty, tj_hs, tj_ls, vgs_class, v_drive)
        p_hs, p_ls = sum(hs.values()), sum(ls.values())
        p_fets = 12 * (p_hs + p_ls)                       # 4 ESC x 3 phases
        i_bat = 4 * i * duty
        rho = ASSUME["rho_cu_20"] * (1 + ASSUME["rho_tempco"] * (80 - 20))
        t_vb = (g["copper"]["In2.Cu"] + g["copper"]["In3.Cu"]) * 1e-3
        t_gn = (g["copper"]["In1.Cu"] + g["copper"]["In4.Cu"]) * 1e-3
        p_planes = i_bat ** 2 * rho * ASSUME["plane_squares"] * (1 / t_vb + 1 / t_gn)
        p_shunt = i_bat ** 2 * ASSUME["shunt_ohm"]
        p_total = p_fets + p_planes + p_shunt + ASSUME["p_misc_w"]
        t_board = ASSUME["t_amb"] + p_total * rba if t_board_override is None else t_board_override
        # each half-bridge site carries top + bottom half-bridges (2 HS + 2 LS)
        t_site = t_board + 2 * (p_hs + p_ls) * net["r_spread_site"]
        r_jc = FET["rth_jc"] + net["r_solder"]
        n_hs = tj_hs
        tj_hs = t_site + p_hs * (r_jc + net["r_hs_local"]) + p_hs * net["r_vias_hs"] * 0.0
        tj_ls = t_site + p_ls * (r_jc + net["r_prepreg_ls"])
        if abs(tj_hs - n_hs) < 1e-4:
            break
    return {"I_motor_A": i, "vbat_V": vbat, "fsw_kHz": fsw / 1e3, "h_W_m2K": h, "duty": duty,
            "vgs_class": vgs_class, "t_on_ns": tsw[0] * 1e9, "t_off_ns": tsw[1] * 1e9,
            "p_hs_W": {k: round(v, 4) for k, v in hs.items()}, "p_ls_W": {k: round(v, 4) for k, v in ls.items()},
            "p_fet_hs_W": p_hs, "p_fet_ls_W": p_ls, "p_fets_total_W": p_fets, "p_planes_W": p_planes,
            "p_shunt_W": p_shunt, "p_misc_W": ASSUME["p_misc_w"], "p_board_total_W": p_total,
            "r_board_air_K_W": rba, "fin_eff": eta, "t_board_C": t_board, "t_site_C": t_site,
            "tj_hs_C": tj_hs, "tj_ls_C": tj_ls, "rds_hs_mohm": rds(tj_hs, vgs_class) * 1e3}


def burst(g, net, base, i_burst, seconds, vbat, fsw, h, vgs_class, v_drive):
    c = heat_capacity(g)
    ss = solve(g, net, i_burst, vbat, fsw, h, vgs_class, v_drive)
    tau = ss["r_board_air_K_W"] * c
    t_b = ss["t_board_C"] + (base["t_board_C"] - ss["t_board_C"]) * math.exp(-seconds / tau)
    r = solve(g, net, i_burst, vbat, fsw, h, vgs_class, v_drive, t_board_override=t_b)
    r.update({"burst_s": seconds, "c_board_J_K": c, "tau_s": tau, "t_board_ss_if_sustained_C": ss["t_board_C"]})
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

    g = read_board(a.board)
    net = network(g)
    V4S, V2S = 16.8, 8.4
    FS48, FS24 = 48e3, 24e3
    hf, hb = ASSUME["h_flight"], ASSUME["h_bench"]

    sc = {}
    sc["S1_4S_20A_flight_48kHz"] = solve(g, net, 20, V4S, FS48, hf, "10V", 16.8)
    sc["S2_4S_20A_flight_24kHz"] = solve(g, net, 20, V4S, FS24, hf, "10V", 16.8)
    sc["S3_2S_20A_flight_24kHz"] = solve(g, net, 20, V2S, FS24, hf, "4.5V", 7.6)
    sc["S4_4S_35A_10s_burst_24kHz"] = burst(g, net, sc["S2_4S_20A_flight_24kHz"], 35, 10, V4S, FS24, hf, "10V", 16.8)
    sc["S5_4S_35A_10s_burst_48kHz"] = burst(g, net, sc["S1_4S_20A_flight_48kHz"], 35, 10, V4S, FS48, hf, "10V", 16.8)
    sc["S6_4S_20A_static_bench_24kHz"] = solve(g, net, 20, V4S, FS24, hb, "10V", 16.8)

    must = {"S2_4S_20A_flight_24kHz": "cont", "S3_2S_20A_flight_24kHz": "cont", "S4_4S_35A_10s_burst_24kHz": "burst"}
    verdict = {}
    for k, r in sc.items():
        lim = LIMITS["tj_burst"] if "burst" in k else LIMITS["tj_cont"]
        ok = max(r["tj_hs_C"], r["tj_ls_C"]) <= lim and (("burst" in k) or r["t_board_C"] <= LIMITS["t_board_cont"])
        verdict[k] = {"pass": ok, "must_pass": k in must, "tj_limit_C": lim,
                      "am32_cutoff_trips": r["t_board_C"] >= AM32_CUTOFF}
    imax = {"flight_4S_24kHz_A": max_current(g, net, V4S, FS24, hf, "10V", 16.8),
            "flight_4S_48kHz_A": max_current(g, net, V4S, FS48, hf, "10V", 16.8),
            "bench_4S_24kHz_A": max_current(g, net, V4S, FS24, hb, "10V", 16.8)}
    fails = [k for k, v in verdict.items() if v["must_pass"] and not v["pass"]]

    report = {
        "revision": "8.0",
        "board_geometry_readback": {k: g[k] for k in ("board_w", "board_h", "copper", "dielectrics",
                                                     "ls_row_vias", "mount_holes")},
        "hs_thermal_arrays": {"count": len(g["hs_via_arrays"]), "vias_each": [len(v) for v in g["hs_via_arrays"]],
                              "drill_mm": sorted({d for v in g["hs_via_arrays"] for d in v})},
        "ls_source_arrays": {"count": len(g["ls_via_arrays"]), "vias_each": [len(v) for v in g["ls_via_arrays"]]},
        "thermal_network": net,
        "assumptions": ASSUME, "limits_C": LIMITS, "am32_temperature_cutoff_C": AM32_CUTOFF,
        "scenarios": sc, "verdict": verdict, "max_continuous_current_per_motor": imax,
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
    print("  max continuous per motor:", {k: round(v, 1) for k, v in imax.items()})
    print("[THERMAL]", report["status"])

    if not a.no_plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        cur = [1 + k * 0.5 for k in range(69)]
        fig, ax = plt.subplots(figsize=(10, 5.5))
        for (fsw, h, lab, st) in [(FS24, hf, "flight 50 W/m2K, 24 kHz", "g-"), (FS48, hf, "flight 50 W/m2K, 48 kHz", "b-"),
                                  (FS24, hb, "static bench 12 W/m2K, 24 kHz", "r--")]:
            rs = [solve(g, net, c, V4S, fsw, h, "10V", 16.8) for c in cur]
            ax.plot(cur, [max(r["tj_hs_C"], r["tj_ls_C"]) for r in rs], st, lw=2, label=f"Tj max, {lab}")
            ax.plot(cur, [r["t_board_C"] for r in rs], st, lw=1, alpha=0.5, label=f"board, {lab}")
        ax.axhline(LIMITS["tj_cont"], color="k", ls=":", label="Tj continuous design limit 125 C")
        ax.axhline(AM32_CUTOFF, color="m", ls=":", label="AM32 TEMPERATURE_CUTOFF 115 C")
        ax.axvline(20, color="0.5", ls="-.")
        ax.set_xlabel("Motor current per ESC, all 4 loaded (A), 4S, D=0.9")
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
