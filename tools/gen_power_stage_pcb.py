#!/usr/bin/env python3
"""
Nano H743 AIO - 6-layer power-stage PCB generator (Revision 8.0)

Writes, from the constants in this file:
  kicad/nano-h743-aio.pretty/*.kicad_mod   project footprint library
  kicad/fp-lib-table                       registers it as "nano-h743-aio"
  kicad/nano-h743-aio.kicad_dru            custom DRC rules (2 oz inner, POFV)
  kicad/nano-h743-aio.kicad_pro            board design rules / net classes
  kicad/nano-h743-aio.kicad_pcb            stackup, outline, 24x TPHR8504PL,
                                           thermal-via arrays, planes, filled

Stackup  : JLCPCB JLC061612-1080, 1.614 mm, 1 oz outer / 2 oz inner
           (fab-modelled finished inner copper 61 um), ENIG, POFV via-in-pad.
           L1 F.Cu sig/FET | In1 GND | In2 VBAT | In3 VBAT | In4 GND | L6 B.Cu
Power    : 4 ESCs x 3 half-bridges.  Every half-bridge is a "pair position"
           around the board edge; the TOP FETs of ESC1/ESC2 and the BOTTOM
           FETs of ESC3/ESC4 share each pair position (same XY, mirrored):
             H slot: top HS + bottom HS, drains = VBAT -> one shared 4x4 array
                     of 0.30/0.50 mm POFV thermal vias solid into In2/In3 VBAT
                     and into BOTH drain tabs.
             L slot: top LS + bottom LS, sources = GND -> 2x2 POFV vias in the
                     two source pads that coincide on both sides (pins 2/3 top
                     == pins 3/2 bottom), solid into In1/In4 GND.
           LS drain tabs (phase nodes of different ESCs on the two sides)
           cannot share a through via, so they have no drain vias by design;
           their heat path is the L1/L6 phase pour + motor pad and the
           0.1375 mm 1080 prepreg into the In1/In4 GND plane (see
           analysis/thermal_power_analysis.py).

Usage:  python3 tools/gen_power_stage_pcb.py [--outdir DIR] [--no-fill]
Requires KiCad 8 pcbnew Python module.
"""
import argparse
import json
import math
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
KDIR = os.path.join(REPO, "kicad")
PROJECT = "nano-h743-aio"
FPLIB = PROJECT
REV = "8.0"

# ----------------------------------------------------------------------------
# Board / stackup constants
# ----------------------------------------------------------------------------
BOARD = 36.0                    # mm, square outline (30.5 x 30.5 M3 pattern)
MOUNT_PITCH = 30.5
MOUNT_DRILL = 3.2               # M3 clearance, NPTH
MOUNT_KEEPOUT_R = 3.2           # grommet flange r (6.4 mm OD soft mount)
EDGE_CLEAR = 0.3                # JLC copper-to-edge (routed)

# (name, kind, thickness_mm, material, epsilon_r)  -- JLC061612-1080
STACKUP = [
    ("F.Cu", "copper", 0.035, None, None),
    ("dielectric 1", "prepreg", 0.1375, "FR4 1080x2 (NP-155F)", 3.91),
    ("In1.Cu", "copper", 0.061, None, None),
    ("dielectric 2", "core", 0.35, "FR4 core (NP-155F)", 4.6),
    ("In2.Cu", "copper", 0.061, None, None),
    ("dielectric 3", "prepreg", 0.325, "FR4 1080+7628+1080 (NP-155F)", 4.24),
    ("In3.Cu", "copper", 0.061, None, None),
    ("dielectric 4", "core", 0.35, "FR4 core (NP-155F)", 4.6),
    ("In4.Cu", "copper", 0.061, None, None),
    ("dielectric 5", "prepreg", 0.1375, "FR4 1080x2 (NP-155F)", 3.91),
    ("B.Cu", "copper", 0.035, None, None),
]
BOARD_THICKNESS = round(sum(t for _, _, t, _, _ in STACKUP), 4)   # 1.614
PLANES = {"In1.Cu": "GND", "In2.Cu": "VBAT", "In3.Cu": "VBAT", "In4.Cu": "GND"}

# ----------------------------------------------------------------------------
# TPHR8504PL, Toshiba SOP Advance (2-5Q1S) recommended land pattern
# Local frame: gate/source row at +y, drain tab at -y, pin 1 at -x.
# ----------------------------------------------------------------------------
PITCH = 1.27
GS_Y = 2.75                     # G/S pad centre (pads span 2.025 .. 3.475)
GS_W, GS_L = 0.85, 1.45
LAND_HALF_W = 2.35              # land 4.7 wide
LAND_HALF_L = 3.475             # land 6.95 long
DRAIN_BODY_Y0, DRAIN_BODY_Y1 = -2.425, 1.325   # solid drain body
FINGER_Y0 = -3.475                              # finger tips (pins 5..8)
PIN_X = [-1.5 * PITCH, -0.5 * PITCH, 0.5 * PITCH, 1.5 * PITCH]  # pin1..4
DRAIN_FINGER_X = {"8": PIN_X[0], "7": PIN_X[1], "6": PIN_X[2], "5": PIN_X[3]}
VIA_DRILL, VIA_PAD = 0.30, 0.50
HS_VIA_X = PIN_X
HS_VIA_Y = [-2.0, -1.0, 0.0, 1.0]               # 4 x 4 = 16 vias in drain body
LS_VIA = [("2", PIN_X[1], GS_Y - 0.35), ("2", PIN_X[1], GS_Y + 0.35),
          ("3", PIN_X[2], GS_Y - 0.35), ("3", PIN_X[2], GS_Y + 0.35)]
CRT_MARGIN = 0.25
LS_ROW_U = [PIN_X[1], 0.0, PIN_X[2]]           # 3 GND return vias inboard of LS sources
LS_ROW_GAP = 0.55                               # land edge -> via centre

FP_PLAIN = "Toshiba_SOP-Advance_TPHR8504PL"
FP_HS = "Toshiba_SOP-Advance_TPHR8504PL_ThermalVias_4x4"
FP_LS = "Toshiba_SOP-Advance_TPHR8504PL_SourceVias_2x2"
FP_MOUNT = "MountingHole_3.2mm_M3_Grommet_Keepout"
FP_MOTOR = "MotorPad_4.0x1.3mm"

# ----------------------------------------------------------------------------
# Placement (canonical frame = north edge: u along edge, v depth from edge)
# ----------------------------------------------------------------------------
G1 = 0.7                        # H-L gap: top phase bridge / bottom HS gate channel
G2 = 0.7                        # left of H: top HS gate escape channel
LAND_EDGE_V = 2.2               # outer land edge depth (motor pad strip outside)
V_C = LAND_EDGE_V + LAND_HALF_L  # FET centre depth 5.675
PAIR_PITCH = G2 + 2 * (2 * LAND_HALF_W) + G1    # 10.8
MOTOR_PAD = (4.0, 1.3)          # along edge x depth
MOTOR_V = 1.15                  # pad centre depth (0.5 .. 1.8)
EDGES = {"N": 0, "E": 1, "S": 2, "W": 3}
# pair positions: (edge, index, n_on_edge) ; ESC phases assigned in order
PAIR_POS = [("N", 0, 2), ("N", 1, 2), ("W", 0, 1),     # ESC1 (top) / ESC3 (bottom)
            ("S", 0, 2), ("S", 1, 2), ("E", 0, 1)]     # ESC2 (top) / ESC4 (bottom)
CENTRE_KEEP = None  # computed: MCU reserve


def fmt(v):
    s = ("%.4f" % v).rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


# ----------------------------------------------------------------------------
# Footprint library (text, deterministic)
# ----------------------------------------------------------------------------
def _fp_header(name, descr, tags, attr="smd"):
    return [f'(footprint "{name}"', '\t(version 20240108)', '\t(generator "nano_h743_gen_power_stage")',
            '\t(generator_version "8.0")', '\t(layer "F.Cu")', f'\t(descr "{descr}")',
            f'\t(tags "{tags}")']


def _prop(name, val, x, y, layer, hide=False, size=0.8):
    h = " (hide yes)" if hide else ""
    return (f'\t(property "{name}" "{val}" (at {fmt(x)} {fmt(y)} 0) (layer "{layer}"){h}\n'
            f'\t\t(effects (font (size {size} {size}) (thickness {fmt(size * 0.15)}))))')


def _line(x1, y1, x2, y2, layer, w):
    return (f'\t(fp_line (start {fmt(x1)} {fmt(y1)}) (end {fmt(x2)} {fmt(y2)}) '
            f'(stroke (width {fmt(w)}) (type solid)) (layer "{layer}"))')


def _rect(x1, y1, x2, y2, layer, w):
    return [_line(x1, y1, x2, y1, layer, w), _line(x2, y1, x2, y2, layer, w),
            _line(x2, y2, x1, y2, layer, w), _line(x1, y2, x1, y1, layer, w)]


def _smd(num, x, y, w, h, layers="\"F.Cu\" \"F.Paste\" \"F.Mask\"", extra=""):
    return f'\t(pad "{num}" smd rect (at {fmt(x)} {fmt(y)}) (size {fmt(w)} {fmt(h)}) (layers {layers}){extra})'


def _via_pad(num, x, y):
    return (f'\t(pad "{num}" thru_hole circle (at {fmt(x)} {fmt(y)}) (size {fmt(VIA_PAD)} {fmt(VIA_PAD)}) '
            f'(drill {fmt(VIA_DRILL)}) (layers "*.Cu") (remove_unused_layers no) (zone_connect 2))')


def fp_mosfet(variant):
    name = {"plain": FP_PLAIN, "hs": FP_HS, "ls": FP_LS}[variant]
    descr = ("Toshiba TPHR8504PL 40V N-MOSFET, SOP Advance (2-5Q1S) 5.0x6.0 mm, Toshiba recommended "
             "land pattern (JEITA ET-7501 L3). Pins 1-3 S, 4 G, 5-8 + tab D. Drain paste 2x2 windowpane 69%.")
    if variant == "hs":
        descr += (" 16x POFV thermal vias 0.30/0.50 mm in drain tab (IPC-4761 Type VII, filled+capped), "
                  "solid zone connection.")
    if variant == "ls":
        descr += " 4x POFV vias 0.30/0.50 mm in source pads 2/3 (IPC-4761 Type VII), solid zone connection."
    out = _fp_header(name, descr, "MOSFET SOP-Advance TPHR8504PL power thermal-via")
    out += [_prop("Reference", "REF**", 0, -4.3, "F.Fab"),
            _prop("Value", "TPHR8504PL", 0, 4.4, "F.Fab"),
            _prop("Footprint", f"{FPLIB}:{name}", 0, 0, "F.Fab", hide=True),
            _prop("Datasheet", "https://toshiba.semicon-storage.com/info/TPHR8504PL_datasheet_en_20191024.pdf?did=15221",
                  0, 0, "F.Fab", hide=True),
            _prop("Description", "N-MOSFET 40V 150A 0.85mOhm@10V SOP Advance", 0, 0, "F.Fab", hide=True),
            '\t(attr smd)', '\t(zone_connect 2)']
    # fab: package body 5.0 x 5.0 (G/S leads protrude 0.6 beyond body)
    out += _rect(-2.5, -2.6, 2.5, 2.4, "F.Fab", 0.1)
    out.append(_line(-2.5, 1.9, -2.0, 2.4, "F.Fab", 0.1))
    # silk: pin-1 tick outside the land, and the two side lines
    out.append(_line(-2.62, 2.0, -2.62, 3.6, "F.SilkS", 0.12))
    out.append(_line(-2.62, 3.6, -2.1, 3.6, "F.SilkS", 0.12))
    # courtyard
    cx, cy = LAND_HALF_W + CRT_MARGIN, LAND_HALF_L + CRT_MARGIN
    out += _rect(-cx, -cy, cx, cy, "F.CrtYd", 0.05)
    # gate / source pads
    for i, x in enumerate(PIN_X):
        out.append(_smd(str(i + 1), x, GS_Y, GS_W, GS_L))
    # drain body (paste supplied by windowpane apertures)
    body_h = DRAIN_BODY_Y1 - DRAIN_BODY_Y0
    out.append(_smd("5", 0, (DRAIN_BODY_Y0 + DRAIN_BODY_Y1) / 2, 2 * LAND_HALF_W, body_h,
                    layers="\"F.Cu\" \"F.Mask\""))
    # drain fingers (pins 5..8), overlapping the body by 0.05
    fl = (DRAIN_BODY_Y0 + 0.05) - FINGER_Y0
    for num in ("5", "6", "7", "8"):
        out.append(_smd(num, DRAIN_FINGER_X[num], FINGER_Y0 + fl / 2, GS_W, fl))
    # windowpane paste apertures on the drain body: 2 x 2, 69 % coverage
    pw, ph = 1.95, 1.55
    for x in (-1.125, 1.125):
        for y in (-1.53, 0.43):
            out.append(_smd("", x, y, pw, ph, layers="\"F.Paste\""))
    if variant == "hs":
        for y in HS_VIA_Y:
            for x in HS_VIA_X:
                out.append(_via_pad("5", x, y))
    if variant == "ls":
        for num, x, y in LS_VIA:
            out.append(_via_pad(num, x, y))
    out.append(')')
    return name, "\n".join(out) + "\n"


def fp_mount():
    out = _fp_header(FP_MOUNT, "M3 NPTH 3.2 mm, 30.5x30.5 FC pattern, 6.4 mm grommet keep-out on all copper",
                     "mounting hole M3 grommet", attr="")
    out += [_prop("Reference", "REF**", 0, -4.0, "F.Fab", hide=True),
            _prop("Value", "M3_Grommet", 0, 4.0, "F.Fab", hide=True),
            _prop("Footprint", f"{FPLIB}:{FP_MOUNT}", 0, 0, "F.Fab", hide=True),
            _prop("Datasheet", "", 0, 0, "F.Fab", hide=True),
            _prop("Description", "M3 mounting hole with grommet keep-out", 0, 0, "F.Fab", hide=True),
            '\t(attr exclude_from_pos_files exclude_from_bom)']
    r = MOUNT_KEEPOUT_R
    for lay in ("F.CrtYd", "B.CrtYd"):
        out.append(f'\t(fp_circle (center 0 0) (end {fmt(r + 0.05)} 0) (stroke (width 0.05) (type solid)) '
                   f'(fill none) (layer "{lay}"))')
    out.append(f'\t(fp_circle (center 0 0) (end {fmt(r)} 0) (stroke (width 0.1) (type solid)) (fill none) '
               f'(layer "Cmts.User"))')
    out.append(f'\t(pad "" np_thru_hole circle (at 0 0) (size {fmt(MOUNT_DRILL)} {fmt(MOUNT_DRILL)}) '
               f'(drill {fmt(MOUNT_DRILL)}) (layers "*.Cu" "*.Mask"))')
    pts = " ".join(f"(xy {fmt(r * math.cos(2 * math.pi * k / 32))} {fmt(r * math.sin(2 * math.pi * k / 32))})"
                   for k in range(32))
    out.append('\t(zone (net 0) (net_name "") (layers "*.Cu") (hatch edge 0.5) (connect_pads (clearance 0)) '
               '(min_thickness 0.25) (filled_areas_thickness no) '
               '(keepout (tracks not_allowed) (vias not_allowed) (pads allowed) (copperpour not_allowed) '
               '(footprints allowed)) (fill (thermal_gap 0.5) (thermal_bridge_width 0.5)) '
               f'(polygon (pts {pts})))')
    out.append(')')
    return FP_MOUNT, "\n".join(out) + "\n"


def fp_motor():
    w, h = MOTOR_PAD
    out = _fp_header(FP_MOTOR, "Motor phase wire pad 4.0x1.3 mm (20-24 AWG), solid zone connection",
                     "motor pad wire")
    out += [_prop("Reference", "REF**", 0, -1.5, "F.Fab", hide=True, size=0.6),
            _prop("Value", "MOTOR", 0, 1.5, "F.Fab", hide=True, size=0.6),
            _prop("Footprint", f"{FPLIB}:{FP_MOTOR}", 0, 0, "F.Fab", hide=True),
            _prop("Datasheet", "", 0, 0, "F.Fab", hide=True),
            _prop("Description", "Motor wire solder pad", 0, 0, "F.Fab", hide=True),
            '\t(attr smd exclude_from_pos_files exclude_from_bom)', '\t(zone_connect 2)']
    out += _rect(-w / 2 - 0.05, -h / 2 - 0.05, w / 2 + 0.05, h / 2 + 0.05, "F.CrtYd", 0.05)
    out.append(_smd("1", 0, 0, w, h))
    out.append(')')
    return FP_MOTOR, "\n".join(out) + "\n"


def write_library(outdir):
    libdir = os.path.join(outdir, f"{FPLIB}.pretty")
    os.makedirs(libdir, exist_ok=True)
    for name, text in [fp_mosfet("plain"), fp_mosfet("hs"), fp_mosfet("ls"), fp_mount(), fp_motor()]:
        with open(os.path.join(libdir, name + ".kicad_mod"), "w") as f:
            f.write(text)
    with open(os.path.join(outdir, "fp-lib-table"), "w") as f:
        f.write('(fp_lib_table\n  (version 7)\n'
                f'  (lib (name "{FPLIB}")(type "KiCad")(uri "${{KIPRJMOD}}/{FPLIB}.pretty")(options "")'
                '(descr "Nano H743 AIO project footprints (generated by tools/gen_power_stage_pcb.py)"))\n)\n')
    return libdir


# ----------------------------------------------------------------------------
# Design rules
# ----------------------------------------------------------------------------
DRU = """(version 1)
# Nano H743 AIO custom DRC rules - generated by tools/gen_power_stage_pcb.py (Rev 8.0)
# Fab: JLCPCB 6-layer JLC061612-1080, 1 oz outer / 2 oz inner, POFV (Type VII) via-in-pad.

(rule "2oz inner copper clearance"
  (layer inner)
  (constraint clearance (min 0.2)))

(rule "2oz inner minimum width"
  (layer inner)
  (constraint track_width (min 0.16)))

(rule "Inner plane hole to copper (JLC 0.2 mm)"
  (layer inner)
  (constraint hole_clearance (min 0.2)))

(rule "MOSFET and motor pads connect solid to copper (no thermal spokes)"
  (condition "A.Type == 'Pad' && (A.memberOfFootprint('Q*') || A.memberOfFootprint('MP*'))")
  (constraint zone_connection solid))

(rule "POFV thermal via drill"
  (condition "A.Type == 'Pad' && A.Pad_Type == 'Through-hole' && A.memberOfFootprint('Q*')")
  (constraint hole_size (min 0.3) (max 0.3)))

(rule "Mount hole grommet clearance"
  (condition "A.Type == 'Pad' && A.Pad_Type == 'NPTH, mechanical'")
  (constraint hole_clearance (min 0.5)))
"""


def write_project(outdir):
    pro = {
        "board": {
            "3dviewports": [],
            "design_settings": {
                "defaults": {"board_outline_line_width": 0.1, "copper_line_width": 0.2,
                             "copper_text_size_h": 1.0, "copper_text_size_v": 1.0,
                             "copper_text_thickness": 0.15, "other_line_width": 0.1,
                             "silk_line_width": 0.12, "silk_text_size_h": 0.8,
                             "silk_text_size_v": 0.8, "silk_text_thickness": 0.12},
                "rules": {
                    "min_clearance": 0.127, "min_track_width": 0.127,
                    "min_connection": 0.15, "min_via_annular_width": 0.075,
                    "min_via_diameter": 0.35, "min_through_hole_diameter": 0.2,
                    "min_hole_clearance": 0.2, "min_hole_to_hole": 0.25,
                    "min_copper_edge_clearance": EDGE_CLEAR, "min_silk_clearance": 0.0,
                    "min_microvia_diameter": 0.2, "min_microvia_drill": 0.1,
                    "solder_mask_to_copper_clearance": 0.0, "min_resolved_spokes": 2,
                    "use_height_for_length_calcs": True, "allow_blind_buried_vias": False,
                    "allow_microvias": False},
                "rule_severities": {"lib_footprint_issues": "warning",
                                    "lib_footprint_mismatch": "warning",
                                    "silk_overlap": "warning", "silk_over_copper": "warning"},
                "track_widths": [0.0, 0.15, 0.2, 0.3, 0.5, 1.0],
                "via_dimensions": [{"diameter": 0.0, "drill": 0.0}, {"diameter": 0.35, "drill": 0.2},
                                   {"diameter": 0.5, "drill": 0.3}],
            },
            "layer_presets": [], "viewports": []},
        "meta": {"filename": f"{PROJECT}.kicad_pro", "version": 1},
        "net_settings": {
            "classes": [
                {"name": "Default", "clearance": 0.127, "track_width": 0.15, "via_diameter": 0.35,
                 "via_drill": 0.2, "microvia_diameter": 0.3, "microvia_drill": 0.1,
                 "diff_pair_gap": 0.15, "diff_pair_via_gap": 0.25, "diff_pair_width": 0.15,
                 "line_style": 0, "pcb_color": "rgba(0, 0, 0, 0.000)",
                 "schematic_color": "rgba(0, 0, 0, 0.000)", "wire_width": 6, "bus_width": 12,
                 "priority": 2147483647},
                {"name": "Power", "clearance": 0.2, "track_width": 1.0, "via_diameter": 0.5,
                 "via_drill": 0.3, "microvia_diameter": 0.3, "microvia_drill": 0.1,
                 "diff_pair_gap": 0.25, "diff_pair_via_gap": 0.25, "diff_pair_width": 0.2,
                 "line_style": 0, "pcb_color": "rgba(0, 0, 0, 0.000)",
                 "schematic_color": "rgba(0, 0, 0, 0.000)", "wire_width": 6, "bus_width": 12,
                 "priority": 0},
                {"name": "Gate", "clearance": 0.2, "track_width": 0.2, "via_diameter": 0.35,
                 "via_drill": 0.2, "microvia_diameter": 0.3, "microvia_drill": 0.1,
                 "diff_pair_gap": 0.25, "diff_pair_via_gap": 0.25, "diff_pair_width": 0.2,
                 "line_style": 0, "pcb_color": "rgba(0, 0, 0, 0.000)",
                 "schematic_color": "rgba(0, 0, 0, 0.000)", "wire_width": 6, "bus_width": 12,
                 "priority": 1}],
            "meta": {"version": 3},
            "netclass_assignments": None,
            "netclass_patterns": [{"netclass": "Power", "pattern": "VBAT"},
                                  {"netclass": "Power", "pattern": "GND"},
                                  {"netclass": "Power", "pattern": "ESC*_PH_*"},
                                  {"netclass": "Gate", "pattern": "ESC*_G?_*"}]},
        "pcbnew": {"last_paths": {"gencad": "", "idf": "", "netlist": "", "plot": "gerbers/",
                                  "pos_files": "", "specctra_dsn": "", "step": "", "svg": "", "vrml": ""},
                   "page_layout_descr_file": ""},
        "schematic": {"legacy_lib_dir": "", "legacy_lib_list": []},
        "sheets": [["", ""]],
        "text_variables": {},
    }
    with open(os.path.join(outdir, f"{PROJECT}.kicad_pro"), "w") as f:
        json.dump(pro, f, indent=2)
        f.write("\n")
    with open(os.path.join(outdir, f"{PROJECT}.kicad_dru"), "w") as f:
        f.write(DRU)


# ----------------------------------------------------------------------------
# Board skeleton (text): layers, stackup, outline, notes
# ----------------------------------------------------------------------------
LAYERS = """  (layers
    (0 "F.Cu" signal)
    (1 "In1.Cu" power)
    (2 "In2.Cu" power)
    (3 "In3.Cu" power)
    (4 "In4.Cu" power)
    (31 "B.Cu" signal)
    (32 "B.Adhes" user "B.Adhesive")
    (33 "F.Adhes" user "F.Adhesive")
    (34 "B.Paste" user)
    (35 "F.Paste" user)
    (36 "B.SilkS" user "B.Silkscreen")
    (37 "F.SilkS" user "F.Silkscreen")
    (38 "B.Mask" user)
    (39 "F.Mask" user)
    (40 "Dwgs.User" user "User.Drawings")
    (41 "Cmts.User" user "User.Comments")
    (44 "Edge.Cuts" user)
    (45 "Margin" user)
    (46 "B.CrtYd" user "B.Courtyard")
    (47 "F.CrtYd" user "F.Courtyard")
    (48 "B.Fab" user)
    (49 "F.Fab" user)
  )
"""


def stackup_text():
    s = ['    (stackup',
         '      (layer "F.SilkS" (type "Top Silk Screen"))',
         '      (layer "F.Paste" (type "Top Solder Paste"))',
         '      (layer "F.Mask" (type "Top Solder Mask") (thickness 0.01))']
    for name, kind, t, mat, er in STACKUP:
        if kind == "copper":
            s.append(f'      (layer "{name}" (type "copper") (thickness {fmt(t)}))')
        else:
            s.append(f'      (layer "{name}" (type "{kind}") (thickness {fmt(t)}) (material "{mat}") '
                     f'(epsilon_r {fmt(er)}) (loss_tangent 0.02))')
    s += ['      (layer "B.Mask" (type "Bottom Solder Mask") (thickness 0.01))',
          '      (layer "B.Paste" (type "Bottom Solder Paste"))',
          '      (layer "B.SilkS" (type "Bottom Silk Screen"))',
          '      (copper_finish "ENIG")',
          '      (dielectric_constraints no)',
          '    )']
    return "\n".join(s)


def skeleton_text():
    L = BOARD
    notes = [
        "Rev 8.0 stackup JLC061612-1080 1.614 mm ENIG, POFV via-in-pad (IPC-4761 VII)",
        "L1 F.Cu 1oz sig/FET | In1 2oz GND | In2 2oz VBAT | In3 2oz VBAT | In4 2oz GND | L6 B.Cu 1oz",
        "FETs: Q1-Q12 top (ESC1/2), Q13-Q24 bottom (ESC3/4), shared XY per half-bridge",
    ]
    txt = [
        "(kicad_pcb",
        "  (version 20240108)",
        '  (generator "nano_h743_gen_power_stage")',
        '  (generator_version "8.0")',
        f"  (general (thickness {fmt(BOARD_THICKNESS)}) (legacy_teardrops no))",
        '  (paper "A4")',
        "  (title_block",
        f'    (title "STM32H743 Nano AIO - 6-layer power stage (Rev {REV})")',
        '    (date "2026-11-06")',
        f'    (rev "{REV}")',
        '    (company "Sreesai Arjun Engineering")',
        '    (comment 1 "Stackup JLC061612-1080: 1oz outer / 2oz inner, 1.614 mm")',
        '    (comment 2 "Generated by tools/gen_power_stage_pcb.py - do not hand edit")',
        "  )",
        LAYERS.rstrip("\n"),
        "  (setup",
        stackup_text(),
        "    (pad_to_mask_clearance 0.05)",
        "    (solder_mask_min_width 0.1)",
        "    (allow_soldermask_bridges_in_footprints no)",
        "  )",
        '  (net 0 "")',
        f'  (gr_rect (start 0 0) (end {fmt(L)} {fmt(L)}) (stroke (width 0.1) (type solid)) (fill none) '
        '(layer "Edge.Cuts"))',
    ]
    for i, n in enumerate(notes):
        txt.append(f'  (gr_text "{n}" (at {fmt(L + 2)} {fmt(2 + 2 * i)} 0) (layer "Cmts.User") '
                   '(effects (font (size 0.8 0.8) (thickness 0.12)) (justify left)))')
    txt.append(")")
    return "\n".join(txt) + "\n"


# ----------------------------------------------------------------------------
# Geometry helpers
# ----------------------------------------------------------------------------
def edge_xform(edge, u, v):
    """canonical (u, v) on the north edge -> board (x, y) for the given edge."""
    c = BOARD / 2
    dx, dy = u - c, v - c
    for _ in range(EDGES[edge]):
        dx, dy = -dy, dx            # 90 deg clockwise on screen (y down)
    return c + dx, c + dy


def local_to_canon(x, y, rot, bottom):
    """footprint local pad (x, y) -> canonical offset, KiCad rotation convention."""
    if bottom:
        x = -x
    t = math.radians(rot)
    return (x * math.cos(t) + y * math.sin(t), -x * math.sin(t) + y * math.cos(t))


def pair_layout():
    """List of pair dicts in canonical coordinates."""
    out = []
    for k, (edge, idx, n) in enumerate(PAIR_POS):
        total = n * PAIR_PITCH
        start = BOARD / 2 - total / 2 + idx * PAIR_PITCH
        cx_h = start + G2 + LAND_HALF_W
        cx_l = cx_h + 2 * LAND_HALF_W + G1
        out.append({"slot": k, "edge": edge, "cx_h": cx_h, "cx_l": cx_l})
    return out


def phase_polygon(cx_h, cx_l, bottom):
    vin = V_C - DRAIN_BODY_Y1 - 0.5            # 3.85, inner edge of HS G/S strip
    vls = LAND_EDGE_V + (DRAIN_BODY_Y1 - FINGER_Y0) + 0.15   # just past LS drain body
    if not bottom:
        # top HS: pins 1-3 toward L, gate at the g2 side
        return [(cx_h - 1.2, 0.4), (cx_l + 2.4, 0.4), (cx_l + 2.4, vls), (cx_l - 2.5, vls),
                (cx_l - 2.5, vin), (cx_h - 1.2, vin)]
    # bottom HS (mirrored): gate toward L -> join HS sources to LS drain through the
    # motor strip and keep g1 free for the bottom gate escape
    strip = LAND_EDGE_V - 0.25
    return [(cx_h - 2.4, 0.4), (cx_l + 2.4, 0.4), (cx_l + 2.4, vls), (cx_l - 2.4, vls),
            (cx_l - 2.4, strip), (cx_h + 1.25, strip), (cx_h + 1.25, vin), (cx_h - 2.4, vin)]


# ----------------------------------------------------------------------------
# Board build (pcbnew)
# ----------------------------------------------------------------------------
def build(outdir, fill=True):
    import pcbnew
    mm = pcbnew.FromMM
    libdir = write_library(outdir)
    write_project(outdir)
    pcb_path = os.path.join(outdir, f"{PROJECT}.kicad_pcb")
    with open(pcb_path, "w") as f:
        f.write(skeleton_text())
    board = pcbnew.LoadBoard(pcb_path)

    nets = {}

    def net(name):
        if name not in nets:
            n = pcbnew.NETINFO_ITEM(board, name)
            board.Add(n)
            nets[name] = n
        return nets[name]

    net("GND"), net("VBAT")
    placed = []
    checks = []

    def place(fpname, ref, value, edge, u, v, rot, bottom, padnets, fields=None):
        fp = pcbnew.FootprintLoad(libdir, fpname)
        fp.SetReference(ref)
        fp.SetValue(value)
        board.Add(fp)
        for k, val in (fields or {}).items():
            fp.SetField(k, val)
            fld = fp.GetFieldByName(k)
            fld.SetVisible(False)
            fld.SetLayer(pcbnew.F_Fab)
            fld.SetTextSize(pcbnew.VECTOR2I(mm(0.5), mm(0.5)))
            fld.SetTextThickness(mm(0.075))
        if bottom:
            fp.Flip(pcbnew.VECTOR2I(0, 0), True)
            orient = rot + 180 - 90 * EDGES[edge]
        else:
            orient = rot - 90 * EDGES[edge]
        fp.SetOrientationDegrees(((orient + 180) % 360) - 180)
        x, y = edge_xform(edge, u, v)
        fp.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
        for p in fp.Pads():
            num = p.GetNumber()
            if num in padnets:
                p.SetNet(net(padnets[num]))
            # closed-form check of the pad position
            lx, ly = pcbnew.ToMM(p.GetFPRelativePosition().x), pcbnew.ToMM(p.GetFPRelativePosition().y)
            if bottom:
                # pcbnew stores back-side pads already mirrored in Y: undo to library frame
                ly = -ly
            cu, cv = local_to_canon(lx, ly, rot, bottom)
            ex, ey = edge_xform(edge, u + cu, v + cv)
            ax, ay = pcbnew.ToMM(p.GetPosition().x), pcbnew.ToMM(p.GetPosition().y)
            checks.append((ref, num, abs(ax - ex) < 1e-3 and abs(ay - ey) < 1e-3, (ax, ay), (ex, ey)))
        placed.append(fp)
        return fp

    # ---- MOSFETs, motor pads, phase zones
    pairs = pair_layout()
    q = 0
    mp = 0
    fet_map = []
    for side_idx, bottom in enumerate((False, True)):
        for p in pairs:
            esc = (1 if p["slot"] < 3 else 2) + (2 if bottom else 0)
            ph = "ABC"[p["slot"] % 3]
            phn = f"ESC{esc}_PH_{ph}"
            gh, gl = f"ESC{esc}_GH_{ph}", f"ESC{esc}_GL_{ph}"
            fields = {"MPN": "TPHR8504PL,L1Q(M", "Manufacturer": "Toshiba", "LCSC": "C5331611",
                      "Package": "SOP Advance (2-5Q1S) - confirm reel is not SOP Advance(N) 2-5W1A"}
            q += 1
            hs_fp = FP_PLAIN if bottom else FP_HS
            place(hs_fp, f"Q{q}", "TPHR8504PL", p["edge"], p["cx_h"], V_C, 180, bottom,
                  {"1": phn, "2": phn, "3": phn, "4": gh, "5": "VBAT", "6": "VBAT", "7": "VBAT", "8": "VBAT"},
                  dict(fields, Role=f"ESC{esc} phase {ph} high side"))
            fet_map.append((f"Q{q}", esc, ph, "HS", "bottom" if bottom else "top", p["edge"]))
            q += 1
            ls_fp = FP_PLAIN if bottom else FP_LS
            place(ls_fp, f"Q{q}", "TPHR8504PL", p["edge"], p["cx_l"], V_C, 0, bottom,
                  {"1": "GND", "2": "GND", "3": "GND", "4": gl, "5": phn, "6": phn, "7": phn, "8": phn},
                  dict(fields, Role=f"ESC{esc} phase {ph} low side"))
            fet_map.append((f"Q{q}", esc, ph, "LS", "bottom" if bottom else "top", p["edge"]))
            mp += 1
            place(FP_MOTOR, f"MP{mp}", f"M{esc}_{ph}", p["edge"], p["cx_l"], MOTOR_V, 0, bottom,
                  {"1": phn})
            # LS source bar pin1..pin3 (GND) on its own side
            ls_layer = pcbnew.B_Cu if bottom else pcbnew.F_Cu
            s1 = local_to_canon(PIN_X[0], GS_Y, 0, bottom)
            s3 = local_to_canon(PIN_X[2], GS_Y, 0, bottom)
            a = edge_xform(p["edge"], p["cx_l"] + s1[0], V_C + s1[1])
            b = edge_xform(p["edge"], p["cx_l"] + s3[0], V_C + s3[1])
            t = pcbnew.PCB_TRACK(board)
            t.SetStart(pcbnew.VECTOR2I(mm(a[0]), mm(a[1])))
            t.SetEnd(pcbnew.VECTOR2I(mm(b[0]), mm(b[1])))
            t.SetWidth(mm(0.6))
            t.SetLayer(ls_layer)
            t.SetNet(net("GND"))
            board.Add(t)
            # LS GND return via row inboard of the source pads (shared by top and
            # bottom LS: both source rows are GND), tied to pads 2/3 on BOTH sides
            if not bottom:
                vrow = V_C + LAND_HALF_L + LS_ROW_GAP
                for du in LS_ROW_U:
                    vx, vy = edge_xform(p["edge"], p["cx_l"] + du, vrow)
                    via = pcbnew.PCB_VIA(board)
                    via.SetPosition(pcbnew.VECTOR2I(mm(vx), mm(vy)))
                    via.SetWidth(mm(VIA_PAD))
                    via.SetDrill(mm(VIA_DRILL))
                    via.SetNet(net("GND"))
                    board.Add(via)
                for lay in (pcbnew.F_Cu, pcbnew.B_Cu):
                    segs = [((LS_ROW_U[0], V_C + GS_Y), (LS_ROW_U[0], vrow)),
                            ((LS_ROW_U[-1], V_C + GS_Y), (LS_ROW_U[-1], vrow)),
                            ((LS_ROW_U[0], vrow), (LS_ROW_U[-1], vrow))]
                    for (u0, v0), (u1, v1) in segs:
                        a = edge_xform(p["edge"], p["cx_l"] + u0, v0)
                        b = edge_xform(p["edge"], p["cx_l"] + u1, v1)
                        t = pcbnew.PCB_TRACK(board)
                        t.SetStart(pcbnew.VECTOR2I(mm(a[0]), mm(a[1])))
                        t.SetEnd(pcbnew.VECTOR2I(mm(b[0]), mm(b[1])))
                        t.SetWidth(mm(0.6))
                        t.SetLayer(lay)
                        t.SetNet(net("GND"))
                        board.Add(t)
            # phase pour on the FET side
            z = pcbnew.ZONE(board)
            z.SetLayer(ls_layer)
            z.SetNet(net(phn))
            ol = z.Outline()
            ol.NewOutline()
            for (cu, cv) in phase_polygon(p["cx_h"], p["cx_l"], bottom):
                x, y = edge_xform(p["edge"], cu, cv)
                ol.Append(mm(x), mm(y))
            z.SetLocalClearance(mm(0.2))
            z.SetMinThickness(mm(0.2))
            z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
            z.SetAssignedPriority(1)
            z.SetZoneName(f"{phn}_POUR")
            board.Add(z)

    # ---- mounting holes
    off = (BOARD - MOUNT_PITCH) / 2
    for i, (x, y) in enumerate([(off, off), (BOARD - off, off), (BOARD - off, BOARD - off), (off, BOARD - off)]):
        fp = pcbnew.FootprintLoad(libdir, FP_MOUNT)
        fp.SetReference(f"H{i + 1}")
        fp.SetValue("M3_Grommet")
        fp.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
        board.Add(fp)

    # ---- inner planes (full board), solid to vias, thermal spokes for any future THT
    for lname, nname in PLANES.items():
        z = pcbnew.ZONE(board)
        z.SetLayer(board.GetLayerID(lname))
        z.SetNet(net(nname))
        ol = z.Outline()
        ol.NewOutline()
        for x, y in [(0, 0), (BOARD, 0), (BOARD, BOARD), (0, BOARD)]:
            ol.Append(mm(x), mm(y))
        z.SetLocalClearance(mm(0.2))
        z.SetMinThickness(mm(0.2))
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
        z.SetThermalReliefGap(mm(0.25))
        z.SetThermalReliefSpokeWidth(mm(0.4))
        z.SetAssignedPriority(0)
        z.SetZoneName(f"{lname.split('.')[0]}_{nname}_PLANE")
        board.Add(z)

    # ---- MCU reserve outline (documentation only)
    lo = LAND_EDGE_V + 2 * LAND_HALF_L + LS_ROW_GAP + VIA_PAD / 2 + 0.2
    r = pcbnew.PCB_SHAPE(board)
    r.SetShape(pcbnew.SHAPE_T_RECT)
    r.SetStart(pcbnew.VECTOR2I(mm(lo), mm(lo)))
    r.SetEnd(pcbnew.VECTOR2I(mm(BOARD - lo), mm(BOARD - lo)))
    r.SetLayer(pcbnew.Cmts_User)
    r.SetWidth(mm(0.1))
    board.Add(r)
    tx = pcbnew.PCB_TEXT(board)
    tx.SetText(f"RESERVED U1 LQFP-100 + core ({BOARD - 2 * lo:.1f} mm sq per side)")
    tx.SetPosition(pcbnew.VECTOR2I(mm(BOARD / 2), mm(BOARD / 2)))
    tx.SetLayer(pcbnew.Cmts_User)
    tx.SetTextSize(pcbnew.VECTOR2I(mm(0.7), mm(0.7)))
    board.Add(tx)

    bad = [c for c in checks if not c[2]]
    if bad:
        for c in bad[:10]:
            print("PAD POSITION MISMATCH", c, file=sys.stderr)
        raise SystemExit(2)

    if fill:
        filler = pcbnew.ZONE_FILLER(board)
        filler.Fill(board.Zones())
    board.Save(pcb_path)
    with open(os.path.join(outdir, "power_stage_fet_map.json"), "w") as f:
        json.dump({"board_mm": BOARD, "thickness_mm": BOARD_THICKNESS, "centre_reserve_mm": BOARD - 2 * lo,
                   "fets": [dict(zip(("ref", "esc", "phase", "role", "side", "edge"), m)) for m in fet_map]},
                  f, indent=1)
        f.write("\n")
    print(f"[gen_power_stage_pcb] {pcb_path}: {q} MOSFETs, {mp} motor pads, "
          f"{len(checks)} pad positions verified, thickness {BOARD_THICKNESS} mm")
    return pcb_path


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--outdir", default=KDIR)
    ap.add_argument("--no-fill", action="store_true")
    a = ap.parse_args()
    build(a.outdir, fill=not a.no_fill)


if __name__ == "__main__":
    main()
