#!/usr/bin/env python3
"""
Nano H743 AIO - MCU core schematic generator (Revision 7.0)

Generates, deterministically:
  kicad/nano-h743-aio.kicad_sym   project-local symbol library
  kicad/nano-h743-aio.kicad_sch   root schematic (KiCad 8, A3)
  kicad/sym-lib-table             registers the project library

Scope of the sheet ("MCU core: HSE clock and analog supply"):
  * U1 STM32H743VIT6 (LQFP-100), all 100 pins.  Pin numbering verified
    against the LCSC/EasyEDA symbol for C114409 (PH0-OSC_IN=12,
    PH1-OSC_OUT=13, VSSA=19, VREF+=20, VDDA=21, VCAP=48/73).
  * 24 MHz HSE: Y1 3225 crystal (CL = 12 pF), C1/C2 16 pF C0G load caps,
    R1 0R drive-level / Rext position on OSC_OUT.
  * VDDA / VREF+ filter: +3V3 -> FB1 (600R@100MHz) -> R2 2R2 damping ->
    +3.3VA, local 1u+100n at VDDA and 1u+100n at VREF+.  VSSA -> GND.
  * VCAP (2 x 2.2 uF), VDD decoupling, VBAT, NRST, BOOT0.
  * Every GPIO that ardupilot/hwdef.dat assigns is broken out with a global
    label named after its hwdef function, so the schematic netlist and the
    firmware pin map can never silently diverge (tools/verify_mcu_core_sch.py
    cross-checks them).  Unassigned GPIOs get explicit no-connect flags.

Component values come from analysis/hse_vdda_analysis.py (see README,
"Revision 7.0 Engineering Changelog").  Re-run this script after any change
to hwdef.dat pin assignments, then re-run ERC.

Usage:  python3 tools/gen_mcu_core_sch.py [--out DIR]
"""
import os
import re
import uuid

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
KDIR = os.path.join(REPO, "kicad")
LIB = "nano-h743-aio"
PROJECT = "nano-h743-aio"
HWDEF = os.path.join(REPO, "ardupilot", "hwdef.dat")

_ns = uuid.UUID("6f1c2d0e-7a43-4b1e-9d1a-743a10c0e007")


def uid(tag):
    """Deterministic UUID so regeneration produces a minimal diff."""
    return str(uuid.uuid5(_ns, tag))


ROOT_UUID = uid("root-sheet")


def q(s):
    return '"' + str(s).replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n") + '"'


def f(v):
    """Format a coordinate (mm) without float noise."""
    v = round(v, 4)
    if v == int(v):
        return str(int(v))
    return ("%.4f" % v).rstrip("0").rstrip(".")


FONT = "(effects (font (size 1.27 1.27)))"
FONT_HIDE = "(effects (font (size 1.27 1.27)) hide)"

# ---------------------------------------------------------------------------
# STM32H743VITx LQFP-100 pin table (number -> name, electrical type)
# ---------------------------------------------------------------------------
MCU_PINS = {
    1: "PE2", 2: "PE3", 3: "PE4", 4: "PE5", 5: "PE6", 6: "VBAT", 7: "PC13",
    8: "PC14-OSC32_IN", 9: "PC15-OSC32_OUT", 10: "VSS", 11: "VDD",
    12: "PH0-OSC_IN", 13: "PH1-OSC_OUT", 14: "NRST", 15: "PC0", 16: "PC1",
    17: "PC2_C", 18: "PC3_C", 19: "VSSA", 20: "VREF+", 21: "VDDA", 22: "PA0",
    23: "PA1", 24: "PA2", 25: "PA3", 26: "VSS", 27: "VDD", 28: "PA4",
    29: "PA5", 30: "PA6", 31: "PA7", 32: "PC4", 33: "PC5", 34: "PB0",
    35: "PB1", 36: "PB2", 37: "PE7", 38: "PE8", 39: "PE9", 40: "PE10",
    41: "PE11", 42: "PE12", 43: "PE13", 44: "PE14", 45: "PE15", 46: "PB10",
    47: "PB11", 48: "VCAP", 49: "VSS", 50: "VDD", 51: "PB12", 52: "PB13",
    53: "PB14", 54: "PB15", 55: "PD8", 56: "PD9", 57: "PD10", 58: "PD11",
    59: "PD12", 60: "PD13", 61: "PD14", 62: "PD15", 63: "PC6", 64: "PC7",
    65: "PC8", 66: "PC9", 67: "PA8", 68: "PA9", 69: "PA10", 70: "PA11",
    71: "PA12", 72: "PA13", 73: "VCAP", 74: "VSS", 75: "VDD", 76: "PA14",
    77: "PA15", 78: "PC10", 79: "PC11", 80: "PC12", 81: "PD0", 82: "PD1",
    83: "PD2", 84: "PD3", 85: "PD4", 86: "PD5", 87: "PD6", 88: "PD7",
    89: "PB3", 90: "PB4", 91: "PB5", 92: "PB6", 93: "PB7", 94: "BOOT0",
    95: "PB8", 96: "PB9", 97: "PE0", 98: "PE1", 99: "VSS", 100: "VDD",
}


def mcu_pin_type(name):
    if name in ("VDD", "VSS", "VDDA", "VSSA", "VREF+", "VBAT"):
        return "power_in"
    if name == "VCAP":
        return "passive"          # LDO output decoupling node
    if name == "PH0-OSC_IN":
        return "input"
    if name == "PH1-OSC_OUT":
        return "output"
    if name == "BOOT0":
        return "input"
    return "bidirectional"      # GPIO and NRST (open-drain I/O)


def port_name(name):
    """'PH0-OSC_IN' -> 'PH0', 'PC2_C' -> 'PC2', 'PA5' -> 'PA5'."""
    m = re.match(r"^(P[A-K]\d+)", name)
    return m.group(1) if m else None


# ---------------------------------------------------------------------------
# hwdef.dat pin map
# ---------------------------------------------------------------------------
def load_hwdef_pins(path):
    pins = {}
    with open(path) as fh:
        for line in fh:
            line = line.split("#", 1)[0].strip()
            m = re.match(r"^(P[A-K]\d+)\s+(\S+)", line)
            if m:
                port, func = m.groups()
                if port in pins:
                    raise SystemExit("hwdef.dat assigns %s twice" % port)
                pins[port] = func
    return pins


def net_for_hwdef(func):
    """hwdef function -> schematic global label (KiCad-safe)."""
    return {"JTMS-SWDIO": "SWDIO", "JTCK-SWCLK": "SWCLK"}.get(func, func)


# ---------------------------------------------------------------------------
# Library symbols
# ---------------------------------------------------------------------------
def prop(name, value, x=0, y=0, rot=0, hide=False):
    return "(property %s %s (at %s %s %s) %s)" % (
        q(name), q(value), f(x), f(y), rot, FONT_HIDE if hide else FONT)


def pin_s(etype, x, y, rot, length, name, number, hide=False):
    return ("(pin %s line (at %s %s %s) (length %s)%s (name %s %s) (number %s %s))"
            % (etype, f(x), f(y), rot, f(length), " hide" if hide else "",
               q(name), FONT, q(number), FONT))


MCU_HALF_W = 25.4
MCU_HALF_H = 60.96
MCU_PIN_LEN = 5.08
MCU_SIDE_TOP = 57.15     # lib Y of the first side pin

# Functional pin layout (pin NUMBERS are the physical LQFP-100 numbers):
#   top    : supplies (VBAT + 5x VDD -> +3V3 bus, VREF+/VDDA -> +3.3VA bus)
#   bottom : returns (5x VSS + VSSA -> GND bus)
#   left   : system pins, then ports A and B
#   right  : ports C, D, E
TOP_PINS = [6, 11, 27, 50, 75, 100, 20, 21]
BOT_PINS = [10, 26, 49, 74, 99, 19]
SYS_PINS = [12, 13, 94, 48, 73, 8, 9, 14]


def _port_sort(num):
    nm = MCU_PINS[num]
    return (nm[1], int(re.match(r"P[A-K](\d+)", nm).group(1)))


def _side_lists():
    ports = [n for n in MCU_PINS if port_name(MCU_PINS[n]) and n not in SYS_PINS]
    left = SYS_PINS + sorted([n for n in ports if MCU_PINS[n][1] in "AB"], key=_port_sort)
    right = sorted([n for n in ports if MCU_PINS[n][1] in "CDE"], key=_port_sort)
    return left, right


def mcu_layout():
    left, right = _side_lists()
    lay = {}
    for i, n in enumerate(left):
        lay[n] = (-(MCU_HALF_W + MCU_PIN_LEN), MCU_SIDE_TOP - i * 2.54, 0)
    for i, n in enumerate(right):
        lay[n] = (MCU_HALF_W + MCU_PIN_LEN, MCU_SIDE_TOP - i * 2.54, 180)
    for i, n in enumerate(TOP_PINS):
        lay[n] = (-10.16 + i * 2.54, MCU_HALF_H + MCU_PIN_LEN, 270)
    for i, n in enumerate(BOT_PINS):
        lay[n] = (-10.16 + i * 2.54, -(MCU_HALF_H + MCU_PIN_LEN), 90)
    assert sorted(lay) == list(range(1, 101)), "pin layout incomplete"
    return lay


_LAYOUT = None


def mcu_pin_lib_xy(num):
    global _LAYOUT
    if _LAYOUT is None:
        _LAYOUT = mcu_layout()
    return _LAYOUT[num]


def sym_mcu(prefix):
    n = "STM32H743VITx"
    body = ("(rectangle (start %s %s) (end %s %s) (stroke (width 0.254) (type default)) "
            "(fill (type background)))" % (f(-MCU_HALF_W), f(MCU_HALF_H),
                                           f(MCU_HALF_W), f(-MCU_HALF_H)))
    pins = []
    for num in range(1, 101):
        x, y, r = mcu_pin_lib_xy(num)
        nm = MCU_PINS[num]
        pins.append(pin_s(mcu_pin_type(nm), x, y, r, MCU_PIN_LEN, nm, str(num)))
    return "\n".join([
        "(symbol %s (pin_names (offset 1.016)) (exclude_from_sim no) (in_bom yes) (on_board yes)" % q(prefix + n),
        prop("Reference", "U", -20.32, MCU_HALF_H + 2.54),
        prop("Value", "STM32H743VITx", 17.78, MCU_HALF_H + 2.54),
        prop("Footprint", "Package_QFP:LQFP-100_14x14mm_P0.5mm", 0, 0, hide=True),
        prop("Datasheet", "https://www.st.com/resource/en/datasheet/stm32h743vi.pdf", 0, 0, hide=True),
        prop("Description", "STM32H743VI Cortex-M7 480 MHz, 2 MB flash, 1 MB RAM, LQFP-100", 0, 0, hide=True),
        "(symbol %s %s)" % (q(n + "_0_1"), body),
        "(symbol %s %s)" % (q(n + "_1_1"), "\n".join(pins)),
        ")"])


def sym_two_pin(prefix, n, ref, desc, graphic):
    return "\n".join([
        "(symbol %s (pin_numbers hide) (pin_names (offset 0) hide) (exclude_from_sim no) (in_bom yes) (on_board yes)" % q(prefix + n),
        prop("Reference", ref, 2.54, 1.27),
        prop("Value", n, 2.54, -1.27),
        prop("Footprint", "", 0, 0, hide=True),
        prop("Datasheet", "~", 0, 0, hide=True),
        prop("Description", desc, 0, 0, hide=True),
        "(symbol %s %s)" % (q(n + "_0_1"), graphic),
        "(symbol %s %s %s)" % (q(n + "_1_1"),
                               pin_s("passive", 0, 3.81, 270, 1.27, "~", "1"),
                               pin_s("passive", 0, -3.81, 90, 1.27, "~", "2")),
        ")"])


def sym_r(prefix):
    g = ("(rectangle (start -1.016 2.54) (end 1.016 -2.54) (stroke (width 0.254) (type default)) "
         "(fill (type none)))")
    return sym_two_pin(prefix, "R", "R", "Resistor", g)


def sym_c(prefix):
    g = ("(polyline (pts (xy -2.032 0.762) (xy 2.032 0.762)) (stroke (width 0.508) (type default)) (fill (type none))) "
         "(polyline (pts (xy -2.032 -0.762) (xy 2.032 -0.762)) (stroke (width 0.508) (type default)) (fill (type none))) "
         "(polyline (pts (xy 0 2.54) (xy 0 0.762)) (stroke (width 0) (type default)) (fill (type none))) "
         "(polyline (pts (xy 0 -2.54) (xy 0 -0.762)) (stroke (width 0) (type default)) (fill (type none)))")
    return sym_two_pin(prefix, "C", "C", "Unpolarized capacitor", g)


def sym_fb(prefix):
    g = ("(polyline (pts (xy -1.905 -1.524) (xy 0.635 2.286) (xy 1.905 1.524) (xy -0.635 -2.286) (xy -1.905 -1.524)) "
         "(stroke (width 0.254) (type default)) (fill (type none))) "
         "(polyline (pts (xy 0 2.54) (xy 0 1.905)) (stroke (width 0) (type default)) (fill (type none))) "
         "(polyline (pts (xy 0 -2.54) (xy 0 -1.905)) (stroke (width 0) (type default)) (fill (type none)))")
    return sym_two_pin(prefix, "FerriteBead", "FB", "Ferrite bead", g)


def sym_crystal(prefix):
    n = "Crystal_GND24"
    g = ("(rectangle (start -1.143 2.54) (end 1.143 -2.54) (stroke (width 0.3048) (type default)) (fill (type none))) "
         "(polyline (pts (xy -2.54 0) (xy -1.905 0)) (stroke (width 0) (type default)) (fill (type none))) "
         "(polyline (pts (xy -1.905 -1.27) (xy -1.905 1.27)) (stroke (width 0.508) (type default)) (fill (type none))) "
         "(polyline (pts (xy 1.905 -1.27) (xy 1.905 1.27)) (stroke (width 0.508) (type default)) (fill (type none))) "
         "(polyline (pts (xy 2.54 0) (xy 1.905 0)) (stroke (width 0) (type default)) (fill (type none))) "
         "(polyline (pts (xy -2.54 -2.286) (xy -2.54 -3.556) (xy 2.54 -3.556) (xy 2.54 -2.286)) (stroke (width 0) (type default)) (fill (type none))) "
         "(polyline (pts (xy -2.54 2.286) (xy -2.54 3.556) (xy 2.54 3.556) (xy 2.54 2.286)) (stroke (width 0) (type default)) (fill (type none))) "
         "(polyline (pts (xy 0 -3.556) (xy 0 -3.81)) (stroke (width 0) (type default)) (fill (type none))) "
         "(polyline (pts (xy 0 3.556) (xy 0 3.81)) (stroke (width 0) (type default)) (fill (type none)))")
    pins = " ".join([
        pin_s("passive", -3.81, 0, 0, 1.27, "1", "1"),
        pin_s("passive", 0, -5.08, 90, 1.27, "2", "2"),
        pin_s("passive", 3.81, 0, 180, 1.27, "3", "3"),
        pin_s("passive", 0, 5.08, 270, 1.27, "4", "4"),
    ])
    return "\n".join([
        "(symbol %s (pin_names (offset 1.016) hide) (exclude_from_sim no) (in_bom yes) (on_board yes)" % q(prefix + n),
        prop("Reference", "Y", 0, 6.35),
        prop("Value", n, 0, -6.35),
        prop("Footprint", "", 0, 0, hide=True),
        prop("Datasheet", "~", 0, 0, hide=True),
        prop("Description", "Four-pin crystal: 1/3 = resonator, 2/4 = case GND", 0, 0, hide=True),
        "(symbol %s %s)" % (q(n + "_0_1"), g),
        "(symbol %s %s)" % (q(n + "_1_1"), pins),
        ")"])


def sym_power(prefix, n, kind):
    """kind: 'gnd' (arrow down) or 'rail' (bar up)."""
    if kind == "gnd":
        g = ("(polyline (pts (xy 0 0) (xy 0 -1.27) (xy 1.27 -1.27) (xy 0 -2.54) (xy -1.27 -1.27) (xy 0 -1.27)) "
             "(stroke (width 0) (type default)) (fill (type none)))")
        vy = -3.81
    else:
        g = ("(polyline (pts (xy -0.762 1.27) (xy 0 2.54)) (stroke (width 0) (type default)) (fill (type none))) "
             "(polyline (pts (xy 0 0) (xy 0 2.54)) (stroke (width 0) (type default)) (fill (type none))) "
             "(polyline (pts (xy 0 2.54) (xy 0.762 1.27)) (stroke (width 0) (type default)) (fill (type none)))")
        vy = 3.556
    return "\n".join([
        "(symbol %s (power) (pin_names (offset 0)) (exclude_from_sim no) (in_bom yes) (on_board yes)" % q(prefix + n),
        prop("Reference", "#PWR", 0, -6.35, hide=True),
        prop("Value", n, 0, vy),
        prop("Footprint", "", 0, 0, hide=True),
        prop("Datasheet", "", 0, 0, hide=True),
        prop("Description", "Power symbol creates a global label with name \"%s\"" % n, 0, 0, hide=True),
        "(symbol %s %s)" % (q(n + "_0_1"), g),
        "(symbol %s %s)" % (q(n + "_1_1"), pin_s("power_in", 0, 0, 90 if kind == "rail" else 270, 0, n, "1", hide=True)),
        ")"])


def sym_pwr_flag(prefix):
    n = "PWR_FLAG"
    g = ("(polyline (pts (xy 0 0) (xy 0 1.27) (xy -1.016 1.905) (xy 0 2.54) (xy 1.016 1.905) (xy 0 1.27)) "
         "(stroke (width 0) (type default)) (fill (type none)))")
    return "\n".join([
        "(symbol %s (power) (pin_numbers hide) (pin_names (offset 0) hide) (exclude_from_sim no) (in_bom yes) (on_board yes)" % q(prefix + n),
        prop("Reference", "#FLG", 0, 1.905, hide=True),
        prop("Value", n, 0, 3.81),
        prop("Footprint", "", 0, 0, hide=True),
        prop("Datasheet", "~", 0, 0, hide=True),
        prop("Description", "Special symbol for telling ERC where power comes from", 0, 0, hide=True),
        "(symbol %s %s)" % (q(n + "_0_1"), g),
        "(symbol %s %s)" % (q(n + "_1_1"), pin_s("power_out", 0, 0, 90, 0, "pwr", "1")),
        ")"])


def all_symbols(prefix):
    return [sym_mcu(prefix), sym_r(prefix), sym_c(prefix), sym_fb(prefix),
            sym_crystal(prefix), sym_power(prefix, "GND", "gnd"),
            sym_power(prefix, "+3V3", "rail"), sym_power(prefix, "+3.3VA", "rail"),
            sym_pwr_flag(prefix)]


# Pin connection offsets in schematic coordinates (Y down) for each symbol
PIN_OFFS = {
    "R": {"1": (0, -3.81), "2": (0, 3.81)},
    "C": {"1": (0, -3.81), "2": (0, 3.81)},
    "FerriteBead": {"1": (0, -3.81), "2": (0, 3.81)},
    "Crystal_GND24": {"1": (-3.81, 0), "3": (3.81, 0), "2": (0, 5.08), "4": (0, -5.08)},
}


# ---------------------------------------------------------------------------
# Schematic builder
# ---------------------------------------------------------------------------
class Sheet:
    def __init__(self):
        self.items = []
        self.pwr_n = 0
        self.flg_n = 0

    def _instances(self, ref):
        return ('(instances (project %s (path %s (reference %s) (unit 1))))'
                % (q(PROJECT), q("/" + ROOT_UUID), q(ref)))

    def symbol(self, libname, ref, value, x, y, rot=0, footprint="", fields=None,
               pins=("1", "2"), in_bom=True, ref_xy=None, val_xy=None, hide_val=False,
               dnp=False):
        fields = fields or {}
        u = uid("sym-" + ref)
        rx, ry = ref_xy if ref_xy else (x + 2.54, y - 1.27)
        vx, vy = val_xy if val_xy else (x + 2.54, y + 1.27)
        just = "(effects (font (size 1.27 1.27)) (justify left))"
        props = [
            "(property \"Reference\" %s (at %s %s 0) %s)" % (q(ref), f(rx), f(ry), FONT_HIDE if ref.startswith("#") else just),
            "(property \"Value\" %s (at %s %s 0) %s)" % (q(value), f(vx), f(vy), FONT_HIDE if hide_val else just),
            "(property \"Footprint\" %s (at %s %s 0) %s)" % (q(footprint), f(x), f(y), FONT_HIDE),
            "(property \"Datasheet\" \"~\" (at %s %s 0) %s)" % (f(x), f(y), FONT_HIDE),
            "(property \"Description\" \"\" (at %s %s 0) %s)" % (f(x), f(y), FONT_HIDE),
        ]
        for k, v in fields.items():
            props.append("(property %s %s (at %s %s 0) %s)" % (q(k), q(v), f(x), f(y), FONT_HIDE))
        pin_s_ = " ".join("(pin %s (uuid %s))" % (q(p), q(uid("pin-%s-%s" % (ref, p)))) for p in pins)
        self.items.append(
            "(symbol (lib_id %s) (at %s %s %d) (unit 1) (exclude_from_sim no) (in_bom %s) (on_board yes) (dnp %s) "
            "(uuid %s)\n%s\n%s\n%s)"
            % (q(LIB + ":" + libname), f(x), f(y), rot, "yes" if in_bom else "no",
               "yes" if dnp else "no", q(u), "\n".join(props), pin_s_, self._instances(ref)))

    def wire(self, x1, y1, x2, y2):
        self.items.append("(wire (pts (xy %s %s) (xy %s %s)) (stroke (width 0) (type default)) (uuid %s))"
                          % (f(x1), f(y1), f(x2), f(y2), q(uid("w-%s-%s-%s-%s" % (f(x1), f(y1), f(x2), f(y2))))))

    def label(self, name, x, y, rot=0):
        just = "(justify right bottom)" if rot == 180 else "(justify left bottom)"
        self.items.append("(label %s (at %s %s %d) (effects (font (size 1.27 1.27)) %s) (uuid %s))"
                          % (q(name), f(x), f(y), rot, just, q(uid("lbl-%s-%s-%s" % (name, f(x), f(y))))))

    def glabel(self, name, x, y, rot=0):
        just = "(justify right)" if rot == 180 else "(justify left)"
        self.items.append(
            "(global_label %s (shape bidirectional) (at %s %s %d) (fields_autoplaced yes) "
            "(effects (font (size 1.27 1.27)) %s) (uuid %s) "
            "(property \"Intersheetrefs\" \"${INTERSHEET_REFS}\" (at %s %s 0) (effects (font (size 1.27 1.27)) hide)))"
            % (q(name), f(x), f(y), rot, just, q(uid("glbl-%s" % name)), f(x), f(y)))

    def junction(self, x, y):
        self.items.append("(junction (at %s %s) (diameter 0) (color 0 0 0 0) (uuid %s))"
                          % (f(x), f(y), q(uid("j-%s-%s" % (f(x), f(y))))))

    def noconnect(self, x, y):
        self.items.append("(no_connect (at %s %s) (uuid %s))" % (f(x), f(y), q(uid("nc-%s-%s" % (f(x), f(y))))))

    def text(self, s, x, y, size=1.27):
        self.items.append("(text %s (exclude_from_sim no) (at %s %s 0) (effects (font (size %s %s)) (justify left top)) (uuid %s))"
                          % (q(s), f(x), f(y), size, size, q(uid("txt-%s-%s" % (f(x), f(y))))))

    def power(self, net, x, y, rot=0):
        self.pwr_n += 1
        ref = "#PWR%03d" % self.pwr_n
        vy = y + 3.81 if net == "GND" else y - 3.556
        if rot == 180:
            vy = y - 3.81 if net == "GND" else y + 3.556
        self.symbol(net, ref, net, x, y, rot=rot, pins=("1",), in_bom=False,
                    ref_xy=(x, y + 6.35), val_xy=(x, vy))
        # center the value text
        self.items[-1] = self.items[-1].replace(
            "(property \"Value\" %s (at %s %s 0) (effects (font (size 1.27 1.27)) (justify left)))" % (q(net), f(x), f(vy)),
            "(property \"Value\" %s (at %s %s 0) %s)" % (q(net), f(x), f(vy), FONT))

    def pwr_flag(self, x, y):
        self.flg_n += 1
        ref = "#FLG%02d" % self.flg_n
        self.symbol("PWR_FLAG", ref, "PWR_FLAG", x, y, pins=("1",), in_bom=False,
                    ref_xy=(x, y - 1.905), val_xy=(x, y - 3.81))

    # Connect a symbol pin endpoint to a net via a stub wire.
    def attach(self, net, px, py, dx, dy, kind="label"):
        ex, ey = px + dx, py + dy
        self.wire(px, py, ex, ey)
        if net in ("GND", "+3V3", "+3.3VA"):
            rot = 0
            if net == "GND" and dy < 0:
                rot = 180
            if net != "GND" and dy > 0:
                rot = 180
            self.power(net, ex, ey, rot)
        elif kind == "global":
            self.glabel(net, ex, ey, 180 if dx < 0 else 0)
        else:
            self.label(net, ex, ey, 180 if dx < 0 else 0)


def passive(sh, libname, ref, value, x, y, net1, net2, footprint, fields, dnp=False):
    """Vertical two-pin part; pin1 up, pin2 down; stubs 2.54 mm."""
    sh.symbol(libname, ref, value, x, y, footprint=footprint, fields=fields,
              ref_xy=(x + 2.54, y - 1.27), val_xy=(x + 2.54, y + 1.27), dnp=dnp)
    o = PIN_OFFS[libname]
    sh.attach(net1, x + o["1"][0], y + o["1"][1], 0, -2.54)
    sh.attach(net2, x + o["2"][0], y + o["2"][1], 0, 2.54)


FP_C0402 = "Capacitor_SMD:C_0402_1005Metric"
FP_C0603 = "Capacitor_SMD:C_0603_1608Metric"
FP_R0402 = "Resistor_SMD:R_0402_1005Metric"
FP_L0402 = "Inductor_SMD:L_0402_1005Metric"
FP_XTAL = "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm"


def build():
    hw = load_hwdef_pins(HWDEF)
    used_ports = {port_name(MCU_PINS[n]) for n in MCU_PINS if port_name(MCU_PINS[n])}
    missing = sorted(set(hw) - used_ports)
    if missing:
        raise SystemExit("hwdef.dat uses pins not on LQFP-100: %s" % missing)

    sh = Sheet()

    # ---------------- U1 -----------------
    UX, UY = 215.9, 148.59
    sh.symbol("STM32H743VITx", "U1", "STM32H743VITx", UX, UY,
              footprint="Package_QFP:LQFP-100_14x14mm_P0.5mm",
              fields={"MPN": "STM32H743VIT6", "Manufacturer": "STMicroelectronics",
                      "LCSC": "C114409",
                      "Silicon_Rev": "V required (DBGMCU REV_ID 0x2003, die mark 'V'); rev Y limited to 400 MHz"},
              pins=tuple(str(n) for n in range(1, 101)),
              ref_xy=(UX - 22.86, UY - MCU_HALF_H - 2.54),
              val_xy=(UX + 10.16, UY - MCU_HALF_H - 2.54))

    special = {
        "VDD": "+3V3", "VBAT": "+3V3", "VSS": "GND", "VSSA": "GND",
        "VDDA": "+3.3VA", "VREF+": "+3.3VA", "VCAP": "VCAP",
        "PH0-OSC_IN": "HSE_IN", "PH1-OSC_OUT": "HSE_OUT_MCU",
        "NRST": "NRST", "BOOT0": "BOOT0",
    }
    def tip(num):
        lx, ly, _ = mcu_pin_lib_xy(num)
        return UX + lx, UY - ly

    # Supply / return pins along the top and bottom edges are tied together
    # by a short bus with one rail symbol (junction dots at every tee).
    def bus(pins, net, dy, sym_at):
        pts = [tip(n) for n in pins]
        by = pts[0][1] + dy
        for (px, py) in pts:
            sh.wire(px, py, px, by)
        for (x1, _), (x2, _) in zip(pts, pts[1:]):
            sh.wire(x1, by, x2, by)
        for (px, _) in pts[1:-1]:
            sh.junction(px, by)
        sx = pts[sym_at][0]
        sh.power(net, sx, by, 0)

    bus([6, 11, 27, 50, 75, 100], "+3V3", -2.54, 0)
    bus([20, 21], "+3.3VA", -2.54, 1)
    bus(BOT_PINS, "GND", 2.54, 0)

    for num in range(1, 101):
        nm = MCU_PINS[num]
        if num in TOP_PINS or num in BOT_PINS:
            continue
        px, py = tip(num)
        lx = mcu_pin_lib_xy(num)[0]
        dx = -5.08 if lx < 0 else 5.08
        if nm in special:
            net = special[nm]
            sh.attach(net, px, py, dx, 0, "global" if net == "NRST" else "label")
        else:
            port = port_name(nm)
            if port in hw:
                sh.attach(net_for_hwdef(hw[port]), px, py, dx, 0, "global")
            else:
                sh.noconnect(px, py)

    # ---------------- HSE block (left, top) ----------------
    hx, hy = 40.64, 50.8
    sh.text("24 MHz HSE (Rev 7.0)", hx - 10.16, hy - 22.86, 2.0)
    sh.symbol("Crystal_GND24", "Y1", "24MHz CL=12pF", hx + 20.32, hy,
              footprint=FP_XTAL,
              fields={"MPN": "TXM24M0004322FBCDO00T", "Manufacturer": "Yajingxin",
                      "LCSC": "C362340", "Spec": "24.000 MHz, CL 12 pF, ESR<=30R, +/-10 ppm, +/-20 ppm -40..85C"},
              pins=("1", "2", "3", "4"),
              ref_xy=(hx + 22.86, hy - 7.62), val_xy=(hx + 22.86, hy + 8.89))
    yx, yy = hx + 20.32, hy
    sh.attach("HSE_IN", yx - 3.81, yy, -2.54, 0)
    sh.attach("HSE_OUT", yx + 3.81, yy, 2.54, 0)
    sh.attach("GND", yx, yy + 5.08, 0, 2.54)
    sh.attach("GND", yx, yy - 5.08, 0, -2.54)

    for ref, net, x in (("C1", "HSE_IN", hx), ("C2", "HSE_OUT", hx + 40.64)):
        passive(sh, "C", ref, "16pF", x, hy + 17.78, net, "GND", FP_C0402,
                {"MPN": "GJM1555C1H160FB01J", "Manufacturer": "Murata", "LCSC": "C3853959",
                 "Spec": "16 pF +/-1% C0G 50V 0402 - CL = 8 pF + 4 pF stray = 12 pF"})
    passive(sh, "R", "R1", "0R", hx + 60.96, hy + 17.78, "HSE_OUT_MCU", "HSE_OUT", FP_R0402,
            {"Spec": "0R 0402 - Rext position; never above 6R8 (gm_crit x1.2 margin, analysis R1_ext_max)",
             "LCSC": "C17168"})

    # ---------------- VDDA / VREF+ filter (left, middle) ----------------
    vx, vy = 40.64, 121.92
    sh.text("VDDA / VREF+ supply filter (Rev 7.0)", vx - 10.16, vy - 22.86, 2.0)
    passive(sh, "FerriteBead", "FB1", "600R@100MHz", vx, vy, "+3V3", "VDDA_FB", FP_L0402,
            {"MPN": "BLM15AX601SN1D", "Manufacturer": "Murata", "LCSC": "C76886",
             "Spec": "600R@100MHz, Rdc 0.34R max, 500 mA"})
    passive(sh, "R", "R2", "2R2", vx + 20.32, vy, "VDDA_FB", "+3.3VA", FP_R0402,
            {"Spec": "2.2R 1% 0402 - LC damping: peaking <=0.5 dB over bead L 0.47-4.7 uH", "MPN": "0402WGF220KTCE", "LCSC": "C25101"})
    passive(sh, "C", "C3", "1uF", vx + 40.64, vy, "+3.3VA", "GND", FP_C0402,
            {"Spec": "1 uF X5R 25V 0402 - at VDDA pin 21", "MPN": "CL05A105KA5NQNC", "LCSC": "C52923"})
    passive(sh, "C", "C4", "100nF", vx + 55.88, vy, "+3.3VA", "GND", FP_C0402,
            {"Spec": "100 nF X7R 16V 0402 - at VDDA pin 21", "LCSC": "C1525"})
    passive(sh, "C", "C5", "1uF", vx + 71.12, vy, "+3.3VA", "GND", FP_C0402,
            {"Spec": "1 uF X5R 25V 0402 - at VREF+ pin 20", "MPN": "CL05A105KA5NQNC", "LCSC": "C52923"})
    passive(sh, "C", "C6", "100nF", vx + 86.36, vy, "+3.3VA", "GND", FP_C0402,
            {"Spec": "100 nF X7R 16V 0402 - at VREF+ pin 20", "LCSC": "C1525"})

    # ---------------- VCAP / VDD / misc (left, bottom) ----------------
    cx, cy = 40.64, 190.5
    sh.text("Core LDO, VDD decoupling, reset, boot", cx - 10.16, cy - 22.86, 2.0)
    passive(sh, "C", "C7", "2.2uF", cx, cy, "VCAP", "GND", FP_C0603,
            {"Spec": "2.2 uF X7R 10V 0603, ESR<100 mR - VCAP pin 48", "MPN": "CL10B225KP8NNNC", "LCSC": "C100082"})
    passive(sh, "C", "C8", "2.2uF", cx + 15.24, cy, "VCAP", "GND", FP_C0603,
            {"Spec": "2.2 uF X7R 10V 0603, ESR<100 mR - VCAP pin 73", "MPN": "CL10B225KP8NNNC", "LCSC": "C100082"})
    for i, ref in enumerate(["C9", "C10", "C11", "C12", "C13"]):
        passive(sh, "C", ref, "100nF", cx + 30.48 + i * 15.24, cy, "+3V3", "GND", FP_C0402,
                {"Spec": "100 nF X7R 16V 0402 - VDD pin %s" % (11, 27, 50, 75, 100)[i], "LCSC": "C1525"})
    passive(sh, "C", "C14", "4.7uF", cx, cy + 25.4, "+3V3", "GND", FP_C0603,
            {"Spec": "4.7 uF X5R 16V 0603 - VDD bulk", "MPN": "CL10A475KO8NNNC", "LCSC": "C19666"})
    passive(sh, "C", "C15", "100nF", cx + 15.24, cy + 25.4, "+3V3", "GND", FP_C0402,
            {"Spec": "100 nF - VBAT pin 6 (tied to VDD, no RTC backup)", "LCSC": "C1525"})
    passive(sh, "C", "C16", "100nF", cx + 30.48, cy + 25.4, "NRST", "GND", FP_C0402,
            {"Spec": "100 nF - NRST filter", "LCSC": "C1525"})
    passive(sh, "R", "R3", "10k", cx + 45.72, cy + 25.4, "BOOT0", "GND", FP_R0402,
            {"Spec": "10k 1% 0402 - BOOT0 pull-down (boot from flash)", "LCSC": "C25744"})

    # ---------------- power flags ----------------
    fx, fy = 345.44, 40.64
    for i, net in enumerate(["+3V3", "GND", "+3.3VA"]):
        x = fx + i * 15.24
        sh.pwr_flag(x, fy)
        sh.wire(x, fy, x, fy + 5.08)
        # place the power symbol so its pin lands on the wire end
        if net == "GND":
            sh.power(net, x, fy + 5.08, 0)
        else:
            sh.wire(x, fy + 5.08, x + 5.08, fy + 5.08)
            sh.power(net, x + 5.08, fy + 5.08, 0)

    # ---------------- design notes ----------------
    notes = (
        "HSE DESIGN (analysis/hse_vdda_analysis.py, report analysis/hse_vdda_report.json)\n"
        "- 24 MHz chosen so ArduPilot can run 480 MHz: PLL1 24/M3=8 MHz ref, xN120 = 960 MHz VCO, /P2 = 480 MHz SYSCLK.\n"
        "  25 MHz cannot: stm32h7_mcuconf.h has no 480 MHz PLL set for 25 MHz (#error). PLL3: 24/6x72/6 = 48 MHz USB.\n"
        "- CL = C1*C2/(C1+C2) + Cstray = 8 pF + ~4 pF = 12.0 pF = spec -> 0 ppm nominal, +24/-20 ppm worst corner (C1m 8 fF).\n"
        "- gm_crit = 4*ESR*(2*pi*f)^2*(C0+CL)^2 = 0.94 mA/V worst corner (ESR 30R, C0 5pF) -> x1.59 vs Gm_critmax 1.5 mA/V.\n"
        "  Substitutes need ESR_max <= 40R at CL 12 pF, <= 20R at CL 20 pF (x1.2 margin). R1 never above 6R8.\n"
        "LAYOUT RULES\n"
        "- Y1, C1, C2 within 5 mm of pins 12/13, same layer as U1, no vias in OSC traces, OSC traces < 0.1 mm wide-match.\n"
        "- Solid GND guard ring around Y1/C1/C2 stitched to In1 GND every 1 mm; no other signals under Y1 on In1.\n"
        "- C1/C2 GND returns to one GND island tied to VSS pin 10 - do NOT route ESC/motor return currents under it.\n"
        "- FB1 + R2 at the +3V3 entry; C3/C4 within 1 mm of pin 21, C5/C6 within 1 mm of pin 20; VSSA pin 19 via\n"
        "  directly into the analog GND region of In1 (single solid plane, no split).\n"
    )
    sh.text(notes, 282.0, 160.0)

    return sh, hw


def write_lib(kdir=KDIR):
    body = "\n".join(all_symbols(""))
    out = "(kicad_symbol_lib (version 20231120) (generator \"nano_h743_gen_mcu_core\") (generator_version \"8.0\")\n%s\n)\n" % body
    with open(os.path.join(kdir, LIB + ".kicad_sym"), "w") as fh:
        fh.write(out)


def write_sch(sh, kdir=KDIR):
    libsyms = "\n".join(all_symbols(LIB + ":"))
    out = "\n".join([
        "(kicad_sch (version 20231120) (generator \"nano_h743_gen_mcu_core\") (generator_version \"8.0\")",
        "(uuid %s)" % q(ROOT_UUID),
        "(paper \"A3\")",
        "(title_block (title \"Nano H743 AIO - MCU core: 24 MHz HSE + VDDA filter\")",
        "  (date \"2026-11-06\") (rev \"7.0\") (company \"Sreesai Arjun Engineering\")",
        "  (comment 1 \"STM32H743VIT6 480 MHz: HSE 24 MHz, PLL1 M3 N120 P2\")",
        "  (comment 2 \"Y1 CL 12 pF, C1/C2 16 pF C0G; VDDA via FB1 + R2 2R2\")",
        "  (comment 3 \"Generated: tools/gen_mcu_core_sch.py + hwdef.dat\")",
        "  (comment 4 \"Production Revision 7.0 / 150\"))",
        "(lib_symbols\n%s\n)" % libsyms,
        "\n".join(sh.items),
        "(sheet_instances (path \"/\" (page \"1\")))",
        ")",
    ])
    with open(os.path.join(kdir, PROJECT + ".kicad_sch"), "w") as fh:
        fh.write(out + "\n")


def write_lib_table(kdir=KDIR):
    with open(os.path.join(kdir, "sym-lib-table"), "w") as fh:
        fh.write("(sym_lib_table\n  (version 7)\n  (lib (name \"%s\")(type \"KiCad\")(uri \"${KIPRJMOD}/%s.kicad_sym\")(options \"\")"
                 "(descr \"Nano H743 AIO project symbols (generated)\"))\n)\n" % (LIB, LIB))


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="Generate the MCU core schematic")
    ap.add_argument("--out", default=KDIR, help="output directory (default: kicad/)")
    out = ap.parse_args().out
    os.makedirs(out, exist_ok=True)
    sheet, hw = build()
    write_lib(out)
    write_sch(sheet, out)
    write_lib_table(out)
    print("hwdef pins broken out: %d; schematic items: %d" % (len(hw), len(sheet.items)))
