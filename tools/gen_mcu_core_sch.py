#!/usr/bin/env python3
"""
Nano H743 AIO - MCU core schematic generator (Revision 7.0, DShot termination
Rev 10.0, dual-IMU block Rev 11.0, DPS310 baro Rev 12.0)

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
  * Rev 10.0: R4..R7 100R series resistors TIM1_CH1..4 -> ESC1..4_DSHOT
    (bidirectional DShot600 lines to the integrated 4-in-1 ESC).
  * Rev 11.0 (sheet grown A3 -> A2; block in the new lower area):
      - U2 IMU1 / U3 IMU2 ICM-42688-P (LGA-14, LCSC C1850418).  Pinout per
        TDK DS-000347 rev 1.2 sec. 4.1 (fetched via LCSC mirror
        https://pcbparts.dev/mcp/datasheet/C1850418): RESV pin 7 "Connect to
        GND" -> GND; RESV 2/3/10/11 "No Connect or Connect to GND" -> NC.
      - Per-IMU supply anti-alias filter: +3V3 -> FB2/FB3 (BLM15AX601SN1D)
        -> IMUn_FB -> R8/R9 22R -> +3V3_IMUn; C17/C20 10uF bulk,
        C18/C21 100nF at VDD (pin 8), C19/C22 100nF at VDDIO (pin 5).
      - SPI source termination 33R: MCU-driven SCK/MOSI/CS (R10-R12, R19-R21)
        at U1, IMU-driven SDO (R18, R22) at the IMU.  Pin 1 of every series
        resistor is the MCU-label (hwdef) net, pin 2 the IMU-pin net.
      - Y2 SX2M32.768KM20F30TNN 32.768 kHz CMOS XO (C2901603), OE strapped
        high by R17 10k, C23 100nF; OUT -> IMU_CLKIN star: R14 33R ->
        IMU1_CLKIN (U2 pin 9), R15 33R -> IMU2_CLKIN (U3 pin 9),
        R16 1k -> IMU_CLKIN_MON (U1 PD12, loss-of-clock monitor).
      - INT1 (pin 4) direct to IMU1_DRDY (PC4, EXTI4) / IMU2_DRDY (PE1, EXTI1).
      Refdes note: the frozen Rev 11 plan named the IMU caps C5..C11; those
      refs are already used on this sheet, so they are C17..C23 here
      (C5->C17, C6->C18, C7->C19, C8->C20, C9->C21, C10->C22, C11->C23).
  * Rev 12.0: U4 DPS310 barometer on I2C1 (hwdef 'BARO DPS310 I2C:0:0x76').
      I2C mode: CSB (2) -> VDDIO rail, SDO (5) -> GND => 7-bit address 0x76.
      +3V3 -> FB4 (BLM15AX601SN1D) -> +3V3_BARO; C24 1uF + C25 100nF at VDD
      (8), C26 100nF at VDDIO (6).  R23/R24 2.2k pull-ups I2C1_SCL/SDA ->
      +3V3 at U1; R25/R26 33R series I2C1_SCL/SDA -> BARO_SCL/BARO_SDA at U4
      (pin 1 = MCU/hwdef net, pin 2 = baro-pin net).  Pinout per LCSC C130156
      symbol: 1 GND, 2 CSB, 3 SDI, 4 SCK, 5 SDO, 6 VDDIO, 7 GND, 8 VDD.
      Footprint: KiCad 8 Package_LGA:Bosch_LGA-8_2x2.5mm_P0.65mm_Clockwise-
      PinNumbering (no Infineon PG-VLGA-8 footprint in the KiCad library; the
      Bosch BMP280 land pattern has the same 2.0x2.5 body, 0.65 pitch, 1.6 mm
      row spacing and clockwise 1..8 numbering as the DPS310 PG-VLGA-8-2).
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


def _sym_ic(prefix, n, ref, value, fp, ds, desc, half_w, half_h, pins, name_off=1.016):
    body = ("(rectangle (start %s %s) (end %s %s) (stroke (width 0.254) (type default)) "
            "(fill (type background)))" % (f(-half_w), f(half_h), f(half_w), f(-half_h)))
    return "\n".join([
        "(symbol %s (pin_names (offset %s)) (exclude_from_sim no) (in_bom yes) (on_board yes)" % (q(prefix + n), f(name_off)),
        prop("Reference", ref, -half_w, half_h + 1.27),
        prop("Value", value, half_w, -half_h - 1.27),
        prop("Footprint", fp, 0, 0, hide=True),
        prop("Datasheet", ds, 0, 0, hide=True),
        prop("Description", desc, 0, 0, hide=True),
        "(symbol %s %s)" % (q(n + "_0_1"), body),
        "(symbol %s %s)" % (q(n + "_1_1"), "\n".join(
            pin_s(t, x, y, r, 2.54, nm, num) for (num, nm, t, x, y, r) in pins)),
        ")"])


# Rev 11.0: ICM-42688-P, LGA-14.  Numbers/names = LCSC C1850418 symbol and
# TDK DS-000347 rev 1.2 table 4.1.  (number, name, type, lib x, lib y, rot)
ICM_PINS = (
    ("13", "AP_SCL/AP_SCLK", "input", -27.94, 5.08, 0),
    ("14", "AP_SDA/AP_SDIO/AP_SDI", "bidirectional", -27.94, 2.54, 0),
    ("12", "AP_CS", "input", -27.94, 0, 0),
    ("1", "AP_SDO/AP_AD0", "tri_state", -27.94, -2.54, 0),
    ("4", "INT1/INT", "output", 27.94, 5.08, 180),
    ("9", "INT2/FSYNC/CLKIN", "bidirectional", 27.94, 0, 180),
    ("2", "RESV", "passive", 27.94, -2.54, 180),
    ("3", "RESV", "passive", 27.94, -5.08, 180),
    ("10", "RESV", "passive", 27.94, -7.62, 180),
    ("11", "RESV", "passive", 27.94, -10.16, 180),
    ("8", "VDD", "power_in", -2.54, 15.24, 270),
    ("5", "VDDIO", "power_in", 2.54, 15.24, 270),
    ("6", "GND", "power_in", -2.54, -15.24, 90),
    ("7", "RESV", "passive", 2.54, -15.24, 90),
)
FP_ICM = "Package_LGA:LGA-14_3x2.5mm_P0.5mm_LayoutBorder3x4y"
FP_XO2520 = "Oscillator:Oscillator_SMD_ECS_2520MV-xxx-xx-4Pin_2.5x2.0mm"

# 4-pin CMOS oscillator: 1 OE, 2 GND, 3 OUT, 4 VDD (LCSC C2901603 symbol)
XO_PINS = (
    ("1", "OE", "input", -10.16, 0, 0),
    ("3", "OUT", "output", 10.16, 0, 180),
    ("4", "VDD", "power_in", 0, 10.16, 270),
    ("2", "GND", "power_in", 0, -10.16, 90),
)


def sym_icm42688(prefix):
    return _sym_ic(prefix, "ICM-42688-P", "U", "ICM-42688-P", FP_ICM,
                   "https://invensense.tdk.com/wp-content/uploads/2020/04/ds-000347_icm-42688-p-datasheet.pdf",
                   "6-axis IMU, SPI/I2C/I3C, CLKIN 31-50 kHz on pin 9, LGA-14 2.5x3 mm",
                   25.4, 12.7, ICM_PINS)


def sym_xo(prefix):
    return _sym_ic(prefix, "XO_OE_4p", "Y", "XO_OE_4p", FP_XO2520, "~",
                   "4-pin CMOS clock oscillator with output enable (1 OE, 2 GND, 3 OUT, 4 VDD)",
                   7.62, 7.62, XO_PINS)


# Rev 12.0: Infineon DPS310 (PG-VLGA-8-2, 2.0 x 2.5 x 1.0 mm).  Numbers/names
# = LCSC C130156 EasyEDA symbol = Infineon DPS310 datasheet pin table.
DPS310_PINS = (
    ("4", "SCK", "input", -12.7, 2.54, 0),
    ("3", "SDI", "bidirectional", -12.7, 0, 0),
    ("2", "CSB", "input", -12.7, -2.54, 0),
    ("5", "SDO", "input", 12.7, 0, 180),       # I2C mode: address-select input (SPI: data out)
    ("8", "VDD", "power_in", -2.54, 10.16, 270),
    ("6", "VDDIO", "power_in", 2.54, 10.16, 270),
    ("1", "GND", "power_in", -2.54, -10.16, 90),
    ("7", "GND", "power_in", 2.54, -10.16, 90),
)
FP_DPS310 = "Package_LGA:Bosch_LGA-8_2x2.5mm_P0.65mm_ClockwisePinNumbering"
DS_DPS310 = ("https://www.infineon.com/dgdl/Infineon-DPS310-DataSheet-v01_02-EN.pdf"
             "?fileId=5546d462576f34750157750826c42242")


def sym_dps310(prefix):
    return _sym_ic(prefix, "DPS310", "U", "DPS310", FP_DPS310, DS_DPS310,
                   "Digital barometric pressure sensor, I2C/SPI, 1.7-3.6 V, PG-VLGA-8-2 2.0x2.5 mm",
                   10.16, 7.62, DPS310_PINS)


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
            sym_crystal(prefix), sym_icm42688(prefix), sym_xo(prefix), sym_dps310(prefix),
            sym_power(prefix, "GND", "gnd"),
            sym_power(prefix, "+3V3", "rail"), sym_power(prefix, "+3.3VA", "rail"),
            sym_pwr_flag(prefix)]


# Pin connection offsets in schematic coordinates (Y down) for each symbol
PIN_OFFS = {
    "R": {"1": (0, -3.81), "2": (0, 3.81)},
    "C": {"1": (0, -3.81), "2": (0, 3.81)},
    "FerriteBead": {"1": (0, -3.81), "2": (0, 3.81)},
    "Crystal_GND24": {"1": (-3.81, 0), "3": (3.81, 0), "2": (0, 5.08), "4": (0, -5.08)},
    "ICM-42688-P": {p[0]: (p[3], -p[4]) for p in ICM_PINS},
    "XO_OE_4p": {p[0]: (p[3], -p[4]) for p in XO_PINS},
    "DPS310": {p[0]: (p[3], -p[4]) for p in DPS310_PINS},
}


# ---------------------------------------------------------------------------
# Schematic builder
# ---------------------------------------------------------------------------
class Sheet:
    def __init__(self):
        self.items = []
        self.pwr_n = 0
        self.flg_n = 0
        self.glabels = set()
        self.refs = []          # BOM references placed (no #PWR/#FLG), for the verifier

    def _instances(self, ref):
        return ('(instances (project %s (path %s (reference %s) (unit 1))))'
                % (q(PROJECT), q("/" + ROOT_UUID), q(ref)))

    def symbol(self, libname, ref, value, x, y, rot=0, footprint="", fields=None,
               pins=("1", "2"), in_bom=True, ref_xy=None, val_xy=None, hide_val=False,
               dnp=False):
        fields = fields or {}
        if not ref.startswith("#"):
            self.refs.append(ref)
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
        # First instance keeps the historical name-only UUID; later instances
        # of the same net (e.g. U1 side + series resistor side) get a
        # position-qualified UUID so no two items share one (Rev 11.0 fix).
        tag = "glbl-%s" % name if name not in self.glabels else "glbl-%s-%s-%s" % (name, f(x), f(y))
        self.glabels.add(name)
        self.items.append(
            "(global_label %s (shape bidirectional) (at %s %s %d) (fields_autoplaced yes) "
            "(effects (font (size 1.27 1.27)) %s) (uuid %s) "
            "(property \"Intersheetrefs\" \"${INTERSHEET_REFS}\" (at %s %s 0) (effects (font (size 1.27 1.27)) hide)))"
            % (q(name), f(x), f(y), rot, just, q(uid(tag)), f(x), f(y)))

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


def passive(sh, libname, ref, value, x, y, net1, net2, footprint, fields, dnp=False, kind="label",
            kind2=None):
    """Vertical two-pin part; pin1 up, pin2 down; stubs 2.54 mm."""
    sh.symbol(libname, ref, value, x, y, footprint=footprint, fields=fields,
              ref_xy=(x + 2.54, y - 1.27), val_xy=(x + 2.54, y + 1.27), dnp=dnp)
    o = PIN_OFFS[libname]
    sh.attach(net1, x + o["1"][0], y + o["1"][1], 0, -2.54, kind)
    sh.attach(net2, x + o["2"][0], y + o["2"][1], 0, 2.54, kind if kind2 is None else kind2)


# Rev 10.0: bidirectional DShot series termination.  hwdef PWM(n) on TIM1_CHx
# -> 100R -> ESCn_DSHOT (global label, ESC MCU on a future sheet).
DSHOT_SERIES = (("R4", "TIM1_CH1", "ESC1_DSHOT"), ("R5", "TIM1_CH2", "ESC2_DSHOT"),
                ("R6", "TIM1_CH3", "ESC3_DSHOT"), ("R7", "TIM1_CH4", "ESC4_DSHOT"))


# Rev 11.0: dual ICM-42688-P.  SPI series tuples: (ref, MCU/hwdef net, IMU net,
# IMU pin).  Pin 1 of each resistor = MCU net, pin 2 = IMU net.
IMU_BLOCKS = (
    {"n": 1, "ref": "U2", "fb": "FB2", "r": "R8", "fb_net": "IMU1_FB", "rail": "+3V3_IMU1",
     "c_bulk": "C17", "c_vdd": "C18", "c_vddio": "C19", "drdy": "IMU1_DRDY",
     "clk_r": "R14", "clk_net": "IMU1_CLKIN", "port": "SPI1",
     "spi": (("R10", "SPI1_SCK", "IMU1_SCLK", "13"), ("R11", "SPI1_MOSI", "IMU1_SDI", "14"),
             ("R12", "IMU1_CS", "IMU1_CS_N", "12"), ("R18", "SPI1_MISO", "IMU1_SDO", "1"))},
    {"n": 2, "ref": "U3", "fb": "FB3", "r": "R9", "fb_net": "IMU2_FB", "rail": "+3V3_IMU2",
     "c_bulk": "C20", "c_vdd": "C21", "c_vddio": "C22", "drdy": "IMU2_DRDY",
     "clk_r": "R15", "clk_net": "IMU2_CLKIN", "port": "SPI4",
     "spi": (("R19", "SPI4_SCK", "IMU2_SCLK", "13"), ("R20", "SPI4_MOSI", "IMU2_SDI", "14"),
             ("R21", "IMU2_CS", "IMU2_CS_N", "12"), ("R22", "SPI4_MISO", "IMU2_SDO", "1"))},
)
IMU_NC_PINS = ("2", "3", "10", "11")      # RESV: "No Connect or Connect to GND"
IMU_CLK = {"xo": "Y2", "src_net": "IMU_CLKIN", "mon_r": "R16", "mon_net": "IMU_CLKIN_MON",
           "oe_r": "R17", "oe_net": "Y2_OE", "c_dec": "C23"}
LCSC_100N = "C1525"

# Rev 12.0: DPS310 barometer on I2C1.  (ref, MCU/hwdef net, baro-pin net, U4 pin)
BARO = {"ref": "U4", "fb": "FB4", "rail": "+3V3_BARO", "c_bulk": "C24", "c_vdd": "C25",
        "c_vddio": "C26", "pullup": (("R23", "I2C1_SCL"), ("R24", "I2C1_SDA")),
        "series": (("R25", "I2C1_SCL", "BARO_SCL", "4"), ("R26", "I2C1_SDA", "BARO_SDA", "3"))}
R2K2 = {"MPN": "0402WGF2201TCE", "Manufacturer": "UNI-ROYAL", "LCSC": "C25879"}
R33 = {"MPN": "0402WGF330JTCE", "Manufacturer": "UNI-ROYAL", "LCSC": "C25105"}


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

    # ---------------- bidirectional DShot series termination (Rev 10.0) ----------------
    dx_, dy_ = 294.64, 91.44
    sh.text("Bidirectional DShot600 series termination (Rev 10.0)\n"
            "100R within 3 mm of U1; no pull-up (ESC bootloader entry)", dx_ - 6.35, dy_ - 20.32, 2.0)
    for i, (ref, mcu_net, esc_net) in enumerate(DSHOT_SERIES):
        passive(sh, "R", ref, "100R", dx_ + i * 25.4, dy_, mcu_net, esc_net, FP_R0402,
                {"MPN": "0402WGF1000TCE", "Manufacturer": "UNI-ROYAL", "LCSC": "C25076",
                 "Spec": "100R 1% 0402 - DShot/GCR source damping, tau = 100R x ~15 pF = 1.5 ns << 1.33 us GCR bit"},
                kind="global")

    # ---------------- dual ICM-42688-P IMU block (Rev 11.0) ----------------
    imu_block(sh, hw)

    # ---------------- DPS310 barometer on I2C1 (Rev 12.0) ----------------
    baro_block(sh, hw)

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
    # Rev 11.0: the filtered IMU rails are fed through FB+R (passives), so
    # tell ERC where their power comes from.
    for i, blk in enumerate(IMU_BLOCKS):
        x = fx + (3 + i) * 15.24 + 5.08
        sh.pwr_flag(x, fy)
        sh.wire(x, fy, x, fy + 5.08)
        sh.label(blk["rail"], x, fy + 5.08)
    # Rev 12.0: +3V3_BARO is fed through FB4 (passive)
    x = fx + 5 * 15.24 + 5.08
    sh.pwr_flag(x, fy)
    sh.wire(x, fy, x, fy + 5.08)
    sh.label(BARO["rail"], x, fy + 5.08)

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


def imu_block(sh, hw):
    """Rev 11.0 dual ICM-42688-P block in the lower sheet area (A2)."""
    for blk in IMU_BLOCKS:
        for _, mcu_net, _, _ in blk["spi"]:
            if mcu_net not in {net_for_hwdef(v) for v in hw.values()}:
                raise SystemExit("IMU block: %s not assigned in hwdef.dat" % mcu_net)
    for net in ("IMU1_DRDY", "IMU2_DRDY", IMU_CLK["mon_net"]):
        if net not in hw.values():
            raise SystemExit("IMU block: %s not assigned in hwdef.dat" % net)

    bx, by0 = 25.4, 252.73
    sh.text("Dual ICM-42688-P IMUs (Rev 11.0): per-IMU supply anti-alias filter, 33R SPI source termination",
            bx, by0, 2.0)
    for i, blk in enumerate(IMU_BLOCKS):
        n = blk["n"]
        ux, uy = 76.2, 289.56 + i * 71.12
        sh.symbol("ICM-42688-P", blk["ref"], "ICM-42688-P", ux, uy, footprint=FP_ICM,
                  fields={"MPN": "ICM-42688-P", "Manufacturer": "TDK InvenSense", "LCSC": "C1850418",
                          "Spec": "IMU%d; pinout DS-000347 rev 1.2 sec 4.1; RESV 7 -> GND, RESV 2/3/10/11 NC" % n},
                  pins=tuple(p[0] for p in ICM_PINS),
                  ref_xy=(ux - 25.4, uy - 13.97), val_xy=(ux + 7.62, uy + 16.51))
        o = PIN_OFFS["ICM-42688-P"]

        def pt(num):
            return ux + o[num][0], uy + o[num][1]
        imu_net = {num: net for _, _, net, num in blk["spi"]}
        for num in ("13", "14", "12", "1"):
            sh.attach(imu_net[num], *pt(num), -5.08, 0)
        sh.attach(blk["drdy"], *pt("4"), 5.08, 0, "global")
        sh.attach(blk["clk_net"], *pt("9"), 5.08, 0)
        for num in IMU_NC_PINS:
            sh.noconnect(*pt(num))
        # VDD (8) + VDDIO (5) -> filtered rail; GND (6) + RESV (7) -> GND
        (x8, y8), (x5, y5) = pt("8"), pt("5")
        sh.wire(x8, y8, x8, y8 - 2.54)
        sh.wire(x5, y5, x5, y5 - 2.54)
        sh.wire(x8, y8 - 2.54, x5, y5 - 2.54)
        sh.label(blk["rail"], x5, y5 - 2.54)
        (x6, y6), (x7, y7) = pt("6"), pt("7")
        sh.wire(x6, y6, x6, y6 + 2.54)
        sh.wire(x7, y7, x7, y7 + 2.54)
        sh.wire(x6, y6 + 2.54, x7, y7 + 2.54)
        sh.power("GND", x6, y6 + 2.54, 0)

        # supply anti-alias filter: +3V3 -> FB -> IMUn_FB -> 22R -> +3V3_IMUn
        px = 139.7
        passive(sh, "FerriteBead", blk["fb"], "600R@100MHz", px, uy, "+3V3", blk["fb_net"], FP_L0402,
                {"MPN": "BLM15AX601SN1D", "Manufacturer": "Murata", "LCSC": "C76886",
                 "Spec": "600R@100MHz, Rdc 0.34R max - IMU%d supply filter (same part as FB1)" % n})
        passive(sh, "R", blk["r"], "22R", px + 20.32, uy, blk["fb_net"], blk["rail"], FP_R0402,
                {"MPN": "0402WGF220JTCE", "Manufacturer": "UNI-ROYAL", "LCSC": "C25092",
                 "Spec": "22R 1%% 0402 - IMU%d RC/LC damping; 22R x 10uF = 220 us" % n})
        passive(sh, "C", blk["c_bulk"], "10uF", px + 35.56, uy, blk["rail"], "GND", FP_C0603,
                {"MPN": "CL10A106KP8NNNC", "Manufacturer": "Samsung", "LCSC": "C19702",
                 "Spec": "10 uF X5R 10V 0603 - +3V3_IMU%d bulk" % n})
        passive(sh, "C", blk["c_vdd"], "100nF", px + 50.8, uy, blk["rail"], "GND", FP_C0402,
                {"Spec": "100 nF X7R 16V 0402 - at %s pin 8 VDD" % blk["ref"], "LCSC": LCSC_100N})
        passive(sh, "C", blk["c_vddio"], "100nF", px + 66.04, uy, blk["rail"], "GND", FP_C0402,
                {"Spec": "100 nF X7R 16V 0402 - at %s pin 5 VDDIO" % blk["ref"], "LCSC": LCSC_100N})
        # SPI source termination: pin 1 = MCU (hwdef global label), pin 2 = IMU
        sx = 223.52
        for j, (ref, mcu_net, inet, _) in enumerate(blk["spi"]):
            at_imu = mcu_net.endswith("MISO")
            passive(sh, "R", ref, "33R", sx + j * 20.32, uy, mcu_net, inet, FP_R0402,
                    dict(R33, Spec="33R 1%% 0402 - %s source termination, place at %s" % (
                        mcu_net, ("%s pin 1 (IMU drives SDO)" % blk["ref"]) if at_imu else "U1 pin")),
                    kind="global", kind2="label")

    # ---------------- shared 32.768 kHz CLKIN ----------------
    cx, cy = 330.2, 289.56
    sh.text("Shared IMU CLKIN 32.768 kHz, RTC mode (Rev 11.0)\n"
            "star from Y2: 33R per IMU branch, 1k to MCU monitor", cx - 12.7, by0, 2.0)
    sh.symbol("XO_OE_4p", IMU_CLK["xo"], "32.768kHz CMOS", cx, cy, footprint=FP_XO2520,
              fields={"MPN": "SX2M32.768KM20F30TNN", "Manufacturer": "SCTF", "LCSC": "C2901603",
                      "Spec": "32.768 kHz CMOS XO, +/-30 ppm stability, 1.62-3.63 V, 90 uA (LCSC C2901603)"},
              pins=("1", "2", "3", "4"), ref_xy=(cx + 2.54, cy - 11.43), val_xy=(cx + 2.54, cy + 11.43))
    o = PIN_OFFS["XO_OE_4p"]
    sh.attach(IMU_CLK["oe_net"], cx + o["1"][0], cy + o["1"][1], -5.08, 0)
    sh.attach(IMU_CLK["src_net"], cx + o["3"][0], cy + o["3"][1], 5.08, 0)
    sh.attach("+3V3", cx + o["4"][0], cy + o["4"][1], 0, -2.54)
    sh.attach("GND", cx + o["2"][0], cy + o["2"][1], 0, 2.54)
    passive(sh, "C", IMU_CLK["c_dec"], "100nF", cx + 30.48, cy, "+3V3", "GND", FP_C0402,
            {"Spec": "100 nF X7R 16V 0402 - at Y2 pin 4 VDD", "LCSC": LCSC_100N})
    passive(sh, "R", IMU_CLK["oe_r"], "10k", cx + 45.72, cy, "+3V3", IMU_CLK["oe_net"], FP_R0402,
            {"Spec": "10k 1% 0402 - Y2 OE strapped high (no MCU control)", "LCSC": "C25744"})
    ry = 360.68
    for j, blk in enumerate(IMU_BLOCKS):
        passive(sh, "R", blk["clk_r"], "33R", cx + j * 20.32, ry, IMU_CLK["src_net"], blk["clk_net"], FP_R0402,
                dict(R33, Spec="33R 1%% 0402 - CLKIN branch to %s pin 9, place at Y2" % blk["ref"]))
    passive(sh, "R", IMU_CLK["mon_r"], "1k", cx + 40.64, ry, IMU_CLK["src_net"], IMU_CLK["mon_net"], FP_R0402,
            {"MPN": "0402WGF1001TCE", "Manufacturer": "UNI-ROYAL", "LCSC": "C11702",
             "Spec": "1k 1% 0402 - isolates MCU PD12 clock monitor from the IMU CLKIN star"},
            kind="label", kind2="global")

    notes = (
        "IMU BLOCK (Rev 11.0)\n"
        "- ICM-42688-P pinout: TDK DS-000347 rev 1.2 sec 4.1 (LCSC C1850418). RESV 7 -> GND (datasheet:\n"
        "  'Connect to GND'); RESV 2/3/10/11 'No Connect or Connect to GND' -> NC.\n"
        "- CLKIN: datasheet range 31-50 kHz (typ 32). RTC_MODE = INTF_CONFIG1 (bank0 0x4D) bit2,\n"
        "  PIN9_FUNCTION = INTF_CONFIG5 (bank1 0x7B) bits[2:1] = 0b10. ODR scales f_CLKIN/32 kHz\n"
        "  (8 kHz -> 8192 Hz). Dead Y2 stalls BOTH IMUs: firmware must watch IMU_CLKIN_MON (PD12)\n"
        "  and fall back to the internal oscillator (RTC_MODE=0).\n"
        "- Supply filter per IMU: FB (600R@100MHz) + 22R + 10uF, 100nF at VDD and at VDDIO.\n"
        "  Datasheet sec 4.2 reference: VDD 0.1uF + 2.2uF, VDDIO 10nF.\n"
        "LAYOUT RULES\n"
        "- R10-R12 / R19-R21 within 3 mm of U1; R18 / R22 within 3 mm of the IMU SDO pin.\n"
        "- R14/R15/R16 at Y2 (star); C18/C19 (C21/C22) within 1 mm of IMU pins 8/5.\n"
        "- No ESC switch-node copper or motor return current under U2/U3/Y2.\n"
    )
    sh.text(notes, 431.8, 252.73)


BARO_I2C_REPORT = os.path.join(REPO, "analysis", "baro_i2c_report.json")


def i2c_pullup_window(path=BARO_I2C_REPORT):
    """I2C1 pull-up window for the sheet note, taken from the E3 gate data of
    analysis/baro_i2c_report.json (VOL 0.4 V at 3 mA with VDD max, tr 300 ns).
    Falls back to the same worst-case formulas if the report is absent."""
    vdd_max, vol, iol, rp = 3.465, 0.4, 3e-3, 2200.0
    rp_min = (vdd_max - vol) / iol
    rp_min_4k7 = rp_max = None
    if os.path.exists(path):
        import json
        rep = json.load(open(path))
        a = rep.get("assumptions", {})
        vdd_max = a.get("vdd", {}).get("max", vdd_max)
        vol = a.get("um10204_fm", {}).get("VOL_max_V_at_3mA", vol)
        w = rep.get("bus", {}).get("Rp_window")
        if w:
            rp_min = w["Rp_min_ohm_VOL_0V4_3mA"]
            rp_min_4k7 = w["Rp_min_ohm_with_4k7_module"]
            rp_max = w["Rp_max_ohm_tr300_at_Cmax"]
            rp = w.get("baseline_Rp_ohm", rp)
    if rp_min_4k7 is None:                      # report absent: 4k7 || Rp at its own VOL limit
        rp_min_4k7 = 1.0 / (1.0 / rp_min - 1.0 / 4700.0)
    if rp_max is None:
        rp_max = float("nan")
    if rp_min >= rp:
        raise SystemExit("I2C1: on-board %.0f R already below the VOL limit %.0f R" % (rp, rp_min))
    return {"vdd_max": vdd_max, "vol": vol, "rp": rp, "rp_min": rp_min, "rp_min_4k7": rp_min_4k7,
            "rp_max": rp_max, "module_min": 1.0 / (1.0 / rp_min - 1.0 / rp),
            "cb_max_pf": 300e-9 / (0.8473 * rp) * 1e12}


def baro_block(sh, hw):
    """Rev 12.0 DPS310 barometer on I2C1, upper-right sheet area."""
    funcs = {net_for_hwdef(v): k for k, v in hw.items()}
    for net, port in (("I2C1_SCL", "PB6"), ("I2C1_SDA", "PB7")):
        if funcs.get(net) != port:
            raise SystemExit("baro block: %s must be on %s in hwdef.dat (got %s)" % (net, port, funcs.get(net)))
    if not re.search(r"^BARO\s+DPS310\s+I2C:0:0x76\b", open(HWDEF).read(), re.M):
        raise SystemExit("baro block: hwdef.dat lacks 'BARO DPS310 I2C:0:0x76' (SDO strap would be wrong)")

    bx, by = 444.5, 25.4
    sh.text("DPS310 barometer on I2C1 400 kHz (Rev 12.0)\n"
            "I2C mode: CSB -> VDDIO, SDO -> GND => addr 0x76", bx, by, 2.0)
    ux, uy = 477.52, 68.58
    sh.symbol("DPS310", BARO["ref"], "DPS310", ux, uy, footprint=FP_DPS310,
              fields={"MPN": "DPS310XTSA1", "Manufacturer": "Infineon Technologies", "LCSC": "C130156",
                      "Spec": "Baro, I2C addr 0x76 (SDO=GND), CSB=VDDIO -> I2C; pinout LCSC C130156 / "
                              "DPS310 DS v01_02; alt LCSC C3232509"},
              pins=tuple(p[0] for p in DPS310_PINS),
              ref_xy=(ux - 10.16, uy - 8.89), val_xy=(ux + 3.81, uy + 8.89))
    o = PIN_OFFS["DPS310"]

    def pt(num):
        return ux + o[num][0], uy + o[num][1]
    for _, _, bnet, num in BARO["series"]:
        sh.attach(bnet, *pt(num), -5.08, 0)
    sh.attach(BARO["rail"], *pt("2"), -5.08, 0)          # CSB high -> I2C
    sh.attach("GND", *pt("5"), 5.08, 0)                  # SDO low -> 0x76
    (x8, y8), (x6, y6) = pt("8"), pt("6")
    sh.wire(x8, y8, x8, y8 - 2.54)
    sh.wire(x6, y6, x6, y6 - 2.54)
    sh.wire(x8, y8 - 2.54, x6, y6 - 2.54)
    sh.label(BARO["rail"], x6, y6 - 2.54)
    (x1, y1), (x7, y7) = pt("1"), pt("7")
    sh.wire(x1, y1, x1, y1 + 2.54)
    sh.wire(x7, y7, x7, y7 + 2.54)
    sh.wire(x1, y1 + 2.54, x7, y7 + 2.54)
    sh.power("GND", x1, y1 + 2.54, 0)

    # supply: +3V3 -> FB4 -> +3V3_BARO, 1u + 100n at VDD, 100n at VDDIO
    py = 106.68
    passive(sh, "FerriteBead", BARO["fb"], "600R@100MHz", bx + 5.08, py, "+3V3", BARO["rail"], FP_L0402,
            {"MPN": "BLM15AX601SN1D", "Manufacturer": "Murata", "LCSC": "C76886",
             "Spec": "600R@100MHz, Rdc 0.34R max - DPS310 supply filter (same part as FB1-FB3)"})
    passive(sh, "C", BARO["c_bulk"], "1uF", bx + 30.48, py, BARO["rail"], "GND", FP_C0402,
            {"Spec": "1 uF X5R 25V 0402 - +3V3_BARO bulk at U4 pin 8 VDD", "MPN": "CL05A105KA5NQNC",
             "Manufacturer": "Samsung", "LCSC": "C52923"})
    passive(sh, "C", BARO["c_vdd"], "100nF", bx + 45.72, py, BARO["rail"], "GND", FP_C0402,
            {"Spec": "100 nF X7R 16V 0402 - at U4 pin 8 VDD", "LCSC": LCSC_100N})
    passive(sh, "C", BARO["c_vddio"], "100nF", bx + 60.96, py, BARO["rail"], "GND", FP_C0402,
            {"Spec": "100 nF X7R 16V 0402 - at U4 pin 6 VDDIO", "LCSC": LCSC_100N})

    # I2C1: 2.2k pull-ups at U1 (pin 1 = +3V3), 33R series at U4 (pin 1 = MCU net)
    ry = 134.62
    for j, (ref, mnet) in enumerate(BARO["pullup"]):
        passive(sh, "R", ref, "2.2k", bx + 5.08 + j * 20.32, ry, "+3V3", mnet, FP_R0402,
                dict(R2K2, Spec="2.2k 1%% 0402 - %s pull-up to +3V3, place at U1 (only pull-up on I2C1)" % mnet),
                kind2="global")
    for j, (ref, mnet, bnet, num) in enumerate(BARO["series"]):
        passive(sh, "R", ref, "33R", bx + 50.8 + j * 20.32, ry, mnet, bnet, FP_R0402,
                dict(R33, Spec="33R 1%% 0402 - %s series, place at U4 pin %s" % (mnet, num)),
                kind="global", kind2="label")

    rw = i2c_pullup_window()
    notes = (
        "BARO BLOCK (Rev 12.0)\n"
        "- U4 DPS310 pinout: LCSC C130156 / Infineon DS v01_02: 1 GND, 2 CSB, 3 SDI, 4 SCK, 5 SDO, 6 VDDIO, 7 GND, 8 VDD.\n"
        "  CSB tied to VDDIO selects I2C; SDO to GND = 0x76 = hwdef 'BARO DPS310 I2C:0:0x76'.\n"
        "- I2C1 PB6 SCL / PB7 SDA, 400 kHz Fm. The ONLY on-board pull-ups on I2C1 are R23/R24 %.1fk; the\n"
        "  GPS/compass module must not add strong pull-ups: combined Rp >= %.0fR for 3 mA at VOL 0.4 V with\n"
        "  VDD max %.3f V => module pull-up >= %.2fk (none/4k7/10k OK; 4k7 module -> E3 window %.0f..%.0f R).\n"
        "  %.1fk meets Fm tr <= 300 ns (0.8473*Rp*Cb) up to Cb = %.0f pF incl. the GPS/compass cable.\n"
        "- Supply: FB4 + C24 1uF + C25 100nF (VDD) + C26 100nF (VDDIO); DS reference 100 nF per supply pin.\n"
        "LAYOUT RULES\n"
        "- U4 on the face/corner farthest from ESC switch nodes and hot FETs: >= 5 mm from any phase node or\n"
        "  MOSFET copper, no motor/battery current under U4 on any layer; not next to the LDO/regulators.\n"
        "- Keep-out under U4: no vias, no traces on the U4 layer between pads, solid GND on the adjacent plane.\n"
        "- Shield U4 from light (opaque foam/cover); vent hole of U4 faces into the foam chamber.\n"
        "- Vent position and keep-out must match the acoustic foam chamber in cad/ (open-cell foam over U4,\n"
        "  no direct prop-wash path; chamber vent away from the FC fan/airflow).\n"
        "- I2C1 SCL/SDA routed on a layer referenced to In1/In4 GND, adjacent pair, no plane splits;\n"
        "  R25/R26 within 2 mm of U4 pins 4/3, R23/R24 within 3 mm of U1 pins 92/93.\n"
        "- C25/C26 within 1 mm of U4 pins 8/6; GND pins 1/7 straight to a GND via.\n"
    ) % (rw["rp"] / 1e3, rw["rp_min"], rw["vdd_max"], rw["module_min"] / 1e3, rw["rp_min_4k7"], rw["rp_max"],
         rw["rp"] / 1e3, rw["cb_max_pf"])
    sh.text(notes, 444.5, 152.4)


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
        "(paper \"A2\")",
        "(title_block (title \"Nano H743 AIO - MCU core: HSE, VDDA filter, dual IMU, baro\")",
        "  (date \"2026-11-06\") (rev \"12.0\") (company \"Sreesai Arjun Engineering\")",
        "  (comment 1 \"STM32H743VIT6 480 MHz: HSE 24 MHz, PLL1 M3 N120 P2\")",
        "  (comment 2 \"Y1 CL 12 pF; VDDA FB1+R2 2R2; U2/U3 ICM-42688-P, Y2 32.768 kHz CLKIN; U4 DPS310 I2C1 0x76\")",
        "  (comment 3 \"Generated: tools/gen_mcu_core_sch.py + hwdef.dat\")",
        "  (comment 4 \"Production Revision 12.0 / 150\"))",
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
