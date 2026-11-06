#!/usr/bin/env python3
"""
Nano H743 AIO - MCU core schematic generator (Revision 7.0, DShot termination
Rev 10.0, dual-IMU block Rev 11.0, DPS310 baro Rev 12.0, AT7456E OSD + 9/12 V
VTX BEC + DJI VTX connector Rev 13.0)

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
  * Rev 13.0: AT7456E analog OSD on SPI2 + 9 V / 12 V 2 A VTX BEC + DJI VTX
    connector (hwdef: SPIDEV osd SPI2 8*MHZ, PD10 VTX_PWR GPIO(81) HIGH,
    PD11 VTX_12V_SEL GPIO(82) LOW, PC5 VTX_VSENSE ADC1 INP8, PE7 UART7_RX).
      - U5 AT7456E (ZHONGKEWEI, LCSC C82351).  Pinout AT7456E-S402-V1.1 p.6
        (= LCSC symbol): 1/2/14/15/16/27/28 NC, 3 DVDD, 4 DGND, 5 CLKIN,
        6 XFB, 7 CLKOUT, 8 /CS, 9 SDIN, 10 SCLK, 11 SDOUT, 12 LOS, 13 SYNC_IN,
        17 /VSYNC, 18 /HSYNC, 19 /RESET, 20 AGND, 21 AVDD, 22 VIN, 23 PGND,
        24 PVDD, 25 SAG, 26 VOUT, 29 EP (= AGND).  +3V3 -> FB5 -> +3V3_OSD on
        DVDD/AVDD/PVDD (3.15-5.25 V, DS p.2); C27 10uF + C28/C29/C30 100nF.
        Y3 27 MHz (YXC X322527MOB4SI, C70595, CL 12 pF) CLKIN-XFB, no load
        caps (DS Fig.2).  R29 10k /RESET pull-up.  33R source termination
        R30 SCK / R31 MOSI / R32 CS at U1, R33 MISO at U5 (pin 1 = hwdef net).
        Video (DS Fig.2, 3.3 V): VIDEO_IN -> R27 75R to GND, C31 100n -> VIN;
        VOUT -> C32 22u, SAG -> C33 22u -> OSD_VOUT_AC -> R28 75R -> VIDEO_OUT.
        LOS, /VSYNC, /HSYNC, SYNC_IN, CLKOUT unused (AP_OSD_MAX7456 polls STAT).
        Footprint: DS p.38 HTSSOP28 EP is D1 6.20 REF x E2 2.75 REF, so the
        stock Package_SO:HTSSOP-28-1EP_4.4x9.7mm_P0.65mm_EP2.75x6.2mm (exact
        EP match) is used, not the EasyEDA 6.7 x 2.9 mm generic EP.
      - U6 TPS62933DRLR (TI SLUSEA4D, C3200405, SOT-583-8: 1 RT, 2 EN, 3 VIN,
        4 GND, 5 SW, 6 BST, 7 SS, 8 FB).  VBAT -> C34/C35 10u 50V + C36 100n
        -> VIN; RT NC = 500 kHz; SS C38 33 nF (4.8 ms); BST C37 100 nF;
        L1 10 uH DMBJ PNLS6045-100M (C2849537) -> +VTX with C39-C41 22u 25V
        + C42 100n.  FB: R34 102k / R35 10.0k (8.96 V), R36 26.7k switched in
        parallel by Q1 2N7002 (C8545; 1 G 2 S 3 D) -> 12.02 V.  Q1 gate: R37 1k
        from VTX_12V_SEL + R38 100k pull-down (high-Z -> 9 V).  EN: VTX_PWR ->
        R39 1k, no pull-down (internal pull-up -> VTX on during boot).  Sense:
        R40 10k / R41 1k (x11.0 = defaults.parm BATT2_VOLT_MULT) + C43 100n.
        Footprint KiCad 8 Package_TO_SOT_SMD:SOT-583-8 (drawn from SLUSEA4D).
      - J1 JST SM06B-SRSS-TB (C160405), DJI FC-side order 1 +VTX, 2 GND,
        3 VTX_TXD (R42 100R from USART1_TX), 4 VTX_RXD (R43 100R to
        USART1_RX), 5 GND, 6 VTX_SBUS (R44 1k to UART7_RX).  J2 = +VTX/GND
        solder pair for > 1 A loads (JST SH is 1 A per contact), no part
        placed (Assembly field), footprint Connector_Wire 0.25 mm^2 x2.
      - New refs: U5, U6, Y3, FB5, L1, Q1, J1, J2, C27-C43, R27-R44 (R13
        stays unused).  Off-sheet global labels VIDEO_IN, VIDEO_OUT, VBAT;
        PWR_FLAGs on +3V3_OSD, +VTX, VBAT.
  * Rev 14.0 (sheet grown A2 -> A1; block in the new right-hand column,
    tools/sch_can_block.py): DroneCAN / CAN FD PHY on FDCAN1.
      U7 TCAN334GDCNR (LCSC C191005, 5 Mbit/s G grade; the non-G TCAN334 is
      1 Mbit/s): TXD <- CAN1_TX (PD1), RXD -> CAN1_RX (PD0), STB <-
      GPIO_CAN1_SILENT (PD3, integrated pull-DOWN), SHDN -> GND; C44 100 nF +
      C45 1 uF at VCC.  Split termination JP1 - R45 60.4R - CAN1_SPLIT -
      R46 60.4R - JP2 with C46 4.7 nF C0G to GND (JP1/JP2 bridged solder
      jumpers, cut both to un-terminate).  D4 PESD2CANFD24V-T (LCSC C552486)
      at J3 JST-GH SM04B-GHS-TB (1 +5V, 2 CANH, 3 CANL, 4 GND, MP GND).
      Values/limits from analysis/can_phy_analysis.py.
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
import sys
import uuid

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sch_can_block as can  # noqa: E402  (Rev 14.0 DroneCAN block)

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


# Rev 13.0: AT7456E analog OSD (HTSSOP-28 + exposed pad).  Numbers/names =
# AT7456E-S402-V1.1 datasheet p.6 pin table = LCSC C82351 EasyEDA symbol
# (CS#/VSYNC#/HSYNC#/RESET# written with KiCad overbars).  EP (29) is
# internally AGND (DS p.6 "EP" row).  Body 30.48 x 30.48 so the vertical
# pin names never touch the side pin names.
AT7456E_PINS = (
    ("8", "~{CS}", "input", -17.78, 7.62, 0),
    ("10", "SCLK", "input", -17.78, 5.08, 0),
    ("9", "SDIN", "input", -17.78, 2.54, 0),
    ("11", "SDOUT", "tri_state", -17.78, 0, 0),          # Hi-Z while /CS high (DS p.6)
    ("19", "~{RESET}", "input", -17.78, -2.54, 0),
    ("5", "CLKIN", "input", -17.78, -5.08, 0),
    ("6", "XFB", "passive", -17.78, -7.62, 0),           # crystal feedback node
    ("7", "CLKOUT", "output", -17.78, -10.16, 0),
    ("22", "VIN", "input", 17.78, 7.62, 180),
    ("26", "VOUT", "output", 17.78, 5.08, 180),
    ("25", "SAG", "input", 17.78, 2.54, 180),
    ("12", "LOS", "open_collector", 17.78, -2.54, 180),
    ("17", "~{VSYNC}", "open_collector", 17.78, -5.08, 180),
    ("18", "~{HSYNC}", "open_collector", 17.78, -7.62, 180),
    ("13", "SYNC_IN", "input", 17.78, -10.16, 180),
    ("1", "NC", "passive", -10.16, 17.78, 270),
    ("2", "NC", "passive", -7.62, 17.78, 270),
    ("3", "DVDD", "power_in", -2.54, 17.78, 270),
    ("21", "AVDD", "power_in", 2.54, 17.78, 270),
    ("24", "PVDD", "power_in", 7.62, 17.78, 270),
    ("14", "NC", "passive", -10.16, -17.78, 90),
    ("15", "NC", "passive", -7.62, -17.78, 90),
    ("16", "NC", "passive", -5.08, -17.78, 90),
    ("4", "DGND", "power_in", -2.54, -17.78, 90),
    ("20", "AGND", "power_in", 0, -17.78, 90),
    ("23", "PGND", "power_in", 2.54, -17.78, 90),
    ("29", "EP", "power_in", 5.08, -17.78, 90),
    ("27", "NC", "passive", 7.62, -17.78, 90),
    ("28", "NC", "passive", 10.16, -17.78, 90),
)
# DS p.38 HTSSOP28 drawing: exposed pad D1 6.20 REF x E2 2.75 REF, body
# 9.70 x 4.40, e 0.65 -> the stock KiCad land pattern with the SAME exposed
# pad (the EasyEDA C82351 footprint's 6.7 x 2.9 EP is a generic oversize).
FP_AT7456E = "Package_SO:HTSSOP-28-1EP_4.4x9.7mm_P0.65mm_EP2.75x6.2mm"
DS_AT7456E = "https://www.lcsc.com/datasheet/lcsc_datasheet_1810301815_ZHONGKEWEI-AT7456E_C82351.pdf"

# Rev 13.0: TI TPS62933 (SOT-583 DRL-8), SLUSEA4D Table 7-1 = LCSC C3200405
# symbol: 1 RT, 2 EN, 3 VIN, 4 GND, 5 SW, 6 BST, 7 SS, 8 FB.
TPS62933_PINS = (
    ("3", "VIN", "power_in", -10.16, 5.08, 0),
    ("2", "EN", "input", -10.16, 0, 0),
    ("1", "RT", "input", -10.16, -2.54, 0),
    ("7", "SS", "passive", -10.16, -5.08, 0),
    ("6", "BST", "passive", 10.16, 5.08, 180),
    ("5", "SW", "power_out", 10.16, 2.54, 180),
    ("8", "FB", "input", 10.16, -2.54, 180),
    ("4", "GND", "power_in", 0, -10.16, 90),
)
# KiCad 8 Package_TO_SOT_SMD:SOT-583-8 is drawn from SLUSEA4D (its descr is
# the TPS62933 datasheet): 8 x 0.67 x 0.3 mm pads, 0.5 mm pitch, 1.48 mm row
# span = the DRL0008A "LAND PATTERN EXAMPLE".
FP_TPS62933 = "Package_TO_SOT_SMD:SOT-583-8"
DS_TPS62933 = "https://www.ti.com/lit/ds/symlink/tps62933.pdf"

# 2N7002 SOT-23, LCSC C8545 symbol: 1 G, 2 S, 3 D
NMOS_PINS = (
    ("1", "G", "passive", -5.08, 0, 0),
    ("3", "D", "passive", 2.54, 5.08, 270),
    ("2", "S", "passive", 2.54, -5.08, 90),
)
FP_SOT23 = "Package_TO_SOT_SMD:SOT-23"
# JST SH 1.0 mm 6-pin side entry (SM06B-SRSS-TB, LCSC C160405)
FP_JST_SH6 = "Connector_JST:JST_SH_SM06B-SRSS-TB_1x06-1MP_P1.00mm_Horizontal"
# DMBJ PNLS6045 (6.0 x 6.0 x 4.5 mm, terminals 1.65 max x 5.0 max per the
# DMBJ PNLS datasheet sec. 4 table) -> stock 6045 land pattern 2 x 1.7 x 5.7
# mm at +/-2.25 mm (KiCad 8 has no DMBJ footprint; APV ANR6045 = same body).
FP_L6045 = "Inductor_SMD:L_APV_ANR6045"
FP_C0805 = "Capacitor_SMD:C_0805_2012Metric"
FP_C1206 = "Capacitor_SMD:C_1206_3216Metric"
# J2: two plated through-holes for a soldered 0.25 mm^2 (AWG 24) VTX power pair
FP_WIRE2 = "Connector_Wire:SolderWire-0.25sqmm_1x02_P4.2mm_D0.65mm_OD1.7mm"


def sym_at7456e(prefix):
    return _sym_ic(prefix, "AT7456E", "U", "AT7456E", FP_AT7456E, DS_AT7456E,
                   "Single-channel monochrome OSD with EEPROM (MAX7456 compatible), SPI, 3.15-5.25 V, HTSSOP-28-EP",
                   15.24, 15.24, AT7456E_PINS)


def sym_tps62933(prefix):
    return _sym_ic(prefix, "TPS62933", "U", "TPS62933", FP_TPS62933, DS_TPS62933,
                   "3.8-30 V 3 A synchronous buck, Vref 0.8 V, RT float = 500 kHz, SOT-583-8",
                   7.62, 7.62, TPS62933_PINS)


def sym_l(prefix):
    g = " ".join(
        "(arc (start 0 %s) (mid 0.635 %s) (end 0 %s) (stroke (width 0) (type default)) (fill (type none)))"
        % (f(2.54 - i * 1.27), f(1.905 - i * 1.27), f(1.27 - i * 1.27)) for i in range(4))
    return sym_two_pin(prefix, "L", "L", "Inductor", g)


def sym_nmos(prefix):
    n = "Q_NMOS_GSD"
    g = " ".join([
        "(polyline (pts (xy -2.54 0) (xy -1.778 0)) (stroke (width 0) (type default)) (fill (type none)))",
        "(polyline (pts (xy -1.778 -1.905) (xy -1.778 1.905)) (stroke (width 0.254) (type default)) (fill (type none)))",
        "(polyline (pts (xy -1.27 -2.032) (xy -1.27 2.032)) (stroke (width 0.254) (type default)) (fill (type none)))",
        "(polyline (pts (xy -1.27 1.524) (xy 2.54 1.524) (xy 2.54 2.54)) (stroke (width 0) (type default)) (fill (type none)))",
        "(polyline (pts (xy -1.27 -1.524) (xy 2.54 -1.524) (xy 2.54 -2.54)) (stroke (width 0) (type default)) (fill (type none)))",
        "(polyline (pts (xy -1.27 0) (xy 0.762 0) (xy 0.762 -1.524)) (stroke (width 0) (type default)) (fill (type none)))",
        "(polyline (pts (xy -1.016 0) (xy -0.254 0.508) (xy -0.254 -0.508) (xy -1.016 0)) (stroke (width 0) (type default)) (fill (type outline)))",
        "(circle (center 0.635 0) (radius 2.794) (stroke (width 0.254) (type default)) (fill (type none)))",
    ])
    pins = " ".join(pin_s(t, x, y, r, 2.54, nm, num) for (num, nm, t, x, y, r) in NMOS_PINS)
    return "\n".join([
        "(symbol %s (pin_names (offset 0) hide) (exclude_from_sim no) (in_bom yes) (on_board yes)" % q(prefix + n),
        prop("Reference", "Q", 5.08, 1.27),
        prop("Value", n, 5.08, -1.27),
        prop("Footprint", "", 0, 0, hide=True),
        prop("Datasheet", "~", 0, 0, hide=True),
        prop("Description", "N-channel MOSFET, pins 1 G, 2 S, 3 D (SOT-23)", 0, 0, hide=True),
        "(symbol %s %s)" % (q(n + "_0_1"), g),
        "(symbol %s %s)" % (q(n + "_1_1"), pins),
        ")"])


def sym_conn(prefix, npins):
    """Generic single-row connector, pin 1 at the top, pins on the left."""
    n = "Conn_01x%02d" % npins
    bot = -(npins - 1) * 2.54 - 1.27
    g = ("(rectangle (start -1.27 1.27) (end 1.27 %s) (stroke (width 0.254) (type default)) "
         "(fill (type background)))" % f(bot))
    pins = " ".join(pin_s("passive", -5.08, -i * 2.54, 0, 3.81, "Pin_%d" % (i + 1), str(i + 1))
                    for i in range(npins))
    return "\n".join([
        "(symbol %s (pin_names (offset 1.016) hide) (exclude_from_sim no) (in_bom yes) (on_board yes)" % q(prefix + n),
        prop("Reference", "J", 0, 2.54),
        prop("Value", n, 0, bot - 1.27),
        prop("Footprint", "", 0, 0, hide=True),
        prop("Datasheet", "~", 0, 0, hide=True),
        prop("Description", "Generic connector, single row, 01x%02d" % npins, 0, 0, hide=True),
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
            sym_crystal(prefix), sym_icm42688(prefix), sym_xo(prefix), sym_dps310(prefix),
            sym_at7456e(prefix), sym_tps62933(prefix), sym_l(prefix), sym_nmos(prefix),
            sym_conn(prefix, 6), sym_conn(prefix, 2),
            sym_power(prefix, "GND", "gnd"),
            sym_power(prefix, "+3V3", "rail"), sym_power(prefix, "+3.3VA", "rail"),
            sym_pwr_flag(prefix)] + can.symbols(sys.modules[__name__], prefix)


# Pin connection offsets in schematic coordinates (Y down) for each symbol
PIN_OFFS = {
    "R": {"1": (0, -3.81), "2": (0, 3.81)},
    "C": {"1": (0, -3.81), "2": (0, 3.81)},
    "FerriteBead": {"1": (0, -3.81), "2": (0, 3.81)},
    "Crystal_GND24": {"1": (-3.81, 0), "3": (3.81, 0), "2": (0, 5.08), "4": (0, -5.08)},
    "ICM-42688-P": {p[0]: (p[3], -p[4]) for p in ICM_PINS},
    "XO_OE_4p": {p[0]: (p[3], -p[4]) for p in XO_PINS},
    "DPS310": {p[0]: (p[3], -p[4]) for p in DPS310_PINS},
    "AT7456E": {p[0]: (p[3], -p[4]) for p in AT7456E_PINS},
    "TPS62933": {p[0]: (p[3], -p[4]) for p in TPS62933_PINS},
    "L": {"1": (0, -3.81), "2": (0, 3.81)},
    "Q_NMOS_GSD": {p[0]: (p[3], -p[4]) for p in NMOS_PINS},
    "Conn_01x06": {str(i + 1): (-5.08, i * 2.54) for i in range(6)},
    "Conn_01x02": {str(i + 1): (-5.08, i * 2.54) for i in range(2)},
}
PIN_OFFS.update(can.pin_offs())       # Rev 14.0 TCAN334G / ESD array / GH4 / solder jumper


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

# Rev 13.0 part dicts (LCSC numbers checked against the JLC catalogue
# 2026-11-06; same parts as existing refs where the value matches).
R10K = {"MPN": "0402WGF1002TCE", "Manufacturer": "UNI-ROYAL", "LCSC": "C25744"}    # as R3/R17
R1K = {"MPN": "0402WGF1001TCE", "Manufacturer": "UNI-ROYAL", "LCSC": "C11702"}     # as R16
R100 = {"MPN": "0402WGF1000TCE", "Manufacturer": "UNI-ROYAL", "LCSC": "C25076"}    # as R4..R7
R75 = {"MPN": "0402WGF750JTCE", "Manufacturer": "UNI-ROYAL", "LCSC": "C25133"}
R102K = {"MPN": "0402WGF1023TCE", "Manufacturer": "UNI-ROYAL", "LCSC": "C26991"}
R26K7 = {"MPN": "0402WGF2672TCE", "Manufacturer": "UNI-ROYAL", "LCSC": "C25876"}
R100K = {"MPN": "0402WGF1003TCE", "Manufacturer": "UNI-ROYAL", "LCSC": "C25741"}
FB_BLM = {"MPN": "BLM15AX601SN1D", "Manufacturer": "Murata", "LCSC": "C76886"}     # as FB1..FB4
C100N = {"MPN": "CL05B104KO5NNNC", "Manufacturer": "Samsung", "LCSC": LCSC_100N}  # 16V X7R
C100N50 = {"MPN": "CL05B104KB54PNC", "Manufacturer": "Samsung", "LCSC": "C307331"}  # 50V X7R
C10U_0603 = {"MPN": "CL10A106KP8NNNC", "Manufacturer": "Samsung", "LCSC": "C19702"}  # as C17/C20
C10U_50V = {"MPN": "CL31A106KBHNNNE", "Manufacturer": "Samsung", "LCSC": "C13585"}
C22U_0805 = {"MPN": "CL21A226MAQNNNE", "Manufacturer": "Samsung", "LCSC": "C45783"}  # 25V X5R
C22U_25V = {"MPN": "CL31A226KAHNNNE", "Manufacturer": "Samsung", "LCSC": "C12891"}
C33N = {"MPN": "CC0402KRX7R9BB333", "Manufacturer": "YAGEO", "LCSC": "C106862"}   # 50V X7R

# Rev 13.0: AT7456E OSD on SPI2.  SPI tuples (ref, MCU/hwdef net, OSD net,
# U5 pin): pin 1 = MCU net, pin 2 = OSD-pin net (Rev 11 convention).
OSD = {"ref": "U5", "fb": "FB5", "rail": "+3V3_OSD", "xtal": "Y3", "c_bulk": "C27",
       "c_dec": (("C28", "3", "DVDD"), ("C29", "21", "AVDD"), ("C30", "24", "PVDD")),
       "r_reset": "R29", "reset_net": "OSD_RESET_N", "clkin_net": "OSD_CLKIN", "xfb_net": "OSD_XFB",
       "spi": (("R30", "SPI2_SCK", "OSD_SCLK", "10"), ("R31", "SPI2_MOSI", "OSD_SDIN", "9"),
               ("R32", "OSD1_CS", "OSD_CS_N", "8"), ("R33", "SPI2_MISO", "OSD_SDOUT", "11")),
       "nc": ("1", "2", "7", "12", "13", "14", "15", "16", "17", "18", "27", "28"),
       "gnd": ("4", "20", "23", "29")}
# Rev 13.0: 9 V / 12 V VTX BEC (U6 TPS62933) and DJI 6-pin VTX connector J1.
BEC = {"ref": "U6", "rail": "+VTX", "vin": "VBAT", "en_r": "R39", "en_net": "BEC_EN",
       "fb_top": "R34", "fb_bot": "R35", "fb_sw_r": "R36", "q": "Q1", "g_r": "R37", "g_pd": "R38",
       "sense_top": "R40", "sense_bot": "R41", "sense_c": "C43",
       "vref": 0.8, "r_top": 102e3, "r_bot": 10e3, "r_sw": 26.7e3, "sense": (10e3, 1e3)}
# DJI-standard FC-side order: (J1 pin, net, series R, MCU/hwdef net)
VTX_J1 = (("1", "+VTX", None, None), ("2", "GND", None, None),
          ("3", "VTX_TXD", "R42", "USART1_TX"), ("4", "VTX_RXD", "R43", "USART1_RX"),
          ("5", "GND", None, None), ("6", "VTX_SBUS", "R44", "UART7_RX"))
DEFAULTS_PARM = os.path.join(REPO, "ardupilot", "defaults.parm")


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

    # ---------------- AT7456E OSD on SPI2 (Rev 13.0) ----------------
    osd_block(sh, hw)

    # ---------------- 9 V / 12 V VTX BEC + DJI VTX connector (Rev 13.0) ----------------
    bec_vtx_block(sh, hw)

    # ---------------- DroneCAN / CAN FD PHY (Rev 14.0, A1 right-hand column) ----------------
    can.place(sys.modules[__name__], sh, hw, 609.6, 25.4)

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
    # Rev 13.0: +3V3_OSD (via FB5), +VTX (U6 output through L1, passive) and
    # VBAT (off-sheet power stage) - second flag row, clear of the baro title.
    for i, net in enumerate([OSD["rail"], BEC["rail"], BEC["vin"]]):
        x, y = fx + 5.08 + i * 15.24, fy + 15.24
        sh.pwr_flag(x, y)
        sh.wire(x, y, x, y + 5.08)
        if net == BEC["vin"]:
            sh.glabel(net, x, y + 5.08)
        else:
            sh.label(net, x, y + 5.08)
    # Rev 14.0: +5V on DroneCAN connector J3 pin 1 comes from the (future)
    # power sheet; flag it so ERC knows the net is driven.
    x, y = fx + 5.08 + 3 * 15.24, fy + 15.24
    sh.pwr_flag(x, y)
    sh.wire(x, y, x, y + 5.08)
    sh.power(can.CAN["vbus"], x, y + 5.08, 0)

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


def _gnd_side(sh, px, py, dx=-2.54):
    """Horizontal stub + sideways GND symbol (value hidden) for connector
    pins that sit 2.54 mm apart, where an upright GND symbol and its text
    would run into the neighbouring pin's label."""
    ex = px + dx
    sh.wire(px, py, ex, py)
    sh.pwr_n += 1
    sh.symbol("GND", "#PWR%03d" % sh.pwr_n, "GND", ex, py, rot=270 if dx < 0 else 90, pins=("1",),
              in_bom=False, ref_xy=(ex, py + 6.35), val_xy=(ex - 3.81, py), hide_val=True)


def defaults_parm(path=DEFAULTS_PARM):
    out = {}
    if os.path.exists(path):
        for line in open(path):
            m = re.match(r"^\s*([A-Z][A-Z0-9_]*)\s+(\S+)", line.split("#", 1)[0])
            if m:
                out[m.group(1)] = m.group(2)
    return out


def osd_block(sh, hw):
    """Rev 13.0 AT7456E analog OSD on SPI2, sheet strip x 280-585 / y 197-250."""
    funcs = {net_for_hwdef(v): k for k, v in hw.items()}
    for net, port in (("SPI2_SCK", "PB13"), ("SPI2_MISO", "PB14"), ("SPI2_MOSI", "PB15"), ("OSD1_CS", "PB12")):
        if funcs.get(net) != port:
            raise SystemExit("OSD block: %s must be on %s in hwdef.dat (got %s)" % (net, port, funcs.get(net)))
    if not re.search(r"^SPIDEV\s+osd\s+SPI2\s+\S+\s+OSD1_CS\s+MODE0\b", open(HWDEF).read(), re.M):
        raise SystemExit("OSD block: hwdef.dat lacks 'SPIDEV osd SPI2 ... OSD1_CS MODE0'")

    tx, ty = 281.94, 198.12
    sh.text("AT7456E analog OSD on SPI2 (Rev 13.0): 33R source termination, 27 MHz crystal, "
            "FB5-filtered +3V3_OSD, DS Fig.2 3.3 V video network", tx, ty, 2.0)
    # ---- U5 ----
    ux, uy = 452.12, 223.52
    sh.symbol("AT7456E", OSD["ref"], "AT7456E", ux, uy, footprint=FP_AT7456E,
              fields={"MPN": "AT7456E", "Manufacturer": "ZHONGKEWEI", "LCSC": "C82351",
                      "Spec": "OSD, MAX7456 register compatible; pinout AT7456E-S402-V1.1 p.6; EP = AGND; "
                              "DVDD/AVDD/PVDD 3.15-5.25 V (DS p.2) on +3V3_OSD; SCLK max 10 MHz (DS p.4)"},
              pins=tuple(p[0] for p in AT7456E_PINS),
              ref_xy=(ux - 15.24, uy - 16.51), val_xy=(ux + 12.7, uy + 19.05))
    o = PIN_OFFS["AT7456E"]

    def pt(num):
        return ux + o[num][0], uy + o[num][1]
    for _, _, onet, num in OSD["spi"]:
        sh.attach(onet, *pt(num), -5.08, 0)
    sh.attach(OSD["reset_net"], *pt("19"), -5.08, 0)
    sh.attach(OSD["clkin_net"], *pt("5"), -5.08, 0)
    sh.attach(OSD["xfb_net"], *pt("6"), -5.08, 0)
    sh.attach("OSD_VIN", *pt("22"), 5.08, 0)
    sh.attach("OSD_VOUT", *pt("26"), 5.08, 0)
    sh.attach("OSD_SAG", *pt("25"), 5.08, 0)
    for num in OSD["nc"]:
        sh.noconnect(*pt(num))
    # DVDD (3) / AVDD (21) / PVDD (24) -> +3V3_OSD bus, one label
    sup = [pt(n) for _, n, _ in OSD["c_dec"]]
    for (x_, y_) in sup:
        sh.wire(x_, y_, x_, y_ - 2.54)
    for (x1_, y1_), (x2_, _) in zip(sup, sup[1:]):
        sh.wire(x1_, y1_ - 2.54, x2_, y1_ - 2.54)
    sh.junction(sup[1][0], sup[1][1] - 2.54)
    sh.label(OSD["rail"], sup[-1][0], sup[-1][1] - 2.54)
    # DGND (4) / AGND (20) / PGND (23) / EP (29) -> GND bus, one GND symbol
    gnd = [pt(n) for n in OSD["gnd"]]
    for (x_, y_) in gnd:
        sh.wire(x_, y_, x_, y_ + 2.54)
    for (x1_, y1_), (x2_, _) in zip(gnd, gnd[1:]):
        sh.wire(x1_, y1_ + 2.54, x2_, y1_ + 2.54)
    for (x_, y_) in gnd[1:-1]:
        sh.junction(x_, y_ + 2.54)
    sh.power("GND", gnd[0][0], gnd[0][1] + 2.54, 0)

    # ---- U1-side parts (left part of the strip) ----
    ry = 223.52
    for j, (ref, mnet, onet, num) in enumerate(OSD["spi"]):
        at_osd = mnet.endswith("MISO")
        passive(sh, "R", ref, "33R", 287.02 + j * 20.32, ry, mnet, onet, FP_R0402,
                dict(R33, Spec="33R 1%% 0402 - %s source termination, place at %s" % (
                    mnet, "U5 pin 11 SDOUT (OSD drives MISO)" if at_osd else "U1 pin")),
                kind="global", kind2="label")
    passive(sh, "R", OSD["r_reset"], "10k", 368.3, ry, OSD["rail"], OSD["reset_net"], FP_R0402,
            dict(R10K, Spec="10k 1% 0402 - U5 /RESET pull-up (power-up with /RESET high allowed, DS p.20; "
                            "AP_OSD_MAX7456 resets via VM0[1])"))
    yx = 396.24
    sh.symbol("Crystal_GND24", OSD["xtal"], "27MHz CL=12pF", yx, ry, footprint=FP_XTAL,
              fields={"MPN": "X322527MOB4SI", "Manufacturer": "YXC", "LCSC": "C70595",
                      "Spec": "27.000 MHz, CL 12 pF, ESR<=50R, +/-10 ppm, +/-20 ppm -40..85C; no load caps "
                              "(AT7456E DS Fig.2); pins 1/3 OSC, 2/4 GND (LCSC C70595 symbol)"},
              pins=("1", "2", "3", "4"), ref_xy=(yx + 2.54, ry - 7.62), val_xy=(yx + 2.54, ry + 8.89))
    sh.attach(OSD["clkin_net"], yx - 3.81, ry, -2.54, 0)
    sh.attach(OSD["xfb_net"], yx + 3.81, ry, 2.54, 0)
    sh.attach("GND", yx, ry + 5.08, 0, 2.54)
    sh.attach("GND", yx, ry - 5.08, 0, -2.54)

    # ---- supply filter + decoupling (right of U5, upper row) ----
    px, r1, r2 = 495.3, 213.36, 236.22
    passive(sh, "FerriteBead", OSD["fb"], "600R@100MHz", px - 5.08, r1, "+3V3", OSD["rail"], FP_L0402,
            dict(FB_BLM, Spec="600R@100MHz, Rdc 0.34R max, 500 mA - OSD supply filter (same part as FB1-FB4); "
                              "U5 72.5 mA max (DS p.2) -> <= 25 mV drop"))
    passive(sh, "C", OSD["c_bulk"], "10uF", px + 20.32, r1, OSD["rail"], "GND", FP_C0603,
            dict(C10U_0603, Spec="10 uF X5R 10V 0603 - +3V3_OSD bulk (same part as C17/C20)"))
    for j, (ref, num, nm) in enumerate(OSD["c_dec"]):
        passive(sh, "C", ref, "100nF", px + (2 + j) * 15.24 + 5.08, r1, OSD["rail"], "GND", FP_C0402,
                dict(C100N, Spec="100 nF X7R 16V 0402 - at U5 pin %s %s (DS p.6: 0.1 uF per supply)" % (num, nm)))
    # ---- video network (lower row): DS Fig.2 3.3 V circuit ----
    passive(sh, "R", "R27", "75R", px, r2, "VIDEO_IN", "GND", FP_R0402,
            dict(R75, Spec="75R 1% 0402 - camera CVBS termination at U5 VIN (DS Fig.2)"), kind="global")
    passive(sh, "C", "C31", "100nF", px + 15.24, r2, "VIDEO_IN", "OSD_VIN", FP_C0402,
            dict(C100N, Spec="100 nF X7R 0402 - VIN AC coupling into the sync-tip clamp (DS p.10: 0.1 uF)"),
            kind="global", kind2="label")
    passive(sh, "C", "C32", "22uF", px + 30.48, r2, "OSD_VOUT", "OSD_VOUT_AC", FP_C0805,
            dict(C22U_0805, Spec="22 uF X5R 25V 0805 - COUT (DS Table 2: 22u/22u -> 0.4 % line-time distortion)"))
    passive(sh, "C", "C33", "22uF", px + 45.72, r2, "OSD_SAG", "OSD_VOUT_AC", FP_C0805,
            dict(C22U_0805, Spec="22 uF X5R 25V 0805 - CSAG (DS Table 2 / Fig.2)"))
    passive(sh, "R", "R28", "75R", px + 60.96, r2, "OSD_VOUT_AC", "VIDEO_OUT", FP_R0402,
            dict(R75, Spec="75R 1% 0402 - CVBS back-termination to the analog VTX (DS Fig.2)"),
            kind="label", kind2="global")

    notes = (
        "OSD BLOCK (Rev 13.0) - U5 AT7456E (LCSC C82351), datasheet AT7456E-S402-V1.1 (DS)\n"
        "- Pinout DS p.6: 1/2/14-16/27/28 NC, 3 DVDD, 4 DGND, 5 CLKIN, 6 XFB, 7 CLKOUT, 8 /CS, 9 SDIN, 10 SCLK, 11 SDOUT,\n"
        "  12 LOS, 13 SYNC_IN, 17 /VSYNC, 18 /HSYNC, 19 /RESET, 20 AGND, 21 AVDD, 22 VIN, 23 PGND, 24 PVDD, 25 SAG, 26 VOUT, EP=AGND.\n"
        "- All three supplies on +3V3_OSD (3.15-5.25 V, DS p.2): SDOUT swings 0..3.3 V into PB14 and every MCU-driven\n"
        "  input stays inside its VDVDD+0.3 V abs max. AVDD min 3.15 V needs +3V3 >= 3.18 V at the FB5 input.\n"
        "- SPI2 MODE0 (SDIN sampled on SCLK rise, SDOUT on fall, DS p.4). hwdef SPIDEV osd 8*MHZ -> 80 MHz/16 = 5 MHz\n"
        "  (tCP 200 ns vs 100 ns min). AP_OSD_MAX7456 polls STAT: LOS, /VSYNC, /HSYNC, SYNC_IN, CLKOUT unused (NC).\n"
        "- NVM read (CMM=0x50) needs >= 22 us on the AT7456E (DS p.36) vs 0.5 us MAX7456: see ardupilot/patches/0003.\n"
        "- 27 MHz crystal Y3 straight between CLKIN/XFB with no load caps (DS Fig.2); VIN 75R + 100 nF AC coupling;\n"
        "  VOUT -> C32 22u, SAG -> C33 22u -> R28 75R back-termination -> VIDEO_OUT (2.0 V/V gain, Zout 0.22R, DS p.3).\n"
        "LAYOUT RULES\n"
        "- R30-R32 within 3 mm of U1 pins 52/54/51; R33 within 3 mm of U5 pin 11. SPI2 over solid In1 GND.\n"
        "- C28/C29/C30 within 1 mm of U5 pins 3/21/24; EP 29 soldered to GND with >= 6 vias (thermal + AGND star);\n"
        "  DGND 4 / AGND 20 / PGND 23 return into the EP island - never as the only ground path (DS p.6).\n"
        "- Y3 within 3 mm of pins 5/6, GND guard, no SPI/video traces under it. VIDEO_IN/OUT 75R-ish traces,\n"
        "  >= 3x width from SPI2 and from the BEC SW node / L1; C31 and R28 at U5 pins 22 / 26.\n"
    )
    sh.text(notes, 281.94, 104.14)


VTX_BEC_REPORT = os.path.join(REPO, "analysis", "osd_vtx_bec_report.json")


def bec_note_values(v9, v12, ratio, path=VTX_BEC_REPORT):
    """Sheet-note numbers for the BEC: corner ranges / dropout / tSS from the
    'schematic_values' block of analysis/osd_vtx_bec_report.json (Vref
    784-816 mV x 1 % resistor corners, hot dropout at 2 A).  The nominal
    values are re-derived here from the placed parts and must agree with the
    report, so a drifted analysis cannot silently annotate the sheet.
    Falls back to the Vref-only spread (SLUSEA4D 8.5) if the report is absent."""
    out = {"r9": (v9 * 0.784 / 0.8, v9 * 0.816 / 0.8), "r12": (v12 * 0.784 / 0.8, v12 * 0.816 / 0.8),
           "vin12": None, "src": "Vref 784-816 mV only (analysis report absent)"}
    if os.path.exists(path):
        import json
        sv = json.load(open(path)).get("schematic_values")
        if sv:
            if (abs(sv["vtx_9V"] - v9) > 0.006 or abs(sv["vtx_12V"] - v12) > 0.006
                    or abs(sv["sense_ratio"] - ratio) > 1e-6):
                raise SystemExit("VTX block: analysis/osd_vtx_bec_report.json (%.2f/%.2f V, x%s) disagrees with "
                                 "the schematic divider (%.2f/%.2f V, x%.1f)"
                                 % (sv["vtx_9V"], sv["vtx_12V"], sv["sense_ratio"], v9, v12, ratio))
            out = {"r9": tuple(sv["vtx_9V_range"]), "r12": tuple(sv["vtx_12V_range"]),
                   "vin12": sv.get("vin_min_12V_V"),
                   "src": "Vref 784-816 mV + 1 % R corners, analysis/osd_vtx_bec_report.json"}
    return out


def bec_vtx_block(sh, hw):
    """Rev 13.0 9 V / 12 V VTX BEC (U6) + DJI 6-pin VTX connector, lower right."""
    funcs = {net_for_hwdef(v): k for k, v in hw.items()}
    for net, port in (("VTX_PWR", "PD10"), ("VTX_12V_SEL", "PD11"), ("VTX_VSENSE", "PC5"),
                      ("UART7_RX", "PE7"), ("USART1_TX", "PA9"), ("USART1_RX", "PA10")):
        if funcs.get(net) != port:
            raise SystemExit("VTX block: %s must be on %s in hwdef.dat (got %s)" % (net, port, funcs.get(net)))
    vref, rt, rb, rs = BEC["vref"], BEC["r_top"], BEC["r_bot"], BEC["r_sw"]
    v9 = vref * (1 + rt / rb)
    v12 = vref * (1 + rt / (rb * rs / (rb + rs)))
    ratio = (BEC["sense"][0] + BEC["sense"][1]) / BEC["sense"][1]
    mult = defaults_parm().get("BATT2_VOLT_MULT")
    if mult is None or abs(float(mult) - ratio) > 1e-6 * ratio:
        raise SystemExit("VTX block: sense divider x%.3f != defaults.parm BATT2_VOLT_MULT %s" % (ratio, mult))

    sh.text("9 V / 12 V 2 A VTX BEC (Rev 13.0): TPS62933 500 kHz, Q1 FB switch (fails to 9 V), "
            "DJI 6-pin SH1.0 VTX connector", 396.24, 288.29, 2.0)
    # ---- row 1: U6, L1, BST, SS, input caps, J2 ----
    y1, y2, y3 = 307.34, 332.74, 358.14
    ux = 424.18
    sh.symbol("TPS62933", BEC["ref"], "TPS62933", ux, y1, footprint=FP_TPS62933,
              fields={"MPN": "TPS62933DRLR", "Manufacturer": "Texas Instruments", "LCSC": "C3200405",
                      "Spec": "VIN 3.8-30 V, 3 A, Vref 0.8 V +/-2% (SLUSEA4D 8.5); RT NC = 500 kHz; "
                              "EN internal pull-up (floating = ON); SS 33 nF = 4.8 ms"},
              pins=tuple(p[0] for p in TPS62933_PINS),
              ref_xy=(ux - 7.62, y1 - 8.89), val_xy=(ux + 2.54, y1 + 8.89))
    o = PIN_OFFS["TPS62933"]

    def pt(num):
        return ux + o[num][0], y1 + o[num][1]
    sh.attach(BEC["vin"], *pt("3"), -5.08, 0, "global")
    sh.attach(BEC["en_net"], *pt("2"), -5.08, 0)
    sh.noconnect(*pt("1"))                          # RT floating -> 500 kHz
    sh.attach("BEC_SS", *pt("7"), -5.08, 0)
    sh.attach("BEC_BST", *pt("6"), 5.08, 0)
    sh.attach("BEC_SW", *pt("5"), 5.08, 0)
    sh.attach("BEC_FB", *pt("8"), 5.08, 0)
    sh.attach("GND", *pt("4"), 0, 2.54)

    px = 457.2
    sh.symbol("L", "L1", "10uH", px, y1, footprint=FP_L6045,
              fields={"MPN": "PNLS6045-100M", "Manufacturer": "DMBJ", "LCSC": "C2849537",
                      "Spec": "10 uH 20%, Isat 4.5 A, DCR 57 mR (+/-30%), 6.0x6.0x4.5 mm shielded; ripple "
                              "0.84 A p-p at 9 V / 16.8 V in (TI DS Table 10-2: 12 uH at 12 V)"},
              ref_xy=(px + 2.54, y1 - 1.27), val_xy=(px + 2.54, y1 + 1.27))
    sh.attach("BEC_SW", px, y1 - 3.81, 0, -2.54)
    sh.attach(BEC["rail"], px, y1 + 3.81, 0, 2.54)
    passive(sh, "C", "C37", "100nF", px + 15.24, y1, "BEC_BST", "BEC_SW", FP_C0402,
            dict(C100N, Spec="100 nF X7R 16V 0402 - bootstrap BST-SW (SLUSEA4D 10.2.2.5: >= 16 V, X5R+)"))
    passive(sh, "C", "C38", "33nF", px + 30.48, y1, "BEC_SS", "GND", FP_C0402,
            dict(C33N, Spec="33 nF X7R 50V 0402 - soft start tSS = Css*0.8V/5.5uA = 4.8 ms "
                            "(SLUSEA4D Eq.6; min 6.8 nF, never float)"))
    for j, ref in enumerate(("C34", "C35")):
        passive(sh, "C", ref, "10uF", px + (3 + j) * 15.24, y1, BEC["vin"], "GND", FP_C1206,
                dict(C10U_50V, Spec="10 uF X5R 50V 1206 - U6 input bulk (50 V keeps capacitance at 16.8-25.2 V)"),
                kind="global", kind2="label")
    passive(sh, "C", "C36", "100nF", px + 76.2, y1, BEC["vin"], "GND", FP_C0402,
            dict(C100N50, Spec="100 nF X7R 50V 0402 - HF bypass at U6 VIN pin 3 / GND pin 4"),
            kind="global", kind2="label")
    jx = 558.8
    sh.symbol("Conn_01x02", "J2", "VTX_PWR_PADS", jx, y1, footprint=FP_WIRE2,
              fields={"Assembly": "none - bare plated pads, no part placed (no LCSC)",
                      "Spec": "+VTX / GND solder pair for > 1 A loads (DJI O3 16 W = 1.33 A @ 12 V, "
                              "1.78 A @ 9 V) - JST SH J1 is rated 1 A per contact"},
              ref_xy=(jx + 2.54, y1 - 2.54), val_xy=(jx + 2.54, y1 + 5.08))
    oj = PIN_OFFS["Conn_01x02"]
    sh.attach(BEC["rail"], jx + oj["1"][0], y1 + oj["1"][1], -2.54, 0)
    _gnd_side(sh, jx + oj["2"][0], y1 + oj["2"][1])

    # ---- row 2: EN, FB divider + Q1 select, output caps ----
    passive(sh, "R", BEC["en_r"], "1k", 401.32, y2, "VTX_PWR", BEC["en_net"], FP_R0402,
            dict(R1K, Spec="1k 1% 0402 - VTX_PWR -> U6 EN; no pull-down: EN internal pull-up keeps the VTX "
                           "ON while PD10 is high-Z (reset/bootloader)"), kind="global", kind2="label")
    passive(sh, "R", BEC["fb_top"], "102k", 416.56, y2, BEC["rail"], "BEC_FB", FP_R0402,
            dict(R102K, Spec="102k 1% 0402 - FB top (Vout = 0.8 V x (1 + Rtop/Rbot), SLUSEA4D Eq.1)"))
    passive(sh, "R", BEC["fb_bot"], "10k", 431.8, y2, "BEC_FB", "GND", FP_R0402,
            dict(R10K, Spec="10.0k 1%% 0402 - FB bottom: Q1 off -> %.2f V" % v9))
    passive(sh, "R", BEC["fb_sw_r"], "26.7k", 447.04, y2, "BEC_FB", "BEC_FB_SW", FP_R0402,
            dict(R26K7, Spec="26.7k 1%% 0402 - switched FB bottom: Q1 on -> 10k||26.7k = 7.275k -> %.2f V" % v12))
    qx = 477.52
    sh.symbol("Q_NMOS_GSD", BEC["q"], "2N7002", qx, y2, footprint=FP_SOT23,
              fields={"MPN": "2N7002", "Manufacturer": "Jiangsu Changjing", "LCSC": "C8545",
                      "Spec": "60 V N-MOSFET, SOT-23 1 G / 2 S / 3 D (LCSC C8545); sinks 0.8 V/26.7k = 30 uA; "
                              "gate 3.27 V when driven > Vgs(th) 2.5 V"},
              pins=("1", "2", "3"), ref_xy=(qx + 5.08, y2 - 1.27), val_xy=(qx + 5.08, y2 + 1.27))
    oq = PIN_OFFS["Q_NMOS_GSD"]
    sh.attach("BEC_FB_SW", qx + oq["3"][0], y2 + oq["3"][1], 0, -2.54)
    sh.attach("GND", qx + oq["2"][0], y2 + oq["2"][1], 0, 2.54)
    sh.attach("BEC_VSEL_G", qx + oq["1"][0], y2 + oq["1"][1], -2.54, 0)
    passive(sh, "R", BEC["g_r"], "1k", 492.76, y2, "VTX_12V_SEL", "BEC_VSEL_G", FP_R0402,
            dict(R1K, Spec="1k 1% 0402 - VTX_12V_SEL (PD11) -> Q1 gate"), kind="global", kind2="label")
    passive(sh, "R", BEC["g_pd"], "100k", 513.08, y2, "BEC_VSEL_G", "GND", FP_R0402,
            dict(R100K, Spec="100k 1% 0402 - Q1 gate pull-down: PD11 high-Z -> Q1 off -> 9 V (failsafe)"))
    for j, ref in enumerate(("C39", "C40", "C41")):
        passive(sh, "C", ref, "22uF", 528.32 + j * 12.7, y2, BEC["rail"], "GND", FP_C1206,
                dict(C22U_25V, Spec="22 uF X5R 25V 1206 - +VTX output (TI DS Table 10-2: >= 10 uF effective at 12 V)"))
    passive(sh, "C", "C42", "100nF", 566.42, y2, BEC["rail"], "GND", FP_C0402,
            dict(C100N50, Spec="100 nF X7R 50V 0402 - +VTX HF bypass at L1 / J1"))

    # ---- row 3: rail sense, VTX UART/SBUS series resistors, J1 ----
    passive(sh, "R", BEC["sense_top"], "10k", 401.32, y3, BEC["rail"], "VTX_VSENSE", FP_R0402,
            dict(R10K, Spec="10k 1%% 0402 - +VTX sense top: x%.1f = BATT2_VOLT_MULT (defaults.parm)" % ratio),
            kind="label", kind2="global")
    passive(sh, "R", BEC["sense_bot"], "1k", 421.64, y3, "VTX_VSENSE", "GND", FP_R0402,
            dict(R1K, Spec="1k 1%% 0402 - +VTX sense bottom: %.2f V -> %.3f V at PC5 (ADC1 INP8)" % (v12, v12 / ratio)),
            kind="global", kind2="label")
    passive(sh, "C", BEC["sense_c"], "100nF", 441.96, y3, "VTX_VSENSE", "GND", FP_C0402,
            dict(C100N, Spec="100 nF X7R 0402 - VTX_VSENSE ADC anti-alias (909R x 100n = 91 us)"),
            kind="global", kind2="label")
    rx = 462.28
    j = 0
    for pin, net, ref, mnet in VTX_J1:
        if ref is None:
            continue
        val, fl = ("1k", dict(R1K, Spec="1k 1% 0402 - UART7_RX (PE7, FT) <- J1 pin 6 SBUS / DJI HDL: limits clamp "
                                         "current from 5 V logic to < 2 mA")) if mnet == "UART7_RX" else \
            ("100R", dict(R100, Spec="100R 1%% 0402 - %s <-> J1 pin %s: hot-plug / ESD current limit, "
                                     "100R x 20 pF = 2 ns at 2 Mbaud" % (mnet, pin)))
        passive(sh, "R", ref, val, rx + j * 20.32, y3, mnet, net, FP_R0402, fl, kind="global", kind2="label")
        j += 1
    jx, jy = 548.64, 351.79
    sh.symbol("Conn_01x06", "J1", "SM06B-SRSS-TB", jx, jy, footprint=FP_JST_SH6,
              fields={"MPN": "SM06B-SRSS-TB(LF)(SN)", "Manufacturer": "JST", "LCSC": "C160405",
                      "Spec": "JST SH 1.0 mm 6-pin side entry, 1 A/contact; DJI FC-side order 1 VIN, 2 GND, "
                              "3 FC TX, 4 FC RX, 5 GND, 6 SBUS/HDL"},
              pins=("1", "2", "3", "4", "5", "6"), ref_xy=(jx - 2.54, jy - 3.81), val_xy=(jx + 2.54, jy + 15.24))
    oj = PIN_OFFS["Conn_01x06"]
    for pin, net, _, _ in VTX_J1:
        x_, y_ = jx + oj[pin][0], jy + oj[pin][1]
        if net == "GND":
            _gnd_side(sh, x_, y_)
        else:
            sh.attach(net, x_, y_, -2.54, 0)

    nv = bec_note_values(v9, v12, ratio)
    notes = (
        "VTX BEC + CONNECTOR (Rev 13.0) - U6 TPS62933DRLR (LCSC C3200405), TI SLUSEA4D\n"
        "- VBAT -> U6 (VIN 3.8-30 V, 2S-6S) -> L1 10uH -> +VTX. Vout = 0.8 V x (1 + 102k/Rbot): Q1 off Rbot 10k = %.2f V,\n"
        "  Q1 on 10k||26.7k = %.2f V. Corners %.2f-%.2f / %.2f-%.2f V (%s).\n"
        "  12 V corner can exceed HDZero's 12 V max: HDZero stays on 9 V.\n"
        "- Failsafe: PD11 high-Z (reset, bootloader) -> R38 100k holds Q1 off -> 9 V. 12 V only while firmware drives\n"
        "  VTX_12V_SEL high (RELAY3, default 0); 12->9 V trips 115%% OVP harmlessly (FB 1.07 V, DS 9.3.11).\n"
        "- EN: VTX_PWR (PD10) -> R39 1k; internal pull-up => VTX on while PD10 is high-Z; RELAY2_DEFAULT 1 (defaults.parm).\n"
        "- 12 V regulates only from %s V in at 2 A (Dmax 0.93 at 500 kHz, hot) => 4S only; 9 V needs ~10 V (3S ok).\n"
        "- Sense R40 10k / R41 1k: x%.1f = BATT2_VOLT_MULT, BATT2_VOLT_PIN 8 = PC5 ADC1_INP8 (STM32H743xx.py:1289).\n"
        "- J1 JST SH 1 A/contact: DJI O3 (16 W) must use the J2 solder pair for +VTX/GND; J1 then carries data only.\n"
        "LAYOUT RULES\n"
        "- C34/C35/C36 at U6 pins 3/4 (shortest VIN-GND loop); SW node (pin 5, L1, C37) minimal copper, no vias,\n"
        "  >= 5 mm from U5/Y3, video and IMU parts; L1 pad-to-pad, output caps C39-C42 return to U6 GND pin 4.\n"
        "- FB network R34/R35/R36/Q1 at U6 pin 8, away from SW; Kelvin-sense +VTX at C39. R40/R41/C43 at U1 PC5.\n"
        "- U6 thermal: RthJA 60.2 K/W (EVM) -> pour GND around pin 4 with vias. J1/J2 at the board edge, +VTX >= 1 mm.\n"
    ) % (v9, v12, nv["r9"][0], nv["r9"][1], nv["r12"][0], nv["r12"][1], nv["src"],
         ("%.2f" % nv["vin12"]) if nv["vin12"] else "~13.3", ratio)
    lines = notes.rstrip("\n").split("\n")
    half = (len(lines) + 1) // 2
    sh.text("\n".join(lines[:half]) + "\n", 25.4, 385.445)
    sh.text("\n".join(lines[half:]) + "\n", 177.8, 385.445)


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
        "(paper \"A1\")",
        "(title_block (title \"Nano H743 AIO - MCU core: IMU, baro, OSD, VTX BEC, DroneCAN\")",
        "  (date \"2026-11-06\") (rev \"14.0\") (company \"Sreesai Arjun Engineering\")",
        "  (comment 1 \"STM32H743VIT6 480 MHz: HSE 24 MHz, PLL1 M3 N120 P2\")",
        "  (comment 2 \"U2/U3 ICM-42688-P; U4 DPS310; U5 AT7456E OSD; U6 9/12 V VTX BEC; J1 DJI\")",
        "  (comment 3 \"Generated: tools/gen_mcu_core_sch.py + hwdef.dat\")",
        "  (comment 4 \"Production Revision 14.0 / 150; U7 TCAN334G CAN FD, J3 DroneCAN\"))",
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
