#!/usr/bin/env python3
"""
Nano H743 AIO - DroneCAN / CAN FD physical-layer schematic block (Revision 14.0)

Imported by tools/gen_mcu_core_sch.py (symbols + placement) and by
tools/verify_mcu_core_sch.py (S11).  Kept in its own module so the CAN
block can be reviewed and regenerated without touching the other blocks.

Circuit (TI TCAN334G DS SLLSEJ9 + Nexperia PESD2CANFD24V-T DS 11 Aug 2020):

  U1 PD1 CAN1_TX ---------------> U7 TXD (1)   [TXD integrated pull-up = recessive]
  U1 PD0 CAN1_RX <--------------- U7 RXD (4)
  U1 PD3 GPIO_CAN1_SILENT ------> U7 STB (8)   [integrated pull-DOWN: high = standby]
  GND --------------------------> U7 SHDN (5)  [integrated pull-down; tied, never used]
  +3V3 -> U7 VCC (3), C44 100 nF + C45 1 uF;  U7 GND (2) -> GND

  U7 CANH (7) -- CAN1_H --+-- D4 K1 (1) --+               J3 (JST-GH SM04B-GHS-TB, DroneCAN std)
                          +-- J3 pin 2    |  D4 CC (3) -> GND     1 +5V   (from the future power sheet)
                          +-- JP1 -- CAN1_TH -- R45 60.4R --+     2 CANH
                                                           |     3 CANL
                                          CAN1_SPLIT ------+     4 GND, MP -> GND
                                                           |
  U7 CANL (6) -- CAN1_L --+-- D4 K2 (2)   C46 4.7 nF C0G --+-> GND
                          +-- J3 pin 3                     |
                          +-- JP2 -- CAN1_TL -- R46 60.4R --+

  JP1/JP2: solder jumpers, bridged by default (this node terminates the bus;
  the FC is normally one end of a DroneCAN chain).  Cut BOTH to remove the
  termination - cutting both keeps C46 off the bus, so no asymmetric
  common-mode load is left behind.
"""

# Reference designators (allocated after the Rev 13.0 OSD / VTX block:
# U1-U6, R1-R44, C1-C43, J1-J2, Q1, FB1-FB5, L1, Y1-Y3).
CAN = {
    "u": "U7", "esd": "D4", "conn": "J3", "jp_h": "JP1", "jp_l": "JP2",
    "r_h": "R45", "r_l": "R46", "c_split": "C46", "c_dec": "C44", "c_bulk": "C45",
    "h": "CAN1_H", "l": "CAN1_L", "th": "CAN1_TH", "tl": "CAN1_TL", "split": "CAN1_SPLIT",
    "tx": "CAN1_TX", "rx": "CAN1_RX", "stb": "GPIO_CAN1_SILENT", "vbus": "+5V",
}

# TCAN334G (SOT-23-8 DCN): LCSC C191005 EasyEDA symbol == TI DS pin table.
# (number, name, type, lib x, lib y, rot)
TCAN_PINS = (
    ("1", "TXD", "input", -12.7, 2.54, 0),
    ("4", "RXD", "output", -12.7, 0, 0),
    ("8", "STB", "input", -12.7, -2.54, 0),
    ("5", "SHDN", "input", -12.7, -5.08, 0),
    ("7", "CANH", "bidirectional", 12.7, 2.54, 180),
    ("6", "CANL", "bidirectional", 12.7, -2.54, 180),
    ("3", "VCC", "power_in", 0, 10.16, 270),
    ("2", "GND", "power_in", 0, -10.16, 90),
)
FP_TCAN = "Package_TO_SOT_SMD:SOT-23-8"
DS_TCAN = "https://www.ti.com/lit/ds/symlink/tcan334g.pdf"

# PESD2CANFD24V-T (SOT23): DS table 2: 1 K1, 2 K2, 3 CC (common).
ESD_PINS = (
    ("1", "K1", "passive", -7.62, 2.54, 0),
    ("2", "K2", "passive", -7.62, -2.54, 0),
    ("3", "CC", "passive", 0, -7.62, 90),
)
FP_ESD = "Package_TO_SOT_SMD:SOT-23"
DS_ESD = "https://assets.nexperia.com/documents/data-sheet/PESD2CANFD24V-T.pdf"

# JST GH 4-pin side entry, DroneCAN/Pixhawk standard pinout; MP = the two
# mechanical solder tabs (one KiCad pad number "MP" for both).
GH4_PINS = (
    ("1", "+5V", "passive", -7.62, 3.81, 0),
    ("2", "CANH", "passive", -7.62, 1.27, 0),
    ("3", "CANL", "passive", -7.62, -1.27, 0),
    ("4", "GND", "passive", -7.62, -3.81, 0),
    ("MP", "MP", "passive", 7.62, -3.81, 180),
)
FP_GH4 = "Connector_JST:JST_GH_SM04B-GHS-TB_1x04-1MP_P1.25mm_Horizontal"
DS_GH4 = "https://www.jst-mfg.com/product/pdf/eng/eGH.pdf"
FP_JP = "Jumper:SolderJumper-2_P1.3mm_Bridged_RoundedPad1.0x1.5mm"
FP_R0603 = "Resistor_SMD:R_0603_1608Metric"

R_TERM = {"MPN": "AC0603FR-7W60R4L", "Manufacturer": "YAGEO", "LCSC": "C20142038"}
C_SPLIT = {"MPN": "CGA3EAC0G2A472JT000E", "Manufacturer": "TDK", "LCSC": "C695387"}
U_TCAN = {"MPN": "TCAN334GDCNR", "Manufacturer": "Texas Instruments", "LCSC": "C191005"}
D_ESD = {"MPN": "PESD2CANFD24V-TR", "Manufacturer": "Nexperia", "LCSC": "C552486"}
J_GH4 = {"MPN": "SM04B-GHS-TB(LF)(SN)", "Manufacturer": "JST", "LCSC": "C189895"}


def symbols(g, prefix):
    """Library symbols for the CAN block (uses the generator's helpers)."""
    tcan = g._sym_ic(prefix, "TCAN334G", "U", "TCAN334G", FP_TCAN, DS_TCAN,
                     "3.3 V CAN FD transceiver, 5 Mbit/s, standby (STB) + shutdown (SHDN), SOT-23-8",
                     10.16, 7.62, TCAN_PINS)
    esd = g._sym_ic(prefix, "ESD_2CH_CC", "D", "ESD_2CH_CC", FP_ESD, DS_ESD,
                    "Dual-line bidirectional ESD array, pins 1/2 = line diodes, 3 = common (SOT23)",
                    5.08, 5.08, ESD_PINS)
    gh4 = g._sym_ic(prefix, "Conn_GH4_MP", "J", "Conn_GH4_MP", FP_GH4, DS_GH4,
                    "JST GH 1.25 mm 4-pin side entry + 2 mechanical tabs (MP), DroneCAN pinout",
                    5.08, 6.35, GH4_PINS)
    jp = g.sym_two_pin(prefix, "SolderJumper_Bridged", "JP",
                       "Solder jumper, closed (bridged) by default; cut to open",
                       "(polyline (pts (xy -1.016 0.508) (xy 1.016 0.508)) (stroke (width 0.254) (type default)) (fill (type none))) "
                       "(polyline (pts (xy -1.016 -0.508) (xy 1.016 -0.508)) (stroke (width 0.254) (type default)) (fill (type none))) "
                       "(rectangle (start -0.508 0.508) (end 0.508 -0.508) (stroke (width 0) (type default)) (fill (type outline))) "
                       "(polyline (pts (xy 0 2.54) (xy 0 0.508)) (stroke (width 0) (type default)) (fill (type none))) "
                       "(polyline (pts (xy 0 -2.54) (xy 0 -0.508)) (stroke (width 0) (type default)) (fill (type none)))")
    return [tcan, esd, gh4, jp, g.sym_power(prefix, CAN["vbus"], "rail")]


def pin_offs():
    return {
        "TCAN334G": {p[0]: (p[3], -p[4]) for p in TCAN_PINS},
        "ESD_2CH_CC": {p[0]: (p[3], -p[4]) for p in ESD_PINS},
        "Conn_GH4_MP": {p[0]: (p[3], -p[4]) for p in GH4_PINS},
        "SolderJumper_Bridged": {"1": (0, -3.81), "2": (0, 3.81)},
    }


def check_hwdef(g, hw):
    want = {"PD0": "CAN1_RX", "PD1": "CAN1_TX", "PD3": "GPIO_CAN1_SILENT"}
    bad = {k: hw.get(k) for k in want if hw.get(k) != want[k]}
    if bad:
        raise SystemExit("CAN block: hwdef.dat must assign %s (got %s)" % (want, bad))


def power_stub(sh, net, x, y, dx, dy):
    """Stub wire to a power symbol for nets Sheet.attach() does not know."""
    sh.wire(x, y, x + dx, y + dy)
    rot = 180 if dy > 0 else 0
    sh.power(net, x + dx, y + dy, rot)


def place(g, sh, hw, bx, by):
    """Place the block with its title at (bx, by); everything else is relative."""
    check_hwdef(g, hw)
    c = CAN
    sh.text("DroneCAN / CAN FD PHY (Rev 14.0): TCAN334G, split 120R termination, CAN-FD ESD array, JST-GH",
            bx, by, 2.0)
    o = pin_offs()

    # ---- U7 transceiver ----
    ux, uy = bx + 27.94, by + 22.86
    sh.symbol("TCAN334G", c["u"], "TCAN334G", ux, uy, footprint=FP_TCAN,
              fields=dict(U_TCAN, Spec="CAN FD 5 Mbit/s grade (the non-G TCAN334 is 1 Mbit/s only); "
                                       "STB/SHDN integrated pull-downs; loop delay < 135 ns"),
              pins=tuple(p[0] for p in TCAN_PINS),
              ref_xy=(ux - 10.16, uy - 8.89), val_xy=(ux + 2.54, uy + 8.89))

    def up(num):
        return ux + o["TCAN334G"][num][0], uy + o["TCAN334G"][num][1]
    sh.attach(c["tx"], *up("1"), -5.08, 0, "global")
    sh.attach(c["rx"], *up("4"), -5.08, 0, "global")
    sh.attach(c["stb"], *up("8"), -5.08, 0, "global")
    sh.attach("GND", *up("5"), -5.08, 0)
    sh.attach("+3V3", *up("3"), 0, -2.54)
    sh.attach("GND", *up("2"), 0, 2.54)

    # ---- D4 ESD array (at J3) ----
    dx, dy = bx + 66.04, uy
    sh.symbol("ESD_2CH_CC", c["esd"], "PESD2CANFD24V", dx, dy, footprint=FP_ESD,
              fields=dict(D_ESD, Spec="CAN-FD ESD, VRWM 24 V, Cd 6 pF max, dCd 0.5 %, IEC 61000-4-2 23 kV; "
                                      "place at J3, GND via at pin 3"),
              pins=("1", "2", "3"), ref_xy=(dx + 2.54, dy - 6.35), val_xy=(dx + 2.54, dy + 8.89))

    def dp(num):
        return dx + o["ESD_2CH_CC"][num][0], dy + o["ESD_2CH_CC"][num][1]
    # U7 CANH/CANL and D4 K1/K2 sit on the same rows: one straight wire each,
    # one net label on it (no facing label pairs)
    for upin, dpin, net in (("7", "1", c["h"]), ("6", "2", c["l"])):
        (x1, y1), (x2, y2) = up(upin), dp(dpin)
        assert abs(y1 - y2) < 1e-6
        sh.wire(x1, y1, x2, y2)
        sh.label(net, x1 + 2.54, y1)
    sh.attach("GND", *dp("3"), 0, 2.54)

    # ---- J3 DroneCAN connector ----
    jx, jy = bx + 99.06, uy
    sh.symbol("Conn_GH4_MP", c["conn"], "DroneCAN GH4", jx, jy, footprint=FP_GH4,
              fields=dict(J_GH4, Spec="DroneCAN / Pixhawk CAN pinout: 1 +5V, 2 CANH, 3 CANL, 4 GND; 1 A/pin"),
              pins=tuple(p[0] for p in GH4_PINS), ref_xy=(jx - 2.54, jy - 8.89),
              val_xy=(jx + 6.35, jy + 10.16))

    def jp_(num):
        return jx + o["Conn_GH4_MP"][num][0], jy + o["Conn_GH4_MP"][num][1]
    power_stub(sh, c["vbus"], *jp_("1"), -5.08, 0)
    sh.attach(c["h"], *jp_("2"), -5.08, 0)
    sh.attach(c["l"], *jp_("3"), -5.08, 0)
    sh.attach("GND", *jp_("4"), -5.08, 0)
    sh.attach("GND", *jp_("MP"), 5.08, 0)

    # ---- supply decoupling + split termination row ----
    ry = by + 55.88
    g.passive(sh, "C", c["c_dec"], "100nF", bx + 5.08, ry, "+3V3", "GND", g.FP_C0402,
              {"Spec": "100 nF X7R 16V 0402 - at U7 pin 3 VCC (TI DS sec 12)", "LCSC": g.LCSC_100N})
    g.passive(sh, "C", c["c_bulk"], "1uF", bx + 20.32, ry, "+3V3", "GND", g.FP_C0402,
              {"Spec": "1 uF X5R 25V 0402 - U7 VCC bulk (55 mA dominant steps)", "MPN": "CL05A105KA5NQNC",
               "Manufacturer": "Samsung", "LCSC": "C52923"})
    jp_fields = {"Spec": "solder jumper, bridged by default - cut JP1 AND JP2 to remove the termination"}
    sh_jp = dict(fp=FP_JP)
    g.passive(sh, "SolderJumper_Bridged", c["jp_h"], "Bridged", bx + 38.1, ry, c["h"], c["th"], sh_jp["fp"], jp_fields)
    g.passive(sh, "R", c["r_h"], "60.4R", bx + 55.88, ry, c["th"], c["split"], FP_R0603,
              dict(R_TERM, Spec="60.4R 1% 0603 200 mW anti-surge - split termination, CANH half"))
    g.passive(sh, "R", c["r_l"], "60.4R", bx + 73.66, ry, c["split"], c["tl"], FP_R0603,
              dict(R_TERM, Spec="60.4R 1% 0603 200 mW anti-surge - split termination, CANL half"))
    g.passive(sh, "SolderJumper_Bridged", c["jp_l"], "Bridged", bx + 91.44, ry, c["tl"], c["l"], sh_jp["fp"], jp_fields)
    g.passive(sh, "C", c["c_split"], "4.7nF", bx + 109.22, ry, c["split"], "GND", g.FP_C0603,
              dict(C_SPLIT, Spec="4.7 nF C0G 100V 0603 - split-termination common-mode cap, fc = 1.12 MHz"))

    notes = (
        "DRONECAN BLOCK (Rev 14.0) - analysis/can_phy_analysis.py -> analysis/can_phy_report.json\n"
        "- U7 TCAN334G (G = 5 Mbit/s CAN FD grade). STB pin 8 has an integrated pull-DOWN (DS pin table):\n"
        "  normal mode during MCU reset (PD3 Hi-Z) and after init (hwdef PD3 OUTPUT LOW); GPIO 70 high = standby.\n"
        "  SHDN pin 5 tied to GND. TXD pull-up keeps the bus recessive while PD1 is Hi-Z.\n"
        "- Termination 2 x 60.4R + 4.7 nF to GND (120.8R, 119.6..122.0R worst case, ISO 11898-2 100..130R).\n"
        "  JP1/JP2 bridged: this node is a bus end. Cut BOTH for a mid-bus FC (keeps C46 off the bus).\n"
        "- D4 PESD2CANFD24V-T: ESD/transient only (VBR >= 25.5 V > U7 abs-max 14 V) - J3 must never see VBAT.\n"
        "- FD data rate: legal CAN_P1_FDBITRATE 1/2/4/5 Mbit/s; 8 is ILLEGAL (TCAN334G 5 Mbit/s, TDC SSP on edge).\n"
        "- J3 pin 1 +5V comes from the power sheet: current-limit to <= 1 A (GH contact rating).\n"
        "LAYOUT RULES\n"
        "- J3 -> D4 -> JP/R45/R46/C46 -> U7 in that order on one layer, CANH/CANL as a coupled symmetric pair,\n"
        "  equal length (+/-0.5 mm), no vias between D4 and J3, <= 25 mm total (lumped, no 120R impedance control).\n"
        "- D4 pin 3 and C46 GND: own via each into In1 GND at the connector edge (short ESD return, no shared via).\n"
        "- C44 within 1 mm of U7 pin 3; U7 >= 5 mm from ESC phase nodes; no CAN pair under or beside motor pads.\n"
    )
    sh.text(notes, bx, ry + 12.7)
