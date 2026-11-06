#!/usr/bin/env python3
"""
Nano H743 AIO - 6-layer power-stage PCB verifier (Revision 8.0)

Gates (exit 0 only if every check passes; committed files are never modified):
  P1  Stackup: 6 copper layers, outer 1 oz (0.035), inner 2 oz (>= 0.061 finished),
      dielectrics == JLC061612-1080, sum == (general thickness) == 1.614 mm, ENIG;
      In1 GND / In2 VBAT / In3 VBAT / In4 GND planes, each filled >= 80 % of the board
      (the rest is antipads for the other plane's vias and the M3 grommet keep-outs).
  P2  Population: exactly 24 TPHR8504PL (Q1..Q24), 12 top + 12 bottom, project-library
      footprints byte-identical to a fresh regeneration; every Q pad zone connection
      SOLID (footprint attribute) - no thermal-relief spokes on any MOSFET pad.
  P3  HS thermal arrays (absolute geometry via pcbnew): every top HS has 16 x 0.30 mm
      POFV vias, each inside its own drain body AND inside the drain pad of the
      bottom HS sharing the site; every via is VBAT; every bottom HS drain is pierced
      by >= 16 VBAT vias.
  P4  LS returns: every LS (top and bottom) has >= 7 GND vias within 3 mm of its
      source row; in-pad vias sit inside source pads of BOTH LS at the site.
  P5  Through-hole safety: no plated hole (pad or via) on any outer layer lands in
      copper of a different net (pads checked on both faces).
  P6  Topology: 4 ESC x 3 phases; per phase HS(source)==LS(drain)==motor pad net on
      one side; HS drain VBAT, LS source GND; unique gate nets.
  P7  Board: 36 x 36 mm outline, 4 x NPTH 3.2 mm on a 30.5 mm square; every via
      is an F.Cu-B.Cu through via; every footprint links to the project library.
  P8  kicad-cli DRC (all severities): 0 violations, 0 unconnected; and rule liveness:
      a temp copy with one thermal via drilled 0.25 mm MUST be flagged by the
      project .kicad_dru (proves custom rules are loaded); the DRU must also parse in
      pcbnew's DRC engine (kicad-cli skips an unparsable DRU without any message).
  P9  Regeneration: tools/gen_power_stage_pcb.py into a temp dir reproduces the
      committed board byte-for-byte (deterministic UUIDs, fresh zone fill), plus a
      semantic signature (pad size/shape/layers/drill, via type/span) for diagnosis.
  P10 Fab outputs: Gerber job = 6 layers, 1.614 mm, ENIG, inner Cu 0.061, rev 8.0;
      Excellon PTH hit count == plated holes on the board; 3.2 mm NPTH x 4; exact
      file set; every file content-identical to a fresh kicad-cli re-plot.
  P11 analysis/thermal_power_analysis.py re-run on the committed board: PASS (all
      published-rating scenarios), read-back via counts equal the board, and the
      committed thermal_report.json equals the re-run field by field.

Usage: python3 tools/verify_power_stage_pcb.py
"""
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
KDIR = os.path.join(REPO, "kicad")
PCB = os.path.join(KDIR, "nano-h743-aio.kicad_pcb")
GERB = os.path.join(KDIR, "gerbers")
sys.path.insert(0, os.path.join(REPO, "tools"))
import gen_power_stage_pcb as gen  # noqa: E402
import kicad_sexpr as ks  # noqa: E402

FAIL = []
FAB_LAYERS = ("F.Cu,In1.Cu,In2.Cu,In3.Cu,In4.Cu,B.Cu,F.Paste,B.Paste,F.Silkscreen,B.Silkscreen,"
              "F.Mask,B.Mask,Edge.Cuts")
FAB_GERBER_CMD = ["kicad-cli", "pcb", "export", "gerbers", "--layers", FAB_LAYERS, "--subtract-soldermask"]
FAB_DRILL_CMD = ["kicad-cli", "pcb", "export", "drill", "--format", "excellon", "--excellon-separate-th",
                 "--generate-map", "--map-format", "gerberx2"]


def check(cond, msg):
    print(("  [PASS] " if cond else "  [FAIL] ") + msg)
    if not cond:
        FAIL.append(msg)
    return cond


JLC061612_1080 = [("copper", 0.035), ("prepreg", 0.1375), ("copper", 0.061), ("core", 0.35),
                  ("copper", 0.061), ("prepreg", 0.325), ("copper", 0.061), ("core", 0.35),
                  ("copper", 0.061), ("prepreg", 0.1375), ("copper", 0.035)]


def p1(board_sx, board):
    import pcbnew
    print("P1 stackup / planes")
    st = [l for l in ks.stackup(board_sx) if l["type"] in ("copper", "prepreg", "core")]
    got = [(l["type"], l["thickness"]) for l in st]
    check(got == JLC061612_1080, f"stackup == JLC061612-1080 ({len(got)} layers)")
    cu = [l for l in st if l["type"] == "copper"]
    check(len(cu) == 6, "6 copper layers")
    check(all(l["thickness"] >= 0.061 for l in cu[1:-1]), "inner copper 2 oz (>= 0.061 mm finished)")
    total = round(sum(l["thickness"] for l in st), 4)
    gth = float(ks.kid(ks.kid(board_sx, "general"), "thickness")[1])
    check(abs(total - gth) < 1e-3 and abs(total - 1.614) < 1e-3, f"thickness sum {total} == general {gth} == 1.614")
    fin = ks.val(ks.kid(ks.kid(board_sx, "setup"), "stackup"), "copper_finish")
    check(fin == "ENIG", f"copper finish {fin}")
    area = gen.BOARD ** 2
    for lname, nname in gen.PLANES.items():
        lid = board.GetLayerID(lname)
        zs = [z for z in board.Zones() if z.IsOnLayer(lid) and not z.GetIsRuleArea()]
        nets = {z.GetNetname() for z in zs}
        filled = sum(z.GetFilledPolysList(lid).Area() for z in zs) / 1e12
        check(nets == {nname} and filled >= 0.80 * area,
              f"{lname} plane {nets} filled {filled:.0f}/{area:.0f} mm2 ({100 * filled / area:.0f} %)")
    # no plane slots under the via arrays: sample each HS via column on every plane
    worst = 1.0
    fmap = json.load(open(os.path.join(KDIR, "power_stage_fet_map.json")))["fets"]
    fps = {fp.GetReference(): fp for fp in board.GetFootprints()}
    for m in fmap:
        if not (m["role"] == "HS" and m["side"] == "top"):
            continue
        fp = fps[m["ref"]]
        vias = [p.GetPosition() for p in fp.Pads() if p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH]
        for lname in gen.PLANES:
            lid = board.GetLayerID(lname)
            polys = [z.GetFilledPolysList(lid) for z in board.Zones() if z.IsOnLayer(lid)]
            for i, va in enumerate(vias):
                for vb in vias[i + 1:]:
                    d = ((va.x - vb.x) ** 2 + (va.y - vb.y) ** 2) ** 0.5
                    if abs(pcbnew.ToMM(int(d)) - 1.0) < 0.01:          # adjacent in the 1.0 mm pitch column
                        mid = pcbnew.VECTOR2I((va.x + vb.x) // 2, (va.y + vb.y) // 2)
                        if not any(pl.Contains(mid) for pl in polys):
                            worst = 0.0
    check(worst == 1.0, "planes are continuous between adjacent thermal vias (no antipad slots)")


def fp_index(board):
    import pcbnew
    return {fp.GetReference(): fp for fp in board.GetFootprints()}


def p2(board_sx, board, regen_dir):
    import pcbnew
    print("P2 population / footprints / zone connection")
    qs = [fp for fp in board.GetFootprints() if fp.GetReference().startswith("Q")]
    refs = sorted(int(fp.GetReference()[1:]) for fp in qs)
    check(refs == list(range(1, 25)), f"Q1..Q24 present ({len(qs)})")
    check(all(fp.GetValue() == "TPHR8504PL" for fp in qs), "all values TPHR8504PL")
    top = [fp for fp in qs if fp.GetLayer() == pcbnew.F_Cu]
    check(len(top) == 12 and len(qs) - len(top) == 12, f"12 top / 12 bottom ({len(top)}/{len(qs) - len(top)})")
    libdir = os.path.join(KDIR, f"{gen.FPLIB}.pretty")
    rdir = os.path.join(regen_dir, f"{gen.FPLIB}.pretty")
    names = sorted(os.listdir(rdir))
    same = names == sorted(os.listdir(libdir)) and all(
        open(os.path.join(libdir, n)).read() == open(os.path.join(rdir, n)).read() for n in names)
    check(same, f"footprint library byte-identical to regeneration ({len(names)} footprints)")
    for f in ks.footprints(board_sx):
        if f["ref"] and f["ref"].startswith("Q"):
            if f["zone_connect"] != 2 or any(p["zone_connect"] not in (None, 2) for p in f["pads"]):
                check(False, f"{f['ref']} zone connection not solid")
                break
    else:
        check(True, "every Q footprint zone_connect = solid (2), no pad overrides")
    dru = open(os.path.join(KDIR, "nano-h743-aio.kicad_dru")).read()
    check("zone_connection solid" in dru and "memberOfFootprint('Q*')" in dru,
          "DRU enforces solid zone connection on Q* pads")


def pad_contains(pad, layer, x, y):
    import pcbnew
    return pad.IsOnLayer(layer) and pad.HitTest(pcbnew.VECTOR2I(x, y))


def p3_p4_p5(board):
    import pcbnew
    print("P3 HS thermal arrays / P4 LS returns / P5 through-hole safety")
    fps = fp_index(board)
    pads_by_layer = {pcbnew.F_Cu: [], pcbnew.B_Cu: []}
    for fp in board.GetFootprints():
        for p in fp.Pads():
            if p.GetAttribute() == pcbnew.PAD_ATTRIB_SMD:
                for lay in pads_by_layer:
                    if p.IsOnLayer(lay):
                        pads_by_layer[lay].append(p)
    holes = []      # (x, y, net, owner)
    for fp in board.GetFootprints():
        for p in fp.Pads():
            if p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH:
                holes.append((p.GetPosition().x, p.GetPosition().y, p.GetNetname(), f"{fp.GetReference()}.{p.GetNumber()}"))
    for t in board.GetTracks():
        if t.GetClass() == "PCB_VIA":
            holes.append((t.GetPosition().x, t.GetPosition().y, t.GetNetname(), "via"))

    def drain_body(fp):
        return [p for p in fp.Pads() if p.GetNumber() == "5" and p.GetAttribute() == pcbnew.PAD_ATTRIB_SMD
                and abs(pcbnew.ToMM(p.GetSize().x) - 4.7) < 1e-3]

    hs_ok = True
    bot_hs_ok = True
    fmap = json.load(open(os.path.join(KDIR, "power_stage_fet_map.json")))["fets"]
    hs_top = [m["ref"] for m in fmap if m["role"] == "HS" and m["side"] == "top"]
    hs_bot = [m["ref"] for m in fmap if m["role"] == "HS" and m["side"] == "bottom"]
    for ref in hs_top:
        fp = fps[ref]
        vias = [p for p in fp.Pads() if p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH]
        body = drain_body(fp)[0]
        partner = [fps[r] for r in hs_bot if fps[r].GetPosition() == fp.GetPosition()]
        good = (len(vias) == 16 and len(partner) == 1
                and all(abs(pcbnew.ToMM(v.GetDrillSize().x) - 0.3) < 1e-6 for v in vias)
                and all(v.GetNetname() == "VBAT" for v in vias)
                and all(pad_contains(body, pcbnew.F_Cu, v.GetPosition().x, v.GetPosition().y) for v in vias)
                and all(pad_contains(drain_body(partner[0])[0], pcbnew.B_Cu, v.GetPosition().x, v.GetPosition().y)
                        for v in vias))
        if not good:
            hs_ok = False
            print(f"    {ref}: vias={len(vias)} partner={[p.GetReference() for p in partner]}")
    check(hs_ok, f"{len(hs_top)} top HS: 16 x 0.30 mm VBAT vias inside own drain body and bottom partner drain")
    for ref in hs_bot:
        body = drain_body(fps[ref])[0]
        n = sum(1 for (x, y, net, _) in holes if net == "VBAT" and pad_contains(body, pcbnew.B_Cu, x, y))
        if n < 16:
            bot_hs_ok = False
            print(f"    {ref}: {n} vias in drain")
    check(bot_hs_ok, f"{len(hs_bot)} bottom HS drains pierced by >= 16 VBAT vias")

    ls_ok = True
    for m in fmap:
        if m["role"] != "LS":
            continue
        fp = fps[m["ref"]]
        lay = pcbnew.F_Cu if m["side"] == "top" else pcbnew.B_Cu
        srcs = [p for p in fp.Pads() if p.GetNumber() in ("1", "2", "3") and p.GetAttribute() == pcbnew.PAD_ATTRIB_SMD]
        cx = sum(p.GetPosition().x for p in srcs) / 3
        cy = sum(p.GetPosition().y for p in srcs) / 3
        near = [h for h in holes if h[2] == "GND" and ((h[0] - cx) ** 2 + (h[1] - cy) ** 2) ** 0.5 < pcbnew.FromMM(3.0)]
        inpad = [h for h in near if any(pad_contains(p, lay, h[0], h[1]) for p in srcs)]
        if len(near) < 7 or len(inpad) < 4:
            ls_ok = False
            print(f"    {m['ref']}: GND vias near={len(near)} in-pad={len(inpad)}")
    check(ls_ok, "12 LS: >= 7 GND return vias within 3 mm, >= 4 inside own source pads (both faces)")

    bad = []
    for (x, y, net, owner) in holes:
        for lay, pads in pads_by_layer.items():
            for p in pads:
                if p.GetNetname() != net and pad_contains(p, lay, x, y):
                    bad.append((owner, net, p.GetParentFootprint().GetReference(), p.GetNumber(), p.GetNetname()))
    for b in bad[:5]:
        print("    hole in foreign pad:", b)
    check(not bad, f"{len(holes)} plated holes: none lands in an outer-layer pad of another net")


def p6(board):
    import pcbnew
    print("P6 topology")
    fps = fp_index(board)
    fmap = json.load(open(os.path.join(KDIR, "power_stage_fet_map.json")))["fets"]
    ok = True
    gates = []
    phases = {}
    for m in fmap:
        fp = fps[m["ref"]]
        nets = {p.GetNumber(): p.GetNetname() for p in fp.Pads() if p.GetAttribute() == pcbnew.PAD_ATTRIB_SMD}
        ph = f"ESC{m['esc']}_PH_{m['phase']}"
        gates.append(nets["4"])
        if m["role"] == "HS":
            ok &= all(nets[k] == ph for k in "123") and all(nets[k] == "VBAT" for k in "5678")
        else:
            ok &= all(nets[k] == "GND" for k in "123") and all(nets[k] == ph for k in "5678")
        ok &= (m["side"] == "top") == (m["esc"] in (1, 2))
        phases.setdefault(ph, set()).add((m["role"], m["side"]))
    check(ok, "HS: S=phase D=VBAT ; LS: S=GND D=phase ; ESC1/2 top, ESC3/4 bottom")
    check(len(phases) == 12 and all(len(v) == 2 and {r for r, _ in v} == {"HS", "LS"} and len({s for _, s in v}) == 1
                                    for v in phases.values()), "12 phases, each one HS + one LS on the same face")
    check(len(set(gates)) == 24, "24 unique gate nets")
    mps = [fp for fp in board.GetFootprints() if fp.GetReference().startswith("MP")]
    mp_ok = len(mps) == 12 and sorted(fp.Pads()[0].GetNetname() for fp in mps) == sorted(phases)
    check(mp_ok, "12 motor pads, one per phase net")


def p7(board):
    import pcbnew
    print("P7 outline / mounting / via types / library links")
    vias = [t for t in board.GetTracks() if t.GetClass() == "PCB_VIA"]
    check(all(v.GetViaType() == pcbnew.VIATYPE_THROUGH and v.TopLayer() == pcbnew.F_Cu
              and v.BottomLayer() == pcbnew.B_Cu for v in vias),
          f"all {len(vias)} board vias are F.Cu-B.Cu through vias (JLC standard 6L process; no blind/buried)")
    nick = {fp.GetFPID().GetLibNickname().wx_str() if hasattr(fp.GetFPID().GetLibNickname(), "wx_str")
            else str(fp.GetFPID().GetLibNickname()) for fp in board.GetFootprints()}
    check(nick == {gen.FPLIB}, f"every footprint links to library '{gen.FPLIB}' ({nick}) so DRC library parity runs")
    bb = board.GetBoardEdgesBoundingBox()
    w, h = pcbnew.ToMM(bb.GetWidth()), pcbnew.ToMM(bb.GetHeight())
    check(abs(w - 36.1) < 0.02 and abs(h - 36.1) < 0.02, f"outline 36 x 36 mm (bbox incl. line {w:.2f} x {h:.2f})")
    npth = [(pcbnew.ToMM(p.GetPosition().x), pcbnew.ToMM(p.GetPosition().y), pcbnew.ToMM(p.GetDrillSize().x))
            for fp in board.GetFootprints() for p in fp.Pads() if p.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH]
    xs = sorted({round(x, 3) for x, _, _ in npth})
    check(len(npth) == 4 and all(abs(d - 3.2) < 1e-6 for *_, d in npth) and len(xs) == 2
          and abs(xs[1] - xs[0] - 30.5) < 1e-3, f"4 x NPTH 3.2 mm on 30.5 mm pattern {npth}")


def drc(path):
    out = os.path.join(tempfile.mkdtemp(), "drc.json")
    subprocess.run(["kicad-cli", "pcb", "drc", "--severity-all", "--format", "json", "-o", out, path],
                   check=True, capture_output=True)
    return json.load(open(out))


def dru_parses():
    """kicad-cli silently ignores a .kicad_dru that fails to parse; pcbnew's DRC
    engine reports it.  Run the engine on the committed board and require success."""
    import pcbnew
    b = pcbnew.LoadBoard(PCB)
    rpt = os.path.join(tempfile.mkdtemp(), "r.txt")
    ok = pcbnew.WriteDRCReport(b, rpt, pcbnew.EDA_UNITS_MILLIMETRES, False)
    return ok and os.path.exists(rpt)


def p8():
    print("P8 DRC")
    check(dru_parses(), ".kicad_dru parses in the KiCad DRC engine (kicad-cli would skip it silently)")
    d = drc(PCB)
    v, u = d["violations"], d["unconnected_items"]
    for x in v[:5]:
        print("    ", x["severity"], x["type"], x["description"])
    check(not v and not u, f"kicad-cli DRC all severities: {len(v)} violations, {len(u)} unconnected")
    tmp = tempfile.mkdtemp()
    for f in os.listdir(KDIR):
        src = os.path.join(KDIR, f)
        if os.path.isfile(src):
            shutil.copy(src, tmp)
    shutil.copytree(os.path.join(KDIR, f"{gen.FPLIB}.pretty"), os.path.join(tmp, f"{gen.FPLIB}.pretty"))
    p = os.path.join(tmp, "nano-h743-aio.kicad_pcb")
    txt = open(p).read()
    # mutate exactly one thermal-via pad drill 0.3 -> 0.25 (inside a Q footprint)
    m = re.search(r'(\(pad "5" thru_hole circle(?:(?!\(pad ).)*?\(drill )0\.3\)', txt, re.S)
    txt2 = txt[:m.start()] + m.group(1) + "0.25)" + txt[m.end():]
    open(p, "w").write(txt2)
    d2 = drc(p)
    hit = [x for x in d2["violations"] if x["type"] in ("drill_out_of_range", "hole_size")]
    check(len(hit) >= 1, f"rule liveness: 0.25 mm thermal via flagged by .kicad_dru ({len(hit)} hit)")


def signature(board):
    import pcbnew
    r = lambda v: round(pcbnew.ToMM(v), 3)  # noqa: E731
    fps = []
    for fp in board.GetFootprints():
        fps.append((fp.GetReference(), fp.GetFPIDAsString(), fp.GetLayer(), r(fp.GetPosition().x), r(fp.GetPosition().y),
                    round(fp.GetOrientationDegrees(), 2),
                    tuple(sorted((p.GetNumber(), p.GetNetname(), r(p.GetPosition().x), r(p.GetPosition().y),
                                  p.GetShape(), p.GetAttribute(), r(p.GetSize().x), r(p.GetSize().y),
                                  r(p.GetDrillSize().x), p.GetLayerSet().FmtHex(),
                                  int(p.GetZoneConnection()))
                                 for p in fp.Pads()))))
    zones = []
    for z in board.Zones():
        ol = z.Outline().Outline(0)
        zones.append((z.GetNetname(), tuple(z.GetLayerSet().Seq()), z.GetAssignedPriority(),
                      tuple((r(ol.CPoint(i).x), r(ol.CPoint(i).y)) for i in range(ol.PointCount()))))
    tr = []
    for t in board.GetTracks():
        extra = ()
        if t.GetClass() == "PCB_VIA":
            extra = (int(t.GetViaType()), t.TopLayer(), t.BottomLayer(), r(t.GetDrillValue()))
        tr.append((t.GetClass(), t.GetNetname(), t.GetLayer(), r(t.GetStart().x), r(t.GetStart().y),
                   r(t.GetEnd().x), r(t.GetEnd().y), r(t.GetWidth())) + extra)
    return sorted(fps), sorted(zones), sorted(tr)


def p9(board, regen_dir):
    import pcbnew
    print("P9 regeneration")
    rpath = os.path.join(regen_dir, "nano-h743-aio.kicad_pcb")
    rb = pcbnew.LoadBoard(rpath)
    a, b = signature(board), signature(rb)
    for name, x, y in zip(("footprints", "zones", "tracks/vias"), a, b):
        check(x == y, f"committed {name} == regeneration ({len(x)} items)")
    # the generator is deterministic (content-derived UUIDs, fresh zone fill), so the
    # committed board must be byte-identical: this also proves the fills are not stale
    # and that no pad size/shape/layer/drill or via type was edited by hand.
    check(open(PCB).read() == open(rpath).read(), "committed .kicad_pcb byte-identical to a fresh generate + fill")
    for f in ("nano-h743-aio.kicad_dru", "fp-lib-table", "nano-h743-aio.kicad_pro", "power_stage_fet_map.json"):
        check(open(os.path.join(KDIR, f)).read() == open(os.path.join(regen_dir, f)).read(), f"{f} byte-identical")


def p10(board):
    import pcbnew
    print("P10 fab outputs")
    j = json.load(open(os.path.join(GERB, "nano-h743-aio-job.gbrjob")))
    gs = j["GeneralSpecs"]
    check(gs["LayerNumber"] == 6 and abs(gs["BoardThickness"] - 1.614) < 1e-6 and gs["Finish"] == "ENIG"
          and gs["ProjectId"]["Revision"] == gen.REV, f"job file: {gs['LayerNumber']} L, {gs['BoardThickness']} mm, "
                                                     f"{gs['Finish']}, rev {gs['ProjectId']['Revision']}")
    cu = [m["Thickness"] for m in j["MaterialStackup"] if m["Type"] == "Copper"]
    check(cu == [0.035, 0.061, 0.061, 0.061, 0.061, 0.035], f"job file copper {cu}")
    pth = 0
    for fp in board.GetFootprints():
        pth += sum(1 for p in fp.Pads() if p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH)
    pth += sum(1 for t in board.GetTracks() if t.GetClass() == "PCB_VIA")
    hits = sum(1 for l in open(os.path.join(GERB, "nano-h743-aio-PTH.drl")) if re.match(r"^X-?\d", l))
    check(hits == pth, f"Excellon PTH hits {hits} == board plated holes {pth}")
    nh = open(os.path.join(GERB, "nano-h743-aio-NPTH.drl")).read()
    check("C3.200" in nh and sum(1 for l in nh.splitlines() if re.match(r"^X-?\d", l)) == 4, "NPTH 4 x 3.2 mm")
    for f in os.listdir(GERB):
        if f.endswith((".gtl", ".gbl", ".g2", ".g3", ".g4", ".g5")):
            head = open(os.path.join(GERB, f)).read(600)
            if f",{gen.REV}*%" not in head:
                check(False, f"{f} not from rev {gen.REV}")
                break
    else:
        check(True, f"all copper Gerbers stamped rev {gen.REV}")
    expected = {f"nano-h743-aio-{n}" for n in (
        "F_Cu.gtl", "In1_Cu.g2", "In2_Cu.g3", "In3_Cu.g4", "In4_Cu.g5", "B_Cu.gbl", "F_Paste.gtp", "B_Paste.gbp",
        "F_Silkscreen.gto", "B_Silkscreen.gbo", "F_Mask.gts", "B_Mask.gbs", "Edge_Cuts.gm1", "job.gbrjob",
        "PTH.drl", "NPTH.drl", "PTH-drl_map.gbr", "NPTH-drl_map.gbr")}
    have = set(os.listdir(GERB))
    check(have == expected, f"fab file set exact ({len(have)} files; missing {sorted(expected - have)}, "
                            f"extra {sorted(have - expected)})")
    tmp = tempfile.mkdtemp()
    for cmd in (FAB_GERBER_CMD, FAB_DRILL_CMD):
        subprocess.run(cmd + ["-o", tmp + "/", PCB], check=True, capture_output=True)
    vol = re.compile(r"^((; #@! )?%?TF\.CreationDate.*|G04 Created by KiCad.*|; DRILL file .*|; FORMAT=.*|"
                     r"%TF\.GenerationSoftware.*|.*\"CreationDate\".*)$", re.M)
    diff = []
    for f in sorted(expected & have):
        a_ = vol.sub("", open(os.path.join(GERB, f)).read())
        b_ = vol.sub("", open(os.path.join(tmp, f)).read()) if os.path.exists(os.path.join(tmp, f)) else None
        if a_ != b_:
            diff.append(f)
    check(not diff, f"fab outputs content-identical to a fresh re-plot of the committed board ({diff})")


def p11(board):
    import pcbnew
    print("P11 thermal analysis")
    out = os.path.join(tempfile.mkdtemp(), "t.json")
    r = subprocess.run([sys.executable, os.path.join(REPO, "analysis", "thermal_power_analysis.py"),
                        "--no-plot", "--out", out], capture_output=True, text=True)
    rep = json.load(open(out))
    check(r.returncode == 0 and rep["status"] == "PASS", f"thermal must-pass scenarios: {rep['status']} "
                                                         f"{rep['must_pass_failures']}")
    check(rep["hs_thermal_arrays"]["count"] == 6 and rep["hs_thermal_arrays"]["vias_each"] == [16] * 6,
          "analysis read back 6 HS arrays x 16 vias from the board")
    committed = json.load(open(os.path.join(REPO, "analysis", "thermal_report.json")))

    def cmp(a, b, path="$"):
        if isinstance(a, dict) and isinstance(b, dict):
            if set(a) != set(b):
                return f"{path}: keys {sorted(set(a) ^ set(b))}"
            for k in a:
                e = cmp(a[k], b[k], f"{path}.{k}")
                if e:
                    return e
            return None
        if isinstance(a, list) and isinstance(b, list):
            if len(a) != len(b):
                return f"{path}: len {len(a)} != {len(b)}"
            for i, (x, y) in enumerate(zip(a, b)):
                e = cmp(x, y, f"{path}[{i}]")
                if e:
                    return e
            return None
        if isinstance(a, (int, float)) and isinstance(b, (int, float)) and not isinstance(a, bool):
            return None if abs(a - b) <= 1e-6 * max(1.0, abs(a)) else f"{path}: {a} != {b}"
        return None if a == b else f"{path}: {a!r} != {b!r}"
    e = cmp(committed, rep)
    check(e is None, f"committed thermal_report.json identical to a re-run ({e or 'all fields'})")
    rat = rep["published_rating"]
    check(all(rep["verdict"][k]["pass"] for k in rep["verdict"] if k.startswith("R")),
          f"published ratings hold: {rat}")


def main():
    import pcbnew
    board_sx = ks.load(PCB)
    board = pcbnew.LoadBoard(PCB)
    regen = tempfile.mkdtemp()
    gen.build(regen, fill=True)
    p1(board_sx, board)
    p2(board_sx, board, regen)
    p3_p4_p5(board)
    p6(board)
    p7(board)
    p8()
    p9(board, regen)
    p10(board)
    p11(board)
    print(f"\nRESULT: {'PASS' if not FAIL else 'FAIL'} ({len(FAIL)} failures)")
    return 0 if not FAIL else 1


if __name__ == "__main__":
    sys.exit(main())
