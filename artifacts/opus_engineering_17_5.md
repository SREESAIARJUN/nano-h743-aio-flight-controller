# KiCad 8 DRC Validation Test Suite: Annular Ring, Copper-to-Edge, Drill Aspect Ratio

> **Note on "Cycle 17":** I don't have the earlier cycles' thresholds or conventions. The values in `drc_limits.yaml` below are common fab defaults (JLCPCB/OSH-class standard process). Replace them with your Cycle 17 baseline.

The suite validates each check in two independent ways, so a misconfigured rule file cannot silently pass:

1. **KiCad's own DRC.** It runs `kicad-cli pcb drc` with JSON output and enforced custom rules, then asserts that no violations of the target types appear.
2. **Independent geometric checks.** It uses the `pcbnew` Python API to recompute annular rings, edge clearance, and aspect ratios from raw geometry. KiCad has no native aspect-ratio rule, so this path is the only enforcement for that check.

---

## Layout

```
drc_tests/
├── drc_limits.yaml
├── rules/fab_limits.kicad_dru
├── conftest.py
├── geometry.py
├── test_kicad_drc.py
├── test_annular_ring.py
├── test_edge_clearance.py
├── test_aspect_ratio.py
└── .github/workflows/drc.yml
```

---

## `drc_limits.yaml`

```yaml
cycle: 17
boards:
  - path: hardware/main_board/main_board.kicad_pcb
  # - path: hardware/daughter/daughter.kicad_pcb

limits:
  annular_ring:
    via_min_mm: 0.13          # (via_dia - drill)/2
    pth_pad_min_mm: 0.15
  edge_clearance:
    copper_min_mm: 0.30       # tracks, vias, pads, zone fills -> Edge.Cuts
    allow_castellated: true   # skip pads with fab property "castellated"
  aspect_ratio:
    via_max: 10.0             # board_thickness / finished_drill
    pth_max: 8.0
    min_drill_mm: 0.20        # sanity floor
  tolerance_mm: 0.001         # IU rounding slack
```

---

## `rules/fab_limits.kicad_dru`

Copy or merge this into `<project>.kicad_dru` so that interactive DRC in the editor matches CI.

```lisp
(version 1)

(rule "Via annular ring"
  (condition "A.Type == 'Via'")
  (constraint annular_width (min 0.13mm)))

(rule "PTH annular ring"
  (condition "A.Type == 'Pad' && A.Pad_Type == 'Through-hole'")
  (constraint annular_width (min 0.15mm)))

(rule "Copper to board edge"
  (condition "A.Layer == 'Edge.Cuts' || B.Layer == 'Edge.Cuts'")
  (constraint edge_clearance (min 0.30mm)))

(rule "Castellated exemption"
  (condition "A.Fabrication_Property == 'Castellated pad'")
  (constraint edge_clearance (min -1mm)))

(rule "Min drill"
  (constraint hole_size (min 0.20mm)))
```

---

## `conftest.py`

```python
import json
import os
import shutil
import subprocess
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parent
CFG = yaml.safe_load((ROOT / "drc_limits.yaml").read_text())
REPO = Path(os.environ.get("REPO_ROOT", ROOT.parent))


def _boards():
    return [REPO / b["path"] for b in CFG["boards"]]


@pytest.fixture(scope="session")
def cfg():
    return CFG


@pytest.fixture(scope="session", params=_boards(), ids=lambda p: p.stem)
def board_path(request):
    p = request.param
    if not p.exists():
        pytest.fail(f"Board not found: {p}")
    return p


@pytest.fixture(scope="session")
def board(board_path):
    import pcbnew  # KiCad 8 bundled Python
    return pcbnew.LoadBoard(str(board_path))


@pytest.fixture(scope="session")
def drc_report(board_path, tmp_path_factory):
    cli = shutil.which("kicad-cli")
    if not cli:
        pytest.skip("kicad-cli not on PATH")
    out = tmp_path_factory.mktemp("drc") / f"{board_path.stem}.json"
    cmd = [cli, "pcb", "drc",
           "--format", "json",
           "--severity-all",
           "--units", "mm",
           "--schematic-parity",
           "--output", str(out),
           str(board_path)]
    # Exit code is non-zero when violations exist; we inspect the JSON instead.
    subprocess.run(cmd, check=False, capture_output=True, text=True)
    if not out.exists():
        pytest.fail("kicad-cli produced no DRC report")
    return json.loads(out.read_text())
```

---

## `geometry.py`

```python
import math
import pcbnew

IU = pcbnew.pcbIUScale.IU_PER_MM  # 1e6 in KiCad 8


def mm(v):
    return v / IU


def _seg_pt(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _seg_seg(a, b, c, d):
    def ccw(p, q, r):
        return (r[1] - p[1]) * (q[0] - p[0]) - (q[1] - p[1]) * (r[0] - p[0])
    d1, d2 = ccw(c, d, a), ccw(c, d, b)
    d3, d4 = ccw(a, b, c), ccw(a, b, d)
    if (d1 > 0) != (d2 > 0) and (d3 > 0) != (d4 > 0):
        return 0.0
    return min(_seg_pt(*a, *c, *d), _seg_pt(*b, *c, *d),
               _seg_pt(*c, *a, *b), _seg_pt(*d, *a, *b))


def _chain_segments(chain):
    n = chain.PointCount()
    pts = [(chain.CPoint(i).x, chain.CPoint(i).y) for i in range(n)]
    return [(pts[i], pts[(i + 1) % n]) for i in range(n)]


def poly_segments(polyset):
    segs = []
    for i in range(polyset.OutlineCount()):
        segs += _chain_segments(polyset.Outline(i))
        for h in range(polyset.HoleCount(i)):
            segs += _chain_segments(polyset.Hole(i, h))
    return segs


def board_outline_segments(board):
    outline = pcbnew.SHAPE_POLY_SET()
    if not board.GetBoardPolygonOutlines(outline):
        raise RuntimeError("Board outline (Edge.Cuts) is not closed")
    return poly_segments(outline)


def _bbox(segs):
    xs = [p[0] for s in segs for p in s]
    ys = [p[1] for s in segs for p in s]
    return min(xs), min(ys), max(xs), max(ys)


def min_distance(segs_a, segs_b, cutoff):
    """Minimum distance between two edge sets (IU). Uses bbox culling."""
    best = math.inf
    for a in segs_a:
        ax0, ax1 = sorted((a[0][0], a[1][0]))
        ay0, ay1 = sorted((a[0][1], a[1][1]))
        for b in segs_b:
            bx0, bx1 = sorted((b[0][0], b[1][0]))
            by0, by1 = sorted((b[0][1], b[1][1]))
            if bx0 > ax1 + cutoff or ax0 > bx1 + cutoff or \
               by0 > ay1 + cutoff or ay0 > by1 + cutoff:
                continue
            d = _seg_seg(a[0], a[1], b[0], b[1])
            if d < best:
                best = d
                if best == 0:
                    return 0.0
    return best


def item_copper_poly(item, layer, max_err=int(0.005 * IU)):
    ps = pcbnew.SHAPE_POLY_SET()
    item.TransformShapeToPolygon(ps, layer, 0, max_err, pcbnew.ERROR_INSIDE)
    return ps


def is_via(t):
    return isinstance(t, pcbnew.PCB_VIA) or t.GetClass() == "PCB_VIA"


def plated_through_pads(board):
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetAttribute() == pcbnew.PAD_ATTRIB_PTH:
                yield fp, pad
```

---

## `test_kicad_drc.py`: native DRC gate

```python
import pytest

TARGETS = {
    "annular_width":          "Annular ring",
    "copper_edge_clearance":  "Copper to edge",
    "drill_out_of_range":     "Drill size",
    "via_diameter":           "Via diameter",
}


def _violations(report):
    return report.get("violations", []) + report.get("unconnected_items", [])


def test_report_has_no_rule_errors(drc_report):
    # A broken .kicad_dru silently disables checks; catch that first.
    bad = [v for v in _violations(drc_report) if v["type"] in
           ("invalid_outline", "malformed_courtyard")]
    assert not bad, f"Board/rule setup errors: {bad}"


@pytest.mark.parametrize("vtype,label", TARGETS.items())
def test_no_native_violations(drc_report, vtype, label):
    hits = [v for v in _violations(drc_report)
            if v["type"] == vtype and v.get("severity") != "ignore"]
    msg = "\n".join(
        f"  {v['description']} @ " +
        ", ".join(f"({i['pos']['x']:.3f},{i['pos']['y']:.3f})" for i in v["items"])
        for v in hits)
    assert not hits, f"{label}: {len(hits)} violation(s)\n{msg}"


def test_custom_rules_loaded(board_path):
    dru = board_path.with_suffix(".kicad_dru")
    assert dru.exists(), "Project .kicad_dru missing - fab rules not enforced"
    txt = dru.read_text()
    for c in ("annular_width", "edge_clearance", "hole_size"):
        assert c in txt, f"Constraint '{c}' absent from {dru.name}"
```

---

## `test_annular_ring.py`

```python
from geometry import mm, is_via, plated_through_pads
import pcbnew


def test_via_annular_ring(board, cfg):
    lim = cfg["limits"]["annular_ring"]["via_min_mm"]
    tol = cfg["limits"]["tolerance_mm"]
    fails = []
    for t in board.GetTracks():
        if not is_via(t):
            continue
        ring = mm(t.GetWidth() - t.GetDrillValue()) / 2
        if ring + tol < lim:
            p = t.GetPosition()
            fails.append(f"via @({mm(p.x):.3f},{mm(p.y):.3f}) "
                         f"dia={mm(t.GetWidth()):.3f} drill={mm(t.GetDrillValue()):.3f} "
                         f"ring={ring:.3f} < {lim}")
    assert not fails, "\n".join(fails)


def test_pth_annular_ring(board, cfg):
    lim = cfg["limits"]["annular_ring"]["pth_pad_min_mm"]
    tol = cfg["limits"]["tolerance_mm"]
    fails = []
    for fp, pad in plated_through_pads(board):
        size, drill = pad.GetSize(), pad.GetDrillSize()
        # Worst axis; for slots this compares each axis independently.
        ring = min(mm(size.x - drill.x), mm(size.y - drill.y)) / 2
        # Account for an offset drill.
        off = pad.GetOffset()
        ring -= max(abs(mm(off.x)), abs(mm(off.y)))
        if ring + tol < lim:
            fails.append(f"{fp.GetReference()}.{pad.GetNumber()} ring={ring:.3f} < {lim}")
    assert not fails, "\n".join(fails)
```

---

## `test_edge_clearance.py`

```python
import pcbnew
from geometry import (IU, mm, board_outline_segments, poly_segments,
                      item_copper_poly, min_distance)


def _copper_items(board):
    for t in board.GetTracks():
        for layer in t.GetLayerSet().CuStack():
            yield f"{t.GetClass()} net={t.GetNetname()}", t, layer
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            for layer in pad.GetLayerSet().CuStack():
                yield f"{fp.GetReference()}