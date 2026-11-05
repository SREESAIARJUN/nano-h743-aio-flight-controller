# KiCad 8 DRC Validation Test Suite: Annular Rings, Copper-to-Edge Clearance, Drill Aspect Ratio

I don't have the earlier cycles of this work, so this suite is self-contained. If Cycle 6 should build on specific thresholds, file layouts or fixtures from Cycles 1–5, share them and I'll align this suite to them.

## Approach

The suite validates each board in two independent ways and requires both to pass:

1. **KiCad's DRC engine.** It runs `kicad-cli pcb drc` once per board, saves the JSON report, and asserts on specific violation types.
2. **Independent geometry checks.** It uses the `pcbnew` Python API to recompute annular rings, edge clearances and aspect ratios. This catches cases where the board's own rule set was loosened. It is also the only way to check aspect ratio, because KiCad 8 has no native aspect-ratio constraint.

```
drc_suite/
├── drc_limits.yaml
├── rules/fab_baseline.kicad_dru
├── conftest.py
├── geometry.py
├── test_drc_engine.py
├── test_annular_ring.py
├── test_edge_clearance.py
├── test_aspect_ratio.py
└── .github/workflows/drc.yml
```

---

## `drc_limits.yaml`

This file holds the fab capability limits, which are the single source of truth for the suite.

```yaml
# All dimensions in mm
default:
  min_annular_ring_via: 0.125
  min_annular_ring_pth: 0.150
  min_copper_edge_clearance: 0.300
  max_aspect_ratio_via: 10.0     # board_thickness / finished_drill
  max_aspect_ratio_pth: 8.0
  min_drill: 0.200
  edge_tolerance: 0.001          # numeric slack for IU rounding

# Per-board overrides keyed by board file stem
overrides:
  hdi_controller:
    min_annular_ring_via: 0.100
    max_aspect_ratio_via: 12.0
```

## `rules/fab_baseline.kicad_dru`

These custom rules enforce the fab minimums inside KiCad's own DRC, in addition to the board setup values.

```lisp
(version 1)

(rule "fab_min_annular_via"
  (constraint annular_width (min 0.125mm))
  (condition "A.Type == 'Via'"))

(rule "fab_min_annular_pth"
  (constraint annular_width (min 0.15mm))
  (condition "A.Type == 'Pad' && A.Pad_Type == 'Through-hole'"))

(rule "fab_copper_edge"
  (constraint edge_clearance (min 0.3mm))
  (condition "A.isCopper()"))

(rule "fab_min_drill"
  (constraint hole_size (min 0.2mm)))
```

The CI workflow copies this file next to each board as `<board>.kicad_dru` only if the board doesn't already ship one. A board's own rules therefore take precedence, and the geometry tests still guard against those rules being too loose.

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

ROOT = Path(__file__).parent
LIMITS = yaml.safe_load((ROOT / "drc_limits.yaml").read_text())
BOARD_GLOB = os.environ.get("KICAD_BOARDS", "boards/**/*.kicad_pcb")
REPORT_DIR = Path(os.environ.get("DRC_REPORT_DIR", "drc_reports"))


def _boards():
    boards = sorted(Path(".").glob(BOARD_GLOB))
    if not boards:
        pytest.exit(f"No boards matched {BOARD_GLOB}", returncode=2)
    return boards


def pytest_generate_tests(metafunc):
    if "board_path" in metafunc.fixturenames:
        boards = _boards()
        metafunc.parametrize("board_path", boards, ids=[b.stem for b in boards],
                             scope="session")


@pytest.fixture(scope="session")
def limits(board_path):
    lim = dict(LIMITS["default"])
    lim.update(LIMITS.get("overrides", {}).get(board_path.stem, {}))
    return lim


@pytest.fixture(scope="session")
def drc_report(board_path):
    """Run kicad-cli DRC once per board and return the parsed JSON."""
    if shutil.which("kicad-cli") is None:
        pytest.skip("kicad-cli not on PATH")
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    out = REPORT_DIR / f"{board_path.stem}.drc.json"
    cmd = [
        "kicad-cli", "pcb", "drc",
        "--format", "json",
        "--units", "mm",
        "--severity-all",
        "--output", str(out),
        str(board_path),
    ]
    # Do not use --exit-code-violations here; the tests do the assertions.
    res = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
    if res.returncode != 0 or not out.exists():
        pytest.fail(f"kicad-cli DRC failed ({res.returncode}):\n{res.stderr}")
    return json.loads(out.read_text())


@pytest.fixture(scope="session")
def board(board_path):
    pcbnew = pytest.importorskip("pcbnew")
    return pcbnew.LoadBoard(str(board_path))
```

## `geometry.py`

These helpers extract dimensions from the board through the KiCad 8 `pcbnew` API.

```python
import math
import pcbnew

IU = pcbnew.FromMM(1)  # internal units per mm (1e6 in KiCad 8)


def mm(v):
    return v / IU


def board_thickness_mm(board):
    return mm(board.GetDesignSettings().GetBoardThickness())


def edge_segments(board):
    """Board outline as a list of ((x1,y1),(x2,y2)) segments in mm.

    Arcs are already polygonised by GetBoardPolygonOutlines.
    """
    poly = pcbnew.SHAPE_POLY_SET()
    if not board.GetBoardPolygonOutlines(poly):
        raise RuntimeError("Board outline is not closed / invalid")
    segs = []
    for oi in range(poly.OutlineCount()):
        chains = [poly.Outline(oi)] + [poly.Hole(oi, h) for h in range(poly.HoleCount(oi))]
        for ch in chains:
            n = ch.PointCount()
            pts = [(mm(ch.CPoint(i).x), mm(ch.CPoint(i).y)) for i in range(n)]
            segs += [(pts[i], pts[(i + 1) % n]) for i in range(n)]
    return segs


def pt_seg_dist(p, a, b):
    ax, ay = a; bx, by = b; px, py = p
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def seg_seg_dist(a1, a2, b1, b2):
    def ccw(A, B, C):
        return (C[1] - A[1]) * (B[0] - A[0]) - (B[1] - A[1]) * (C[0] - A[0])
    d1, d2 = ccw(b1, b2, a1), ccw(b1, b2, a2)
    d3, d4 = ccw(a1, a2, b1), ccw(a1, a2, b2)
    if (d1 * d2 < 0) and (d3 * d4 < 0):
        return 0.0
    return min(pt_seg_dist(a1, b1, b2), pt_seg_dist(a2, b1, b2),
               pt_seg_dist(b1, a1, a2), pt_seg_dist(b2, a1, a2))


def min_dist_to_edge(point_or_seg, segs):
    if isinstance(point_or_seg[0], tuple):
        a, b = point_or_seg
        return min(seg_seg_dist(a, b, s0, s1) for s0, s1 in segs)
    return min(pt_seg_dist(point_or_seg, s0, s1) for s0, s1 in segs)


def pos(item):
    p = item.GetPosition()
    return (mm(p.x), mm(p.y))


def ref(item):
    """Human-readable locator for failure messages."""
    x, y = pos(item)
    if isinstance(item, pcbnew.PAD):
        return f"{item.GetParentFootprint().GetReference()}.{item.GetNumber()} @({x:.3f},{y:.3f})"
    return f"{item.GetClass()} @({x:.3f},{y:.3f})"


def vias(board):
    return [t for t in board.GetTracks() if isinstance(t, pcbnew.PCB_VIA)]


def tracks(board):
    return [t for t in board.GetTracks()
            if t.GetClass() == "PCB_TRACK"]  # excludes vias and arcs


def pth_pads(board):
    return [p for p in board.GetPads()
            if p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH and p.GetDrillSize().x > 0]
```

---

## `test_drc_engine.py`

These tests assert on the violation categories reported by KiCad's DRC engine.

```python
import pytest

TARGET_TYPES = {
    "annular_width": "Annular ring below minimum",
    "copper_edge_clearance": "Copper too close to board edge",
    "hole_size": "Drill below minimum",
    "invalid_outline": "Board outline invalid (edge checks unreliable)",
}


def _fmt(v):
    locs = "; ".join(
        f"{i.get('description', '')} @({i['pos']['x']:.3f},{i['pos']['y']:.3f})"
        for i in v.get("items", []))
    return f"[{v['severity']}] {v['description']} -> {locs}"


@pytest.mark.parametrize("vtype", list(TARGET_TYPES))
def test_no_engine_violations(drc_report, vtype):
    hits = [v for v in drc_report.get("violations", [])
            if v["type"] == vtype and v["severity"] in ("error", "warning")]
    assert not hits, (f"{TARGET_TYPES[vtype]}: {len(hits)} violation(s)\n" +
                      "\n".join(_fmt(v) for v in hits[:25]))


def test_rule_checks_not_excluded(drc_report):
    """Guard against target checks being silenced via exclusions or severity=ignore."""
    excluded = [v for v in drc_report.get("violations", [])
                if v["type"] in TARGET_TYPES and v.get("excluded")]
    assert not excluded, f"{len(excluded)} target violations are excluded in the board file"
```

## `test_annular_ring.py`

```python
import pytest
from geometry import mm, vias, pth_pads, ref


def test_via_annular_ring(board, limits):
    lim = limits["min_annular_ring_via"] - limits["edge_tolerance"]
    bad = []
    for v in vias(board):
        ring = (mm(v.GetWidth()) - mm(v.GetDrillValue())) / 2  # KiCad 8 API
        if ring < lim:
            bad.append(f"{ref(v)} ring={ring:.3f}mm")
    assert not bad, f"Via annular ring < {limits['min_annular_ring_via']}mm:\n" + "\n".join(bad[:50])


def test_pth_pad_annular_ring(board, limits):
    lim = limits["min_annular_ring_pth"] - limits["edge_tolerance"]
    bad = []
    for p in pth_pads(board):
        size, drill = p.GetSize(), p.GetDrillSize()
        # Worst case: narrowest copper dimension vs. largest drill dimension.
        # This is conservative for slots and offset drills.
        ring = (mm(min(size.x, size.y)) - mm(max(drill.x, drill.y))) / 2
        off = p.GetOffset()
        ring -= max(abs(mm(off.x)), abs(mm(off.y)))
        if ring < lim:
            bad.append(f"{ref(p)} ring={ring:.3f}mm")
    assert not bad, f"PTH annular ring < {limits['min_annular_ring_pth']}mm:\n" + "\n".join(bad[:50])


def test_min_drill(board, limits):
    bad = [f"{ref(v)} drill={mm(v.GetDrillValue()):.3f}"
           for v in vias(board) if mm(v.GetDrillValue()) < limits["min_drill"]]
    bad += [f"{ref(p)} drill={mm(min(p.GetDrillSize().x, p.GetDrillSize().y)):.3f}"
            for p in pth_pads(board)
            if mm(min(p.GetDrillSize().x, p.GetDrillSize().y)) < limits["min_drill"]]
    assert not bad, "Drill below fab minimum:\n" + "\n".join(bad[:50])
```

## `test_edge_clearance.py`

```python
import pytest
from geometry import edge_segments, min_dist_to_edge, mm, pos, vias, tracks, ref, pcbnew


@pytest.fixture(scope="module")
def edges(board):
    return edge_segments(board)


def _check(items, edges, limit, radius_fn, geom_fn):
    bad = []
    for it in items:
        d = min_dist_to_edge(geom_fn(it), edges) - radius_fn(it)
        if d < limit:
            bad.append(f"{ref(it)} clearance={d:.3f}mm")
    return bad


def test_via_edge_clearance(board, edges, limits):
    lim = limits["min_copper_edge_clearance"] - limits["edge_tolerance"]
    bad = _check(vias(board), edges, lim, lambda v: mm(v.