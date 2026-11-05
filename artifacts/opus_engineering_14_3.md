# KiCad 8 DRC Validation Test Suite

I don't have any context for "Cycle 14," such as your fab's capability sheet, prior cycle thresholds, or board list. The suite below therefore puts **all limits in one config file**. Replace the placeholder values with your Cycle 14 numbers.

The suite checks each rule in two independent ways:

1. **Native DRC** (`kicad-cli pcb drc` JSON report) confirms that KiCad's own rule engine passes.
2. **Independent geometric checks** (`pcbnew` Python API) re-measure annular rings, copper-to-edge clearance, and drill aspect ratio directly. This catches cases where someone loosened the board's design rules to make DRC pass. Aspect ratio is not a native KiCad DRC check, so it is computed here.

---

## Layout

```
drc_suite/
├── drc_limits.yaml
├── conftest.py
├── lib/
│   ├── geometry.py
│   └── board_model.py
├── tests/
│   ├── test_native_drc.py
│   ├── test_annular_ring.py
│   ├── test_edge_clearance.py
│   ├── test_aspect_ratio.py
│   └── test_rules_not_relaxed.py
└── .github/workflows/drc.yml
```

---

## `drc_limits.yaml`

```yaml
# Cycle 14 limits: PLACEHOLDERS. Replace with your fab's capability values.
cycle: 14
units: mm

boards:
  - path: hardware/main/main.kicad_pcb
  - path: hardware/io/io.kicad_pcb

annular_ring:
  via_min: 0.125          # (pad_dia - drill)/2
  pth_pad_min: 0.150
  check_pad_both_axes: true

edge_clearance:
  copper_min: 0.300       # copper to Edge.Cuts
  include_zones: true
  include_pads: true
  include_tracks: true
  include_vias: true

aspect_ratio:
  through_hole_max: 10.0  # board_thickness / finished_drill
  microvia_max: 0.8       # dielectric depth / drill (blind/micro)
  default_board_thickness: 1.6   # used only if stackup missing

native_drc:
  # Violation types that must be zero
  blocking_types:
    - annular_width
    - copper_edge_clearance
    - hole_clearance
    - drill_out_of_range
    - via_diameter
    - microvia_drill_out_of_range
  allow_exclusions: false   # fail if a blocking violation was "excluded" in the GUI
  refill_zones: true
```

---

## `conftest.py`

```python
import json
import shutil
import subprocess
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).parent
LIMITS = yaml.safe_load((ROOT / "drc_limits.yaml").read_text())


def pytest_generate_tests(metafunc):
    if "board_path" in metafunc.fixturenames:
        paths = [ROOT.parent / b["path"] for b in LIMITS["boards"]]
        metafunc.parametrize("board_path", paths, ids=[p.stem for p in paths])


@pytest.fixture(scope="session")
def limits():
    return LIMITS


@pytest.fixture(scope="session")
def kicad_cli():
    exe = shutil.which("kicad-cli")
    if not exe:
        pytest.skip("kicad-cli not found on PATH")
    out = subprocess.run([exe, "--version"], capture_output=True, text=True).stdout
    assert out.strip().startswith("8."), f"Expected KiCad 8.x, got {out!r}"
    return exe


@pytest.fixture
def board_path(request):
    p = request.param
    assert p.exists(), f"Board not found: {p}"
    return p


@pytest.fixture
def drc_report(kicad_cli, board_path, tmp_path, limits):
    report = tmp_path / f"{board_path.stem}_drc.json"
    cmd = [kicad_cli, "pcb", "drc",
           "--format", "json",
           "--severity-all",
           "--units", "mm",
           "--all-track-errors",
           "-o", str(report)]
    if limits["native_drc"].get("refill_zones"):
        cmd.append("--refill-zones")
    cmd.append(str(board_path))
    res = subprocess.run(cmd, capture_output=True, text=True)
    assert report.exists(), f"DRC did not produce a report:\n{res.stderr}"
    return json.loads(report.read_text())


@pytest.fixture
def board(board_path):
    pcbnew = pytest.importorskip("pcbnew")
    b = pcbnew.LoadBoard(str(board_path))
    if LIMITS["native_drc"].get("refill_zones"):
        pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    return b
```

---

## `lib/geometry.py`

```python
import math


def seg_point_dist(ax, ay, bx, by, px, py):
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 == 0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _ccw(a, b, c):
    return (c[1] - a[1]) * (b[0] - a[0]) > (b[1] - a[1]) * (c[0] - a[0])


def segs_intersect(a, b, c, d):
    return _ccw(a, c, d) != _ccw(b, c, d) and _ccw(a, b, c) != _ccw(a, b, d)


def seg_seg_dist(a, b, c, d):
    if segs_intersect(a, b, c, d):
        return 0.0
    return min(
        seg_point_dist(*a, *b, *c), seg_point_dist(*a, *b, *d),
        seg_point_dist(*c, *d, *a), seg_point_dist(*c, *d, *b),
    )


def min_dist_point_to_edges(pt, edges):
    return min(seg_point_dist(*e[0], *e[1], *pt) for e in edges)


def min_dist_seg_to_edges(a, b, edges):
    return min(seg_seg_dist(a, b, e[0], e[1]) for e in edges)
```

---

## `lib/board_model.py`

This module extracts board data from `pcbnew` into plain Python structures, all in mm.

```python
import pcbnew

NM = 1e6  # nm per mm


def mm(v):
    return v / NM


def board_thickness_mm(board, default):
    t = board.GetDesignSettings().GetBoardThickness()
    return mm(t) if t > 0 else default


def outline_edges(board, arc_error_nm=5000):
    """Board outline (incl. cutouts) as list of ((x,y),(x,y)) in mm."""
    poly = pcbnew.SHAPE_POLY_SET()
    ok = board.GetBoardPolygonOutlines(poly)
    assert ok, "Edge.Cuts outline is not closed/valid"
    edges = []
    for oi in range(poly.OutlineCount()):
        chains = [poly.Outline(oi)] + [poly.Hole(oi, h) for h in range(poly.HoleCount(oi))]
        for ch in chains:
            n = ch.PointCount()
            for i in range(n):
                p, q = ch.CPoint(i), ch.CPoint((i + 1) % n)
                edges.append(((mm(p.x), mm(p.y)), (mm(q.x), mm(q.y))))
    return edges


def vias(board):
    out = []
    for t in board.GetTracks():
        if isinstance(t, pcbnew.PCB_VIA):
            p = t.GetPosition()
            out.append(dict(
                ref=f"via@({mm(p.x):.3f},{mm(p.y):.3f})",
                x=mm(p.x), y=mm(p.y),
                dia=mm(t.GetWidth()),
                drill=mm(t.GetDrillValue()),
                vtype=t.GetViaType(),
                top=t.TopLayer(), bottom=t.BottomLayer(),
            ))
    return out


def tracks(board):
    out = []
    for t in board.GetTracks():
        if type(t) is pcbnew.PCB_TRACK:  # excludes vias and arcs
            s, e = t.GetStart(), t.GetEnd()
            out.append(dict(
                ref=f"track {t.GetNetname()} {t.GetLayerName()}",
                a=(mm(s.x), mm(s.y)), b=(mm(e.x), mm(e.y)),
                w=mm(t.GetWidth()),
            ))
        elif isinstance(t, pcbnew.PCB_ARC):
            # approximate arc by its chord endpoints + mid
            s, m, e = t.GetStart(), t.GetMid(), t.GetEnd()
            for a, b in ((s, m), (m, e)):
                out.append(dict(
                    ref=f"arc {t.GetNetname()} {t.GetLayerName()}",
                    a=(mm(a.x), mm(a.y)), b=(mm(b.x), mm(b.y)),
                    w=mm(t.GetWidth()),
                ))
    return out


def plated_pads(board):
    out = []
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetAttribute() != pcbnew.PAD_ATTRIB_PTH:
                continue
            p = pad.GetPosition()
            size, drill = pad.GetSize(), pad.GetDrillSize()
            out.append(dict(
                ref=f"{fp.GetReference()}.{pad.GetNumber()}",
                x=mm(p.x), y=mm(p.y),
                sx=mm(size.x), sy=mm(size.y),
                dx=mm(drill.x), dy=mm(drill.y),
                shape=pad.GetShape(),
                offset=(mm(pad.GetOffset().x), mm(pad.GetOffset().y)),
            ))
    return out


def copper_pad_polys(board):
    """All copper pads (SMD+PTH) as polygon edge lists, for edge clearance."""
    out = []
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if not pad.IsOnCopperLayer():
                continue
            for layer in pad.GetLayerSet().CuStack():
                poly = pcbnew.SHAPE_POLY_SET()
                pad.TransformShapeToPolygon(poly, layer, 0, 5000, pcbnew.ERROR_INSIDE)
                out.append((f"{fp.GetReference()}.{pad.GetNumber()}", _poly_pts(poly)))
                break  # same shape on all layers for typical pads
    return out


def zone_fill_polys(board):
    out = []
    for z in board.Zones():
        if not z.IsOnCopperLayer():
            continue
        for layer in z.GetLayerSet().CuStack():
            fill = z.GetFilledPolysList(layer)
            if fill and fill.OutlineCount():
                out.append((f"zone {z.GetNetname()} {board.GetLayerName(layer)}",
                            _poly_pts(fill)))
    return out


def _poly_pts(poly):
    pts = []
    for oi in range(poly.OutlineCount()):
        ch = poly.Outline(oi)
        pts.extend((mm(ch.CPoint(i).x), mm(ch.CPoint(i).y)) for i in range(ch.PointCount()))
    return pts


def layer_depth_mm(board, top, bottom):
    """Dielectric depth between two copper layers from the stackup."""
    ds = board.GetDesignSettings()
    stack = ds.GetStackupDescriptor()
    depth, inside = 0.0, False
    for item in stack.GetList():
        lid = item.GetBrdLayerId()
        if lid == top:
            inside = True
            continue
        if inside:
            if lid == bottom:
                break
            depth += mm(item.GetThickness())
    return depth
```

---

## `tests/test_native_drc.py`

```python
from collections import Counter


def _blocking(report, limits):
    types = set(limits["native_drc"]["blocking_types"])
    allow_excl = limits["native_drc"]["allow_exclusions"]
    hits = []
    for v in report.get("violations", []) + report.get("unconnected_items", []):
        if v.get("type") not in types:
            continue
        if v.get("excluded") and allow_excl:
            continue
        hits.append(v)
    return hits


def test_native_drc_blocking_rules_clean(drc_report, limits, board_path):
    hits = _blocking(drc_report, limits)
    summary = Counter(v["type"] for v in hits)
    detail = "\n".join(
        f"  [{v['type']}] {v['description']} :: "
        + "; ".join(i.get("description", "") for i in v.get("items", []))
        for v in hits[:50]
    )
    assert not hits, f"{board_path.name}: {dict(summary)}\n{detail}"


def test_drc_report_is_kicad8(drc_report):
    assert drc_report.get("kicad_version", "").startswith("8."), drc_report.get("kicad_version")
```

---

## `tests/test_annular_ring.py`

```python
import pcbnew
import pytest
from lib.board_model import vias, plated_pads


def test_via_annular_ring(board, limits):
    vmin = limits["annular_ring"]["via_min"]
    bad = []
    for v in vias(board):
        ring = (v["dia"] - v["drill"]) / 2
        if ring + 1e-6 < vmin:
            bad.append(f"{v['ref']} dia={v['dia']:.3f} drill={v['drill']:.3f} ring={ring:.3f}")
    assert not bad, f"{len(bad)} vias below {vmin} mm annular ring:\n" + "\n".join(bad[:50])


def test_pth_pad_annular_ring(board, limits):
    pmin = limits["annular_ring"]["pth_pad_min"]
    both = limits["annular_ring"]["check_pad_both_axes"]
    bad = []
    for p in plated_pads(board):
        ox, oy = (abs(o) for o in p["offset"])
        # worst-case ring per axis, accounting for drill offset within pad
        ring_x = (p["sx"] - p["dx"]) / 2 - ox
        ring_y = (p["sy"] - p["dy"]) / 2 - oy
        ring = min(ring_x, ring_y) if both else max(ring_x, ring_y)
        if ring + 1e-6 