# KiCad 8 DRC Validation Test Suite (Cycle 4)

I don't have Cycles 1–3, so this is a self-contained suite. Thresholds are in one config file so you can set them to match earlier cycles or your fab's capabilities.

The suite uses two independent layers of checking:

1. **Native DRC.** It runs `kicad-cli pcb drc`, parses the JSON report, and fails on specific violation types.
2. **Independent geometric checks.** It loads the board with the `pcbnew` Python API and recomputes annular rings, copper-to-edge distances and drill aspect ratios.

The second layer is needed because KiCad 8 has no native aspect-ratio constraint. It also catches boards whose design rules were loosened to make DRC pass.

---

## Directory Layout

```
drc_tests/
├── drc_config.toml
├── rules/fab_cycle4.kicad_dru
├── conftest.py
├── geometry.py
├── test_native_drc.py
├── test_annular_ring.py
├── test_edge_clearance.py
├── test_aspect_ratio.py
└── .github/workflows/drc.yml
```

---

## `drc_config.toml`

```toml
[boards]
# Glob patterns relative to repo root
include = ["hardware/**/*.kicad_pcb"]
exclude = ["hardware/archive/**"]

[annular_ring]          # mm
via_min      = 0.125
pth_pad_min  = 0.150

[edge_clearance]        # mm, copper-to-Edge.Cuts
track_min    = 0.300
pad_min      = 0.300
via_min      = 0.300

[aspect_ratio]          # board_thickness / finished_drill
through_via_max = 10.0
pth_max         = 8.0
min_drill_mm    = 0.20

[native_drc]
# KiCad JSON violation "type" keys that must be zero
blocking_types = [
  "annular_width",
  "copper_edge_clearance",
  "drill_out_of_range",
  "via_diameter",
  "hole_clearance",
  "unconnected_items",
]
schematic_parity = false
```

---

## `rules/fab_cycle4.kicad_dru`

Copy this file next to your `.kicad_pcb` (same basename) so native DRC enforces the same values.

```lisp
(version 1)

(rule "Cycle4 via annular ring"
  (condition "A.Type == 'Via'")
  (constraint annular_width (min 0.125mm)))

(rule "Cycle4 PTH annular ring"
  (condition "A.Type == 'Pad' && A.Pad_Type == 'Through-hole'")
  (constraint annular_width (min 0.15mm)))

(rule "Cycle4 copper to edge"
  (constraint edge_clearance (min 0.3mm)))

(rule "Cycle4 min drill"
  (constraint hole_size (min 0.2mm)))
```

---

## `conftest.py`

```python
import fnmatch
import json
import shutil
import subprocess
import tomllib
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
CFG = tomllib.loads((Path(__file__).parent / "drc_config.toml").read_text())


def _discover_boards():
    inc, exc = CFG["boards"]["include"], CFG["boards"]["exclude"]
    found = set()
    for pat in inc:
        found.update(ROOT.glob(pat))
    return sorted(
        p for p in found
        if not any(fnmatch.fnmatch(str(p.relative_to(ROOT)), e) for e in exc)
    )


BOARDS = _discover_boards()


def pytest_generate_tests(metafunc):
    if "board_path" in metafunc.fixturenames:
        metafunc.parametrize(
            "board_path", BOARDS,
            ids=[str(b.relative_to(ROOT)) for b in BOARDS],
            scope="module",
        )


@pytest.fixture(scope="session")
def cfg():
    return CFG


@pytest.fixture(scope="module")
def board(board_path):
    pcbnew = pytest.importorskip("pcbnew")
    return pcbnew.LoadBoard(str(board_path))


@pytest.fixture(scope="module")
def drc_report(board_path, tmp_path_factory):
    cli = shutil.which("kicad-cli")
    if not cli:
        pytest.skip("kicad-cli not on PATH")
    out = tmp_path_factory.mktemp("drc") / f"{board_path.stem}.json"
    cmd = [cli, "pcb", "drc", "--format", "json", "--severity-all",
           "--units", "mm", "-o", str(out), str(board_path)]
    if CFG["native_drc"]["schematic_parity"]:
        cmd.insert(3, "--schematic-parity")
    proc = subprocess.run(cmd, capture_output=True, text=True)
    # Non-zero is expected only with --exit-code-violations; any other failure is real
    if not out.exists():
        pytest.fail(f"kicad-cli DRC failed:\n{proc.stdout}\n{proc.stderr}")
    return json.loads(out.read_text())
```

---

## `geometry.py`

This module holds pure-Python geometry helpers. All coordinates are KiCad internal nanometres.

```python
import math

import pcbnew


def nm_to_mm(v):
    return v / 1e6


def _pt_seg(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 == 0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _ccw(a, b, c):
    return (c[1] - a[1]) * (b[0] - a[0]) - (b[1] - a[1]) * (c[0] - a[0])


def _intersects(a, b, c, d):
    d1, d2 = _ccw(c, d, a), _ccw(c, d, b)
    d3, d4 = _ccw(a, b, c), _ccw(a, b, d)
    return (d1 * d2 < 0) and (d3 * d4 < 0)


def seg_seg(a, b, c, d):
    if _intersects(a, b, c, d):
        return 0.0
    return min(_pt_seg(*a, *c, *d), _pt_seg(*b, *c, *d),
               _pt_seg(*c, *a, *b), _pt_seg(*d, *a, *b))


def _chain_segments(chain):
    n = chain.PointCount()
    pts = [(chain.CPoint(i).x, chain.CPoint(i).y) for i in range(n)]
    return [(pts[i], pts[(i + 1) % n]) for i in range(n)]


def board_edge_segments(board):
    """All Edge.Cuts outline + cutout segments as ((x,y),(x,y))."""
    poly = pcbnew.SHAPE_POLY_SET()
    if not board.GetBoardPolygonOutlines(poly):
        raise RuntimeError("Board outline is not closed / invalid")
    segs = []
    for i in range(poly.OutlineCount()):
        segs += _chain_segments(poly.Outline(i))
        for h in range(poly.HoleCount(i)):
            segs += _chain_segments(poly.Hole(i, h))
    return segs


def min_dist_to_edges(item_segs, edges):
    return min(seg_seg(a, b, c, d) for (a, b) in item_segs for (c, d) in edges)


def pad_polygon_segments(pad, layer):
    try:
        shape = pad.GetEffectivePolygon(pcbnew.ERROR_INSIDE)
    except TypeError:  # older binding signature
        shape = pad.GetEffectivePolygon()
    segs = []
    for i in range(shape.OutlineCount()):
        segs += _chain_segments(shape.Outline(i))
    return segs
```

---

## `test_native_drc.py`

```python
from collections import Counter


def test_drc_report_has_no_blocking_violations(drc_report, cfg):
    blocking = set(cfg["native_drc"]["blocking_types"])
    hits = [v for v in drc_report.get("violations", [])
            if v.get("type") in blocking and v.get("severity") == "error"]
    hits += [v for v in drc_report.get("unconnected_items", [])
             if "unconnected_items" in blocking]
    if hits:
        summary = Counter(v.get("type") for v in hits)
        detail = "\n".join(
            f"  [{v.get('type')}] {v.get('description')} @ "
            + ", ".join(f"({i['pos']['x']:.3f},{i['pos']['y']:.3f})"
                        for i in v.get("items", []))
            for v in hits[:25])
        raise AssertionError(f"Native DRC: {dict(summary)}\n{detail}")


def test_custom_rules_file_present(board_path):
    dru = board_path.with_suffix(".kicad_dru")
    assert dru.exists(), f"Missing {dru.name}; copy rules/fab_cycle4.kicad_dru"
    assert "Cycle4" in dru.read_text(), "Rules file is not the Cycle 4 ruleset"
```

---

## `test_annular_ring.py`

```python
import pcbnew

from geometry import nm_to_mm


def _via_annular(via):
    return (via.GetWidth(pcbnew.F_Cu) - via.GetDrillValue()) / 2


def _pad_annular(pad):
    size, drill = pad.GetSize(), pad.GetDrillSize()
    return min(size.x - drill.x, size.y - drill.y) / 2


def test_via_annular_ring(board, cfg):
    lim = cfg["annular_ring"]["via_min"]
    bad = []
    for t in board.GetTracks():
        if isinstance(t, pcbnew.PCB_VIA):
            ar = nm_to_mm(_via_annular(t))
            if ar + 1e-6 < lim:
                p = t.GetPosition()
                bad.append(f"via @({nm_to_mm(p.x):.3f},{nm_to_mm(p.y):.3f}) "
                           f"AR={ar:.3f}mm net={t.GetNetname()}")
    assert not bad, f"{len(bad)} vias below {lim}mm:\n" + "\n".join(bad[:25])


def test_pth_pad_annular_ring(board, cfg):
    lim = cfg["annular_ring"]["pth_pad_min"]
    bad = []
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetAttribute() != pcbnew.PAD_ATTRIB_PTH:
                continue
            ar = nm_to_mm(_pad_annular(pad))
            if ar + 1e-6 < lim:
                bad.append(f"{fp.GetReference()}.{pad.GetNumber()} AR={ar:.3f}mm")
    assert not bad, f"{len(bad)} PTH pads below {lim}mm:\n" + "\n".join(bad[:25])
```

---

## `test_edge_clearance.py`

```python
import pytest
import pcbnew

from geometry import (board_edge_segments, min_dist_to_edges,
                      nm_to_mm, pad_polygon_segments)


@pytest.fixture(scope="module")
def edges(board):
    return board_edge_segments(board)


def _xy(p):
    return (p.x, p.y)


def test_track_to_edge(board, edges, cfg):
    lim = cfg["edge_clearance"]["track_min"]
    bad = []
    for t in board.GetTracks():
        if isinstance(t, pcbnew.PCB_VIA):
            continue
        if isinstance(t, pcbnew.PCB_ARC):
            segs = [(_xy(t.GetStart()), _xy(t.GetMid())),
                    (_xy(t.GetMid()), _xy(t.GetEnd()))]
        else:
            segs = [(_xy(t.GetStart()), _xy(t.GetEnd()))]
        d = nm_to_mm(min_dist_to_edges(segs, edges) - t.GetWidth() / 2)
        if d + 1e-6 < lim:
            bad.append(f"track {t.GetNetname()} on {t.GetLayerName()}: {d:.3f}mm")
    assert not bad, f"{len(bad)} tracks < {lim}mm from edge:\n" + "\n".join(bad[:25])


def test_via_to_edge(board, edges, cfg):
    lim = cfg["edge_clearance"]["via_min"]
    bad = []
    for v in board.GetTracks():
        if not isinstance(v, pcbnew.PCB_VIA):
            continue
        c = _xy(v.GetPosition())
        d = nm_to_mm(min_dist_to_edges([(c, c)], edges)
                     - v.GetWidth(pcbnew.F_Cu) / 2)
        if d + 1e-6 < lim:
            bad.append(f"via {v.GetNetname()} @{c}: {d:.