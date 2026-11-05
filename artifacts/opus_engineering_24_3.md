# KiCad 8 DRC Validation Test Suite

I don't have the specifics of your "Cycle 24" (fab-house limits, board list, or prior-cycle findings). This suite therefore uses a parameterized threshold file. Set the values in `drc_limits.yaml` to match your Cycle 24 fabrication spec.

The suite uses two layers of checking:

1. **Native DRC** through `kicad-cli pcb drc` with JSON output. This covers annular ring and copper-to-edge clearance using KiCad's own engine.
2. **Independent geometry checks** through the `pcbnew` Python API. These re-verify the same rules against your fab limits and add **drill aspect ratio**, which KiCad 8 DRC does not check natively.

---

## Directory layout

```
drc_suite/
├── drc_limits.yaml
├── boards.txt
├── rules/cycle24.kicad_dru
├── conftest.py
├── helpers.py
├── test_native_drc.py
├── test_annular_ring.py
├── test_edge_clearance.py
├── test_aspect_ratio.py
└── .github/workflows/drc.yml
```

---

## `drc_limits.yaml`

```yaml
cycle: 24
units: mm

annular_ring:
  via_min: 0.125          # (via_dia - drill)/2
  pth_pad_min: 0.150
  allow_microvia_min: 0.075

edge_clearance:
  copper_min: 0.300       # copper to Edge.Cuts
  npth_to_copper_min: 0.200

aspect_ratio:
  through_max: 10.0       # board_thickness / finished_drill
  microvia_max: 0.8       # dielectric_depth / drill
  min_drill: 0.200

native_drc:
  fail_on_severity: [error]          # add "warning" to tighten
  required_clean_types:
    - annular_width
    - copper_edge_clearance
    - hole_clearance
    - drill_out_of_range
    - via_diameter
```

## `boards.txt`

```
hw/main_board/main_board.kicad_pcb
hw/io_module/io_module.kicad_pcb
```

---

## `rules/cycle24.kicad_dru`

These rules enforce the same limits inside KiCad, so designers see violations interactively. Append them to each board's `.kicad_dru`, or copy them there in CI.

```
(version 1)

(rule "C24 via annular ring"
  (condition "A.Type == 'Via' && A.Via_Type != 'Micro'")
  (constraint annular_width (min 0.125mm)))

(rule "C24 microvia annular ring"
  (condition "A.Via_Type == 'Micro'")
  (constraint annular_width (min 0.075mm)))

(rule "C24 PTH pad annular ring"
  (condition "A.Type == 'Pad' && A.Pad_Type == 'Through-hole'")
  (constraint annular_width (min 0.150mm)))

(rule "C24 copper to edge"
  (constraint edge_clearance (min 0.300mm)))

(rule "C24 min drill"
  (condition "A.isPlated()")
  (constraint hole_size (min 0.200mm)))
```

---

## `conftest.py`

```python
import json
import os
import pathlib
import subprocess

import pytest
import yaml

ROOT = pathlib.Path(__file__).parent
LIMITS = yaml.safe_load((ROOT / "drc_limits.yaml").read_text())


def _boards():
    env = os.environ.get("KICAD_BOARDS")
    lines = env.split(",") if env else (ROOT / "boards.txt").read_text().splitlines()
    return [pathlib.Path(l.strip()) for l in lines if l.strip() and not l.startswith("#")]


BOARDS = _boards()


def pytest_generate_tests(metafunc):
    if "board_path" in metafunc.fixturenames:
        metafunc.parametrize("board_path", BOARDS, ids=[b.stem for b in BOARDS])


@pytest.fixture(scope="session")
def limits():
    return LIMITS


@pytest.fixture(scope="session")
def _board_cache():
    return {}


@pytest.fixture
def board(board_path, _board_cache):
    import pcbnew
    key = str(board_path)
    if key not in _board_cache:
        assert board_path.exists(), f"Board not found: {board_path}"
        _board_cache[key] = pcbnew.LoadBoard(key)
    return _board_cache[key]


@pytest.fixture
def drc_report(board_path, tmp_path_factory):
    out = tmp_path_factory.mktemp("drc") / f"{board_path.stem}.json"
    cmd = [
        "kicad-cli", "pcb", "drc",
        "--format", "json",
        "--severity-all",
        "--units", "mm",
        "--output", str(out),
        str(board_path),
    ]
    res = subprocess.run(cmd, capture_output=True, text=True)
    # kicad-cli returns non-zero only with --exit-code-violations; rely on JSON
    assert out.exists(), f"kicad-cli failed:\n{res.stdout}\n{res.stderr}"
    return json.loads(out.read_text())
```

---

## `helpers.py`

```python
import pcbnew

nm_to_mm = pcbnew.ToMM


def fmt_pos(item):
    p = item.GetPosition()
    return f"({nm_to_mm(p.x):.3f}, {nm_to_mm(p.y):.3f})"


def vias(board):
    return [t for t in board.GetTracks() if isinstance(t, pcbnew.PCB_VIA)]


def pth_pads(board):
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetAttribute() == pcbnew.PAD_ATTRIB_PTH:
                yield fp, pad


def npth_pads(board):
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:
                yield fp, pad


def board_thickness_mm(board):
    return nm_to_mm(board.GetDesignSettings().GetBoardThickness())


def board_outline(board):
    poly = pcbnew.SHAPE_POLY_SET()
    ok = board.GetBoardPolygonOutlines(poly)
    assert ok, "Board outline (Edge.Cuts) is not closed"
    return poly


def outline_segments(poly):
    segs = []
    for oi in range(poly.OutlineCount()):
        chains = [poly.Outline(oi)] + [poly.Hole(oi, h) for h in range(poly.HoleCount(oi))]
        for ch in chains:
            for si in range(ch.SegmentCount()):
                segs.append(ch.CSegment(si))
    return segs


def min_dist_to_edge_mm(shape, segs):
    """Min distance (mm) from a SHAPE to any outline segment."""
    best = None
    for seg in segs:
        # SHAPE.Collide(SEG, clearance, &actual) is wrapped in KiCad 8 Python;
        # fall back to centre-distance if not available in your build.
        try:
            hit, actual = shape.Collide(seg, pcbnew.FromMM(50), True)
            d = actual if hit else None
        except TypeError:
            c = shape.Centre()
            d = seg.Distance(pcbnew.VECTOR2I(int(c.x), int(c.y)))
        if d is not None and (best is None or d < best):
            best = d
    return None if best is None else nm_to_mm(best)
```

---

## `test_native_drc.py`

```python
import pytest

RULE_TYPES = ["annular_width", "copper_edge_clearance", "hole_clearance",
              "drill_out_of_range", "via_diameter"]


def _fmt(v):
    locs = "; ".join(
        f"{i.get('description','?')} @ ({i['pos']['x']:.3f},{i['pos']['y']:.3f})"
        for i in v.get("items", [])
    )
    return f"[{v['severity']}] {v['type']}: {v['description']} -> {locs}"


def test_report_units_are_mm(drc_report):
    assert drc_report.get("coordinate_units", "mm") == "mm"


@pytest.mark.parametrize("rule_type", RULE_TYPES)
def test_native_rule_clean(drc_report, limits, rule_type):
    if rule_type not in limits["native_drc"]["required_clean_types"]:
        pytest.skip(f"{rule_type} not required this cycle")
    sev = set(limits["native_drc"]["fail_on_severity"])
    hits = [v for v in drc_report.get("violations", [])
            if v["type"] == rule_type and v["severity"] in sev]
    assert not hits, f"{len(hits)} '{rule_type}' violations:\n" + "\n".join(map(_fmt, hits))


def test_no_unconnected_items(drc_report):
    un = drc_report.get("unconnected_items", [])
    assert not un, f"{len(un)} unconnected items"
```

---

## `test_annular_ring.py`

```python
import pcbnew
from helpers import vias, pth_pads, nm_to_mm, fmt_pos


def test_via_annular_ring(board, limits):
    lim = limits["annular_ring"]
    bad = []
    for v in vias(board):
        ring = nm_to_mm(v.GetWidth() - v.GetDrillValue()) / 2
        is_micro = v.GetViaType() == pcbnew.VIATYPE_MICROVIA
        req = lim["allow_microvia_min"] if is_micro else lim["via_min"]
        if ring < req - 1e-6:
            bad.append(f"via {v.GetNetname()} @ {fmt_pos(v)} ring={ring:.3f} < {req}")
    assert not bad, "\n".join(bad)


def test_pth_pad_annular_ring(board, limits):
    req = limits["annular_ring"]["pth_pad_min"]
    bad = []
    for fp, pad in pth_pads(board):
        size, drill, off = pad.GetSize(), pad.GetDrillSize(), pad.GetOffset()
        # Offset drill shifts the hole toward one side, shrinking that ring.
        ring_x = nm_to_mm(size.x - drill.x) / 2 - abs(nm_to_mm(off.x))
        ring_y = nm_to_mm(size.y - drill.y) / 2 - abs(nm_to_mm(off.y))
        ring = min(ring_x, ring_y)
        if ring < req - 1e-6:
            bad.append(f"{fp.GetReference()}.{pad.GetNumber()} @ {fmt_pos(pad)} "
                       f"ring={ring:.3f} < {req}")
    assert not bad, "\n".join(bad)


def test_no_zero_or_negative_drill(board):
    bad = [f"via @ {fmt_pos(v)}" for v in vias(board) if v.GetDrillValue() <= 0]
    bad += [f"{fp.GetReference()}.{p.GetNumber()}"
            for fp, p in pth_pads(board) if min(p.GetDrillSize().x, p.GetDrillSize().y) <= 0]
    assert not bad, "Zero/negative drills:\n" + "\n".join(bad)
```

---

## `test_edge_clearance.py`

```python
import pcbnew
from helpers import (board_outline, outline_segments, min_dist_to_edge_mm,
                     vias, pth_pads, npth_pads, fmt_pos)


def _copper_items(board):
    for t in board.GetTracks():
        yield t, t.GetLayer() if not isinstance(t, pcbnew.PCB_VIA) else pcbnew.F_Cu
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetAttribute() != pcbnew.PAD_ATTRIB_NPTH:
                yield pad, pcbnew.F_Cu if pad.IsOnLayer(pcbnew.F_Cu) else pcbnew.B_Cu


def test_outline_is_closed(board):
    board_outline(board)  # asserts internally


def test_copper_to_edge(board, limits):
    req = limits["edge_clearance"]["copper_min"]
    segs = outline_segments(board_outline(board))
    bad = []
    for item, layer in _copper_items(board):
        d = min_dist_to_edge_mm(item.GetEffectiveShape(layer), segs)
        if d is not None and d < req - 1e-6:
            bad.append(f"{item.GetClass()} {item.GetNetname()} @ {fmt_pos(item)} "
                       f"edge={d:.3f} < {req}")
    assert not bad, f"{len(bad)} copper-to-edge violations:\n" + "\n".join(bad[:50])


def test_zone_fill_to_edge(board, limits):
    req = limits["edge_clearance"]["copper_min"]
    segs = outline_segments(board_outline(board))
    bad = []
    for zone in board.Zones():
        if zone.GetIsRuleArea():
            continue
        for layer in zone.GetLayerSet().CuStack():
            if not zone.HasFilledPolysForLayer(layer):
                continue
            d = min_dist_to_edge_mm(zone.GetEffectiveShape(layer), segs)
            if d is not None and d < req - 1e-6:
                bad.append(f"zone {zone.GetNetname()} L{layer} edge={d:.3f} < {req}")
    assert not bad, "\n".join(bad)
```

---

## `test_aspect_ratio.py`

```python
import pcbnew
from helpers import vias, pth_pads, board_thickness_mm, nm_to_mm, fmt_pos


def _dielectric_between(board, top, bot):
    """Sum of dielectric thickness between two copper layers (mm)."""
    stack = board.GetDesignSettings().GetStackupDescriptor()
    total, inside = 0.0, False
    for item in stack.GetList():
        if item.GetBrdLayerId() == top:
            inside = True
            continue
        if item.GetBrdLayerId() ==