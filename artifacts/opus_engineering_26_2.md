# KiCad 8 DRC Validation Test Suite

I don't have context for "Cycle 26" or earlier iterations, so this is a self-contained suite. Thresholds are centralized in one config file. Adjust them to your fab's capabilities or to whatever limits Cycle 26 requires.

## Approach

The suite runs two independent layers of checks:

1. **Native DRC via `kicad-cli`**, enforced by a custom `.kicad_dru` rules file. This uses KiCad's own geometry engine.
2. **Independent geometric checks via the `pcbnew` Python API.** These cross-check the native DRC. They also cover drill aspect ratio, which KiCad DRC cannot express natively.

## Layout

```
drc_suite/
├── drc_limits.yaml
├── fab_rules.kicad_dru
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
# All dimensions in mm
annular_ring:
  via_min: 0.125
  pth_pad_min: 0.150
copper_to_edge:
  min: 0.300
  include_zones: true
aspect_ratio:
  plated_max: 10.0        # board_thickness / finished_drill
  npth_max: 12.0
  microvia_max: 1.0       # dielectric depth / drill (blind/micro vias)
tolerance: 0.001          # numerical slop for float compares
```

## `fab_rules.kicad_dru`

Copy this file next to the `.kicad_pcb` as `<board>.kicad_dru`, or let the fixture do it for you.

```lisp
(version 1)

(rule "Via annular ring"
  (condition "A.Type == 'Via'")
  (constraint annular_width (min 0.125mm)))

(rule "PTH pad annular ring"
  (condition "A.Type == 'Pad' && A.Pad_Type == 'Through-hole'")
  (constraint annular_width (min 0.15mm)))

(rule "Copper to board edge"
  (constraint edge_clearance (min 0.3mm)))

(rule "Minimum drill"
  (constraint hole_size (min 0.2mm)))
```

---

## `helpers.py`

```python
import json
import math
import shutil
import subprocess
from pathlib import Path

import pcbnew
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union

NM_PER_MM = 1_000_000


def mm(nm: int) -> float:
    return nm / NM_PER_MM


def run_kicad_drc(board: Path, out_json: Path) -> dict:
    cli = shutil.which("kicad-cli")
    if not cli:
        raise RuntimeError("kicad-cli not found on PATH")
    cmd = [
        cli, "pcb", "drc",
        "--format", "json",
        "--severity-all",
        "--units", "mm",
        "-o", str(out_json),
        str(board),
    ]
    # Exit code is non-zero only for real failures (we don't pass --exit-code-violations)
    subprocess.run(cmd, check=True, capture_output=True, text=True)
    return json.loads(out_json.read_text())


def board_outline(board: "pcbnew.BOARD"):
    """Return the board outline as a shapely boundary (MultiLineString/LineString)."""
    polyset = pcbnew.SHAPE_POLY_SET()
    if not board.GetBoardPolygonOutlines(polyset):
        raise ValueError("Board outline (Edge.Cuts) is not closed/valid")
    polys = []
    for i in range(polyset.OutlineCount()):
        outline = polyset.Outline(i)
        pts = [(mm(outline.CPoint(j).x), mm(outline.CPoint(j).y))
               for j in range(outline.PointCount())]
        holes = []
        for h in range(polyset.HoleCount(i)):
            hole = polyset.Hole(i, h)
            holes.append([(mm(hole.CPoint(k).x), mm(hole.CPoint(k).y))
                          for k in range(hole.PointCount())])
        polys.append(Polygon(pts, holes))
    return unary_union(polys).boundary


def polyset_to_shapely(polyset: "pcbnew.SHAPE_POLY_SET"):
    polys = []
    for i in range(polyset.OutlineCount()):
        o = polyset.Outline(i)
        pts = [(mm(o.CPoint(j).x), mm(o.CPoint(j).y)) for j in range(o.PointCount())]
        if len(pts) >= 3:
            polys.append(Polygon(pts))
    return unary_union(polys) if polys else None


def copper_items(board: "pcbnew.BOARD", include_zones: bool):
    """Yield (description, shapely_geometry) for every copper object."""
    max_err = board.GetDesignSettings().m_MaxError

    for t in board.GetTracks():
        if isinstance(t, pcbnew.PCB_VIA):
            c = t.GetPosition()
            geom = Point(mm(c.x), mm(c.y)).buffer(mm(t.GetWidth()) / 2)
            yield f"Via @ ({mm(c.x):.3f},{mm(c.y):.3f})", geom
        elif isinstance(t, pcbnew.PCB_ARC):
            # Approximate arc by polygonizing
            ps = pcbnew.SHAPE_POLY_SET()
            t.TransformShapeToPolygon(ps, t.GetLayer(), 0, max_err, pcbnew.ERROR_INSIDE)
            yield f"Arc on {t.GetLayerName()}", polyset_to_shapely(ps)
        else:
            s, e = t.GetStart(), t.GetEnd()
            line = LineString([(mm(s.x), mm(s.y)), (mm(e.x), mm(e.y))])
            yield (f"Track {t.GetLayerName()} ({mm(s.x):.3f},{mm(s.y):.3f})",
                   line.buffer(mm(t.GetWidth()) / 2))

    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:
                continue
            for layer in pad.GetLayerSet().CuStack():
                ps = pcbnew.SHAPE_POLY_SET()
                pad.TransformShapeToPolygon(ps, layer, 0, max_err, pcbnew.ERROR_INSIDE)
                g = polyset_to_shapely(ps)
                if g is not None:
                    yield f"{fp.GetReference()}.{pad.GetNumber()}", g
                break  # one copper layer is enough for edge distance

    if include_zones:
        for zone in board.Zones():
            if zone.GetIsRuleArea():
                continue
            for layer in zone.GetLayerSet().CuStack():
                fill = zone.GetFilledPolysList(layer)
                g = polyset_to_shapely(fill) if fill else None
                if g is not None and not g.is_empty:
                    yield f"Zone '{zone.GetZoneName()}' {board.GetLayerName(layer)}", g


def via_span_thickness(board: "pcbnew.BOARD", via: "pcbnew.PCB_VIA") -> float:
    """Dielectric depth spanned by a via, in mm.

    Blind/micro vias use the stackup; through vias use the full board thickness.
    """
    ds = board.GetDesignSettings()
    if via.GetViaType() == pcbnew.VIATYPE_THROUGH:
        return mm(ds.GetBoardThickness())
    top, bot = via.TopLayer(), via.BottomLayer()
    stackup = ds.GetStackupDescriptor()
    depth, inside = 0, False
    for item in stackup.GetList():
        if item.GetBrdLayerId() == top:
            inside = True
        if inside and item.GetType() == pcbnew.BS_ITEM_TYPE_DIELECTRIC:
            depth += item.GetThickness()
        if item.GetBrdLayerId() == bot:
            break
    # Fall back to a proportional estimate if the stackup is unpopulated
    if depth == 0:
        n = board.GetCopperLayerCount()
        span = abs(bot - top) / 2 if n > 1 else 1  # Cu layer IDs step by 2 in KiCad 8
        return mm(ds.GetBoardThickness()) * span / max(n - 1, 1)
    return mm(depth)
```

## `conftest.py`

```python
import os
import shutil
from pathlib import Path

import pcbnew
import pytest
import yaml

from helpers import run_kicad_drc

SUITE_DIR = Path(__file__).parent


def pytest_addoption(parser):
    parser.addoption("--boards", default=os.environ.get("KICAD_BOARDS", "."),
                     help="Directory (searched recursively) or single .kicad_pcb file")
    parser.addoption("--inject-rules", action="store_true",
                     help="Copy fab_rules.kicad_dru next to each board before DRC")


def _boards(config):
    p = Path(config.getoption("--boards"))
    return [p] if p.suffix == ".kicad_pcb" else sorted(p.rglob("*.kicad_pcb"))


def pytest_generate_tests(metafunc):
    if "board_path" in metafunc.fixturenames:
        boards = _boards(metafunc.config)
        metafunc.parametrize("board_path", boards, ids=[b.stem for b in boards])


@pytest.fixture(scope="session")
def limits():
    return yaml.safe_load((SUITE_DIR / "drc_limits.yaml").read_text())


@pytest.fixture
def board(board_path):
    return pcbnew.LoadBoard(str(board_path))


@pytest.fixture
def drc_report(board_path, tmp_path, request):
    if request.config.getoption("--inject-rules"):
        dst = board_path.with_suffix(".kicad_dru")
        if not dst.exists():
            shutil.copy(SUITE_DIR / "fab_rules.kicad_dru", dst)
    return run_kicad_drc(board_path, tmp_path / f"{board_path.stem}_drc.json")
```

---

## `test_native_drc.py`

```python
import pytest

CHECKS = {
    "annular_width": "Annular ring",
    "copper_edge_clearance": "Copper-to-edge clearance",
    "drill_out_of_range": "Drill size",
}


def _fmt(v):
    items = "; ".join(i.get("description", "") for i in v.get("items", []))
    return f"  [{v['severity']}] {v['description']} -> {items}"


@pytest.mark.parametrize("vtype", CHECKS.keys(), ids=CHECKS.values())
def test_no_native_violations(drc_report, vtype):
    hits = [v for v in drc_report.get("violations", []) if v["type"] == vtype]
    assert not hits, f"{len(hits)} {CHECKS[vtype]} violation(s):\n" + "\n".join(map(_fmt, hits))


def test_report_schema(drc_report):
    """Guard against silent kicad-cli output format changes."""
    assert "violations" in drc_report
    assert drc_report.get("kicad_version", "").startswith("8"), \
        f"Expected KiCad 8, got {drc_report.get('kicad_version')}"
```

## `test_annular_ring.py`

```python
import pcbnew
from helpers import mm


def test_via_annular_ring(board, limits):
    lim = limits["annular_ring"]["via_min"] - limits["tolerance"]
    bad = []
    for t in board.GetTracks():
        if isinstance(t, pcbnew.PCB_VIA):
            ring = (mm(t.GetWidth()) - mm(t.GetDrillValue())) / 2
            if ring < lim:
                p = t.GetPosition()
                bad.append(f"Via @ ({mm(p.x):.3f},{mm(p.y):.3f}) ring={ring:.4f}mm")
    assert not bad, f"Via annular ring < {lim:.3f}mm:\n  " + "\n  ".join(bad)


def test_pth_pad_annular_ring(board, limits):
    lim = limits["annular_ring"]["pth_pad_min"] - limits["tolerance"]
    bad = []
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetAttribute() != pcbnew.PAD_ATTRIB_PTH:
                continue
            size, drill = pad.GetSize(), pad.GetDrillSize()
            # Worst axis; handles oval pads and slotted drills
            ring = min(mm(size.x) - mm(drill.x), mm(size.y) - mm(drill.y)) / 2
            if ring < lim:
                bad.append(f"{fp.GetReference()}.{pad.GetNumber()} ring={ring:.4f}mm")
    assert not bad, f"PTH annular ring < {lim:.3f}mm:\n  " + "\n  ".join(bad)
```

## `test_edge_clearance.py`

```python
from helpers import board_outline, copper_items


def test_copper_to_edge(board, limits):
    cfg = limits["copper_to_edge"]
    lim = cfg["min"] - limits["tolerance"]
    edge = board_outline(board)
    bad = []
    for desc, geom in copper_items(board, cfg["include_zones"]):
        if geom is None or geom.is_empty:
            continue
        d = edge.distance(geom)
        if d < lim:
            bad.append(f"{desc}: {d:.4f}mm")
    assert not bad, f"Copper within {cfg['min']}mm of edge