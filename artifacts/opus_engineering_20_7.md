# KiCad 8 DRC Validation Test Suite: Annular Rings, Copper-to-Edge, Drill Aspect Ratio

> **Note on "Cycle 20":** I don't have earlier cycles of this work, so this is a complete, self-contained suite rather than a diff. If you have a prior baseline (thresholds, waiver files, fixture boards), the config and waiver mechanisms below let you drop them in directly.

## Architecture

The suite uses two independent validation layers. A board must pass both.

| Layer | Mechanism | Purpose |
|---|---|---|
| **1. Native DRC** | `kicad-cli pcb drc --format json` | KiCad's own geometry engine, using the board's rules |
| **2. Independent DFM** | `pcbnew` Python API + `shapely` | Checks against the **fab's** limits, independent of whatever rules the designer set |

Layer 2 exists because native DRC only enforces the rules stored in the board. If someone loosens `min annular width` in Board Setup, native DRC goes green. Layer 2 also checks that the board's rules are at least as strict as the fab's. Drill aspect ratio is not a native KiCad 8 DRC check, so Layer 2 handles it entirely.

```
dfm-tests/
├── dfm_config.yaml
├── waivers.yaml
├── conftest.py
├── kicad_helpers.py
├── test_native_drc.py
├── test_rule_settings.py
├── test_annular_ring.py
├── test_edge_clearance.py
├── test_aspect_ratio.py
├── fixtures/                 # known-bad boards for negative tests
│   ├── bad_annular.kicad_pcb
│   ├── bad_edge.kicad_pcb
│   └── bad_aspect.kicad_pcb
└── .github/workflows/dfm.yml
```

---

## `dfm_config.yaml`: Fab capability profile

```yaml
# All dimensions in mm. Values below are typical of a standard
# 4-6 layer prototype process. Replace with your fab's capability sheet.
profile: standard_proto

annular_ring:
  via_min: 0.125          # (via_dia - drill) / 2
  pth_min: 0.150          # plated through-hole pads
  npth_check: false       # NPTH have no copper ring
  # For oval/slot drills, the ring is measured on the minimum axis

edge_clearance:
  copper_min: 0.300       # any copper to Edge.Cuts
  zone_min: 0.300         # filled zones (often set equal)
  # Items allowed to violate (e.g. castellated pads, edge connectors)
  exempt_footprint_refs: []
  exempt_pad_attrs: []    # e.g. ["castellated"]

aspect_ratio:
  through_max: 10.0       # board_thickness / finished_drill
  blind_max: 1.0          # depth / drill (microvia / laser)
  min_drill: 0.20         # mechanical drill floor
  # Fallback if stackup thickness is not defined in board setup
  default_board_thickness: 1.6

geometry:
  arc_tolerance_mm: 0.005 # polygon approximation tolerance for outline
```

## `waivers.yaml`: Documented exceptions

```yaml
# Every waiver needs a reason and an owner. Expired waivers fail the suite.
waivers:
  # - check: edge_clearance
  #   board: "hw/main/main.kicad_pcb"
  #   item: "J5"              # footprint ref or via UUID
  #   reason: "Card-edge connector, gold fingers intentionally at edge"
  #   owner: "jdoe"
  #   expires: 2026-01-01
```

---

## `kicad_helpers.py`

```python
"""Shared helpers for KiCad 8 DFM validation."""
from __future__ import annotations

import datetime as dt
import json
import shutil
import subprocess
from dataclasses import dataclass, field
from pathlib import Path

import yaml

try:
    import pcbnew  # provided by KiCad's Python
except ImportError as e:  # pragma: no cover
    raise ImportError(
        "pcbnew not importable. Run with KiCad's bundled Python, or set "
        "PYTHONPATH to KiCad's site-packages (e.g. /usr/lib/python3/dist-packages)."
    ) from e

NM_PER_MM = 1_000_000


def mm(nm: int | float) -> float:
    return nm / NM_PER_MM


# --------------------------------------------------------------------------
# Config / waivers
# --------------------------------------------------------------------------
def load_yaml(path: Path) -> dict:
    with open(path) as f:
        return yaml.safe_load(f) or {}


@dataclass
class Waivers:
    entries: list[dict] = field(default_factory=list)

    @classmethod
    def load(cls, path: Path) -> "Waivers":
        if not path.exists():
            return cls()
        return cls(load_yaml(path).get("waivers", []) or [])

    def expired(self) -> list[dict]:
        today = dt.date.today()
        out = []
        for w in self.entries:
            exp = w.get("expires")
            if exp and dt.date.fromisoformat(str(exp)) < today:
                out.append(w)
        return out

    def is_waived(self, check: str, board: Path, item: str) -> bool:
        for w in self.entries:
            if (w.get("check") == check
                    and str(board).endswith(str(w.get("board", "")))
                    and w.get("item") == item):
                return True
        return False


# --------------------------------------------------------------------------
# Native DRC via kicad-cli
# --------------------------------------------------------------------------
def run_kicad_drc(board: Path, out_dir: Path) -> dict:
    cli = shutil.which("kicad-cli")
    if cli is None:
        raise RuntimeError("kicad-cli not found on PATH (requires KiCad >= 8)")
    report = out_dir / f"{board.stem}.drc.json"
    cmd = [
        cli, "pcb", "drc",
        "--format", "json",
        "--severity-all",
        "--units", "mm",
        "--refill-zones",     # do not trust stale zone fills
        "--output", str(report),
        str(board),
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    # kicad-cli returns non-zero only with --exit-code-violations; any
    # other non-zero is a real tool failure.
    if proc.returncode != 0:
        raise RuntimeError(f"kicad-cli failed:\n{proc.stdout}\n{proc.stderr}")
    return json.loads(report.read_text())


# KiCad 8 DRC violation type keys relevant to this suite
NATIVE_TYPES = {
    "annular": {"annular_width"},
    "edge": {"copper_edge_clearance"},
    "drill": {"drill_out_of_range", "hole_clearance", "hole_to_hole",
              "via_diameter", "microvia_drill_out_of_range"},
}


def violations_of(report: dict, kinds: set[str]) -> list[dict]:
    return [v for v in report.get("violations", [])
            if v.get("type") in kinds and v.get("severity") != "exclusion"]


# --------------------------------------------------------------------------
# Board loading and geometry
# --------------------------------------------------------------------------
def load_board(path: Path) -> "pcbnew.BOARD":
    return pcbnew.LoadBoard(str(path))


def board_thickness_mm(board, default: float) -> float:
    t = board.GetDesignSettings().GetBoardThickness()
    return mm(t) if t > 0 else default


def iter_vias(board):
    for t in board.GetTracks():
        if isinstance(t, pcbnew.PCB_VIA):
            yield t


def iter_pads(board):
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            yield fp, pad


def via_kind(via) -> str:
    vt = via.GetViaType()
    if vt == pcbnew.VIATYPE_THROUGH:
        return "through"
    if vt == pcbnew.VIATYPE_MICROVIA:
        return "micro"
    return "blind_buried"


def item_id(obj, fp=None) -> str:
    if fp is not None:
        return f"{fp.GetReference()}.{obj.GetNumber()}"
    return str(obj.m_Uuid.AsString())


def item_pos(obj) -> str:
    p = obj.GetPosition()
    return f"({mm(p.x):.3f}, {mm(p.y):.3f}) mm"


def board_outline(board):
    """Return the Edge.Cuts outline as a shapely geometry (boundary only)."""
    from shapely.geometry import LineString, Polygon
    from shapely.ops import unary_union

    polyset = pcbnew.SHAPE_POLY_SET()
    ok = board.GetBoardPolygonOutlines(polyset)
    if not ok or polyset.OutlineCount() == 0:
        raise ValueError("Board outline (Edge.Cuts) is missing or not closed")

    def chain_to_coords(chain):
        return [(mm(chain.CPoint(i).x), mm(chain.CPoint(i).y))
                for i in range(chain.PointCount())]

    polys = []
    for i in range(polyset.OutlineCount()):
        outer = chain_to_coords(polyset.Outline(i))
        holes = [chain_to_coords(polyset.Hole(i, h))
                 for h in range(polyset.HoleCount(i))]
        polys.append(Polygon(outer, holes))
    shape = unary_union(polys)
    # Distance to the *edge*, so use the boundary (outer + cutouts)
    return shape, shape.boundary


def polyset_to_shapely(polyset):
    from shapely.geometry import Polygon
    from shapely.ops import unary_union

    out = []
    for i in range(polyset.OutlineCount()):
        o = polyset.Outline(i)
        coords = [(mm(o.CPoint(k).x), mm(o.CPoint(k).y))
                  for k in range(o.PointCount())]
        if len(coords) >= 3:
            out.append(Polygon(coords))
    return unary_union(out) if out else None


def copper_geometry(board):
    """Yield (label, shapely_geometry) for every copper object."""
    from shapely.geometry import LineString, Point, box

    for t in board.GetTracks():
        if isinstance(t, pcbnew.PCB_VIA):
            r = mm(t.GetWidth()) / 2
            p = t.GetPosition()
            yield f"via {item_id(t)} @ {item_pos(t)}", Point(mm(p.x), mm(p.y)).buffer(r)
        elif isinstance(t, pcbnew.PCB_ARC):
            # Approximate the arc by its polygon via KiCad's own converter
            ps = pcbnew.SHAPE_POLY_SET()
            t.TransformShapeToPolygon(ps, t.GetLayer(), 0,
                                      pcbnew.ARC_HIGH_DEF, pcbnew.ERROR_INSIDE)
            g = polyset_to_shapely(ps)
            if g is not None:
                yield f"arc track {item_id(t)} @ {item_pos(t)}", g
        elif t.IsOnCopperLayer():
            s, e = t.GetStart(), t.GetEnd()
            line = LineString([(mm(s.x), mm(s.y)), (mm(e.x), mm(e.y))])
            yield (f"track {item_id(t)} @ {item_pos(t)}",
                   line.buffer(mm(t.GetWidth()) / 2, cap_style=1))

    for fp, pad in iter_pads(board):
        if pad.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:
            continue
        label = f"pad {item_id(pad, fp)} @ {item_pos(pad)}"
        try:
            ps = pcbnew.SHAPE_POLY_SET()
            pad.TransformShapeToPolygon(ps, pad.GetPrincipalLayer(), 0,
                                        pcbnew.ARC_HIGH_DEF, pcbnew.ERROR_INSIDE)
            g = polyset_to_shapely(ps)
        except Exception:
            g = None
        if g is None:  # conservative fallback: bounding box
            bb = pad.GetBoundingBox()
            g = box(mm(bb.GetLeft()), mm(bb.GetTop()),
                    mm(bb.GetRight()), mm(bb.GetBottom()))
        yield label, g

    for zone in board.Zones():
        if zone.GetIsRuleArea():
            continue
        for layer in zone.GetLayerSet().CuStack():
            fill = zone.