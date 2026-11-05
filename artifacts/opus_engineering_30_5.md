# KiCad 8 DRC Validation Test Suite (Cycle 30)

I don't have context from earlier cycles, so this is a self-contained suite. The thresholds in `dfm_rules.yaml` are typical standard-class fab values. Replace them with your fab's or the previous cycle's numbers.

The suite has three layers:

1. **Native DRC gate.** Runs `kicad-cli pcb drc` and fails on annular-ring, edge-clearance or drill violations reported by KiCad.
2. **Independent DFM checks.** Uses the `pcbnew` Python API to measure geometry directly against fab limits. This catches boards whose own design rules are too lax. KiCad 8 has no native aspect-ratio rule, so that check exists only here.
3. **Checker self-tests.** Builds synthetic boards with known defects to prove the checkers actually detect them.

```
dfm/
  __init__.py
  geometry.py
  checks.py
dfm_rules.yaml
tests/
  conftest.py
  test_native_drc.py
  test_dfm_rules.py
  test_checker_selftest.py
.github/workflows/drc.yml
```

---

## `dfm_rules.yaml`

```yaml
cycle: 30
boards_glob: "hardware/**/*.kicad_pcb"

annular_ring:
  via_min_mm: 0.125         # (pad dia - drill) / 2
  pth_pad_min_mm: 0.15
  microvia_min_mm: 0.075

copper_to_edge:
  min_mm: 0.30              # copper (tracks, pads, vias, zones, graphics) to Edge.Cuts
  include_zones: true
  refill_zones: true        # refill before measuring so stale fills don't hide issues

drill_aspect_ratio:
  through_max: 10.0         # depth : drill
  blind_buried_max: 8.0
  microvia_max: 1.0
  drill_oversize_mm: 0.10   # finished -> drilled (plating allowance)

native_drc:
  fail_on_types:
    - annular_width
    - copper_edge_clearance
    - drill_out_of_range
    - microvia_drill_out_of_range
    - via_diameter
    - hole_clearance
    - hole_to_hole
  fail_on_severity: [error]
```

---

## `dfm/geometry.py`

```python
"""Geometry extraction from KiCad 8 boards via the pcbnew API. Units: nm internally."""
from __future__ import annotations
from dataclasses import dataclass
import pcbnew
from shapely.geometry import Polygon, MultiPolygon
from shapely.ops import unary_union

MAX_ERR = pcbnew.FromMM(0.005)


def nm_to_mm(v: int | float) -> float:
    return v / 1e6


def polyset_to_shapely(ps: "pcbnew.SHAPE_POLY_SET"):
    polys = []
    for i in range(ps.OutlineCount()):
        outline = ps.Outline(i)
        shell = [(outline.CPoint(j).x, outline.CPoint(j).y) for j in range(outline.PointCount())]
        holes = []
        for h in range(ps.HoleCount(i)):
            hole = ps.Hole(i, h)
            holes.append([(hole.CPoint(j).x, hole.CPoint(j).y) for j in range(hole.PointCount())])
        if len(shell) >= 3:
            polys.append(Polygon(shell, holes))
    if not polys:
        return None
    return unary_union(polys) if len(polys) > 1 else polys[0]


def board_outline(board: "pcbnew.BOARD"):
    ps = pcbnew.SHAPE_POLY_SET()
    ok = board.GetBoardPolygonOutlines(ps)
    if not ok or ps.OutlineCount() == 0:
        raise ValueError("Board outline (Edge.Cuts) is missing or not closed")
    return polyset_to_shapely(ps)


@dataclass
class CopperShape:
    kind: str
    ref: str
    layer: str
    geom: object
    pos_mm: tuple[float, float]


def _item_shape(item, layer):
    ps = pcbnew.SHAPE_POLY_SET()
    item.TransformShapeToPolygon(ps, layer, 0, MAX_ERR, pcbnew.ERROR_INSIDE)
    return polyset_to_shapely(ps)


def _pos(item):
    p = item.GetPosition()
    return (round(nm_to_mm(p.x), 3), round(nm_to_mm(p.y), 3))


def copper_shapes(board, include_zones=True, refill_zones=True):
    if include_zones and refill_zones:
        pcbnew.ZONE_FILLER(board).Fill(board.Zones())

    copper_layers = [l for l in board.GetEnabledLayers().CuStack()]
    name = board.GetLayerName

    for t in board.GetTracks():
        for layer in copper_layers:
            if t.IsOnLayer(layer):
                g = _item_shape(t, layer)
                if g is not None:
                    kind = "via" if isinstance(t, pcbnew.PCB_VIA) else "track"
                    yield CopperShape(kind, t.GetNetname() or "<no net>", name(layer), g, _pos(t))

    for fp in board.GetFootprints():
        ref = fp.GetReference()
        for pad in fp.Pads():
            for layer in copper_layers:
                if pad.IsOnLayer(layer) and pad.FlashLayer(layer):
                    g = _item_shape(pad, layer)
                    if g is not None:
                        yield CopperShape("pad", f"{ref}.{pad.GetNumber()}", name(layer), g, _pos(pad))
        for gi in fp.GraphicalItems():
            if gi.GetLayer() in copper_layers:
                g = _item_shape(gi, gi.GetLayer())
                if g is not None:
                    yield CopperShape("fp_graphic", ref, name(gi.GetLayer()), g, _pos(gi))

    for d in board.GetDrawings():
        if d.GetLayer() in copper_layers and not isinstance(d, pcbnew.PCB_TEXTBOX):
            g = _item_shape(d, d.GetLayer())
            if g is not None:
                yield CopperShape("graphic", "board", name(d.GetLayer()), g, _pos(d))

    if include_zones:
        for z in board.Zones():
            if z.GetIsRuleArea():
                continue
            for layer in copper_layers:
                if z.IsOnLayer(layer) and z.HasFilledPolysForLayer(layer):
                    g = polyset_to_shapely(z.GetFilledPolysList(layer))
                    if g is not None:
                        yield CopperShape("zone", z.GetNetname() or z.GetZoneName(),
                                          name(layer), g, _pos(z))


def span_depth_nm(board, top_layer, bottom_layer) -> int:
    """Physical depth between two copper layers from the stackup.
    Falls back to full board thickness if the stackup is unavailable."""
    ds = board.GetDesignSettings()
    full = ds.GetBoardThickness()
    try:
        items = ds.GetStackupDescriptor().GetList()
        depth, inside = 0, False
        for it in items:
            t = it.GetType()
            if t == pcbnew.BS_ITEM_TYPE_COPPER:
                lid = it.GetBrdLayerId()
                if lid == top_layer:
                    inside = True
                    depth += it.GetThickness()
                    continue
                if inside:
                    depth += it.GetThickness()
                    if lid == bottom_layer:
                        return depth
            elif inside and t == pcbnew.BS_ITEM_TYPE_DIELECTRIC:
                for sub in range(it.GetSublayersCount()):
                    depth += it.GetThickness(sub)
    except Exception:
        pass
    return full
```

---

## `dfm/checks.py`

```python
"""Independent DFM checks. Each returns a list of human-readable violation strings."""
from __future__ import annotations
import pcbnew
from .geometry import nm_to_mm, board_outline, copper_shapes, span_depth_nm


def _fmt(item, pos):
    return f"{item} @ ({pos[0]:.3f}, {pos[1]:.3f}) mm"


def check_annular_rings(board, rules) -> list[str]:
    out = []
    via_min = pcbnew.FromMM(rules["via_min_mm"])
    uvia_min = pcbnew.FromMM(rules["microvia_min_mm"])
    pth_min = pcbnew.FromMM(rules["pth_pad_min_mm"])

    for t in board.GetTracks():
        if not isinstance(t, pcbnew.PCB_VIA):
            continue
        ring = (t.GetWidth() - t.GetDrillValue()) // 2
        is_micro = t.GetViaType() == pcbnew.VIATYPE_MICROVIA
        limit = uvia_min if is_micro else via_min
        if ring < limit:
            p = t.GetPosition()
            out.append(_fmt(f"{'Microvia' if is_