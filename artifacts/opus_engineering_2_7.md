# KiCad 8 DRC Validation Suite: Annular Ring, Copper-to-Edge, and Drill Aspect Ratio

I don't have the Cycle 1 deliverable, so this suite is self-contained.

## How the suite works

It uses two independent layers of checking:

1. **The native KiCad DRC engine.** `kicad-cli pcb drc` runs with a fab-specific `.kicad_dru` rule file. The suite parses the JSON report and filters it by violation type.
2. **Independent geometric checks.** These use the `pcbnew` Python API. They catch two problems:
   - **Misconfigured design rules.** If a board's rule is looser than the fab's limit, KiCad passes the board anyway. The geometric checks compare against the fab limits directly.
   - **Drill aspect ratio.** KiCad 8 has no native constraint for it.

**Negative fixtures** (known-bad boards) prove that each check actually fires.

## Layout

```
drc-suite/
├── drc_config.toml
├── rules/fab.kicad_dru
├── tests/
│   ├── conftest.py
│   ├── kicad_geom.py
│   ├── test_drc_engine.py
│   ├── test_annular_ring.py
│   ├── test_edge_clearance.py
│   ├── test_aspect_ratio.py
│   └── fixtures/
│       ├── good_board.kicad_pcb
│       ├── bad_annular.kicad_pcb
│       ├── bad_edge.kicad_pcb
│       └── bad_aspect.kicad_pcb
└── .github/workflows/drc.yml
```

---

## `drc_config.toml`

```toml
# Fab capability limits (mm). These are the source of truth.
[limits]
min_via_annular_ring_mm   = 0.125
min_pth_annular_ring_mm   = 0.15
min_copper_edge_clear_mm  = 0.30
max_aspect_ratio_through  = 10.0   # board_thickness / finished_drill
max_aspect_ratio_blind    = 1.0    # depth / drill (microvia / blind)
min_drill_mm              = 0.20

[tolerance]
# Float slack for nm -> mm conversions
eps_mm = 0.0005

[boards]
# Globs relative to repo root
include = ["hardware/**/*.kicad_pcb"]
exclude = ["**/backup/**", "**/*-bak*"]
```

## `rules/fab.kicad_dru`

```lisp
(version 1)

(rule "fab_via_annular"
  (condition "A.Type == 'Via'")
  (constraint annular_width (min 0.125mm)))

(rule "fab_pth_annular"
  (condition "A.Type == 'Pad' && A.Pad_Type == 'Through-hole'")
  (constraint annular_width (min 0.15mm)))

(rule "fab_copper_edge"
  (constraint edge_clearance (min 0.3mm)))

(rule "fab_min_drill"
  (condition "A.isPlated()")
  (constraint hole_size (min 0.2mm)))
```

Before running DRC, the suite copies this file next to each board as `<board>.kicad_dru`. This ensures the fab rules apply even if a designer edited or deleted the project's own rule file.

---

## `tests/kicad_geom.py`

```python
"""Geometry helpers on top of the KiCad 8 pcbnew API (units: nm internally)."""
from __future__ import annotations
import math
from dataclasses import dataclass
import pcbnew

NM_PER_MM = 1_000_000


def mm(nm: int | float) -> float:
    return nm / NM_PER_MM


@dataclass
class Finding:
    kind: str
    ref: str
    value_mm: float
    limit: float
    x_mm: float
    y_mm: float

    def __str__(self) -> str:
        return (f"[{self.kind}] {self.ref} @ ({self.x_mm:.3f},{self.y_mm:.3f}) "
                f"value={self.value_mm:.4f} limit={self.limit}")


def load(path: str) -> pcbnew.BOARD:
    return pcbnew.LoadBoard(str(path))


def board_thickness_nm(board) -> int:
    return board.GetDesignSettings().GetBoardThickness()


# ---------- Annular ring ----------
def via_annular_rings(board):
    for t in board.GetTracks():
        if isinstance(t, pcbnew.PCB_VIA):
            ring = (t.GetWidth() - t.GetDrillValue()) / 2  # KiCad 8: single width
            p = t.GetPosition()
            yield t, mm(ring), mm(p.x), mm(p.y)


def pth_pad_annular_rings(board):
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetAttribute() != pcbnew.PAD_ATTRIB_PTH:
                continue
            size, drill = pad.GetSize(), pad.GetDrillSize()
            if drill.x == 0:
                continue
            ring = min(size.x - drill.x, size.y - drill.y) / 2
            p = pad.GetPosition()
            ref = f"{fp.GetReference()}.{pad.GetNumber()}"
            yield ref, mm(ring), mm(p.x), mm(p.y)


# ---------- Aspect ratio ----------
def _layer_span_thickness_nm(board, top_layer: int, bot_layer: int) -> int | None:
    """Sum stackup thickness between two copper layers. None if unavailable."""
    try:
        stack = board.GetDesignSettings().GetStackupDescriptor()
        items = list(stack.GetList())
        inside, total = False, 0
        for it in items:
            lid = it.GetBrdLayerId()
            if lid == top_layer:
                inside = True
                continue
            if lid == bot_layer and inside:
                return total
            if inside and it.GetType() == pcbnew.BS_ITEM_TYPE_DIELECTRIC:
                for sub in range(it.GetSublayersCount()):
                    total += it.GetThickness(sub)
            elif inside and it.GetType() == pcbnew.BS_ITEM_TYPE_COPPER:
                total += it.GetThickness()
        return None
    except Exception:
        return None


def via_aspect_ratios(board):
    thick = board_thickness_nm(board)
    for t in board.GetTracks():
        if not isinstance(t, pcbnew.PCB_VIA):
            continue
        drill = t.GetDrillValue()
        if drill <= 0:
            continue
        vt = t.GetViaType()
        if vt == pcbnew.VIATYPE_THROUGH:
            depth, kind = thick, "through"
        else:
            span = _layer_span_thickness_nm(board, t.TopLayer(), t.BottomLayer())
            depth, kind = (span if span else thick), "blind"  # conservative fallback
        p = t.GetPosition()
        yield kind, depth / drill, mm(drill), mm(p.x), mm(p.y)


def pth_pad_aspect_ratios(board):
    thick = board_thickness_nm(board)
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetAttribute() not in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH):
                continue
            d = pad.GetDrillSize()
            drill = min(d.x, d.y)  # slots: narrow dimension governs plating
            if drill <= 0:
                continue
            p = pad.GetPosition()
            yield f"{fp.GetReference()}.{pad.GetNumber()}", thick / drill, mm(p.x), mm(p.y)


# ---------- Copper-to-edge ----------
def _pt_seg_dist(px, py, ax, ay, bx, by) -> float:
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 == 0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _segs_intersect(a, b, c, d) -> bool:
    def orient(p, q, r):
        return (q[0]-p[0])*(r[1]-p[1]) - (q[1]-p[1])*(r[0]-p[0])
    o1, o2, o3, o4 = orient(a, b, c), orient(a, b, d), orient(c, d, a), orient(c, d, b)
    return (o1 * o2 < 0) and (o3 * o4 < 0)


def _seg_seg_dist(a, b, c, d) -> float:
    if _segs_intersect(a, b, c, d):
        return 0.0
    return min(_pt_seg_dist(*a, *c, *d), _pt_seg_dist(*b, *c, *d),
               _pt_seg_dist(*c, *a, *b), _pt_seg_dist(*d, *a, *b))


def outline_segments(board):
    """All Edge.Cuts outline segments (outer outlines + cutouts), arcs linearized."""
    poly = pcbnew.SHAPE_POLY_SET()
    if not board.GetBoardPolygonOutlines(poly):
        raise RuntimeError("Board outline is not closed / invalid")
    segs = []

    def chain(lc):
        n = lc.PointCount()
        for i in range(n):
            p, q = lc.CPoint(i), lc.CPoint((i + 1) % n)
            segs.append(((p.x, p.y), (q.x, q.y)))

    for o in range(poly.OutlineCount()):
        chain(poly.Outline(o))
        for h in range(poly.HoleCount(o)):
            chain(poly.Hole(o, h))
    return segs


def track_edge_clearances(board):
    """Yields (desc, clearance_mm, x, y) for tracks and vias vs Edge.Cuts."""
    segs = outline_segments(board)
    for t in board.GetTracks():
        if isinstance(t, pcbnew.PCB_VIA):
            p = t.GetPosition()
            centre = (p.x, p.y)
            d = min(_pt_seg_dist(*centre, *s[0], *s[1]) for s in segs)
            yield f"via[{t.GetNetname()}]", mm(d - t.GetWidth() / 2), m