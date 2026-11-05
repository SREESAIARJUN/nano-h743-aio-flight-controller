# KiCad 8 DRC Validation Test Suite: Annular Rings, Copper-to-Edge, Drill Aspect Ratios

> **Note on "Cycle 12":** I don't have earlier cycles of this work, so I can't diff against previous thresholds or carry over prior decisions. This is a complete, self-contained suite. If Cycle 12 has specific limits from your fab or earlier cycles, put them in `drc_limits.json` (Section 2) and the tests will use them.

The suite uses two independent checks:

1. **Native DRC.** It runs `kicad-cli pcb drc` (KiCad 8) and parses the JSON report. This uses the board's own design rules and custom rules.
2. **Independent geometric checks.** It uses the `pcbnew` Python API to measure every via and PTH pad against the fab limits. This catches boards whose design rules were relaxed or misconfigured.

---

## 1. Layout

```
drc_tests/
├── drc_limits.json
├── conftest.py
├── kicad_helpers.py
├── test_native_drc.py
├── test_annular_ring.py
├── test_copper_edge.py
├── test_aspect_ratio.py
└── fixtures/              # known-bad boards for negative tests
    ├── bad_annular.kicad_pcb
    ├── bad_edge.kicad_pcb
    └── bad_aspect.kicad_pcb
```

---

## 2. `drc_limits.json` (fab limits, all in mm)

```json
{
  "boards": ["../hardware/main_board.kicad_pcb"],
  "annular_ring": {
    "via_min_mm": 0.125,
    "pth_pad_min_mm": 0.15,
    "tolerance_mm": 0.001
  },
  "copper_edge": {
    "min_clearance_mm": 0.30,
    "tolerance_mm": 0.001
  },
  "aspect_ratio": {
    "via_max": 8.0,
    "pth_max": 10.0,
    "board_thickness_override_mm": null
  },
  "native_drc": {
    "fail_on_severity": ["error"],
    "watched_types": [
      "annular_width",
      "copper_edge_clearance",
      "drill_out_of_range",
      "via_diameter",
      "hole_clearance",
      "edge_clearance"
    ]
  }
}
```

---

## 3. `kicad_helpers.py`

```python
"""Shared helpers for KiCad 8 DRC validation."""
from __future__ import annotations

import json
import math
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path

import pcbnew  # Requires KiCad 8's Python (or the kicad/kicad:8.0 container)

NM_PER_MM = 1_000_000


def mm(nm: int | float) -> float:
    return nm / NM_PER_MM


# ---------------------------------------------------------------- native DRC
def find_kicad_cli() -> str:
    exe = shutil.which("kicad-cli")
    if not exe:
        raise RuntimeError("kicad-cli not found on PATH (KiCad 8 required)")
    out = subprocess.run([exe, "--version"], capture_output=True, text=True)
    if not out.stdout.strip().startswith("8."):
        raise RuntimeError(f"Expected KiCad 8.x, got: {out.stdout.strip()}")
    return exe


def run_native_drc(board: Path, out_dir: Path) -> dict:
    """Run kicad-cli DRC and return the parsed JSON report."""
    report = out_dir / f"{board.stem}.drc.json"
    cmd = [
        find_kicad_cli(), "pcb", "drc",
        "--format", "json",
        "--severity-all",
        "--units", "mm",
        "--output", str(report),
        str(board),
    ]
    # Don't pass --exit-code-violations: we want the report even on failure.
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0 or not report.exists():
        raise RuntimeError(f"kicad-cli drc failed:\n{proc.stdout}\n{proc.stderr}")
    return json.loads(report.read_text())


# ---------------------------------------------------------------- geometry
@dataclass(frozen=True)
class Hole:
    kind: str           # "via" | "pth"
    ref: str            # footprint ref + pad number, or "VIA"
    x_mm: float
    y_mm: float
    drill_mm: float     # finished hole diameter (minor axis for slots)
    copper_mm: float    # copper diameter (minor axis for non-circular pads)
    net: str

    @property
    def annular_ring_mm(self) -> float:
        return (self.copper_mm - self.drill_mm) / 2.0

    def loc(self) -> str:
        return f"{self.kind} {self.ref} net={self.net!r} @ ({self.x_mm:.3f}, {self.y_mm:.3f}) mm"


def load_board(path: Path) -> pcbnew.BOARD:
    board = pcbnew.LoadBoard(str(path))
    if board is None:
        raise RuntimeError(f"Failed to load {path}")
    return board


def board_thickness_mm(board: pcbnew.BOARD) -> float:
    return mm(board.GetDesignSettings().GetBoardThickness())


def iter_holes(board: pcbnew.BOARD):
    """Yield every plated hole (vias + PTH pads)."""
    for t in board.GetTracks():
        if isinstance(t, pcbnew.PCB_VIA):
            pos = t.GetPosition()
            yield Hole(
                kind="via",
                ref="VIA",
                x_mm=mm(pos.x), y_mm=mm(pos.y),
                drill_mm=mm(t.GetDrillValue()),
                copper_mm=mm(t.GetWidth()),
                net=t.GetNetname(),
            )

    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetAttribute() != pcbnew.PAD_ATTRIB_PTH:
                continue
            drill = pad.GetDrillSize()
            size = pad.GetSize()
            pos = pad.GetPosition()
            yield Hole(
                kind="pth",
                ref=f"{fp.GetReference()}.{pad.GetNumber()}",
                x_mm=mm(pos.x), y_mm=mm(pos.y),
                # Worst case for slots / oblong pads: the minor axis.
                drill_mm=mm(min(drill.x, drill.y)),
                copper_mm=mm(min(size.x, size.y)),
                net=pad.GetNetname(),
            )


# ---------------------------------------------------------------- board outline
def board_outline_segments(board: pcbnew.BOARD):
    """Return the Edge.Cuts outline as line segments ((x1,y1),(x2,y2)) in mm.

    Uses KiCad's own outline builder, so arcs and circles are already
    polygonised. Includes holes and cutouts in the outline.
    """
    polys = pcbnew.SHAPE_POLY_SET()
    if not board.GetBoardPolygonOutlines(polys):
        raise RuntimeError("Board outline (Edge.Cuts) is not closed/valid")

    segs = []
    for oi in range(polys.OutlineCount()):
        chains = [polys.COutline(oi)] + [
            polys.CHole(oi, hi) for hi in range(polys.HoleCount(oi))
        ]
        for chain in chains:
            n = chain.PointCount()
            for i in range(n):
                a, b = chain.CPoint(i), chain.CPoint((i + 1) % n)
                segs.append(((mm(a.x), mm(a.y)), (mm(b.x), mm(b.y))))
    return segs


def point_seg_dist(p, a, b) -> float:
    px, py = p
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 == 0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def seg_seg_dist(a1, a2, b1, b2) -> float:
    # Segments are assumed non-intersecting; the native DRC catches crossings.
    return min(
        point_seg_dist(a1, b1, b2), point_seg_dist(a2, b1, b2),
        point_seg_dist(b1, a1, a2), point_seg_dist(b2, a1, a2),
    )


def iter_copper_features(board: pcbnew.BOARD):
    """Yield (label, kind, geometry, radius_mm) for copper features.

    kind = "point" -> geometry=(x, y); kind = "seg" -> geometry=((x1, y1), (x2, y2))
    Pads use a conservative bounding-circle radius (half the larger dimension).
    Zones are checked by the native DRC only.
    """
    for t in board.GetTracks():
        if isinstance(t, pcbnew.PCB_VIA):
            p = t.GetPosition()
            yield (f"VIA net={t.GetNetname()!r}", "point",
                   (mm(p.x), mm(p.y)), mm(t.GetWidth()) / 2)
        elif isinstance(t, pcbnew.PCB_ARC):
            # Approximate the arc by its chord plus a midpoint check.
            s, m, e = t.GetStart(), t.GetMid(), t.GetEnd()
            r = mm(t.GetWidth()) / 2
            yield (f"ARC net={t.GetNetname()!r}", "seg",
                   ((mm(s.x), mm(s.y)), (mm(m.x), mm(m.y))), r)
            yield (f"ARC net={t.GetNetname()!r}", "seg",
                   ((mm(m.x), mm(m.y)), (mm(e.x), mm(e.y))), r)
        else:
            s, e = t.GetStart(), t.GetEnd()
            yield (f"TRACK net={t.GetNetname()!r} layer={t.GetLayerName()}", "seg",
                   ((mm(s.x), mm(s.y)), (mm(e.x), mm(e.y))), mm(t.GetWidth()) / 2)

    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if not pad.IsOnCopperLayer():
                continue
            p, sz = pad.GetPosition(), pad.GetSize()
            yield (f"PAD {fp.GetReference()}.{pad.GetNumber()}", "point",
                   (mm(p.x), mm(p.y)), mm(max(sz.x, sz.y)) / 2)
```

---

## 4. `conftest.py`

```python
import json
from pathlib import Path

import pytest

from kicad_helpers import load_board, run_native_drc

HERE = Path(__file__).parent
LIMITS = json.loads((HERE / "drc_limits.json").read_text())
BOARDS = [(HERE / b).resolve() for b in LIMITS["boards"]]


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
    if board_path not in _board_cache:
        _board_cache[board_path] = load_board(board_path)
    return _board_cache[board_path]


@pytest.fixture(scope="session")
def report_dir(tmp_path_factory):
    return tmp_path_factory.mktemp("drc_reports")


@pytest.fixture
def native_report(board_path, report_dir):
    return run_native_drc(board_path, report_dir)


def format_failures(title: str, items: list[str], limit: int = 25) -> str:
    shown = "\n  ".join(items[:limit])
    more = f"\n  ... and {len(items) - limit} more" if len(items) > limit else ""
    return f"{title} ({len(items)} violations):\n  {shown}{more}"
```

---

## 5. `test_native_drc.py`

```python
from conftest import format_failures


def _violations(report, types, severities):
    return [
        v for v in report.get("violations", [])
        if v.get("type") in types and v.get("severity") in severities
    ]


def _fmt(v):
    items = "; ".join(
        f"{i.get('description', '?')} @ ({i['pos']['x']:.3f},{i['pos']['y']:.3f})"
        for i in v.get("items", []) if "pos" in i
    )
    return f"[{v['type']}] {v['description']} :: {items}"


def test_native_drc_report_is_kicad8(native_report):
    assert native_report.get("kicad_version", "").startswith("8."), \
        f"Report not produced by KiCad 8: {native_report.get('kicad_version')}"


def test_native_annular_ring(native_report, limits):
    sev = limits["native_drc"]["fail_on_severity"]
    bad = _violations(native_report, {"annular_width"}, sev)
    assert not bad, format_failures("Native annular-ring violations", [_fmt(v) for v in bad])


def test_native_copper_edge(native_report, limits):
    sev = limits["native_drc"]["fail_on_severity"]
    bad = _violations(native_report, {"copper_edge_clearance", "edge_clearance"}, sev)
    assert not bad, format_failures("Native copper-to-edge violations", [_fmt(v) for v in bad])


def test_native_drill(native_report, limits):
    sev = limits["native_drc"]["fail_on_severity"]
    bad = _violations(native_report, {"drill_out_of_range", "via_diameter", "hole_clearance"}, sev)
    assert not bad, format_failures("Native drill/via violations", [_fmt(v) for v in bad])


def test_no_unconnected_items(native_report):
    unconnected = native_report.get("unconnected_items", [])
    assert not unconn