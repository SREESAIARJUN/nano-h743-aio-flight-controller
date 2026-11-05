# KiCad 8 DRC Validation Test Suite: Annular Rings, Copper-to-Edge Clearance, Drill Aspect Ratio

**Note on "Cycle 23":** I don't have access to your internal Cycle 23 requirements, such as fab house limits, IPC class, or board list. All thresholds are in a single config file (`drc_limits.yaml`). The placeholder values below are typical IPC-2221 Class 2 numbers. Replace them with your Cycle 23 values.

The suite uses two independent layers of checking:

1. **Native DRC** via `kicad-cli pcb drc --format json`. This checks the board against the rules stored in the board itself.
2. **Independent geometric checks** via the `pcbnew` Python API. These check the board against *your* limits, so a board whose own rules were loosened still fails. KiCad has no built-in aspect-ratio rule, so that check only exists here.

---

## Directory layout

```
drc_suite/
├── drc_limits.yaml
├── conftest.py
├── kicad_helpers.py
├── test_native_drc.py
├── test_annular_ring.py
├── test_edge_clearance.py
├── test_drill_aspect_ratio.py
├── requirements.txt
└── .github/workflows/drc.yml   (optional CI)
```

---

## `drc_limits.yaml`

```yaml
cycle: 23
boards:
  - path: ../hardware/main_board/main_board.kicad_pcb
  - path: ../hardware/io_board/io_board.kicad_pcb

limits:
  # Annular ring = (copper diameter - finished drill diameter) / 2
  min_annular_ring_via_mm: 0.125
  min_annular_ring_pth_mm: 0.150
  # Copper (tracks, vias, pads, zones) to Edge.Cuts
  min_copper_to_edge_mm: 0.300
  # Board thickness / smallest drill diameter
  max_aspect_ratio_through: 10.0
  # Applied to blind/buried vias (dielectric span / drill)
  max_aspect_ratio_microvia: 0.8
  min_drill_mm: 0.200

native_drc:
  # KiCad 8 violation "type" keys that must never appear
  blocking_types:
    - annular_width
    - copper_edge_clearance
    - drill_out_of_range
    - via_diameter
    - hole_clearance
    - hole_to_hole
  allow_unconnected: false
  check_schematic_parity: false

# Optional per-item waivers, e.g. mounting holes or edge connectors
waivers:
  edge_clearance_refs: []      # e.g. ["J1"] for card-edge fingers
  annular_ring_refs: []
```

---

## `requirements.txt`

```
pytest>=7.4
pyyaml>=6.0
shapely>=2.0
```

`pcbnew` comes with KiCad 8. Run the tests using KiCad's bundled Python, or a Python whose `sys.path` includes KiCad's site-packages:
- Linux: `/usr/lib/python3/dist-packages`
- Windows: `C:\Program Files\KiCad\8.0\bin\python.exe`

---

## `kicad_helpers.py`

```python
"""Helpers wrapping the KiCad 8 pcbnew API and kicad-cli."""
from __future__ import annotations

import json
import shutil
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path

import pcbnew
from shapely.geometry import Polygon, MultiPolygon
from shapely.ops import unary_union

NM_PER_MM = 1_000_000


def nm_to_mm(v: int | float) -> float:
    return v / NM_PER_MM


def load_board(path: str | Path) -> pcbnew.BOARD:
    board = pcbnew.LoadBoard(str(path))
    if board is None:
        raise RuntimeError(f"Failed to load board: {path}")
    return board


def board_thickness_mm(board: pcbnew.BOARD) -> float:
    return nm_to_mm(board.GetDesignSettings().GetBoardThickness())


# ---------------------------------------------------------------- native DRC
def run_kicad_cli_drc(board_path: Path, parity: bool = False) -> dict:
    cli = shutil.which("kicad-cli")
    if cli is None:
        raise RuntimeError("kicad-cli not found on PATH")
    with tempfile.TemporaryDirectory() as td:
        out = Path(td) / "drc.json"
        cmd = [cli, "pcb", "drc",
               "--format", "json",
               "--severity-all",
               "--units", "mm",
               "--output", str(out)]
        if parity:
            cmd.append("--schematic-parity")
        cmd.append(str(board_path))
        proc = subprocess.run(cmd, capture_output=True, text=True)
        if not out.exists():
            raise RuntimeError(
                f"kicad-cli DRC produced no report (rc={proc.returncode}):\n"
                f"{proc.stdout}\n{proc.stderr}")
        return json.loads(out.read_text())


# ---------------------------------------------------------------- holes
@dataclass
class Hole:
    kind: str            # "via", "pth"
    ref: str             # footprint ref or "VIA"
    pos_mm: tuple[float, float]
    drill_mm: float      # min drill dimension (slots: minor axis)
    copper_mm: float     # min copper dimension on the pad/via
    via_type: int | None = None
    layer_span: tuple[int, int] | None = None

    @property
    def annular_ring_mm(self) -> float:
        return (self.copper_mm - self.drill_mm) / 2.0

    def where(self) -> str:
        return f"{self.ref} @ ({self.pos_mm[0]:.3f}, {self.pos_mm[1]:.3f}) mm"


def iter_holes(board: pcbnew.BOARD):
    for t in board.GetTracks():
        if isinstance(t, pcbnew.PCB_VIA):
            p = t.GetPosition()
            top, bot = t.TopLayer(), t.BottomLayer()
            yield Hole(
                kind="via", ref="VIA",
                pos_mm=(nm_to_mm(p.x), nm_to_mm(p.y)),
                drill_mm=nm_to_mm(t.GetDrillValue()),
                copper_mm=nm_to_mm(t.GetWidth()),
                via_type=t.GetViaType(),
                layer_span=(top, bot),
            )

    for fp in board.GetFootprints():
        ref = fp.GetReference()
        for pad in fp.Pads():
            if pad.GetAttribute() != pcbnew.PAD_ATTRIB_PTH:
                continue
            d = pad.GetDrillSize()
            s = pad.GetSize()
            p = pad.GetPosition()
            yield Hole(
                kind="pth", ref=f"{ref}.{pad.GetNumber()}",
                pos_mm=(nm_to_mm(p.x), nm_to_mm(p.y)),
                drill_mm=nm_to_mm(min(d.x, d.y)),
                copper_mm=nm_to_mm(min(s.x, s.y)),
            )


def layer_span_thickness_mm(board: pcbnew.BOARD, top: int, bot: int) -> float:
    """Approximate dielectric thickness between two copper layers using the
    board stackup; falls back to proportional thickness if stackup is empty."""
    ds = board.GetDesignSettings()
    stackup = ds.GetStackupDescriptor()
    items = list(stackup.GetList())
    total = 0.0
    inside = False
    for it in items:
        lid = it.GetBrdLayerId()
        if lid == top:
            inside = True
            continue
        if lid == bot:
            break
        if inside and it.GetType() == pcbnew.BS_ITEM_TYPE_DIELECTRIC:
            total += nm_to_mm(it.GetThickness())
    if total > 0:
        return total
    # Fallback: equal split across copper layers
    n = board.GetCopperLayerCount()
    cu = list(board.GetEnabledLayers().CuStack())
    try:
        span = abs(cu.index(bot) - cu.index(top))
    except ValueError:
        span = n - 1
    return board_thickness_mm(board) * span / max(n - 1, 1)


# ---------------------------------------------------------------- outline / copper
def _poly_set_to_shapely(ps: pcbnew.SHAPE_POLY_SET):
    polys = []
    for i in range(ps.OutlineCount()):
        outline = ps.Outline(i)
        shell = [(nm_to_mm(outline.CPoint(k).x), nm_to_mm(outline.CPoint(k).y))
                 for k in range(outline.PointCount())]
        holes = []
        for h in range(ps.HoleCount(i)):
            hc = ps.Hole(i, h)
            holes.append([(nm_to_mm(hc.CPoint(k).x), nm_to_mm(hc.CPoint(k).y))
                          for k in range(hc.PointCount())])
        if len(shell) >= 3:
            polys.append(Polygon(shell, holes))
    return unary_union(polys) if polys else MultiPolygon()


def board_outline(board: pcbnew.BOARD):
    ps = pcbnew.SHAPE_POLY_SET()
    ok = board.GetBoardPolygonOutlines(ps)
    if not ok or ps.OutlineCount() == 0:
        raise RuntimeError("Board outline (Edge.Cuts) is missing or not closed")
    return _poly_set_to_shapely(ps)


@dataclass
class CopperItem:
    desc: str
    geom: object  # shapely geometry


def iter_copper_items(board: pcbnew.BOARD, max_err_nm: int = 5000):
    cu_layers = list(board.GetEnabledLayers().CuStack())

    def item_poly(item, layer):
        ps = pcbnew.SHAPE_POLY_SET()
        item.TransformShapeToPolygon(ps, layer, 0, max_err_nm, pcbnew.ERROR_INSIDE)
        return _poly_set_to_shapely(ps)

    for t in board.GetTracks():
        kind = "VIA" if isinstance(t, pcbnew.PCB_VIA) else "TRACK"
        for layer in cu_layers:
            if t.IsOnLayer(layer):
                p = t.GetPosition()
                yield CopperItem(
                    f"{kind} net={t.GetNetname()} layer={board.GetLayerName(layer)} "
                    f"@ ({nm_to_mm(p.x):.3f},{nm_to_mm(p.y):.3f})",
                    item_poly(t, layer))
                if kind == "VIA":
                    break  # via barrel geometry is identical on each layer

    for fp in board.GetFootprints():
        for pad in fp.Pads():
            for layer in cu_layers:
                if pad.IsOnLayer(layer) and pad.FlashLayer(layer):
                    yield CopperItem(
                        f"PAD {fp.GetReference()}.{pad.GetNumber()} "
                        f"layer={board.GetLayerName(layer)}",
                        item_poly(pad, layer))

    for zone in board.Zones():
        if zone.GetIsRuleArea():
            continue
        for layer in cu_layers:
            if zone.IsOnLayer(layer) and zone.HasFilledPolysForLayer(layer):
                fill = zone.GetFilledPolysList(layer)
                yield CopperItem(
                    f"ZONE net={zone.GetNetname()} layer={board.GetLayerName(layer)}",
                    _poly_set_to_shapely(fill))
```

---

## `conftest.py`

```python
from pathlib import Path

import pytest
import yaml

HERE = Path(__file__).parent
CFG = yaml.safe_load((HERE / "drc_limits.yaml").read_text())


def _board_paths():
    return [(HERE / b["path"]).resolve() for b in CFG["boards"]]


@pytest.fixture(scope="session")
def cfg():
    return CFG


@pytest.fixture(scope="session")
def limits():
    return CFG["limits"]


@pytest.fixture(scope="session")
def waivers():
    return CFG.get("waivers", {}) or {}


def pytest_generate_tests(metafunc):
    if "board_path" in metafunc.fixturenames:
        paths = _board_paths()
        metafunc.parametrize("board_path", paths, ids=[p.stem for p in paths])


@pytest.fixture(scope="session")
def _board_cache():
    return {}


@pytest.fixture
def board(board_path, _board_cache):
    from kicad_helpers import load_board
    if board_path not in _board_cache:
        assert board_path.exists(), f"Board not found: {board_path}"
        _board_cache[board_path] = load_board(board_path)
    return _board_cache[board_path]


def format_failures(title, rows, limit=50):
    shown = "\n  ".join(rows[:limit])
    extra = f"\n  ... and {len(rows) - limit} more" if len(rows) > limit else ""
    return f"{title} ({len(rows)} violations):\n  {shown}{extra}"
```

---

## `test_native_drc.py`

```python
import pytest
from conftest import format_failures
from kicad_helpers import run_kicad_cli_drc


@pytest.fixture
def drc_report(board_path, cfg):
    return run_kicad_cli_drc(board_path,
                             parity=cfg["native_drc"]["check_schematic_parity"])


def _desc(v):
    items = v.get("items", [])
    where = "; ".join(
        f"{i.get('description','')} @ ({i.get('pos',{}).get('x','?')},"
        f"{i.get('pos',{}).get('y','?')})" for i in items)
    return f"[{v.get('severity')}] {v.get('type')}: {v.get('description')} -- {where}"


def test