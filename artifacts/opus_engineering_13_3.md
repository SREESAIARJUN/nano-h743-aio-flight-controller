# KiCad 8 DRC Validation Test Suite: Annular Rings, Copper-to-Edge Clearance, Drill Aspect Ratio

**Note on "Cycle 13":** I don't have the context from earlier cycles, such as your prior thresholds, board list, or fab house constraints. All limits below are in a single config file (`drc_limits.yaml`). Replace them with your Cycle 13 values. The defaults are typical IPC-2221 Class 2 / standard-fab numbers.

---

## 1. Layout

```
drc-suite/
├── drc_limits.yaml
├── conftest.py
├── lib/
│   ├── __init__.py
│   ├── kicad_cli.py          # wraps `kicad-cli pcb drc`
│   └── board_geom.py         # independent checks via pcbnew Python API
├── tests/
│   ├── test_design_rules_configured.py
│   ├── test_kicad_drc_report.py
│   ├── test_annular_ring.py
│   ├── test_copper_edge_clearance.py
│   ├── test_drill_aspect_ratio.py
│   └── test_suite_self_check.py   # known-bad fixtures must FAIL
├── fixtures/
│   ├── good_board/good.kicad_pcb
│   ├── bad_annular/bad_annular.kicad_pcb
│   ├── bad_edge/bad_edge.kicad_pcb
│   └── bad_aspect/bad_aspect.kicad_pcb
└── .github/workflows/drc.yml
```

The suite uses two independent layers:

1. **KiCad's own DRC** (`kicad-cli`, JSON report) is the authoritative check.
2. **Independent geometric re-computation** (pcbnew API) catches cases where the board's rule settings were loosened. KiCad DRC only enforces whatever is in the board file.

---

## 2. `drc_limits.yaml`

```yaml
# All dimensions in millimetres. Replace with Cycle 13 fab values.
boards:
  - path: hw/main/main.kicad_pcb
  - path: hw/io/io.kicad_pcb

limits:
  annular_ring:
    via_min_mm: 0.125          # (via_dia - drill) / 2
    pth_pad_min_mm: 0.150      # worst-case axis for non-circular pads
  copper_to_edge:
    min_mm: 0.300
    include_zones: true
  drill_aspect_ratio:
    max_through: 10.0          # board_thickness / finished_drill
    max_microvia: 0.8          # dielectric_thickness / drill (if used)
  min_drill_mm: 0.200

kicad_drc:
  fail_on_severity: error      # error | warning
  required_clean_types:        # KiCad 8 violation type keys
    - annular_width
    - copper_edge_clearance
    - drill_out_of_range
    - via_diameter
    - hole_clearance
  allow_exclusions: false      # treat GUI-excluded markers as failures
  timeout_s: 300
```

---

## 3. `lib/kicad_cli.py`

```python
"""Thin wrapper around KiCad 8 `kicad-cli pcb drc`."""
from __future__ import annotations
import json, shutil, subprocess, tempfile
from pathlib import Path

KICAD_CLI = shutil.which("kicad-cli") or "kicad-cli"


def kicad_version() -> str:
    out = subprocess.run([KICAD_CLI, "version"], capture_output=True, text=True, check=True)
    return out.stdout.strip()


def run_drc(board: Path, timeout_s: int = 300, schematic_parity: bool = False) -> dict:
    with tempfile.TemporaryDirectory() as td:
        report = Path(td) / "drc.json"
        cmd = [
            KICAD_CLI, "pcb", "drc",
            "--format", "json",
            "--units", "mm",
            "--severity-all",
            "--all-track-errors",
            "--refill-zones",          # zone fill must be current for edge clearance
            "-o", str(report),
        ]
        if schematic_parity:
            cmd.append("--schematic-parity")
        cmd.append(str(board))
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s)
        if not report.exists():
            raise RuntimeError(
                f"kicad-cli produced no report (rc={proc.returncode})\n"
                f"stdout:\n{proc.stdout}\nstderr:\n{proc.stderr}"
            )
        return json.loads(report.read_text())


def iter_violations(report: dict, include_excluded: bool):
    """Yield violations from both 'violations' and 'unconnected_items' sections."""
    for section in ("violations", "unconnected_items", "schematic_parity"):
        for v in report.get(section, []) or []:
            if v.get("excluded") and not include_excluded:
                continue
            yield v
```

---

## 4. `lib/board_geom.py`: Independent Checks

```python
"""Independent geometry checks using the KiCad 8 pcbnew Python API.
Run with KiCad's bundled Python (e.g. `/usr/bin/python3` on Linux packages,
or `C:\\Program Files\\KiCad\\8.0\\bin\\python.exe` on Windows)."""
from __future__ import annotations
from dataclasses import dataclass
import pcbnew

NM_PER_MM = 1_000_000
to_mm = lambda nm: nm / NM_PER_MM


@dataclass
class Finding:
    kind: str
    ref: str
    pos_mm: tuple[float, float]
    value: float
    limit: float

    def __str__(self):
        return (f"{self.kind} {self.ref} @({self.pos_mm[0]:.3f},{self.pos_mm[1]:.3f}) "
                f"value={self.value:.4f} limit={self.limit:.4f}")


def load(path: str) -> pcbnew.BOARD:
    return pcbnew.LoadBoard(str(path))


def _pos(item):
    p = item.GetPosition()
    return (to_mm(p.x), to_mm(p.y))


def _pad_ref(pad):
    fp = pad.GetParentFootprint()
    return f"{fp.GetReference() if fp else '?'}.{pad.GetNumber()}"


# ---------------------------------------------------------------- vias
def iter_vias(board):
    for t in board.GetTracks():
        if isinstance(t, pcbnew.PCB_VIA):
            yield t


def via_annular_ring_mm(via) -> float:
    # KiCad 8: PCB_VIA.GetWidth() is the pad diameter (single value for all layers)
    return to_mm(via.GetWidth() - via.GetDrillValue()) / 2


def is_microvia(via) -> bool:
    return via.GetViaType() == pcbnew.VIATYPE_MICROVIA


# ---------------------------------------------------------------- PTH pads
def iter_pth_pads(board):
    for pad in board.GetPads():
        if pad.GetAttribute() == pcbnew.PAD_ATTRIB_PTH:
            yield pad


def pad_annular_ring_mm(pad) -> float:
    """Worst-case axis ring. Ignores pad/hole offset first, then subtracts it."""
    size, drill, off = pad.GetSize(), pad.GetDrillSize(), pad.GetOffset()
    ring_x = (size.x - drill.x) / 2 - abs(off.x)
    ring_y = (size.y - drill.y) / 2 - abs(off.y)
    return to_mm(min(ring_x, ring_y))


# ---------------------------------------------------------------- aspect ratio
def board_thickness_mm(board) -> float:
    return to_mm(board.GetDesignSettings().GetBoardThickness())


def all_plated_holes(board):
    """(kind, ref, pos, finished_drill_mm, item)"""
    for v in iter_vias(board):
        yield ("via", f"via[{v.GetNetname()}]", _pos(v), to_mm(v.GetDrillValue()), v)
    for p in iter_pth_pads(board):
        d = p.GetDrillSize()
        yield ("pth", _pad_ref(p), _pos(p), to_mm(min(d.x, d.y)), p)


# ---------------------------------------------------------------- edge clearance
def edge_shapes(board):
    out = []
    for d in board.GetDrawings():
        if d.GetLayer() == pcbnew.Edge_Cuts:
            out.append(d)
    for fp in board.GetFootprints():          # footprint-level Edge.Cuts (cutouts)
        for g in fp.GraphicalItems():
            if g.GetLayer() == pcbnew.Edge_Cuts:
                out.append(g)
    return out


def copper_items(board, include_zones: bool):
    copper = set(board.GetEnabledLayers().CuStack())
    for t in board.GetTracks():
        for layer in copper:
            if t.IsOnLayer(layer):
                yield (t, layer, f"{t.GetClass()}[{t.GetNetname()}]")
                if not isinstance(t, pcbnew.PCB_VIA):
                    break
    for pad in board.GetPads():
        for layer in copper:
            if pad.IsOnLayer(layer) and pad.FlashLayer(layer):
                yield (pad, layer, _pad_ref(pad))
    if include_zones:
        for z in board.Zones():
            if z.GetIsRuleArea():
                continue
            for layer in copper:
                if z.IsOnLayer(layer) and z.HasFilledPolysForLayer(layer):
                    yield (z, layer, f"zone[{z.GetNetname()}]")


def copper_edge_violations(board, min_mm: float, include_zones: bool):
    """Uses SHAPE::Collide(other, clearance, &actual) for exact distance."""
    clearance = int(min_mm * NM_PER_MM)
    edges = [(e, e.GetEffectiveShape()) for e in edge_shapes(board)]
    for item, layer, ref in copper_items(board, include_zones):
        shape = item.GetEffectiveShape(layer)
        for _edge, eshape in edges:
            # Collide returns True if distance < clearance.
            # Subtract 1 nm so exact-limit geometry passes.
            if shape.Collide(eshape, clearance - 1):
                yield Finding("copper_to_edge", f"{ref}@{board.GetLayerName(layer)}",
                              _pos(item), float("nan"), min_mm)
                break
```

> **SWIG note:** The `actual` out-parameter of `SHAPE::Collide` isn't reliably exposed in the KiCad 8 bindings. This check therefore reports pass/fail and leaves the measured distance to KiCad's DRC report, which gives exact values.

---

## 5. `conftest.py`

```python
import os, sys
from pathlib import Path
import pytest, yaml

ROOT = Path(__file__).parent
sys.path.insert(0, str(ROOT))
CFG = yaml.safe_load((ROOT / "drc_limits.yaml").read_text())


def _boards():
    override = os.environ.get("DRC_BOARDS")      # colon/semicolon-separated list
    if override:
        return [Path(p) for p in override.replace(";", ":").split(":") if p]
    return [Path(b["path"]) for b in CFG["boards"]]


@pytest.fixture(scope="session")
def cfg():
    return CFG


@pytest.fixture(scope="session")
def limits():
    return CFG["limits"]


def pytest_generate_tests(metafunc):
    if "board_path" in metafunc.fixturenames:
        boards = _boards()
        metafunc.parametrize("board_path", boards, ids=[b.stem for b in boards], scope="session")


_board_cache, _drc_cache = {}, {}


@pytest.fixture(scope="session")
def board(board_path):
    from lib.board_geom import load
    if board_path not in _board_cache:
        assert board_path.exists(), f"missing board {board_path}"
        _board_cache[board_path] = load(board_path)
    return _board_cache[board_path]


@pytest.fixture(scope="session")
def drc_report(board_path, cfg):
    from lib.kicad_cli import run_drc
    if board_path not in _drc_cache:
        _drc_cache[board_path] = run_drc(board_path, cfg["kicad_drc"]["timeout_s"])
    return _drc_cache[board_path]


def fmt(findings, limit=25):
    lines = [str(f) for f in findings[:limit]]
    if len(findings) > limit:
        lines.append(f"... and {len(findings) - limit} more")
    return "\n".join(lines)
```

---

## 6. Tests

### `tests/test_design_rules_configured.py`

This test guards against someone loosening the board's own rules so that KiCad DRC passes falsely.

```python
from lib.board_geom import to_mm


def test_min_annular_ring_rule(board, limits):
    ds = board.GetDesignSettings()
    assert to_mm(ds.m_MinRingWidth) >= limits["annular_ring"]["via_min_mm"] - 1e-6, \
        f"Board rule min annular width {to_mm(ds.m_MinRingWidth)} mm is looser than required"


def test_copper_edge_clearance_rule(board, limits):
    ds = board.GetDesignSettings()
    assert to_mm(ds.m_CopperEdgeClearance) >= limits["copper_to_edge"]["min_mm"] - 1e-6


def test_min_through_drill_rule(board, limits):
    ds = board.GetDesignSettings()
    assert to_mm(ds.m_MinThroughDrill) >= limits["min_drill_mm"] - 1e-6


def test_board_thickness_sane(board):
    t = to_mm(board.GetDesignSettings().GetBoardThickness())
    assert 0.2 <= t <= 6.0, f"Implausible board thickness {t} mm, so