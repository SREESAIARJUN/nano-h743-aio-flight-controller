# KiCad 8 DRC Validation Suite: Cycle 10

> **Note on context:** I don't have the artifacts from Cycles 1–9, such as your existing thresholds, fab house, or directory layout. This is a self-contained suite with every limit in one config file, so you can carry over your prior values. If Cycle 10 was meant to address specific earlier findings, share them and I'll adjust.

## Approach

Three independent layers, so one failing silently doesn't let violations through:

| Layer | Annular Ring | Copper-to-Edge | Drill Aspect Ratio |
|---|---|---|---|
| 1. `kicad-cli pcb drc` (native engine) | ✅ `annular_width` | ✅ `copper_edge_clearance` | ⚠️ via custom `.kicad_dru` (min-drill proxy) |
| 2. Independent geometry check (`pcbnew` API) | ✅ vias and PTH pads | ✅ design-settings verification | ✅ exact `thickness / drill` |
| 3. Config integrity (`.kicad_pro`) | ✅ severity not set to `ignore` | ✅ | ✅ |

There are also **negative fixtures**: known-bad boards that *must* fail. These prove the suite actually detects problems.

KiCad 8's rule engine has no native aspect-ratio constraint. Layer 2 therefore computes it directly from board thickness and drill size.

---

## Directory Layout

```
drc_suite/
├── drc_config.toml
├── rules/fab_limits.kicad_dru
├── conftest.py
├── lib/
│   ├── drc_runner.py
│   └── board_geometry.py
├── tests/
│   ├── test_native_drc.py
│   ├── test_geometry.py
│   ├── test_project_config.py
│   └── test_negative_fixtures.py
├── fixtures/
│   ├── bad_annular/bad_annular.kicad_pcb
│   ├── bad_edge/bad_edge.kicad_pcb
│   └── bad_aspect/bad_aspect.kicad_pcb
└── .github/workflows/drc.yml
```

---

## `drc_config.toml`

```toml
[meta]
cycle = 10
kicad_major = 8

[boards]
# Glob(s) relative to repo root
include = ["hardware/**/*.kicad_pcb"]
exclude = ["**/fixtures/**", "**/*-backups/**"]

[limits]
units = "mm"
min_annular_ring_via = 0.125     # typical std-class; tighten/loosen per fab
min_annular_ring_pth = 0.150
min_copper_edge_clearance = 0.300
max_aspect_ratio_through = 10.0  # board_thickness / finished_drill
min_drill_through = 0.200
geometry_tolerance = 0.0005      # mm, guards float rounding

[severity]
# DRC violation types that fail the build
blocking = [
  "annular_width",
  "copper_edge_clearance",
  "drill_out_of_range",
  "hole_clearance",
  "hole_to_hole",
  "via_diameter",
]
```

---

## `rules/fab_limits.kicad_dru`

Copy or merge this into each project's `<project>.kicad_dru` so that the GUI DRC and the CLI agree.

```lisp
(version 1)

(rule "fab_min_annular_via"
  (condition "A.Type == 'Via'")
  (constraint annular_width (min 0.125mm)))

(rule "fab_min_annular_pth"
  (condition "A.Type == 'Pad' && A.isPlated()")
  (constraint annular_width (min 0.15mm)))

(rule "fab_copper_edge"
  (constraint edge_clearance (min 0.3mm)))

# Aspect-ratio proxy: for a 1.6 mm board at 10:1, the minimum drill is 0.16 mm.
# The exact ratio is verified in test_geometry.py using real board thickness.
(rule "fab_min_drill"
  (constraint hole_size (min 0.2mm)))
```

---

## `lib/drc_runner.py`

```python
"""Wrapper around `kicad-cli pcb drc` (KiCad 8.x)."""
from __future__ import annotations
import json, shutil, subprocess
from dataclasses import dataclass, field
from pathlib import Path

@dataclass
class Violation:
    type: str
    severity: str
    description: str
    items: list = field(default_factory=list)

    def location(self) -> str:
        if not self.items:
            return "?"
        it = self.items[0]
        p = it.get("pos", {})
        return f"{it.get('description','')} @ ({p.get('x')}, {p.get('y')})"

@dataclass
class DrcReport:
    board: Path
    kicad_version: str
    violations: list[Violation]
    raw: dict

    def of_type(self, *types: str) -> list[Violation]:
        return [v for v in self.violations if v.type in types]

def find_kicad_cli() -> str:
    exe = shutil.which("kicad-cli")
    if not exe:
        raise RuntimeError("kicad-cli not found on PATH (KiCad 8 required)")
    return exe

def kicad_version(cli: str) -> str:
    out = subprocess.run([cli, "version"], capture_output=True, text=True, check=True)
    return out.stdout.strip()

def run_drc(board: Path, out_dir: Path) -> DrcReport:
    cli = find_kicad_cli()
    out_dir.mkdir(parents=True, exist_ok=True)
    report_path = out_dir / f"{board.stem}.drc.json"
    cmd = [
        cli, "pcb", "drc",
        "--format", "json",
        "--units", "mm",
        "--severity-all",
        "--all-track-errors",
        "--output", str(report_path),
        str(board),
    ]
    # No --exit-code-violations: we want the report and let pytest decide.
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0 or not report_path.exists():
        raise RuntimeError(
            f"kicad-cli failed on {board} (rc={proc.returncode}):\n"
            f"{proc.stdout}\n{proc.stderr}"
        )
    data = json.loads(report_path.read_text())
    viols = [
        Violation(v.get("type", ""), v.get("severity", ""),
                  v.get("description", ""), v.get("items", []))
        for v in data.get("violations", [])
    ]
    return DrcReport(board, data.get("kicad_version", "?"), viols, data)
```

---

## `lib/board_geometry.py`

This module is independent of the DRC engine and uses the KiCad 8 `pcbnew` Python API.

```python
"""Independent geometric checks using the KiCad 8 pcbnew API."""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path

import pcbnew  # ships with KiCad; use KiCad's bundled Python

@dataclass
class HoleRecord:
    kind: str          # "via" | "pth"
    ref: str           # footprint ref/pad or net name for vias
    x_mm: float
    y_mm: float
    drill_mm: float    # min drill dimension
    annular_mm: float  # min ring width
    aspect: float      # thickness / drill

def _mm(v) -> float:
    return pcbnew.ToMM(v)

def load(board_path: Path):
    return pcbnew.LoadBoard(str(board_path))

def board_thickness_mm(board) -> float:
    return _mm(board.GetDesignSettings().GetBoardThickness())

def copper_edge_clearance_setting_mm(board) -> float:
    return _mm(board.GetDesignSettings().m_CopperEdgeClearance)

def collect_holes(board) -> list[HoleRecord]:
    t = board_thickness_mm(board)
    out: list[HoleRecord] = []

    for trk in board.GetTracks():
        if not isinstance(trk, pcbnew.PCB_VIA):
            continue
        # Restrict to through vias; blind/buried use span thickness (not modelled here)
        if trk.GetViaType() != pcbnew.VIATYPE_THROUGH:
            continue
        drill = _mm(trk.GetDrillValue())
        dia = _mm(trk.GetWidth())
        pos = trk.GetPosition()
        out.append(HoleRecord(
            "via", trk.GetNetname() or "<no-net>",
            _mm(pos.x), _mm(pos.y), drill,
            (dia - drill) / 2.0,
            t / drill if drill > 0 else float("inf"),
        ))

    for pad in board.GetPads():
        if pad.GetAttribute() != pcbnew.PAD_ATTRIB_PTH:
            continue
        ds, ps = pad.GetDrillSize(), pad.GetSize()
        dx, dy = _mm(ds.x), _mm(ds.y)
        px, py = _mm(ps.x), _mm(ps.y)
        if dx <= 0 or dy <= 0:
            continue
        ring = min((px - dx) / 2.0, (py - dy) / 2.0)
        drill = min(dx, dy)
        pos = pad.GetPosition()
        ref = f"{pad.GetParentFootprint().GetReference()}.{pad.GetNumber()}"
        out.append(HoleRecord(
            "pth", ref, _mm(pos.x), _mm(pos.y), drill, ring,
            t / drill,
        ))
    return out
```

---

## `conftest.py`

```python
from __future__ import annotations
import tomllib
from pathlib import Path
import pytest

from lib.drc_runner import run_drc

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parent
CFG = tomllib.loads((ROOT / "drc_config.toml").read_text())

def _discover_boards() -> list[Path]:
    inc = CFG["boards"]["include"]
    exc = CFG["boards"]["exclude"]
    found = set()
    for pat in inc:
        found.update(REPO.glob(pat))
    return sorted(p for p in found if not any(p.match(e) for e in exc))

BOARDS = _discover_boards()

def pytest_generate_tests(metafunc):
    if "board_path" in metafunc.fixturenames:
        metafunc.parametrize("board_path", BOARDS,
                             ids=[b.stem for b in BOARDS] or None)

@pytest.fixture(scope="session")
def cfg():
    return CFG

@pytest.fixture(scope="session")
def report_dir(tmp_path_factory):
    return tmp_path_factory.mktemp("drc_reports")

_cache: dict = {}

@pytest.fixture
def drc_report(board_path, report_dir):
    if board_path not in _cache:
        _cache[board_path] = run_drc(board_path, report_dir)
    return _cache[board_path]

def test_boards_discovered():
    assert BOARDS, "No .kicad_pcb files matched drc_config.toml [boards].include"
```

---

## `tests/test_native_drc.py`

```python
import pytest

def _fmt(viols, limit=25):
    lines = [f"  - [{v.severity}] {v.type}: {v.description} | {v.location()}"
             for v in viols[:limit]]
    if len(viols) > limit:
        lines.append(f"  ... and {len(viols) - limit} more")
    return "\n".join(lines)

def test_kicad_version_is_8(drc_report):
    assert drc_report.kicad_version.startswith("8."), \
        f"Expected KiCad 8.x, got {drc_report.kicad_version}"

def test_annular_ring_native(drc_report):
    v = drc_report.of_type("annular_width")
    assert not v, f"Annular ring violations:\n{_fmt(v)}"

def test_copper_edge_clearance_native(drc_report):
    v = drc_report.of_type("copper_edge_clearance")
    assert not v, f"Copper-to-edge violations:\n{_fmt(v)}"

def test_drill_size_native(drc_report):
    v = drc_report.of_type("drill_out_of_range", "hole_size")
    assert not v, f"Drill size violations:\n{_fmt(v)}"

def test_no_other_blocking(drc_report, cfg):
    v = drc_report.of_type(*cfg["severity"]["blocking"])
    v = [x for x in v if x.severity == "error"]
    assert not v, f"Blocking DRC errors:\n{_fmt(v)}"
```

---

## `tests/test_geometry.py`

```python
import pytest
from lib import board_geometry as bg

@pytest.fixture
def board(board_path):
    return bg.load(board_path)

@pytest.fixture
def holes(board):
    return bg.collect_holes(board)

def _fmt(rows):
    return "\n".join(
        f"  - {h.kind} {h.ref} @ ({h.x_mm:.3f},{h.y_mm:.3f}) "
        f"drill={h.drill_mm:.3f} ring={h.annular_mm:.3f} AR={h.aspect:.2f}"
        for h in rows[:25])

def test_annular_ring_vias(holes, cfg):
    lim, tol = cfg["limits"]["min_annular_ring_via"], cfg["limits"]["geometry_tolerance"]
    bad = [h for h in holes if h.kind == "via" and h.annular_mm + tol < lim]
    assert not bad, f"Via annular ring < {lim} mm:\n{_fmt(bad)}"

def test_annular_ring_pth(holes, cfg):
    lim, tol = cfg["limits"]["min_annular_ring_p