# KiCad 8 DRC Validation Test Suite: Annular Rings, Edge Clearance, Drill Aspect Ratio

I don't have context for what "Cycle 8" refers to in your process, such as a fab-house revision, a release gate, or an earlier spec. All limits therefore live in one config file. Set those values to your Cycle 8 numbers and the suite enforces them.

## How It Works

The suite verifies each board in two independent ways.

1. **KiCad's own DRC engine, with fab rules injected.** The suite copies the project to a temp directory and appends fab-limit rules to the `.kicad_dru` file. It then runs `kicad-cli pcb drc` and parses the JSON output. The injected rules come last, so they take priority over the designer's own rules. As a result, a designer cannot pass by loosening the board's settings.
2. **Direct geometry checks through the `pcbnew` Python API.**
   - It recomputes annular rings for vias and PTH pads, including pad drill offset.
   - It computes drill aspect ratios. KiCad has no native DRC constraint for aspect ratio, so this check is only done here.

A self-test then builds deliberately bad boards and confirms that every check catches them. This guards against tests that silently pass everything.

```
drc_suite/
├── fab_limits.yaml
├── conftest.py
├── drc_lib.py
├── test_drc_engine.py
├── test_geometry.py
├── test_selftest.py
└── .github/workflows/drc.yml
```

---

## `fab_limits.yaml`

```yaml
# Cycle 8 fabrication limits — edit to match your fab's capability sheet.
cycle: 8
annular_ring:
  via_min_mm: 0.125          # (via pad dia - finished drill) / 2
  pth_min_mm: 0.200          # through-hole component pads
copper_to_edge:
  min_mm: 0.300              # copper (tracks, pads, zones, vias) to Edge.Cuts
drill_aspect_ratio:
  through_max: 10.0          # board thickness / finished drill dia
  microvia_max: 0.8          # depth / drill (laser microvias)
  microvia_depth_mm: 0.075   # dielectric depth used for microvia AR
  blind_buried_max: 10.0
  blind_buried_depth_mm: null  # null = use full board thickness (conservative)
boards:
  glob: "boards/**/*.kicad_pcb"
```

---

## `drc_lib.py`

```python
"""Shared helpers: config, kicad-cli DRC runner, pcbnew geometry extraction."""
from __future__ import annotations

import json
import shutil
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path

import yaml

CFG_PATH = Path(__file__).with_name("fab_limits.yaml")


def load_cfg() -> dict:
    return yaml.safe_load(CFG_PATH.read_text())


# ---------------------------------------------------------------- DRC engine
def fab_rules_text(cfg: dict, include_version: bool) -> str:
    ar, ec = cfg["annular_ring"], cfg["copper_to_edge"]
    hdr = "(version 1)\n" if include_version else "\n# ---- injected fab rules ----\n"
    return hdr + f"""
(rule "FAB_C{cfg['cycle']}_annular_via"
  (constraint annular_width (min {ar['via_min_mm']}mm))
  (condition "A.Type == 'Via'"))

(rule "FAB_C{cfg['cycle']}_annular_pth"
  (constraint annular_width (min {ar['pth_min_mm']}mm))
  (condition "A.Type == 'Pad' && A.Pad_Type == 'Through-hole'"))

(rule "FAB_C{cfg['cycle']}_edge_clearance"
  (constraint edge_clearance (min {ec['min_mm']}mm)))
"""


def run_drc(board: Path, cfg: dict) -> dict:
    """Copy project to temp dir, inject fab rules, run kicad-cli DRC, return JSON."""
    if shutil.which("kicad-cli") is None:
        raise RuntimeError("kicad-cli not on PATH (KiCad 8 required)")

    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        # copy every sibling project file so .kicad_pro / .kicad_dru resolve
        for f in board.parent.iterdir():
            if f.is_file() and f.stem == board.stem:
                shutil.copy2(f, td / f.name)
        work = td / board.name
        dru = work.with_suffix(".kicad_dru")
        if dru.exists():
            dru.write_text(dru.read_text() + fab_rules_text(cfg, False))
        else:
            dru.write_text(fab_rules_text(cfg, True))

        out = td / "drc.json"
        proc = subprocess.run(
            ["kicad-cli", "pcb", "drc", "--format", "json", "--severity-all",
             "--units", "mm", "--output", str(out), str(work)],
            capture_output=True, text=True,
        )
        if not out.exists():
            raise RuntimeError(f"kicad-cli DRC failed:\n{proc.stdout}\n{proc.stderr}")
        return json.loads(out.read_text())


def violations_of(report: dict, *types: str) -> list[dict]:
    return [v for v in report.get("violations", []) if v.get("type") in types]


def fmt_violation(v: dict) -> str:
    locs = "; ".join(
        f"{i.get('description','?')} @ ({i['pos']['x']:.3f},{i['pos']['y']:.3f})"
        for i in v.get("items", []) if "pos" in i
    )
    return f"[{v['type']}] {v.get('description','')} :: {locs}"


# --------------------------------------------------------- geometry (pcbnew)
@dataclass
class HoleItem:
    kind: str           # "via-through" | "via-blind" | "via-micro" | "pth"
    ref: str
    x_mm: float
    y_mm: float
    pad_mm: float       # smallest copper dimension
    drill_mm: float     # smallest drill dimension (finished)
    ring_mm: float


def extract_holes(board_path: Path) -> tuple[list[HoleItem], float]:
    import pcbnew  # imported lazily; requires KiCad's Python

    b = pcbnew.LoadBoard(str(board_path))
    mm = pcbnew.ToMM
    thickness = mm(b.GetDesignSettings().GetBoardThickness())
    items: list[HoleItem] = []

    for t in b.GetTracks():
        if not isinstance(t, pcbnew.PCB_VIA):
            continue
        vt = t.GetViaType()
        kind = {pcbnew.VIATYPE_THROUGH: "via-through",
                pcbnew.VIATYPE_BLIND_BURIED: "via-blind",
                pcbnew.VIATYPE_MICROVIA: "via-micro"}.get(vt, "via-through")
        w, d = mm(t.GetWidth()), mm(t.GetDrillValue())
        p = t.GetPosition()
        items.append(HoleItem(kind, f"via:{t.GetNetname() or '<no net>'}",
                              mm(p.x), mm(p.y), w, d, (w - d) / 2))

    for pad in b.GetPads():
        if pad.GetAttribute() != pcbnew.PAD_ATTRIB_PTH:
            continue
        sz, dr, off = pad.GetSize(), pad.GetDrillSize(), pad.GetOffset()
        # ring per axis, reduced by drill offset; take the worst axis
        ring_x = (mm(sz.x) - mm(dr.x)) / 2 - abs(mm(off.x))
        ring_y = (mm(sz.y) - mm(dr.y)) / 2 - abs(mm(off.y))
        p = pad.GetPosition()
        fp = pad.GetParentFootprint()
        ref = f"{fp.GetReference() if fp else '?'}.{pad.GetNumber()}"
        items.append(HoleItem("pth", ref, mm(p.x), mm(p.y),
                              min(mm(sz.x), mm(sz.y)),
                              min(mm(dr.x), mm(dr.y)),
                              min(ring_x, ring_y)))
    return items, thickness


def aspect_limit_and_depth(item: HoleItem, cfg: dict, thickness: float):
    a = cfg["drill_aspect_ratio"]
    if item.kind == "via-micro":
        return a["microvia_max"], a["microvia_depth_mm"]
    if item.kind == "via-blind":
        return a["blind_buried_max"], a["blind_buried_depth_mm"] or thickness
    return a["through_max"], thickness
```

---

## `conftest.py`

```python
import os
from pathlib import Path

import pytest

from drc_lib import load_cfg, run_drc, extract_holes

CFG = load_cfg()
ROOT = Path(__file__).parent


def _boards():
    override = os.environ.get("DRC_BOARDS")  # e.g. "a.kicad_pcb:b.kicad_pcb"
    if override:
        return [Path(p) for p in override.split(os.pathsep)]
    return sorted(ROOT.glob(CFG["boards"]["glob"]))


def pytest_generate_tests(metafunc):
    if "board" in metafunc.fixturenames:
        boards = _boards()
        metafunc.parametrize("board", boards, ids=[b.stem for b in boards])


@pytest.fixture(scope="session")
def cfg():
    return CFG


_drc_cache, _geo_cache = {}, {}


@pytest.fixture
def drc_report(board, cfg):
    if board not in _drc_cache:
        _drc_cache[board] = run_drc(board, cfg)
    return _drc_cache[board]


@p