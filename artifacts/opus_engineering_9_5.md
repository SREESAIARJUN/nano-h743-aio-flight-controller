# KiCad 8 DRC Validation Test Suite: Annular Rings, Copper-to-Edge Clearance, Drill Aspect Ratio

**Note on "Cycle 9":** I don't have access to earlier cycles or your project's prior thresholds and conventions. This suite is self-contained. All limits live in one config file, so you can plug in your Cycle 9 values. If earlier cycles fixed specific numbers or naming, share them and I'll align the suite.

---

## 1. Approach

The suite uses two independent layers so that a misconfigured rule file cannot hide a defect:

| Check | Layer A: native `kicad-cli` DRC with injected `.kicad_dru` rules | Layer B: independent geometry check via `pcbnew` Python API |
|---|---|---|
| Annular ring | `annular_width` constraint | Computes (pad or via size − drill) / 2 per item |
| Copper-to-edge | `edge_clearance` constraint | Uses the native result. Exact edge geometry is best left to the DRC engine. |
| Drill aspect ratio | Not available as a native KiCad constraint | Board thickness ÷ drill diameter per hole |

---

## 2. Directory Layout

```
drc_suite/
├── drc_config.yaml
├── rules/
│   └── fab_limits.kicad_dru.j2
├── conftest.py
├── lib/
│   ├── __init__.py
│   ├── drc_runner.py
│   └── board_geometry.py
├── tests/
│   ├── test_annular_ring.py
│   ├── test_copper_edge_clearance.py
│   ├── test_drill_aspect_ratio.py
│   └── test_native_drc_clean.py
└── .github/workflows/drc.yml
```

---

## 3. Configuration: `drc_config.yaml`

```yaml
# Replace with your Cycle 9 fab limits.
boards_glob: "hardware/**/*.kicad_pcb"

units: mm

annular_ring:
  via_min: 0.125        # external & internal via annular ring
  pth_pad_min: 0.150    # plated through-hole component pads
  check_npth: false     # NPTH has no copper ring requirement

copper_edge:
  min_clearance: 0.300  # copper to Edge.Cuts
  include_zones: true

drill_aspect_ratio:
  max_ratio_via: 10.0        # thickness : drill
  max_ratio_pth: 8.0
  fallback_board_thickness: 1.6   # used only if the stackup reports 0

# Severity of the native DRC items that fail the build
fail_on_types:
  - annular_width
  - copper_edge_clearance
  - drill_out_of_range
  - via_diameter
```

---

## 4. Injected Rule Template: `rules/fab_limits.kicad_dru.j2`

This uses KiCad 8 custom rule syntax.

```lisp
(version 1)

(rule "CYCLE9_via_annular_ring"
  (condition "A.Type == 'Via'")
  (constraint annular_width (min {{ annular_ring.via_min }}mm)))

(rule "CYCLE9_pth_annular_ring"
  (condition "A.Type == 'Pad' && A.Pad_Type == 'Through-hole'")
  (constraint annular_width (min {{ annular_ring.pth_pad_min }}mm)))

(rule "CYCLE9_copper_to_edge"
  (constraint edge_clearance (min {{ copper_edge.min_clearance }}mm)))
```

If the project already has a `.kicad_dru`, the runner appends these rules to it instead of replacing them.

---

## 5. `lib/drc_runner.py`: Native DRC via `kicad-cli`

```python
"""Runs KiCad 8 native DRC with injected fab-limit rules and returns parsed JSON."""
from __future__ import annotations

import json
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path

import jinja2


@dataclass
class Violation:
    type: str
    severity: str
    description: str
    items: list[dict]

    def location_str(self) -> str:
        locs = []
        for it in self.items:
            pos = it.get("pos", {})
            locs.append(f"{it.get('description', '?')} @ ({pos.get('x')}, {pos.get('y')})")
        return "; ".join(locs)


def _render_rules(template: Path, cfg: dict) -> str:
    env = jinja2.Environment(undefined=jinja2.StrictUndefined)
    return env.from_string(template.read_text()).render(**cfg)


def run_drc(board: Path, cfg: dict, rules_template: Path, workdir: Path) -> list[Violation]:
    """Copy board + project into an isolated dir, inject rules, run kicad-cli DRC."""
    stem = board.stem
    dst = workdir / stem
    dst.mkdir(parents=True, exist_ok=True)

    for ext in (".kicad_pcb", ".kicad_pro", ".kicad_dru"):
        src = board.with_suffix(ext)
        if src.exists():
            shutil.copy2(src, dst / src.name)

    # KiCad reads <project>.kicad_dru next to the .kicad_pro
    pro = dst / f"{stem}.kicad_pro"
    if not pro.exists():
        pro.write_text("{}")  # minimal project so the rules file is honored

    dru = dst / f"{stem}.kicad_dru"
    injected = _render_rules(rules_template, cfg)
    if dru.exists():
        existing = dru.read_text()
        # strip duplicate version header from the injected block
        injected_body = injected.replace("(version 1)", "", 1)
        dru.write_text(existing.rstrip() + "\n\n" + injected_body)
    else:
        dru.write_text(injected)

    report = dst / "drc.json"
    cmd = [
        "kicad-cli", "pcb", "drc",
        "--format", "json",
        "--severity-all",
        "--units", "mm",
        "--output", str(report),
        str(dst / board.name),
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if not report.exists():
        raise RuntimeError(
            f"kicad-cli DRC produced no report for {board}\n"
            f"stdout:\n{proc.stdout}\nstderr:\n{proc.stderr}"
        )

    data = json.loads(report.read_text())
    out: list[Violation] = []
    for v in data.get("violations", []) + data.get("unconnected_items", []):
        out.append(Violation(
            type=v.get("type", ""),
            severity=v.get("severity", ""),
            description=v.get("description", ""),
            items=v.get("items", []),
        ))
    return out
```

---

## 6. `lib/board_geometry.py`: Independent Checks via the `pcbnew` API

```python
"""Independent geometric checks using the KiCad 8 pcbnew Python API."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import pcbnew  # provided by the KiCad 8 install / docker image


@dataclass
class HoleItem:
    kind: str          # "via" | "pth" | "npth"
    ref: str           # footprint ref + pad number, or net name for vias
    x_mm: float
    y_mm: float
    drill_mm: float    # smallest drill dimension (handles slots)
    outer_mm: float    # smallest copper dimension
    @property
    def annular_mm(self) -> float:
        return (self.outer_mm - self.drill_mm) / 2.0


def load(board_path: Path) -> "pcbnew.BOARD":
    return pcbnew.LoadBoard(str(board_path))


def board_thickness_mm(board, fallback: float) -> float:
    t = pcbnew.ToMM(board.GetDesignSettings().GetBoardThickness())
    return t if t > 0 else fallback


def collect_holes(board) -> list[HoleItem]:
    items: list[HoleItem] = []

    for t in board.GetTracks():
        if isinstance(t, pcbnew.PCB_VIA):
            pos = t.GetPosition()
            items.append(HoleItem(
                kind="via",
                ref=f"via[{t.GetNetname() or 'no-net'}]",
                x_mm=pcbnew.ToMM(pos.x), y_mm=pcbnew.ToMM(pos.y),
                drill_mm=pcbnew.ToMM(t.GetDrillValue()),
                outer_mm=pcbnew.ToMM(t.GetWidth()),
            ))

    for pad in board.GetPads():
        attr = pad.GetAttribute()
        if attr == pcbnew.PAD_ATTRIB_PTH:
            kind = "pth"
        elif attr == pcbnew.PAD_ATTRIB_NPTH:
            kind = "npth"
        else:
            continue
        drill = pad.GetDrillSize()
        size = pad.GetSize()
        pos = pad.GetPosition()
        fp = pad.GetParentFootprint()
        ref = f"{fp.GetReference() if fp else '?'}.{pad.GetNumber()}"
        items.append(HoleItem(
            kind=kind, ref=ref,
            x_mm=pcbnew.ToMM(pos.x), y_mm=pcbnew.ToMM(pos.y),
            drill_mm=pcbnew.ToMM(min(drill.x, drill.y)),
            outer_mm=pcbnew.ToMM(min(size.x, size.y)),
        ))
    return items
```

**Limitation:** For pads that use custom shapes or per-layer padstacks, `min(size)` is conservative. KiCad 8 has a single padstack per pad, so this is accurate for standard round, oval, and rectangular pads.

---

## 7. `conftest.py`

```python
from pathlib import Path
import pytest
import yaml

ROOT = Path(__file__).parent
CFG = yaml.safe_load((ROOT / "drc_config.yaml").read_text())
REPO = ROOT.parent if (ROOT.parent / ".git").exists() else ROOT


def _boards():
    boards = sorted(REPO.glob(CFG["boards_glob"]))
    # skip KiCad autosave / backup files
    return [b for b in boards if not b.name.startswith(("_autosave", "~"))]


BOARDS = _boards()


def pytest_generate_tests(metafunc):
    if "board_path" in metafunc.fixturenames:
        if not BOARDS:
            pytest.fail(f"No boards matched {CFG['boards_glob']!r}")
        metafunc.parametrize("board_path", BOARDS, ids=[b.stem for b in BOARDS])


@pytest.fixture(scope="session")
def cfg():
    return CFG


@pytest.fixture(scope="session")
def rules_template():
    return ROOT / "rules" / "fab_limits.kicad_dru.j2"


_drc_cache: dict = {}


@pytest.fixture
def drc_violations(board_path, cfg, rules_template, tmp_path_factory):
    """Run native DRC once per board per session."""
    from lib.drc_runner import run_drc
    if board_path not in _drc_cache:
        wd = tmp_path_factory.mktemp(f"drc_{board_path.stem}")
        _drc_cache[board_path] = run_drc(board_path, cfg, rules_template, wd)
    return _drc_cache[board_path]


@pytest.fixture
def board(board_path):
    from lib.board_geometry import load
    return load(board_path)
```

---

## 8. Tests

### `tests/test_annular_ring.py`

```python
from lib.board_geometry import collect_holes

TOL = 1e-4  # mm, absorbs nm rounding


def test_annular_ring_native(drc_violations):
    bad = [v for v in drc_violations if v.type == "annular_width"]
    assert not bad, "Native DRC annular ring violations:\n" + "\n".join(
        f"  - {v.description} :: {v.location_str()}" for v in bad)


def test_annular_ring_geometry(board, cfg):
    ar = cfg["annular_ring"]
    failures = []
    for h in collect_holes(board):
        if h.kind == "npth" and not ar["check_npth"]:
            continue
        limit = ar["via_min"] if h.kind == "via" else ar["pth_pad_min"]
        if h.annular_mm + TOL < limit:
            failures.append(
                f"  - {h.kind} {h.ref} @ ({h.x_mm:.3f},{h.y_mm:.3f}) "
                f"ring={h.annular_mm:.4f}mm < {limit}mm "
                f"(outer={h.outer_mm:.3f}, drill={h.drill_mm:.3f})")
    assert not failures, "Annular ring below limit:\n" + "\n".join(failures)
```

### `tests/test_copper_edge_clearance.py`

```python
def test_copper_to_edge_clearance(drc_violations, cfg):
    bad = [v for v in drc_violations if v.type == "copper_edge_clearance"]
    if not cfg["copper_edge"]["include_zones"]:
        bad = [v for v in bad
               if not any("Zone" in i.get("description", "") for i in v.items)]
    assert not bad, (
        f"Copper within {cfg['copper_edge']['min_clearance']}mm of board edge:\n"
        + "\n".join(f"  - {v.description} :: {v.location_str()}" for v in bad))


def test_board_has_closed_outline(drc_violations):
    """Edge clearance is meaningless without a valid Edge.Cuts outline."""
    bad = [v for v in drc_violations if v.type in ("invalid_outline",)]
    assert not bad, "Board outline invalid:\n"