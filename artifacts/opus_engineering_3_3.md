# KiCad 8 DRC Validation Test Suite (Cycle 3)

I don't have the Cycle 1 or Cycle 2 artifacts, so this suite is self-contained. If you share the earlier thresholds or file layout, I can align names and limits with them.

The suite checks each rule in two independent ways:

1. **KiCad's own DRC engine**, run through `kicad-cli pcb drc` with JSON output.
2. **Direct geometry inspection** through the `pcbnew` Python API. This catches cases where the board's DRC rules are looser than the fab spec.

Aspect ratio is not a native KiCad 8 DRC check. It is enforced by a custom `.kicad_dru` rule and by the Python check.

---

## Layout

```
drc_tests/
├── fab_spec.yaml
├── fab_rules.kicad_dru
├── conftest.py
├── kicad_helpers.py
├── test_annular_ring.py
├── test_copper_edge_clearance.py
├── test_drill_aspect_ratio.py
├── test_validator_selfcheck.py
└── .github/workflows/drc.yml
```

---

## `fab_spec.yaml`

```yaml
# Fabrication limits. All lengths are in mm.
fab_house: "Generic Class 2"
annular_ring:
  via_min: 0.125          # (via_diameter - drill) / 2
  pth_pad_min: 0.150
copper_edge_clearance:
  min: 0.300
drill:
  min_through: 0.200
  max_aspect_ratio: 8.0   # board_thickness / finished_drill
boards:
  - path: "../hardware/main_board/main_board.kicad_pcb"
  - path: "../hardware/io_board/io_board.kicad_pcb"
```

---

## `fab_rules.kicad_dru`

Copy this into each project's `<project>.kicad_dru`. It makes KiCad's own DRC enforce the same limits.

```lisp
(version 1)

(rule "fab_via_annular"
  (condition "A.Type == 'Via'")
  (constraint annular_width (min 0.125mm)))

(rule "fab_pth_annular"
  (condition "A.Type == 'Pad' && A.Pad_Type == 'Through-hole'")
  (constraint annular_width (min 0.15mm)))

(rule "fab_edge_clearance"
  (constraint edge_clearance (min 0.3mm)))

(rule "fab_min_drill"
  (constraint hole_size (min 0.2mm)))

# Aspect ratio 8:1 on a 1.6 mm board means the drill must be at least 0.2 mm.
# Update this if the board thickness changes. The Python test computes it dynamically.
(rule "fab_aspect_ratio_1v6"
  (condition "A.isPlated()")
  (constraint hole_size (min 0.2mm)))
```

---

## `kicad_helpers.py`

```python
"""Helpers wrapping kicad-cli and the pcbnew API (KiCad 8.x)."""
from __future__ import annotations

import json
import shutil
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path

import pcbnew  # Use KiCad's bundled Python, or set PYTHONPATH accordingly.

NM_PER_MM = 1_000_000


def mm(nm: int | float) -> float:
    return nm / NM_PER_MM


def kicad_cli() -> str:
    exe = shutil.which("kicad-cli")
    if not exe:
        raise RuntimeError("kicad-cli not found on PATH (KiCad 8 required)")
    return exe


def kicad_version() -> str:
    out = subprocess.run([kicad_cli(), "version"], capture_output=True, text=True)
    return out.stdout.strip()


def run_drc(board: Path) -> dict:
    """Run KiCad DRC and return the parsed JSON report."""
    with tempfile.TemporaryDirectory() as td:
        report = Path(td) / "drc.json"
        cmd = [
            kicad_cli(), "pcb", "drc",
            "--format", "json",
            "--units", "mm",
            "--severity-all",
            "--schematic-parity" if False else "",  # parity is out of scope here
            "-o", str(report),
            str(board),
        ]
        cmd = [c for c in cmd if c]
        proc = subprocess.run(cmd, capture_output=True, text=True)
        if not report.exists():
            raise RuntimeError(f"DRC failed to produce a report:\n{proc.stderr}")
        return json.loads(report.read_text())


def violations_of(report: dict, *types: str) -> list[dict]:
    return [v for v in report.get("violations", []) if v.get("type") in types]


def fmt_violations(vs: list[dict]) -> str:
    lines = []
    for v in vs:
        locs = "; ".join(
            f"{i.get('description', '?')} @ ({i['pos']['x']:.3f},{i['pos']['y']:.3f})"
            for i in v.get("items", []) if "pos" in i
        )
        lines.append(f"  [{v.get('severity')}] {v.get('type')}: {v.get('description')} -> {locs}")
    return "\n".join(lines)


@dataclass
class HoleInfo:
    kind: str         # "via" or "pth"
    ref: str
    x_mm: float
    y_mm: float
    drill_mm: float
    outer_mm: float   # copper diameter (smallest pad dimension for pads)

    @property
    def annular_mm(self) -> float:
        return (self.outer_mm - self.drill_mm) / 2


def load_board(path: Path) -> pcbnew.BOARD:
    return pcbnew.LoadBoard(str(path))


def board_thickness_mm(board: pcbnew.BOARD) -> float:
    return mm(board.GetDesignSettings().GetBoardThickness())


def collect_holes(board: pcbnew.BOARD) -> list[HoleInfo]:
    holes: list[HoleInfo] = []

    for t in board.GetTracks():
        if isinstance(t, pcbnew.PCB_VIA) or t.Type() == pcbnew.PCB_VIA_T:
            via = pcbnew.Cast_to_PCB_VIA(t) if hasattr(pcbnew, "Cast_to_PCB_VIA") else t
            pos = via.GetPosition()
            holes.append(HoleInfo(
                kind="via",
                ref=f"via[{via.GetNetname() or 'no-net'}]",
                x_mm=mm(pos.x), y_mm=mm(pos.y),
                drill_mm=mm(via.GetDrillValue()),
                outer_mm=mm(via.GetWidth()),
            ))

    for pad in board.GetPads():
        if pad.GetAttribute() != pcbnew.PAD_ATTRIB_PTH:
            continue
        drill = pad.GetDrillSize()
        size = pad.GetSize()
        pos = pad.GetPosition()
        fp = pad.GetParentFootprint()
        ref = f"{fp.GetReference() if fp else '?'}.{pad.GetNumber()}"
        holes.append(HoleInfo(
            kind="pth",
            ref=ref,
            x_mm=mm(pos.x), y_mm=mm(pos.y),
            # Worst case: largest drill dimension against smallest pad dimension.
            drill_mm=mm(max(drill.x, drill.y)),
            outer_mm=mm(min(size.x, size.y)),
        ))
    return holes
```

---

## `conftest.py`

```python
from __future__ import annotations

from pathlib import Path

import pytest
import yaml

import kicad_helpers as kh

HERE = Path(__file__).parent
SPEC = yaml.safe_load((HERE / "fab_spec.yaml").read_text())
BOARD_PATHS = [(HERE / b["path"]).resolve() for b in SPEC["boards"]]


def pytest_report_header(config):
    return f"KiCad: {kh.kicad_version()} | fab spec: {SPEC['fab_house']}"


@pytest.fixture(scope="session")
def spec() -> dict:
    return SPEC


@pytest.fixture(scope="session", params=BOARD_PATHS, ids=lambda p: p.stem)
def board_path(request) -> Path:
    p: Path = request.param
    if not p.exists():
        pytest.fail(f"Board file missing: {p}")
    return p


_drc_cache: dict[Path, dict] = {}
_board_cache: dict[Path, object] = {}


@pytest.fixture(scope="session")
def drc_report(board_path) -> dict:
    if board_path not in _drc_cache:
        _drc_cache[board_path] = kh.run_drc(board_path)
    return _drc_cache[board_path]


@pytest.fixture(scope="session")
def board(board_path):
    if board_path not in _board_cache:
        _board_cache[board_path] = kh.load_board(board_path)
    return _board_cache[board_path]


@pytest.fixture(scope="session")
def holes(board):
    return kh.collect_holes(board)
```

---

## `test_annular_ring.py`

```python
import pytest

import kicad_helpers as kh

TOL = 1e-4  # mm, absorbs nm rounding


def test_drc_reports_no_annular_violations(drc_report):
    vs = kh.violations_of(drc_report, "annular_width")
    assert not vs, "KiCad DRC annular ring violations:\n" + kh.fmt_violations(vs)


def test_board_rules_not_looser_than_fab(board, spec):
    ds = board.GetDesignSettings()
    board_min = kh.mm(ds.m_ViasMinAnnularWidth)
    assert board_min + TOL >= spec["annular_ring"]["via_min"], (
        f"Board setup min annular {board_min:.3f} mm is looser than fab "
        f"{spec['annular_ring']['via_min']} mm. Tighten Board Setup or add fab_rules.kicad_dru."
    )


@pytest.mark.parametrize("kind,key", [("via", "via_min"), ("pth", "pth_pad_min")])
def test_geometric_annular_ring(holes, spec, kind, key):
    limit = spec["annular_ring"][key]
    bad = [h for h in holes if h.kind == kind and h.annular_mm + TOL < limit]
    assert not bad, f"{len(bad)} {kind} hole(s) below {limit} mm annular ring:\n" + "\n".join(
        f"  {h.ref} @ ({h.x_mm:.3f},{h.y_mm:.3f}): ring={h.annular_mm:.3f} "
        f"(Ø{h.outer_mm:.3f}/drill {h.drill_mm:.3f})"
        for h in bad[:50]
    )


def test_no_drill_exceeds_copper(holes):
    bad = [h for h in holes if h.drill_mm >= h.outer_mm]
    assert not bad, "Drill ≥ copper diameter (zero/negative ring):\n" + "\n".join(
        f"  {h.ref} @ ({h.x_mm:.3f},{h.y_mm:.3f})" for h in bad
    )
```

---

## `test_copper_edge_clearance.py`

```python
import kicad_helpers as kh

TOL = 1e-4


def test_drc_reports_no_edge_clearance_violations(drc_report):
    vs = kh.violations_of(drc_report, "copper_edge_clearance")
    assert not vs, "Copper-to-edge violations:\n" + kh.fmt_violations(vs)


def test_board_edge_clearance_setting(board, spec):
    ds = board.GetDesignSettings()
    setting = kh.mm(ds.m_CopperEdgeClearance)
    assert setting + TOL >= spec["copper_edge_clearance"]["min"], (
        f"Board setup copper-edge clearance {setting:.3f} mm < fab "
        f"{spec['copper_edge_clearance']['min']} mm. DRC would under-report."
    )


def test_board_outline_is_valid(drc_report):
    # A broken Edge.Cuts outline silently disables edge clearance checking.
    vs = kh.violations_of(drc_report, "invalid_outline")
    assert not vs, "Board outline invalid; edge clearance unverifiable:\n" + kh.fmt_violations(vs)


def test_zones_are_filled(drc_report):
    # Unfilled or stale zones can hide edge violations.
    vs = kh.violations_of(drc_report, "zones_intersect", "unfilled_zone")
    assert not vs, "Zone issues:\n" + kh.fmt_violations(vs)
```

---

## `test_drill_aspect_ratio.py`

```python
import kicad_helpers as kh

TOL = 1e-4


def test_board_thickness_defined(board):
    t = kh.board_thickness_mm(board)
    assert 0.2 <= t <= 6.0, f"Implausible board thickness {t:.3f} mm. Check the stackup."


def test_min_through_drill(holes, spec):
    limit = spec["drill"]["min_through"]
    bad = [h for h in holes if h.drill_mm + TOL < limit]
    assert not bad, f"Drills below {limit} mm:\n" + "\n".join(
        f"  {h.ref} drill={h.drill_mm:.3f}" for h in bad[:50]
    )


def test_aspect_ratio(board, holes, spec):
    thickness = kh.board_thickness_mm(board)
    max_ar = spec["drill"]["max_aspect_ratio"]
    bad = [(h, thickness / h.drill_mm) for h in holes
           if h.drill_mm > 0 and thickness / h.drill_mm > max_ar + TOL]
    assert not bad, (
        f"Aspect ratio > {max_ar}:1 on {thickness:.2f} mm board "
        f"(min drill {thickness / max_ar:.3f} mm):\n" + "\n".join(
            f"  {h.