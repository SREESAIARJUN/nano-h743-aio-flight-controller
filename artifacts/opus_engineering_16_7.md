# KiCad 8 DRC Validation Test Suite (Cycle 16)

I don't have the earlier cycles, so this is a **self-contained** suite. If Cycle 15 used different thresholds, file names, or report formats, change `drc_policy.yaml` and the fixture paths. The test logic should not need changes.

The suite checks each rule in three layers:

1. **Native DRC** runs through `kicad-cli pcb drc`, which produces a JSON report.
2. **Rule-floor checks** confirm that nobody lowered the project's own rule values (`.kicad_pro`) or excluded violations to get DRC to pass.
3. **Independent geometric checks** use the `pcbnew` Python API. KiCad has **no native drill aspect-ratio check**, so the aspect-ratio check is done only here.

---

## Directory layout

```
drc_tests/
├── drc_policy.yaml
├── boards.txt                 # one .kicad_pcb path per line
├── policy.kicad_dru           # optional custom rules, copied next to boards in CI
├── conftest.py
├── kicad_helpers.py
├── test_annular_ring.py
├── test_edge_clearance.py
├── test_drill_aspect_ratio.py
├── test_drc_integrity.py
└── .github/workflows/drc.yml
```

---

## `drc_policy.yaml`

```yaml
# All dimensions in mm. Set these values from your fab's capability sheet.
annular_ring:
  via_min: 0.125          # IPC-6012 Class 2 ≈ 0.05 external; fab limits usually dominate
  pth_pad_min: 0.15
  microvia_min: 0.075

edge_clearance:
  copper_min: 0.30        # copper to Edge.Cuts
  vscore_min: 0.40        # used if a V-score layer/rule is present
  hole_to_edge_min: 0.50  # drilled hole wall to board edge

aspect_ratio:
  through_max: 10.0       # board_thickness / finished drill
  blind_max: 1.0          # depth / drill (controlled-depth blind)
  microvia_max: 0.8

drc:
  # DRC violation types that fail this suite outright
  blocking_types:
    - annular_width
    - copper_edge_clearance
    - hole_clearance
    - hole_to_hole
    - drill_out_of_range
    - microvia_drill_out_of_range
    - via_diameter
  # Exclusions in .kicad_pro may not hide these types
  forbid_exclusions_for:
    - annular_width
    - copper_edge_clearance
    - drill_out_of_range

tolerance_mm: 0.001       # rounding slack for nm→mm conversions
```

---

## `policy.kicad_dru` (optional, enforces the floors inside KiCad itself)

```lisp
(version 1)

(rule "policy: via annular ring"
  (constraint annular_width (min 0.125mm))
  (condition "A.Type == 'Via' && A.Via_Type != 'Micro'"))

(rule "policy: microvia annular ring"
  (constraint annular_width (min 0.075mm))
  (condition "A.Via_Type == 'Micro'"))

(rule "policy: PTH pad annular ring"
  (constraint annular_width (min 0.15mm))
  (condition "A.Type == 'Pad' && A.Pad_Type == 'Through-hole'"))

(rule "policy: copper to edge"
  (constraint edge_clearance (min 0.30mm)))

(rule "policy: hole to edge"
  (constraint edge_clearance (min 0.50mm))
  (condition "A.isPlated() || A.Type == 'Via'"))
```

A `.kicad_dru` file can't express a ratio, so it can't check drill aspect ratio. That check lives in `test_drill_aspect_ratio.py`.

---

## `kicad_helpers.py`

```python
"""Shared helpers: kicad-cli invocation, report parsing, unit conversion."""
from __future__ import annotations

import json
import shutil
import subprocess
from dataclasses import dataclass, field
from pathlib import Path

NM_PER_MM = 1_000_000


def nm_to_mm(v: int | float) -> float:
    return v / NM_PER_MM


def find_kicad_cli() -> str:
    cli = shutil.which("kicad-cli")
    if not cli:
        raise RuntimeError("kicad-cli not on PATH (KiCad 8.0+ required)")
    return cli


def kicad_version(cli: str) -> tuple[int, int, int]:
    out = subprocess.run([cli, "version"], capture_output=True, text=True, check=True)
    parts = out.stdout.strip().split(".")[:3]
    return tuple(int("".join(c for c in p if c.isdigit()) or 0) for p in parts)


@dataclass
class Violation:
    type: str
    severity: str
    description: str
    items: list[dict] = field(default_factory=list)

    def locations(self) -> str:
        locs = []
        for it in self.items:
            pos = it.get("pos", {})
            locs.append(f"{it.get('description', '?')} @ ({pos.get('x')}, {pos.get('y')})")
        return "; ".join(locs)


@dataclass
class DrcReport:
    board: Path
    raw: dict
    violations: list[Violation]

    def of_type(self, *types: str) -> list[Violation]:
        return [v for v in self.violations if v.type in types]


def run_drc(cli: str, board: Path, out_dir: Path) -> DrcReport:
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
    # Without --exit-code-violations, a nonzero exit means a tool failure, not violations.
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0 or not report_path.exists():
        raise RuntimeError(
            f"kicad-cli DRC failed on {board} (rc={proc.returncode})\n"
            f"stdout:\n{proc.stdout}\nstderr:\n{proc.stderr}"
        )
    raw = json.loads(report_path.read_text())
    violations = [
        Violation(
            type=v.get("type", ""),
            severity=v.get("severity", ""),
            description=v.get("description", ""),
            items=v.get("items", []),
        )
        for v in raw.get("violations", [])
    ]
    if raw.get("coordinate_units", "mm") != "mm":
        raise RuntimeError(f"Unexpected report units: {raw.get('coordinate_units')}")
    return DrcReport(board=board, raw=raw, violations=violations)


def load_project_settings(board: Path) -> dict:
    pro = board.with_suffix(".kicad_pro")
    if not pro.exists():
        return {}
    return json.loads(pro.read_text())


def format_failures(lines: list[str], limit: int = 25) -> str:
    head = lines[:limit]
    more = f"\n... and {len(lines) - limit} more" if len(lines) > limit else ""
    return "\n".join(head) + more
```

---

## `conftest.py`

```python
from __future__ import annotations

import os
from pathlib import Path

import pytest
import yaml

from kicad_helpers import DrcReport, find_kicad_cli, kicad_version, run_drc

HERE = Path(__file__).parent


def _board_list() -> list[Path]:
    env = os.environ.get("KICAD_BOARDS")
    if env:
        return [Path(p) for p in env.split(os.pathsep) if p]
    lst = HERE / "boards.txt"
    return [
        (HERE / line.strip()).resolve()
        for line in lst.read_text().splitlines()
        if line.strip() and not line.startswith("#")
    ]


BOARDS = _board_list()


def pytest_generate_tests(metafunc):
    if "board_path" in metafunc.fixturenames:
        metafunc.parametrize("board_path", BOARDS, ids=[b.stem for b in BOARDS], scope="session")


@pytest.fixture(scope="session")
def policy() -> dict:
    return yaml.safe_load((HERE / "drc_policy.yaml").read_text())


@pytest.fixture(scope="session")
def kicad_cli() -> str:
    cli = find_kicad_cli()
    major = kicad_version(cli)[0]
    if major < 8:
        pytest.exit(f"KiCad >= 8 required, found {major}", returncode=3)
    return cli


_drc_cache: dict[Path, DrcReport] = {}


@pytest.fixture(scope="session")
def drc_report(kicad_cli, board_path, tmp_path_factory) -> DrcReport:
    if board_path not in _drc_cache:
        out = tmp_path_factory.getbasetemp() / "drc_reports"
        _drc_cache[board_path] = run_drc(kicad_cli, board_path, out)
    return _drc_cache[board_path]


_board_cache: dict = {}


@pytest.fixture(scope="session")
def pcb(board_path):
    """Loaded pcbnew BOARD. Skips if the KiCad Python module is unavailable."""
    pcbnew = pytest.importorskip("pcbnew", reason="KiCad pcbnew Python module not available")
    if board_path not in _board_cache:
        _board_cache[board_path] = pcbnew.LoadBoard(str(board_path))
    return _board_cache[board_path]


@pytest.fixture(scope="session")
def pcbnew_mod():
    return pytest.importorskip("pcbnew")
```

---

## `test_annular_ring.py`

```python
from __future__ import annotations

import pytest

from kicad_helpers import format_failures, load_project_settings, nm_to_mm


class TestAnnularRingDRC:
    def test_no_native_annular_violations(self, drc_report):
        hits = drc_report.of_type("annular_width")
        assert not hits, (
            f"{len(hits)} annular_width violation(s) in {drc_report.board.name}:\n"
            + format_failures([f"[{v.severity}] {v.description} :: {v.locations()}" for v in hits])
        )

    def test_project_rule_not_below_policy(self, board_path, policy):
        rules = (load_project_settings(board_path)
                 .get("board", {}).get("design_settings", {}).get("rules", {}))
        if "min_via_annular_width" not in rules:
            pytest.skip("No min_via_annular_width in .kicad_pro")
        floor = min(policy["annular_ring"]["via_min"], policy["annular_ring"]["microvia_min"])
        assert rules["min_via_annular_width"] + policy["tolerance_mm"] >= floor, (
            f"Project min_via_annular_width={rules['min_via_annular_width']} mm is below "
            f"policy floor {floor} mm. DRC passing is not meaningful."
        )


class TestAnnularRingGeometry:
    """Independent recomputation using the pcbnew API (KiCad 8 single-width vias)."""

    def test_via_annular_ring(self, pcb, pcbnew_mod, policy):
        tol = policy["tolerance_mm"]
        bad = []
        for t in pcb.GetTracks():
            if not isinstance(t, pcbnew_mod.PCB_VIA):
                continue
            is_micro = t.GetViaType() == pcbnew_mod.VIATYPE_MICROVIA
            limit = policy["annular_ring"]["microvia_min" if is_micro else "via_min"]
            ring = nm_to_mm(t.GetWidth() - t.GetDrillValue()) / 2
            if ring + tol < limit:
                p = t.GetPosition()
                bad.append(f"via @ ({nm_to_mm(p.x):.3f}, {nm_to_mm(p.y):.3f}) "
                           f"ring={ring:.3f} < {limit}")
        assert not bad, format_failures(bad)

    def test_pth_pad_annular_ring(self, pcb, pcbnew_mod, policy):
        tol = policy["tolerance_mm"]
        limit = policy["annular_ring"]["pth_pad_min"]
        bad = []
        for fp in pcb.GetFootprints():
            for pad in fp.Pads():
                if pad.GetAttribute() != pcbnew_mod.PAD_ATTRIB_PTH:
                    continue
                size, drill, off = pad.GetSize(), pad.GetDrillSize(), pad.GetOffset()
                if drill.x == 0:
                    continue
                # Worst axis, including the drill offset. This is conservative for
                # rotated/odd shapes, so custom-shape pads may need manual review.
                ring_x = (size.x - drill.x) / 2 - abs(off.x)
                ring_y = (size.y - drill.y) / 2 - abs(off.y)
                