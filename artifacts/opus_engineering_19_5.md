# KiCad 8 DRC Validation Test Suite: Annular Ring, Edge Clearance and Aspect Ratio

**Note on "Cycle 19":** I don't have your project's Cycle 19 history, thresholds, or previous suite. The suite below is complete and runnable. All limits live in one config file, so you can replace the placeholders with your fab house's Cycle 19 values.

The suite uses two layers of checking:

1. **Native KiCad DRC** (`kicad-cli pcb drc`, JSON output). This catches `annular_width` and `copper_edge_clearance` violations.
2. **Independent geometric checks** (the `pcbnew` Python API). These re-verify annular rings and compute drill aspect ratios. KiCad 8 has no native aspect-ratio constraint. These checks also confirm that the board's design rules aren't looser than your fab limits. Otherwise a permissive board setup could pass DRC trivially.

---

## Directory layout

```
drc_suite/
├── drc_limits.yaml
├── conftest.py
├── kicad_drc.py
├── test_annular_ring.py
├── test_edge_clearance.py
├── test_aspect_ratio.py
├── fab_rules.kicad_dru        # optional custom rules to copy next to the board
└── .github/workflows/drc.yml
```

---

## `drc_limits.yaml`

```yaml
# Fab limits — replace with Cycle 19 values
units: mm
boards_glob: "../hardware/**/*.kicad_pcb"

annular_ring:
  via_min: 0.125          # (via diameter - drill) / 2
  pth_pad_min: 0.150      # plated through-hole pads
  ignore_npth: true

edge_clearance:
  copper_to_edge_min: 0.300
  allow_exclusions: false # if false, DRC exclusions don't hide violations

aspect_ratio:
  max_through: 10.0       # board_thickness / finished_drill
  max_via: 8.0
  min_drill: 0.200        # sanity floor

drc:
  severity: all           # error | warning | all
  fail_on_warning_types: [annular_width, copper_edge_clearance]
```

---

## `kicad_drc.py`: CLI runner and JSON parser

```python
"""Wrapper around `kicad-cli pcb drc` (KiCad 8.x)."""
from __future__ import annotations
import json, shutil, subprocess, tempfile
from dataclasses import dataclass, field
from pathlib import Path

KICAD_CLI = shutil.which("kicad-cli") or "kicad-cli"


@dataclass
class Violation:
    type: str
    severity: str
    description: str
    items: list = field(default_factory=list)
    excluded: bool = False

    def where(self) -> str:
        locs = []
        for it in self.items:
            p = it.get("pos", {})
            locs.append(f"{it.get('description','?')} @ ({p.get('x')},{p.get('y')})")
        return "; ".join(locs)


@dataclass
class DrcReport:
    board: Path
    violations: list[Violation]
    raw: dict

    def of_type(self, *types: str, include_excluded=False) -> list[Violation]:
        return [v for v in self.violations
                if v.type in types and (include_excluded or not v.excluded)]


def run_drc(board: Path, severity: str = "all") -> DrcReport:
    board = Path(board).resolve()
    with tempfile.TemporaryDirectory() as td:
        out = Path(td) / "drc.json"
        cmd = [KICAD_CLI, "pcb", "drc",
               "--format", "json", "--units", "mm",
               f"--severity-{severity}",
               "-o", str(out), str(board)]
        proc = subprocess.run(cmd, capture_output=True, text=True)
        if not out.exists():
            raise RuntimeError(
                f"kicad-cli DRC failed ({proc.returncode}) for {board}\n"
                f"STDOUT:\n{proc.stdout}\nSTDERR:\n{proc.stderr}")
        raw = json.loads(out.read_text())

    viols = [Violation(type=v.get("type", ""),
                       severity=v.get("severity", ""),
                       description=v.get("description", ""),
                       items=v.get("items", []),
                       excluded=bool(v.get("excluded", False)))
             for v in raw.get("violations", [])]
    return DrcReport(board=board, violations=viols, raw=raw)
```

---

## `conftest.py`

```python
import glob
from pathlib import Path
import pytest, yaml

from kicad_drc import run_drc

HERE = Path(__file__).parent
LIMITS = yaml.safe_load((HERE / "drc_limits.yaml").read_text())


def _boards():
    pattern = str((HERE / LIMITS["boards_glob"]).resolve())
    found = sorted(Path(p) for p in glob.glob(pattern, recursive=True))
    # skip KiCad autosave/backup files
    return [b for b in found if not b.name.startswith(("_autosave", "~"))]


BOARDS = _boards()


def pytest_generate_tests(metafunc):
    if "board_path" in metafunc.fixturenames:
        if not BOARDS:
            pytest.fail(f"No boards matched {LIMITS['boards_glob']}")
        metafunc.parametrize("board_path", BOARDS, ids=[b.stem for b in BOARDS])


@pytest.fixture(scope="session")
def limits():
    return LIMITS


_drc_cache: dict = {}

@pytest.fixture
def drc_report(board_path, limits):
    if board_path not in _drc_cache:
        _drc_cache[board_path] = run_drc(board_path, limits["drc"]["severity"])
    return _drc_cache[board_path]


_board_cache: dict = {}

@pytest.fixture
def pcb(board_path):
    pcbnew = pytest.importorskip(
        "pcbnew", reason="Run with KiCad's bundled Python for geometric checks")
    if board_path not in _board_cache:
        _board_cache[board_path] = pcbnew.LoadBoard(str(board_path))
    return _board_cache[board_path]


@pytest.fixture
def mm():
    import pcbnew
    return pcbnew.ToMM
```

---

## `test_annular_ring.py`

```python
import pytest

EPS = 1e-4  # mm, avoids float rounding false-fails


def test_drc_reports_no_annular_violations(drc_report, limits):
    """Native KiCad DRC: 'annular_width'."""
    incl = not limits["edge_clearance"]["allow_exclusions"]
    viols = drc_report.of_type("annular_width", include_excluded=incl)
    assert not viols, "Annular ring DRC violations:\n" + "\n".join(
        f"  [{v.severity}] {v.description} :: {v.where()}" for v in viols)


def test_board_rule_not_looser_than_fab(pcb, mm, limits):
    """The board's own min annular width must be >= the fab minimum."""
    ds = pcb.GetDesignSettings()
    board_min = mm(ds.m_MinRingWidth)
    assert board_min + EPS >= limits["annular_ring"]["via_min"], (
        f"Board Setup min annular width {board_min:.3f} mm is below fab "
        f"limit {limits['annular_ring']['via_min']} mm; DRC would under-report.")


def test_via_annular_ring_geometry(pcb, mm, limits):
    import pcbnew
    lim = limits["annular_ring"]["via_min"]
    bad = []
    for t in pcb.GetTracks():
        if not isinstance(t, pcbnew.PCB_VIA):
            continue
        ring = (mm(t.GetWidth()) - mm(t.GetDrillValue())) / 2
        if ring + EPS < lim:
            p = t.GetPosition()
            bad.append(f"via @ ({mm(p.x):.3f},{mm(p.y):.3f}) "
                       f"Ø{mm(t.GetWidth()):.3f}/drill {mm(t.GetDrillValue()):.3f} "
                       f"ring {ring:.3f} < {lim}")
    assert not bad, "Via annular ring too small:\n  " + "\n  ".join(bad)


def test_pth_pad_annular_ring_geometry(pcb, mm, limits):
    import pcbnew
    lim = limits["annular_ring"]["pth_pad_min"]
    bad = []
    for pad in pcb.GetPads():
        attr = pad.GetAttribute()
        if attr == pcbnew.PAD_ATTRIB_NPTH and limits["annular_ring"]["ignore_npth"]:
            continue
        if attr != pcbnew.PAD_ATTRIB_PTH:
            continue
        size, drill = pad.GetSize(), pad.GetDrillSize()
        if drill.x == 0 or drill.y == 0:
            continue
        # worst axis (handles oval pads and slots)
        ring = min(mm(size.x) - mm(drill.x), mm(size.y) - mm(drill.y)) / 2
        if ring + EPS < lim:
            fp = pad.GetParentFootprint()
            ref = fp.GetReference() if fp else "?"
            bad.append(f"{ref}.{pad.GetNumber()} ring {ring:.3f} < {lim}")
    assert not bad, "PTH pad annular ring too small:\n  " + "\n  ".join(bad)
```

---

## `test_edge_clearance.py`

```python
EPS = 1e-4


def test_drc_reports_no_edge_clearance_violations(drc_report, limits):
    """Native KiCad DRC: 'copper_edge_clearance'."""
    incl = not limits["edge_clearance"]["allow_exclusions"]
    viols = drc_report.of_type("copper_edge_clearance", include_excluded=incl)
    assert not viols, "Copper-to-edge violations:\n" + "\n".join(
        f"  [{v.severity}] {v.description} :: {v.where()}" for v in viols)


def test_board_edge_rule_not_looser_than_fab(pcb, mm, limits):
    ds = pcb.GetDesignSettings()
    board_val = mm(ds.m_CopperEdgeClearance)
    need = limits["edge_clearance"]["copper_to_edge_min"]
    assert board_val + EPS >= need, (
        f"Board Setup copper-to-edge clearance {board_val:.3f} mm < fab {need} mm")


def test_board_outline_is_closed(drc_report):
    """An open Edge.Cuts outline makes edge-clearance results meaningless."""
    viols = drc_report.of_type("invalid_outline", include_excluded=True)
    assert not viols, "Board outline invalid:\n" + "\n".join(
        f"  {v.description} :: {v.where()}" for v in viols)


def test_zones_respect_edge(drc_report):
    """Zone fills are rebuilt only if --refill-zones is used; flag stale fills."""
    viols = drc_report.of_type("zones_intersect", "copper_edge_clearance")
    zone_viols = [v for v in viols if "zone" in v.where().lower()]
    assert not zone_viols, "Zone copper too close to edge:\n" + "\n".join(
        v.where() for v in zone_viols)
```

---

## `test_aspect_ratio.py`

KiCad 8 has no built-in aspect-ratio DRC, so this check is computed directly.

```python
EPS = 1e-6


def _thickness_mm(pcb, mm):
    return mm(pcb.GetDesignSettings().GetBoardThickness())


def test_board_thickness_sane(pcb, mm):
    t = _thickness_mm(pcb, mm)
    assert 0.2 <= t <= 6.0, f"Implausible board thickness {t} mm (check Stackup)"


def test_via_aspect_ratio(pcb, mm, limits):
    import pcbnew
    t = _thickness_mm(pcb, mm)
    max_ar = limits["aspect_ratio"]["max_via"]
    bad = []
    for v in pcb.GetTracks():
        if not isinstance(v, pcbnew.PCB_VIA):
            continue
        drill = mm(v.GetDrillValue())
        # blind/buried: use the actual span, not full thickness
        span = t
        if v.GetViaType() != pcbnew.VIATYPE_THROUGH:
            top, bot = v.TopLayer(), v.BottomLayer()
            stack = pcb.GetDesignSettings().GetStackupDescriptor()
            span = mm(stack.GetLayerDistance(top, bot)) or t
        ar = span / drill if drill else float("inf")
        if ar > max_ar + EPS:
            p = v.GetPosition()
            bad.append(f"via @ ({mm(p.x):.3f},{mm(p.y):.3f}) drill {drill:.3f} "
                       f"span {span:.3f} AR {ar:.2f} > {max_ar}")
    assert not bad, "Via aspect ratio exceeded:\n  " + "\n  ".join(bad)


def test_pth_aspect_ratio(pcb, mm, limits):
    import pcbnew
    t = _thickness_mm(pcb, mm)
    max_ar = limits["aspect_ratio"]["max_through"]
    bad = []
    for pad in pcb.GetPads():
        if pad.GetAttribute() != pcbnew.PAD_ATTRIB_PTH:
            continue
        d = pad.GetDrillSize()
        drill = mm(min(d.x, d.y))  # slots: narrow dimension governs plating
        if drill <= 0:
            continue
        ar = t / drill
        if ar > max_ar + EPS:
            fp = pad.GetParentFootprint()
            bad.append(f"{fp.GetReference() if fp else '?'}.{pad.GetNumber()} "
                       f"drill {drill:.3f} AR {ar:.2f} > {max_ar}")
    assert not bad, "PTH aspect ratio exceeded:\n  " + "\n  