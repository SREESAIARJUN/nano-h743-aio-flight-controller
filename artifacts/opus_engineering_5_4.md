# KiCad 8 DRC Validation Test Suite: Annular Rings, Copper-to-Edge Clearance, Drill Aspect Ratio

> **Note on "Cycle 5":** I don't have the earlier cycles of this work, so I can't carry over your previous thresholds, file names, or decisions. The suite below is self-contained. Fab limits live in one config file so you can drop in your Cycle 1–4 values. If you share the previous cycle's output, I can align naming and diff against it.

---

## 1. Layout

```
drc-tests/
├── fab_limits.yaml          # Single source of truth for thresholds
├── rules/
│   └── fab.kicad_dru        # Custom DRC rules, copied next to the .kicad_pcb
├── conftest.py              # Fixtures: runs kicad-cli, loads the board via pcbnew
├── test_drc_report.py       # Asserts on kicad-cli DRC JSON output
├── test_geometry.py         # Independent geometry checks via pcbnew API
├── test_settings.py         # Verifies board settings match fab limits
└── .github/workflows/drc.yml
```

The suite uses two independent layers on purpose:

1. **kicad-cli DRC** is the authoritative engine. It is used for edge clearance and for the built-in annular-ring check.
2. **pcbnew Python API** recomputes the geometry directly. This catches cases where someone relaxed the board rules to silence DRC. It is also the only option for aspect ratio, because KiCad 8 has no native aspect-ratio constraint.

---

## 2. `fab_limits.yaml`

```yaml
# All dimensions in mm. Replace with your fab's capability sheet / Cycle 4 values.
fab: "GenericFab-Standard"
board_thickness_mm: 1.6
board_thickness_tolerance_mm: 0.16

annular_ring:
  via_min_mm: 0.125
  pth_pad_min_mm: 0.15

edge_clearance:
  copper_to_edge_min_mm: 0.30

drill:
  min_through_drill_mm: 0.20
  max_aspect_ratio: 10.0        # thickness : finished drill diameter
  max_aspect_ratio_pth: 8.0     # stricter limit for component holes

report:
  fail_on_severity: ["error"]   # add "warning" for stricter gating
```

---

## 3. `rules/fab.kicad_dru`

```lisp
(version 1)

(rule "Via annular ring"
  (constraint annular_width (min 0.125mm))
  (condition "A.Type == 'Via'"))

(rule "PTH pad annular ring"
  (constraint annular_width (min 0.15mm))
  (condition "A.Type == 'Pad' && A.Pad_Type == 'Through-hole'"))

(rule "Copper to board edge"
  (constraint edge_clearance (min 0.30mm)))

(rule "Min through drill"
  (constraint hole_size (min 0.20mm))
  (condition "A.isPlated()"))

# Aspect ratio cannot be expressed natively in KiCad 8. The bound below is a
# proxy: 1.6mm / 10 = 0.16mm. The true ratio is checked in test_geometry.py.
(rule "Aspect ratio proxy (via)"
  (constraint hole_size (min 0.16mm))
  (condition "A.Type == 'Via'"))
```

---

## 4. `conftest.py`

```python
import json
import os
import shutil
import subprocess
from pathlib import Path

import pytest
import yaml

HERE = Path(__file__).parent
BOARD = Path(os.environ.get("KICAD_BOARD", "hardware/main.kicad_pcb")).resolve()
NM_PER_MM = 1_000_000


def mm(nm: int) -> float:
    return nm / NM_PER_MM


@pytest.fixture(scope="session")
def limits():
    with open(HERE / "fab_limits.yaml") as f:
        return yaml.safe_load(f)


@pytest.fixture(scope="session")
def board_path():
    assert BOARD.exists(), f"Board not found: {BOARD}"
    return BOARD


@pytest.fixture(scope="session")
def drc_report(board_path, tmp_path_factory):
    """Run kicad-cli DRC once per session and return the parsed JSON report."""
    # KiCad only picks up a .kicad_dru that sits next to the project file.
    dru_target = board_path.with_suffix(".kicad_dru")
    if not dru_target.exists():
        shutil.copy(HERE / "rules" / "fab.kicad_dru", dru_target)

    out = tmp_path_factory.mktemp("drc") / "drc.json"
    cmd = [
        "kicad-cli", "pcb", "drc",
        "--format", "json",
        "--severity-all",
        "--units", "mm",
        "--all-track-errors",
        "--output", str(out),
        str(board_path),
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    # Exit code is nonzero only on CLI failure (no --exit-code-violations).
    assert proc.returncode == 0, f"kicad-cli failed:\n{proc.stdout}\n{proc.stderr}"
    with open(out) as f:
        return json.load(f)


@pytest.fixture(scope="session")
def board(board_path):
    pcbnew = pytest.importorskip("pcbnew", reason="Requires KiCad's Python (pcbnew)")
    return pcbnew.LoadBoard(str(board_path))


@pytest.fixture(scope="session")
def pcbnew_mod():
    return pytest.importorskip("pcbnew")
```

---

## 5. `test_drc_report.py`: assertions on kicad-cli output

```python
import pytest

# KiCad 8 violation type keys
ANNULAR = {"annular_width"}
EDGE = {"copper_edge_clearance"}
DRILL = {"drill_out_of_range", "hole_size", "via_diameter"}


def _violations(report, types, severities):
    found = []
    for v in report.get("violations", []):
        if v.get("type") in types and v.get("severity") in severities:
            found.append(v)
    return found


def _fmt(vs):
    lines = []
    for v in vs[:25]:
        items = "; ".join(
            f"{i.get('description', '')} @ ({i.get('pos', {}).get('x')}, {i.get('pos', {}).get('y')})"
            for i in v.get("items", [])
        )
        lines.append(f"  [{v['severity']}] {v['type']}: {v['description']} -> {items}")
    if len(vs) > 25:
        lines.append(f"  ... and {len(vs) - 25} more")
    return "\n".join(lines)


def test_report_is_kicad8(drc_report):
    ver = drc_report.get("kicad_version", "")
    assert ver.startswith("8."), f"Expected KiCad 8.x, got '{ver}'"


def test_no_annular_ring_violations(drc_report, limits):
    vs = _violations(drc_report, ANNULAR, limits["report"]["fail_on_severity"])
    assert not vs, f"{len(vs)} annular-ring violation(s):\n{_fmt(vs)}"


def test_no_copper_edge_violations(drc_report, limits):
    vs = _violations(drc_report, EDGE, limits["report"]["fail_on_severity"])
    assert not vs, f"{len(vs)} copper-to-edge violation(s):\n{_fmt(vs)}"


def test_no_drill_violations(drc_report, limits):
    vs = _violations(drc_report, DRILL, limits["report"]["fail_on_severity"])
    assert not vs, f"{len(vs)} drill-size violation(s):\n{_fmt(vs)}"


def test_board_outline_is_closed(drc_report):
    # An open Edge.Cuts outline silently disables edge-clearance checking.
    vs = [v for v in drc_report.get("violations", [])
          if v.get("type") in {"invalid_outline", "malformed_courtyard"}
          and "outline" in v.get("description", "").lower()]
    assert not vs, f"Board outline problems invalidate edge checks:\n{_fmt(vs)}"


@pytest.mark.parametrize("key", ["violations", "unconnected_items"])
def test_report_structure(drc_report, key):
    assert key in drc_report, f"DRC JSON missing '{key}', report format changed?"
```

---

## 6. `test_geometry.py`: independent recomputation via pcbnew

```python
import pytest
from conftest import mm


def _vias(board, pcbnew):
    return [t for t in board.GetTracks() if isinstance(t, pcbnew.PCB_VIA)]


def _pth_pads(board, pcbnew):
    return [p for p in board.GetPads()
            if p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH]


def _thickness_mm(board):
    return mm(board.GetDesignSettings().GetBoardThickness())


def _loc(item):
    p = item.GetPosition()
    return f"({mm(p.x):.3f}, {mm(p.y):.3f})"


# ---------- Annular rings ----------

def test_via_annular_ring(board, pcbnew_mod, limits):
    min_ar = limits["annular_ring"]["via_min_mm"]
    bad = []
    for v in _vias(board, pcbnew_mod):
        ar = (mm(v.GetWidth()) - mm(v.GetDrillValue())) / 2
        if ar + 1e-6 < min_ar:
            bad.append(f"via {_loc(v)} net={v.GetNetname()} AR={ar:.3f}mm")
    assert not bad, f"{len(bad)} via(s) below {min_ar}mm AR:\n" + "\n".join(bad[:25])


def test_pth_pad_annular_ring(board, pcbnew_mod, limits):
    min_ar = limits["annular_ring"]["pth_pad_min_mm"]
    bad = []
    for p in _pth_pads(board, pcbnew_mod):
        size, drill = p.GetSize(), p.GetDrillSize()
        # Use the worst axis. This handles oval pads and slots.
        ar = min(mm(size.x) - mm(drill.x), mm(size.y) - mm(drill.y)) / 2
        if ar + 1e-6 < min_ar:
            ref = p.GetParentFootprint().GetReference()
            bad.append(f"{ref}.{p.GetNumber()} {_loc(p)} AR={ar:.3f}mm")
    assert not bad, f"{len(bad)} PTH pad(s) below {min_ar}mm AR:\n" + "\n".join(bad[:25])


# ---------- Drill size and aspect ratio ----------

def test_min_drill(board, pcbnew_mod, limits):
    dmin = limits["drill"]["min_through_drill_mm"]
    bad = [f"via {_loc(v)} drill={mm(v.GetDrillValue()):.3f}"
           for v in _vias(board, pcbnew_mod)
           if mm(v.GetDrillValue()) + 1e-6 < dmin]
    assert not bad, "Drills below minimum:\n" + "\n".join(bad[:25])


def test_via_aspect_ratio(board, pcbnew_mod, limits):
    t = _thickness_mm(board)
    max_ar = limits["drill"]["max_aspect_ratio"]
    bad = []
    for v in _vias(board, pcbnew_mod):
        # Blind/buried vias span fewer layers. Through-via ratio is the
        # conservative bound; refine per layer pair if your fab requires it.
        d = mm(v.GetDrillValue())
        ratio = t / d if d else float("inf")
        if ratio > max_ar + 1e-6:
            bad.append(f"via {_loc(v)} drill={d:.3f} ratio={ratio:.1f}:1")
    assert not bad, f"Vias exceed {max_ar}:1 (t={t}mm):\n" + "\n".join(bad[:25])


def test_pth_aspect_ratio(board, pcbnew_mod, limits):
    t = _thickness_mm(board)
    max_ar = limits["drill"]["max_aspect_ratio_pth"]
    bad = []
    for p in _pth_pads(board, pcbnew_mod):
        d = min(mm(p.GetDrillSize().x), mm(p.GetDrillSize().y))
        ratio = t / d if d else float("inf")
        if ratio > max_ar + 1e-6:
            ref = p.GetParentFootprint().GetReference()
            bad.append(f"{ref}.{p.GetNumber()} drill={d:.3f} ratio={ratio:.1f}:1")
    assert not bad, f"PTH exceed {max_ar}:1:\n" + "\n".join(bad[:25])


def test_board_thickness_matches_stackup(board, limits):
    t = _thickness_mm(board)
    exp = limits["board_thickness_mm"]
    tol = limits["board_thickness_tolerance_mm"]
    assert abs(t - exp) <= tol, (
        f"Board thickness {t}mm != {exp}±{tol}mm, so aspect-ratio results are invalid")
```

---

## 7. `test_settings.py`: guard against relaxed rules

```python
from conftest import mm


def test_rule_floors_not_relaxed(board, limits):
    """Fail if someone lowered Board Setup constraints below fab limits."""
    bds = board.GetDesignSettings()
    checks = {
        "Copper-to-edge clearance": (mm(bds.m_CopperEdgeClearance),
                                     limits["edge_clearance"]["copper_to_edge_min_mm"]),
        "Min via annular width": (mm(bds.m_ViasMinAnnularWidth),
                                  limits["annular_ring"]["via_min_mm"]),
        "Min through-hole drill": (mm(bds.m_MinThroughDrill),
                                   limits["drill"]["min_through_drill_mm"]),
    }
    bad = [f"{k}: board={v:.3f}mm < fab={f:.3f}mm"
           for k, (v, f) in checks.items() if v + 1e-6 < f]
    assert not bad, "Board Setup is looser than fab limits:\n" + "\n".join(bad)


def test_custom_rules_present(board_path):
    dru = board_path.with_suffix(".kicad_dru")
    assert dru.exists(), "Custom .kicad_dru missing"
    text = dru.read_text()
    for needle in ("annular_width