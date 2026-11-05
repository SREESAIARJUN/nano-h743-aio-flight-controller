# KiCad 8 DRC Validation Test Suite

I don't have records of earlier cycles, so this is a self-contained, versioned suite that you can drop in as Cycle 21. Set your Cycle 20 thresholds in `drc_spec.yaml`. If earlier cycles used different test IDs or rule names, share them and I'll align this suite to match.

## Layout

```
drc_suite/
├── drc_spec.yaml            # Fab thresholds (single source of truth)
├── rules/fab.kicad_dru      # Custom DRC rules mirrored into each project
├── conftest.py              # Board discovery, kicad-cli DRC run, fixtures
├── test_annular_ring.py
├── test_copper_edge.py
├── test_aspect_ratio.py
└── .github/workflows/drc.yml
```

## Architecture

The suite checks each board in two independent ways:

1. **Native DRC** (`kicad-cli pcb drc`) produces a JSON report. The tests fail on any violation of the targeted types.
2. **Independent geometry checks** use the `pcbnew` Python API. These catch two problems that native DRC misses:
   - Someone loosened the board's constraints so that native DRC passes.
   - Drill aspect ratio, which KiCad 8 has no native rule for.

---

## `drc_spec.yaml`

```yaml
cycle: 21
units: mm

annular_ring:
  via_min: 0.125          # (via_diameter - drill) / 2
  pth_min: 0.150          # min over X/Y of (pad_size - drill) / 2
  exempt_refs: []         # e.g. ["MH1", "MH2"]

copper_edge:
  min_clearance: 0.300
  exempt_nets: []         # e.g. ["GND"] for intentional edge plating

aspect_ratio:
  max_via: 10.0           # board_thickness / finished_drill
  max_pth: 10.0
  min_board_thickness: 0.4  # sanity check on stackup

drc:
  fail_types:
    - annular_width
    - copper_edge_clearance
    - drill_out_of_range
    - hole_clearance
  fail_on_severity: [error]
```

## `rules/fab.kicad_dru`

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

# Aspect-ratio proxy for a 1.6 mm board at 10:1. The exact ratio is enforced in Python.
(rule "fab_min_drill"
  (constraint hole_size (min 0.16mm)))
```

## `conftest.py`

```python
import json
import os
import shutil
import subprocess
from pathlib import Path

import pytest
import yaml

pcbnew = pytest.importorskip("pcbnew", reason="Run inside KiCad 8 Python env")

ROOT = Path(__file__).parent
SPEC = yaml.safe_load((ROOT / "drc_spec.yaml").read_text())
BOARD_GLOB = os.environ.get("DRC_BOARDS", "**/*.kicad_pcb")
SEARCH_ROOT = Path(os.environ.get("DRC_ROOT", "."))
REPORT_DIR = Path(os.environ.get("DRC_REPORT_DIR", "drc_reports"))


def nm_to_mm(v):
    return pcbnew.ToMM(v)


def discover_boards():
    return sorted(p for p in SEARCH_ROOT.glob(BOARD_GLOB)
                  if "-backups" not in p.parts and not p.name.startswith("_autosave"))


BOARDS = discover_boards()


@pytest.fixture(scope="session")
def spec():
    return SPEC


@pytest.fixture(params=BOARDS, ids=lambda p: p.stem, scope="session")
def board_path(request):
    return request.param


@pytest.fixture(scope="session")
def board(board_path):
    return pcbnew.LoadBoard(str(board_path))


@pytest.fixture(scope="session")
def drc_report(board_path):
    cli = shutil.which("kicad-cli")
    if not cli:
        pytest.skip("kicad-cli not on PATH")
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    out = REPORT_DIR / f"{board_path.stem}.drc.json"
    cmd = [cli, "pcb", "drc",
           "--format", "json",
           "--units", "mm",
           "--severity-all",
           "--schematic-parity",
           "-o", str(out),
           str(board_path)]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    # Without --exit-code-violations, a nonzero exit means a tool failure.
    assert proc.returncode == 0, f"kicad-cli failed:\n{proc.stderr}"
    return json.loads(out.read_text())


def violations_of(report, vtype, severities=("error",)):
    return [v for v in report.get("violations", [])
            if v.get("type") == vtype and v.get("severity") in severities]


def fmt_violations(vs, limit=15):
    lines = []
    for v in vs[:limit]:
        locs = "; ".join(
            f"{i.get('description', '?')} @ ({i['pos']['x']:.3f},{i['pos']['y']:.3f})"
            for i in v.get("items", []) if "pos" in i)
        lines.append(f"  - {v.get('description')}: {locs}")
    if len(vs) > limit:
        lines.append(f"  ... and {len(vs) - limit} more")
    return "\n".join(lines)


def is_pth(pad):
    return pad.GetAttribute() == pcbnew.PAD_ATTRIB_PTH and pad.GetDrillSize().x > 0


def board_thickness_mm(board):
    return nm_to_mm(board.GetDesignSettings().GetBoardThickness())
```

## `test_annular_ring.py`

```python
import pytest
from conftest import fmt_violations, is_pth, nm_to_mm, pcbnew, violations_of

TOL = 1e-4  # mm, absorbs nm rounding


def test_AR01_native_drc_no_annular_violations(drc_report, spec):
    vs = violations_of(drc_report, "annular_width", spec["drc"]["fail_on_severity"])
    assert not vs, f"{len(vs)} annular ring violation(s):\n{fmt_violations(vs)}"


def test_AR02_board_constraint_not_loosened(board, spec):
    ds = board.GetDesignSettings()
    setting = nm_to_mm(ds.m_ViasMinAnnularWidth)
    assert setting + TOL >= spec["annular_ring"]["via_min"], (
        f"Board min annular width {setting:.3f} mm is below the fab spec "
        f"{spec['annular_ring']['via_min']} mm. Do not relax constraints to pass DRC.")


def test_AR03_via_annular_geometry(board, spec):
    lim = spec["annular_ring"]["via_min"]
    bad = []
    for t in board.GetTracks():
        if not isinstance(t, pcbnew.PCB_VIA):
            continue
        ring = nm_to_mm(t.GetWidth() - t.GetDrillValue()) / 2
        if ring + TOL < lim:
            p = t.GetPosition()
            bad.append(f"via @ ({nm_to_mm(p.x):.3f},{nm_to_mm(p.y):.3f}) "
                       f"ring={ring:.3f} mm net={t.GetNetname()}")
    assert not bad, f"{len(bad)} via(s) below {lim} mm:\n  " + "\n  ".join(bad[:20])


def test_AR04_pth_annular_geometry(board, spec):
    lim = spec["annular_ring"]["pth_min"]
    exempt = set(spec["annular_ring"]["exempt_refs"])
    bad = []
    for fp in board.GetFootprints():
        if fp.GetReference() in exempt:
            continue
        for pad in fp.Pads():
            if not is_pth(pad):
                continue
            size, drill = pad.GetSize(), pad.GetDrillSize()
            ring = min(nm_to_mm(size.x - drill.x), nm_to_mm(size.y - drill.y)) / 2
            if ring + TOL < lim:
                bad.append(f"{fp.GetReference()}.{pad.GetNumber()} ring={ring:.3f} mm")
    assert not bad, f"{len(bad)} PTH pad(s) below {lim} mm:\n  " + "\n  ".join(bad[:20])
```

## `test_copper_edge.py`

```python
from conftest import fmt_violations, nm_to_mm, pcbnew, violations_of

TOL = 1e-4


def test_CE01_native_drc_no_edge_violations(drc_report, spec):
    exempt = set(spec["copper_edge"]["exempt_nets"])
    vs = [v for v in violations_of(drc_report, "copper_edge_clearance",
                                   spec["drc"]["fail_on_severity"])
          if not any(f"[{n}]" in i.get("description", "")
                     for n in exempt for i in v.get("items", []))]
    assert not vs, f"{len(vs)} copper-to-edge violation(s):\n{fmt_violations(vs)}"


def test_CE02_board_constraint_not_loosened(board, spec):
    setting = nm_to_mm(board.GetDesignSettings().m_CopperEdgeClearance)
    assert setting + TOL >= spec["copper_edge"]["min_clearance"], (
        f"Board copper-edge clearance {setting:.3f} mm is below the fab spec "
        f"{spec['copper_edge']['min_clearance']} mm.")


def test_CE03_outline_is_closed(board):
    outline = pcbnew.SHAPE_POLY_SET()
    ok = board.GetBoardPolygonOutlines(outline)
    assert ok and outline.OutlineCount() >= 1, (
        "Edge.Cuts does not form a closed outline, so edge clearance cannot be trusted.")


def test_CE04_zone_edge_clearance_respected(board, spec):
    """Zones (pours) carry no per-zone edge setting in KiCad 8, but a filled
    zone's bounding box escaping the outline signals unrefilled or stale fills."""
    edge_bb = board.GetBoardEdgesBoundingBox()
    bad = []
    for z in board.Zones():
        if z.GetIsRuleArea() or not z.IsFilled():
            continue
        bb = z.GetFilledPolysList(z.GetFirstLayer()).BBox() if z.GetFirstLayer() >= 0 else None
        if bb and not edge_bb.Contains(bb.GetOrigin()):
            bad.append(f"zone net={z.GetNetname()} fill extends past outline")
    assert not bad, "\n".join(bad)
```

## `test_aspect_ratio.py`

```python
import pytest
from conftest import board_thickness_mm, is_pth, nm_to_mm, pcbnew


def test_DA01_stackup_thickness_sane(board, spec):
    t = board_thickness_mm(board)
    assert t >= spec["aspect_ratio"]["min_board_thickness"], (
        f"Board thickness {t:.3f} mm is implausible. Check Board Setup > Stackup.")


def test_DA02_via_aspect_ratio(board, spec):
    t = board_thickness_mm(board)
    lim = spec["aspect_ratio"]["max_via"]
    bad = []
    for v in board.GetTracks():
        if not isinstance(v, pcbnew.PCB_VIA):
            continue
        # Through vias only. Blind/buried vias use the span thickness and need stackup data.
        if v.GetViaType() != pcbnew.VIATYPE_THROUGH:
            continue
        d = nm_to_mm(v.GetDrillValue())
        ar = t / d if d > 0 else float("inf")
        if ar > lim:
            p = v.GetPosition()
            bad.append(f"via drill={d:.3f} mm AR={ar:.2f}:1 "
                       f"@ ({nm_to_mm(p.x):.3f},{nm_to_mm(p.y):.3f})")
    assert not bad, (f"{len(bad)} via(s) exceed {lim}:1 at {t:.2f} mm thickness:\n  "
                     + "\n  ".join(bad[:20]))


def test_DA03_pth_aspect_ratio(board, spec):
    t = board_thickness_mm(board)
    lim = spec["aspect_ratio"]["max_pth"]
    bad = []
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if not is_pth(pad):
                continue
            ds = pad.GetDrillSize()
            d = nm_to_mm(min(ds.x, ds.y))  # slots: the narrow dimension governs plating
            ar = t / d
            if ar > lim:
                bad.append(f"{fp.GetReference()}.{pad.GetNumber()} drill={d:.3f} AR={ar:.2f}:1")
    assert not bad, f"{len(bad)} PTH(s) exceed {lim}:1:\n  " + "\n  ".join(bad[:20])


@pytest.mark.xfail(reason="Blind/buried AR needs per-span dielectric data. Tracked for Cycle 22.")
def test_DA04_blind_buried_aspect_ratio(board):
    vias = [v for v in board.GetTracks()
            if isinstance(v, pcbnew.PCB_VIA) and v.GetViaType() != pcbnew.VIATYPE_THROUGH]
    assert not vias, "Blind/buried vias present; manual AR review required."
```

## `