# KiCad 8 DRC Validation Test Suite: Annular Ring, Copper-to-Edge, Drill Aspect Ratio

I don't have context for earlier cycles, so this suite is self-contained. All thresholds sit in one spec file. Swap in your Cycle 11 fab values there (e.g., the IPC class or fab house limits).

## Layout

```
drc_tests/
├── drc_spec.yaml          # Cycle 11 thresholds (edit here)
├── conftest.py            # board discovery, kicad-cli DRC run, pcbnew loading
├── test_rule_integrity.py # board rules must not be loosened below spec
├── test_native_drc.py     # kicad-cli DRC violations by category
├── test_geometry.py       # independent geometric checks via pcbnew API
└── run_drc_suite.sh       # CI entry point
```

## Approach

The suite uses three layers of defense:

1. **Rule integrity.** KiCad DRC only enforces what's in the board setup. If someone lowers a constraint to make DRC pass, that hides real problems. These tests compare the board's design settings against the spec.
2. **Native DRC.** `kicad-cli pcb drc` produces a JSON report, and the tests filter it to the relevant violation types.
3. **Independent geometry.** The tests recompute annular rings and aspect ratios directly from pad and via data. This is required for aspect ratio because KiCad 8 has no built-in aspect-ratio rule. It also cross-checks the native results.

---

### `drc_spec.yaml`

```yaml
cycle: 11
units: mm

board_thickness_default: 1.6       # used only if board stackup thickness is unavailable

annular_ring:
  via_min: 0.125                   # (via_diameter - drill) / 2
  pth_pad_min: 0.150               # (min pad dim - max drill dim) / 2
  tolerance: 0.0005

copper_edge_clearance:
  min: 0.300
  tolerance: 0.0005

drill:
  min_through_drill: 0.200
  max_aspect_ratio_through: 10.0   # board_thickness / finished drill
  max_aspect_ratio_microvia: 0.8   # dielectric depth / drill (approximated, see test)

drc:
  # Native kicad-cli violation "type" keys mapped to each category
  categories:
    annular_ring: [annular_width]
    edge_clearance: [copper_edge_clearance]
    drill: [drill_out_of_range, microvia_drill_out_of_range]
  fail_on_severity: [error]        # add "warning" for stricter gating
  allow_excluded: false            # if true, DRC exclusions in .kicad_pro are honored

boards:
  glob: "hardware/**/*.kicad_pcb"
```

### `conftest.py`

```python
import json
import os
import shutil
import subprocess
from pathlib import Path

import pytest
import yaml

ROOT = Path(os.environ.get("DRC_REPO_ROOT", Path(__file__).resolve().parents[1]))
SPEC_PATH = Path(__file__).with_name("drc_spec.yaml")
REPORT_DIR = Path(os.environ.get("DRC_REPORT_DIR", ROOT / "drc_reports"))


def _load_spec():
    with open(SPEC_PATH) as f:
        return yaml.safe_load(f)


SPEC = _load_spec()


def _discover_boards():
    boards = sorted(ROOT.glob(SPEC["boards"]["glob"]))
    # Skip KiCad autosave/backup artifacts
    return [b for b in boards if not b.name.startswith(("_autosave", "~"))]


BOARDS = _discover_boards()


@pytest.fixture(scope="session")
def spec():
    return SPEC


def pytest_generate_tests(metafunc):
    if "board_path" in metafunc.fixturenames:
        if not BOARDS:
            pytest.exit(f"No boards found for glob {SPEC['boards']['glob']}", returncode=2)
        metafunc.parametrize("board_path", BOARDS, ids=[str(b.relative_to(ROOT)) for b in BOARDS],
                             scope="session")


@pytest.fixture(scope="session")
def kicad_cli():
    exe = shutil.which("kicad-cli") or os.environ.get("KICAD_CLI")
    if not exe:
        pytest.skip("kicad-cli not found on PATH (set KICAD_CLI)")
    out = subprocess.run([exe, "version"], capture_output=True, text=True).stdout.strip()
    if not out.startswith("8."):
        pytest.fail(f"Expected KiCad 8.x, got '{out}'")
    return exe


_drc_cache = {}


@pytest.fixture(scope="session")
def drc_report(board_path, kicad_cli):
    """Run kicad-cli DRC once per board and return the parsed JSON report."""
    if board_path in _drc_cache:
        return _drc_cache[board_path]
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    out = REPORT_DIR / f"{board_path.stem}_drc.json"
    cmd = [
        kicad_cli, "pcb", "drc",
        "--format", "json",
        "--units", "mm",
        "--severity-all",
        "--all-track-errors",
        "--output", str(out),
        str(board_path),
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode not in (0, 5) or not out.exists():  # 5 = violations present (with --exit-code-violations)
        pytest.fail(f"kicad-cli DRC failed for {board_path}:\n{proc.stdout}\n{proc.stderr}")
    with open(out) as f:
        report = json.load(f)
    _drc_cache[board_path] = report
    return report


_board_cache = {}


@pytest.fixture(scope="session")
def pcb(board_path):
    """Load the board via KiCad 8's bundled pcbnew Python module."""
    pcbnew = pytest.importorskip("pcbnew", reason="Run with KiCad's bundled Python")
    if board_path not in _board_cache:
        _board_cache[board_path] = pcbnew.LoadBoard(str(board_path))
    return _board_cache[board_path]


@pytest.fixture(scope="session")
def board_thickness_mm(pcb, spec):
    import pcbnew
    t = pcb.GetDesignSettings().GetBoardThickness()
    return pcbnew.ToMM(t) if t > 0 else spec["board_thickness_default"]
```

### `test_rule_integrity.py`

```python
"""Fail if board-level constraints were loosened below the Cycle 11 spec.
Native DRC is only as strong as these settings."""
import pytest

pcbnew = pytest.importorskip("pcbnew")
EPS = 1e-6


def _mm(v):
    return pcbnew.ToMM(v)


def test_min_via_annular_ring_setting(pcb, spec):
    ds = pcb.GetDesignSettings()
    actual = _mm(ds.m_ViasMinAnnularWidth)
    assert actual + EPS >= spec["annular_ring"]["via_min"], (
        f"Board 'Minimum annular width' = {actual:.3f} mm < spec {spec['annular_ring']['via_min']} mm"
    )


def test_copper_edge_clearance_setting(pcb, spec):
    ds = pcb.GetDesignSettings()
    actual = _mm(ds.m_CopperEdgeClearance)
    assert actual + EPS >= spec["copper_edge_clearance"]["min"], (
        f"Board 'Copper to edge clearance' = {actual:.3f} mm < spec {spec['copper_edge_clearance']['min']} mm"
    )


def test_min_through_drill_setting(pcb, spec):
    ds = pcb.GetDesignSettings()
    actual = _mm(ds.m_MinThroughDrill)
    assert actual + EPS >= spec["drill"]["min_through_drill"], (
        f"Board 'Minimum through hole' = {actual:.3f} mm < spec {spec['drill']['min_through_drill']} mm"
    )


def test_drc_severities_not_ignored(board_path, spec):
    """Ensure the relevant checks aren't set to 'ignore' in the .kicad_pro."""
    import json
    pro = board_path.with_suffix(".kicad_pro")
    if not pro.exists():
        pytest.skip("No .kicad_pro; KiCad defaults apply")
    sev = (json.loads(pro.read_text())
           .get("board", {}).get("design_settings", {}).get("rule_severities", {}))
    keys = [k for cat in spec["drc"]["categories"].values() for k in cat]
    ignored = [k for k in keys if sev.get(k) == "ignore"]
    assert not ignored, f"DRC checks disabled in project: {ignored}"
```

### `test_native_drc.py`

```python
"""Assert no kicad-cli DRC violations in the targeted categories."""
import pytest

CATEGORIES = ["annular_ring", "edge_clearance", "drill"]


def _relevant(report, spec, category):
    types = set(spec["drc"]["categories"][category])
    fail_sev = set(spec["drc"]["fail_on_severity"])
    hits = []
    for v in report.get("violations", []):
        if v.get("type") not in types or v.get("severity") not in fail_sev:
            continue
        if v.get("excluded") and spec["drc"]["allow_excluded"]:
            continue
        hits.append(v)
    return hits


def _fmt(v):
    items = "; ".join(
        f"{i.get('description', '?')} @ ({i['pos']['x']:.3f}, {i['pos']['y']:.3f})"
        for i in v.get("items", []) if "pos" in i
    )
    return f"[{v['type']}] {v.get('description', '')} -> {items}"


@pytest.mark.parametrize("category", CATEGORIES)
def test_native_drc_category(drc_report, spec, category):
    hits = _relevant(drc_report, spec, category)
    assert not hits, f"{len(hits)} {category} violation(s):\n" + "\n".join(_fmt(v) for v in hits)


def test_report_schema_sane(drc_report):
    assert "violations" in drc_report, "Unexpected DRC JSON format (KiCad version mismatch?)"
```

### `test_geometry.py`

```python
"""Independent geometric checks: annular rings, drill size, aspect ratio, copper-to-edge."""
import pytest

pcbnew = pytest.importorskip("pcbnew")


def _mm(v):
    return pcbnew.ToMM(v)


def _xy(item):
    p = item.GetPosition()
    return f"({_mm(p.x):.3f}, {_mm(p.y):.3f})"


def _vias(pcb):
    return [t for t in pcb.GetTracks() if isinstance(t, pcbnew.PCB_VIA)]


def _pth_pads(pcb):
    return [p for p in pcb.GetPads() if p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH]


# ---------- Annular ring ----------

def test_via_annular_ring(pcb, spec):
    lim = spec["annular_ring"]["via_min"] - spec["annular_ring"]["tolerance"]
    bad = []
    for v in _vias(pcb):
        ring = (_mm(v.GetWidth()) - _mm(v.GetDrillValue())) / 2
        if ring < lim:
            bad.append(f"via {_xy(v)} net={v.GetNetname()} ring={ring:.3f} mm")
    assert not bad, f"Via annular ring < {spec['annular_ring']['via_min']} mm:\n" + "\n".join(bad)


def test_pth_pad_annular_ring(pcb, spec):
    lim = spec["annular_ring"]["pth_pad_min"] - spec["annular_ring"]["tolerance"]
    bad = []
    for p in _pth_pads(pcb):
        size, drill = p.GetSize(), p.GetDrillSize()
        # Conservative: worst-case ring = smallest pad dim vs largest drill dim
        ring = (min(_mm(size.x), _mm(size.y)) - max(_mm(drill.x), _mm(drill.y))) / 2
        if ring < lim:
            ref = p.GetParentFootprint().GetReference() if p.GetParentFootprint() else "?"
            bad.append(f"{ref}.{p.GetNumber()} {_xy(p)} ring={ring:.3f} mm")
    assert not bad, f"PTH annular ring < {spec['annular_ring']['pth_pad_min']} mm:\n" + "\n".join(bad)


# ---------- Drill size & aspect ratio ----------

def _through_holes(pcb):
    for v in _vias(pcb):
        if v.GetViaType() == pcbnew.VIATYPE_THROUGH:
            yield v, _mm(v.GetDrillValue()), f"via {_xy(v)}"
    for p in _pth_pads(pcb):
        d = p.GetDrillSize()
        ref = p.GetParentFootprint().GetReference() if p.GetParentFootprint() else "?"
        yield p, min(_mm(d.x), _mm(d.y)), f"{ref}.{p.GetNumber()} {_xy(p)}"


def test_min_through_drill(pcb, spec):
    lim = spec["drill"]["min_through_drill"]
    bad = [f"{lbl} drill={d:.3f} mm" for _, d, lbl in _through_holes(pcb) if d + 1e-6 < lim]
    assert not bad, f"Through drill < {lim} mm:\n" + "\n".join(bad)


def test_through_hole_aspect_ratio(pcb, spec, board_thickness_mm):
    lim = spec["drill"]["max_aspect_ratio_through"]
    bad = []
    for _, d, lbl in _through_holes(pcb):
        if d <= 0:
            bad.append(f"{lbl} has zero drill")
            continue
        ar = board_thickness_mm / d
        if ar > lim:
            bad.append(f"{lbl} drill={d