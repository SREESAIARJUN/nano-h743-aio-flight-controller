I don't have access to your project's existing "Cycle 28" suite or its prior revisions, so below is a complete, self-contained suite you can drop in and adjust. If you share your previous cycle's thresholds or file layout, I can align it exactly.

The suite uses two independent layers of checking:
1. **KiCad's native DRC** via `kicad-cli pcb drc` (JSON output), filtered for the relevant violation types.
2. **Independent geometric checks** via the `pcbnew` Python API. These catch cases where the board's own design rules are set too loosely. KiCad 8 has no native drill aspect ratio rule, so that check exists only here.

---

## Layout

```
drc_tests/
├── drc_spec.yaml          # fab thresholds (edit per fab / cycle)
├── conftest.py            # board discovery, DRC run, fixtures
├── test_annular_ring.py
├── test_copper_edge.py
├── test_aspect_ratio.py
└── requirements.txt       # pytest, pyyaml
```

---

## `drc_spec.yaml`

```yaml
cycle: 28
units: mm

annular_ring:
  via_min: 0.125          # min (pad - drill)/2 for vias
  pth_pad_min: 0.150      # min for plated through-hole pads
  allow_microvia_min: 0.075

copper_edge:
  min_clearance: 0.300    # copper to Edge.Cuts
  require_board_setting_at_least_spec: true

aspect_ratio:
  max_through: 10.0       # board_thickness / finished drill
  max_microvia: 0.8       # dielectric depth / drill (approximation, see notes)
  fallback_board_thickness: 1.6

kicad_drc:
  fail_on_types:
    - annular_width
    - copper_edge_clearance
    - drill_out_of_range
    - via_diameter
    - hole_clearance
  severities: [error, warning]
```

---

## `conftest.py`

```python
import json
import os
import shutil
import subprocess
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parent
SPEC = yaml.safe_load((ROOT / "drc_spec.yaml").read_text())

# Boards: env var (colon/semicolon separated) or every .kicad_pcb under repo
def _discover_boards():
    env = os.environ.get("KICAD_BOARDS")
    if env:
        return [Path(p) for p in env.replace(";", ":").split(":") if p]
    repo = Path(os.environ.get("KICAD_REPO_ROOT", ROOT.parent))
    return sorted(p for p in repo.rglob("*.kicad_pcb")
                  if "-backups" not in p.parts and not p.name.startswith("_autosave"))

BOARDS = _discover_boards()


def pytest_generate_tests(metafunc):
    if "board_path" in metafunc.fixturenames:
        metafunc.parametrize("board_path", BOARDS, ids=[b.stem for b in BOARDS])


@pytest.fixture(scope="session")
def spec():
    return SPEC


@pytest.fixture(scope="session")
def kicad_cli():
    exe = shutil.which("kicad-cli") or os.environ.get("KICAD_CLI")
    if not exe:
        pytest.skip("kicad-cli not found (set KICAD_CLI or add to PATH)")
    out = subprocess.run([exe, "version"], capture_output=True, text=True)
    if not out.stdout.strip().startswith("8."):
        pytest.skip(f"Expected KiCad 8.x, got {out.stdout.strip()!r}")
    return exe


_drc_cache = {}

@pytest.fixture
def drc_report(board_path, kicad_cli, tmp_path_factory):
    """Run kicad-cli DRC once per board, return parsed JSON."""
    if board_path in _drc_cache:
        return _drc_cache[board_path]
    out = tmp_path_factory.mktemp("drc") / f"{board_path.stem}.json"
    cmd = [kicad_cli, "pcb", "drc",
           "--format", "json",
           "--severity-all",
           "--units", "mm",
           "--all-track-errors",
           "--output", str(out),
           str(board_path)]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if not out.exists():
        pytest.fail(f"DRC did not produce a report:\n{proc.stdout}\n{proc.stderr}")
    report = json.loads(out.read_text())
    _drc_cache[board_path] = report
    return report


_board_cache = {}

@pytest.fixture
def board(board_path):
    pcbnew = pytest.importorskip("pcbnew", reason="KiCad Python (pcbnew) unavailable")
    if board_path not in _board_cache:
        _board_cache[board_path] = pcbnew.LoadBoard(str(board_path))
    return _board_cache[board_path]


def fmt_violation(v):
    locs = "; ".join(
        f"{i.get('description','?')} @ ({i['pos']['x']:.3f},{i['pos']['y']:.3f})"
        for i in v.get("items", []) if "pos" in i)
    return f"[{v['severity']}] {v['type']}: {v['description']} -> {locs}"


def drc_violations_of(report, types, severities):
    return [v for v in report.get("violations", [])
            if v.get("type") in types and v.get("severity") in severities]
```

---

## `test_annular_ring.py`

```python
import pytest
from conftest import drc_violations_of, fmt_violation


def test_kicad_drc_annular(drc_report, spec):
    viols = drc_violations_of(drc_report, {"annular_width"},
                              spec["kicad_drc"]["severities"])
    assert not viols, "KiCad annular ring violations:\n" + \
        "\n".join(fmt_violation(v) for v in viols)


def test_via_annular_geometry(board, spec):
    import pcbnew
    mm = pcbnew.ToMM
    ar = spec["annular_ring"]
    failures = []
    for t in board.GetTracks():
        if not isinstance(t, pcbnew.PCB_VIA):
            continue
        is_micro = t.GetViaType() == pcbnew.VIATYPE_MICROVIA
        limit = ar["allow_microvia_min"] if is_micro else ar["via_min"]
        ring = mm(t.GetWidth() - t.GetDrillValue()) / 2
        if ring + 1e-6 < limit:
            p = t.GetPosition()
            failures.append(
                f"via @ ({mm(p.x):.3f},{mm(p.y):.3f}) net={t.GetNetname()!r} "
                f"Ø{mm(t.GetWidth()):.3f}/{mm(t.GetDrillValue()):.3f} "
                f"ring={ring:.3f} < {limit}")
    assert not failures, f"{len(failures)} via annular failures:\n" + "\n".join(failures)


def test_pth_pad_annular_geometry(board, spec):
    import pcbnew
    mm = pcbnew.ToMM
    limit = spec["annular_ring"]["pth_pad_min"]
    failures = []
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetAttribute() != pcbnew.PAD_ATTRIB_PTH:
                continue
            size, drill = pad.GetSize(), pad.GetDrillSize()
            if drill.x == 0:
                continue
            # Worst case of both axes (handles oval/slot holes)
            ring = min(mm(size.x - drill.x), mm(size.y - drill.y)) / 2
            if ring + 1e-6 < limit:
                failures.append(
                    f"{fp.GetReference()}.{pad.GetNumber()} "
                    f"pad {mm(size.x):.3f}x{mm(size.y):.3f} "
                    f"drill {mm(drill.x):.3f}x{mm(drill.y):.3f} ring={ring:.3f} < {limit}")
    assert not failures, f"{len(failures)} PTH annular failures:\n" + "\n".join(failures)


def test_board_rule_not_looser_than_spec(board, spec):
    import pcbnew
    ds = board.GetDesignSettings()
    setting = pcbnew.ToMM(ds.m_MinRingWidth)
    assert setting + 1e-6 >= spec["annular_ring"]["via_min"], (
        f"Board min annular rule {setting:.3f} mm is looser than spec "
        f"{spec['annular_ring']['via_min']} mm; native DRC would under-report.")
```

---

## `test_copper_edge.py`

```python
from conftest import drc_violations_of, fmt_violation


def test_kicad_drc_copper_edge(drc_report, spec):
    viols = drc_violations_of(drc_report, {"copper_edge_clearance"},
                              spec["kicad_drc"]["severities"])
    assert not viols, "Copper-to-edge violations:\n" + \
        "\n".join(fmt_violation(v) for v in viols)


def test_board_edge_setting_meets_spec(board, spec):
    import pcbnew
    ce = spec["copper_edge"]
    if not ce["require_board_setting_at_least_spec"]:
        return
    setting = pcbnew.ToMM(board.GetDesignSettings().m_CopperEdgeClearance)
    assert setting + 1e-6 >= ce["min_clearance"], (
        f"Board copper-edge clearance rule {setting:.3f} mm < spec "
        f"{ce['min_clearance']} mm; tighten Board Setup > Constraints.")


def test_edge_cuts_closed(drc_report):
    # An open outline silently disables meaningful edge-clearance checking
    viols = [v for v in drc_report.get("violations", [])
             if v.get("type") in {"invalid_outline", "malformed_courtyard"}
             and v.get("type") == "invalid_outline"]
    assert not viols, "Board outline invalid:\n" + \
        "\n".join(fmt_violation(v) for v in viols)
```

---

## `test_aspect_ratio.py`

```python
import pytest


def _thickness_mm(board, spec):
    import pcbnew
    t = pcbnew.ToMM(board.GetDesignSettings().GetBoardThickness())
    return t if t > 0 else spec["aspect_ratio"]["fallback_board_thickness"]


def test_via_aspect_ratio(board, spec):
    import pcbnew
    mm = pcbnew.ToMM
    thick = _thickness_mm(board, spec)
    ar = spec["aspect_ratio"]
    failures = []
    for t in board.GetTracks():
        if not isinstance(t, pcbnew.PCB_VIA):
            continue
        drill = mm(t.GetDrillValue())
        if drill <= 0:
            continue
        vt = t.GetViaType()
        if vt == pcbnew.VIATYPE_THROUGH:
            depth, limit = thick, ar["max_through"]
        elif vt == pcbnew.VIATYPE_MICROVIA:
            # Approximate one dielectric layer; replace with stackup lookup if needed
            layers = board.GetCopperLayerCount()
            depth, limit = thick / max(layers - 1, 1), ar["max_microvia"]
        else:  # blind/buried: scale by layer span
            top, bot = t.TopLayer(), t.BottomLayer()
            layers = board.GetCopperLayerCount()
            span = abs(board.GetLayerID(board.GetLayerName(bot)) -
                       board.GetLayerID(board.GetLayerName(top))) or 1
            depth, limit = thick * min(span, layers - 1) / max(layers - 1, 1), ar["max_through"]
        ratio = depth / drill
        if ratio > limit + 1e-6:
            p = t.GetPosition()
            failures.append(
                f"via @ ({mm(p.x):.3f},{mm(p.y):.3f}) drill={drill:.3f} "
                f"depth={depth:.3f} AR={ratio:.2f} > {limit}")
    assert not failures, f"{len(failures)} aspect-ratio failures " \
        f"(thickness {thick:.3f} mm):\n" + "\n".join(failures)


def test_pth_pad_aspect_ratio(board, spec):
    import pcbnew
    mm = pcbnew.ToMM
    thick = _thickness_mm(board, spec)
    limit = spec["aspect_ratio"]["max_through"]
    failures = []
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetAttribute() != pcbnew.PAD_ATTRIB_PTH:
                continue
            d = pad.GetDrillSize()
            drill = mm(min(d.x, d.y))  # slots: narrow axis governs plating
            if drill <= 0:
                continue
            if thick / drill > limit + 1e-6:
                failures.append(f"{fp.GetReference()}.{pad.GetNumber()} "
                                f"drill={drill:.3f} AR={thick/drill:.2f} > {limit}")
    assert not failures, "\n".join(failures)
```

---

## CI (GitHub Actions example)

```yaml
name: drc-cycle-28
on: [push, pull_request]
jobs:
  drc:
    runs-on: ubuntu-latest
    container: kicad/kicad:8.0
    steps:
      - uses: actions/checkout@v4
      - run: pip install --break-system-packages pytest pyyaml
      - run: pytest drc_tests -v --junitxml=drc-results.xml
      - uses: actions/upload-artifact@v4
        if: always()
        with: { name: drc-results, path: drc-results.xml }
```

---

## Notes and caveats

- **API version:** These checks target the KiCad 8 `pcbnew` API. In KiCad 9, `PCB_VIA.GetWidth()` and `PAD.GetSize()` take a layer argument, so those calls need updating.
- **Microvia depth** is approximated as thickness divided by (copper layers − 1). For accurate checks, read dielectric thicknesses from `board.GetDesignSettings().GetStackupDescriptor()`.
- **Drill size:** Fabs quote aspect ratio against the **finished** or **tool** drill. KiCad stores finished size, so if your fab uses tool size (typically about +0.1 mm), add an offset in the spec.
- **Cycle-specific requirements:** Pin `drc_spec.yaml` per cycle. Pass/fail is then traceable to the exact thresholds used for Cycle 28.