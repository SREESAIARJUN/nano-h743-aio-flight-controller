# KiCad 8 DRC Validation Suite: Annular Rings, Copper-to-Edge, Drill Aspect Ratio

I don't have records of your previous cycles or your fab's specific limits. This is a complete, self-contained suite with all thresholds in one config file, so you can match them to your Cycle 29 requirements.

## Approach

| Check | Method | Why |
|---|---|---|
| Annular ring | `kicad-cli pcb drc` with injected `.kicad_dru` rules, plus an independent pcbnew geometry check | Native DRC is authoritative, and the API check catches cases where rule severity is set to "ignore" in the project |
| Copper-to-edge | `kicad-cli pcb drc` with injected `edge_clearance` constraint | KiCad's geometry engine handles arcs, zones and cutouts correctly |
| Drill aspect ratio | pcbnew Python API (board thickness ÷ drill) | KiCad 8 has no native aspect-ratio constraint |

## Layout

```
drc_suite/
├── drc_config.yaml
├── conftest.py
├── drc_runner.py
├── test_annular_ring.py
├── test_edge_clearance.py
├── test_aspect_ratio.py
└── .github/workflows/drc.yml
```

## `drc_config.yaml`

```yaml
cycle: 29
boards:
  - path: ../hardware/main_board/main_board.kicad_pcb

limits:
  annular_ring_via_min_mm: 0.125
  annular_ring_pth_min_mm: 0.15
  copper_to_edge_min_mm: 0.30
  aspect_ratio_max: 10.0        # board thickness / finished drill
  min_drill_mm: 0.20
  # Optional: different aspect limit for microvias / blind vias
  aspect_ratio_microvia_max: 1.0

# Violations that are reviewed and accepted (by UUID of the item)
waivers: []
```

## `drc_runner.py`

```python
"""Helpers: run kicad-cli DRC with injected fab rules, and load boards via pcbnew."""
import json
import shutil
import subprocess
import tempfile
from pathlib import Path

DRU_TEMPLATE = """(version 1)
(rule "CYCLE_MIN_ANNULAR_VIA"
  (condition "A.Type == 'Via'")
  (constraint annular_width (min {via_ar}mm)))
(rule "CYCLE_MIN_ANNULAR_PTH"
  (condition "A.Type == 'Pad' && A.Pad_Type == 'Through-hole'")
  (constraint annular_width (min {pth_ar}mm)))
(rule "CYCLE_COPPER_TO_EDGE"
  (constraint edge_clearance (min {edge}mm)))
(rule "CYCLE_MIN_HOLE"
  (constraint hole_size (min {drill}mm)))
"""

RELEVANT_TYPES = {
    "annular_width",
    "copper_edge_clearance",
    "hole_size",            # reported as drill_out_of_range in some builds
    "drill_out_of_range",
}


def run_drc(board_path: Path, limits: dict) -> dict:
    """Copy board to a temp dir, append fab rules to its .kicad_dru, and run DRC.
    The original project is never modified."""
    board_path = Path(board_path).resolve()
    stem = board_path.stem
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        for ext in (".kicad_pcb", ".kicad_pro", ".kicad_dru"):
            src = board_path.with_suffix(ext)
            if src.exists():
                shutil.copy(src, tmp / src.name)

        dru = tmp / f"{stem}.kicad_dru"
        existing = dru.read_text() if dru.exists() else ""
        # Keep project rules; strip their version header and add ours
        body = "\n".join(l for l in existing.splitlines()
                         if not l.strip().startswith("(version"))
        dru.write_text(DRU_TEMPLATE.format(
            via_ar=limits["annular_ring_via_min_mm"],
            pth_ar=limits["annular_ring_pth_min_mm"],
            edge=limits["copper_to_edge_min_mm"],
            drill=limits["min_drill_mm"],
        ) + body)

        report = tmp / "drc.json"
        proc = subprocess.run(
            ["kicad-cli", "pcb", "drc",
             "--format", "json",
             "--severity-all",
             "--units", "mm",
             "--all-track-errors",
             "-o", str(report),
             str(tmp / board_path.name)],
            capture_output=True, text=True,
        )
        if not report.exists():
            raise RuntimeError(f"kicad-cli DRC failed:\n{proc.stdout}\n{proc.stderr}")
        return json.loads(report.read_text())


def violations_of(report: dict, types: set, waivers=()) -> list:
    out = []
    for v in report.get("violations", []):
        if v.get("type") not in types:
            continue
        uuids = {i.get("uuid") for i in v.get("items", [])}
        if uuids & set(waivers):
            continue
        out.append(v)
    return out


def fmt(vs: list) -> str:
    lines = []
    for v in vs:
        locs = "; ".join(
            f"{i.get('description','?')} @ ({i['pos']['x']:.3f},{i['pos']['y']:.3f})"
            for i in v.get("items", []) if "pos" in i
        )
        lines.append(f"  [{v['severity']}] {v['type']}: {v['description']} -> {locs}")
    return "\n".join(lines)
```

## `conftest.py`

```python
from pathlib import Path
import pytest
import yaml
from drc_runner import run_drc

CFG_PATH = Path(__file__).parent / "drc_config.yaml"
CFG = yaml.safe_load(CFG_PATH.read_text())


def pytest_generate_tests(metafunc):
    if "board_path" in metafunc.fixturenames:
        paths = [(CFG_PATH.parent / b["path"]).resolve() for b in CFG["boards"]]
        metafunc.parametrize("board_path", paths, ids=[p.stem for p in paths])


@pytest.fixture(scope="session")
def limits():
    return CFG["limits"]


@pytest.fixture(scope="session")
def waivers():
    return CFG.get("waivers", [])


_drc_cache = {}

@pytest.fixture
def drc_report(board_path, limits):
    if board_path not in _drc_cache:
        _drc_cache[board_path] = run_drc(board_path, limits)
    return _drc_cache[board_path]


@pytest.fixture
def board(board_path):
    pcbnew = pytest.importorskip("pcbnew")  # requires KiCad's bundled Python
    return pcbnew.LoadBoard(str(board_path))
```

## `test_annular_ring.py`

```python
import pcbnew
from drc_runner import violations_of, fmt

NM = 1e6  # nm per mm


def test_drc_annular_ring(drc_report, waivers):
    vs = violations_of(drc_report, {"annular_width"}, waivers)
    assert not vs, f"Annular ring violations:\n{fmt(vs)}"


def test_geometric_annular_ring_vias(board, limits):
    """Independent check that is immune to project severity overrides."""
    min_ar = limits["annular_ring_via_min_mm"]
    bad = []
    for t in board.GetTracks():
        if isinstance(t, pcbnew.PCB_VIA):
            ar = (t.GetWidth() - t.GetDrillValue()) / 2 / NM
            if ar + 1e-6 < min_ar:
                p = t.GetPosition()
                bad.append(f"via @ ({p.x/NM:.3f},{p.y/NM:.3f}) AR={ar:.3f}mm")
    assert not bad, f"Vias below {min_ar}mm AR:\n  " + "\n  ".join(bad)


def test_geometric_annular_ring_pth(board, limits):
    min_ar = limits["annular_ring_pth_min_mm"]
    bad = []
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetAttribute() != pcbnew.PAD_ATTRIB_PTH:
                continue
            size, drill = pad.GetSize(), pad.GetDrillSize()
            ar = min(size.x - drill.x, size.y - drill.y) / 2 / NM
            if ar + 1e-6 < min_ar:
                bad.append(f"{fp.GetReference()}.{pad.GetNumber()} AR={ar:.3f}mm")
    assert not bad, f"PTH pads below {min_ar}mm AR:\n  " + "\n  ".join(bad)
```

## `test_edge_clearance.py`

```python
import pcbnew
from drc_runner import violations_of, fmt


def test_board_has_closed_outline(board):
    outline = pcbnew.SHAPE_POLY_SET()
    assert board.GetBoardPolygonOutlines(outline), \
        "Edge.Cuts outline is not closed; edge clearance cannot be evaluated"
    assert outline.OutlineCount() >= 1


def test_drc_copper_to_edge(drc_report, waivers):
    vs = violations_of(drc_report, {"copper_edge_clearance"}, waivers)
    assert not vs, f"Copper-to-edge violations:\n{fmt(vs)}"


def test_board_edge_clearance_setting(board, limits):
    """Board setup must not be looser than fab limit (catches drift in .kicad_pro)."""
    ds = board.GetDesignSettings()
    actual = ds.m_CopperEdgeClearance / 1e6
    assert actual + 1e-6 >= limits["copper_to_edge_min_mm"], \
        f"Board Setup copper-to-edge = {actual}mm < {limits['copper_to_edge_min_mm']}mm"
```

## `test_aspect_ratio.py`

```python
import pcbnew

NM = 1e6


def _thickness_mm(board):
    return board.GetDesignSettings().GetBoardThickness() / NM


def test_board_thickness_sane(board):
    t = _thickness_mm(board)
    assert 0.2 <= t <= 6.0, f"Implausible board thickness {t}mm; check stackup"


def test_via_aspect_ratio(board, limits):
    thick = _thickness_mm(board)
    bad = []
    for t in board.GetTracks():
        if not isinstance(t, pcbnew.PCB_VIA):
            continue
        drill = t.GetDrillValue() / NM
        if t.GetViaType() == pcbnew.VIATYPE_MICROVIA:
            # Depth of a microvia is the span between its layers, approximated
            # here as one dielectric layer, so use the stricter microvia limit.
            limit = limits["aspect_ratio_microvia_max"]
            depth = thick / max(board.GetCopperLayerCount() - 1, 1)
        else:
            limit = limits["aspect_ratio_max"]
            depth = thick  # conservative for blind/buried vias
        ratio = depth / drill
        if ratio > limit + 1e-6:
            p = t.GetPosition()
            bad.append(f"via @ ({p.x/NM:.3f},{p.y/NM:.3f}) drill={drill:.3f}mm "
                       f"ratio={ratio:.2f}:1 > {limit}:1")
    assert not bad, f"Board {thick}mm, aspect ratio violations:\n  " + "\n  ".join(bad)


def test_pth_aspect_ratio(board, limits):
    thick = _thickness_mm(board)
    limit = limits["aspect_ratio_max"]
    bad = []
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetAttribute() not in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH):
                continue
            d = pad.GetDrillSize()
            drill = min(d.x, d.y) / NM  # slots: narrow dimension governs
            if drill <= 0:
                continue
            ratio = thick / drill
            if ratio > limit + 1e-6:
                bad.append(f"{fp.GetReference()}.{pad.GetNumber()} "
                           f"drill={drill:.3f}mm ratio={ratio:.2f}:1")
    assert not bad, f"PTH aspect ratio > {limit}:1:\n  " + "\n  ".join(bad)
```

## CI: `.github/workflows/drc.yml`

```yaml
name: PCB DRC
on: [push, pull_request]
jobs:
  drc:
    runs-on: ubuntu-latest
    container: kicad/kicad:8.0
    steps:
      - uses: actions/checkout@v4
      - name: Install test deps into KiCad's Python
        run: |
          python3 -m pip install --break-system-packages pytest pyyaml
      - name: Run DRC suite
        working-directory: drc_suite
        run: python3 -m pytest -v --junitxml=drc-results.xml
      - uses: actions/upload-artifact@v4
        if: always()
        with:
          name: drc-results
          path: drc_suite/drc-results.xml
```

## Notes

- **pcbnew must come from KiCad's interpreter.** On Windows, use KiCad's bundled `python.exe`. On macOS, use `/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/.../python3`.
- **API version pinning.** `PCB_VIA.GetWidth()` takes no argument in KiCad 8. KiCad 9 changes this to `GetWidth(layer)`, so pin the container to `kicad/kicad:8.0`.
- **Finished vs. drilled hole.** KiCad stores finished hole size. If your fab computes aspect ratio on the drilled size (typically finished plus about 0.05–0.1 mm), tighten `aspect_ratio_max` accordingly.
- **Blind and buried vias.** The aspect check treats them as full-thickness, which is conservative. For exact results, compute depth from the stackup using `t.GetLayerSet()` and the dielectric thicknesses in `ds.GetStackupDescriptor()`.
- **Waivers.** Add the item UUIDs from the DRC JSON to `waivers` in the config. Keep a justification comment beside each one for the cycle review.

If you share your fab house's capability sheet or the Cyc