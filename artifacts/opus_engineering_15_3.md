# KiCad 8 DRC Validation Suite: Annular Ring, Copper-to-Edge, Drill Aspect Ratio

I don't have the artifacts from earlier cycles, so this is a complete, self-contained suite. If Cycle 14 used different thresholds, file names, or a different fab profile, change `drc_config.json` and the `.kicad_dru` file. The test logic does not need to change.

The suite checks each rule in two independent ways:

1. **Native DRC** runs through `kicad-cli pcb drc` with a custom rules file. This is the authoritative KiCad engine.
2. **Independent geometry checks** read the board through the `pcbnew` Python API. These catch rule-file regressions, such as someone loosening a constraint. They also cover drill aspect ratio, which KiCad 8 does not check natively.

---

## Layout

```
drc-suite/
├── drc_config.json
├── rules/
│   └── fab_profile.kicad_dru
├── tests/
│   ├── conftest.py
│   ├── geometry.py
│   ├── test_native_drc.py
│   ├── test_annular_ring.py
│   ├── test_edge_clearance.py
│   ├── test_aspect_ratio.py
│   └── test_self_check.py
├── fixtures/                 # known-bad boards for self-test
│   ├── bad_annular.kicad_pcb
│   ├── bad_edge.kicad_pcb
│   └── bad_aspect.kicad_pcb
└── .github/workflows/drc.yml
```

---

## `drc_config.json`

```json
{
  "cycle": 15,
  "boards": ["hardware/main/main.kicad_pcb"],
  "units": "mm",
  "annular_ring": {
    "via_min": 0.125,
    "pth_pad_min": 0.15,
    "microvia_min": 0.075
  },
  "edge_clearance": {
    "copper_min": 0.30,
    "exempt_footprints": ["MountingHole*", "TestPoint_Edge*"]
  },
  "aspect_ratio": {
    "through_max": 10.0,
    "blind_buried_max": 1.0,
    "microvia_max": 0.8,
    "fallback_board_thickness_mm": 1.6
  },
  "tolerance_mm": 0.0005
}
```

---

## `rules/fab_profile.kicad_dru`

```lisp
(version 1)

(rule "via_annular_ring"
  (condition "A.Type == 'Via' && A.Via_Type != 'Micro'")
  (constraint annular_width (min 0.125mm)))

(rule "microvia_annular_ring"
  (condition "A.Via_Type == 'Micro'")
  (constraint annular_width (min 0.075mm)))

(rule "pth_pad_annular_ring"
  (condition "A.Type == 'Pad' && A.Pad_Type == 'Through-hole'")
  (constraint annular_width (min 0.15mm)))

(rule "copper_to_edge"
  (condition "A.isPlated() || A.Type == 'Track' || A.Type == 'Zone' || A.Type == 'Via' || A.Type == 'Pad'")
  (constraint edge_clearance (min 0.30mm)))

(rule "edge_exempt_mounting"
  (condition "A.memberOfFootprint('MountingHole*')")
  (constraint edge_clearance (min 0mm)))
```

Install the rules by copying the file next to each board as `<board>.kicad_dru`. KiCad 8 only picks up rules from the project directory. The conftest fixture below does this automatically.

---

## `tests/conftest.py`

```python
import json
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
CONFIG = json.loads((ROOT / "drc_config.json").read_text())


def pytest_generate_tests(metafunc):
    if "board_path" in metafunc.fixturenames:
        boards = [ROOT / b for b in CONFIG["boards"]]
        metafunc.parametrize("board_path", boards, ids=[b.stem for b in boards])


@pytest.fixture(scope="session")
def cfg():
    return CONFIG


@pytest.fixture(scope="session")
def kicad_cli():
    exe = shutil.which("kicad-cli")
    if not exe:
        pytest.skip("kicad-cli not on PATH")
    ver = subprocess.run([exe, "version"], capture_output=True, text=True).stdout.strip()
    assert ver.startswith("8."), f"Expected KiCad 8.x, got {ver!r}"
    return exe


@pytest.fixture(scope="session")
def pcbnew_mod():
    return pytest.importorskip("pcbnew")


def install_rules(board_path: Path) -> Path:
    dst = board_path.with_suffix(".kicad_dru")
    shutil.copyfile(ROOT / "rules" / "fab_profile.kicad_dru", dst)
    return dst


def run_drc(kicad_cli: str, board_path: Path, out_dir: Path) -> dict:
    install_rules(board_path)
    report = out_dir / f"{board_path.stem}_drc.json"
    subprocess.run(
        [kicad_cli, "pcb", "drc",
         "--format", "json", "--units", "mm",
         "--severity-all", "--all-track-errors",
         "-o", str(report), str(board_path)],
        check=False, capture_output=True, text=True,
    )
    assert report.exists(), "kicad-cli produced no report"
    return json.loads(report.read_text())


@pytest.fixture
def drc_report(kicad_cli, board_path, tmp_path_factory):
    return run_drc(kicad_cli, board_path, tmp_path_factory.mktemp("drc"))


@pytest.fixture
def board(pcbnew_mod, board_path):
    return pcbnew_mod.LoadBoard(str(board_path))
```

---

## `tests/geometry.py`: independent measurement helpers

```python
import fnmatch
import math

import pcbnew

NM = 1e-6  # nm -> mm


def mm(v):
    return v * NM


def via_kind(via):
    t = via.GetViaType()
    if t == pcbnew.VIATYPE_MICROVIA:
        return "micro"
    if t == pcbnew.VIATYPE_BLIND_BURIED:
        return "blind_buried"
    return "through"


def via_annular(via):
    return mm(via.GetWidth() - via.GetDrillValue()) / 2.0


def pad_annular(pad):
    """Minimum ring for a through-hole pad (handles oval/rect conservatively)."""
    size, drill = pad.GetSize(), pad.GetDrillSize()
    return min(mm(size.x - drill.x), mm(size.y - drill.y)) / 2.0


def is_pth(pad):
    return pad.GetAttribute() == pcbnew.PAD_ATTRIB_PTH and pad.GetDrillSize().x > 0


# ---- Edge outline ---------------------------------------------------------

def outline_segments(board):
    poly = pcbnew.SHAPE_POLY_SET()
    ok = board.GetBoardPolygonOutlines(poly)
    assert ok, "Board outline (Edge.Cuts) is not closed"
    segs = []
    for oi in range(poly.OutlineCount()):
        chains = [poly.Outline(oi)] + [poly.Hole(oi, h) for h in range(poly.HoleCount(oi))]
        for ch in chains:
            n = ch.PointCount()
            for i in range(n):
                a, b = ch.CPoint(i), ch.CPoint((i + 1) % n)
                segs.append(((mm(a.x), mm(a.y)), (mm(b.x), mm(b.y))))
    return segs


def _pt_seg(p, a, b):
    ax, ay = a; bx, by = b; px, py = p
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _seg_seg(a1, a2, b1, b2):
    def ccw(p, q, r):
        return (r[1] - p[1]) * (q[0] - p[0]) - (q[1] - p[1]) * (r[0] - p[0])
    d1, d2 = ccw(b1, b2, a1), ccw(b1, b2, a2)
    d3, d4 = ccw(a1, a2, b1), ccw(a1, a2, b2)
    if (d1 * d2 < 0) and (d3 * d4 < 0):
        return 0.0
    return min(_pt_seg(a1, b1, b2), _pt_seg(a2, b1, b2),
               _pt_seg(b1, a1, a2), _pt_seg(b2, a1, a2))


def dist_point_to_edge(p, segs):
    return min(_pt_seg(p, a, b) for a, b in segs)


def dist_segment_to_edge(p1, p2, segs):
    return min(_seg_seg(p1, p2, a, b) for a, b in segs)


def footprint_exempt(item, patterns):
    fp = item.GetParentFootprint() if hasattr(item, "GetParentFootprint") else None
    if fp is None:
        return False
    name = fp.GetFPIDAsString().split(":")[-1]
    return any(fnmatch.fnmatch(name, pat) for pat in patterns)


# ---- Stackup / drill depth -----------------------------------------------

def board_thickness(board, fallback):
    t = mm(board.GetDesignSettings().GetBoardThickness())
    return t if t > 0 else fallback


def span_thickness(board, top_layer, bot_layer, fallback_total):
    """Physical depth between two copper layers using the stackup.
    Falls back to proportional estimate if no stackup is defined."""
    ds = board.GetDesignSettings()
    stack = ds.GetStackupDescriptor()
    items = list(stack.GetList()) if stack else []
    if not items:
        n = board.GetCopperLayerCount()
        i_t = 0 if top_layer == pcbnew.F_Cu else (n - 1 if top_layer == pcbnew.B_Cu else top_layer // 2)
        i_b = 0 if bot_layer == pcbnew.F_Cu else (n - 1 if bot_layer == pcbnew.B_Cu else bot_layer // 2)
        return fallback_total * abs(i_b - i_t) / max(n - 1, 1)

    depth, inside = 0.0, False
    for it in items:
        lid = it.GetBrdLayerId()
        if lid == top_layer:
            inside = True
            continue
        if inside:
            if lid == bot_layer:
                break
            for s in range(it.GetSublayersCount()):
                depth += mm(it.GetThickness(s))
    return depth
```

---

## `tests/test_native_drc.py`: authoritative KiCad engine

```python
import pytest

CATEGORIES = {
    "annular_width": "Annular ring",
    "copper_edge_clearance": "Copper-to-edge clearance",
    "drill_out_of_range": "Drill size",
    "via_diameter": "Via diameter",
}


def _of_type(report, t):
    return [v for v in report.get("violations", []) if v["type"] == t]


@pytest.mark.parametrize("vtype", list(CATEGORIES))
def test_no_native_violations(drc_report, vtype):
    hits = _of_type(drc_report, vtype)
    detail = "\n".join(
        f"  [{v['severity']}] {v['description']} @ "
        + ", ".join(f"({i['pos']['x']:.3f},{i['pos']['y']:.3f})" for i in v.get("items", []))
        for v in hits
    )
    assert not hits, f"{CATEGORIES[vtype]}: {len(hits)} violation(s)\n{detail}"


def test_no_excluded_relevant_violations(drc_report):
    """Exclusions must not silently hide the rule classes under audit."""
    hidden = [v for v in drc_report.get("violations", [])
              if v.get("excluded") and v["type"] in CATEGORIES]
    assert not hidden, f"{len(hidden)} relevant violation(s) are marked excluded"


def test_board_outline_valid(drc_report):
    bad = _of_type(drc_report, "invalid_outline")
    assert not bad, "Edge.Cuts outline invalid; edge-clearance results unreliable"
```

---

## `tests/test_annular_ring.py`

```python
import pytest
from geometry import via_annular, via_kind, pad_annular, is_pth


def test_via_annular_ring(board, cfg):
    tol = cfg["tolerance_mm"]
    ar = cfg["annular_ring"]
    fails = []