# KiCad 8 DRC Validation Test Suite: Annular Ring, Edge Clearance, and Drill Aspect Ratio

I don't have the earlier cycles of this work, so this suite is self-contained. If Cycle 21 set specific thresholds, fixture boards, or naming conventions, update `dfm_limits.toml` and the paths to match.

## How the Checks Work

The suite uses two independent layers of checking:

1. **Native DRC.** `kicad-cli pcb drc` runs with generated custom rules, and the suite parses the JSON report.
2. **Independent geometry checks.** The `pcbnew` Python API recomputes each metric. This catches cases where the native DRC was misconfigured, rules were overridden, or severities were downgraded to "ignore" in the project file.

Drill aspect ratio is not a built-in KiCad DRC constraint. The suite handles it in two ways:
- It derives a minimum hole size from the board thickness and injects it as a `hole_size` rule.
- It checks the ratio directly in Python.

```
dfm-tests/
├── dfm_limits.toml
├── conftest.py
├── kicad_helpers.py
├── test_native_drc.py
├── test_annular_ring.py
├── test_edge_clearance.py
├── test_aspect_ratio.py
├── fixtures/          # known-good and known-bad boards
└── .github/workflows/dfm.yml
```

---

## `dfm_limits.toml`

```toml
[annular_ring]
pth_pad_min_mm   = 0.15     # plated through-hole pads
via_min_mm       = 0.125    # through vias
npth_exempt      = true     # NPTH has no copper ring

[edge_clearance]
copper_min_mm    = 0.30     # copper to Edge.Cuts
include_zones    = true

[aspect_ratio]
max_through      = 10.0     # board_thickness / finished_drill
max_microvia     = 0.8      # dielectric_thickness / drill (approximation, see test)
plating_allowance_mm = 0.0  # set >0 if drills are specified pre-plating

[run]
boards_glob      = "boards/**/*.kicad_pcb"
fixtures_glob    = "fixtures/**/*.kicad_pcb"
```

---

## `kicad_helpers.py`

```python
"""Shared helpers. Must run under KiCad 8's bundled Python so `import pcbnew` works."""
from __future__ import annotations
import json, subprocess, tempfile, shutil, tomllib
from dataclasses import dataclass
from pathlib import Path

import pcbnew

NM_PER_MM = 1_000_000
LIMITS = tomllib.loads(Path(__file__).with_name("dfm_limits.toml").read_text())


def mm(nm: int | float) -> float:
    return nm / NM_PER_MM


@dataclass
class Violation:
    rule: str
    ref: str
    pos_mm: tuple[float, float]
    measured: float
    limit: float

    def __str__(self):
        return (f"[{self.rule}] {self.ref} @ ({self.pos_mm[0]:.3f},{self.pos_mm[1]:.3f}) mm: "
                f"measured {self.measured:.4f} vs limit {self.limit:.4f}")


def load_board(path: Path) -> pcbnew.BOARD:
    return pcbnew.LoadBoard(str(path))


def board_thickness_mm(board) -> float:
    return mm(board.GetDesignSettings().GetBoardThickness())


def item_ref(item) -> str:
    if isinstance(item, pcbnew.PAD):
        fp = item.GetParentFootprint()
        return f"{fp.GetReference() if fp else '?'}.{item.GetNumber()}"
    if isinstance(item, pcbnew.PCB_VIA):
        return f"via[{item.GetNetname() or 'no-net'}]"
    return item.GetClass()


def pos_mm(item) -> tuple[float, float]:
    p = item.GetPosition()
    return (mm(p.x), mm(p.y))


# ---------- custom rules generation ----------

def generate_dru(board) -> str:
    """Build a .kicad_dru that enforces our limits natively in KiCad DRC."""
    L = LIMITS
    t = board_thickness_mm(board)
    min_hole_for_ar = t / L["aspect_ratio"]["max_through"] - L["aspect_ratio"]["plating_allowance_mm"]
    return f"""(version 1)
(rule "DFM annular ring PTH"
  (condition "A.Type == 'Pad' && A.Pad_Type == 'Through-hole'")
  (constraint annular_width (min {L['annular_ring']['pth_pad_min_mm']}mm)))
(rule "DFM annular ring via"
  (condition "A.Type == 'Via' && A.Via_Type == 'Through'")
  (constraint annular_width (min {L['annular_ring']['via_min_mm']}mm)))
(rule "DFM copper to edge"
  (constraint edge_clearance (min {L['edge_clearance']['copper_min_mm']}mm)))
(rule "DFM through-drill aspect ratio"
  (condition "A.Type == 'Via' && A.Via_Type == 'Through' || A.Pad_Type == 'Through-hole'")
  (constraint hole_size (min {min_hole_for_ar:.4f}mm)))
"""


def run_native_drc(pcb_path: Path) -> dict:
    """Copy the project to a temp dir, inject custom rules, run kicad-cli DRC, and return the parsed JSON."""
    board = load_board(pcb_path)
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        src_dir = pcb_path.parent
        stem = pcb_path.stem
        for ext in (".kicad_pcb", ".kicad_pro", ".kicad_prl"):
            f = src_dir / f"{stem}{ext}"
            if f.exists():
                shutil.copy(f, td / f.name)
        # Merge with any existing project rules: ours are appended, and later rules win.
        existing = src_dir / f"{stem}.kicad_dru"
        rules = generate_dru(board)
        if existing.exists():
            body = existing.read_text().split("\n", 1)[1] if existing.read_text().startswith("(version") else existing.read_text()
            rules = rules.replace("(version 1)\n", "(version 1)\n" + body + "\n")
        (td / f"{stem}.kicad_dru").write_text(rules)

        out = td / "drc.json"
        cmd = ["kicad-cli", "pcb", "drc",
               "--format", "json", "--units", "mm",
               "--severity-all", "--all-track-errors",
               "--output", str(out), str(td / f"{stem}.kicad_pcb")]
        proc = subprocess.run(cmd, capture_output=True, text=True)
        if not out.exists():
            raise RuntimeError(f"kicad-cli failed ({proc.returncode}):\n{proc.stderr}")
        return json.loads(out.read_text())
```

---

## `conftest.py`

```python
import glob
from pathlib import Path
import pytest
from kicad_helpers import LIMITS, load_board, run_native_drc

ROOT = Path(__file__).parent

def _collect(pattern):
    return sorted(Path(p) for p in glob.glob(str(ROOT / pattern), recursive=True))

BOARDS = _collect(LIMITS["run"]["boards_glob"])
FIXTURES = _collect(LIMITS["run"]["fixtures_glob"])

def pytest_generate_tests(metafunc):
    if "pcb_path" in metafunc.fixturenames:
        metafunc.parametrize("pcb_path", BOARDS, ids=[p.stem for p in BOARDS])
    if "fixture_path" in metafunc.fixturenames:
        metafunc.parametrize("fixture_path", FIXTURES, ids=[p.stem for p in FIXTURES])

@pytest.fixture
def board(pcb_path):
    return load_board(pcb_path)

_drc_cache = {}
@pytest.fixture
def drc_report(pcb_path):
    if pcb_path not in _drc_cache:
        _drc_cache[pcb_path] = run_native_drc(pcb_path)
    return _drc_cache[pcb_path]
```

---

## `test_native_drc.py`

```python
"""Layer 1: KiCad's own DRC engine with the injected DFM rules."""
import pytest

CATEGORIES = {
    "annular_ring":   {"annular_width"},
    "edge_clearance": {"copper_edge_clearance"},
    "aspect_ratio":   {"drill_out_of_range"},   # hole_size constraint violations
}

def _violations(report, types):
    return [v for v in report.get("violations", [])
            if v.get("type") in types and v.get("severity") != "exclusion"]

def _fmt(vs):
    lines = []
    for v in vs:
        items = "; ".join(f"{i['description']} @ ({i['pos']['x']:.3f},{i['pos']['y']:.3f})"
                          for i in v.get("items", []))
        lines.append(f"  {v['type']}: {v['description']} -> {items}")
    return "\n".join(lines)

@pytest.mark.parametrize("category", CATEGORIES)
def test_native_drc_category(drc_report, category):
    vs = _violations(drc_report, CATEGORIES[category])
    assert not vs, f"{len(vs)} native DRC '{category}' violation(s):\n{_fmt(vs)}"

def test_rules_not_silenced(pcb_path):
    """Fail if the project file sets any of our rule types to 'ignore'."""
    import json
    pro = pcb_path.with_suffix(".kicad_pro")
    if not pro.exists():
        pytest.skip("no .kicad_pro")
    sev = (json.loads(pro.read_text())
           .get("board", {}).get("design_settings", {}).get("rule_severities", {}))
    silenced = [k for k in ("annular_width", "copper_edge_clearance", "drill_out_of_range")
                if sev.get(k) == "ignore"]
    assert not silenced, f"DRC severities set to ignore in project: {silenced}"
```

---

## `test_annular_ring.py`

```python
"""Layer 2: independent annular-ring computation."""
import pcbnew
from kicad_helpers import LIMITS, Violation, mm, item_ref, pos_mm

L = LIMITS["annular_ring"]

def pad_annular_ring_mm(pad) -> float | None:
    if pad.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:
        return None
    if pad.GetAttribute() != pcbnew.PAD_ATTRIB_PTH:
        return None
    drill = pad.GetDrillSize()
    if drill.x <= 0:
        return None
    # Conservative: use the smallest copper extent on any copper layer the pad uses.
    rings = []
    for layer in pad.GetLayerSet().CuStack():
        size = pad.GetSize()  # KiCad 8 uses one size for all layers (padstacks arrive in v9)
        off = pad.GetOffset()
        rx = (size.x - drill.x) / 2 - abs(off.x)
        ry = (size.y - drill.y) / 2 - abs(off.y)
        rings.append(min(rx, ry))
    return mm(min(rings)) if rings else None

def via_annular_ring_mm(via) -> float:
    return mm((via.GetWidth() - via.GetDrillValue()) / 2)

def test_pth_pad_annular_ring(board):
    bad = []
    for pad in board.GetPads():
        r = pad_annular_ring_mm(pad)
        if r is not None and r < L["pth_pad_min_mm"] - 1e-6:
            bad.append(Violation("annular_ring_pad", item_ref(pad), pos_mm(pad), r, L["pth_pad_min_mm"]))
    assert not bad, "\n".join(map(str, bad))

def test_via_annular_ring(board):
    bad = []
    for t in board.GetTracks():
        if isinstance(t, pcbnew.PCB_VIA):
            r = via_annular_ring_mm(t)
            if r < L["via_min_mm"] - 1e-6:
                bad.append(Violation("annular_ring_via", item_ref(t), pos_mm(t), r, L["via_min_mm"]))
    assert not bad, "\n".join(map(str, bad))
```

---

## `test_edge_clearance.py`

```python
"""Layer 2: copper-to-Edge.Cuts distance computed with shapely."""
import pcbnew
from shapely.geometry import Polygon, MultiPolygon
from shapely.ops import unary_union
from kicad_helpers import LIMITS, Violation, mm, item_ref, pos_mm

L = LIMITS["edge_clearance"]
MAX_ERR = pcbnew.FromMM(0.005)

def _poly_set_to_shapely(ps: pcbnew.SHAPE_POLY_SET):
    polys = []
