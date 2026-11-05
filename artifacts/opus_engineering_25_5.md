# KiCad 8 DRC Validation Test Suite

I don't have the artifacts or decisions from your earlier cycles, so this is a complete, self-contained suite built against the KiCad 8 toolchain. If Cycle 24 established specific thresholds, fixture boards or CI conventions, map them into `drc_limits.yaml` and the fixture directory.

## Design

The suite uses two independent layers of checking:

| Check | Native DRC (`kicad-cli`) | Independent geometry check (`pcbnew` API) |
|---|---|---|
| Annular ring | `annular_width` violations | Recomputes the ring for every via and PTH pad |
| Copper-to-edge | `copper_edge_clearance` violations | Confirms the rule exists, meets the limit and isn't suppressed |
| Drill aspect ratio | *Not a native KiCad check* | Board thickness or span depth ÷ drill diameter |

The independent geometry checks catch two failure modes that native DRC misses:
- Someone weakened the board's `.kicad_dru` or Board Setup constraints.
- Someone set a violation's severity to "ignore".

---

## Directory Layout

```
drc_tests/
├── drc_limits.yaml
├── conftest.py
├── kicad_helpers.py
├── test_annular_ring.py
├── test_edge_clearance.py
├── test_aspect_ratio.py
├── test_suite_self_validation.py
└── fixtures/
    ├── good_board/good.kicad_pcb
    ├── bad_annular/bad_annular.kicad_pcb
    ├── bad_edge/bad_edge.kicad_pcb
    └── bad_aspect/bad_aspect.kicad_pcb
```

---

## `drc_limits.yaml`

```yaml
# Fab capability limits (mm). Tune to your fab house's spec sheet.
annular_ring:
  via_min_mm: 0.125
  pth_pad_min_mm: 0.15
  tolerance_mm: 0.0005        # float/rounding slack

copper_edge_clearance:
  min_mm: 0.30

aspect_ratio:
  through_max: 10.0           # board thickness : drill
  blind_buried_max: 8.0
  microvia_max: 1.0           # laser microvia depth : drill

# Severity names KiCad must NOT suppress
required_active_checks:
  - annular_width
  - copper_edge_clearance
  - hole_size

boards:
  glob: "hardware/**/*.kicad_pcb"
```

---

## `kicad_helpers.py`

```python
"""Helpers wrapping kicad-cli and the KiCad 8 pcbnew Python API."""
from __future__ import annotations

import json
import shutil
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path

import pcbnew  # Requires KiCad 8's bundled Python or PYTHONPATH pointing at it

NM_PER_MM = 1_000_000


def to_mm(nm: int) -> float:
    return nm / NM_PER_MM


# ---------------------------------------------------------------- native DRC
def run_native_drc(board_path: Path) -> dict:
    """Run kicad-cli DRC and return the parsed JSON report."""
    cli = shutil.which("kicad-cli")
    if cli is None:
        raise RuntimeError("kicad-cli not found on PATH (KiCad 8 required)")

    with tempfile.TemporaryDirectory() as tmp:
        report = Path(tmp) / "drc.json"
        cmd = [
            cli, "pcb", "drc",
            "--format", "json",
            "--severity-all",
            "--units", "mm",
            "--all-track-errors",
            "--output", str(report),
            str(board_path),
        ]
        # Don't use --exit-code-violations here; the tests assert on content.
        proc = subprocess.run(cmd, capture_output=True, text=True)
        if not report.exists():
            raise RuntimeError(
                f"DRC failed to produce a report for {board_path}:\n"
                f"{proc.stdout}\n{proc.stderr}"
            )
        return json.loads(report.read_text())


def violations_of_type(report: dict, vtype: str) -> list[dict]:
    return [
        v for v in report.get("violations", [])
        if v.get("type") == vtype and v.get("severity") != "exclusion"
    ]


def format_violations(vs: list[dict]) -> str:
    lines = []
    for v in vs:
        loc = ""
        if v.get("items"):
            p = v["items"][0].get("pos", {})
            loc = f" @ ({p.get('x', '?')}, {p.get('y', '?')})"
        lines.append(f"  [{v.get('severity')}] {v.get('description')}{loc}")
    return "\n".join(lines)


# ---------------------------------------------------------- geometry model
@dataclass
class HoleRecord:
    kind: str            # "via-through" | "via-blind" | "via-micro" | "pth"
    ref: str             # footprint ref/pad number, or net name for vias
    x_mm: float
    y_mm: float
    drill_mm: float
    copper_mm: float     # smallest copper dimension around the hole
    depth_mm: float      # plated span depth

    @property
    def annular_mm(self) -> float:
        return (self.copper_mm - self.drill_mm) / 2.0

    @property
    def aspect_ratio(self) -> float:
        return self.depth_mm / self.drill_mm if self.drill_mm > 0 else float("inf")


def load_board(path: Path) -> pcbnew.BOARD:
    return pcbnew.LoadBoard(str(path))


def board_thickness_mm(board: pcbnew.BOARD) -> float:
    return to_mm(board.GetDesignSettings().GetBoardThickness())


def _span_depth_mm(board: pcbnew.BOARD, top: int, bot: int) -> float:
    """Sum stackup thicknesses between two copper layers (inclusive of dielectric)."""
    stackup = board.GetDesignSettings().GetStackupDescriptor()
    items = list(stackup.GetList())
    depth_nm, inside = 0, False
    for it in items:
        layer = it.GetBrdLayerId()
        if layer == top:
            inside = True
        if inside:
            # Sum all sublayers (KiCad stores dielectric sublayers separately)
            for sub in range(it.GetSublayersCount()):
                depth_nm += it.GetThickness(sub)
        if layer == bot and inside:
            break
    # Fallback if no stackup is defined: proportional estimate
    if depth_nm == 0:
        cu = board.GetCopperLayerCount()
        t = board.GetDesignSettings().GetBoardThickness()
        idx = lambda L: 0 if L == pcbnew.F_Cu else (cu - 1 if L == pcbnew.B_Cu
                                                     else (L - pcbnew.In1_Cu) // 2 + 1
                                                     if pcbnew.In1_Cu <= L else 0)
        depth_nm = t * abs(idx(bot) - idx(top)) / max(cu - 1, 1)
    return to_mm(depth_nm)


def collect_holes(board: pcbnew.BOARD) -> list[HoleRecord]:
    holes: list[HoleRecord] = []
    thickness = board_thickness_mm(board)

    # Vias
    for trk in board.GetTracks():
        if not isinstance(trk, pcbnew.PCB_VIA):
            continue
        vt = trk.GetViaType()
        if vt == pcbnew.VIATYPE_THROUGH:
            kind, depth = "via-through", thickness
        else:
            top, bot = trk.TopLayer(), trk.BottomLayer()
            depth = _span_depth_mm(board, top, bot)
            kind = "via-micro" if vt == pcbnew.VIATYPE_MICROVIA else "via-blind"
        pos = trk.GetPosition()
        holes.append(HoleRecord(
            kind=kind,
            ref=f"via[{trk.GetNetname() or 'no-net'}]",
            x_mm=to_mm(pos.x), y_mm=to_mm(pos.y),
            drill_mm=to_mm(trk.GetDrillValue()),
            copper_mm=to_mm(trk.GetWidth()),   # KiCad 8: single padstack width
            depth_mm=depth,
        ))

    # Plated through-hole pads
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetAttribute() != pcbnew.PAD_ATTRIB_PTH:
                continue
            drill = pad.GetDrillSize()
            size = pad.GetSize()
            # Slots: the governing ring is on the narrow axis
            drill_min = min(drill.x, drill.y)
            copper_min = min(size.x, size.y)
            if drill_min <= 0:
                continue
            pos = pad.GetPosition()
            holes.append(HoleRecord(
                kind="pth",
                ref=f"{fp.GetReference()}.{pad.GetNumber()}",
                x_mm=to_mm(pos.x), y_mm=to_mm(pos.y),
                drill_mm=to_mm(drill_min),
                copper_mm=to_mm(copper_min),
                depth_mm=thickness,
            ))
    return holes
```

> **KiCad 8 vs 9 note:** `PCB_VIA.GetWidth()` with no argument is correct for KiCad 8. KiCad 9 introduced per-layer padstacks, where the call becomes `GetWidth(layer)`. Pin your CI image to 8.x, or add a version branch before upgrading.

---

## `conftest.py`

```python
from pathlib import Path

import pytest
import yaml

from kicad_helpers import collect_holes, load_board, run_native_drc

ROOT = Path(__file__).parent
REPO = ROOT.parent
LIMITS = yaml.safe_load((ROOT / "drc_limits.yaml").read_text())


def _discover_boards() -> list[Path]:
    return sorted(
        p for p in REPO.glob(LIMITS["boards"]["glob"])
        if "fixtures" not in p.parts and not p.name.startswith("_autosave")
    )


BOARDS = _discover_boards()


def pytest_generate_tests(metafunc):
    if "board_path" in metafunc.fixturenames:
        metafunc.parametrize("board_path", BOARDS, ids=[b.stem for b in BOARDS])


@pytest.fixture(scope="session")
def limits():
    return LIMITS


# Cache expensive work per board across tests
_drc_cache, _board_cache, _hole_cache = {}, {}, {}


@pytest.fixture
def drc_report(board_path):
    if board_path not in _drc_cache:
        _drc_cache[board_path] = run_native_drc(board_path)
    return _drc_cache[board_path]


@pytest.fixture
def board(board_path):
    if board_path not in _board_cache:
        _board_cache[board_path] = load_board(board_path)
    return _board_cache[board_path]


@pytest.fixture
def holes(board, board_path):
    if board_path not in _hole_cache:
        _hole_cache[board_path] = collect_holes(board)
    return _hole_cache[board_path]
```

---

## `test_annular_ring.py`

```python
import pcbnew

from kicad_helpers import format_violations, to_mm, violations_of_type


def test_native_drc_no_annular_violations(drc_report):
    vs = violations_of_type(drc_report, "annular_width")
    assert not vs, f"{len(vs)} annular ring violation(s):\n{format_violations(vs)}"


def test_via_annular_ring_independent(holes, limits):
    lim = limits["annular_ring"]
    floor = lim["via_min_mm"] - lim["tolerance_mm"]
    bad = [h for h in holes if h.kind.startswith("via") and h.annular_mm < floor]
    assert not bad, "Via annular ring below {:.3f} mm:\n{}".format(
        lim["via_min_mm"],
        "\n".join(f"  {h.ref} @ ({h.x_mm:.3f},{h.y_mm:.3f}) "
                  f"drill={h.drill_mm:.3f} dia={h.copper_mm:.3f} "
                  f"ring={h.annular_mm:.4f}" for h in bad),
    )


def test_pth_pad_annular_ring_independent(holes, limits):
    lim = limits["annular_ring"]
    floor = lim["pth_pad_min_mm"] - lim["tolerance_mm"]
    bad = [h for h in holes if h.kind == "pth" and h.annular_mm < floor]
    assert not bad, "PTH annular ring below {:.3f} mm:\n{}".format(
        lim["pth_pad_min_mm"],
        "\n".join(f"  {h.ref} drill={h.drill_mm:.3f} pad={h.copper_mm:.3f} "
                  f"ring={h.annular_mm:.4f}" for h in bad),
    )


def test_board_rule_not_weaker_than_fab_limit(board, limits):
    """Guard against someone loosening the board's own constraint."""
    ds = board.GetDesignSettings()
    configured = to_mm(ds.m_MinRingWidth)
    required = min(limits["annular_ring"]["via_min_mm"],
                   limits["annular_ring"]["pth_pad_min_mm"])
    assert configured >= required - 1e-6, (
        f"Board Setup min annular width {configured:.3f}