# KiCad 8 DRC Validation Test Suite (Cycle 7)

> **Note:** I don't have the Cycle 1–6 artifacts or your project's specific thresholds. This suite is self-contained, with all limits in one policy file. Replace the default values with your fab's capabilities or your earlier-cycle numbers.

## Overview

The suite validates three rule families using two independent methods, so a misconfigured board rule set cannot hide a violation.

| Check | Layer A: `kicad-cli` DRC | Layer B: Independent geometry check (pcbnew API) |
|---|---|---|
| Annular ring | `annular_width` violations | Computes `(pad/via size − drill) / 2` for every plated hole |
| Copper-to-edge | `copper_edge_clearance` violations | Verifies the board's configured clearance meets the policy minimum |
| Drill aspect ratio | Not natively checked in KiCad 8 | Computes `board_thickness / drill` for every via and PTH |

It also checks that the board's own design rules are not looser than the policy.

---

## Directory layout

```
drc_suite/
├── policy.yaml
├── conftest.py
├── drc_lib.py
├── tests/
│   ├── test_cli_drc.py
│   ├── test_geometry.py
│   ├── test_rule_config.py
│   └── test_selftest.py
├── fixtures/
│   └── bad/               # known-bad boards, one violation type each
└── .github/workflows/drc.yml
```

---

## `policy.yaml`

```yaml
# Fab capability limits. All dimensions are in mm.
cycle: 7
annular_ring:
  via_min_mm: 0.125         # IPC-2221 Class 2 typical
  pth_min_mm: 0.150
copper_to_edge:
  min_mm: 0.30
  # Optional exemption list, e.g. edge-plated or castellated pads.
  exempt_refs: []
aspect_ratio:
  via_max: 8.0              # thickness : drill
  pth_max: 10.0
  # Used only if the stackup does not define a thickness.
  default_board_thickness_mm: 1.6
drc:
  # DRC types that must always be zero, regardless of severity.
  fatal_types:
    - annular_width
    - copper_edge_clearance
    - edge_clearance
    - hole_clearance
    - drill_out_of_range
    - via_diameter
  fail_on_unconnected: false
boards:
  glob: "hardware/**/*.kicad_pcb"
```

---

## `drc_lib.py`

```python
"""Helpers for running kicad-cli DRC and inspecting KiCad 8 boards."""
from __future__ import annotations
import json, shutil, subprocess, tempfile
from dataclasses import dataclass
from pathlib import Path

import pcbnew  # Provided by the KiCad 8 install

NM_PER_MM = 1_000_000


def mm(nm: int) -> float:
    return nm / NM_PER_MM


# ---------- Layer A: kicad-cli ----------
def run_cli_drc(board: Path) -> dict:
    cli = shutil.which("kicad-cli")
    if not cli:
        raise RuntimeError("kicad-cli not found on PATH")
    with tempfile.TemporaryDirectory() as td:
        out = Path(td) / "drc.json"
        cmd = [cli, "pcb", "drc",
               "--format", "json",
               "--severity-all",
               "--units", "mm",
               "--schematic-parity" if False else "",
               "-o", str(out), str(board)]
        cmd = [c for c in cmd if c]
        proc = subprocess.run(cmd, capture_output=True, text=True)
        # Exit code 0 means the run succeeded, even if violations exist.
        if proc.returncode not in (0, 5) or not out.exists():
            raise RuntimeError(f"kicad-cli failed ({proc.returncode}):\n{proc.stderr}")
        return json.loads(out.read_text())


def violations_by_type(report: dict) -> dict[str, list[dict]]:
    buckets: dict[str, list[dict]] = {}
    for v in report.get("violations", []):
        buckets.setdefault(v["type"], []).append(v)
    return buckets


# ---------- Layer B: geometry ----------
@dataclass
class Hole:
    kind: str          # "via" or "pth"
    ref: str
    x_mm: float
    y_mm: float
    drill_mm: float
    size_mm: float     # Smallest copper dimension around the hole

    @property
    def annular_mm(self) -> float:
        return (self.size_mm - self.drill_mm) / 2

    def where(self) -> str:
        return f"{self.kind} {self.ref} @ ({self.x_mm:.3f}, {self.y_mm:.3f})"


def load_board(path: Path) -> "pcbnew.BOARD":
    return pcbnew.LoadBoard(str(path))


def board_thickness_mm(board, default_mm: float) -> float:
    t = board.GetDesignSettings().GetBoardThickness()
    return mm(t) if t > 0 else default_mm


def plated_holes(board) -> list[Hole]:
    holes: list[Hole] = []
    for t in board.GetTracks():
        if isinstance(t, pcbnew.PCB_VIA):
            pos = t.GetPosition()
            holes.append(Hole("via", f"net:{t.GetNetname() or '?'}",
                              mm(pos.x), mm(pos.y),
                              mm(t.GetDrillValue()), mm(t.GetWidth())))
    for pad in board.GetPads():
        if pad.GetAttribute() != pcbnew.PAD_ATTRIB_PTH:
            continue
        d = pad.GetDrillSize()
        s = pad.GetSize()
        if d.x <= 0:
            continue
        pos = pad.GetPosition()
        fp = pad.GetParentFootprint()
        ref = f"{fp.GetReference() if fp else '?'}.{pad.GetNumber()}"
        # Slots and oval pads: compare the minor axes conservatively.
        holes.append(Hole("pth", ref, mm(pos.x), mm(pos.y),
                          mm(max(d.x, d.y)) if d.x != d.y else mm(d.x),
                          mm(min(s.x, s.y))))
    return holes
```

> **Version note:** `PCB_VIA.GetWidth()` takes no arguments in KiCad 8. It requires a layer argument in KiCad 9+. Pin your runner to KiCad 8.0.x.

---

## `conftest.py`

```python
from pathlib import Path
import pytest, yaml
from drc_lib import run_cli_drc, load_board

ROOT = Path(__file__).resolve().parent
POLICY = yaml.safe_load((ROOT / "policy.yaml").read_text())


def _boards():
    repo = ROOT.parent
    found = sorted(repo.glob(POLICY["boards"]["glob"]))
    return [b for b in found if "fixtures" not in b.parts]


def pytest_generate_tests(metafunc):
    if "board_path" in metafunc.fixturenames:
        boards = _boards()
        metafunc.parametrize("board_path", boards,
                             ids=[b.stem for b in boards])


@pytest.fixture(scope="session")
def policy():
    return POLICY


_cache: dict = {}


@pytest.fixture
def drc_report(board_path):
    if ("drc", board_path) not in _cache:
        _cache[("drc", board_path)] = run_cli_drc(board_path)
    return _cache[("drc", board_path)]


@pytest.fixture
def board(board_path):
    if ("brd", board_path) not in _cache:
        _cache[("brd", board_path)] = load_board(board_path)
    return _cache[("brd", board_path)]
```

---

## `tests/test_cli_drc.py`

```python
from drc_lib import violations_by_type


def _fmt(vs):
    return "\n".join(f"  - {v['description']} | "
                     + "; ".join(i.get("description", "") for i in v.get("items", []))
                     for v in vs[:25])


def test_no_annular_ring_violations(drc_report):
    v = violations_by_type(drc_report).get("annular_width", [])
    assert not v, f"{len(v)} annular ring violation(s):\n{_fmt(v)}"


def test_no_copper_edge_violations(drc_report, policy):
    exempt = set(policy["copper_to_edge"]["exempt_refs"])
    b = violations_by_type(drc_report)
    v = b.get("copper_edge_clearance", []) + b.get("edge_clearance", [])
    v = [x for x in v
         if not any(r in i.get("description", "")
                    for i in x.get("items", []) for r in exempt)]
    assert not v, f"{len(v)} copper-to-edge violation(s):\n{_fmt(v)}"


def test_no_fatal_drc_types(drc_report, policy):
    b = violations_by_type(drc_report)
    bad = {t: b[t] for t in policy["drc"]["fatal_types"] if t in b}
    assert not bad, "Fatal DRC types present: " + \
        ", ".join(f"{t}={len(v)}" for t, v in bad.items())


def test_unconnected(drc_report, policy):
    if not policy["drc"]["fail_on_unconnected"]:
        return
    u = drc_report.get("unconnected_items", [])
    assert not u, f"{len(u)} unconnected item(s)"
```

---

## `tests/test_geometry.py`

This is the independent check. It does not trust the board's own rules.

```python
from drc_lib import plated_holes, board_thickness_mm

EPS = 1e-4  # mm, tolerance for nm rounding


def test_annular_ring_geometry(board, policy):
    lim = policy["annular_ring"]
    fails = []
    for h in plated_holes(board):
        need = lim["via_min_mm"] if h.kind == "via" else lim["pth_min_mm"]
        if h.annular_mm + EPS < need:
            fails.append(f"{h.where()}: ring {h.annular_mm:.3f} < {need:.3f} mm "
                         f"(size {h.size_mm:.3f}, drill {h.drill_mm:.3f})")
    assert not fails, f"{len(fails)} undersized ring(s):\n" + "\n".join(fails[:25])


def test_drill_aspect_ratio(board, policy):
    ar = policy["aspect_ratio"]
    t = board_thickness_mm(board, ar["default_board_thickness_mm"])
    fails = []
    for h in plated_holes(board):
        limit = ar["via_max"] if h.kind == "via" else ar["pth_max"]
        ratio = t / h.drill_mm
        if ratio > limit + EPS:
            fails.append(f"{h.where()}: AR {ratio:.2f}:1 > {limit}:1 "
                         f"(t={t:.3f}, drill={h.drill_mm:.3f})")
    assert not fails, f"{len(fails)} aspect-ratio violation(s):\n" + "\n".join(fails[:25])


def test_board_has_outline(board):
    import pcbnew
    poly = pcbnew.SHAPE_POLY_SET()
    ok = board.GetBoardPolygonOutlines(poly)
    assert ok and poly.OutlineCount() > 0, \
        "Edge.Cuts outline missing or not closed, so the edge clearance check is meaningless"
```

---

## `tests/test_rule_config.py`

This catches boards that pass DRC only because someone loosened their rules.

```python
from drc_lib import mm
import pytest


def test_copper_edge_rule_not_loosened(board, policy):
    ds = board.GetDesignSettings()
    configured = mm(ds.m_CopperEdgeClearance)
    need = policy["copper_to_edge"]["min_mm"]
    assert configured + 1e-4 >= need, \
        f"Board copper-to-edge rule {configured:.3f} mm < policy {need:.3f} mm"


def test_annular_rule_not_loosened(board, policy):
    ds = board.GetDesignSettings()
    configured = mm(ds.m_ViasMinAnnularWidth)
    need = min(policy["annular_ring"]["via_min_mm"],
               policy["annular_ring"]["pth_min_mm"])
    assert configured + 1e-4 >= need, \
        f"Board min annular rule {configured:.3f} mm < policy {need:.3f} mm"


def test_board_thickness_defined(board):
    if board.GetDesignSettings().GetBoardThickness() <= 0:
        pytest.xfail("No stackup thickness; aspect ratio uses the policy default")
```

---

## `tests/test_selftest.py`

This proves the suite actually catches each violation type (mutation-style). Place one deliberately broken board per check in `fixtures/bad/`:

| Fixture | Defect |
|---|---|
| `bad_annular.kicad_pcb` | Via 0.45 mm / drill 0.30 mm (0.075 mm ring) |
| `bad_edge.kicad_pcb` | Track 0.10 mm from Edge.Cuts, board rule set to 0.30 mm |
| `bad_aspect.kicad_pcb` | 2.4 mm stackup with a 0.20 mm via drill (12:1) |

```python
from pathlib import Path
import pytest
from drc_lib import run_cli_drc, violations_by_type, load_board, plated_holes, board_thickness_mm

BAD = Path(__file__).resolve().parents[1] / "fixtures" / "bad"


@pytest.mark.selftest
def test_detects_bad_annular(policy):
    b = BAD / "bad_annular.kicad_pcb"
    assert violations_by_type(run_cli_drc(b)).get("annular_width")
    rings = [h.annular_mm for h in plated_holes(load_board(b))]
    assert min(rings) < policy["annular_ring"]["via_min_mm"]


@pytest.mark.selftest
def test_detects_bad_edge():
    v = violations_by_type(run_cli_drc(BAD / "bad_edge.kicad_pcb"))
    assert v.get("copper_edge_clearance") or v.get("edge_clearance")


@pytest.mark.selftest
def test_detects_bad_aspect(policy):
    brd = load_board(BAD / "bad_aspect.kicad_pcb")
    t = board_thickness_mm(brd, 1.6)
    assert max(t / h.drill_mm