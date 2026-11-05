# KiCad 8 DRC Validation Test Suite

I don't have context on what "Cycle 31" covers in your process, such as prior cycles, fab house, or thresholds. This suite is self-contained, with all thresholds in one config file. Adjust the values to match your Cycle 31 requirements.

## Overview

The suite runs three layers of checks:

1. **Native DRC** uses `kicad-cli pcb drc` with JSON output. It fails on `annular_width`, `copper_edge_clearance`, and drill-related violations.
2. **Rule-floor audit** confirms the project's `.kicad_pro` design rules are at least as strict as the fab minimums. This stops someone from loosening a rule to make DRC pass.
3. **Independent geometry checks** parse the `.kicad_pcb` file directly. They recompute annular rings and drill aspect ratios. KiCad has no native aspect-ratio rule, so this check is necessary.

```
drc_tests/
├── drc_config.yaml
├── kicad_board.py        # S-expression parser + kicad-cli wrapper
├── conftest.py
├── test_native_drc.py
├── test_annular_ring.py
├── test_edge_clearance.py
├── test_aspect_ratio.py
└── .github/workflows/drc.yml
```

---

## `drc_config.yaml`

```yaml
cycle: 31
boards:
  - path: hardware/main/main.kicad_pcb
  # - path: hardware/daughter/daughter.kicad_pcb

limits:
  annular_ring:
    via_min_mm: 0.125          # (via_dia - drill) / 2
    pth_pad_min_mm: 0.15
  copper_to_edge:
    min_mm: 0.30
  drill:
    min_mech_drill_mm: 0.20
    max_aspect_ratio: 10.0     # board_thickness / drill_dia (through holes)
    max_aspect_ratio_microvia: 0.8  # depth / drill for blind/micro vias
  default_board_thickness_mm: 1.6

# DRC violation types that must be zero (KiCad 8 JSON "type" field)
blocking_violation_types:
  - annular_width
  - copper_edge_clearance
  - drill_out_of_range
  - microvia_drill_out_of_range
  - hole_clearance
  - hole_to_hole
  - edge_clearance   # legacy alias, harmless if absent

kicad_cli: kicad-cli
```

---

## `kicad_board.py`

```python
"""Minimal KiCad 8 board/project reader and kicad-cli DRC runner."""
from __future__ import annotations

import json
import math
import re
import shutil
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path

# ---------------------------------------------------------------- S-expr ---
_TOKEN = re.compile(r'\(|\)|"(?:\\.|[^"\\])*"|[^\s()"]+')


def parse_sexpr(text: str):
    stack, cur = [], []
    for tok in _TOKEN.findall(text):
        if tok == "(":
            stack.append(cur)
            cur = []
        elif tok == ")":
            done, cur = cur, stack.pop()
            cur.append(done)
        else:
            if tok.startswith('"'):
                tok = bytes(tok[1:-1], "utf-8").decode("unicode_escape")
            cur.append(tok)
    return cur[0]


def children(node, name):
    return [c for c in node if isinstance(c, list) and c and c[0] == name]


def child(node, name):
    found = children(node, name)
    return found[0] if found else None


# ------------------------------------------------------------- Geometry ---
@dataclass
class Hole:
    kind: str            # "via" | "pth"
    ref: str             # footprint ref/pad or "via"
    x: float
    y: float
    copper_w: float      # smallest copper dimension
    drill: float         # smallest drill dimension
    layers: tuple[str, str]
    via_type: str = "through"   # through | blind | micro

    @property
    def annular_ring(self) -> float:
        return (self.copper_w - self.drill) / 2.0


class Board:
    def __init__(self, path: Path):
        self.path = Path(path)
        self.tree = parse_sexpr(self.path.read_text(encoding="utf-8"))
        self.project = self._load_project()

    # -- project rules ----------------------------------------------------
    def _load_project(self) -> dict:
        pro = self.path.with_suffix(".kicad_pro")
        return json.loads(pro.read_text()) if pro.exists() else {}

    @property
    def rules(self) -> dict:
        return (self.project.get("board", {})
                .get("design_settings", {}).get("rules", {}))

    # -- stackup ----------------------------------------------------------
    def thickness(self, default: float) -> float:
        gen = child(self.tree, "general")
        if gen and (t := child(gen, "thickness")):
            return float(t[1])
        return default

    def copper_layer_order(self) -> list[str]:
        layers = child(self.tree, "layers") or []
        return [l[1] for l in layers[1:]
                if isinstance(l, list) and l[1].endswith(".Cu")]

    def span_depth(self, a: str, b: str, total: float) -> float:
        """Approximate drilled depth between two copper layers."""
        order = self.copper_layer_order()
        if len(order) < 2 or a not in order or b not in order:
            return total
        i, j = sorted((order.index(a), order.index(b)))
        return total * (j - i) / (len(order) - 1)

    # -- holes ------------------------------------------------------------
    def holes(self) -> list[Hole]:
        out: list[Hole] = []
        for v in children(self.tree, "via"):
            at = child(v, "at")
            size = float(child(v, "size")[1])
            drill = float(child(v, "drill")[1])
            lyr = child(v, "layers")
            vtype = "through"
            if any(c == "blind" for c in v):
                vtype = "blind"
            if any(c == "micro" for c in v):
                vtype = "micro"
            out.append(Hole("via", "via", float(at[1]), float(at[2]),
                            size, drill, (lyr[1], lyr[2]), vtype))

        for fp in children(self.tree, "footprint"):
            ref = next((p[2] for p in children(fp, "property")
                        if p[1] == "Reference"), "?")
            fat = child(fp, "at")
            fx, fy = float(fat[1]), float(fat[2])
            for pad in children(fp, "pad"):
                if pad[2] != "thru_hole":       # skip np_thru_hole / smd
                    continue
                d = child(pad, "drill")
                nums = [float(x) for x in d[1:]
                        if isinstance(x, str) and _is_num(x)]
                drill = min(nums) if nums else 0.0
                sz = child(pad, "size")
                cu = min(float(sz[1]), float(sz[2]))
                pat = child(pad, "at")
                out.append(Hole("pth", f"{ref}.{pad[1]}",
                                fx + float(pat[1]), fy + float(pat[2]),
                                cu, drill, ("F.Cu", "B.Cu")))
        return out


def _is_num(s: str) -> bool:
    try:
        float(s)
        return True
    except ValueError:
        return False


# ---------------------------------------------------------- kicad-cli ---
def run_drc(board: Path, cli: str = "kicad-cli") -> dict:
    if not shutil.which(cli):
        raise RuntimeError(f"{cli} not found on PATH")
    with tempfile.TemporaryDirectory() as td:
        report = Path(td) / "drc.json"
        cmd = [cli, "pcb", "drc", "--format", "json", "--severity-all",
               "--units", "mm", "-o", str(report), str(board)]
        proc = subprocess.run(cmd, capture_output=True, text=True)
        if not report.exists():
            raise RuntimeError(f"DRC failed:\n{proc.stdout}\n{proc.stderr}")
        return json.loads(report.read_text())
```

---

## `conftest.py`

```python
from pathlib import Path

import pytest
import yaml

from kicad_board import Board, run_drc

ROOT = Path(__file__).resolve().parent
CFG = yaml.safe_load((ROOT / "drc_config.yaml").read_text())
REPO = ROOT.parent


def pytest_generate_tests(metafunc):
    if "board" in metafunc.fixturenames:
        paths = [b["path"] for b in CFG["boards"]]
        metafunc.parametrize("board", paths, indirect=True, ids=paths)


@pytest.fixture(scope="session")
def cfg():
    return CFG


@pytest.fixture(scope="session")
def _board_cache():
    return {}


@pytest.fixture
def board(request, _board_cache):
    p = REPO / request.param
    if p not in _board_cache:
        _board_cache[p] = Board(p)
    return _board_cache[p]


@pytest.fixture(scope="session")
def _drc_cache():
    return {}


@pytest.fixture
def drc_report(board, cfg, _drc_cache):
    if board.path not in _drc_cache:
        _drc_cache[board.path] = run_drc(board.path, cfg["kicad_cli"])
    return _drc_cache[board.path]


def fmt(items, n=25):
    lines = [str(i) for i in items[:n]]
    if len(items) > n:
        lines.append(f"... and {len(items) - n} more")
    return "\n".join(lines)
```

---

## `test_native_drc.py`

```python
import pytest

from conftest import fmt


def _violations(report, vtype):
    return [v for v in report.get("violations", []) if v["type"] == vtype]


@pytest.mark.parametrize("vtype", [
    "annular_width", "copper_edge_clearance", "drill_out_of_range",
    "microvia_drill_out_of_range", "hole_clearance", "hole_to_hole",
])
def test_no_blocking_violation(drc_report, cfg, vtype):
    if vtype not in cfg["blocking_violation_types"]:
        pytest.skip("not blocking in this cycle")
    found = _violations(drc_report, vtype)
    msgs = [f'{v["description"]} @ '
            f'{[(i["pos"]["x"], i["pos"]["y"]) for i in v.get("items", [])]}'
            for v in found]
    assert not found, f"{len(found)} '{vtype}' violation(s):\n{fmt(msgs)}"


def test_drc_report_is_kicad8(drc_report):
    ver = drc_report.get("kicad_version", "")
    assert ver.startswith("8."), f"Expected KiCad 8 DRC, got '{ver}'"


def test_no_rule_exclusions_hide_blocking(board, cfg):
    """Exclusions in .kicad_pro must not silence blocking checks."""
    excl = (board.project.get("board", {}).get("design_settings", {})
            .get("drc_exclusions", []))
    bad = [e for e in excl
           if any(e.startswith(t + "|") for t in cfg["blocking_violation_types"])]
    assert not bad, f"Blocking violations excluded:\n{fmt(bad)}"
```

---

## `test_annular_ring.py`

```python
from conftest import fmt

EPS = 1e-4


def test_project_rule_floor(board, cfg):
    rule = board.rules.get("min_via_annular_width")
    need = cfg["limits"]["annular_ring"]["via_min_mm"]
    assert rule is not None, "min_via_annular_width not set in .kicad_pro"
    assert rule + EPS >= need, (
        f"Project rule min_via_annular_width={rule} mm is looser than "
        f"fab minimum {need} mm")


def test_via_annular_rings(board, cfg):
    need = cfg["limits"]["annular_ring"]["via_min_mm"]
    bad = [f"via @({h.x:.3f},{h.y:.3f}) ring={h.annular_ring:.3f}"
           for h in board.holes()
           if h.kind == "via" and h.annular_ring + EPS < need]
    assert not bad, f"Vias below {need} mm annular ring:\n{fmt(bad)}"


def test_pth_pad_annular_rings(board, cfg):
    need = cfg["limits"]["annular_ring"]["pth_pad_min_mm"]
    bad = [f"{h.ref} ring={h.annular_ring:.3f} "
           f"(pad {h.copper_w:.3f}, drill {h.drill:.3f})"
           for h in board.holes()
           if h.kind == "pth" and h.annular_ring + EPS < need]
    assert not bad, f"PTH pads below {need} mm annular ring:\n{fmt(bad)}"


def test_no_drill_exceeds_copper(board):
    bad = [f"{h.ref} @({h.x:.2f},{h.y:.2f})"
           for h in board.holes() if h.drill >= h.copper_w]
    assert not bad, f"Drill >= copper (zero/negative ring):\n{fmt(bad)}"
```

---

## `test_edge_clearance.py`

```python
from conftest import fmt

EPS = 1e-4


def test_project_rule_floor(board, cfg):
    rule = board.rules.get("min_copper_edge_clearance")
    need = cfg["limits"]["copper_to_edge"]["min_mm"]
    assert rule is not None, "min_copper_edge_clearance not set in .kicad_pro"
    assert rule + EPS >= need, (
        f"Project rule min_copper_edge_clearance={rule} mm < fab {need} mm")


def test_board_has_closed_outline(drc_report):
    bad = [v["description"] for v in drc_report.get("violations", [])
           if v["type"] in ("invalid_outline", "malformed_courtyard")
           and "outline" in v["description"].lower()]
    assert not bad, ("Invalid Edge.Cuts outline. Edge clearance results "
                     f"are unreliable:\n{fmt(bad)}")


def test_no_copper_edge_violations(drc_report):
    found = [v for v in drc_report.get("violations", [])
             if v["type"] == "copper_edge_clearance"]
    msgs = [v["description"] for v in found]
    assert not found, f"{len(found)} copper-to-edge violation(s):\n{fmt(msgs)}"


def test_custom_rules_do_not_relax_edge_clearance(board, cfg):
    """Custom .kicad_dru rules must not drop edge_clearance below the floor."""
    import re
    dru = board.path.with_suffix(".kicad_dru")
    if not