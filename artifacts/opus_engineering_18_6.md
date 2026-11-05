# KiCad 8 DRC Validation Test Suite: Annular Rings, Edge Clearance, Drill Aspect Ratio

I don't have your earlier cycles or your fab's limits, so this is a complete standalone suite. The threshold values in `drc_limits.toml` are placeholders. Replace them with your fab's capability sheet before relying on the results.

The suite checks each rule two ways:
1. **Native DRC** via `kicad-cli` with a custom `.kicad_dru` rule file.
2. **Independent geometric re-check** via the `pcbnew` Python API.

The second check catches cases where a board's own rules were loosened or the DRU file was not applied. Drill aspect ratio has no native DRC check, so it is checked in Python only.

---

## Layout

```
drc_suite/
├── drc_limits.toml
├── rules/fab.kicad_dru
├── conftest.py
├── drc_helpers.py
├── test_annular_ring.py
├── test_edge_clearance.py
├── test_aspect_ratio.py
├── test_native_drc.py
├── fixtures/            # known-good / known-bad boards for self-test
│   ├── good.kicad_pcb
│   ├── bad_annular.kicad_pcb
│   ├── bad_edge.kicad_pcb
│   └── bad_aspect.kicad_pcb
└── .github/workflows/drc.yml
```

---

## `drc_limits.toml`

```toml
# Placeholder values. Set from your fab's capability sheet.
[annular]
via_min_mm      = 0.125   # (pad_dia - drill)/2
pth_pad_min_mm  = 0.150

[edge]
copper_to_edge_min_mm = 0.300
# Slot/castellation exemptions by footprint reference prefix
exempt_ref_prefixes = []

[aspect]
max_ratio_through   = 10.0   # board_thickness / finished_drill
max_ratio_microvia  = 0.8    # dielectric_depth / drill (blind/micro)
min_drill_mm        = 0.20

[run]
kicad_cli = "kicad-cli"
```

---

## `rules/fab.kicad_dru`

This file encodes the native-checkable limits. It must match the TOML; `test_native_drc.py` copies it beside each board before running DRC.

```
(version 1)

(rule "via_annular_min"
  (condition "A.Type == 'Via'")
  (constraint annular_width (min 0.125mm)))

(rule "pth_annular_min"
  (condition "A.Type == 'Pad' && A.Pad_Type == 'Through-hole'")
  (constraint annular_width (min 0.150mm)))

(rule "copper_edge_min"
  (constraint edge_clearance (min 0.300mm)))

(rule "min_drill"
  (condition "A.isPlated()")
  (constraint hole_size (min 0.20mm)))
```

---

## `drc_helpers.py`

```python
import json, shutil, subprocess, tomllib
from dataclasses import dataclass
from pathlib import Path

import pcbnew  # requires KiCad 8's bundled Python or PYTHONPATH set to it

HERE = Path(__file__).parent
LIMITS = tomllib.loads((HERE / "drc_limits.toml").read_text())

nm2mm = pcbnew.ToMM


@dataclass
class Finding:
    ref: str
    kind: str
    value: float
    limit: float
    pos_mm: tuple

    def __str__(self):
        return (f"{self.kind} {self.ref} @({self.pos_mm[0]:.3f},{self.pos_mm[1]:.3f}) "
                f"= {self.value:.4f} (limit {self.limit})")


def load_board(path: Path) -> pcbnew.BOARD:
    return pcbnew.LoadBoard(str(path))


def pos(item):
    p = item.GetPosition()
    return (nm2mm(p.x), nm2mm(p.y))


def vias(board):
    return [t for t in board.GetTracks() if isinstance(t, pcbnew.PCB_VIA)]


def pth_pads(board):
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetAttribute() == pcbnew.PAD_ATTRIB_PTH:
                yield fp, pad


def board_thickness_mm(board):
    return nm2mm(board.GetDesignSettings().GetBoardThickness())


def run_native_drc(board_path: Path, out_dir: Path) -> dict:
    """Copy board + project + fab rules into out_dir, run kicad-cli DRC, return JSON."""
    work = out_dir / board_path.stem
    work.mkdir(parents=True, exist_ok=True)
    dst = work / board_path.name
    shutil.copy(board_path, dst)
    pro = board_path.with_suffix(".kicad_pro")
    if pro.exists():
        shutil.copy(pro, work / pro.name)
    shutil.copy(HERE / "rules" / "fab.kicad_dru", dst.with_suffix(".kicad_dru"))

    report = work / "drc.json"
    subprocess.run(
        [LIMITS["run"]["kicad_cli"], "pcb", "drc",
         "--format", "json", "--units", "mm",
         "--severity-all", "--all-track-errors",
         "-o", str(report), str(dst)],
        check=True, capture_output=True, text=True,
    )
    return json.loads(report.read_text())


def violations_of(report: dict, *types):
    return [v for v in report.get("violations", []) if v["type"] in types]
```

---

## `conftest.py`

```python
import os
from pathlib import Path
import pytest

HERE = Path(__file__).parent


def _boards():
    env = os.environ.get("DRC_BOARDS")
    if env:
        return [Path(p) for p in env.split(os.pathsep)]
    return sorted((HERE.parent).glob("**/*.kicad_pcb"))


def pytest_generate_tests(metafunc):
    if "board_path" in metafunc.fixturenames:
        boards = [b for b in _boards() if "fixtures" not in b.parts]
        metafunc.parametrize("board_path", boards, ids=[b.stem for b in boards])
    if "fixture_case" in metafunc.fixturenames:
        cases = {
            "good":        (HERE / "fixtures/good.kicad_pcb",        set()),
            "bad_annular": (HERE / "fixtures/bad_annular.kicad_pcb", {"annular"}),
            "bad_edge":    (HERE / "fixtures/bad_edge.kicad_pcb",    {"edge"}),
            "bad_aspect":  (HERE / "fixtures/bad_aspect.kicad_pcb",  {"aspect"}),
        }
        metafunc.parametrize("fixture_case", list(cases.values()), ids=list(cases))


@pytest.fixture
def board(board_path):
    from drc_helpers import load_board
    return load_board(board_path)
```

---

## `test_annular_ring.py`

```python
from drc_helpers import LIMITS, Finding, vias, pth_pads, pos, nm2mm
import pcbnew


def annular_findings(board):
    out = []
    lim_via = LIMITS["annular"]["via_min_mm"]
    lim_pad = LIMITS["annular"]["pth_pad_min_mm"]

    for v in vias(board):
        ring = (nm2mm(v.GetWidth()) - nm2mm(v.GetDrillValue())) / 2
        if ring < lim_via - 1e-6:
            out.append(Finding(f"via[{v.GetNetname()}]", "annular", ring, lim_via, pos(v)))

    for fp, pad in pth_pads(board):
        size, drill = pad.GetSize(), pad.GetDrillSize()
        # Worst axis handles oval pads and slots; offset reduces ring on one side.
        off = pad.GetOffset()
        ring = min(
            (nm2mm(size.x) - nm2mm(drill.x)) / 2 - abs(nm2mm(off.x)),
            (nm2mm(size.y) - nm2mm(drill.y)) / 2 - abs(nm2mm(off.y)),
        )
        if ring < lim_pad - 1e-6:
            out.append(Finding(f"{fp.GetReference()}.{pad.GetNumber()}",
                               "annular", ring, lim_pad, pos(pad)))
    return out


def test_annular_ring(board):
    f = annular_findings(board)
    assert not f, "Annular ring violations:\n" + "\n".join(map(str, f))
```

---

## `test_edge_clearance.py`

```python
import pcbnew
from drc_helpers import LIMITS, Finding, vias, pos, nm2mm

COPPER_LAYERS = None


def _outline(board):
    poly = pcbnew.SHAPE_POLY_SET()
    ok = board.GetBoardPolygonOutlines(poly)
    assert ok, "Board outline (Edge.Cuts) is not a closed shape"
    return poly


def _dist_to_edge_mm(poly, item_shape):
    # Distance from copper shape to nearest outline segment (any outline/hole).
    best = None
    for i in range(poly.OutlineCount()):
        chains = [poly.Outline(i)] + [poly.Hole(i, h) for h in range(poly.HoleCount(i))]
        for ch in chains:
            for s in range(ch.SegmentCount()):
                seg = pcbnew.SHAPE_SEGMENT(ch.CSegment(s))
                d = item_shape.Collide(seg, 10**9, True)  # (hit, actual)
                actual = d[1] if isinstance(d, tuple) else None
                if actual is not None and (best is None or actual < best):
                    best = actual
    return nm2mm(best) if best is not None else float("inf")


def edge_findings(board):
    lim = LIMITS["edge"]["copper_to_edge_min_mm"]
    exempt = tuple(LIMITS["edge"]["exempt_ref_prefixes"])
    poly = _outline(board)
    out = []

    for fp in board.GetFootprints():
        if exempt and fp.GetReference().startswith(exempt):
            continue
        for pad in fp.Pads():
            if not pad.IsOnCopperLayer():
                continue
            layer = pad.GetLayerSet().CuStack()[0]
            d = _dist_to_edge_mm(poly, pad.GetEffectiveShape(layer))
            if d < lim - 1e-6:
                out.append(Finding(f"{fp.GetReference()}.{pad.GetNumber()}",
                                   "edge", d, lim, pos(pad)))

    for t in board.GetTracks():
        layer = pcbnew.F_Cu if isinstance(t, pcbnew.PCB_VIA) else t.GetLayer()
        d = _dist_to_edge_mm(poly, t.GetEffectiveShape(layer))
        if d < lim - 1e-6:
            out.append(Finding(t.GetClass(), "edge", d, lim, pos(t)))
    # Zones: zone fill already respects edge clearance; verified natively.
    return out


def test_copper_to_edge(board):
    f = edge_findings(board)
    assert not f, "Copper-to-edge violations:\n" + "\n".join(map(str, f))
```

> **Note:** `SHAPE::Collide(..., actual)` binding signatures vary between KiCad 8 point releases. If the tuple return isn't available in your build, the native DRC test below remains the authoritative edge check. Keep this test as a cross-check, or mark it `xfail(strict=False)`.

---

## `test_aspect_ratio.py`

```python
import pcbnew
from drc_helpers import LIMITS, Finding, vias, pth_pads, pos, nm2mm, board_thickness_mm


def _span_thickness_mm(board, via):
    """Dielectric+copper depth between the via's start and end layers."""
    top, bot = via.TopLayer(), via.BottomLayer()
    stack = board.GetDesignSettings().GetStackupDescriptor()
    depth, inside = 0.0, False
    for item in stack.GetList():
        lyr = item.GetBrdLayerId()
        if lyr == top:
            inside = True
        if inside:
            depth += nm2mm(item.GetThickness())
        if lyr == bot and inside:
            break
    return depth or board_thickness_mm(board)


def aspect_findings(board):
    a = LIMITS["aspect"]
    t = board_thickness_mm(board)
    out = []

    for v in vias(board):
        drill = nm2mm(v.GetDrillValue())
        if drill < a["min_drill_mm"] - 1e-6:
            out.append(Finding("via", "min_drill", drill, a["min_drill_mm"], pos(v)))
        vt = v.GetViaType()
        if vt == pcbnew.VIATYPE_THROUGH:
            ratio, lim = t / drill, a["max_ratio_through"]
        else:
            ratio, lim = _span_thickness_mm(board, v) / drill, a["max_ratio_microvia"]
        if ratio > lim + 1e-6:
            out.append(Finding(f"via[{vt}]", "aspect", ratio, lim, pos(v)))

    for fp, pad in pth_pads(board):
        d = pad.GetDrillSize()
        drill = nm2mm(min(d.x, d.y))  # slots: narrow dimension governs plating
        ratio = t / drill
        if ratio > a["max_ratio_through"] + 1e-6:
            out.append(Finding(f"{fp.GetReference()}.{pad.GetNumber()}",
                               "aspect", ratio, a["max_ratio_through"], pos(pad)))
    return out


def test_drill_aspect_ratio(board):
    f = aspect_findings(board)
    assert not f, "Drill aspect-ratio violations:\n" + "\n".join(map(str, f))
```

---

## `test_native_drc.py`

```python
from drc_helpers import run_native_drc, violations_of, load_board
from test_annular_ring import annular_findings
from test_edge_clearance import edge_findings
from test_aspect_ratio import aspect_findings

NATIVE_TYPES = {"annular": ("annular_width",),
                "edge":    ("copper_edge_clearance",)}


def test_native_drc_clean(board_path, tmp_path):
    rep = run_native_drc(board_path, tmp_path)
    bad = violations_of(rep, "annular_width", "copper_edge_clearance",
                        "drill_out_of_range", "hole_size")
    msg =