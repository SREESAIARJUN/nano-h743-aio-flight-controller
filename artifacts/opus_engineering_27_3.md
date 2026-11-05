# KiCad 8 DRC Validation Test Suite

This suite automatically checks KiCad 8 boards for three manufacturability problems:

- **Annular ring** too small
- **Copper too close to the board edge**
- **Drill aspect ratio** too high

**Note on "Cycle 27":** I don't have records of earlier cycles. I built this suite from scratch. If you share the Cycle 26 suite or your fab's thresholds, I can align naming, limits and structure with it.

## How it works

The suite uses two independent checks, so a mistake in one is caught by the other:

1. **KiCad's own DRC.** `kicad-cli pcb drc` runs with a generated `.kicad_dru` rules file, and the JSON report is parsed for violations.
2. **A geometry cross-check.** The `pcbnew` Python API measures each via and pad directly. This layer also handles aspect ratio, which KiCad's DRC cannot check.

It also includes **seeded-defect tests**. These programmatically build small boards with known violations and confirm that each one is detected. This proves the checks can actually fail.

---

## Directory layout

```
drc_suite/
├── drc_limits.yaml
├── conftest.py
├── kicad_drc.py            # kicad-cli wrapper + rules generation
├── geometry_checks.py      # pcbnew-based independent checks
├── fixtures_gen.py         # seeded-defect board generator
├── test_drc_boards.py      # production board tests
├── test_drc_selfcheck.py   # negative/positive control tests
└── .github/workflows/drc.yml
```

---

## `drc_limits.yaml`

```yaml
# Fab capability limits (adjust to your fab's capability sheet)
annular_ring:
  via_min_mm: 0.125
  pth_pad_min_mm: 0.15
copper_edge_clearance_mm: 0.30
aspect_ratio:
  through_max: 10.0      # board_thickness / finished drill
  blind_buried_max: 1.0  # dielectric span / drill (microvia rule)
min_drill_mm: 0.20

# Production boards under test (globs relative to repo root)
boards:
  - "hardware/**/*.kicad_pcb"

# DRC violation types treated as failures for this suite
fail_types:
  - annular_width
  - copper_edge_clearance
  - edge_clearance
  - drill_out_of_range
```

---

## `kicad_drc.py`

```python
"""Thin wrapper around `kicad-cli pcb drc` (KiCad 8.x)."""
from __future__ import annotations
import json, shutil, subprocess
from dataclasses import dataclass, field
from pathlib import Path

KICAD_CLI = shutil.which("kicad-cli") or "kicad-cli"

RULES_TEMPLATE = """(version 1)

(rule "suite_via_annular_ring"
  (constraint annular_width (min {via_ar}mm))
  (condition "A.Type == 'Via'"))

(rule "suite_pth_annular_ring"
  (constraint annular_width (min {pth_ar}mm))
  (condition "A.Type == 'Pad' && A.Pad_Type == 'Through-hole'"))

(rule "suite_copper_edge_clearance"
  (constraint edge_clearance (min {edge}mm)))

(rule "suite_min_drill"
  (constraint hole_size (min {drill}mm)))
"""


@dataclass
class Violation:
    type: str
    severity: str
    description: str
    items: list = field(default_factory=list)

    def __str__(self):
        locs = "; ".join(
            f"{i.get('description','?')} @ ({i['pos']['x']:.3f},{i['pos']['y']:.3f})"
            for i in self.items if "pos" in i
        )
        return f"[{self.severity}] {self.type}: {self.description} -> {locs}"


def render_rules(limits: dict) -> str:
    return RULES_TEMPLATE.format(
        via_ar=limits["annular_ring"]["via_min_mm"],
        pth_ar=limits["annular_ring"]["pth_pad_min_mm"],
        edge=limits["copper_edge_clearance_mm"],
        drill=limits["min_drill_mm"],
    )


def stage_board(board: Path, workdir: Path, limits: dict) -> Path:
    """Copy board (+ project) into an isolated dir and inject suite rules.

    KiCad loads custom rules from <project>.kicad_dru beside the .kicad_pro,
    so we stage a copy rather than touching the user's files.
    """
    workdir.mkdir(parents=True, exist_ok=True)
    dst = workdir / board.name
    shutil.copy2(board, dst)
    pro_src = board.with_suffix(".kicad_pro")
    pro_dst = dst.with_suffix(".kicad_pro")
    if pro_src.exists():
        shutil.copy2(pro_src, pro_dst)
    else:
        pro_dst.write_text("{}\n")  # minimal project so rules get picked up
    dru_dst = dst.with_suffix(".kicad_dru")
    user_dru = board.with_suffix(".kicad_dru")
    suite_rules = render_rules(limits)
    if user_dru.exists():
        # Keep project rules; append suite rules (the later rule wins on ties)
        body = user_dru.read_text().replace("(version 1)", "", 1)
        dru_dst.write_text(suite_rules + "\n" + body)
    else:
        dru_dst.write_text(suite_rules)
    return dst


def run_drc(board: Path, report: Path) -> dict:
    cmd = [
        KICAD_CLI, "pcb", "drc",
        "--format", "json",
        "--units", "mm",
        "--severity-all",
        "--all-track-errors",
        "--output", str(report),
        str(board),
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
    if not report.exists():
        raise RuntimeError(
            f"kicad-cli failed (rc={proc.returncode}):\n{proc.stdout}\n{proc.stderr}"
        )
    return json.loads(report.read_text())


def violations(report: dict, types: set[str]) -> list[Violation]:
    out = []
    for v in report.get("violations", []):
        if v.get("type") in types and v.get("severity") != "exclusion":
            out.append(Violation(v["type"], v.get("severity", "?"),
                                 v.get("description", ""), v.get("items", [])))
    return out
```

---

## `geometry_checks.py`

```python
"""Independent geometric checks using the KiCad 8 pcbnew Python API."""
from __future__ import annotations
from dataclasses import dataclass
import pcbnew

nm2mm = pcbnew.ToMM


@dataclass
class Finding:
    kind: str
    ref: str
    x_mm: float
    y_mm: float
    value: float
    limit: float

    def __str__(self):
        return (f"{self.kind} {self.ref} @({self.x_mm:.3f},{self.y_mm:.3f}): "
                f"{self.value:.3f} vs limit {self.limit:.3f}")


def _pos(item):
    p = item.GetPosition()
    return nm2mm(p.x), nm2mm(p.y)


def board_thickness_mm(board) -> float:
    return nm2mm(board.GetDesignSettings().GetBoardThickness())


def _span_thickness_mm(board, via) -> float:
    """Approximate dielectric span of a blind/buried via from the stackup.
    Falls back to full board thickness if the stackup is unavailable."""
    try:
        stack = board.GetDesignSettings().GetStackupDescriptor()
        top, bot = via.TopLayer(), via.BottomLayer()
        total, inside = 0, False
        for item in stack.GetList():
            lid = item.GetBrdLayerId()
            if lid == top:
                inside = True
                continue
            if lid == bot:
                break
            if inside and item.GetType() == pcbnew.BS_ITEM_TYPE_DIELECTRIC:
                total += item.GetThickness()
        return nm2mm(total) if total > 0 else board_thickness_mm(board)
    except Exception:
        return board_thickness_mm(board)


def check_vias(board, limits) -> list[Finding]:
    f = []
    ar_min = limits["annular_ring"]["via_min_mm"]
    thick = board_thickness_mm(board)
    for t in board.GetTracks():
        if not isinstance(t, pcbnew.PCB_VIA):
            continue
        dia, drill = nm2mm(t.GetWidth()), nm2mm(t.GetDrillValue())
        x, y = _pos(t)
        ring = (dia - drill) / 2
        if ring + 1e-6 < ar_min:
            f.append(Finding("via_annular_ring", "via", x, y, ring, ar_min))
        if drill <= 0:
            continue
        if t.GetViaType() == pcbnew.VIATYPE_THROUGH:
            ratio, lim = thick / drill, limits["aspect_ratio"]["through_max"]
        else:
            ratio = _span_thickness_mm(board, t) / drill
            lim = limits["aspect_ratio"]["blind_buried_max"]
        if ratio > lim + 1e-6:
            f.append(Finding("via_aspect_ratio", "via", x, y, ratio, lim))
    return f


def check_pth_pads(board, limits) -> list[Finding]:
    f = []
    ar_min = limits["annular_ring"]["pth_pad_min_mm"]
    thick = board_thickness_mm(board)
    lim = limits["aspect_ratio"]["through_max"]
    for pad in board.GetPads():
        if pad.GetAttribute() != pcbnew.PAD_ATTRIB_PTH:
            continue
        size, drill = pad.GetSize(), pad.GetDrillSize()
        dmin = nm2mm(min(drill.x, drill.y))
        if dmin <= 0:
            continue
        ring = min(nm2mm(size.x - drill.x), nm2mm(size.y - drill.y)) / 2
        ref = f"{pad.GetParentFootprint().GetReference()}.{pad.GetNumber()}"
        x, y = _pos(pad)
        # Conservative for rect/oval pads; DRC's polygon check is authoritative
        if ring + 1e-6 < ar_min:
            f.append(Finding("pad_annular_ring", ref, x, y, ring, ar_min))
        ratio = thick / dmin
        if ratio > lim + 1e-6:
            f.append(Finding("pad_aspect_ratio", ref, x, y, ratio, lim))
    return f


def run_all(board_path: str, limits: dict) -> list[Finding]:
    board = pcbnew.LoadBoard(board_path)
    return check_vias(board, limits) + check_pth_pads(board, limits)
```

---

## `fixtures_gen.py`

This module builds the seeded-defect boards used by the self-check tests.

```python
"""Generate minimal boards with known defects for self-check tests."""
import pcbnew

MM = pcbnew.FromMM


def _outline(board, w=30, h=20):
    s = pcbnew.PCB_SHAPE(board)
    s.SetShape(pcbnew.SHAPE_T_RECT)
    s.SetStart(pcbnew.VECTOR2I(MM(0), MM(0)))
    s.SetEnd(pcbnew.VECTOR2I(MM(w), MM(h)))
    s.SetLayer(pcbnew.Edge_Cuts)
    s.SetWidth(MM(0.1))
    board.Add(s)


def _via(board, x, y, dia, drill):
    v = pcbnew.PCB_VIA(board)
    v.SetViaType(pcbnew.VIATYPE_THROUGH)
    v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
    v.SetPosition(pcbnew.VECTOR2I(MM(x),