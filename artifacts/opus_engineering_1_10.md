# KiCad 8 DRC Validation Test Suite (Cycle 1)

This suite checks three manufacturability properties of KiCad 8 boards: annular rings, copper-to-edge clearance, and drill aspect ratio. It uses three layers of checking:

| Layer | What it does | Why |
|---|---|---|
| **A. Rule-floor check** | Confirms the board's design rules are at least as strict as the fab limits. | KiCad DRC only enforces the rules stored in the board's project. Someone could loosen them to make DRC pass. |
| **B. Native DRC check** | Runs `kicad-cli pcb drc` and requires zero violations of the relevant types. | Uses KiCad's own geometry engine, including zones, arcs and custom pad shapes. |
| **C. Independent geometric check** | Recomputes annular rings and aspect ratios with the `pcbnew` Python API. | Cross-checks KiCad's DRC. KiCad 8 has no built-in aspect-ratio rule, so this is the only aspect-ratio check. |

A self-test package runs the suite against generated boards with known defects. This confirms the suite actually detects failures.

---

## 1. Repository layout

```
kicad-drc-suite/
├── config/
│   ├── fab_rules.yaml          # Fab capability limits (source of truth)
│   └── waivers.yaml            # Explicit, reviewed exceptions
├── drc_suite/
│   ├── __init__.py
│   ├── config.py               # Loads rules and waivers
│   ├── kicad_cli.py            # kicad-cli DRC runner + JSON parsing
│   ├── geometry.py             # Independent annular-ring / aspect-ratio math
│   └── stackup.py              # Copper-span thickness for blind/buried/micro vias
├── tests/
│   ├── conftest.py             # Board discovery, session-cached DRC results
│   ├── test_annular_ring.py
│   ├── test_copper_edge_clearance.py
│   ├── test_drill_aspect_ratio.py
│   └── selftest/
│       ├── generate_fixtures.py
│       └── test_suite_detects_defects.py
├── pytest.ini
├── Makefile
└── .github/workflows/drc.yml
```

---

## 2. Configuration

### `config/fab_rules.yaml`

```yaml
# Fab capability limits. All lengths in mm.
# These are FLOORS: board design rules may be stricter, never looser.
fab_profile: "Generic 2-6L standard (Cycle 1)"

annular_ring:
  via_min_mm: 0.125          # (via_diameter - drill) / 2
  pth_pad_min_mm: 0.150      # PTH component pads
  skip_custom_pad_shapes: true   # Custom-shape pads are covered by native DRC only in Cycle 1

copper_edge_clearance:
  min_mm: 0.300              # Copper to Edge.Cuts (and board outline cutouts)
  # Slightly lower floor permitted where castellations/edge plating are intentional (waive explicitly)

drill_aspect_ratio:
  through_max: 10.0          # board_thickness / finished_drill
  blind_buried_max: 10.0     # span_thickness / drill
  microvia_max: 0.8          # dielectric_depth / drill (laser vias)
  min_drill_mm: 0.200        # Absolute mechanical drill floor
  use_finished_hole_size: true

drc:
  refill_zones: true
  # kicad-cli JSON violation "type" keys mapped to each check
  violation_types:
    annular_ring: ["annular_width"]
    edge_clearance: ["copper_edge_clearance", "edge_clearance"]
    drill: ["drill_out_of_range", "microvia_drill_out_of_range"]
```

### `config/waivers.yaml`

```yaml
# Each waiver needs: board (filename), check, uuid (item KIID from DRC/pcbnew), reason, approver.
# Waivers without reason/approver are rejected by the loader.
waivers:
  # - board: "power_stage.kicad_pcb"
  #   check: "copper_edge_clearance"
  #   uuid: "3f2b9c1e-...."
  #   reason: "Intentional castellated edge pad, fab confirmed edge plating"
  #   approver: "j.doe"
  #   expires: "2025-12-31"
```

---

## 3. Library code

### `drc_suite/config.py`

```python
from __future__ import annotations
import datetime as _dt
from dataclasses import dataclass, field
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parent.parent
NM_PER_MM = 1_000_000


def nm_to_mm(v: int | float) -> float:
    return float(v) / NM_PER_MM


@dataclass(frozen=True)
class Waiver:
    board: str
    check: str
    uuid: str
    reason: str
    approver: str
    expires: _dt.date | None = None

    def active(self, today: _dt.date | None = None) -> bool:
        today = today or _dt.date.today()
        return self.expires is None or today <= self.expires


@dataclass
class SuiteConfig:
    rules: dict
    waivers: list[Waiver] = field(default_factory=list)

    def is_waived(self, board: str, check: str, uuid: str) -> bool:
        return any(
            w.board == board and w.check == check and w.uuid == uuid and w.active()
            for w in self.waivers
        )


def load_config(
    rules_path: Path = ROOT / "config/fab_rules.yaml",
    waivers_path: Path = ROOT / "config/waivers.yaml",
) -> SuiteConfig:
    rules = yaml.safe_load(rules_path.read_text())
    waivers: list[Waiver] = []
    if waivers_path.exists():
        raw = yaml.safe_load(waivers_path.read_text()) or {}
        for entry in raw.get("waivers") or []:
            missing = [k for k in ("board", "check", "uuid", "reason", "approver") if not entry.get(k)]
            if missing:
                raise ValueError(f"Waiver {entry} missing required fields: {missing}")
            exp = entry.get("expires")
            if isinstance(exp, str):
                exp = _dt.date.fromisoformat(exp)
            waivers.append(Waiver(entry["board"], entry["check"], entry["uuid"],
                                  entry["reason"], entry["approver"], exp))
    return SuiteConfig(rules=rules, waivers=waivers)
```

### `drc_suite/kicad_cli.py`

```python
from __future__ import annotations
import json
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path


class KiCadCliError(RuntimeError):
    pass


@dataclass(frozen=True)
class DrcViolation:
    type: str
    severity: str
    description: str
    items: tuple[dict, ...]

    @property
    def uuids(self) -> tuple[str, ...]:
        return tuple(i.get("uuid", "") for i in self.items)

    def location_str(self) -> str:
        locs = [
            f"({i['pos']['x']:.3f}, {i['pos']['y']:.3f}) {i.get('description', '')}"
            for i in self.items if "pos" in i
        ]
        return "; ".join(locs)


def kicad_cli_path() -> str:
    exe = shutil.which("kicad-cli")
    if not exe:
        raise KiCadCliError("kicad-cli not found on PATH (KiCad 8.0+ required)")
    return exe


def kicad_version() -> str:
    out = subprocess.run([kicad_cli_path(), "version"], capture_output=True, text=True, check=True)
    return out.stdout.strip()


def run_drc(board: Path, out_dir: Path, refill_zones: bool = True) -> list[DrcViolation]:
    """Run KiCad DRC and return parsed violations.

    The board's .kicad_pro must sit next to the .kicad_pcb.
    Without it, KiCad falls back to default rules, which makes the result meaningless.
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    report = out_dir / f"{board.stem}.drc.json"
    cmd = [
        kicad_cli_path(), "pcb", "drc",
        "--format", "json",
        "--units", "mm",
        "--severity-all",
        "--output", str(report),
    ]
    if refill_zones:
        cmd.append("--refill-zones")
    cmd.