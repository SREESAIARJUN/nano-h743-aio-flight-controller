# M.2 NVMe (PCIe Gen2 x1) Routing and Return-Path Spec: Radxa CM4 Carrier, Cycle 5

**Scope:** PCIe Gen2 x1 (5 GT/s, Nyquist 2.5 GHz) from the CM4 board-to-board connector to an M.2 M-key socket (2230/2242/2280).

**Assumptions:** I don't have the Cycle 1–4 outputs, so this spec is self-contained. The numbers marked *(verify)* must be checked against the Radxa CM4 datasheet and your fab's impedance calculator before release.

---

## 1. Stackup (recommended 4-layer, 1.6 mm)

| Layer | Use | Notes |
|---|---|---|
| L1 | PCIe pairs, REFCLK, components | Microstrip over L2 |
| L2 | **Solid GND** | No splits or slots under any PCIe or REFCLK route |
| L3 | Power (3V3 for M.2, others) | Pour 3V3_M2 locally |
| L4 | Low-speed signals, GND pour | Avoid routing PCIe here |

- **Prepreg L1–L2:** use thin prepreg (~0.1 mm, e.g., 3313 class) rather than 7628 (~0.21 mm). Thin prepreg gives tighter coupling to GND, narrower pairs, and less crosstalk.
- **Rule:** route all PCIe on L1, single layer, end-to-end where possible. This gives zero layer transitions.

## 2. Differential Pair Parameters

| Parameter | Data lanes (TX0/RX0) | REFCLK (100 MHz HCSL) |
|---|---|---|
| Differential impedance | **85 Ω ±10%** (90 Ω ±10% acceptable if the module guide specifies it) *(verify)* | 85–100 Ω ±10% |
| Single-ended impedance | ~45–50 Ω | ~50 Ω |
| Starting geometry (0.1 mm prepreg, Er≈4.1, 1 oz finished) | W ≈ 0.13 mm, S ≈ 0.15 mm *(run calculator)* | Same as data |
| Intra-pair skew (P vs N) | **≤ 0.13 mm (5 mil)** | ≤ 0.13 mm |
| Inter-pair matching (TX vs RX) | Not required | REFCLK not matched to data |
| Max carrier route length | Target ≤ 100 mm (hard limit ~150 mm, Gen2 budget) | ≤ 150 mm |
| Pair-to-pair spacing (TX↔RX) | **≥ 5W** (≥ 0.65 mm) | ≥ 5W from data pairs |
| Pair to other signals | ≥ 5W; ≥ 1.5 mm from switching regulators and inductors | Same |
| Distance to plane edge / pour edge | Reference plane extends ≥ 3W beyond pair on both sides | Same |
| Bends | 45° or arcs only; no 90° corners | Same |
| Skew compensation | Small serpentine (amplitude ≤ 2S, segment ≥ 3W), placed **within 5 mm of the mismatch source** (pin fanout or bend) | Same |
| Neck-down at pins | Allowed ≤ 2.5 mm length at connector fanout | Same |

**Intra-pair coupling:** keep pairs coupled continuously. Split only at pads, and keep split segments symmetric.

## 3. Signal Mapping and AC Coupling

| Net | Host (CM4) | M.2 M-key pin | AC cap on carrier? |
|---|---|---|---|
| Host TX → card RX | PCIe_TX_P/N | PERp0/PERn0 (43/41) | **No** if the CM4 module carries TX caps *(verify in Radxa datasheet; if absent, add 100 nF 0201/0402 near the host side)* |
| Card TX → host RX | PCIe_RX_P/N | PETp0/PETn0 (49/47) | No. NVMe cards carry their TX caps on-card |
| REFCLK | PCIe_CLK_P/N | REFCLKp/n (55/53) | **No caps** (HCSL, DC-coupled) |
| PERST# | PCIe_nRST | 50 | Series 0 Ω option, 3V3 domain |
| CLKREQ# | PCIe_CLKREQ | 52 | Open-drain; pull-up 10 kΩ to 3V3 (check host side for existing pull-up) |
| PEWAKE# | optional | 54 | 10 kΩ pull-up, or NC |

- Confirm direction naming against the M.2 spec table during schematic review. The rule is: host TX lands on the card's receiver.
- P/N swap is allowed (PCIe lane polarity inversion). Use it to avoid crossovers rather than adding vias.
- If AC caps are needed:
  - Place both caps symmetric, side by side.
  - Void L2 under the cap pads (cutout = pad + ~0.1 mm) to cancel pad capacitance. Use this cutout **only** if L3 then serves as the reference for that small region; otherwise skip it.

## 4. Return Path and Via Stitching

### 4.1 Layer transitions (avoid; if unavoidable)
- **Max 2 vias per signal** across the whole link.
- Signal vias: 0.2 mm drill / 0.45 mm pad. Antipad ≈ 0.75–0.85 mm, as one oblong antipad spanning both P/N vias.
- **Each pair transition needs 2 GND return vias** (4 preferred), placed symmetrically:
  - ≤ 0.65 mm center-to-center from the signal vias
  - never > 1.0 mm
- If the reference changes GND→3V3 (e.g., L2→L3 reference), add a **stitching cap** (10 nF + 100 nF, 0402) within 2 mm of the transition. GND→GND transitions need GND vias only.
- Backdrill is not needed at 5 GT/s on 1.6 mm. Still route to minimize stubs: L1→L4 transitions are preferred over L1→L3.

### 4.2 Pair-flanking stitching
- GND pour on L1 alongside pairs, kept **≥ 3W from the pair edge** so it doesn't detune impedance. If you can't hold 3W, remove the pour.
- Stitch the L1 pour to L2 with vias at **≤ 2.5 mm pitch** along the route. This is ≤ λ/10 at the 5th harmonic, ~12.5 GHz in FR4.
- No floating copper islands: every pour fragment needs ≥ 2 vias, or delete it.

### 4.3 Connector regions
- **CM4 B2B connector:** place a GND via adjacent to each GND pin flanking the PCIe pins, within 0.5 mm of the pad.
- **M.2 socket:**
  - Stitch every GND pin (pins 39, 45, 51, 57 around the PCIe group) directly to L2 with its own via, ≤ 0.5 mm from the pad.
  - Add a GND via row behind the socket (pitch ≤ 2 mm).
  - Tie the standoff/mounting-screw pad to GND with ≥ 4 vias.

### 4.4 Plane integrity
- No L2 splits, slots, or via-antipad "picket fences" under PCIe routes. Watch dense via fields near the B2B connector; enforce web ≥ 0.15 mm between antipads.
- Do not cross pairs over L3 power splits if any route uses L3-side reference.

## 5. Power to the M.2 Socket
- 3V3 at ≥ 2.5 A capability (NVMe peaks ~2–2.5 A on 2280 drives).
- Bulk decoupling: 2× 22 µF plus 4× 100 nF at the socket 3V3 pins.
- Optional load switch with soft-start to limit inrush on the CM4 supply.
- Keep the buck regulator and inductor ≥ 10 mm from the PCIe pairs and the M.2 socket pin field.

## 6. DRC / Constraint Set (enter in EDA)

```
Class PCIE_DATA: Zdiff 85Ω, W=0.13 S=0.15 (calc), skew ≤0.127mm,
                 clearance to other nets 0.65mm, max vias 2, max length 100mm
Class PCIE_REFCLK: Zdiff 85–100Ω, same geometry, skew ≤0.127mm,
                 clearance 0.65mm
Rule: GND via within 0.65mm of any PCIe signal via (manual check / script)
Rule: L1 GND pour clearance to PCIE nets 0.4mm (≥3W)
```

## 7. Cycle 5 Verification Checklist
- [ ] Fab impedance report confirms 85 Ω ±10% on the actual stackup (request impedance coupons).
- [ ] Intra-pair skew report ≤ 0.13 mm for TX, RX, and REFCLK.
- [ ] Every PCIe via has ≥ 2 GND return vias ≤ 0.65 mm away.
- [ ] Continuous L2 under all pairs (visual check with L2-only overlay).
- [ ] AC cap location and count match the Radxa module datasheet (no double caps, no missing caps).
- [ ] CLKREQ# and PERST# pull-ups not duplicated with the module.
- [ ] Bring-up checks:
  - `lspci -vv` shows **LnkSta: Speed 5GT/s, Width x1**.
  - Run a sustained `fio` test (≥10 min); confirm no AER correctable errors (`dmesg | grep -i aer`).
- [ ] Optional: TDR on a coupon; confirm the via-transition dip is within ±10 Ω.

If you share the Cycle 4 stackup, fab house, or layout review notes, I can convert the starting geometry into exact W/S values. I can also flag the specific deltas from the previous cycle.