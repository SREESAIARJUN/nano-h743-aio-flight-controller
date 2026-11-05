# M.2 NVMe PCIe Gen2 x1 Routing and Return-Path Design: Radxa CM4-Form-Factor Carrier (Cycle 1)

## 0. Scope and Assumptions (verify before layout freeze)

| Item | Assumption | Action |
|---|---|---|
| Module | CM4-footprint module (Radxa CM3/CM-series or RPi CM4 pin-compatible), PCIe Gen2 x1, 5 GT/s | Confirm against the Radxa module datasheet and carrier reference schematic |
| Host TX AC-coupling caps | Located **on the module** (true for RPi CM4) | **Must be verified for the Radxa module.** If absent, add 0201/0402 100–220 nF caps on the carrier PET pair |
| Device TX AC caps | On the SSD (required by the M.2 spec) | None needed on the carrier PER pair |
| REFCLK | 100 MHz HCSL, sourced by the module | Route as a differential pair. No AC caps and no termination unless the module datasheet says otherwise |
| Connector | M.2 M-key, 2242/2280, single lane 0 | Lanes 1–3 are no-connect |
| Fab | 4-layer, 1.6 mm, JLC04161H-7628-class stackup | Re-run impedance with the actual fab stackup |

## 1. Stackup and Layer Assignment

| Layer | Use | Notes |
|---|---|---|
| L1 (Top) | PCIe TX/RX/REFCLK pairs, M.2 connector | Microstrip referenced to L2 |
| L2 | **Solid GND**, no splits under the PCIe corridor | Primary return plane |
| L3 | Power (3V3_M2, etc.) | Never used as a reference for PCIe |
| L4 (Bottom) | Low-speed signals; GND pour | Avoid PCIe here if possible |

**Goal: route all PCIe on L1 with zero layer changes.** On the CM4 form factor, the Hirose DF40 connectors and the M.2 slot are both top-side, so this is achievable.

## 2. Differential Pair Parameters

| Parameter | Value |
|---|---|
| Differential impedance | **90 Ω ±10%** (CM4 guideline; also covers the 85 Ω M.2 spec within tolerance) |
| Single-ended impedance | ~50 Ω (reference) |
| Starting geometry (L1 over ~0.21 mm 7628 prepreg, Er≈4.4, 1 oz) | **W ≈ 0.18 mm, S ≈ 0.20 mm**. Confirm in the fab impedance calculator and order impedance control |
| Intra-pair skew (P/N) | **≤ 0.13 mm (5 mil)** |
| Skew compensation | Small serpentine bumps within 0.5–1 mm of the mismatch source, not lumped at one end |
| Inter-pair matching (TX vs RX) | Not required (independent lanes) |
| REFCLK vs data | Not required. Keep REFCLK P/N skew ≤ 0.13 mm |
| Max length (module pin → M.2 pin) | Target **< 75 mm**, hard limit ~150 mm on FR-4 |
| Gen2 loss budget | Carrier < ~3 dB at 2.5 GHz (FR-4 ≈ 0.08–0.1 dB/mm at 2.5 GHz). 75 mm ≈ 0.7 dB, which leaves ample margin |
| Pair-to-pair spacing (TX↔RX, data↔REFCLK) | **≥ 4W** (≥ 0.75 mm); ≥ 5W where parallel run > 10 mm |
| Pair to other signals | ≥ 5W, or **≥ 1 mm to clocks/switchers** |
| Pair to plane/pour edge | ≥ 3W (≥ 0.55 mm) from the coplanar GND pour edge |
| Bends | 45° or arcs only, no 90° corners. Keep P/N symmetric |
| Pad entry | Symmetric fan-out at the DF40 and M.2 pins. Uncouple ≤ 2.5 mm total per pair |
| Polarity | P/N swap is allowed (PCIe auto-polarity), but avoid it if routing allows. **TX↔RX must cross correctly** |

## 3. Pin Mapping (Lane 0)

| Module signal | M.2 M-key pin | Note |
|---|---|---|
| PCIE_TX_P / N (host TX) | **49 PETp0 / 47 PETn0** (device RX) | AC caps on host side (verify) |
| PCIE_RX_P / N (host RX) | **43 PERp0 / 41 PERn0** (device TX) | Caps on the SSD |
| PCIE_CLK_P / N | 55 REFCLKp / 53 REFCLKn | HCSL |
| PCIE_nRST | 50 PERST# | 3.3 V, place an RC/pull-up per the module guide |
| PCIE_CLK_nREQ | 52 CLKREQ# | Open-drain, 3.3 V pull-up |
| — | 54 PEWAKE# | Optional, pull-up |
| — | 69 PEDET | NC or GND per M.2 SSD convention |
| 3V3 (≥ 2.5 A rail) | 2, 4, 12, 14, 16, 18, 70, 72, 74 | Bulk 47 µF + 10 µF + 4× 100 nF near the connector |

## 4. Return-Path and Via-Stitching Design

### 4.1 Reference continuity
- L2 GND is **unbroken** under the full corridor, extended ≥ 3W beyond each side of the outermost pair.
- No L2 slots, antipads, or via voids crossing under the pairs. Keep other nets' vias ≥ 0.5 mm away from the corridor.
- Do not route PCIe across L3 power-split edges. L3 is not a reference, but it can affect L2 if L2 has voids.

### 4.2 Coplanar GND guard on L1 (optional but recommended)
- If you use a L1 GND pour alongside the pairs, keep ≥ 3W clearance to the pair. Recalculate impedance as a coplanar waveguide only if the gap is < 3W.
- Stitch the pour edges to L2 with a **via fence**:
  - Via: 0.3 mm drill / 0.5–0.6 mm pad
  - Pitch: **≤ 2.5 mm** (≈ λ/20 at the 5th harmonic of the 2.5 GHz Nyquist frequency is ~3 mm in FR-4; 2.5 mm adds margin)
  - Offset: 0.5–0.8 mm from the pour edge, both sides of the corridor
- Remove floating GND slivers. Any copper island < 3 mm wide must be stitched at both ends or deleted.

### 4.3 Connector-region return vias
- **Module (DF40) breakout:** place ≥ 1 GND via within **1 mm** of each pair's pad pair, tying the connector GND pins to L2. Match the GND-pin pattern of the module's high-speed connector.
- **M.2 connector:** place a GND via at each GND pin adjacent to lane 0 (pins 39, 45, 51, 57) within 0.5 mm of the pad. These carry the return current into L2.

### 4.4 If a layer change is unavoidable (Cycle 2 fallback)
- Use the same via geometry for P and N, placed symmetrically.
- Add **2 return GND vias per pair, ≤ 1 mm** from the signal vias (one per side), connecting every reference plane the signal moves between.
- Use a 0.3/0.6 mm via and an antipad ≈ 0.9–1.0 mm on planes. Backdrill is not needed for Gen2 on 1.6 mm.
- Limit to **≤ 2 vias per net**.

### 4.5 Board-edge and enclosure
- Keep pairs ≥ 2 mm from the board edge.
- Run a GND via stitch along the board perimeter at ≤ 5 mm pitch (EMI).

## 5. Component Placement Rules
- If carrier AC caps are required: use 0201 or 0402, placed symmetrically and close to the source or sink. Add a void on L2 under the cap pads only if the fab stackup simulation shows > 10% impedance dip. With 0.21 mm prepreg and 0402 pads, a void is usually beneficial, and reference to L3 then needs L3 = GND locally.
- No test points on the data pairs. If needed, use in-line 0.5 mm pads, symmetric, ≤ 1 per net.
- Place the M.2 standoff/screw and its GND connection close to the pair exit (chassis return).
- Do not route clocks, SDIO, Ethernet magnetics, or DC/DC switching nodes within 2 mm of the corridor. Keep the 3V3 buck inductor ≥ 5 mm away.

## 6. Cycle 1 DRC / Constraint Setup (KiCad/Altium)

```
NetClass PCIe_Diff:
  Nets: PCIE_TX_P/N, PCIE_RX_P/N, PCIE_CLK_P/N
  Width: 0.18 mm   DiffGap: 0.20 mm   (update from fab calc)
  Clearance: 0.55 mm (to other nets)
  Max uncoupled: 2.5 mm
  Max skew (intra-pair): 0.13 mm
  Max length: 75 mm (warn), 150 mm (error)
  Vias allowed: 0 (warn if >0)
Rule: Pair-to-pair clearance PCIe_Diff↔PCIe_Diff = 0.75 mm
```

## 7. Cycle 1 Exit Checklist
- [ ] Module datasheet confirms TX AC caps on the module, the REFCLK type, and the PERST#/CLKREQ# voltage
- [ ] Fab impedance calculation returns 90 Ω ±10% with the chosen W/S. Impedance control is ordered
- [ ] All PCIe on L1, continuous L2 GND, zero vias
- [ ] Intra-pair skew ≤ 0.13 mm. Length report saved
- [ ] TX/RX crossover verified against the M.2 pinout (host TX → pins 47/49)
- [ ] GND via within 1 mm of each connector pair breakout. Guard fence ≤ 2.5 mm pitch
- [ ] 3V3_M2 rail rated ≥ 2.5 A with bulk capacitance near the connector

## 8. Open Items for Cycle 2
1. Run a 2D field-solver check, and optionally an S-parameter check, of the DF40 breakout plus the M.2 pad transition (insertion loss < 1.5 dB and return loss < −10 dB up to 2.5 GHz).
2. Validate on hardware: `lspci -vv` should show the link at 5 GT/s x1, and AER should report no correctable errors under an `fio` stress run.
3. Evaluate a TDR coupon on the panel for impedance verification.
4. Assess the effect of an M.2 heatsink or thermal pad on the nearby GND reference and EMI.