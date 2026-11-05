# M.2 NVMe (PCIe Gen2 x1) Routing Specification: Radxa CM4-Format Carrier, Cycle 9

I don't have your project history, so I can't see your earlier cycles, schematic, or chosen fab stackup. This document is a self-contained design baseline. Values marked **[VERIFY]** must be checked against your Radxa module datasheet (CM3/CM3I/CM5 variant) and your fab's impedance calculator before release.

---

## 1. Interface Summary

| Item | Value |
|---|---|
| Link | PCIe Gen2, x1, 5 GT/s (8b/10b) |
| Nyquist frequency | 2.5 GHz. Design for ≥7.5 GHz (3rd harmonic) |
| Host | Radxa CM module via 2× 100-pin Hirose DF40 (CM4 pinout) |
| Device | M.2 M-key socket, 2242/2260/2280 |
| Signals | TX pair, RX pair, REFCLK pair, PERST#, CLKREQ#, PEWAKE# |
| Reference clock | 100 MHz HCSL, from module |

**M.2 M-key pins (lane 0):**
- PERn0/PERp0: 41/43
- PETn0/PETp0: 47/49
- REFCLKn/p: 53/55
- PERST#: 50
- CLKREQ#: 52
- PEWAKE#: 54
- 3.3 V: pins 2, 4, 12, 14, 16, 18, 70, 72, 74

**[VERIFY]** Host TX must land on SSD RX. Cross-check the M.2 spec naming convention against your module's PCIe_TX/RX nets before layout freeze. P/N polarity inversion within a pair is tolerated by link training. A TX↔RX swap is not.

---

## 2. Impedance Targets

| Net class | Target | Tolerance |
|---|---|---|
| PCIe TX/RX pairs | **90 Ω differential** (CM4 design-guide convention, compatible with M.2's 85 Ω) | ±10% |
| REFCLK pair | 90–100 Ω differential (90 Ω acceptable, keeps one rule set) | ±10% |
| Sideband (PERST#, CLKREQ#, WAKE#) | 50 Ω SE (not critical) | n/a |

### Example stackup (4-layer, 1.6 mm, JLC04161H-3313 class)

| Layer | Function |
|---|---|
| L1 | Signal (PCIe routed here) |
| PP | ~0.1 mm prepreg (3313), εr ≈ 4.1 |
| L2 | **Solid GND**, no splits under PCIe |
| Core | ~1.2 mm core |
| L3 | Power (3V3 / 5V pours) |
| PP | ~0.1 mm prepreg |
| L4 | Signal / GND fill |

**Starting geometry (L1 edge-coupled microstrip over L2):**
- Trace width ≈ 0.13 mm (5.1 mil)
- Gap ≈ 0.15 mm (5.9 mil)
- Result ≈ 90 Ω differential

**[VERIFY]** Confirm these with your fab's calculator. If you use a 7628 prepreg stackup (~0.2 mm), the width must grow to roughly 0.20–0.22 mm.

Pour clearance from a pair to coplanar GND must be **≥ 3× the trace width** (≥0.4 mm). This keeps the impedance microstrip-dominated rather than an uncontrolled CPW.

---

## 3. Routing Rules

| Parameter | Rule |
|---|---|
| Intra-pair skew (P vs N) | **≤ 0.127 mm (5 mil)**. Match at the mismatch source using small serpentine bumps (amplitude < 2× gap) |
| Inter-pair skew (TX vs RX) | Not required (independent lanes) |
| REFCLK intra-pair skew | ≤ 0.25 mm |
| Max carrier trace length | ≤ 100 mm per pair (target < 50 mm; place M.2 near the DF40 connector) |
| Pair-to-pair spacing (TX↔RX) | ≥ 5W (≥ 0.65 mm), or a GND guard with vias |
| Pair to other signals | ≥ 5W. Keep away from USB3, HDMI, SDIO, and switch-node nets |
| Pair to board edge | ≥ 1 mm |
| Vias per pair | **0 preferred**, max 2 per pair. Use symmetric via placement |
| Bends | 45° or arc only, no 90°. Alternate bend direction to self-compensate skew |
| Reference plane | Continuous L2 GND for the full route. **No crossing of plane splits or voids** |
| AC-coupling caps | **[VERIFY]** The CM4 form factor puts host-TX caps on the module and the SSD carries its own TX caps, so none are normally needed on the carrier. If your module datasheet says otherwise, use 0402 (or 0201) 220 nF X7R, placed symmetrically with GND anti-pad cutouts under the pads on L2 sized to pad + 0.2 mm |
| Connector breakout | Neck-down allowed within ≤ 2 mm of the DF40/M.2 pads, then return to nominal geometry immediately |
| Test points | None on pairs. If probing is needed, use pads inline on the trace (no stubs) |

---

## 4. Return-Path and Via Stitching Plan

### 4.1 Layer-transition stitching (mandatory wherever a pair changes layer)
- Place **2 GND vias per signal-via pair**, one on each side of the pair.
- Keep them **≤ 1.0 mm (40 mil)** center-to-center from the signal vias.
- These give return current a path between the L2 reference and the new reference plane.
- If the pair transitions L1→L4, the L4 reference is L3 (power). In that case add **100 nF stitching caps (0402) between 3V3 and GND within 2 mm** of the transition, or avoid L4 entirely. **L1-only routing is strongly preferred.**

### 4.2 Signal via geometry (if used)
- Drill 0.2–0.3 mm, pad 0.45–0.55 mm.
- Anti-pad on L2/L3 ≈ 0.9–1.0 mm, shared oval anti-pad around the P/N pair.
- Back-drilling is not needed at Gen2 on 1.6 mm. The stub resonance is far above 7.5 GHz for an L1→L4 transition.

### 4.3 Guard / fence vias along the route
- Wavelength at 7.5 GHz in FR-4 (εeff ≈ 3.3) is about 22 mm. λ/10 ≈ 2.2 mm.
- **Fence pitch: ≤ 2.0 mm** along any coplanar GND pour bordering the PCIe corridor.
- Place fence vias ≥ 0.5 mm from pair copper, on the pour's edge.
- Run a fence row between the TX and RX pairs where they run parallel for > 5 mm.

### 4.4 Connector-region stitching
- **DF40 connector:** place GND vias on every module GND pin adjacent to the PCIe pins, with ≥ 1 via per GND pad and ≤ 0.6 mm via-in-pad or a short neck.
- **M.2 socket:** stitch GND pins 3, 9, 15, 21, 27, 33, 39, 45, 51, 57, 71, 73, 76, plus mounting standoff/screw GND to L2. Put two vias on the GND pins flanking the PCIe lanes (39, 45, 51, 57).
- The M.2 standoff/nut pad connects to GND with ≥ 4 vias.

### 4.5 Plane integrity checks
- No L2 cutouts or slot antipads within 3× the dielectric height (~0.3 mm) of the pair centerline. Generalize to ≥ 1 mm keep-clear.
- Do not route other L2/L3 traces underneath the PCIe corridor.
- Edge stitching around the board perimeter at ≤ 5 mm pitch to suppress cavity resonance between GND and power planes.

---

## 5. Power and Sideband

- **3V3 to M.2:** budget ≥ 2.5 A peak (worst-case NVMe write).
  - Use a dedicated buck or a load switch with soft-start.
  - Bulk capacitance: 2× 22 µF + 4× 100 nF placed at the socket pins.
  - Copper: pour or ≥ 1.0 mm trace, with ≥ 4 vias per layer transition.
- **PERST#:** from module, add a 10 k pull-down option.
  - **[VERIFY]** whether it is open-drain. Route away from the pairs.
- **CLKREQ#:** 10 k pull-up to 3V3, or tie low if the module does not support clock power management.
- **PEWAKE#:** 10 k pull-up, optional.
- **Pin 69 (PEDET) and SUSCLK (pin 68):** leave per the M.2 spec. A 32 kHz SUSCLK is optional, so leave it NC if unavailable.

---

## 6. DRC / Constraint Setup (KiCad / Altium)

```
NetClass PCIe_DP:  width 0.13 mm, diff gap 0.15 mm, clearance 0.40 mm
  Members: PCIE_TX_P/N, PCIE_RX_P/N, PCIE_CLK_P/N
Rules:
  - diff_pair_uncoupled_max   2.0 mm
  - skew_max (intra-pair)     0.127 mm  (CLK: 0.25 mm)
  - length_max                100 mm
  - via_count_max             2
  - layer restriction         L1 (L4 only by exception)
```

---

## 7. Cycle 9 Review Checklist

- [ ] Impedance calculated with the fab's tool and noted on the fab drawing ("90 Ω ±10% diff, L1 ref L2")
- [ ] TX/RX direction cross-checked against the module and M.2 spec
- [ ] AC-cap location confirmed (module vs carrier)
- [ ] Intra-pair skew ≤ 5 mil, verified by a length report
- [ ] Zero plane-split crossings (run a 3D or plane overlay check)
- [ ] Return vias ≤ 1 mm from every signal via
- [ ] Fence vias ≤ 2 mm pitch along pours adjacent to the corridor
- [ ] 3V3 rail sized for 2.5 A, capacitors at the socket
- [ ] Bring-up test: `lspci -vv` shows `LnkSta: Speed 5GT/s, Width x1`, then `fio` sequential read ≈ 380–420 MB/s (the Gen2 x1 ceiling)

If you share your actual stackup, fab, and Radxa module variant, I can tighten the trace geometry to exact numbers and adjust the AC-cap and pinout notes.