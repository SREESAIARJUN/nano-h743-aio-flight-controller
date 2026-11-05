# M.2 NVMe (PCIe Gen2 x1) Routing Specification: Radxa CM4 Expansion Board, Cycle 3

## 0. Assumptions to Confirm Before Layout

| Item | Assumed value | Verify against |
|---|---|---|
| Host link | PCIe Gen2 x1, 5 GT/s, 8b/10b | CM4/SoC datasheet |
| Module connector | 2× 100-pin high-density board-to-board (CM4 form factor) | Radxa CM4 pinout |
| M.2 slot | M-key, 2242/2280, socket 3 | Mechanical drawing |
| Stackup | 4-layer, 1.6 mm, FR-4 (Dk ≈ 4.3–4.5 @ 1 GHz) | Fab stackup sheet |
| Layer use | L1 signal, L2 solid GND, L3 power, L4 signal | Cycle 2 stackup |
| TX AC-coupling caps | Host side on module; device side on SSD (per M.2 spec) | Module schematic |

If the module does **not** have host-TX caps on board, add 2× 100 nF (0201/0402, X7R) on the carrier. See §3.

---

## 1. Net List and M.2 M-key Pin Map (Lane 0)

| Function | Carrier net | M.2 pin | Direction (host view) |
|---|---|---|---|
| Host TX+ / TX− | PCIE_TX_P / PCIE_TX_N | 49 (PERp0) / 47 (PERn0) | Out → SSD RX |
| Host RX+ / RX− | PCIE_RX_P / PCIE_RX_N | 43 (PETp0) / 41 (PETn0) | In ← SSD TX |
| REFCLK+ / REFCLK− | PCIE_CLK_P / PCIE_CLK_N | 55 / 53 | Out |
| PERST# | PCIE_PERST_N | 50 | Out, 3.3 V OD/PP |
| CLKREQ# | PCIE_CLKREQ_N | 52 | Bidirectional, OD, pull-up 10 k to 3.3 V |
| PEWAKE# | PCIE_WAKE_N | 54 | In, OD, pull-up 10 k (optional) |

> The M.2 pin names (PERx/PETx) are from the **SSD's** perspective. The host TX connects to the M.2 **PER** pins. This is the most common respin error, so check it explicitly at schematic review.

PCIe permits P/N polarity inversion per pair, and link training handles it. Still route without inversion. Swap only if it removes vias, and record any swap in the layout notes.

---

## 2. Differential Pair Parameters

### 2.1 Impedance Targets

| Pair | Target Zdiff | Tolerance | Notes |
|---|---|---|---|
| TX, RX | **85 Ω** (90 Ω acceptable) | ±10% | PCIe base board spec is 85 Ω. 90 Ω matches the CM4-class module traces; pick one and hold it for the whole board |
| REFCLK (HCSL) | 85–100 Ω | ±10% | 100 Ω is fine; route as a tight pair |
| Single-ended reference | 42–50 Ω | ±10% | Natural result of the pair geometry |

**Starting geometry** (L1 microstrip over L2 GND, prepreg ≈ 0.21 mm, 1 oz finished copper):

| Target | Trace width W | Gap S | Pair-to-pair / other-net clearance |
|---|---|---|---|
| 85 Ω | ≈ 0.22 mm | ≈ 0.18 mm | ≥ 0.5 mm (≥ 3×S, aim 5W to aggressors) |
| 90 Ω | ≈ 0.20 mm | ≈ 0.20 mm | ≥ 0.5 mm |

These are estimates. Confirm them with the fab's impedance calculator for the exact prepreg and copper, and request controlled-impedance coupons on the panel.

**If L4 is used:** recompute the geometry for the L3/L4 dielectric. Better still, keep all PCIe routing on L1 so the pairs never reference the L3 power plane.

### 2.2 Length and Skew

| Rule | Value |
|---|---|
| Intra-pair skew (P vs N) | ≤ 0.13 mm (5 mil), measured connector-pad to connector-pad |
| Inter-pair matching (TX vs RX vs CLK) | **Not required.** The pairs are independently clocked |
| Max carrier trace length, TX/RX | ≤ 100 mm target, 150 mm hard limit (Gen2 channel budget with module + SSD) |
| REFCLK length | ≤ 150 mm |
| Skew compensation location | Within 5 mm of the mismatch source (bend or pin escape) |
| Serpentine geometry | Amplitude ≤ 2×S, segment length ≥ 3W, 45° corners only |

### 2.3 Routing Rules

1. Use 45° bends or arcs only. No 90° corners.
2. Keep the pair symmetric through pad escapes. Fan out at the module and M.2 pins with equal-length, mirrored stubs.
3. Use **at most 2 vias per signal** end-to-end, and ideally 0 on L1-only routing. Vias are always placed as a pair.
4. Don't route over any split, slot, or void in L2. Keep at least 1 mm from the L2 plane edge, board edge, and mounting holes.
5. Keep 1.5 mm (≥ 5W) from switching nodes, crystals, USB/HDMI pairs, and the M.2 3.3 V regulator inductor.
6. Don't run the TX and RX pairs adjacent and parallel for more than 10 mm. Put a ground-stitched pour between them if they must share a corridor.
7. Remove all test-point stubs on the pairs. If probing is needed, use pads inline on the trace (no stubs).
8. Pull pour copper back ≥ 0.5 mm from the pair edges. This keeps the line a true microstrip, not an uncontrolled coplanar structure.

---

## 3. AC-Coupling Capacitors (only if required on the carrier)

- **Value and package:** 100 nF (75–265 nF allowed), 0201 preferred, 0402 maximum.
- **Location:** within 10 mm of the module connector (transmitter side). Place P and N caps side by side, at the same distance from the connector.
- **Pad treatment:** void L2 GND under the cap pads (pad footprint + 0.1 mm) to reduce the capacitive dip. Re-check by TDR or simulation.
- **Signal path:** keep the pair straight through the caps. No neck-down or neck-out.

---

## 4. High-Speed Return Path and Via Stitching

### 4.1 Design Basis

- Gen2 fundamental (Nyquist) is 2.5 GHz; design for content up to roughly 7.5 GHz (3rd harmonic).
- Wavelength in FR-4 at 7.5 GHz is about 3e8 / (7.5e9 × √4.3) ≈ 19 mm.
- Stitch pitch is set at λ/8 to λ/10, which gives **≤ 2.5 mm**.

### 4.2 Return Vias at Every Layer Transition

| Rule | Value |
|---|---|
| GND return vias per diff-via pair | **2 minimum**, one on each side, symmetric about the pair |
| Distance, return via to signal via (center-to-center) | ≤ 1.0 mm (target 0.7 mm) |
| Signal via | 0.25 mm drill / 0.45 mm pad |
| GND return via | Same size as the signal via, or larger |
| Antipad on L2/L3 | Shared oblong antipad around both signal vias, ≈ 0.25 mm clearance beyond the pads. Don't merge it with the return-via antipads |
| Reference change L2 → L3 (power) | Avoid. If unavoidable, add a 100 nF + 10 nF stitching cap pair between 3V3 and GND within 2 mm of the via |
| Via stubs | Through-hole on 1.6 mm is acceptable at Gen2 when routing L1 → L4. Avoid L1 → L3 transitions, which leave long stubs |

### 4.3 Stitching Along the Route

- **GND guard fence (optional, if L1 GND pour runs alongside the pairs):** vias at **≤ 2.5 mm pitch**, centered ≥ 0.5 mm from the nearest trace edge. The fence must be continuous. A guard trace or pour that isn't stitched behaves as a resonant stub, so either stitch it or remove it.
- **Between TX and RX corridors:** a single stitched GND strip, ≥ 0.6 mm wide, with vias at 2.5 mm pitch.
- **General L1/L4 pour-to-L2 stitching:** grid at ≤ 5 mm across the board. In the PCIe region, tighten to ≤ 2.5 mm.

### 4.4 Connector Return Path

**M.2 socket:**
- Tie each GND pin adjacent to the lane pins (pins 33, 39, 45, 51, 57 and so on) to L2 with its own via, placed ≤ 0.5 mm from the pad.
- Don't daisy-chain GND pins on L1 before dropping to the plane.
- Place 2 GND vias at the M.2 standoff/latch and at the socket shell tabs.

**Module board-to-board connector:**
- Each GND pin flanking the PCIe pins gets a dedicated via to L2, ≤ 0.5 mm from the pad.
- Keep L2 solid under the connector footprint except for the required antipads.

**Under the M.2 signal pads:**
- Optionally void L2 under the PCIe pads only (pad + 0.1 mm), backed by a solid L3 reference.
- Use this only if TDR or simulation shows an impedance dip below −10%. Otherwise leave L2 solid.

### 4.5 Plane Integrity

- L2 must be one continuous GND from the module connector to the M.2 socket under all PCIe routing. No splits, slots, or via-antipad "picket fences" crossing the path.
- If a row of vias (for example, the module connector field) perforates L2, check that the antipads don't merge. Keep ≥ 0.15 mm copper web between adjacent antipads.

---

## 5. REFCLK and Sideband Rules

- **REFCLK:** route as a diff pair (§2) with ≤ 0.13 mm skew and the same stitching rules. Use ≥ 1 mm clearance to TX/RX pairs. Add no series termination unless the host datasheet specifies it.
- **PERST#, CLKREQ#, PEWAKE#:** route as single-ended, non-critical signals, ≥ 0.5 mm from the diff pairs.
  - Place the CLKREQ# pull-up at the host side.
  - Add a 3.3 V-tolerance check if the module I/O is 1.8 V. Insert a level shifter or open-drain buffer if needed.

---

## 6. Power at the M.2 Socket (affects the return path)

- **3.3 V supply:** rated ≥ 2.5 A; NVMe peaks can exceed 2 A during writes.
- **Bulk decoupling:** 2× 22 µF + 1× 10 µF placed at the 3.3 V pins (2, 4, 12, 14, 16, 70, 72, 74).
- **Local decoupling:** 4× 100 nF, one per 3.3 V pin pair, each with its own GND via ≤ 0.5 mm away.
- **Return currents:** keep the regulator loop and its return current out of the region under the PCIe pairs.

---

## 7. Cycle 3 Verification Checklist

| # | Check | Pass criterion |
|---|---|---|
| 1 | Host TX → M.2 PER pins, Host RX ← M.2 PET pins | Matches pin map in §1 |
| 2 | Fab stackup confirmed; impedance coupons ordered | Zdiff within ±10% |
| 3 | Intra-pair skew report | ≤ 0.13 mm on all 3 pairs |
| 4 | Carrier length TX/RX | ≤ 100 mm (150 mm absolute) |
| 5 | Vias per signal | ≤ 2, each with ≥ 2 adjacent GND return vias ≤ 1 mm away |
| 6 | Reference plane | L2 continuous under 100% of each pair route (DRC: no split crossings) |
| 7 | Stitch pitch in PCIe region | ≤ 2.5 mm; no unstitched guard copper |
| 8 | AC caps | Present exactly once per TX direction across module + carrier + SSD |
| 9 | Connector GND pins | Each has its own via ≤ 0.5 mm away |
| 10 | Bring-up | `lspci -vv` reports LnkSta 5 GT/s x1; zero AER corrected errors after a 30 min `fio` stress run |

---

**Open items to resolve before release:**
1. The exact Cycle 3 stackup numbers.
2. Whether the Radxa module carries host-side TX AC caps.
3. Confirmation of the 85 Ω vs 90 Ω target against the module's own trace impedance.

Once the fab stackup is fixed, the trace geometry in §2.1 should be recalculated and locked in the layout DRC constraint set.