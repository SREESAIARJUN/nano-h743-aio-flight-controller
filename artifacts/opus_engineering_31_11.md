# M.2 NVMe (PCIe Gen2 x1) Routing and Return-Path Design: Radxa CM4-Form-Factor Expansion Board, Cycle 31

## 0. Scope and Assumptions

| Item | Assumption (verify against your module) |
|---|---|
| Host | Radxa CM3 (RK3566) or other CM4-pinout module, PCIe 2.1 **x1**, 5 GT/s |
| Interface | 1 TX pair, 1 RX pair, 1 REFCLK pair (100 MHz HCSL) through the CM4 high-density connectors |
| Slot | M.2 M-key, 2242/2280, socket 3 |
| Board | 4-layer, 1.6 mm, FR-4 (Dk ≈ 4.2–4.4), 1 oz outer / 0.5 oz inner |
| Nyquist | 2.5 GHz. Design bandwidth to 5th harmonic of the edge, about 7.5 GHz |

**Check before release:** Confirm whether the module already places the host-TX AC-coupling capacitors on-module. The Pi CM4 does. Check the Radxa CM3 schematic. Never double-cap a lane.

---

## 1. Stackup

| Layer | Function | Note |
|---|---|---|
| L1 | PCIe pairs, M.2 connector, CM connectors | Microstrip over L2 |
| — | Prepreg ~0.21 mm (7628) | |
| L2 | **Solid GND**: no splits under PCIe | Primary reference |
| — | Core ~1.065 mm | |
| L3 | Power (3V3_M2, 5V) plus slow signals | Keep 3V3 pour out of PCIe corridors |
| — | Prepreg ~0.21 mm | |
| L4 | GND pour plus low-speed signals | Secondary reference / stitching |

**Rule:** Route all PCIe on **L1 only**, with zero layer changes if possible. The CM connector and M.2 socket are both top-side SMT, so this is achievable and removes most via return-path problems.

---

## 2. Differential Pair Parameters

### 2.1 Impedance target
- **Target: 85 Ω ±10% differential.** The PCIe CEM/M.2 recommendation is 85 Ω. The CM4 module side is ~90 Ω, and 85–90 Ω is fully acceptable at Gen2.
- Single-ended: ~45–50 Ω.

### 2.2 Starting geometry (L1 over L2, h ≈ 0.21 mm, Dk 4.4, plated 1 oz)

| Target | Trace W | Gap S | Expected Zdiff |
|---|---|---|---|
| 85 Ω | 0.20 mm (7.9 mil) | 0.18 mm (7 mil) | ~85–87 Ω |
| 90 Ω | 0.18 mm (7 mil) | 0.20 mm (7.9 mil) | ~90 Ω |

These are starting values. Re-solve with the fab's impedance calculator (e.g. JLC JLC04161H-7628) and order controlled impedance with a coupon.

### 2.3 Routing rules

| Parameter | Value |
|---|---|
| Intra-pair skew (P vs N) | **≤ 0.13 mm (5 mil)** total; compensate within 2 mm of the mismatch source |
| Inter-pair skew (TX vs RX) | Not required (independent lanes) |
| REFCLK intra-pair skew | ≤ 0.25 mm |
| REFCLK vs data length | No match required (common-clock architecture) |
| Max length, CM connector to M.2 pad | **≤ 100 mm** target (Gen2 budget allows far more; keep short for margin) |
| Pair-to-pair spacing (TX↔RX) | ≥ 4W (≥ 0.8 mm), or GND guard between |
| Pair to other signals | ≥ 5W (≥ 1.0 mm); ≥ 2 mm to switching regulators and USB 3 |
| Pair to plane edge / void | ≥ 3h (≥ 0.65 mm) from any L2 edge |
| Bends | 45° or arcs (R ≥ 3W); no 90° corners |
| Skew serpentine | Amplitude ≤ 2S, segment length ≥ 3W, placed near the mismatch |
| Breakout neck-down | Allowed ≤ 3 mm at connector pins; keep pair coupled |
| Stubs / test points | None on data pairs; use in-line pads only if required |
| Polarity | P/N swap allowed (PCIe auto-polarity), but avoid it for clean bring-up |
| Lane mapping | Host TX → M.2 PERp0/n0 (pins 47/49 are device RX); Host RX ← M.2 PETp0/n0 (pins 41/43); REFCLK → pins 53/55 |

### 2.4 AC coupling
- **Value:** 100 nF (75–265 nF allowed), 0201 preferred (0402 acceptable), X7R/X5R, ≥ 6.3 V.
- **Location:** Host TX only, if not on-module. The device-TX caps are on the SSD itself.
- **Placement:** Symmetric pair, near the M.2 connector or near the source; be consistent.
- **Footprint:** Void L2 under the cap pads (rectangle = pad outline + 0.1 mm), then stitch the void edge (see §3.3). This counters the pad's capacitive impedance dip (~70 Ω otherwise).

### 2.5 Connector pad compensation
- **M.2 socket SMT pads:** Void L2 under the PCIe signal pads only. Optionally keep a reference on L3 if the stackup allows; for 4-layer, void L2 and reference L4 GND.
- **CM connector pins:** Follow the module vendor's breakout. Do not run pairs between connector rows with ground fragmentation.

---

## 3. Return Path and Via Stitching

### 3.1 Wavelength-based pitch

At 7.5 GHz in FR-4 (εeff ≈ 3.3 microstrip):
λ = c / (f·√εeff) ≈ 300 / (7.5 · 1.82) ≈ **22 mm**

λ/10 ≈ 2.2 mm. Use **≤ 2.5 mm** for PCIe-adjacent stitching.

### 3.2 Stitching rules

| Location | Rule |
|---|---|
| Along PCIe corridor (L1 GND pour ↔ L2 ↔ L4) | Via fence both sides, **pitch ≤ 2.5 mm**, center ≥ 0.6 mm from nearest trace edge |
| L1 copper pour near pairs | Pull back ≥ 3W (0.6 mm) from pair, or omit pour in corridor. **No floating copper islands**: every pour fragment gets ≥ 2 vias |
| General board GND stitching | 5 mm grid |
| Board edge | 2.5 mm pitch fence around perimeter near the M.2 slot |
| M.2 connector GND pins | Each GND pad gets its own via to L2 (via-in-pad tented or via ≤ 0.3 mm from pad) |
| CM connector GND pins adjacent to PCIe pins | One via per GND pin, shortest possible |
| AC-cap void | 2 GND vias flanking the cap pair, ≤ 1 mm from void edge |
| M.2 mounting standoff | Tie to GND with ≥ 4 vias around the pad (it is a return/shield point for the SSD) |

### 3.3 Layer transitions (only if unavoidable)
- **Via size:** 0.3 mm drill / 0.55 mm pad, through-hole. At 1.6 mm Gen2 the stub is tolerable; no backdrill needed.
- **Antipad:** 0.85–0.95 mm diameter on all planes. Use an oval/merged antipad covering both P and N vias.
- **Return vias:** **2 GND vias per pair**, each ≤ 1.0 mm (ideally 0.8 mm) center-to-center from the signal vias, placed symmetrically.
- **Matching:** Both P and N transition together with an identical count of vias.
- **Reference:** Changing from L2 to L4 reference is acceptable only with the stitching above. **Never** transition to a layer referenced to L3 power without stitching capacitors. Avoid this entirely.

### 3.4 Plane integrity checklist
- No splits, slots, or via-antipad "chains" in L2 under any PCIe pair. Check for merged antipads from the CM connector field.
- The L3 3V3_M2 pour must not intrude under PCIe on L2's far side such that L2 is perforated.
- No L2 cutouts for keepouts under the M.2 SSD except pad voids specified above.

---

## 4. Supporting Circuitry (affects SI margin)

| Signal | Implementation |
|---|---|
| 3V3 to M.2 (pins 2,4,12,14,16,18,70,72,74) | Dedicated load switch, ≥ 3 A capable. Bulk 2×22 µF + 4×100 nF at socket. Pour wide on L3 |
| PERST# (pin 50) | From module PCIe_nRST, 3.3 V, 10 k pull-down optional |
| CLKREQ# (pin 52) | Open-drain, 10 k pull-up to 3V3, route to module CLKREQ (or tie low if unsupported) |
| PEWAKE# (pin 54) | 10 k pull-up, optional to module |
| SUSCLK (pin 68) | NC or 32.768 kHz if available |
| PEDET (pin 69) | Read via GPIO for SATA/NVMe detect, optional |
| LED (pin 10) | Open-drain LED with resistor to 3V3 |
| ESD on PCIe pairs | Generally **omit** (internal connector); if required, use ≤ 0.3 pF flow-through diodes |

---

## 5. Verification Plan (Cycle 31 Exit Criteria)

1. **Fab:** Controlled-impedance order with coupon; TDR report 85 Ω ±10%.
2. **Pre-layout:** Field-solve the final stackup and log W/S per fab data.
3. **Post-layout DRC:**
   - Intra-pair skew report ≤ 0.13 mm.
   - Plane-void scan under nets `PCIE_*`.
   - Via-to-return-via distance ≤ 1.0 mm.
4. **Optional SI sim:** Insertion loss < 3 dB at 2.5 GHz and return loss < −10 dB to 2.5 GHz for the channel. Gen2 has ample budget.
5. **Bring-up:**
   - `lspci -vv` shows `LnkSta: Speed 5GT/s, Width x1`.
   - Run `fio` sequential reads for ~30 min.
   - Check `dmesg` for zero AER correctable errors (`RxErr`, `BadTLP`, `BadDLLP`).
6. **Margin check:** Force Gen1 versus Gen2 and compare error counters. Any correctable errors at Gen2 mean revisiting §2.5 and §3.3.

---

## 6. Cycle 31 Change Log Template

| ID | Change | Reason | Status |
|---|---|---|---|
| C31-01 | Impedance retarget to 85 Ω | M.2 spec alignment | |
| C31-02 | Fence pitch reduced to ≤ 2.5 mm | λ/10 at 7.5 GHz | |
| C31-03 | AC-cap L2 voids plus flanking vias | Pad discontinuity | |
| C31-04 | Verified no double TX caps vs module | Lane integrity | |

If you share the actual fab stackup (dielectric thicknesses and Dk) and the board outline/connector placement, I can give exact W/S values and a concrete route length estimate.