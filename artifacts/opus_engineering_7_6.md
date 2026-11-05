# M.2 NVMe PCIe Gen2 Routing and Return-Path Stitching Specification

**Radxa CM4-form-factor carrier, Design Cycle 7**

I don't have the Cycle 1–6 records, so this document stands alone. It also lists the assumptions to check against your previous cycle's stackup and placement. Treat every numeric value as a design target. Confirm each one with your fab's impedance calculator and with the module datasheet before release.

---

## 1. Interface Scope and Assumptions

| Item | Assumption |
|---|---|
| Host | Radxa CM3 / CM5 (or RPi CM4-pinout-compatible module), PCIe 2.x **x1** on the 100-pin high-speed connector |
| Device | M.2 M-key socket, 2230/2242/2280, NVMe |
| Data rate | 5.0 GT/s, 8b/10b. Nyquist frequency is 2.5 GHz; design for content up to about 12.5 GHz (5th harmonic). |
| Signals | PCIE_TX±, PCIE_RX±, PCIE_CLK± (100 MHz HCSL), PERST#, CLKREQ#, optional WAKE# |
| Board | 4-layer, 1.6 mm, FR-4 (JLC-class process) |

---

## 2. Stackup (4-layer, 1.6 mm)

| Layer | Function | Notes |
|---|---|---|
| L1 | Signal: all PCIe routing | Microstrip |
| — | Prepreg, ~0.1 mm (3313-type, Er ≈ 4.1) | A thin dielectric keeps traces narrow and coupling tight |
| L2 | **Solid GND**, unbroken under the PCIe corridor | Primary reference plane |
| — | Core, ~1.2 mm | |
| L3 | Power (3V3, 5V) plus low-speed signals | **Never** route PCIe here |
| — | Prepreg, ~0.1 mm | |
| L4 | GND pour plus low-speed signals | Secondary return path; stitched to L2 |

**Rule:** route PCIe on **L1 only**, with zero layer changes if placement allows. The CM connector and the M.2 socket are both top-side SMT, so a 0-via route is achievable and is the target for this cycle.

---

## 3. Differential Impedance and Geometry

| Parameter | Target | Starting geometry (L1 over L2, h ≈ 0.1 mm) |
|---|---|---|
| TX/RX differential impedance | **85 Ω ±10%** (PCIe spec base; 90 Ω is acceptable if the fab can't hit 85) | W ≈ 0.13 mm / S ≈ 0.13 mm. **Verify with the fab calculator.** |
| REFCLK differential impedance | 85–100 Ω ±10% | Same geometry as data pairs is acceptable |
| Single-ended (per leg) | ~45–50 Ω | — |
| Pair-to-pair spacing (TX↔RX) | **≥ 4W** (≥ 0.5 mm); ≥ 5W where TX/RX run parallel > 10 mm | Limits NEXT |
| Pair to any other signal | ≥ 5W (≥ 0.65 mm) | ≥ 1 mm from switching power traces or inductors |
| Pair to plane/pour edge on L1 | ≥ 3W | Prevents coplanar loading that shifts impedance |
| Pair to L2 plane edge (void, split, slot) | ≥ 3h, never crossing | Hard rule |

Order controlled impedance on L1 with tolerance ±10% and request a coupon.

---

## 4. Length and Skew Budget

| Parameter | Limit | Notes |
|---|---|---|
| Intra-pair skew (P vs N), each pair | **≤ 0.127 mm (5 mil)** | Match at the end where the mismatch originates, within 5 mm of it |
| Inter-pair skew (TX vs RX) | No requirement | PCIe lanes are independent; don't waste space matching them |
| REFCLK intra-pair | ≤ 0.127 mm | |
| Carrier trace length, connector → M.2 pads | **≤ 75 mm target, ≤ 100 mm max** | Gen2 budget with module and card losses: roughly 4–5 dB on the carrier at 2.5 GHz is safe |
| Length-tuning serpentine | Amplitude ≤ 2W, segment ≥ 3W, gap ≥ 4W within the pair (S ≥ 3× trace width) | Use small "trombone" bumps, not long serpentines |
| Bends | 45° or arcs only; no 90° corners | Alternate bend direction to self-compensate skew |

---

## 5. AC Coupling and Pin Mapping

- **Host TX → device RX:** the AC coupling caps are normally **on the compute module**. The RPi CM4 places them on-module. **Confirm this in the Radxa CM3/CM5 datasheet for this cycle.** If they are not on the module, place 100 nF (0201, or 0402 X7R) caps on the carrier, within 10 mm of the module connector, with symmetric P/N placement.
- **Device TX → host RX:** the M.2 SSD carries its own TX caps. **Do not add caps** on the carrier for this direction.
- Pad-to-trace transition at any cap: void L2 under the cap pads (a rectangle ≈ pad footprint plus 0.1 mm) to cancel the pad capacitance. Do this only on caps placed on the carrier.
- **Crossing check:** host PCIE_TX± connects to M.2 **PERp0/PERn0** (device receive, pins 47/49). Host PCIE_RX± connects to M.2 **PETp0/PETn0** (pins 41/43). Polarity inversion inside a pair is tolerated by link training but should be avoided. **TX/RX swap is not tolerated.**

---

## 6. Return-Path Via Stitching

### 6.1 Along the route

| Rule | Value |
|---|---|
| L1 GND guard pour beside the pairs | Optional. If used, keep ≥ 3W clearance and stitch it, never leave it floating. |
| Stitching via pitch along guard and pour edges | **≤ 2.5 mm** (≈ λ/10 at 12.5 GHz in FR-4; λ ≈ 12 mm) |
| Via size | 0.3 mm drill / 0.5 mm pad (0.25/0.45 if the fab allows) |
| Isolated L1 copper islands | Remove any island that can't take ≥ 2 vias |

### 6.2 At the connectors (the critical part)

- **CM high-speed connector:** place at least 1 GND via per GND pin adjacent to the PCIe pins, ≤ 0.5 mm from the pad, connected straight to L2.
- **M.2 socket:** place a GND via at each GND pin flanking pins 41–49 (data) and 53/55 (REFCLK). Ensure an unbroken L2 under the full fan-out region.
- Mounting standoff (M2/M2.5 hole) for the card: tie it to GND with ≥ 4 vias around the pad.

### 6.3 If a layer change is unavoidable (fallback)

- Use a symmetric via pair for P/N with antipad ≈ 0.8–0.9 mm (merged oval antipad preferred). Target via differential impedance ≈ 85–90 Ω.
- Place **two GND return vias per signal-via pair**, one on each side, **≤ 0.75 mm (30 mil)** center-to-center from the signal vias.
- Both reference planes must be GND (L2 ↔ L4). Never transition across a power reference. If you must, add a 100 nF stitching cap within 1.5 mm.
- Limit to **≤ 2 vias per net** across the entire route. Backdrilling is unnecessary at 1.6 mm and Gen2.

### 6.4 Board-level

- Use a perimeter stitch ring at ≤ 5 mm pitch (L1/L2/L4 GND), plus a grid at ≤ 10 mm across general pour areas.
- Don't route any L3 trace under the PCIe corridor in a way that requires slots in L2.

---

## 7. REFCLK and Sideband Signals

| Signal | Requirement |
|---|---|
| REFCLK± | Route as a diff pair, same rules as data. Keep it ≥ 5W from TX/RX. Termination is on-module (HCSL); confirm in the datasheet. |
| PERST# | 3.3 V, routed point to point. Add a 10 kΩ pull-down if the module leaves it floating at boot. |
| CLKREQ# | Open-drain; 10 kΩ pull-up to 3V3. Tie to GND if you want the clock always on (simplest bring-up). |
| M.2 pin 69 (PEDET) / SUSCLK | Leave NC or follow the NVMe card guidance. SUSCLK is optional. |

---

## 8. M.2 Power (feeds SI margin)

- 3V3 rail: rated for **≥ 2.5 A continuous** (3 A peak for Gen3 cards running at Gen2).
- Decoupling at the socket: 2× 22 µF + 4× 100 nF, with each 100 nF placed at a 3V3 pin and connected to L2 with its own via.
- Keep the buck converter and its inductor **≥ 10 mm** away from the PCIe corridor, and don't place them over it on any layer.

---

## 9. Cycle 7 Verification Checklist

- [ ] Fab impedance calculation matches 85 Ω ±10% for the chosen W/S; coupon ordered
- [ ] Zero vias on TX/RX/REFCLK (or the §6.3 rules are applied)
- [ ] Intra-pair skew ≤ 5 mil on all three pairs (DRC report attached)
- [ ] Carrier length ≤ 75 mm
- [ ] No L2 voids or splits under the corridor (plane-check screenshot)
- [ ] Stitch pitch ≤ 2.5 mm along pours; GND vias at every connector GND pin
- [ ] TX/RX crossing verified against the M.2 pinout (PET↔host RX, PER↔host TX)
- [ ] AC-cap location confirmed against the current Radxa module datasheet revision
- [ ] Optional: run an S-parameter extraction of the corridor; target SDD21 > −3 dB and SDD11 < −10 dB at 2.5 GHz

**Bring-up check:** confirm with `lspci -vv` that LnkSta reports "Speed 5GT/s, Width x1". If the link trains at 2.5 GT/s, first suspect skew or REFCLK quality, then return-path discontinuities at the connectors.

If you share the Cycle 6 stackup, the CM/M.2 placement distance, or the review findings, I can turn this into a delta-only change list and recalculate the trace geometry for your exact fab.