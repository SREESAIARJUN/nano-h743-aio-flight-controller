# M.2 NVMe (PCIe Gen2 x1) Routing and Return-Path Design: Radxa CM4 Carrier, Cycle 13

## 0. Assumptions

I don't have your Cycle 1–12 records, so this design rests on the assumptions below. Verify each against your BOM and stackup before releasing.

| Item | Assumed value |
|---|---|
| Module | Radxa CM4-form-factor module, PCIe Gen2 x1 (5 GT/s) over two Hirose DF40 board-to-board connectors |
| Target | M.2 M-key socket (2230/2242/2280), single lane |
| PCB | 4-layer, 1.6 mm (JLC04161H-7628-class stackup): L1 Sig, L2 GND, L3 PWR, L4 Sig |
| Host TX AC caps | Verify in the Radxa module datasheet whether they sit on the module. If they don't, place them on the carrier (see §3). |
| Device TX AC caps | On the SSD (required by the M.2 specification) |

---

## 1. Net List and Pin Map (M.2 M-Key)

M.2 net names are from the SSD's point of view: **PER = SSD receive = host TX**.

| Function | Host side (CM4 B2B) | M.2 pin | Notes |
|---|---|---|---|
| PCIe TX (host→SSD) | PCIE_TX_P/N | 43 PERp0 / 41 PERn0 | Needs AC caps (host side) |
| PCIe RX (SSD→host) | PCIE_RX_P/N | 49 PETp0 / 47 PETn0 | Caps are on the SSD, so route direct |
| REFCLK 100 MHz HCSL | PCIE_CLK_P/N | 55 REFCLKp / 53 REFCLKn | DC-coupled, no caps |
| PERST# | PCIE_nRST | 50 | 3.3 V, 10 k pull-up optional |
| CLKREQ# | PCIE_CLK_nREQ | 52 | Open-drain, pull-up to 3.3 V |
| PEWAKE# | — | 54 | NC or pull-up |
| 3.3 V | — | 2, 4, 12, 14, 16, 18, 70, 72, 74 | ≥2.5 A rail |
| GND | — | All GND pins | Stitch each pin to L2 with its own via |

---

## 2. Differential Routing Parameters

### 2.1 Impedance and Geometry

**Target impedance: 90 Ω differential ±10%.** This matches the module-side routing. Use the same target for all three pairs, including REFCLK.

The geometries below are calculated starting points from closed-form approximations. Run your fab's field solver and request controlled impedance with a coupon.

| Stackup option | Dielectric to L2 | Trace width W | Gap S | Approx. Zdiff |
|---|---|---|---|---|
| 7628 prepreg (h ≈ 0.21 mm, εr ≈ 4.4) | 0.21 mm | 0.28–0.30 mm | 0.18 mm | ~90 Ω |
| 3313 prepreg (h ≈ 0.10 mm, εr ≈ 4.1) | 0.10 mm | ~0.13 mm | ~0.13 mm | ~90 Ω |
| Stripline (6-layer, if migrating) | — | ~0.10 mm | ~0.15 mm | ~90 Ω |

**Recommendation:** use the thinner-prepreg stackup (3313/2116). Narrower pairs fan out from the DF40 (0.4 mm pitch) and M.2 (0.5 mm pitch) pins with less necking, and the tighter coupling to GND improves crosstalk.

### 2.2 Constraints Table (Netclass `PCIE_DP`)

| Parameter | Value |
|---|---|
| Intra-pair skew (P vs N) | ≤ 0.127 mm (5 mil), matched near the mismatch source |
| Inter-pair skew (TX vs RX vs CLK) | Not required (each lane recovers its own clock); keep within ~25 mm for tidiness |
| Max carrier length (B2B pad → M.2 pad) | ≤ 75 mm recommended, 100 mm absolute |
| Layer | L1 preferred (direct to both connectors); L4 allowed, referenced to L3 only if L3 is a solid plane for the full run |
| Layer changes | ≤ 2 vias per signal, symmetric in each pair |
| Pair-to-pair spacing (TX↔RX) | ≥ 5×W or ≥ 0.75 mm edge-to-edge |
| Pair to any other signal | ≥ 5×W (≥ 1 mm to clocks, SDIO, Ethernet, DC/DC switch nodes) |
| Pair to plane edge / board edge | ≥ 3×h from the reference plane edge; ≥ 1 mm from board edge |
| Bends | 45° or arcs only; no 90° corners |
| Skew compensation | Small serpentine bumps (amplitude ≤ 2×S, length ≥ 3×W), placed within ~5 mm of the bend causing the mismatch |
| Uncoupled length (breakout) | ≤ 3 mm total per pair |
| Stubs | None. No test points on the pairs; use pads inline if probing is essential. |
| Polarity swap | Permitted by PCIe (lane polarity inversion), but avoid it. If you must swap, swap only within a pair, never TX↔RX. |

### 2.3 REFCLK Specifics
- Route REFCLK at 90 Ω (HCSL tolerates 85–100 Ω) with intra-pair skew ≤ 0.127 mm.
- Place no series resistors on the carrier unless the module datasheet requires them. HCSL termination is normally handled at the source.
- Keep ≥ 1.5 mm (or a guard trace with stitching) from the TX/RX pairs over their parallel run.

### 2.4 Sideband Signals
PERST#, CLKREQ# and PEWAKE# are slow, so a 0.15 mm trace is fine. Keep them out of the corridor between the high-speed pairs.

---

## 3. AC Coupling Capacitors (Host TX Only, If Not on the Module)

| Parameter | Value |
|---|---|
| Value | 100 nF (Gen2 allows 75–200 nF), X7R, ±10% |
| Package | 0201 preferred; 0402 acceptable |
| Placement | Within 10 mm of the M.2 connector, P and N caps side by side and symmetric |
| Pad compensation | Cut out the L2 GND under the cap pads (void = pad outline + 0.1 mm) and reference L3 instead. This raises pad impedance back toward 90 Ω. Only do this if L3 is solid GND or well-decoupled. On 3313 stackups with 0201 parts, the cutout is often unnecessary; check with the field solver. |
| Fan-in | Taper into the pads symmetrically; no unequal stubs |

**Do not** add caps on the RX pair (PETp0/n0) or on REFCLK.

---

## 4. High-Speed Return Path and Via Stitching

### 4.1 Reference Plane Rules
1. Keep L2 as solid GND under the entire route, from DF40 pads to M.2 pads. No splits, slots, or via-antipad "swiss cheese" under the pairs. Check that adjacent via antipads don't merge into a slot.
2. No L2 cutouts for power routing anywhere inside the pair's 3×h shadow.
3. If the route crosses to L4 referencing L3, L3 must be GND for that span. A power plane reference forces the return current through decoupling caps and adds impedance discontinuities. Simplest rule: keep the pairs on L1 the whole way.

### 4.2 Signal Via Transitions
Return current switches reference planes at every layer change, so it needs a nearby GND via.

| Rule | Value |
|---|---|
| Return vias per differential via pair | 2 GND vias, placed symmetrically, one on each side of the pair along the pair axis |
| Return via distance | ≤ 0.6 mm center-to-center from the signal via (≤ 1 mm absolute) |
| Signal via | 0.3 mm drill / 0.5–0.55 mm pad |
| Antipad | ~0.8–0.9 mm diameter on all planes. Use an oblong shared antipad around the P/N via pair to tune impedance toward 90 Ω. |
| Via pitch within a pair | Match trace pitch (W+S) as closely as drill rules allow |
| Unused pads | Remove non-functional inner pads on signal vias |
| Back-drill | Not needed at 5 GT/s and 1.6 mm |

### 4.3 Routing-Corridor Stitching (Guard Fence)

The design frequency is set by the third harmonic of 2.5 GHz, about 7.5 GHz. In FR-4 (εeff ≈ 3.3) the wavelength there is λ ≈ 22 mm.

| Parameter | Value |
|---|---|
| Stitch-via pitch along the corridor | ≤ 3 mm (≈ λ/8 at 7.5 GHz; λ/20 if the budget allows ~1.5 mm near connectors) |
| Fence offset from pair edge | ≥ 3×W, or 0.5 mm, whichever is larger |
| L1 GND pour clearance to pairs | ≥ 3×S (≥ 0.5 mm) so the pour doesn't load the impedance. Otherwise you've built coplanar waveguide and must recalculate impedance with the pour. |
| Pour slivers / islands | Delete any L1 copper finger < 2 mm wide or not stitched at both ends. Floating copper resonates. |
| Via size | 0.3 / 0.5 mm, tied L1–L2–L3(if GND)–L4 GND |

### 4.4 Connector-Zone Stitching

**DF40 B2B (module side):**
- Give every DF40 GND pin adjacent to the PCIe pins its own GND via within 0.5 mm of the pad. Don't share one via between two GND pins.
- Place 4 or more GND vias at ≤ 1.5 mm pitch in a ring around the PCIe pin group.

**M.2 M-key:**
- Give each GND pin flanking the lanes its own via within 0.5 mm. These are pins 39, 45, 51 and 57 between the pairs.
- Put a row of GND vias at ≤ 1.5 mm pitch behind the connector pad row on L1→L2.

**M.2 standoff/mounting:**
- Tie the standoff pad to GND with 4 or more vias. This provides a return path for the SSD's own ground.

### 4.5 Plane-to-Plane Return Bridging
If L3 is a power plane and any part of the route references it, place 100 nF 0201 stitching caps (L3 PWR ↔ GND) within 2 mm of each layer-transition via. The better option is to avoid L3 referencing entirely.

---

## 5. Power Integrity for the M.2 Socket

| Item | Value |
|---|---|
| 3.3 V bulk | 2 × 22 µF 0805 within 10 mm of the socket |
| Local decoupling | 4 × 100 nF + 1 × 1 µF placed at the 3.3 V pin groups |
| Rail capacity | ≥ 2.5 A peak (NVMe write bursts), ≤ 3% droop |
| Current path | Wide L3 pour; avoid routing 3.3 V across the PCIe pair shadow on L2 |
| Load switch (optional) | Add one