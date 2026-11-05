# M.2 NVMe (PCIe Gen2 x1) Routing and Return-Path Design for a Radxa CM4-Form-Factor Carrier, Cycle 20

**Scope:** Radxa CM3 or CM4-pinout module → carrier board → M.2 Key-M 2230/2242/2280 NVMe SSD. The link is PCIe Gen2, 5 GT/s, x1 lane, with a 100 MHz HCSL reference clock.

> **Verify before layout freeze:** Check the module pin numbers against the current Radxa datasheet for your exact SKU. Pinouts have shifted between CM3, CM3S and CM5 revisions.

---

## 1. Signal Inventory

| Function | Module side (CM4 pinout) | M.2 Key-M pin | Notes |
|---|---|---|---|
| Host TX+ / TX− | PCIE_TX_P / N (~114 / 116) | PERp0 43 / PERn0 41 | Host → SSD |
| Host RX+ / RX− | PCIE_RX_P / N (~108 / 110) | PETp0 49 / PETn0 47 | SSD → Host |
| REFCLK+ / − | PCIE_CLK_P / N (~102 / 104) | REFCLKp 55 / REFCLKn 53 | 100 MHz HCSL |
| PERST# | PCIE_nRST (~109) | 50 | 3.3 V open-drain or push-pull |
| CLKREQ# | PCIE_CLK_nREQ (~100) | 52 | 10 k pull-up to 3.3 V |
| PEWAKE# | optional GPIO | 54 | 10 k pull-up; optional |
| 3.3 V | carrier regulator | 2, 4, 12, 14, 16, 18, 70, 72, 74 | ≥2.5 A rated, low ESR bulk |
| GND | — | all GND pins | Every GND pin to the plane |

**AC coupling:**
- **Host TX:** The capacitors are on the module. Do not add a second set on the carrier.
- **SSD TX:** The capacitors are on the M.2 card itself.
- **Result:** The carrier has no series capacitors on any data lane. Route straight through.
- **If a future host lacks on-module capacitors:** Place 0201 220 nF capacitors within 10 mm of the host, with matched pads.

---

## 2. Stackup (4-layer, 1.6 mm, JLC04161H-3313 or equivalent)

| Layer | Use | Dielectric to next |
|---|---|---|
| L1 | PCIe pairs, components | 3313 prepreg, ~0.10 mm, εr ≈ 4.1 |
| L2 | **Solid GND (no splits under PCIe)** | Core ~1.26 mm |
| L3 | Power (3.3 V / 5 V pours), slow signals | ~0.10 mm |
| L4 | Low-speed signals, GND fill | — |

**Routing rule:** Route all PCIe and REFCLK traces on L1 only, referenced to L2. The goal is zero layer changes. If a change is unavoidable, go L1→L4 and follow §4.2.

---

## 3. Differential Routing Parameters

### 3.1 Impedance and geometry

| Parameter | Value |
|---|---|
| Target Zdiff, data lanes | **85 Ω ±10%** (PCIe base spec). 90 Ω is acceptable if the fab only offers USB-shared geometry. |
| Target Zdiff, REFCLK | 85–100 Ω ±10% (use the same geometry as data for simplicity) |
| Single-ended reference | ~45–50 Ω |
| Trace width, starting point | **0.13 mm (5.1 mil)** |
| Pair gap, starting point | **0.15 mm (5.9 mil)** |
| Copper | 1 oz outer finished |

Confirm the final width and gap with the fab's impedance calculator (JLC/Polar), and order a controlled-impedance coupon.

### 3.2 Length and skew

| Rule | Value |
|---|---|
| Intra-pair skew (P vs N), each pair | **≤ 0.127 mm (5 mil)** |
| Intra-pair skew, REFCLK | ≤ 0.127 mm |
| TX-pair vs RX-pair matching | **Not required** (independent links) |
| Data vs REFCLK matching | Not required (common-clock tolerant) |
| Max carrier trace length, module connector → M.2 pad | **≤ 100 mm** recommended (target < 60 mm) |
| Skew compensation location | Within 5 mm of the mismatch source. Use small serpentine bumps with amplitude ≤ 2× gap and length ≥ 3× width. Never compensate at the far end. |

### 3.3 Spacing and keep-outs

| Rule | Value |
|---|---|
| Pair-to-pair (TX ↔ RX) | **≥ 4× trace width edge-to-edge (≥ 0.5 mm)**. Prefer a GND fence between the pairs. |
| Pair to any other signal | ≥ 5W (≥ 0.65 mm) |
| Pair to coplanar GND pour on L1 | ≥ 3W (≥ 0.4 mm) so the pair stays edge-coupled, not CPW-dominated |
| Pair to board edge | ≥ 2 mm |
| Plane split / void under pair | **Forbidden.** Keep ≥ 3W of solid L2 either side of the pair. |
| Over L3 power pour edges | Not applicable (L2 shields), but keep 3.3 V switching nodes ≥ 5 mm away |

### 3.4 Geometry hygiene

- Use 45° bends or arcs only, with no 90° corners. Stagger bends so the inner and outer traces see matched lengths.
- Keep the pair symmetric through connectors. Break out from the M.2 pads directly and symmetrically, with no neck-down longer than 1 mm.
- **Polarity:** PCIe supports lane polarity inversion. If P/N must cross to reach the M.2 pins, swap P↔N rather than crossing with vias. **This does not apply to REFCLK.** Keep REFCLK polarity correct.
- **Test points:** Do not put test points on data lanes. If debug access is needed, add 0201 pads inline with no stub.

---

## 4. Return Path and Via Stitching

### 4.1 Reference continuity

- L2 GND must be unbroken from the module connector footprint to the M.2 connector footprint along the pair corridor.
- Check the module connector antipad field. Large merged antipads under the Hirose DF40 footprints can create slots in L2. Route the pairs between pin rows where L2 is intact, or neck the antipads to restore copper webs of at least 0.15 mm.

### 4.2 Signal-via transitions (only if a layer change is unavoidable)

| Rule | Value |
|---|---|
| Signal via | 0.3 mm drill / 0.5 mm pad. Keep the P/N vias side-by-side at the pair pitch. |
| Antipad | ~0.8–0.9 mm. Use one oval antipad around both P and N vias. |
| GND return vias per pair transition | **2 minimum**, placed symmetrically, one adjacent to each signal via |
| GND via distance from signal via | **≤ 0.8 mm center-to-center (≤ 1.0 mm max)** |
| Stub | Through-hole L1→L4 has no stub. Avoid L1→L3 transitions on a through via (that leaves an L3→L4 stub). |

### 4.3 Stitching fence along the route

- **Wavelength basis:** The 5 GT/s signal has a 2.5 GHz fundamental, and the design covers the 3rd harmonic (7.5 GHz). In FR4 microstrip (εeff ≈ 3.2), λ ≈ 22 mm at 7.5 GHz. λ/10 gives ≈ **2.2 mm**.
- **Fence pitch:** Place GND stitching vias (0.3/0.5 mm) along both sides of each pair, connecting L1 GND pour to L2 and L4, at **≤ 2.5 mm pitch**.
- **Fence offset:** Set the fence ≥ 3W from the pair, about 0.45–0.6 mm from the trace edge.
- **Isolation fence:** Run one shared fence row between the TX and RX pairs.
- **Floating copper:** Remove any L1/L4 copper island that has fewer than 2 stitching vias, or stitch it. Islands must not resonate.

### 4.4 Connector zones

- **M.2 connector:** Place a GND via within 1 mm of every M.2 GND pad adjacent to the PCIe pins (pins 39, 45, 51, 57). Place 2 or more vias at the connector-shell / standoff ground.
- **Module connector:** Give each GND pin flanking the PCIe pins its own via to L2. Do not share one via between two pads.
- **REFCLK:** Fence it as aggressively as the data pairs. It is the most EMI-prone net on the board.

### 4.5 Decoupling return

- **M.2 3.3 V:** Place 1× 22 µF + 2× 100 nF within 3 mm of pins 2/4 and 70/72/74, each with its own GND via. Add 1× 100 µF bulk near the regulator.
- **Isolation:** Keep the 3.3 V regulator's switching loop and inductor at least 5 mm from the pairs, on the opposite side of the board.

---

## 5. Sideband and Power Details

| Net | Treatment |
|---|---|
| PERST# | Series 33 Ω optional. Hold low until 3.3 V is stable for ≥ 100 ms (module firmware usually handles this). |
| CLKREQ# | 10 k pull-up to 3.3 V. Tie low if the SoC lacks CLKREQ support, to force REFCLK always on. |
| SUSCLK (pin 68) | Leave NC or drive 32.768 kHz if the SSD needs it (rarely). |
| PEDET (pin 69) | NC, or a GPIO for SATA/NVMe detect. |
| DAS/DSS LED (pin 10) | Optional LED + 330 Ω to 3.3 V. |
| 3.3 V rail | Dedicated buck rated ≥ 3 A peak, with soft-start to tame NVMe inrush. |

---

## 6. Cycle 20 Verification Checklist

1. [ ] Impedance coupon specified on the fab drawing: 85 Ω diff L1/L2, ±10%.
2. [ ] DRC diff-pair rule set: W/G = 0.13/0.15 mm, uncoupled length ≤ 1.5 mm total.
3. [ ] Skew report per pair: ≤ 0.127 mm.
4. [ ] Total length per pair: ≤ 100 mm (record actual values).
5. [ ] L2 plane shows no voids or slots under the pairs. Check with the plane-only view plus the antipad overlay.
6. [ ] Every signal via has 2 or more GND vias within 0.8 mm.
7. [ ] Stitching fence pitch ≤ 2.5 mm, with no orphan copper islands.
8. [ ] No duplicate AC caps on the TX lane. P/N swap documented if used.
9. [ ] REFCLK polarity confirmed as unswapped.
10. [ ] Bring-up test: run `lspci -vv`, confirm `LnkSta: Speed 5GT/s, Width x1`. Then run `fio` sequential read for 10 minutes and check `dmesg` for AER correctable errors (target: 0).

If AER errors appear, check in this order: REFCLK fencing and plane continuity at the module connector, then the M.2 3.3 V ripple (should be < 50 mVpp), then the skew report.