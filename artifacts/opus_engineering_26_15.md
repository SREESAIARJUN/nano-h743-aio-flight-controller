# M.2 NVMe (PCIe Gen2 x1) Routing and Return-Path Design: Radxa CM4-Format Carrier, Cycle 26

## 0. Scope and Assumptions

| Item | Value |
|---|---|
| Link | PCIe Gen2 x1 (5 GT/s, 8b/10b), Nyquist = 2.5 GHz |
| Host | Radxa CM-series module on CM4-compatible 2×100-pin Hirose DF40 connectors |
| Device | M.2 2230/2242/2280 M-key socket, NVMe SSD (lane 0 only) |
| PCB | 4-layer, 1.6 mm, FR-4 (Dk ≈ 4.2–4.4 at 1–3 GHz, Df ≈ 0.02) |
| Signals | TX pair (host→SSD), RX pair (SSD→host), REFCLK (100 MHz HCSL), sideband |

**Verify before release:** Check the module datasheet for two things. First, whether the host TX AC-coupling capacitors are already on the module. Second, the exact DF40 pin numbers for this module revision. Radxa and RPi CM4 pinouts match functionally, but confirm against the Cycle 26 schematic library.

---

## 1. Stackup and Impedance Targets

### Recommended 4-layer stackup (JLC/PCBWay-style 7628 prepreg)

| Layer | Use | Thickness |
|---|---|---|
| L1 Top | PCIe pairs, M.2 socket, components | 35 µm Cu + plating |
| Prepreg | 7628, Dk ≈ 4.4 | 0.21 mm |
| L2 | **Solid GND (no splits under PCIe)** | 17.5 µm (0.5 oz) |
| Core | — | ~1.065 mm |
| L3 | Power (3V3_SSD, 5V, 3V3) | 17.5 µm |
| Prepreg | 7628 | 0.21 mm |
| L4 Bottom | Low-speed signals, GND pour | 35 µm |

### Differential impedance

- **Target: 90 Ω ±10% differential.** This is the CM4 module guidance, and it sits between the PCIe CEM 85 Ω and the common 100 Ω.
- Single-ended impedance should come out around 50 Ω.

### Example geometry (L1 microstrip over L2, H = 0.21 mm)

| Parameter | Value |
|---|---|
| Trace width W | 0.20 mm (≈ 8 mil) |
| Intra-pair gap S | 0.20 mm (≈ 8 mil) |
| Result | ≈ 88–92 Ω diff (confirm with the fab's calculator and order impedance control) |
| Solder mask over traces | Lowers Z by ~2–3 Ω. Include it in the calculation. |

The REFCLK pair uses the same geometry. The HCSL clock is specified at 100 Ω diff, but 85–100 Ω works fine at 100 MHz.

---

## 2. Differential Pair Routing Rules

| Rule | Value | Notes |
|---|---|---|
| Intra-pair skew (P vs N) | ≤ 0.127 mm (5 mil) | Match at the mismatched end, using small serpentines (amplitude ≤ 2×S, segment ≥ 3×W) |
| Inter-pair skew (TX vs RX) | Not required | Lanes are independent. Only refclk-to-data relationships matter loosely, and those have no tight spec. |
| Max length, module connector → M.2 pins | ≤ 100 mm recommended (Gen2 budget allows ~250 mm on FR-4) | Keep it short. Place the M.2 adjacent to the DF40 PCIe pins. |
| Pair-to-pair spacing (TX↔RX) | ≥ 4×W edge-to-edge (≥ 0.8 mm); ≥ 5×W where parallel > 10 mm | TX/RX NEXT is the main crosstalk concern |
| Pair to other signals | ≥ 5×H (≥ 1.0 mm), ≥ 1.5 mm from clocks/SMPS/USB | |
| Spacing to GND pour on same layer | ≥ 3×W (≥ 0.6 mm) | Avoids coplanar loading that shifts Z |
| Bends | 45° or arcs, no 90° | Keep P/N bends symmetric |
| Layer changes | **Zero preferred.** Max 2 vias per pair end-to-end. | Route fully on L1 if the placement allows |
| Reference plane | L2 GND, continuous for the entire length | No crossing of plane gaps or slots, and no routing over L3 power splits where L2 has voids |
| Board edge | ≥ 1.0 mm from trace to edge of L2 plane | |
| Stubs | None | Test points only as via-less pads in-line, if any |
| Breakout at the DF40 / M.2 pads | Uncoupled region ≤ 2.5 mm, both legs equal length | |

### AC coupling capacitors

- **Host TX → SSD RX (PETp0/PETn0 at the M.2):** 100 nF (75–265 nF allowed), 0201 preferred, otherwise 0402, X7R. Place them near the host side. **Omit them if the module already has them.**
- **SSD TX → host RX (PERp0/PERn0):** No caps on the carrier. The SSD contains its own AC caps per the M.2 spec.
- **Pad layout for the caps:**
  - Place the cap pads side by side, symmetric.
  - Cut out L2 GND under the cap pads (anti-pad void the size of the pad + 0.1 mm), referencing L3, to cancel pad capacitance. Without the void, expect a ~5–8 Ω dip.
  - Put one GND stitching via on each side of the cap pair.

### M.2 M-key lane 0 pinout (socket side)

| Signal | Pin |
|---|---|
| PERn0 / PERp0 (SSD TX → host RX) | 41 / 43 |
| PETn0 / PETp0 (host TX → SSD RX) | 47 / 49 |
| REFCLKn / REFCLKp | 53 / 55 |
| PERST# | 50 |
| CLKREQ# | 52 |
| PEWAKE# | 54 |
| Lanes 1–3 | Not connected |

**Polarity:** PCIe allows P/N inversion, so a P/N swap within a pair is acceptable to avoid crossing. Lane polarity inversion is auto-negotiated. Never swap TX with RX.

---

## 3. High-Speed Return Path and Via Stitching

### 3.1 Signal via transitions (if a layer change is unavoidable)

- Use a via pair with 0.3 mm drill and 0.6 mm pad, P/N vias placed symmetrically, at the same pitch as the trace gap where possible.
- **Return vias:** 2 GND vias per diff-via pair, one on each side. Center-to-center distance from signal via ≤ 1.0 mm (target 0.8 mm).
  - These GND vias are mandatory when changing reference planes, for example L1/L2 → L4/L3.
  - If the reference switches from GND to a power plane, add a 100 nF 0201 stitching cap between L3 power and GND within 2 mm of the via.
- **Anti-pad:**
  - Use an oval anti-pad on L2/L3 enclosing both vias, with 0.25–0.35 mm clearance around the pads.
  - Do not let anti-pads of adjacent pairs merge into a slot.
  - Remove non-functional pads on the inner layers.
- **Stub:** On a 1.6 mm board an L1→L4 via has no stub. An L1→L3 via leaves a ~0.2 mm stub, which is negligible at 2.5 GHz. No back-drill is needed.

### 3.2 Guard / stitching along the channel

- The rule is λ/20 at the highest frequency of interest. Using the 5 GHz harmonic in FR-4 (εeff ≈ 3.3): λ ≈ 33 mm, so λ/20 ≈ 1.6 mm. Use a **2.0–2.5 mm pitch** for practical manufacturing.
- If an L1 GND pour runs alongside the pairs:
  - Keep it ≥ 0.6 mm from the traces.
  - Stitch it to L2 every **≤ 2.5 mm**, with a via row centered in the pour.
  - Remove any pour fingers or islands that cannot be stitched (no floating copper).
- Between TX and RX pairs where they run parallel, a stitched GND strip (≥ 0.6 mm wide, vias at 2.5 mm) is optional but recommended. It adds roughly 10 dB of NEXT isolation.

### 3.3 Connector transitions

- **DF40 module connector:**
  - Tie every GND pin adjacent to the PCIe pins directly to L2 with its own via, ≤ 0.5 mm from the pad. Do not daisy-chain GND pins on top copper.
  - Add one additional GND via per PCIe pair at the point where the breakout begins.
- **M.2 socket:**
  - Every GND pin (pins 33, 39, 45, 51, 57, etc.) gets its own via to L2, placed directly at the pad.
  - Add a row of GND stitching vias at 2 mm pitch across the socket's rear edge and around the pin field.
  - Tie the mounting standoff / screw pad to GND with ≥ 4 vias.
- **Plane edge stitching:** Place a ring of GND vias at ≤ 5 mm pitch around the board perimeter and around the M.2 keep-out zone. This suppresses plane-cavity resonance and edge radiation.

### 3.4 Plane integrity checks

- No L2 voids beneath the pairs, apart from the intentional AC cap voids.
- L3 power split edges must not fall under the PCIe corridor (±3 mm).
- Group via fields from other nets so their anti-pads do not perforate L2 under the pairs.

---

## 4. REFCLK and Sideband

- **REFCLK:**
  - Use the same 90–100 Ω geometry.
  - Skew ≤ 0.25 mm intra-pair.
  - Keep it ≥ 1.5 mm from the data pairs.
  - Stitch it like the data pairs.
  - No series termination unless the module datasheet calls for it. HCSL is terminated at the source.
- **PERST#:** Connect directly from the module.
- **CLKREQ#:** Add a 10 kΩ pull-up to 3V3 if the host does not provide one. Some SSDs require CLKREQ# to be low to enable the clock. If the module has no CLKREQ input, tie it to GND through 0 Ω.
- **PEWAKE#:** Optional. Provide a pull-up if used.
- Pin 69 PEDET: leave NC. SUSCLK (pin 68): NC or 32.768 kHz.
- These sideband lines are routable on L4 with no constraints.

---

## 5. Power Delivery to the M.2 (Return-Path Relevant)

- **Supply:** 3V3_SSD on pins 2, 4, 12, 14, 16, 18, 70, 72, 74. Budget **≥ 2.5 A peak** (Gen3 parts can draw 3 A briefly, even when linked at Gen2).
- **Regulation:** Use a dedicated buck or a load switch with soft-start, because SSD inrush is high.
- **Decoupling at the socket:**
  - 1 × 22 µF 0805
  - 2 × 10 µF
  - 4 × 100 nF, one at each 3V3 pin cluster
  - Each cap gets its own GND via to L2 at the pad.
- **Layout:** Keep the buck switch node and inductor ≥ 10 mm from the PCIe corridor, and on the opposite side of the L2 plane where possible.

---

## 6. Loss Budget Sanity Check (Gen2)

| Contributor | Estimate at 2.5 GHz |
|---|---|
| 100 mm FR-4 microstrip @ ~0.07 dB/mm | ~0.7 dB |
| 2 vias | ~0.2 dB |
| AC caps | ~0.1 dB |
| M.2 connector | ~0.5 dB |
| DF40 connector + module routing | ~1.5 dB |
| **Total carrier contribution** | **≈ 3 dB** vs. Gen2 system budget ~13–15 dB → large margin |

The design is dominated by impedance discontinuities and return-path breaks, not by loss. That is why the stitching rules above matter more than trace length.

---

## 7. Cycle 26 Verification Checklist

1. Fab impedance coupon ordered, 90 Ω ±10% diff on L1.
2. DRC net classes created:
   - `PCIe_DATA` (W 0.20 / S 0.20, clearance 0.8)
   - `PCIe_CLK`
   - Diff-pair skew rule 0.127 mm
3. 2D/3D review: no PCIe trace crosses an L2 void. Check this with an L2-only view overlay.
4. Every signal via has ≥ 2 GND vias within 1 mm.
5. AC cap placement and voids verified, and the module-side cap presence confirmed.
6. Stitching via pitch ≤ 2.5 mm along pours, ≤ 5 mm perimeter.
7. Bring-up checks:
   - `lspci -vv` shows `LnkSta: Speed 5GT/s, Width x1`.
   - `dmesg` is free of AER corrected errors under `fio` sustained load (≥ 10 min).
   - Optionally, run TDR on a coupon to confirm 90 Ω ±9 Ω through the cap and via transitions.

If you share the Cycle 26 placement (module orientation and M.2 position/length), I can turn this into specific route lengths, a serpentine budget, and KiCad/Altium net-class rule files.