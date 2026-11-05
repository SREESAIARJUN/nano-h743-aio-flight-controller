# M.2 NVMe (PCIe Gen2 x1) Routing and Return-Path Design: Radxa CM4 Expansion Board, Cycle 14

## 0. Scope and Assumptions

I don't have the earlier cycles of this design, so this spec rests on the assumptions below. Please verify them against your project before adopting the numbers.

| Item | Assumption |
|---|---|
| Host | Radxa CM-series module in CM4-compatible form factor (Rockchip SoC), PCIe 2.0 x1 on the B2B connectors |
| Target | M.2 2230/2242/2280 M-key socket, NVMe, PCIe x1 link |
| PCB | 4-layer, 1.6 mm, FR-4 (JLC7628-class stackup), 1 oz outer / 0.5 oz inner |
| Layers | L1 Sig, L2 GND (solid), L3 PWR/GND, L4 Sig |
| Data rate | 5 GT/s, 8b/10b. Nyquist = 2.5 GHz. Design for content to 7.5 GHz (3rd harmonic). |

---

## 1. Signal Inventory

| Net (host side) | M.2 M-key pin | Type | Notes |
|---|---|---|---|
| PCIE_TX_P / N (host TX) | 43 PERp0 / 41 PERn0 | Diff, 5 GT/s | Host TX → device RX |
| PCIE_RX_P / N (host RX) | 49 PETp0 / 47 PETn0 | Diff, 5 GT/s | Device TX → host RX |
| PCIE_CLK_P / N | 55 REFCLKp / 53 REFCLKn | Diff HCSL, 100 MHz | |
| PCIE_RST_L | 50 PERST# | 3.3 V CMOS | Host-driven |
| PCIE_CLKREQ_L | 52 CLKREQ# | Open-drain | 10 kΩ pull-up to 3.3 V |
| PCIE_WAKE_L | 54 PEWAKE# | Open-drain | Optional. Pull-up if used. |
| — | 69 PEDET | — | NC or GND per NVMe = PCIe |

**Naming trap.** M.2 pin names are from the device's point of view:
- Host TX goes to **PER** (pins 41/43).
- Host RX comes from **PET** (pins 47/49).

TX and RX must not be swapped. P/N inversion within a pair is tolerated by PCIe link training, but avoid it anyway for clean review.

---

## 2. Differential Routing Parameters

### 2.1 Impedance targets

| Pair | Z_diff | Tolerance | Rationale |
|---|---|---|---|
| TX / RX data | **90 Ω** | ±10 % | Matches the CM4-family carrier guideline. 85 Ω (PCIe CEM) is also acceptable; pick one and use it end-to-end. |
| REFCLK | **90–100 Ω** | ±10 % | HCSL is tolerant. Using 90 Ω keeps one geometry. |

### 2.2 Starting geometry (to confirm with the fab's impedance calculator)

The values below assume L1 over L2 GND with 0.21 mm 7628 prepreg (Er ≈ 4.4).

| Parameter | L1 / L4 microstrip |
|---|---|
| Trace width W | ≈ 0.18 mm |
| Intra-pair gap S | ≈ 0.20 mm |
| Pair-to-pair spacing | ≥ 5W (≥ 0.9 mm). TX↔RX ≥ 1.0 mm. |
| Pair to other signals | ≥ 5W. ≥ 1.5 mm from switching regulators and their inductors. |
| Pair to coplanar GND pour edge | ≥ 3W (≥ 0.55 mm), or model as coplanar. |
| Solder mask | Include in the calculation. It lowers Z by about 2–4 Ω. |

Order the boards with **controlled impedance**, stating "90 Ω diff, L1/L4 ref L2/L3". Lock the final W/S from the fab's solver, not from this table.

### 2.3 Length and skew

| Rule | Value |
|---|---|
| Intra-pair skew (P vs N), per pair | ≤ 0.13 mm (5 mil). Target ≤ 0.05 mm. |
| Skew compensation location | Within 5 mm of where the mismatch occurs (bends/pins), using small serpentine bumps with amplitude ≤ 2S and length ≥ 3W |
| TX vs RX inter-pair matching | **Not required** (independent lanes) |
| REFCLK vs data matching | Not required (common-clock architecture) |
| Max carrier trace length, B2B pad → M.2 pad | ≤ 100 mm. Target < 60 mm. |
| Insertion loss budget (carrier portion) | ≤ 3 dB at 2.5 GHz. ~0.3–0.4 dB per 10 mm FR-4 microstrip leaves ample margin. |

### 2.4 Topology rules

- Place the M.2 socket so the pairs run **straight from the CM B2B connector**. Prefer zero layer changes, routing everything on L1.
- Use 45° bends or arcs only. No 90° corners.
- Keep pairs symmetric through pads and bends. Fan out both legs together.
- No stubs or test points on data pairs. If probing is needed, use in-line pads sized ≤ trace width.
- Do not route under the M.2 standoff screw or the board-edge mounting hole keep-out.

### 2.5 AC coupling capacitors

| Lane | Location | Value |
|---|---|---|
| Host TX → device RX (PER) | Transmitter side | 100 nF (75–265 nF range), 0201 or 0402 X7R |
| Device TX → host RX (PET) | **On the SSD module** | No carrier caps |

**Check the Radxa module schematic.** RPi CM4 places host TX caps on the module itself. If the Radxa module does too, **omit them on the carrier**, because double caps are harmless but waste length and add discontinuities.

If carrier caps are needed:
- Place them as close as practical to the B2B connector, symmetric, side by side.
- Add a **GND plane cutout under the cap pads on L2**, sized to the pad footprint, to cancel excess pad capacitance. The reference then becomes L3, so L3 must be GND or stitched there.

---

## 3. Return Path and Via Stitching

### 3.1 Reference plane rules

1. Every high-speed pair must have **one continuous, unbroken reference plane** (L2 GND for L1 routing) from B2B to M.2.
2. **No pair may cross a plane split, slot, or void.** Check L2 under the B2B connector and M.2 pin field, where antipads often merge into slots. Fix by enlarging the spacing between via rows, or shrinking antipads to leave a GND web ≥ 0.15 mm between adjacent antipads.
3. If L3 is a power plane and a pair must use L4, the return switches L2→L3. Place a **100 nF stitching cap (0402) L3-PWR → GND within 1 mm** of each transition via pair, in addition to the GND vias below. Prefer L3 = GND in the pair's corridor.

### 3.2 Signal via transitions (if a layer change is unavoidable)

| Parameter | Value |
|---|---|
| Max layer changes per pair | 1. Zero for TX/RX preferred. |
| Signal via | 0.3 mm drill / 0.5–0.55 mm pad |
| Via pitch within pair | Match trace pitch as closely as possible (~0.8 mm) |
| Antipad on planes | Ø 0.8–0.9 mm. Or an oblong single antipad around the pair, ~1.0 × 1.8 mm, to tune via impedance toward 90 Ω. |
| **Return GND vias** | **2 per pair transition**, placed symmetrically: one beside each signal via, centre-to-centre ≤ 1.0 mm (target 0.7 mm) |
| Layer path | L1↔L4 through-hole (no stubs). Avoid L1→L3-type transitions, which leave stubs. |
| Via stub | None for through-hole L1↔L4. Backdrill not required at Gen2. |

### 3.3 Ground stitching along the channel

| Feature | Rule |
|---|---|
| GND pour on L1/L4 adjacent to pairs | Allowed only if fully stitched. Otherwise remove the pour near pairs. Floating or poorly stitched copper resonates. |
| Fence via pitch along pairs | **≤ 2.5 mm** (≈ λ/8 at 7.5 GHz in FR-4, λ ≈ 20 mm) |
| Fence offset from trace edge | ≥ 3W (~0.55 mm) to via pad edge |
| Pour island ends / slivers | Stitch every tip. Delete slivers < 1 mm wide. |
| Fence via size | 0.3 / 0.5 mm, tented |

### 3.4 Connector regions

| Region | Rule |
|---|---|
| **CM B2B connector** | Use all GND pins adjacent to the PCIe pins, each with its own via to L2/L3 (no shared vias). Add 1 extra GND via within 1 mm of each pair's breakout. |
| **M.2 socket** | Same rule: dedicated via for every GND pin (pins 39/45/51/57 bracket the PCIe pairs). Stitch the M.2 shield/standoff GND with ≥ 4 vias. |
| **Ground pins** | Never route a pair between a GND pin and its via. Keep GND pin stubs < 0.5 mm. |

### 3.5 Board-level stitching

- Use a perimeter fence of GND vias at ≤ 5 mm pitch around the board edge, tightened to ≤ 2.5 mm along the PCIe corridor.
- Stitch L2 and L3 GND areas together at ≤ 5 mm grid under the M.2 socket.
- Use the SSD's ground plane as a heat-spreading path as well.

---

## 4. REFCLK and Sideband

- REFCLK uses the same rules as data, except skew is ≤ 0.25 mm and length is ≤ 120 mm.
- Keep REFCLK ≥ 1.5 mm from TX/RX pairs. Do not run it parallel to them for more than 10 mm.
- REFCLK needs no termination on the carrier if the SoC uses an internal-termination HCSL-like output. **Confirm with the Radxa module datasheet.** If external termination is required, place 49.9 Ω to GND per leg at the source.
- PERST#, CLKREQ#, and PEWAKE# are low speed. Use ≥ 3W spacing from pairs.
- PERST# gets a 10 kΩ pull-down, or follows the host's default.
- CLKREQ# and PEWAKE# get 10 kΩ pull-ups to 3.3 V.

---

## 5. Power Delivery (affects return-path integrity)

- 3.3 V rail at **≥ 2.5 A** (NVMe peaks ~2–3 A). Pins 2/4/12/14/16/18/70/72/74.
- Use ≥ 1.0 mm traces or a pour, plus ≥ 4 vias.
- Bulk capacitance: 2 × 22 µF + 1 × 100 µF at the socket.
- Decoupling: 4 × 100 nF on the pins.
- Keep the 3.3 V regulator's switch node and inductor **≥ 10 mm** from the PCIe corridor, on the opposite side of the board if possible.

---

## 6. Cycle 14 Verification Checklist

| # | Check | Pass criterion |
|---|---|---|
| 1 | Fab impedance report | 90 Ω ±10 % on L1/L4 coupons |
| 2 | DRC diff-pair rule set | W/S/skew per §2 with zero violations |
| 3 | TX/RX pin mapping | Host TX → pins 41/43, host RX ← pins 47/49 |
| 4 | AC caps | Present exactly once per direction |
| 5 | Plane continuity | L2 under all pairs has no splits or slots (visual + plane-overlay check) |
| 6 | Transition vias | Each has 2 GND return vias ≤ 1 mm away |
| 7 | Fence pitch | ≤ 2.5 mm, no floating copper |
| 8 | Optional SI sim | Return loss < −10 dB to 2.5 GHz, insertion loss < 3 dB at 2.5 GHz (carrier only) |
| 9 | Bring-up | `lspci -vv` shows LnkSta Speed 5GT/s, Width x1. `fio` sequential read runs ~400 MB/s with no AER errors in `dmesg`. |

---

**Open items to confirm from the Radxa module documentation:**
1. Whether TX AC caps are on the module.
2. The module's recommended differential impedance (85 vs 90 Ω).
3. REFCLK termination requirements.
4. The ex