# M.2 NVMe PCIe Gen2 Routing and Return-Path Design for a Radxa CM4 Expansion Board (Cycle 2)

## 0. Scope and Assumptions

| Item | Assumption (verify against your module's datasheet) |
|---|---|
| Link | PCIe Gen2, x1 lane, 5 GT/s, 8b/10b encoding. Nyquist is 2.5 GHz. Design for content up to about 7.5 GHz (3rd harmonic). |
| Host | Radxa CM4 through the CM4-style 2×100-pin high-density connectors |
| Device | M.2 2230/2242/2280 M-key NVMe SSD, socket 3 |
| PCB | 4-layer, 1.6 mm. Stackup: SIG / GND / PWR / SIG |
| Clock | 100 MHz HCSL REFCLK, sourced by the module |

"Cycle 2" means this revision tightens the Cycle 1 rules. It focuses on impedance control, via transitions, and return-path continuity, which are the usual causes of Gen2 link training failures and fallback to Gen1.

---

## 1. Net List and Pin Mapping (M.2 M-Key, Lane 0)

| Function | M.2 pins | Notes |
|---|---|---|
| PERn0 / PERp0 | 41 / 43 | Differential pair |
| PETn0 / PETp0 | 47 / 49 | Differential pair |
| REFCLKn / REFCLKp | 53 / 55 | Differential, 100 MHz |
| PERST# | 50 | 3.3 V, open drain from host |
| CLKREQ# | 52 | Pull-up 10 kΩ to 3.3 V (or tie low if the host has no CLKREQ support) |
| PEWAKE# | 54 | Pull-up 10 kΩ, optional |
| SUSCLK | 68 | NC, or 32.768 kHz |
| 3.3 V | 2, 4, 12, 14, 16, 18, 70, 72, 74 | All pins connected |
| GND | All GND pins | Each pin gets its own via to L2 |

**Cycle 2 check: TX/RX direction.** The M.2 PET/PER names are easy to cross. The host transmitter must land on the SSD receiver, and the SSD transmitter must land on the host receiver. Confirm this against the Radxa reference carrier schematic before release. P/N polarity inversion *within* a pair is tolerated by PCIe link training. A TX↔RX swap is not.

---

## 2. Stackup and Impedance

PCIe Base specifies **85 Ω differential**. Gen2 also works well at 100 Ω ±10% for short carrier runs. Cycle 2 targets **85 Ω ±10%** for spec compliance.

Recommended: JLC 4-layer 1.6 mm with 3313 prepreg (L1–L2 ≈ 0.10 mm, εr ≈ 4.1).

| Parameter | Value (estimate; confirm with the fab's impedance calculator) |
|---|---|
| Structure | Edge-coupled microstrip, L1 over L2 GND |
| Trace width W | ~0.13 mm (5.1 mil) |
| Gap S | ~0.15 mm (6 mil) |
| Z_diff target | 85 Ω ±10% |
| Single-ended Z0 | ~48–50 Ω |
| Copper | 1 oz outer, ENIG finish preferred |
| Order option | Specify impedance control on the fab order |

If you use 7628 prepreg (~0.21 mm), widen to about W 0.22 mm / S 0.18 mm. Recalculate rather than reusing these numbers.

**REFCLK** uses the same 85 Ω geometry. HCSL tolerates 85–100 Ω.

---

## 3. Differential Pair Routing Rules

| Rule | Cycle 2 value |
|---|---|
| Intra-pair skew (P vs N) | ≤ 0.127 mm (5 mil). Compensate within 5 mm of the mismatch source, using small serpentine bumps (amplitude ≤ 2·S). |
| Inter-pair skew (TX vs RX vs CLK) | Not required |
| Max carrier length (module connector → M.2 pad) | ≤ 100 mm. Target ≤ 60 mm. |
| Layer changes | ≤ 2 vias per pair (0 preferred). Route fully on L1 if possible. |
| Pair-to-pair spacing | ≥ 4·W (≥ 0.5 mm). TX-to-RX ≥ 5·W, to reduce NEXT. |
| Spacing to other signals | ≥ 5·W (≥ 0.65 mm). ≥ 1.5 mm to switching nets (DC-DC, clocks, USB). |
| Distance to plane/board edge | ≥ 1 mm from the edge of the L2 reference |
| Bends | 45° or arc only. No 90° corners. Bend symmetrically. |
| Pad entry | Symmetric fan-out, uncoupled length ≤ 1.5 mm |
| Test points | None on the pairs. Use stitching-free probe access only at the AC caps if needed. |
| Stubs | None. No branching, no unpopulated footprint stubs. |

---

## 4. AC Coupling

- **SSD → host:** the SSD already carries its own TX coupling capacitors. Do not add any on the carrier.
- **Host → SSD:** check whether the Radxa CM4 module places its TX caps on-module. Raspberry Pi CM4-style modules typically do.
  - If yes, use a direct connection.
  - If no, place **2× 220 nF, X7R, 0201 or 0402** in series on the TX pair, close to the module connector.
    - Mount the caps side by side, not staggered.
    - Cut a **GND void on L2 directly under the cap pads**, sized to the pad plus 0.1 mm. This offsets the excess capacitance of the pads.

---

## 5. Return Path and Via Stitching (Cycle 2 Focus)

### 5.1 Reference plane continuity
- L2 must be **solid GND under the entire route**, from the module connector to the M.2 socket.
- No splits, slots, or via-antipad chains under the pairs. If several vias sit near a pair, confirm that their antipads don't merge into a slot.
- Do not route the pairs over L3 power split boundaries if any segment uses L4.

### 5.2 Signal via transitions (only if a layer change is unavoidable)

| Parameter | Value |
|---|---|
| Signal via | 0.3 mm drill / 0.5 mm pad |
| Antipad (L2/L3) | 0.9–1.0 mm diameter, oblong around the pair. Single shared antipad for both vias. |
| P/N via pitch | ~0.8 mm, kept symmetric |
| **Return vias** | **2 GND vias per pair**, one beside each signal via, ≤ 1.0 mm center-to-center |
| Return-via shape | Mirror-symmetric (GND–P–N–GND pattern) |
| Reference change L2(GND) → L3(PWR) | Avoid. If unavoidable, add a 100 nF 0201 stitching cap between PWR and GND within 2 mm of the via. |
| Stub | Prefer transitions L1→L4, which leave no via stub on a through-hole board |

### 5.3 Ground stitching fence and pour
- Pour L1 GND alongside the pairs with **≥ 3·W clearance** (≥ 0.4 mm) from the pair edges.
- Stitch that pour to L2 with a via row every **2.0 mm** along the route. This is below λ/10 at 7.5 GHz in FR4, where λ ≈ 19 mm.
- Remove copper slivers and isolated pour fingers. They act as resonant stubs.

### 5.4 Connector regions
- **M.2 socket:** give every GND pin its own via to L2, placed within 0.5 mm of the pad. Add 4–6 extra GND vias around the PCIe pin group (pins 39–57).
- **Module board-to-board connector:** use one GND via per GND pin. Add extra GND vias between the TX, RX, and REFCLK pin pairs where pin pitch allows.
- **Standoff/mounting hole:** tie it to GND. This gives a low-impedance chassis return near the SSD.

---

## 6. Power Integrity (Affects Signal Quality)

- **3.3 V rail:** ≥ 3 A capability, since NVMe peaks at 2.5 A or more.
  - Use a dedicated buck regulator, or a load switch from the carrier's 3.3 V supply.
  - Use a soft-start load switch (e.g., TPS22918/AP22653), so SSD inrush current doesn't brown out the module.
- **Decoupling at the M.2 socket:** 1× 47 µF + 2× 10 µF + 4× 100 nF. Place the 100 nF caps within 3 mm of the 3.3 V pins.
- **3.3 V routing:** pour on L3 or wide on L4. Keep the DC-DC inductor and switch node ≥ 10 mm from the PCIe pairs.

---

## 7. Sideband Signals

- PERST#: series 33 Ω near the source. Optionally add 100 kΩ to GND so the SSD stays in reset during module boot.
- CLKREQ#, PEWAKE#: 10 kΩ pull-ups to 3.3 V.
- Route sideband signals ≥ 1 mm away from the differential pairs.

---

## 8. Cycle 2 Changes vs Cycle 1 and Sign-Off

| # | Cycle 2 action | Done |
|---|---|---|
| 1 | Switch to an impedance-controlled stackup; target 85 Ω and order with impedance control | ☐ |
| 2 | Intra-pair skew ≤ 5 mil, verified in the DRC length report | ☐ |
| 3 | Via count ≤ 2 per pair, each with 2 GND return vias ≤ 1 mm away | ☐ |
| 4 | Stitching fence at 2 mm pitch along the full route | ☐ |
| 5 | L2 GND void under AC-cap pads (if caps are present) | ☐ |
| 6 | No plane split or merged antipad under any pair (check in the L2 plane view) | ☐ |
| 7 | TX/RX direction cross-checked against the reference schematic | ☐ |
| 8 | 3.3 V load switch and bulk caps sized for ≥ 3 A | ☐ |
| 9 | Optional: TDR coupon on the panel edge | ☐ |

## 9. Bring-Up Validation

1. Run `lspci -vv` and check that `LnkSta` reports **Speed 5GT/s, Width x1**. If it reports 2.5GT/s, the link is marginal and has fallen back to Gen1.
2. Run `fio` sequential read for 10 minutes or more. Expect ~400 MB/s (Gen2 x1 practical ceiling) with no AER errors in `dmesg`.
3. Check `/sys/bus/