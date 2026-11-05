# M.2 NVMe (PCIe Gen2 x1) Routing and Return-Path Design: Radxa CM4-Form-Factor Carrier, Cycle 21

## 0. Scope and Assumptions

| Item | Assumption (verify against your module datasheet) |
|---|---|
| Host | Radxa CM-series module in the CM4 form factor (e.g., CM3 RK3566), PCIe 2.x x1 on the 2× 100-pin Hirose DF40 connectors |
| Link | 1 lane, 5 GT/s (Gen2), 8b/10b. Nyquist = 2.5 GHz. Design bandwidth to ~7.5 GHz (3rd harmonic) |
| Target | M.2 Key-M socket (2230/2242/2280), NVMe SSD |
| AC coupling | Host TX caps are on the module, and SSD TX caps are on the SSD. **The carrier needs no series caps.** Confirm this on the module schematic; if it lacks TX caps, add 220 nF 0201 on the PET pair near the module connector. |
| Board | 4-layer, 1.6 mm, FR-4 (Dk ≈ 4.1–4.4 @ 1 GHz) |

---

## 1. Stackup (Cycle 21 Baseline)

| Layer | Use | Thickness |
|---|---|---|
| L1 | PCIe pairs, components | 35 µm Cu (after plating) |
| PP | 3313 prepreg | ~0.10 mm |
| **L2** | **Solid GND (no splits under PCIe)** | 17.5 µm |
| Core | | ~1.26 mm |
| L3 | Power (3V3_M2, 5V, 1V8) | 17.5 µm |
| PP | 3313 prepreg | ~0.10 mm |
| L4 | Low-speed signals, GND pour | 35 µm |

**Rule:** route all PCIe signals on L1, referenced to L2. Avoid L4 for PCIe because it references the L3 power plane, which forces the return current through decoupling caps.

---

## 2. Differential Pair Parameters

| Parameter | Value |
|---|---|
| Differential impedance | **90 Ω ±10%** (85 Ω also acceptable; the PCIe base spec uses 85 Ω and CM4-class modules specify 90 Ω) |
| Trace width / gap (L1, 0.10 mm PP) | **≈0.13 mm / 0.15 mm**, a starting point only. Confirm with the fab's impedance calculator and order impedance control on the pairs. |
| Pair-to-pair spacing (TX to RX) | ≥ 3× trace-to-plane height (≥ 0.5 mm). Use ≥ 5× where routed in parallel for > 10 mm. |
| Pair to other signals | ≥ 0.75 mm. Use ≥ 1.0 mm from switching nodes (buck SW, clocks). |
| Pair to GND pour edge (coplanar) | ≥ 0.4 mm, or remove the L1 pour along the pair |
| Intra-pair skew (P/N) | **≤ 0.13 mm (5 mil)**. Compensate within 5 mm of the mismatch source. |
| Inter-pair skew (TX vs RX) | No requirement (independent lanes) |
| REFCLK pair | 90–100 Ω diff (HCSL), same geometry. Intra-pair skew ≤ 0.13 mm. |
| Max length (module connector to M.2) | Target **≤ 75 mm**, hard limit 120 mm |
| Bends | 45° or arcs. No 90° corners. Skew serpentines with amplitude ≤ 2× gap and segment length ≥ 3× width. |
| Vias per net | Target 0 (all on L1). Max 2 per net, symmetric for P/N. |

### Channel Loss Budget (Carrier Share at 2.5 GHz)

| Element | Loss |
|---|---|
| 75 mm FR-4 microstrip (~0.05 dB/mm @ 2.5 GHz) | ~0.4 dB |
| DF40 module connector | ~0.3 dB |
| M.2 connector | ~0.5 dB |
| **Carrier total** | **~1.2 dB**, well under the ~6–8 dB available after module and SSD |

Gen2 margin is comfortable. The real risks are discontinuities and return-path breaks, not loss.

---

## 3. Net List and Pin Handling

| Signal | Direction | Notes |
|---|---|---|
| PCIE_TX_P/N (module) → PERp0/n (M.2 pins 47/49) | Host → SSD | Polarity inversion is permitted by spec, but keep it straight |
| PETp0/n (M.2 pins 41/43) → PCIE_RX_P/N (module) | SSD → Host | |
| REFCLKp/n (M.2 pins 53/55) | Host → SSD | 100 MHz HCSL. Termination is on module or SoC, so add none on the carrier unless the datasheet says so. |
| PERST# (pin 50) | Host → SSD | 3.3 V. 10 k pull-down so the SSD stays in reset until the host releases it. |
| CLKREQ# (pin 52) | SSD → Host | Open drain. 10 k pull-up to 3V3. Tie to GND if the module has no CLKREQ input. |
| PEWAKE# (pin 54) | SSD → Host | Optional. 10 k pull-up. |
| SUSCLK (pin 68) | | NC, or 32.768 kHz if available |
| PEDET (pin 69) | | Leave per NVMe (SSD grounds it for PCIe) |
| 3V3 (pins 2/4/12/14/16/18/70/72/74) | | Up to 2.5 A peak. Use 2× 22 µF + 4× 100 nF at the socket and a ≥ 1.5 mm wide feed or L3 plane. |

---

## 4. High-Speed Return Path and Via Stitching

### 4.1 Signal-Via Return Vias (Mandatory)
Every place a PCIe pair changes layer, the return current also changes reference plane.

- Place **1 GND via per signal via, minimum 2 per pair** (one beside P, one beside N, mirror-symmetric).
- Keep them ≤ 0.6 mm center-to-center from the signal via.
- Via geometry: 0.3 mm drill / 0.55 mm pad. Antipad on L2/L3 is ~0.85 mm diameter, as an oblong shared antipad around both P/N vias.
- If a pair transitions L1↔L4 (avoid), the reference changes GND→PWR. Then also add a **100 nF 0201 stitching cap (3V3 to GND) within 1 mm** of the via pair, in addition to the GND vias.

### 4.2 Connector Transitions
- **Module DF40 connector:** pin out the pairs between connector GND pins. Drop a GND via directly at every GND pad adjacent to the PCIe pins (≤ 0.5 mm from pad). No shared vias between pads.
- **M.2 socket:** each GND pin flanking the pairs (pins 39, 45, 51, 57) gets its own via at the pad. Add 2 extra GND vias at each end of the socket body.

### 4.3 Coplanar Stitching Fence (When L1 GND Pour Is Used)
- Spacing ≤ λ/10 at 7.5 GHz in FR-4: λ ≈ 3e8 / (7.5e9 × √4.2) ≈ 19.5 mm, so **pitch ≤ 2.0 mm**. Use **1.5 mm** for margin.
- Offset the fence ≥ 0.4 mm from the pair edge, one row each side, running the full length.
- Do not leave any L1 GND copper island floating or connected at only one end. Remove fingers narrower than 0.5 mm.

### 4.4 Reference-Plane Integrity
- L2 is continuous under the pair plus **≥ 3× H (≥ 0.3 mm, use 1 mm) on each side**.
- No slots, via-antipad chains, or splits cross under the pairs. Check the DF40 and M.2 fan-out via fields on L2 so they do not merge antipads into a slot.
- No L3 power-plane splits directly beneath the pairs. L2 is the reference, but L3 discontinuities still couple through the thin stack.
- Keep pairs ≥ 2 mm from the board edge, or add an edge via fence at ≤ 1.5 mm pitch.

### 4.5 General Board Stitching
- Stitch L1/L4 GND pours to L2 on a ≤ 5 mm grid board-wide.
- Use a ≤ 3 mm grid within 10 mm of the M.2 socket and module connectors.

---

## 5. Placement Guidance
- Orient the M.2 socket so PET/PER pins face the module connector's PCIe pins, giving straight runs with no crossing.
- If polarity must swap for routability, swap P/N within a pair (Gen2 link training handles lane polarity) rather than adding vias.
- Place the 3V3 buck for the SSD ≥ 10 mm from the pairs, with its SW node on the opposite side of the L2 plane.
- Put the M.2 standoff at a GND-connected mounting hole, stitched with 6–8 vias around the ring.

---

## 6. Cycle 21 Sign-Off Checklist

| # | Check | Pass Criterion |
|---|---|---|
| 1 | Fab impedance report | 90 Ω ±10% on L1 coupon |
| 2 | P/N skew per pair | ≤ 0.13 mm |
| 3 | Length module→M.2 | ≤ 75 mm |
| 4 | Signal vias | 0 (or symmetric with 2 GND return vias each) |
| 5 | L2 under pairs | Continuous, no antipad slots (visual + DRC) |
| 6 | Fence pitch | ≤ 1.5 mm, both sides |
| 7 | AC-cap location | Confirmed on module, none duplicated on carrier |
| 8 | PERST#/CLKREQ# pulls | Present and correct |
| 9 | 3V3 at socket | Droop < 5% under 2.5 A load step |
| 10 | Bring-up | `lspci -vv` shows `LnkSta: Speed 5GT/s, Width x1`; zero AER errors after a 30 min `fio` run |

Optional for higher confidence: run a TDR on the coupon and 2-port S-parameter extraction (insertion loss ≤ 1.5 dB, return loss ≤ −15 dB to 2.5 GHz).