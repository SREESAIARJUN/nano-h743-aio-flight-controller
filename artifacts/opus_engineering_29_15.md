# M.2 NVMe PCIe Gen2 x1 Routing Specification: Radxa CM4 Expansion Board (Cycle 29)

## 0. Scope and Assumptions

| Item | Assumption (verify against your module datasheet) |
|---|---|
| Host | Radxa CM-series module in CM4-compatible footprint. PCIe Gen2 x1 on the 2×100-pin Hirose DF40 connectors |
| Target | M.2 M-key socket, 2230/2242/2280 NVMe (PCIe x1 used; lanes 1–3 NC) |
| Data rate | 5 GT/s, 8b/10b. Nyquist frequency is 2.5 GHz; design for content up to about 7.5 GHz (3rd harmonic) |
| PCB | 4-layer, 1.6 mm, FR-4 (Er ≈ 4.3–4.5 @ 1 GHz, Df ≈ 0.02) |
| Signals | TX0± (host→SSD), RX0± (SSD→host), REFCLK±, PERST#, CLKREQ#, PEWAKE# |

---

## 1. Stack-up and Impedance Targets

**Recommended 4-layer stack-up** (JLC04161H-7628 equivalent):

| Layer | Use | Thickness |
|---|---|---|
| L1 | PCIe pairs, connectors, components | 35 µm Cu (1 oz finished) |
| PP 7628 | — | ~0.21 mm |
| L2 | **Solid GND, no splits under PCIe** | 17.5 µm |
| Core | — | ~1.07 mm |
| L3 | Power (3.3 V M.2 rail, 5 V) | 17.5 µm |
| PP 7628 | — | ~0.21 mm |
| L4 | Low-speed, power, optional PCIe escape | 35 µm |

**Impedance targets:**

| Net class | Target | Tolerance | Starting geometry (L1 over L2, h ≈ 0.21 mm)* |
|---|---|---|---|
| PCIE_TX/RX | **90 Ω diff** (CM4 ecosystem convention; PCIe CEM allows 85 Ω) | ±10 % | W = 0.20 mm, S = 0.18 mm |
| PCIE_REFCLK | 90–100 Ω diff (HCSL) | ±10 % | Same as data pairs |
| Single-ended reference | ~50 Ω | ±10 % | — |

*Starting values only. Run your fab's impedance calculator or a 2D field solver (Saturn PCB, Polar SI9000) with their exact prepreg thickness and Er. Order controlled impedance with a coupon.

---

## 2. Differential Pair Routing Rules

| Parameter | Rule |
|---|---|
| Intra-pair skew (P vs N) | **≤ 0.13 mm (5 mil)** target; ≤ 0.25 mm max. Match at the end where the mismatch originates, not at the far end. |
| Inter-pair skew (TX vs RX) | Not required (independent embedded clocks). Keep lengths similar for tidiness only. |
| REFCLK vs data | No matching needed (common-clock architecture). |
| Max length, host connector → M.2 pad | **≤ 100 mm** recommended; Gen2 permits roughly 250 mm on FR-4, but shorter adds margin. |
| Pair-to-pair spacing (TX↔RX, edge to edge) | **≥ 5W** (≈1.0 mm) where they run in parallel. ≥ 3W absolute minimum. |
| PCIe to other aggressors (USB3, HDMI, ETH, SMPS) | **≥ 1.5 mm**. Never route under the SMPS inductor or switch node. |
| Distance to GND pour edge (coplanar) | ≥ 3W (≈0.6 mm), or leave no pour beside the pair. Do not let the pour accidentally form an untuned CPWG. |
| Distance to board edge / plane edge | ≥ 1.5 mm. The pair must keep ≥ 3× h of continuous L2 GND on each side. |
| Bends | 45° or arc only. No 90° corners. |
| Length tuning | Small serpentines with amplitude ≤ 2S, segment length ≥ 3W. Place them within 5 mm of the bend that caused the skew. |
| Pad entry | Symmetric fan-out into the DF40 and M.2 pads. Uncoupled length ≤ 1.5 mm per pair. |
| Neck-down | Allowed only within 1 mm of the connector pads. |
| Test points / stubs | **None** on TX/RX. |
| Layer | **Route entirely on L1 with zero vias** if placement allows (strongly preferred). |

### AC Coupling Capacitors
- **Host TX (CM→SSD):** The SSD has RX caps internally? No. Per PCIe CEM, TX caps sit at the transmitter. On CM4-class modules the host TX caps are normally **on the module**, so the carrier needs none. Confirm in the Radxa module schematic. If they are absent, place 220 nF 0201/0402 caps on the carrier near the module connector.
- **SSD TX (SSD→CM):** Caps are on the M.2 SSD per the M.2 spec. The carrier needs none.
- If caps are required, place them as a symmetric pair. Void L2 under the cap pads (antipad ≈ pad + 0.1 mm), or use 0201 parts to minimize the capacitive discontinuity.

---

## 3. Return Path and Via Stitching

### 3.1 Reference Plane
- L2 is **unbroken GND** under every PCIe pair from module connector to M.2 socket. No slots, splits, or via-antipad "swiss cheese" from adjacent via fields.
- If a PCIe pair must pass over an L3 power-split region, it doesn't matter, because L1 references only L2. Check this rule if you ever route on L4.

### 3.2 Signal Layer Transitions (if L1→L4 is unavoidable)
Use this only if necessary. L4 references L3 (power), so return current must jump planes.

| Item | Rule |
|---|---|
| Signal via | 0.3 mm drill / 0.6 mm pad. Plane antipad ≈ 0.9–1.0 mm, merged oval antipad around the pair. |
| **GND return via** | **One GND via per signal via, ≤ 0.6 mm center-to-center**, placed symmetrically (GND–P–N–GND pattern). |
| Plane-to-plane return | L4 references L3 (power). Add **one 100 nF 0402 stitching cap (3V3→GND) within 2 mm of each transition**, or rearrange the stack so the escape layer references GND. |
| Via count | ≤ 2 vias per lane end-to-end. Keep identical via counts on P and N. |
| Stub | L1→L4 through-via has no stub. Avoid L1→L3 transitions. |

### 3.3 Edge Stitching Fence

| Item | Rule |
|---|---|
| Where | Along any L1 GND pour that runs beside the PCIe pairs, and around the M.2 and module connector perimeter. |
| Pitch | **≤ 2.5 mm** (≈ λ/20 at the 7.5 GHz 3rd harmonic in FR-4, λ ≈ 20 mm). 2 mm is preferred near connectors. |
| Offset from pair | Via center ≥ 0.6 mm (3W) from the nearest trace edge. |
| Via size | 0.3 mm drill / 0.5–0.6 mm pad, tied L1–L2–L4 GND. |
| Isolated pour | Delete any copper island with fewer than 2 vias, or any island beyond the fence that could resonate. |

### 3.4 Connector Ground Transitions
- **DF40 module connector:** Every GND pin adjacent to PCIe pins gets its own via to L2 within 0.5 mm of the pad. Do not share vias between GND pins.
- **M.2 socket:** Give each GND pad between the lanes (pins 39, 45, 51, 57, etc.) a dedicated L2 via. Add a ground stitch row of 3–5 vias across the socket's front edge.
- **M.2 standoff / mounting nut:** Connect it to GND with 4 vias. It provides a chassis return and an ESD path.

---

## 4. Reference Clock and Sideband

| Signal | Rule |
|---|---|
| REFCLK± | 100 MHz HCSL, 90–100 Ω diff, skew ≤ 0.25 mm. Keep ≥ 1 mm from TX/RX. Same stitching rules apply. Leave termination as the module provides it (usually source-terminated on-module). |
| PERST# | 3.3 V CMOS. Add a 10 kΩ pull-up only if the module lacks one. Route as ordinary 50 Ω, kept away from pairs. |
| CLKREQ# | Open-drain. 10 kΩ pull-up to 3V3. Tie it to GND if the module cannot gate REFCLK (CM4 behaviour: often tied low so the clock is always on). Check the Radxa module. |
| PEWAKE# | Optional. 10 kΩ pull-up, or leave NC. |
| SUSCLK (M.2 pin 68) | NC or 32.768 kHz if available. |
| M.2 3V3 | ≥ 2.5 A capable (NVMe peaks of 2–3 A). Use 10 µF + 4×100 nF at the socket, plus a ≥ 22 µF bulk cap. A load switch with soft-start avoids module brown-out on hot plug. |

---

## 5. Loss Budget Sanity Check (Gen2 @ 2.5 GHz)

| Contributor | Est. loss |
|---|---|
| 100 mm microstrip FR-4 (~0.08 dB/mm @ 2.5 GHz incl. Cu) | ~0.8 dB |
| DF40 mezzanine connector | ~0.5 dB |
| M.2 connector | ~0.5 dB |
| 2 vias (if used) | ~0.2 dB |
| **Total carrier channel** | **≈ 2.0 dB** |

The Gen2 system budget is roughly 13 dB across the full channel, so there is large margin. **The real risks are reflections from return-path discontinuities and crosstalk, not insertion loss.** That is why sections 3–4 are the critical ones.

---

## 6. CAD Net-Class Setup (KiCad / Altium)

```
Net class: PCIE_DIFF  (PCIE_TX0_P/N, PCIE_RX0_P/N, PCIE_CLK_P/N)
  Track width        0.20 mm
  Diff pair gap      0.18 mm   (update after fab impedance calc)
  Clearance          0.60 mm   (to other nets / pour)
  Via                0.30/0.60 mm, max 2 per net
  Max uncoupled      1.5 mm
  Skew tolerance     0.13 mm
  Max length         100 mm
Rule: PCIE_DIFF to PCIE_DIFF (different pair) clearance 1.0 mm
Rule: Keepout — no L2 copper void intersecting PCIE_DIFF corridor ±1 mm
```

---

## 7. Cycle 29 Verification Checklist

- [ ] Fab impedance calculation confirms 90 Ω ±10 % with actual prepreg. Coupon ordered.
- [ ] PCIe routed on L1 only. If not, every transition has a GND via ≤ 0.6 mm away and a 100 nF stitch cap.
- [ ] L2 continuous under all pairs. Visually inspect L2 with all other layers hidden.
- [ ] Intra-pair skew ≤ 0.13 mm (length tuner report).
- [ ] AC-cap location confirmed against the Radxa module schematic, with no double caps.
- [ ] Stitching fence ≤ 2.5 mm pitch. No unstitched copper islands.
- [ ] TX↔RX ≥ 5W. PCIe ≥ 1.5 mm from USB3/HDMI/SMPS.
- [ ] CLKREQ#/PERST# pull-ups agree with module defaults.
- [ ] M.2 3V3 rail sized for 3 A, with soft-start.
- [ ] Bring-up: `lspci` shows link at **5 GT/s x1** (`LnkSta: Speed 5GT/s, Width x1`). Run `fio` for 30 min and check `dmesg` for AER correctable errors (should be 0).

If the board shows link-training fallback to 2.5 GT/s or AER errors, inspect in this order: return path at vias/connectors, skew at pad fan-out, AC-cap placement, then REFCLK integrity.