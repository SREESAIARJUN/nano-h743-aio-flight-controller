# M.2 NVMe (PCIe Gen2 x1) Routing and Return-Path Design for a CM4-Form-Factor Carrier

> **Assumptions.** "Cycle 24" looks like an internal revision tag I don't have context for, so this is a standalone design package. Both the Radxa CM3 and the Raspberry Pi CM4 expose **one PCIe Gen2 lane (5 GT/s)** through the high-density connectors, so this design targets a single lane plus REFCLK to an **M.2 M-key** socket.
>
> Before you commit to layout, verify the following against your module's datasheet: pin numbers, AC-coupling cap location, and CLKREQ# behavior.

---

## 1. Signal Inventory

| Signal | Source → Sink | M.2 M-key pins | Notes |
|---|---|---|---|
| PCIe TX (host) | SoM TX± → SSD **PERp0/PERn0** | 43 / 41 | Host TX caps are usually on the SoM (CM4: yes). Confirm for Radxa CM3. |
| PCIe RX (host) | SSD **PETp0/PETn0** → SoM RX± | 49 / 47 | The SSD carries its own TX caps, so no carrier caps are needed. |
| REFCLK± | SoM → SSD REFCLKp/n | 55 / 53 | 100 MHz HCSL. Route as a differential pair. |
| PERST# | SoM → SSD | 50 | 3.3 V. Add a 10 kΩ pull-down or follow the SoM recommendation. |
| CLKREQ# | SSD ↔ SoM | 52 | Open-drain with a 10 kΩ pull-up to 3.3 V. Tie low if the SoM lacks it. |
| PEWAKE# | SSD → SoM | 54 | Optional. Pull up to 3.3 V. |
| 3.3 V | — | 2, 4, 12, 14, 16, 18, 70, 72, 74 | Provide ≥2.5 A capable supply (~3 A recommended for NVMe peaks). |

**Polarity and naming:**

- PCIe tolerates **P/N polarity inversion** within a pair (link training corrects it). Use this to avoid crossovers and vias if needed, but document it.
- Lane reversal does not apply to an x1 link.
- Watch the naming crossover: host **TX** connects to SSD **PER** (the SSD's receive pair). This is the most common schematic error.

---

## 2. Stackup (4-Layer Recommended)

| Layer | Use |
|---|---|
| L1 | PCIe pairs, REFCLK, components |
| L2 | **Solid GND** (no splits under any HS route) |
| L3 | Power (3.3 V / 5 V pours) + low-speed signals |
| L4 | Low-speed signals, GND fill |

- Use a **thin prepreg** between L1 and L2 (~0.1 mm, e.g., a 3313-type prepreg). This gives narrow traces, tight coupling to the reference plane, and low crosstalk.
- Avoid thick 7628 prepreg (~0.21 mm) if possible. It forces wide traces that are hard to fan out of the M.2 and SoM connectors.

---

## 3. Differential Routing Parameters

| Parameter | Target | Rationale |
|---|---|---|
| Differential impedance (TX/RX) | **85 Ω ±10%** (90 Ω acceptable) | PCIe base spec recommends 85 Ω. Most SoM/SSD designs tolerate 85–100 Ω. |
| REFCLK impedance | 85–100 Ω diff | HCSL is forgiving. Match the SoM vendor's guidance. |
| Starting geometry (L1, ~0.1 mm to L2, εr≈4.1, 1 oz) | W ≈ 0.15 mm / S ≈ 0.15 mm (≈5/5 mil) | **This is a starting point only.** Confirm with the fab's impedance calculator or a field solver and order controlled impedance. |
| Intra-pair skew (P vs N) | **≤ 0.13 mm (5 mil)** | Compensate within 5 mm of the mismatch source, using small serpentines near the bend that caused it. |
| Inter-pair skew (TX vs RX) | Not required | Lanes are independent and the receiver recovers the clock. |
| REFCLK vs data length | Not required | Common-clock architecture. |
| Max length (connector-to-connector) | Keep **< 100 mm**. Hard limit ~150 mm on FR4. | Gen2 Nyquist frequency is 2.5 GHz. FR4 loss is ~0.5–0.8 dB/in there, so 100 mm uses little of the channel budget, leaving margin for the SoM and SSD. |
| Pair-to-pair spacing | **≥ 3W** (≥ 0.5 mm). Use ≥ 5W between TX and RX if they run parallel > 10 mm. | Limits NEXT between TX and RX. |
| Spacing to other signals | ≥ 5W (≥ 0.75 mm). ≥ 1 mm from clocks and switching nodes. | Keep away from SMPS inductors and the SW node entirely. |
| Board-edge clearance | ≥ 1 mm and ≥ 3× dielectric height from plane edge | Prevents impedance rise and edge radiation. |
| Bends | 45° or arcs. No 90° corners. | Use symmetric bends to minimize skew generation. |
| Vias per pair | **0 preferred, ≤ 2 max** | Keep the whole route on L1 if the connectors allow. |
| Via geometry (if used) | 0.2–0.25 mm drill, minimum antipad (~0.6–0.7 mm on GND). Back-drill unnecessary at 5 GT/s on 1.6 mm boards. | Stub resonance is well above Gen2 frequencies. |
| Test points / stubs | None on TX, RX, or REFCLK | Any stub degrades the eye. |
| Serpentine tuning | Amplitude ≤ 2S, segment length ≥ 3W, coupled-pair bumps | Avoid sawtooth tuning that breaks coupling. |

**AC coupling (only if your SoM lacks TX caps):**

- Use 100 nF (75–265 nF per spec), X7R, 0201 or 0402, placed near the **transmitter** side.
- Place the two caps side-by-side.
- Void L2 GND under the pads (cutout ≈ pad footprint) to cancel the pad's excess capacitance.
- Keep the pair symmetric through the caps.

---

## 4. High-Speed Return Path and Via Stitching

### 4.1 Reference Plane Rules

- Route every HS segment over **unbroken L2 GND**:
  - No splits, slots, or via-antipad "swiss cheese" under the pair.
  - Keep the pair ≥ 3W from any plane-edge void.
- Never route over the 3.3 V/5 V split on L3. This is irrelevant if L2 is solid GND and the pair stays on L1, which is why that arrangement is preferred.

### 4.2 Signal Via Transitions (If Unavoidable)

- Place **one GND stitching via per signal via**, ≤ 0.5–1.0 mm (center-to-center) from it. That means 2 GND vias per differential via pair, placed symmetrically (e.g., one on each side, aligned with the pair axis).
- If the pair changes reference plane (e.g., L1/L2 GND to L4/L3 PWR), add a **10 nF + 100 nF** stitching cap between the planes within 2 mm of the transition. Avoiding this case is better.
- Use an oval/shared antipad around both P and N vias. Do not let antipads merge with neighboring antipads and cut the return path.

### 4.3 Connector Regions

| Location | Stitching requirement |
|---|---|
| SoM high-density connector GND pins | A GND via directly at every GND pin adjacent to HS pins (via-in-pad or ≤ 0.3 mm dogbone). |
| M.2 socket GND pins (3, 9, 15, 21, 27, 33, 39, 45, 51, 57, 71, 73) | Pins 39, 45, 51, 57 flank the HS pairs. Give each its own GND via ≤ 0.5 mm away. |
| M.2 standoff/screw mount | Bond it to GND with ≥ 4 vias. This forms a chassis/return path for the SSD. |

### 4.4 Ground Fill and Fencing on L1

- If you pour L1 GND near the pairs:
  - Keep it ≥ 3W from the pair. Otherwise the line becomes coplanar and the impedance shifts, so recalculate.
  - Stitch it at **≤ 2.5 mm pitch** along the pair. That is roughly λ/20 at the 5th harmonic (12.5 GHz, λ≈12 mm in FR4).
  - Remove isolated copper islands.
- A via fence along long parallel routes is optional at Gen2 but cheap. Use 2.5–3 mm pitch, 1 mm off the outer trace.

### 4.5 Board-Level Stitching

- Use a GND-via grid of **≤ 5 mm pitch** across all GND pours, and **≤ 3 mm** within 10 mm of the PCIe path and the M.2 socket.
- Run a board-perimeter via ring at ≤ 3 mm pitch to suppress plane-edge resonance and EMI.

---

## 5. Power and Sideband Notes Affecting SI

- **M.2 3.3 V decoupling:** use 1× 22 µF + 2× 10 µF bulk near the socket, plus 100 nF at the pin clusters. Put the power path on L3, not under the PCIe pairs.
- **Buck converter placement:** keep the 3.3 V buck (if dedicated) ≥ 15 mm from the PCIe route. Orient the SW node away from it.
- **Slow sidebands:** PERST#, CLKREQ#, and PEWAKE# are slow. Route them on L3/L4, away from the pairs.

---

## 6. Verification Checklist

1. Order controlled-impedance fabrication (85 Ω diff on L1). Request a test coupon or TDR report.
2. DRC rules:
   - Diff-pair gap and width are locked.
   - Intra-pair skew ≤ 5 mil.
   - Pair-to-pair clearance ≥ 0.5 mm.
3. Visual return-path audit: overlay L1 routes on L2 and confirm zero voids under the pairs.
4. Confirm every HS signal via has a GND via within 1 mm.
5. Confirm TX→PER and PET→RX mapping in the schematic. Note any intentional P/N swaps.
6. Bring-up:
   - Run `lspci -vv` and check `LnkSta: Speed 5GT/s, Width x1`.
   - Check for AER errors under sustained `fio` load.
7. If the link trains at 2.5 GT/s only, suspect excess loss or skew. Check the cap placement, then reduce vias.

---

If you share your actual stackup (fab, prepreg type, copper weight) and the SoM's PCIe pin map, I can tighten the trace and gap numbers and propose a concrete fan-out for the M.2 and SoM connector footprints.