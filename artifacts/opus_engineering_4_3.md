# M.2 NVMe (PCIe Gen2 x1) Routing and Return-Path Design

**Radxa CM4-class carrier, Revision "Cycle 4"**

## 0. Assumptions (verify before layout)

| Item | Assumption | Verify against |
|---|---|---|
| Host link | PCIe Gen2 x1 (5 GT/s) from the module's B2B connector | Radxa module datasheet / pinout |
| TX AC-coupling | Host TX caps are **on the module**. SSD TX caps are on the SSD. | Module schematic. If they are absent, add 220 nF 0201 on carrier HOST-TX. |
| REFCLK | 100 MHz HCSL from the module, common-clock | Module datasheet |
| Slot | M.2 M-key, 2230/2242/2280 | Mechanical drawing |
| Stackup | 4-layer, 1.6 mm, FR-4 (JLC-style 7628) | Fab's impedance calculator |

## 1. Stackup and Reference Planes

```
L1  Signal (PCIe pairs, top)     35 µm Cu
    Prepreg 7628   h ≈ 0.21 mm   εr ≈ 4.4 (≈4.0–4.1 effective @ 2.5 GHz)
L2  GND (solid, NO splits under PCIe)
    Core ≈ 1.07 mm
L3  PWR (3V3 / 5V pours)
    Prepreg ≈ 0.21 mm
L4  Signal / low-speed
```

**Rule:** Route all PCIe pairs on **L1, referenced to L2 GND, for the full length.** Avoid layer changes. If one is unavoidable, use L4, accepting the L3 PWR reference only if stitching caps are present (see §4.4). Routing L4 against L2 through the thick core is not usable for impedance.

## 2. Differential Pair Parameters

| Parameter | Value | Notes |
|---|---|---|
| Differential impedance (TX, RX) | **85 Ω ±10%** | PCIe CEM target. 90–100 Ω is tolerable at Gen2 for <100 mm, but 85 Ω matches the M.2 connector and SSD. |
| Differential impedance (REFCLK) | 85–100 Ω ±10% | HCSL is tolerant. Using the same geometry as the data pairs is fine. |
| Starting geometry (L1/L2, h = 0.21 mm) | **W ≈ 0.19 mm, S ≈ 0.20 mm** (edge-coupled microstrip) | Starting point only. Lock W/S with the fab's calculator and order controlled impedance. |
| Pair-to-pair spacing | ≥ 3·W edge-to-edge (≥ 0.6 mm). TX↔RX ≥ 5·W (≥ 1.0 mm). | Limits NEXT between TX and RX |
| Pair to other signals / clocks | ≥ 1.0 mm. Keep away from SDIO, Ethernet, and switching nodes. | |
| Pair to plane/pour edge | ≥ 3·h (≥ 0.65 mm) to coplanar pour, or design as CPWG | Do not let top-layer GND pour wander close to the pair |
| Intra-pair skew (P vs N) | **≤ 0.127 mm (5 mil)**. Compensate within 15 mm of the mismatch source. | Use small serpentine bumps: amplitude ≤ 2·S, length ≥ 3·W |
| Inter-pair skew (TX vs RX vs CLK) | Not required | Lanes are independent. The clock is not data-aligned at Gen2. |
| Max length, host connector → M.2 | **≤ 100 mm** recommended (practical limit about 200 mm on FR-4 at Gen2) | Gen2 loss at 2.5 GHz on FR-4 is about 0.03–0.05 dB/mm |
| Vias per net | ≤ 2 (target 0) | |
| Bends | 45° or arcs. No 90° bends. Keep P/N symmetric through bends. | |
| Breakout | Uncouple for ≤ 3 mm at pins only. Keep the uncoupled section equal on P and N. | |

**Loss budget sanity check (Gen2 channel ≈ −13 dB @ 2.5 GHz incl. both ends):**
80 mm × 0.04 dB/mm ≈ 3.2 dB of trace loss, plus about 1 dB for the connector and the B2B header. This leaves ample margin.

### Polarity and lane notes

- Connect host TX to M.2 **PERp0/PERn0** (SSD RX) and M.2 **PETp0/PETn0** (SSD TX) to host RX.
- PCIe tolerates P/N polarity inversion on data lanes, but do not use it as a routing shortcut on REFCLK.
- Leave lanes 1–3 of the M.2 connector NC.

## 3. Sideband and Power (non-SI but required)

| Signal | Treatment |
|---|---|
| PERST# | From host GPIO/PERST. 10 k pull-down optional so the SSD stays in reset during boot. |
| CLKREQ# | 10 k pull-up to 3V3. Route to host if supported, otherwise tie low through 0 Ω (DNP option). |
| PEWAKE# | 10 k pull-up, optional. |
| 3V3 to M.2 | Dedicated rail rated ≥ **2.5 A** (NVMe peaks of about 2–3 A). Bulk 2× 22 µF plus 4× 100 nF at the connector. Pour width ≥ 2 mm. |
| SUSCLK | NC or 32.768 kHz if available. |
| LED / DAS | Optional. |

## 4. Return Path and Via Stitching

### 4.1 Reference continuity

- L2 must be **unbroken** beneath every pair, plus 3·h on each side.
- No splits, slots, or via-antipad chains crossing the pair path.
- Check that THT connector or mounting-hole antipads do not cut under the route.

### 4.2 Signal-via return (where a layer change is unavoidable)

- Place **2 GND vias per differential via pair**, symmetric, centre-to-centre ≤ **1.0 mm** (target 0.6–0.8 mm) from the signal vias.
- Signal via: 0.3 mm drill / 0.5 mm pad. Antipad on planes about 0.9–1.0 mm, shared oval antipad around the P/N pair.
- Place P and N vias side by side, matching each other's position, to preserve coupling.
- Stub: on a 1.6 mm board with L1→L4 transitions, stubs are minimal. No back-drill is needed at Gen2.

### 4.3 Stitching fence and pour stitching

- Highest frequency of concern: about 5th harmonic of the 2.5 GHz Nyquist, so 12.5 GHz. λ in FR-4 is about 12 mm, and λ/10 gives a spacing of about 1.2 mm.
- **Spacing:**
  - Along the PCIe corridor edges (if a top-layer GND pour is present): place vias every **≤ 2.5 mm** (λ/20 at 6 GHz is a practical floor).
  - Rest of board: L1/L4 GND pours stitched to L2 on a grid of about **5 mm**.
- **Isolated copper:** delete any top-layer GND copper islands with fewer than 2 vias.

### 4.4 Connector and plane transitions

- **Module B2B connector:** surround the PCIe pin field with GND vias. Place 1 GND via per GND pin, directly at the pad, ≤ 0.5 mm away.
- **M.2 connector:** place a GND via at every M.2 GND pad adjacent to PET/PER/REFCLK pins (pins 4x/5x region), ≤ 0.5 mm from the pad.
- **Reference swap to L3 PWR (if forced):** place a **100 nF + 10 nF 0201 stitching capacitor pair** between 3V3 and GND within **2 mm** of each transition via.

## 5. Placement

1. Place the M.2 connector as close to the host connector as mechanics allow, so the trace runs straight.
2. Put the standoff/screw at 2242/2280 positions with a GND-tied nut.
3. Keep the SSD thermal zone clear of the PCIe corridor on L1.
4. Keep switching regulators (5V→3V3 buck) **≥ 10 mm** from the pairs. Do not route the inductor/SW node over or under the pairs.

## 6. Cycle-4 Verification Checklist

- [ ] Controlled impedance ordered: 85 Ω diff on L1/L2, with the coupon or the fab's report attached.
- [ ] DRC net-class "PCIE": W/S locked, 5-mil skew rule, max 2 vias, max length 100 mm.
- [ ] Visual check for no reference split under the pairs (L2 inspection with only the PCIe nets highlighted).
- [ ] Every signal via has ≥ 2 GND vias within 1 mm.
- [ ] AC-cap ownership confirmed. No double caps, no missing caps.
- [ ] CLKREQ#/PERST# defaults are correct, so the SSD is held in reset until the host is ready.
- [ ] 3V3 M.2 rail meets 2.5 A with < 3% droop. Bulk capacitance is at the connector.
- [ ] **Bring-up:** `lspci -vv` shows `LnkSta: Speed 5GT/s, Width x1`. Run `fio` sequential read (about 400+ MB/s expected at Gen2 x1) with no AER errors in `dmesg`.

If you share the fab stackup or the board outline and connector positions, I can tighten the exact W/S numbers and give a length estimate for your placement.