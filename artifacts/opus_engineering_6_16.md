# M.2 NVMe (PCIe Gen2 x1) Routing Specification: CM4-Form-Factor Carrier, Cycle 6

## 0. Scope and Assumptions

I don't have access to your Cycle 6 schematics, stackup or prior review notes. This spec therefore uses the following assumptions. Correct any that don't match your design.

| Item | Assumption |
|---|---|
| Module | Radxa CM3/CM3S-class SoM (RK3566) or RPi CM4 pin-compatible, on 2× Hirose DF40 100-pin |
| Interface | PCIe Gen2 x1 (5 GT/s, 8b/10b), Nyquist 2.5 GHz, design bandwidth to about 7.5 GHz (3rd harmonic) |
| Slot | M.2 Key-M socket, 2242/2280 support, lane 0 only |
| PCB | 4-layer, 1.6 mm, FR-4 (JLC7628-type): L1 sig / L2 GND / L3 PWR / L4 sig |
| Clock | 100 MHz HCSL REFCLK supplied by the SoM |

## 1. Signal Inventory and M.2 Pin Map

M.2 signal names are **from the module's (SSD's) perspective**. Host TX lands on PER; host RX comes from PET. Getting this backwards is the most common M.2 bring-up failure.

| Function | Host (SoM) side | M.2 Key-M pins | Notes |
|---|---|---|---|
| Host TX → SSD RX | PCIE_TX_P/N | **PERp0 = 43, PERn0 = 41** | AC caps: see §3 |
| SSD TX → Host RX | PCIE_RX_P/N | **PETp0 = 49, PETn0 = 47** | Caps are on the SSD card |
| Ref clock | PCIE_CLK_P/N | **REFCLKp = 55, REFCLKn = 53** | DC-coupled HCSL, no caps |
| Reset | PCIE_nRST | PERST# = 50 | 3.3 V, series 0–33 Ω optional |
| Clock request | PCIE_CLKREQ# | CLKREQ# = 52 | 10 k pull-up to 3.3 V (open-drain) |
| Wake | — | PEWAKE# = 54 | Pull-up, optional to GPIO |
| Power | 3.3 V | 2, 4, 12, 14, 16, 18, 70, 72, 74 | ≥3 A rail, see §7 |
| GND | — | All GND pins, including pins between pairs | Every one via'd to L2 |

Leave lanes 1–3 (pins 5–39 PER/PET 1–3) unconnected. Do not ground them.

## 2. Differential Pair Parameters

### 2.1 Impedance target

**90 Ω ±10 % differential**, which matches the CM4-family module guidance. This is within the PCIe base spec window of 85 Ω ±15 %. Use the same target for REFCLK. Single-ended impedance should be about 50 Ω.

### 2.2 Geometry (starting values, confirm with the fab's field solver)

| Layer | Reference | Dielectric h / εr | Trace W | Gap S | Zdiff (approx.) |
|---|---|---|---|---|---|
| L1 microstrip | L2 GND | 0.21 mm / 4.4 | 0.20 mm (7.9 mil) | 0.20 mm | 89–91 Ω |
| L4 microstrip | L3 (**must be GND under pairs**, see §5) | 0.21 mm / 4.4 | 0.20 mm | 0.20 mm | 89–91 Ω |

If you move to 6-layer (recommended for Cycle 7 if density allows), use L1 over L2 GND and L6 over L5 GND, with symmetric stripline only if loss budget permits.

Order a **controlled-impedance build with coupons** and specify "90 Ω diff, L1/L4, 0.20/0.20 mm" in the fab notes.

### 2.3 Routing rules

| Parameter | Rule |
|---|---|
| Intra-pair length match (P vs N) | ≤ 0.127 mm (5 mil). Compensate within 5 mm of the mismatch source using small serpentine bumps (amplitude ≤ 2S, length ≥ 3W) |
| Inter-pair match (TX vs RX vs CLK) | **Not required.** Lanes are independent and the clock is common-clock architecture |
| Max length, SoM connector → M.2 pin | ≤ 100 mm target, 150 mm hard limit. Gen2 loss budget at 2.5 GHz on FR-4 is about 0.08 dB/mm |
| Pair-to-pair spacing (TX↔RX) | ≥ 4W (0.8 mm). TX and RX routed on the same layer and parallel for more than 10 mm: ≥ 5W |
| Spacing to other signals | ≥ 5W (1.0 mm). To clocks, SDIO, or SMPS switch nodes: ≥ 2 mm |
| Bends | 45° or arc only. Bend each pair symmetrically |
| Uncoupled length (breakout) | ≤ 3 mm total per pair, at the connector fan-outs only |
| Via count | ≤ 2 per trace, i.e. one layer change. Zero is preferred: route TX/RX/CLK entirely on L1 |
| Stubs / test points | None on data pairs. If a probe point is required, use pads in-line on the trace, not on a stub |
| Polarity swap | PCIe tolerates P/N inversion, but don't rely on it. Route straight |
| Board edge | Keep pairs ≥ 1.5 mm from the board edge and ≥ 0.5 mm from the plane edge |

### 2.4 Via design (if a layer change is unavoidable)

| Parameter | Value |
|---|---|
| Drill / pad | 0.25 mm / 0.45 mm |
| Antipad on planes | 0.75–0.85 mm, preferably oblong (stadium) around both vias of the pair |
| Signal via pitch (P↔N) | 0.8–1.0 mm, symmetric |
| Stub | L1→L4 transition is through-hole with no stub, which is ideal. Avoid L1→L3 |

## 3. AC Coupling Capacitors

- **Host TX (to PER pins):** CM4-class modules typically place the TX AC caps **on the module**. Check the Radxa CM3 datasheet / hardware reference for your exact SoM. If the caps are on-module, put **no caps on the carrier**; a second set in series is harmless electrically but adds discontinuities.
- **If caps are needed on the carrier:**
  - Use 100 nF, X7R, **0201** (0402 is acceptable), placed side-by-side and symmetric, within 15 mm of the transmitter.
  - Make the trace width equal to the pad width where possible.
  - **Void L2 under the cap pads** (pad outline + 0.1 mm) to cancel the excess pad capacitance, and add 2 GND stitch vias at the void edges.
- **SSD TX (PET pins):** the M.2 card carries these caps. Put none on the carrier.
- **REFCLK:** DC-coupled. No caps. If the SoM spec requires source termination, the series resistors go at the source, not on the carrier.

## 4. High-Speed Return Path and Via Stitching

### 4.1 Principle

At 2.5–7.5 GHz the return current flows in the reference plane directly under the trace. Every place the reference changes (layer transitions, connector pins, plane edges) needs a low-inductance path for that current. Without it you get common-mode conversion, EMI and eye closure.

Wavelength at 7.5 GHz in microstrip (εeff ≈ 3.3) is about 22 mm. Stitching spacing is set to ≤ λ/10 ≈ 2 mm near the pairs.

### 4.2 Stitching rules

| Location | Rule |
|---|---|
| **Signal via transitions** | 1 GND via per signal via, i.e. **2 GND vias per pair**, placed symmetrically and ≤ 0.8 mm center-to-center from the signal vias. The GND vias connect L2 and L3 (both GND under the pair). |
| **SoM DF40 connector breakout** | Via every GND pin adjacent to the PCIe pins directly to L2, ≤ 0.5 mm from the pad. No shared vias between GND pins. |
| **M.2 socket breakout** | Via each GND pin flanking 41/43, 47/49, 53/55 (pins 39, 45, 51, 57) to L2 with dedicated vias, ≤ 0.5 mm. |
| **Coplanar ground pour on L1 near pairs** | Optional. If used, keep pour ≥ 3W (0.6 mm) from the trace edge (so it doesn't load Zdiff) and stitch its edge every **2 mm** to L2. Unstitched or floating copper slivers are worse than none: remove them. |
| **Along the route channel** | GND via fence (on both sides of the pair bundle) every 2–2.5 mm wherever the pair passes near the SMPS, the HDMI area, or the board edge. |
| **Plane-to-plane (L2↔L3 GND regions)** | Grid stitching ≤ 5 mm across the whole PCIe region. Board-edge stitching every 3 mm around the perimeter. |
| **M.2 standoff / mounting screw** | Plate the standoff pad to GND with ≥ 4 vias. It forms part of the card's return path. |

### 4.3 L3 handling (4-layer specific)

L3 is a power layer in this stackup, so any trace on L4 references a split-prone layer. Choose one of the following, in order of preference:

1. **Route all PCIe pairs on L1 only.** This is the preferred option and removes the L3 problem.
2. Flood a **solid GND island on L3** under the full L4 PCIe corridor (corridor width + 3 mm each side) and stitch it to L2 every 2 mm along both edges.
3. **Last resort only:** if the pair must reference a power plane, place a **stitching capacitor** (10 nF, 0201) GND↔3V3 within 1.5 mm of every via transition.

**Never** route a pair across a plane split or slot, including the L2 voids under the DF40 connectors. Check this in a plane-overlay view.

## 5. REFCLK Specifics

- 90 Ω diff, same geometry as the data pairs, intra-pair match ≤ 5 mil.
- Keep ≥ 1 mm (5W) from TX/RX pairs. A ground via fence between REFCLK and the data pairs is recommended.
- Do not run REFCLK under or near the 3.3 V M.2 regulator inductor. Keep ≥ 5 mm clearance.
- If CLKREQ# is not supported by the SoM driver, tie CLKREQ# low through 0 Ω (DNP option) so the clock runs continuously. Some SSDs won't link up otherwise.

## 6. Constraint-Manager / DRC Rule Set (KiCad/Altium)

```
Netclass PCIE_DIFF: PCIE_TX_P/N, PCIE_RX_P/N, PCIE_CLK_P/N
  Track width        0.20 mm
  Diff pair gap      0.20 mm
  Diff pair uncoupled max   3.0 mm
  Max skew (intra)   0.127 mm
  Clearance to other nets   1.0 mm
  Clearance PCIE↔PCIE pairs 0.8 mm
  Max vias           2
  Allowed layers     F.Cu (pref), B.Cu (only with L3 GND island)
