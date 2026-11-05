# M.2 NVMe (PCIe Gen2 x1) Routing and Return-Path Stitching Design

**Radxa CM4 Expansion Board, Design Cycle 8**

> **Assumptions.** The PCIe values below come from the PCIe Base/CEM and M.2 specifications. Confirm all CM4 board-to-board connector pin numbers, and whether AC-coupling caps are already on the module, against the current Radxa CM4 hardware design guide and schematic before release. All trace geometry must be re-solved in your fab's impedance calculator for the actual stackup.

---

## 1. Interface Summary

| Parameter | Value |
|---|---|
| Link | PCIe Gen2, x1 lane, 5.0 GT/s, 8b/10b |
| Nyquist frequency | 2.5 GHz (design for clean behavior to ~7.5 GHz, 3rd harmonic) |
| Connector | M.2 Socket 3, M-key, 2242/2280 mounting |
| Signals | TX pair, RX pair, REFCLK pair (100 MHz HCSL), PERST#, CLKREQ#, PEWAKE# |
| Power | 3.3 V, ≥2.5 A continuous rated (3 A recommended for Gen3/4 SSDs running at Gen2) |

### M.2 M-key pin map (lane 0, named from the SSD's perspective)

| M.2 Pin | Signal | Connects to (host / CM4) |
|---|---|---|
| 41 / 43 | PERn0 / PERp0 (SSD RX) | CM4 PCIe **TX** n/p, through AC caps on the carrier |
| 47 / 49 | PETn0 / PETp0 (SSD TX) | CM4 PCIe **RX** n/p (SSD supplies its own AC caps) |
| 53 / 55 | REFCLKn / REFCLKp | CM4 PCIe REFCLK n/p |
| 50 | PERST# | CM4 PCIe reset GPIO (3.3 V, open-drain-compatible) |
| 52 | CLKREQ# | CM4 CLKREQ, or pull down to GND if unused (10 kΩ pull-up if used) |
| 54 | PEWAKE# | Optional, 10 kΩ pull-up to 3.3 V |
| 2,4,12,14,16,18,70,72,74 | 3.3 V | Power plane |

> **Common error:** host TX connects to M.2 **PER** pins. Double-check the swap at the netlist level, not just visually.

---

## 2. Stackup (Recommended: 6-Layer)

| Layer | Function | Notes |
|---|---|---|
| L1 | Signal (PCIe, short breakout) | Microstrip, referenced to L2 |
| L2 | **GND (solid)** | Primary reference; no splits under PCIe |
| L3 | Signal (PCIe main route) | Stripline between L2 and L4 |
| L4 | **GND** | Second reference for L3 |
| L5 | Power (3.3 V M.2, 5 V, etc.) | |
| L6 | Signal / low-speed | |

**4-layer fallback** (SIG / GND / PWR / SIG): route all PCIe on L1 over solid L2 GND, with **zero layer changes**. Never route PCIe on L4, which references the power plane.

---

## 3. Differential Routing Parameters

### 3.1 Impedance targets

| Net class | Target Zdiff | Tolerance | Notes |
|---|---|---|---|
| PCIE_TX / PCIE_RX | **85 Ω** | ±10% | PCIe spec value; 90–100 Ω is acceptable for short Gen2 runs if your fab only offers that |
| PCIE_REFCLK | **100 Ω** (HCSL) | ±10% | Single-ended 50 Ω |

### 3.2 Example geometry

These are starting points only. Re-solve with your fab's calculator (e.g. JLC, Polar Si9000).

| Stackup case | Layer | Width | Gap | Zdiff (approx.) |
|---|---|---|---|---|
| 6L, ~0.1 mm prepreg L1→L2 (εr ≈ 4.1) | L1 microstrip | 0.10 mm (4 mil) | 0.15 mm | ~85 Ω |
| 6L, L3 stripline (~0.2 mm to each plane) | L3 | 0.10 mm | 0.18 mm | ~85 Ω |
| 4L JLC04161H-3313 (0.099 mm PP) | L1 | 0.11 mm | 0.15 mm | ~88 Ω |
| REFCLK, 6L L1 | L1 | 0.09 mm | 0.20 mm | ~100 Ω |

### 3.3 Length and skew rules

| Rule | Value |
|---|---|
| Intra-pair skew (P vs N) | **≤ 5 mil (0.127 mm)** |
| Skew compensation location | Within 0.5 mm of the mismatch source (pin or bend), using small serpentine bumps |
| Inter-pair matching (TX vs RX) | **Not required** for PCIe; each direction is independent |
| REFCLK intra-pair skew | ≤ 5 mil |
| Max total length, carrier only | Target **< 100 mm** (Gen2 budget is generous; shorter is better) |
| Include module-side length | Add the CM4 internal trace length from the Radxa length table, if published, for P/N skew |

### 3.4 Spacing and geometry rules

| Rule | Value |
|---|---|
| Pair-to-pair spacing (TX↔RX) | ≥ **3W**, preferably 5W |
| PCIe to any other signal | ≥ **5W** (≥ 0.5 mm) |
| PCIe to REFCLK | ≥ 5W |
| Distance to plane/reference edge | ≥ **3× dielectric height** from any split or void edge |
| Bends | 45° or arcs; no 90° corners |
| Stubs | None. Test points must sit in-line on the pad, not on a branch |
| Breakout of uncoupled region at pins | ≤ 2.5 mm |
| Max vias per pair (each net) | **2**, i.e. one layer transition |
| Polarity inversion | Allowed by the PCIe spec (auto-detected), but avoid it to keep debugging simple |

### 3.5 AC coupling capacitors (TX pair only)

| Item | Value |
|---|---|
| Value | **100 nF**, X7R, ±10% (spec range 75–265 nF for Gen2) |
| Package | **0201** preferred, 0402 acceptable |
| Location | Closer to the M.2 connector *or* the CM4 connector, but symmetric between P and N; never in the middle of a serpentine |
| Placement | P/N caps side by side, aligned; no stagger |
| Pad anti-pad | **Void L2 GND under the cap pads** (pad size + ~0.1 mm) and reference L3/next GND to correct pad capacitance. This is optional at Gen2, but recommended for 0402 |
| Check | If the CM4 module already includes TX caps, **do not add a second set**. Use 0 Ω only if you need a footprint for options; otherwise leave them out |

RX pair: no caps on the carrier. The SSD includes its own TX caps.

---

## 4. High-Speed Return Path and Via Stitching

### 4.1 Return-path principles

- Every PCIe segment must have an **unbroken GND reference directly beneath it** (L2, or L2/L4 for stripline).
- No PCIe trace may cross a plane split, slot, or anti-pad chain (e.g. via rows on connector fields).
- Return current follows the reference plane. At a layer change it needs a nearby GND via to jump between reference planes.

### 4.2 Signal-via transition stitching

For every differential via pair that changes layers (L1→L3):

| Parameter | Value |
|---|---|
| GND return vias | **2 per pair**, one on each side, symmetric about the pair (4 for best results) |
| Distance from signal via center | **≤ 0.5 mm (20 mil)**, target 0.4 mm |
| Signal via | 0.2 mm drill / 0.4 mm pad (or 0.25/0.45 per fab capability) |
| Anti-pad | Oblong/shared anti-pad around the P/N pair, ~0.25 mm clearance; tune for ~85 Ω via |
| Via pitch (P to N) | Match the trace pitch as closely as possible (~0.25–0.3 mm with fab permitting) |
| Stub | L1→L3 on a 6L, 1.6 mm board leaves a ~1.2 mm stub. That is acceptable at Gen2 (resonance well above 10 GHz). Back-drill is not required |
| Unused pads | Remove non-functional internal pads on signal vias |

```
   (G)            (G)
     \            /
      [P]    [N]          G = GND stitch via, ≤0.5 mm
     /            \       P/N = signal vias in shared oblong anti-pad
   (G)            (G)
```

### 4.3 Via fencing along routes

| Location | Rule |
|---|---|
| Along L1 microstrip runs | Optional at Gen2. If coplanar GND pour is used, fence at **≤ 2.5 mm pitch**, pour ≥ 3W from the pair |
| Around M.2 connector PCIe pins | GND vias at the connector GND pins (M.2 pins 39, 45, 51, 57) placed **directly on/adjacent to the pad**, ≤ 0.3 mm |
| Around CM4 connector PCIe pins | Each GND pin flanking the PCIe pins gets its own via, no shared via |
| Board edge / outline | GND via fence at **≤ 3 mm pitch** around the perimeter, tying all GND layers |
| Between TX and RX groups | Optional GND guard with vias every ≤ 2.5 mm if spacing < 5W is unavoidable |
| L2↔L4 plane stitching (general) | Grid of GND vias at **≤ 5 mm** across the PCIe routing region |

> λ/10 check: at 7.5 GHz in FR-4 (εeff ≈ 3.5), λ ≈ 21 mm, so λ/10 ≈ 2.1 mm. A 2–2.5 mm fence pitch is therefore appropriate. 5 mm is fine for general plane stitching at Gen2.

### 4.4 AC cap and connector return path

- Place a GND via within **0.5 mm** of each cap pair end if the pair changes reference at the cap void.
- The M.2 connector shell/standoff mounting must connect to GND with ≥ 2 vias per pad.
- The M.2 mounting screw standoff connects to GND.

---

## 5. REFCLK and Sideband

| Net | Rule |
|---|---|
| REFCLK | 100 Ω diff, ≤ 5 mil skew, same layer and spacing rules as data; route away from switching regulators by ≥ 5 mm |
| REFCLK termination | HCSL source termination is usually inside the SoC/module. Check the Radxa guide; no carrier termination unless specified |
| PERST# | 3.3 V; add 10 kΩ pull-down if the CM4 drives it via GPIO, so the SSD stays in reset during boot |
| CLKREQ# | Tie to GND through 0 Ω, or connect to the CM4 if ASPM L1 substates are used |
| PEWAKE# | 10 kΩ pull-up to 3.3 V; optional connection |
| SUSCLK (pin 68) | Optional 32.768 kHz; leave NC if unsupported |

---

## 6. M.2 Power Delivery

| Item | Value |
|---|---|
| Rail | 3.3 V dedicated buck or high-current LDO-less rail, ≥ 3 A |
| Bulk caps | 2× 22 µF 0805 + 1× 100 µF (polymer or 1206) near the connector |
| Decoupling | 4× 100 nF 0402 at the 3.3 V pin clusters |
| Plane | 3.3 V pour on L5 under the M.2 area; ≥ 2 vias per power pin |
| Load switch (optional) | Allows SSD power cycling from GPIO; ramp ≥ 1 ms |
| Keep-out | Buck inductor ≥ 10 mm from PCIe/REFCLK; no switching node under PCIe routes |

---

## 7. Cycle 8 Verification Checklist

- [ ] Netlist: CM4 TX → M.2 PERp0/PERn0 (43/41); CM4 RX ← M.2 PETp0/PETn0 (49/47)
- [ ] Confirm whether AC caps are on the module or carrier; no doubling
- [ ] Fab stackup confirmed; 85 Ω / 100 Ω geometries from the fab calculator inserted into net classes
- [ ] Intra-pair skew ≤ 5 mil, including package/module length if available
- [ ] ≤ 2 vias per net; every transition has ≥ 2 GND vias within 0.5 mm
- [ ] No PCIe over splits, voids, or power-plane references
- [ ] Fence/stitching pitch ≤ 2.5 mm near routes, ≤ 5 mm general
- [ ] Cap pad GND voids applied (if 0402)
- [ ] 3.3 V M.2 rail ≥ 3 A with bulk and decoupling placed
- [ ] Fab impedance coupons ordered (85 Ω and 100 Ω diff)
- [ ] Bring-up: `lspci -vv` shows LnkSta 5 GT/s x1; run `fio` sequential read (~400 MB/s expected ceiling for Gen2 x1) and check AER error counters stay at 0

---

## 8. Constraint Manager Summary

```
NET CLASS PCIE_DATA:   Zdiff=85Ω±10%, W=0.10mm, Gap=0.15mm(L1)/0.18mm(L3),
                       skew≤0.127mm, maxVias=2, clearance=0.5mm
NET CLASS PCIE_REFCLK: Zdiff=100Ω±10%, W=0.09mm, Gap=0.20mm, skew≤0.127mm,
                       maxVias=2, clearance=0.5mm
STITCH RULES:          transition GND via ≤0.5mm (×2 min), fence ≤2.5mm,
                       plane stitch