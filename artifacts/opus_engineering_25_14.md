# M.2 NVMe (PCIe Gen2 x1) Routing Spec: CM4-Form-Factor Expansion Board, Cycle 25

**Assumptions.** The board is a carrier for a CM4-pin-compatible Radxa module (CM3/CM3S/CM5 family) driving PCIe Gen2 x1 to an M.2 M-key slot. Two items must be confirmed against the module schematic before freeze (see §2 and §9):

- **Module AC caps:** whether the module already places AC-coupling caps on PCIe TX.
- **PHY mode:** whether the PCIe PHY is muxed with SATA/USB3. On RK3566, combo-PHY mode is fixed by the device tree.

---

## 1. Signal Inventory

| Net (carrier) | Module side | M.2 M-key pin | Type |
|---|---|---|---|
| PCIE_TX_P/N (host→SSD) | PCIe_TX_P/N | 41 / 43 (PERp0/PERn0) | 5 GT/s diff |
| PCIE_RX_P/N (SSD→host) | PCIe_RX_P/N | 47 / 49 (PETp0/PETn0) | 5 GT/s diff |
| PCIE_CLK_P/N | PCIe_CLK_P/N | 53 / 55 (REFCLKp/n) | 100 MHz HCSL |
| PCIE_nRST | PCIe_nRST | 50 (PERST#) | 3.3 V CMOS |
| PCIE_CLKREQ_N | PCIe_CLK_nREQ | 52 (CLKREQ#) | OD, pull-up 10 k to 3V3 |
| — | — | 54 (PEWAKE#) | Optional, OD |
| 3V3_M2 | — | 2,4,12,14,16,18,70,72,74 | Power, ≥2.5 A |

The M.2 naming is from the SSD's perspective. The host TX lands on the card's PERp/n. Do not swap.

---

## 2. AC Coupling

- **Host TX → SSD:** the M.2 spec requires the *card* to own AC caps on its own TX only. Host-TX caps (75–265 nF, typically 100 nF 0201/0402) must be on the module or the carrier.
  - The Raspberry Pi CM4 places these on-module. Verify the Radxa module schematic.
  - If they are absent, place 100 nF 0402 X7R on the carrier, within 10 mm of the M.2 connector, symmetric per pair.
  - Void L2 copper under the cap pads: a pad-sized cutout, or reference L3 GND if the stackup allows.
- **SSD TX → host RX:** no carrier caps (the caps are on the SSD).
- **REFCLK:** DC-coupled, no caps. The module drives HCSL with terminations already present; add no carrier termination unless the module guide says otherwise.

---

## 3. Stackup and Impedance

The target is a **4-layer, 1.6 mm build: Sig / GND / PWR / Sig.**

| Parameter | Target |
|---|---|
| Differential impedance | **85 Ω ±10%** (PCIe base spec). 90 Ω ±10% is acceptable and matches CM4 module routing; pick one and hold it across the channel. |
| Single-ended (reference) | ~45–50 Ω |
| Layer | L1 microstrip over solid L2 GND (preferred). L4 only if L3 is converted to GND under the route. |

Example geometry for a JLC-style 7628 prepreg (h ≈ 0.21 mm, εr ≈ 4.4, 1 oz finished):

| Target | Trace W | Gap S | Pitch |
|---|---|---|---|
| 90 Ω | 0.20 mm | 0.20 mm | 0.40 mm |
| 85 Ω | 0.22 mm | 0.18 mm | 0.40 mm |

These are starting points only. Re-run the fab's impedance calculator (or Polar/Saturn) with the real prepreg and solder-mask values. Mask lowers Zdiff by ~2–4 Ω. Order impedance-controlled with coupons on the PCIe pairs.

REFCLK uses the same diff geometry (85–100 Ω). It is not loss-critical but is jitter-sensitive.

---

## 4. Length, Skew, and Loss Budget

| Rule | Value |
|---|---|
| Intra-pair skew (P vs N) | **≤ 0.13 mm (5 mil)** per segment, matched at the connector end |
| Inter-pair (TX vs RX vs CLK) | No matching needed |
| Max carrier length, TX and RX | **≤ 100 mm** (target < 60 mm) |
| Max REFCLK length | ≤ 120 mm |
| Insertion loss, carrier portion @ 2.5 GHz | ≤ ~4 dB. FR4 microstrip is ≈ 0.03–0.04 dB/mm at 2.5 GHz, so 100 mm ≈ 3.5 dB plus vias/connectors. The full Gen2 channel budget is ~13 dB, and the module and SSD consume the rest. |

**Skew compensation:**
- Fix skew within 5 mm of the bend or pin that caused it, not at the far end.
- Use small "sawtooth" bumps on the short leg: bump height ≤ 2× gap, length ≥ 3× trace width, and ≥ 4W between bumps.
- Avoid long accordion serpentines on Gen2 lanes.

**Bends:** 45° or arc only, no 90° corners. Keep the pair symmetric through bends.

---

## 5. Spacing and Keep-Outs

| Item | Clearance |
|---|---|
| Pair to pair (TX↔RX) | ≥ 4× trace width edge-to-edge (~0.8–1.0 mm). Route TX and RX on opposite sides of a GND guard if they run parallel > 20 mm. |
| Pair to other signals | ≥ 5W (~1.0 mm). ≥ 1.5 mm from clocks, SDIO, Ethernet MDI, and switching regulators. |
| Pair to board edge | ≥ 1.0 mm, with GND pour and via fence between pair and edge |
| Pair to copper pour (same layer) | ≥ 3W (~0.6 mm). Otherwise it becomes coplanar and Zdiff drops. |
| Under the route | Solid L2 GND, with no splits, slots, via antipad chains, or plane necks |
| Test points | None on TX/RX. If needed, use an inline 0.6 mm pad with no stub. |

---

## 6. High-Speed Return Path: Via and Stitching Design

### 6.1 Layer transitions
- **Limit:** ≤ 2 signal vias per lane end-to-end. The ideal is 0 (module connector and M.2 connector both on top).
- **Signal via:** 0.3 mm drill, 0.55 mm pad. Antipad on L2/L3 is a merged oblong around both P and N, 0.25–0.3 mm larger than the pad.
  - The antipad must not cut a reference plane under any other pair.
- **Stubs:** if routing L1→L4 on 1.6 mm, there is no stub. If L1→L3 were ever used, the stub is ~1.4 mm. That is acceptable at 5 GT/s but avoid it.
- **GND return vias:**
  - Place **2 per diff via pair**, one on each side, mirror-symmetric to the P/N axis.
  - Keep them **≤ 1.0 mm center-to-center** from the signal vias (target 0.8 mm).
  - These carry return current between the L2 and L3/L4 references.
- **If the new reference plane is PWR (L3):** convert L3 to GND under the PCIe corridor, or add a 100 nF 0402 stitching cap GND↔PWR within 2 mm of the transition. Plane conversion is strongly preferred.

### 6.2 Corridor stitching (via fence)
- **Wavelength:** in FR4 microstrip (εeff ≈ 3.3), λ is ≈ 33 mm at 5 GHz and ≈ 22 mm at 7.5 GHz (3rd harmonic of 2.5 GHz).
- **Fence spacing:** ≤ λ/10 at 7.5 GHz. Use a **2.0–2.5 mm** pitch along both sides of the pair group where it runs next to top-layer GND pour.
- **Fence position:** keep fence vias ≥ 3W from the pair edge so the via does not perturb impedance.
- **Open areas:** where there is no top-side pour, no fence is needed. Instead, place stitching vias at **≤ 5 mm** grid across the L1/L4 GND pours near the route.

### 6.3 Connector return paths
- **M.2 connector:** give every GND pin flanking the pairs (pins 39, 45, 51, 57) its own via to L2, ≤ 0.5 mm from the pad. Pins 33, 39, 45, 51, 57 are the relevant GNDs; none may share a via.
- **Module board-to-board connectors (Hirose DF40):** stitch every GND pin adjacent to PCIe pins with a dedicated L2 via. Add 2 extra GND vias at each end of the connector footprint near the PCIe pins.
- **M.2 standoff/screw pad:** tie to GND with ≥ 4 vias. It is part of the card's return and ESD path.

### 6.4 Plane discontinuity rules
- No route over an L2 split.
- When unavoidable, cross at 90°, with two 100 nF stitching caps bridging the gap ≤ 1 mm from each trace.
- This should not occur in Cycle 25. Treat any instance as a DRC error.

---

## 7. Sideband, Clock, and Power

- **CLKREQ#:** 10 k pull-up to 3V3. Some modules/OS need CLKREQ# tied low or ASPM-L1 substates disabled. Provide a 0 Ω option to GND.
- **PERST#:** route directly. Add an optional 0402 pad (100 nF to GND, DNP) for slow-ramp debug.
- **REFCLK:**
  - Keep it ≥ 1 mm from TX/RX pairs and away from the 3V3 buck inductor.
  - Use the same GND via-fence rules as §6.2.
- **3V3_M2:**
  - Source a ≥ 3 A rail. A dedicated buck or load switch with soft-start is recommended, since NVMe peaks reach 2.5 A+.
  - Decoupling at the connector: 2× 22 µF + 4× 100 nF.
  - Pour the rail on L3. Do not route it under the PCIe pairs on L4.
  - Keep the buck SW node ≥ 10 mm from all PCIe nets and REFCLK.
- **Hardware strap:** pin 69 (PEDET) floating / NC for an NVMe-only slot. Pin 38 (DEVSLP) is NC.

---

## 8. DRC / Constraint-Manager Rules

```
NetClass PCIE_DIFF  : TX_P/N, RX_P/N
  Zdiff 85Ω (or 90Ω), W=0.22, S=0.18 (L1) – verify per stackup
  Uncoupled length max 2.5 mm (connector breakout only)
  Intra-pair skew ≤ 0.13 mm
  Max length 100 mm, max vias 2
  Clearance to other nets 1.0 mm; to same class (other pair) 0.8 mm

NetClass PCIE_CLK   : CLK_P/N
  Zdiff 85–100Ω, skew ≤ 0.25 mm, max length 120 mm, clearance 1.0 mm

Rule: no PCIE_* segment over L2 void/split
Rule: each PCIE_* via has ≥ 2 GND vias within 1.0 mm
```

---

## 9. Cycle 25 Sign-Off Checklist

- [ ] Module schematic confirms on-module TX AC caps. If not, add carrier caps per §2.
- [ ] Fab impedance stackup received; W/S updated; coupons ordered.
- [ ] P/N polarity and TX↔RX direction verified at M.2 pins 41/43/47/49. Lane polarity inversion is auto-handled by PCIe, but don't rely on it.
- [ ] Skew report ≤ 5 mil per pair; lengths within budget.
- [ ] Return-via audit: every signal via and every connector GND pin is stitched.
- [ ] L2 continuity check, with the plane-split overlay reviewed.
- [ ] DTS/overlay enables the PCIe controller. On RK3566-class modules, also set combo-PHY to PCIe mode.
- [ ] Bring-up plan:
  - Confirm `lspci` link at 5 GT/s x1 (`LnkSta`).
  - Run sustained `fio` and watch AER errors in `dmesg`.
  - Optionally measure TDR on the coupon.

If you provide your actual stackup (prepreg type and thicknesses) and the specific Radxa module variant, I can tighten the W/S numbers and the loss budget to exact values.