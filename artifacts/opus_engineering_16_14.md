# Radxa CM4 Expansion Board: M.2 NVMe (PCIe Gen2 x1) Routing and Return-Path Design

**Revision context:** Design Cycle 16. This is a self-contained specification. Reconcile it against your Cycle 15 constraint file before release.

---

## 1. Interface Summary

| Item | Value |
|---|---|
| Link | PCIe Gen2 x1, 5.0 GT/s, 8b/10b |
| Unit interval | 200 ps |
| Nyquist frequency | 2.5 GHz. Design for the 3rd harmonic at 7.5 GHz for return-path and via features. |
| Connector | M.2 Key M, 2280 (optionally 2242/2230 standoffs) |
| Diff pairs | 1× TX, 1× RX, 1× REFCLK (100 MHz HCSL) |
| Sideband | PERST#, CLKREQ#, PEWAKE# (3.3 V, open-drain where applicable) |
| Power | 3.3 V @ 3 A budget (Gen2 SSDs peak at roughly 2–2.5 A) |

> **Verify against the module datasheet:** confirm the module-side pin assignment and whether the **host TX AC-coupling capacitors are already on the CM4 module**. Many CM4-class modules place them on-module. If they are, do **not** add a second set.

---

## 2. M.2 Key M Pin Mapping (Lane 0)

M.2 signal names are from the **SSD (add-in card) perspective**:

| M.2 Pin | Signal | Connects to Host |
|---|---|---|
| 41 / 43 | PERn0 / PERp0 (SSD receive) | **Host TX** (via AC caps if not on module) |
| 47 / 49 | PETn0 / PETp0 (SSD transmit) | **Host RX** (no caps; SSD has them on-card) |
| 53 / 55 | REFCLKn / REFCLKp | Host REFCLK out (DC-coupled) |
| 50 | PERST# | Host GPIO/PERST, 10 k pull-down optional |
| 52 | CLKREQ# | Host CLKREQ, 10 k pull-up to 3.3 V |
| 54 | PEWAKE# | Host WAKE, 10 k pull-up (optional) |
| 2,4,12,14,16,18,70,72,74 | 3.3 V | Power plane/pour |
| GND pins | GND | All to solid L2 ground, one via per pad minimum |

- Lanes 1–3 (pins 5–37) are **not connected**. Leave them as NC pads. Do not route stubs to them.
- **P/N polarity inversion is allowed** by PCIe link training. You may swap P/N *within* a pair to avoid crossovers. Note this in the schematic.
- **Never swap TX with RX.**

---

## 3. Stack-up and Impedance Targets

### Recommended 4-layer stack (1.6 mm, JLC-style 7628 prepreg; adapt to your fab)

| Layer | Use |
|---|---|
| L1 | PCIe pairs (microstrip), components |
| L2 | **Solid GND**, unbroken under all PCIe routes |
| L3 | Power (3.3 V, 5 V), slow signals |
| L4 | Low-speed / escape. No PCIe here unless unavoidable. |

### Impedance targets

| Net class | Target | Tolerance | Starting geometry (L1 over L2, ~0.21 mm prepreg, εr ≈ 4.4) |
|---|---|---|---|
| PCIE_TX / PCIE_RX | **85 Ω diff** | ±10 % | W ≈ 0.20 mm, S ≈ 0.18 mm |
| PCIE_REFCLK | 85–100 Ω diff | ±10 % | Same as above (85 Ω is acceptable) |
| Single-ended reference | ~50 Ω | — | — |

- If your module's documentation specifies 90 Ω or 100 Ω, either is tolerable at Gen2 x1 for short runs. **Pick one value and keep it consistent end-to-end.**
- The geometries above are starting points only. Confirm with your fab's impedance calculator or a field solver, and order **controlled impedance with coupons**.

---

## 4. Differential Routing Rules

| Parameter | Rule |
|---|---|
| Intra-pair skew (P vs N) | ≤ **0.13 mm (5 mil)**. Compensate within 5 mm of the mismatch source using small serpentines. |
| Inter-pair skew (TX vs RX) | Not required (independent lanes) |
| REFCLK vs data | No matching required (common-clock tolerant) |
| Max length, module connector → M.2 | ≤ **100 mm** recommended; ≤ 150 mm absolute for this design |
| Pair-to-pair spacing (TX↔RX) | ≥ **4W** (~0.8 mm); ≥ 5W if parallel > 10 mm |
| Pair to other signals | ≥ **5W** (~1.0 mm); ≥ 2 mm from SMPS switch nodes and inductors |
| Pair to plane/pour edge | ≥ 3W, or ≥ 0.5 mm from the L2 void edge |
| Bends | 45° or arcs only. No 90° corners. |
| Serpentine | Amplitude ≤ 2S, segment ≥ 3W, spacing between bumps ≥ 4S |
| Layer changes | Max **2 vias per signal** (ideally 0 on L1-only routes) |
| Reference plane | Continuous L2 GND under the full route. **No split crossings, ever.** |
| Board edge | ≥ 1.5 mm from board edge |
| Breakout | Keep uncoupled length ≤ 2.5 mm at the connector and module pads. Fan out symmetrically. |

### AC-coupling capacitors (host TX → SSD PERp/n, only if absent on module)

- **Value and package:** 100 nF (75–265 nF allowed), X7R, **0201 preferred**, 0402 acceptable.
- **Placement:** symmetric, side by side, near the **transmitter end** or within ~10 mm of the connector. Be consistent.
- **Pad voids:** remove L2 copper under the cap pads (pad footprint + 0.1 mm). This offsets the pad capacitance dip. If L3 is not a plane, reference L3 instead.
- **Pad width:** keep the pad no wider than ~1.5× trace width where practical.

---

## 5. High-Speed Return-Path Via Stitching

### 5.1 Signal-via return vias (mandatory at every layer transition)

- Place **one GND via per signal via**, or at minimum 2 per pair.
- Locate them **≤ 0.8 mm (center-to-center)** from the signal vias, symmetric about the pair axis.
- Signal vias: 0.3 mm drill / 0.5 mm pad typical.
- **Antipad:** oval, shared antipad around both P and N vias on all GND layers. Size it at ~0.9 × 1.6 mm, then tune to ~85 Ω in a 3D solver if available.
- **Stub:** on 1.6 mm boards, a transition from L1 to L4 has no stub. Avoid L1→L3 transitions; the L3→L4 stub (~0.2 mm) is acceptable at Gen2, and backdrill is not needed.
- **Plane change:** if the reference changes (e.g., GND to the 3.3 V plane), add a **stitching capacitor** (100 nF 0201 between planes) within 1 mm. Better still, avoid the change entirely.

### 5.2 Corridor stitching fence

- **Pitch:** at 7.5 GHz in FR-4 (εeff ≈ 3.4), λ ≈ 21.7 mm. Use λ/10–λ/20, which gives **≤ 2.0 mm pitch** along any L1 GND pour adjacent to the pairs.
- **Clearance:** keep the pour ≥ 3W from the pair edges, and the fence ≥ 0.5 mm beyond the pour edge.
- **Coverage:** tie all coplanar GND pours to L2. Remove floating or dead copper islands near the pairs.
- **Simplest option:** if no L1 pour flanks the pairs, rely on solid L2 and skip the fence. That is preferable to an un-stitched pour.

### 5.3 Connector and module launch

- **M.2 GND pins:** each pin gets ≥ 1 via to L2 directly at the pad. The GND pins between pairs (pins 39, 45, 51, 57) get **two vias each**.
- **Module connector:** stitch the GND pins adjacent to the PCIe pins with ≥ 1 via each, ≤ 0.5 mm from the pad.
- **Ground perimeter:** run a stitched GND ring of 0.3 mm vias at 2–3 mm pitch around the M.2 connector footprint.

### 5.4 Under the SSD

- Keep L1 under the M.2 card as **GND pour stitched at 3–5 mm pitch**, or leave it bare. Do not route other signals there.
- **Standoff:** the M.2 mounting screw/standoff pad goes to GND and gets ≥ 4 vias. This provides an EMI return and thermal path.

---

## 6. REFCLK and Sideband

- **REFCLK:** 100 MHz HCSL. No series caps. Same pair rules as data, but intra-pair skew ≤ 0.25 mm is sufficient. Add termination only if the host's REFCLK driver requires it (most SoC outputs are internally terminated; check the module datasheet).
- **CLKREQ#:** 10 k pull-up to 3.3 V. Some hosts require it tied low if CLKREQ is not supported, so confirm.
- **PERST#:** route from the host GPIO. Add an optional 100 nF RC footprint (DNP) for power-on delay tuning.
- **Spacing:** keep sidebands ≥ 1 mm from the data pairs.

---

## 7. Power Delivery to M.2

- **Source:** a dedicated 3.3 V load switch or regulator rated ≥ 3 A. Do not take it from the module's 3.3 V output, which is usually current-limited.
- **Decoupling at the connector:**
  - 1× 22 µF + 2× 10 µF (0805/0603)
  - 4× 100 nF (0402), one near each 3.3 V pin cluster
- **Planes:** use a solid L3 3.3 V pour. It must not intrude under the PCIe pairs on L2, since L2 stays all GND.
- **Thermal:** the GND pour under the SSD with stitching provides heat spreading. Consider a thermal-pad keepout and clearance for a heatsink.

---

## 8. Cycle 16 Verification Checklist

- [ ] Module datasheet confirms TX AC-cap location; caps fitted only if needed
- [ ] TX↔PER, RX↔PET mapping verified (the M.2 naming trap)
- [ ] Fab stack-up confirmed; diff impedance coupon ordered (85 Ω ±10 %)
- [ ] DRC net classes: PCIE_DIFF (W/S/clearance), intra-pair skew ≤ 0.13 mm
- [ ] No L2 splits under any PCIe segment (check with a plane-only view)
- [ ] Every signal via has a GND return via ≤ 0.8 mm
- [ ] AC-cap pad voids present on L2
- [ ] Stitching fence ≤ 2 mm pitch on all adjacent pours; no floating copper
- [ ] Lanes 1–3 unconnected with no stubs
- [ ] Optional: run a 2.5D/3D S-parameter check. Target insertion loss ≥ −3 dB @ 2.5 GHz (board portion) and return loss ≤ −12 dB @ 2.5 GHz.
- [ ] Bring-up test: run `lspci -vv` and confirm LnkSta reports **5GT/s, Width x1**. A link at 2.5 GT/s indicates an SI problem.

---

If you share your actual fab stack-up or Cycle 15 constraint values, I can recompute exact trace geometries and flag any rule changes between cycles.