# M.2 NVMe (PCIe Gen2 x1) Routing Spec: Radxa CM4-Form-Factor Carrier, Cycle 18

**Assumptions to confirm before you commit them to the design:**

- **Module:** The host is a Radxa CM4-pinout module, such as CM3 (RK3566) or ROCK CM-series, exposing **PCIe 2.0 x1** on the Hirose DF40 connectors (Raspberry Pi CM4 compatible pinout).
- **Board:** 4-layer, 1.6 mm, 1 oz outer copper. A 6-layer variant is noted where relevant.
- **Connector:** M.2 **M-key**, 2242/2280 SSD.

"Cycle 18" carries no prior context on my side. Treat this as a self-contained baseline and diff it against your Cycle 17 rules.

---

## 1. Electrical Targets

| Parameter | Value | Notes |
|---|---|---|
| Data rate | 5 GT/s (8b/10b) | Nyquist 2.5 GHz. Design for content to ~7.5 GHz (3rd harmonic). |
| Differential impedance (TX/RX) | **90 Ω ±10%** | Compromise between module-side 90 Ω and PCIe-native 85 Ω. Either is acceptable at Gen2. Stay consistent end to end. |
| REFCLK impedance | **100 Ω ±10%** diff (HCSL) | 85–100 Ω is acceptable. Use the module's documented value. |
| Single-ended reference | 45–50 Ω | Result of the coupling geometry |
| Intra-pair skew (P vs N) | **≤ 0.13 mm (5 mil)** | Match within 5 mm of the mismatch source (pin escape, bend) |
| Inter-pair skew (TX vs RX) | No requirement | The link is asynchronous between lanes and directions. |
| REFCLK vs data skew | No tight requirement | Keep REFCLK reasonably short (< 100 mm). |
| Max carrier trace length | **≤ 100 mm** per pair (target < 60 mm) | Module routing consumes part of the ~-13 dB @ 2.5 GHz channel budget. |
| Insertion loss (carrier only) | < 3 dB @ 2.5 GHz | Standard FR-4 suffices at Gen2. |
| Vias per pair | **≤ 2** (ideally 0–2) | Each via is a discontinuity. |

---

## 2. Stackup and Geometry

### 4-layer, 1.6 mm (e.g., JLC7628-type)

| Layer | Use |
|---|---|
| L1 | Signal: PCIe pairs (microstrip over L2) |
| L2 | **Solid GND**, no splits under PCIe |
| L3 | Power (3V3_SSD pour, others) |
| L4 | Signal / low-speed |

**Indicative geometries (verify with your fab's impedance calculator and request controlled impedance):**

| Net class | Layer | Width / Gap | Prepreg h |
|---|---|---|---|
| PCIe_DATA_90 | L1 over L2 | ~0.20 mm / 0.20 mm | ~0.21 mm (7628, εr ≈ 4.4) |
| PCIe_REFCLK_100 | L1 over L2 | ~0.17 mm / 0.25 mm | same |

- **Pair-to-pair clearance:** ≥ 3× trace width (≥ 0.6 mm). TX-to-RX spacing should be ≥ 5W to limit NEXT.
- **Clearance to other nets:** ≥ 0.5 mm. Keep ≥ 1 mm from switching regulators, clocks, and the SSD-power inductor.
- **Copper pour clearance (coplanar GND on L1):** ≥ 3W gap. Alternatively, keep the pour clear of the pairs so it does not pull impedance.
- **Plane edge:** keep pairs ≥ 1 mm (≥ 3h) from the edge of the L2 plane.

### 6-layer option

Use stripline on L3 between GND L2 and GND L4. This gives better isolation, but every pair then needs 2 vias. Only worth it if L1 is congested.

---

## 3. Topology and Components

```
CM4-form module (DF40)  ──PCIe_TX_P/N──►  M.2 PERp0/PERn0 (pins 43/41)
                        ◄─PCIe_RX_P/N──   M.2 PETp0/PETn0 (pins 49/47)
                        ──REFCLK_P/N──►   M.2 REFCLKp/n  (pins 55/53)
                        ──PERST#──────►   pin 50
                        ◄─CLKREQ#─────    pin 52  (pull-up 10k to 3V3 per module doc)
                          PEWAKE#         pin 54  (optional / NC)
```

### AC coupling caps

- **Host TX:** The Raspberry Pi CM4 places the TX caps **on the module**. Verify the same on the Radxa module schematic. If they are present, put **no caps** on the carrier.
- **SSD TX:** The M.2 SSD provides its own TX caps, so put no caps on the carrier for the RX path either.
- **If the module lacks TX caps:** add **100 nF, 0402 (or 0201)** on the host-TX pair. Place them symmetrically, side by side, ideally near the M.2 connector. Add a **GND void on L2 under the pads**, sized to pad + ~0.2 mm, to offset the pad capacitance.

### Polarity

Do **not** rely on PCIe lane polarity inversion. Route P to P and N to N, even though Gen2 permits inversion.

### Power and sideband

- **3V3 supply:** Feed pins 2/4/12/14/16/18/70/72/74 from a **≥ 3 A** rail (NVMe peaks of 2.5 A+).
- **Bulk capacitance:** 2× 22 µF + 100 nF at the connector.
- **Sideband nets:** PERST# and CLKREQ# are low-speed. They should still not cross the pairs on adjacent layers.

---

## 4. Return Path and Via Stitching

### 4.1 Reference continuity

- Every PCIe and REFCLK segment must reference **GND only**: L2 for L1 routes, L2/L4 for stripline.
- **No crossings** of plane splits, voids, slots, or via antipad fields.
- The 3V3_SSD pour on L3 must not become a reference for any PCIe segment. In the 4-layer stack, L1 sees L2 GND, so this is satisfied automatically.

### 4.2 Signal-via return (layer transitions)

For each pair layer change:

- **Return vias:** Place **≥ 2 GND stitching vias per pair**, one beside each signal via. Use a symmetric ("GSSG") arrangement.
- **Distance:** ≤ 1.0 mm center-to-center from the signal via (target 0.7–0.8 mm).
- **Via size:** 0.3 mm drill / 0.6 mm pad, the same as signal vias.
- **Antipad:** Use one oval antipad that encloses both signal vias. Start at ~0.9 × 1.6 mm and tune to ~90 Ω in a field solver if available.
- **Stubs:** Remove unused inner pads (non-functional pad removal) to reduce via capacitance.
- **Reference change:** If the reference changes between two GND planes (6-layer), the stitching vias tie them directly. If one reference is power, add a 100 nF stitching cap within 2 mm. Avoid this case entirely.

### 4.3 Edge and fence stitching

- **GND via fence:** Place a fence along both sides of the PCIe channel, offset ≥ 3W (≥ 0.6 mm) from the outer trace edge.
- **Pitch:** ≤ **2.5 mm**. λ/8 at 7.5 GHz in FR-4 is ≈ 2.7 mm.
- **Coplanar pour:** Stitch any L1 GND pour near the pairs at the same ≤ 2.5 mm pitch so it cannot resonate as a floating patch.
- **Board perimeter:** Stitch at ≤ 5 mm pitch. This is EMI hygiene.

### 4.4 Connector transitions

- **DF40 escape:** Place GND vias at the module connector's GND pins immediately adjacent to the PCIe pin pairs, at ≤ 0.5 mm via-in-pad or a short dogbone.
- **M.2 connector:** Tie each GND pin flanking lanes and REFCLK (pins 39/45/51/57) with its own via to L2. Use one via per pin and do not share.
- **M.2 mounting standoff:** Tie the standoff/screw to GND with ≥ 2 vias.

---

## 5. Routing Rules Checklist

1. **Routing layer:** Route all PCIe on L1, with no vias if possible. Both connectors are top-side, so 0 vias is achievable.
2. **Coupling:** Keep pairs tightly coupled. Uncoupled breakouts must be ≤ 3 mm at the pins.
3. **Bends:** Use 45° bends or arcs, never 90°.
4. **Skew compensation:** Use small serpentine bumps (amplitude ≤ 2W, length ≥ 3W), placed **near the mismatch** rather than at the far end.
5. **Test points:** Avoid them on the pairs. If needed, use pads with no stub.
6. **Isolation from other high-speed nets:** Keep ≥ 2 mm from USB/HDMI/Ethernet pairs. Never route parallel on adjacent layers.
7. **Spacing to the M.2 latch:** Keep the pairs ≥ 1 mm from the M.2 key slot and the mounting hole keepout.
8. **Fab notes:** Specify controlled impedance with tolerances and coupon, plus 1 oz outer finished copper and ENIG.

---

## 6. Verification for Cycle 18 Sign-off

- **Pre-layout:** Field-solve the stackup and record the W/S values.
- **Post-layout:**
  - Run a length/skew report with intra-pair ≤ 0.13 mm.
  - Run a DRC for reference-plane crossings.
  - Verify via-to-GND-via distance.
- **Optional simulation:** Extract S-parameters (SDD21 > -3 dB @ 2.5 GHz carrier-only, SDD11 < -10 dB).
- **Bring-up:**
  - Confirm with `lspci -vv` that LnkSta shows **Speed 5GT/s, Width x1**.
  - Check that AER counters stay clean under a `fio` sustained read/write.
  - Verify the 3V3 droop stays < 5% at peak load.

Share the Cycle 17 stackup and fab, plus your module variant, and I can tighten the W/S numbers and antipad sizes to specific values.