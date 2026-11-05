# M.2 NVMe (PCIe Gen2 x1) Routing Specification: Radxa CM4 Expansion Board, Cycle 27

> **Scope:** CM4-form-factor host (Radxa CM3/CM4-class SoM, PCIe 2.1 x1 at 5 GT/s) through the 2×100-pin board-to-board connectors to an M.2 2280/2242 M-key socket.
> **Before you lay out:** Verify every module-side item marked ⚠ against the exact Radxa SoM schematic and pinout revision you are using.

---

## 1. Stack-up and Impedance Targets

### Recommended 4-layer stack-up (1.6 mm, JLC-style 7628 prepreg)

| Layer | Function | Notes |
|---|---|---|
| L1 | PCIe pairs, REFCLK, components | Microstrip, referenced to L2 |
| L2 | **Solid GND** | No splits or slots under any HS route |
| L3 | Power (3V3 for M.2, 5V) + slow signals | Avoid routing HS here |
| L4 | GND pour + low-speed signals | Stitched to L2 |

**Rule:** Route all PCIe on L1 and keep it there. If a layer change is unavoidable, go L1 to L4 only. L4 must have a GND pour under the route and stitching vias at the transition (see §5).

### Impedance

| Net class | Target | Tolerance | Rationale |
|---|---|---|---|
| PCIe TX/RX pairs | **90 Ω differential** | ±10% | Compromise between the PCIe CEM/M.2 85 Ω and the CM4-ecosystem 90 Ω guideline |
| REFCLK 100 MHz (HCSL) | 90 Ω differential | ±10% | Same geometry as the data pairs for simplicity |
| Single-ended sideband | 50 Ω (not critical) | — | PERST#, CLKREQ#, PEWAKE# |

**Starting geometry (L1 over L2, h ≈ 0.21 mm, εr ≈ 4.4, 1 oz finished):**
- Trace width ≈ 0.15 mm
- Gap ≈ 0.18–0.20 mm

These are approximate. **Confirm with your fab's impedance calculator** and order controlled impedance with a test coupon. Do not vary the width or gap along a pair. Neck down only at the connector pads, for less than 1 mm.

---

## 2. Net Mapping (M.2 M-key, names from the SSD's perspective)

| Function | M.2 pin | Connect to |
|---|---|---|
| PERn0 / PERp0 (SSD receive) | 41 / 43 | **Host TX** (via AC caps if not on the SoM) |
| PETn0 / PETp0 (SSD transmit) | 47 / 49 | **Host RX** (the SSD carries its own TX caps) |
| REFCLKn / REFCLKp | 53 / 55 | Host REFCLK output |
| PERST# | 50 | Host PCIe reset GPIO (3.3 V, OD/PP per SoM) |
| CLKREQ# | 52 | Host CLKREQ# or 10 kΩ pull-down to GND if unused |
| PEWAKE# | 54 | Host WAKE# or 10 kΩ pull-up to 3V3, optional |
| 3.3 V | 2, 4, 12, 14, 16, 18, 70, 72, 74 | 3V3 rail rated ≥ 2.5 A |
| GND | All GND pins | Via directly into L2 at each pad |

**Polarity:**
- **Data pairs:** P/N swaps are permitted (PCIe link training auto-corrects polarity). Use them to avoid crossovers and vias.
- **REFCLK:** Never swap P/N.
- **TX/RX:** Lanes must not be cross-swapped.

---

## 3. AC Coupling Capacitors

| Path | Requirement |
|---|---|
| Host TX to SSD RX | 0.1 µF (spec range 75–265 nF), X7R, **0201 preferred / 0402 max** ⚠ Check whether the Radxa SoM already places these. On the RPi CM4 they are on-module, so the carrier places none. Never double them. |
| SSD TX to Host RX | **None on carrier.** The M.2 SSD has them on-card. |

**If you place caps:**
- Put them near the M.2 connector, within 10 mm.
- Place the pair side by side, symmetric, and not staggered.
- Add a **GND cutout on L2 under the cap pads** (pad footprint + ~0.1 mm) to cancel the pad capacitance dip. Only do this when L2 is the immediate reference, and keep L3 unbroken beneath.

---

## 4. Length and Routing Rules

| Parameter | Value |
|---|---|
| Intra-pair skew (P vs N) | **≤ 0.127 mm (5 mil)** |
| Skew compensation location | At the mismatched end (within 5 mm of the bend/pin causing it). Use small serpentine bumps with amplitude ≤ 2× gap and length ≥ 3× width. |
| Inter-pair matching (TX vs RX vs REFCLK) | **Not required** |
| Max carrier length (B2B connector to M.2 pin) | **≤ 100 mm** recommended. Gen2 budget allows more, but the module plus connector consume margin. |
| Via count per net | **≤ 2** (ideally 0) |
| Pair-to-pair spacing (TX to RX) | ≥ 5× trace width (≈ 0.75 mm), or a GND guard pour between pairs |
| Pair to other signals | ≥ 5W. To clocks, USB 3, Ethernet, HDMI, or switchers: ≥ 1.5 mm. |
| Pair to plane/pour edge | ≥ 3× gap (≈ 0.6 mm) |
| Bends | 45° or arcs only, no 90°. Keep P/N bend counts balanced. |
| Over plane splits | **Forbidden** |
| Board edge | ≥ 1 mm from edge, with a GND via fence along the edge |
| Under the M.2 connector | Keep L2 solid. No routing on L3 directly beneath the pin field. |
| Test points | Avoid on HS pairs. If needed, use pads in-line, not stubs. |

**Breakout:** Exit B2B and M.2 pads directly and symmetrically. Couple the pair within 1–2 mm of the pad and keep uncoupled length < 2 mm per end.

---

## 5. Return Path and Via Stitching

### 5.1 Signal-via return stitching (mandatory at any layer change)
- **Two GND vias per differential via pair**, one beside each signal via, symmetric about the pair.
- Place them **≤ 0.8 mm** center-to-center from the signal via, never more than 1 mm.
- Anti-pad: ~0.9–1.0 mm diameter for a 0.3 mm drill / 0.5 mm pad. Use an **oblong shared anti-pad around the pair** on L2/L3 to improve differential via impedance.
- Remove non-functional pads on inner layers if your fab allows it.
- **L1 to L4 transitions:**
  - The signal reference moves L2 to L3. The GND vias tie L2 to the L4 pour.
  - If L3 is a power plane at the transition, also add a **10 nF 0201 stitching cap (3V3 to GND)** within 1 mm, or locally make L3 GND under that segment.

### 5.2 Via fence along the route
- GND vias on both sides of the TX, RX, and REFCLK groups, tying the L1 pour to L2 (and L4).
- **Fence pitch ≤ 2.5 mm.** That is about λ/20 at the 5th harmonic region. For Gen2 (2.5 GHz fundamental, λ_FR4 ≈ 60 mm), it keeps pours from resonating.
- Fence offset: ≥ 0.6 mm from trace edge (3× gap) so the impedance is not affected.
- Remove any copper pour "islands" or slivers narrower than 0.5 mm near pairs, or stitch every island with ≥ 2 vias.

### 5.3 Connector return paths
- **B2B connectors:** Every GND pin adjacent to the PCIe pins gets its own via to L2 within 0.5 mm of the pad. Do not share or daisy-chain them.
- **M.2 socket:** Do the same for the GND pins around 41/43/47/49/53/55 (pins 39, 45, 51, 57). Place 2 or more extra GND vias between the TX and RX pin groups.
- **M.2 standoff/screw pad:** Tie it to GND with ≥ 4 vias. It provides a secondary return and an ESD path.

### 5.4 Global stitching
- L1/L4 GND pours to L2: a via grid at ≤ 5 mm pitch across the board, tighter (≤ 2.5 mm) within the HS corridor.

---

## 6. M.2 Power (affects signal integrity)
- 3V3 rail: ≥ 2.5 A (NVMe peaks of about 2–3 A for 2280 drives).
- Decoupling at the socket: 1× 47 µF + 2× 10 µF + 4× 100 nF (one per 3V3 pin group), each with its own GND via.
- Route 3V3 on L3 as a pour. Keep switcher inductors and the SW node ≥ 10 mm from the PCIe corridor.

---

## 7. Cycle 27 DRC / Review Checklist

- [ ] Fab stack-up confirmed; 90 Ω ±10% coupon ordered
- [ ] Pair width/gap constant end to end (neck-down only at pads)
- [ ] Intra-pair skew ≤ 0.127 mm for TX, RX, REFCLK
- [ ] REFCLK polarity correct (P to 55, N to 53)
- [ ] Host TX goes to M.2 PER (41/43); host RX goes to M.2 PET (47/49)
- [ ] AC caps: present exactly once on the host TX path (⚠ SoM checked)
- [ ] ≤ 2 vias per net; each via pair has 2 GND return vias at ≤ 0.8 mm
- [ ] No HS trace crosses an L2 void, split, or slot (check the reference-plane overlay)
- [ ] Via fence pitch ≤ 2.5 mm; no unstitched pour slivers
- [ ] TX-to-RX and pair-to-aggressor spacing per §4
- [ ] CLKREQ# / PERST# / PEWAKE# terminated per §2
- [ ] 3V3 decoupling placed and via'd at the socket
- [ ] Post-layout: run a 2D field-solver check on the actual geometry. Optionally, run a channel S-parameter sim (target IL > −6 dB at 2.5 GHz carrier-only, RL < −10 dB).

---

If you share the actual stack-up from your fab and the exact SoM model/revision, I can tighten the trace geometry numbers and confirm the AC-cap placement and B2B pin assignments.