# M.2 NVMe PCIe Gen2 x1 Routing and Return-Path Design for a CM4-Form-Factor Expansion Board (Cycle 30)

## 0. Scope and Assumptions

**Target link.** The module is a CM4-footprint compute module (Radxa CM3/CM4-compatible pinout). It exposes one PCIe Gen2 x1 lane at 5 GT/s with 8b/10b encoding.
- Nyquist frequency is 2.5 GHz. Design for content up to about 7.5 GHz (the 3rd harmonic).

**Connector.** M.2 M-key socket. Supported cards are 2230, 2242, and 2280, depending on the standoff options you populate.

**Lane allocation.** Only lane 0 is used on the M.2 pins:
- PETp0/PETn0 and PERp0/PERn0
- REFCLKp/REFCLKn
- PERST#, CLKREQ#, PEWAKE#

**Before you lay out, confirm these against your exact module's datasheet:**
1. Whether the module already places the TX AC-coupling caps on-module. On the RPi CM4 they are on-module; Radxa variants should be verified.
2. The specified differential impedance (90 Ω on the CM4 reference designs).
3. The pin-swap and polarity-inversion capability of the module's PCIe PHY.

---

## 1. Stackup (4-Layer, 1.6 mm, FR-4 / Fab-Standard)

| Layer | Function | Notes |
|---|---|---|
| L1 | PCIe pairs, refclk, sidebands | Route over a solid L2 plane |
| L2 | **Solid GND** | No splits, no slots under any pair |
| L3 | Power (3V3_M2, 5V, etc.) | Pour; keep it away from under the L1 pairs where possible |
| L4 | GND + low-speed routing | Stitched to L2 |

**Preferred variant.** Use a thin L1–L2 dielectric, about 0.1 mm (for example, a 3313 prepreg class). It gives:
- tighter coupling to the reference plane
- narrower traces, which fan out more easily at the M.2 and B2B connector pins
- less crosstalk

---

## 2. Differential Pair Parameters

### 2.1 Impedance Targets

| Net group | Target | Tolerance |
|---|---|---|
| PETx / PERx (data) | **90 Ω diff** (85 Ω per the M.2 spec is also acceptable; pick one and stay consistent) | ±10 % |
| REFCLK (100 MHz HCSL) | 90–100 Ω diff | ±10 % |
| Single-ended reference | 50 Ω | ±10 % |

### 2.2 Starting Geometry (Edge-Coupled Microstrip on L1)

These are starting points only. Re-run them in your fab's impedance calculator and request controlled-impedance coupons.

| Dielectric L1–L2 | εr | Cu | W / S for ≈90 Ω | W / S for ≈100 Ω |
|---|---|---|---|---|
| ~0.10 mm (3313) | 4.1 | 1 oz finished | 0.12 mm / 0.15 mm | 0.10 mm / 0.18 mm |
| ~0.21 mm (7628) | 4.4 | 1 oz finished | 0.20 mm / 0.20 mm | 0.16 mm / 0.25 mm |

### 2.3 Routing Rules

**Length and skew**
- **Intra-pair skew:** ≤ 0.13 mm (5 mil). Match close to where the skew originates, such as the connector pin breakout. Use small serpentine bumps no taller than 2×S and spaced ≥ 3×W apart.
- **Inter-pair matching:** TX vs. RX vs. REFCLK does **not** need matching. Gen2 uses an embedded clock and each lane is independent.
- **Total length budget (baseboard only):** keep each pair ≤ 75 mm. A typical CM4 carrier run is 20–50 mm.
  - At about 0.03–0.04 dB/mm at 2.5 GHz on FR-4, 50 mm costs well under 2.5 dB.
  - Most of the channel budget is left for the module and the SSD.

**Geometry and spacing**
- **Pair-to-pair spacing:** ≥ 4×W edge-to-edge, or ≥ 3×H. TX-to-RX spacing should be ≥ 5×W, or put a GND guard with vias between them.
- **Spacing to other signals:** ≥ 5×W, and ≥ 1 mm from switching regulators, inductors, and their SW nodes.
- **Bends:** 45° or arc only. Mirror bends within the pair to keep it symmetric.
- **Coupling:** keep the pair coupled for the entire run. Any uncoupled section at pin breakout should be ≤ 2 mm.

**Reference plane and layer changes**
- **Board edge:** keep pairs ≥ 1 mm, ideally 2 mm, from the plane edge and the board edge.
- **Reference:** L2 GND only. Never cross a split, slot, or via antipad field.
- **Layer changes:** zero is the target. If unavoidable, allow at most 2 vias per pair, and each signal via needs a GND return via (see §4).
- **Polarity inversion:** the PCIe link layer can fix P/N polarity, so P/N inversion is allowed if it avoids crossing via swaps. **Lanes must never be swapped TX↔RX.**

### 2.4 AC Coupling

| Direction | Cap location | Value / package |
|---|---|---|
| Host TX → SSD RX (PETp/n0) | On the module (CM4) **or** on the baseboard near the module if not on-module | 100 nF (75–265 nF range), **0201 or 0402**, X7R |
| SSD TX → Host RX (PERp/n0) | On the SSD (per M.2 spec) | **None on baseboard** |

**If you place the caps on the baseboard:**
- Place them symmetrically, side by side.
- Void the L2 plane under the pads, sized to the pad footprint plus about 0.1 mm. This compensates the capacitive pad discontinuity.
- Alternatively, if your 3D solver shows it is within ±10 %, use 0201 parts with no void.

### 2.5 REFCLK and Sidebands

**REFCLK**
- Use the same pair rules as the data pairs.
- Keep it ≥ 1 mm away from the data pairs.
- No series termination is needed unless your module datasheet calls for 33 Ω series resistors with an HCSL driver.

**PERST#**
- Route it from the module's PCIe reset GPIO.
- Add a 10 k pull-down so the SSD stays in reset until the host releases it.

**CLKREQ#**
- Add a 10 k pull-up to 3V3.
- Leaving it unconnected is acceptable if the clock is always on.

**PEWAKE# / SUSCLK**
- Optional. Tie off per the M.2 spec if unused.

---

## 3. M.2 Connector Region

**Breakout and pads**
- Enter the M.2 pads straight, pair-symmetric.
- Neck the traces down to fit the 0.5 mm pitch for ≤ 1.5 mm, then restore the full width.
- Add a plane cutout on L2 under the M.2 signal pads only if the pad width is much larger than the trace width (> 2×W). In that case, verify with a solver. Otherwise keep L2 solid.

**Mounting and power**
- Ground the M.2 mounting-hole standoff and mid-mount nut to GND, using ≥ 4 vias around each.
- **3V3_M2 rail:** 3.3 V at ≥ 2.5 A continuous (NVMe peaks around 3 A).
  - Bulk: 2 × 22 µF + 100 µF near the connector.
  - Decoupling: 100 nF on each of pins 2/4/70/72/74.
  - Use a load switch with soft start if hot-enable is needed.

---

## 4. High-Speed Return Path and Via Stitching

### 4.1 Signal-Via Return Vias (Mandatory at Any Layer Change)

**Return-via placement**
- Place 2 GND vias per differential via pair, one on each side, symmetric.
- Keep them ≤ 0.8 mm center-to-center from the signal via (≤ 0.5 mm preferred).
- The return vias must connect **both** reference planes involved, for example L2 to L4 GND.
- If the reference changes GND → PWR, add 10 nF + 100 nF 0201 stitching caps within 1 mm, or avoid the transition entirely.

**Signal-via geometry**
- Drill 0.2–0.25 mm, pad 0.45 mm.
- Antipad about 0.8–0.9 mm, and use a shared oval antipad around the pair.
- Remove non-functional pads on the inner layers.
- The stub (L1 → L4 through-via used only to reach L1–L3) adds a resonance well above 15 GHz at 1.6 mm, so it is acceptable for Gen2 with no backdrill.

### 4.2 Ground-Plane Stitching Along the Channel

The pitch is set by the highest frequency of interest:

> λ at 7.5 GHz in FR-4 (εeff ≈ 3.4) ≈ 22 mm. Using λ/10 gives about 2.2 mm.

| Location | Stitch pitch | Offset from pair edge |
|---|---|---|
| Along the PCIe corridor (both sides, L1 GND pour ↔ L2 ↔ L4) | **≤ 2.5 mm** | ≥ 3×W, or ≥ 0.4 mm (do not pull the pour closer) |
| Around the M.2 connector and B2B connector PCIe pins | ≤ 1.5 mm, fence on both sides | Edge of pin field |
| Board perimeter (EMI fence) | ≤ 5 mm | 0.5 mm from edge |
| General plane-to-plane stitching (L2 ↔ L4) | ~5 mm grid | n/a |

**Copper pour rules on L1**
- Use L1 GND pour only if it is fully stitched at the pitch above. Floating or poorly stitched copper resonates.
- Remove pour slivers narrower than 0.5 mm or with fewer than 2 vias.

### 4.3 Return-Path Hygiene Checklist

- [ ] L2 is unbroken beneath every PCIe pair from the B2B connector to the M.2 connector.
- [ ] No vias or antipads from other nets perforate L2 under or within 3×H of the pairs.
- [ ] B2B (Hirose DF40) PCIe pins: the GND pins adjacent to the pairs each have a dedicated via to L2 within 0.5 mm.
- [ ] Connector GND pins on the M.2 side (pins 1, 3, 9, 15, 21, 27, 33, 39, 45, 51, 57, 71, 73, 75…) each have a dedicated GND via.
- [ ] No high-current return (fan, USB VBUS, DC-DC) shares the PCIe corridor.

---

## 5. Verification for Cycle 30

1. **Fab:** order controlled-impedance coupons for 90 Ω diff and 50 Ω SE on L1, and require a TDR report.
2. **Pre-layout:** run a 2D field solve of the final stackup and confirm geometry within ±5 % of target.
3. **Post-layout:** extract S-parameters of the baseboard channel. Targets:
   - SDD21 ≥ −3 dB @ 2.5 GHz
   - SDD11 ≤ −10 dB up to 2.5 GHz
   - SCD21 ≤ −25 dB
4. **Bring-up:**
   - Run `lspci -vv` and confirm `LnkSta: Speed 5GT/s, Width x1`.
   - Check for no AER correctable errors under `fio` sustained load for 30 min or more.
   - Check the error counters in `/sys/bus/pci/devices/*/aer_dev_correctable`.
5. **Debug fallback:** if the link trains at 2.5 GT/s only, check, in order:
   - TX cap placement or duplication (caps on both module and baseboard)
   - REFCLK integrity
   - PERST# timing (≥ 100 ms after power stable)

---

**Summary.** Use 90 Ω ±10 % coupled microstrip on L1 over solid L2 GND, with ≤ 5 mil intra-pair skew and no inter-pair matching. Avoid vias; where a layer change is unavoidable, place 2 GND return vias within 0.8 mm. Fence the corridor with GND stitching at ≤ 2.5 mm pitch, tightening to ≤ 1.5 mm at the connectors. Fit TX AC caps only if they are absent from the module, and never fit RX caps on the baseboard.