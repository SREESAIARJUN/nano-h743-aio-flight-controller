# M.2 NVMe (PCIe Gen2 x1) Routing Design: Radxa CM4-Form-Factor Expansion Board, Cycle 19

I don't have your Cycle 1–18 history, stackup file or schematic. This package therefore uses a typical 4-layer 1.6 mm build and standard PCIe Gen2 practice. **Re-run impedance numbers in your fab's calculator** before freezing.

---

## 1. Interface Summary

| Item | Value |
|---|---|
| Link | PCIe Gen2 x1, 5 GT/s, UI = 200 ps |
| Spectral content of concern | 2.5 GHz fundamental, design for ≥7.5 GHz (3rd harmonic) |
| Nets | TX± (host→SSD), RX± (SSD→host), REFCLK± (100 MHz HCSL), PERST#, CLKREQ#, PEWAKE# |
| Connector | M.2 Key M, 2280 (support 2230/2242 mount holes optional) |

**M.2 Key M lane 0 pins:**
- PERn0/PERp0: 41/43
- PETn0/PETp0: 47/49
- REFCLKn/p: 53/55
- PERST#: 50
- CLKREQ#: 52
- PEWAKE#: 54

M.2 naming is from the card's perspective. Cross host TX to the card's receive pins and verify against the CM module pinout. Polarity inversion within a pair is tolerated by PCIe. Lane swap is not relevant at x1.

---

## 2. AC Coupling

- **SSD side:** M.2 SSDs place their own TX coupling caps on-card, so the SSD→host pair needs no carrier caps.
- **Host TX side:**
  - Raspberry Pi CM4 has its TX caps on-module.
  - **Verify for the Radxa module you target (CM3/CM3I/CM5 variants differ).** If the module lacks them, add 100 nF X7R 0201 (or 0402) caps on host TX, placed near the connector.
- **Cap pads:**
  - Remove the reference plane under the pads (anti-pad cutout on L2, pad size + 0.1 mm), or
  - Use 0201 caps to keep the impedance dip under about 5 Ω.
- **Placement:** keep cap placement symmetric within the pair.

---

## 3. Stackup & Impedance Targets

Example 4-layer, 1.6 mm, FR-4 (Er ≈ 4.4 @ 1 GHz):

| Layer | Use | Dielectric to next |
|---|---|---|
| L1 | PCIe diff pairs (microstrip) | 0.21 mm prepreg (7628) |
| L2 | **Solid GND**, unbroken under PCIe | 1.07 mm core |
| L3 | Power / slow signals | 0.21 mm prepreg |
| L4 | GND pour + slow signals | — |

| Parameter | Target | Starting geometry (L1, 1 oz finished) |
|---|---|---|
| Diff impedance TX/RX | **85 Ω ±10%** (90 Ω acceptable; CM4 guidance uses 90 Ω) | W = 0.16 mm, S = 0.18 mm, about 87 Ω |
| REFCLK diff | 100 Ω ±10% (85 Ω acceptable) | W = 0.13 mm, S = 0.20 mm |
| Single-ended (ref) | ~50 Ω | — |

- **Pick one target:** choose 85 Ω or 90 Ω and use it for the whole channel. Don't mix.
- **Fab control:** order controlled impedance with a coupon.

---

## 4. Differential Routing Rules

| Rule | Value |
|---|---|
| Intra-pair skew (P vs N) | ≤ 0.127 mm (5 mil); match at the mismatch source, within 5 mm |
| Pair-to-pair (TX vs RX) | No matching required (independent lanes) |
| REFCLK vs data | No matching required |
| Max length module→M.2 | ≤ 100 mm recommended (Gen2 budget allows more, but keep it short) |
| Pair-to-pair spacing | ≥ 3×W edge-to-edge, i.e. ≥ 0.5 mm (prefer 5×W between TX and RX) |
| Pair to other signals | ≥ 0.75 mm (≥ 5×H); ≥ 1.0 mm to clocks and switchers |
| Pair to plane/pour edge on L1 | ≥ 0.5 mm, or ≥ 3×W coplanar clearance |
| Distance from L2 plane edge | Pair centerline ≥ 1.0 mm (≥ 5×H) inside the ground edge |
| Bends | 45° or arcs only. No 90° bends. Alternate bend direction to self-compensate skew. |
| Serpentine (skew tune) | Amplitude ≤ 2×S, segment length ≥ 3×W, gap ≥ 4×W, placed near the mismatch |
| Layer changes | **Max 2 vias per signal, target 0** (route fully on L1) |
| Stubs / test points | None. Use inline pads if test is needed; no via stubs on through-hole if avoidable. |
| Plane splits / slots | **Never cross** a split, slot or anti-pad void on L2 |
| Breakout at connectors | Uncoupled length ≤ 3 mm; keep P/N symmetric into the pads |

**Connector pad compensation:**
- **Module B2B connector:** void L2 under the pad fields only if the solver shows capacitance excess. Usually it is fine as is on 0.21 mm prepreg.
- **M.2 edge connector:** fine as is. Keep ground pins 45/51/57 tied directly to L2 with a via per pin.

---

## 5. Return Path & Via Stitching

### 5a. Signal-via transitions (if a layer change is unavoidable)
- Place **2 GND vias per differential via pair**, symmetric, one on each side, ≤ 0.8 mm (≈30 mil) center-to-center from the signal vias.
- Both reference layers on either side of the transition must be GND. If transitioning to a layer referenced by the L3 power plane, add **100 nF 0201 stitching caps** within 1 mm, or prefer L4 GND.
- **Signal via:** 0.25 mm drill / 0.45 mm pad, anti-pad 0.85–1.0 mm oval around the pair. Remove unused inner pads (NFP removal) if your fab supports it.

### 5b. Ground stitching fence along the channel
- L1 GND pour flanking the pairs (coplanar guard), stitched to L2/L4.
- **Via pitch ≤ 2.0 mm** along the route (≈ λ/10 at 7.5 GHz in FR-4, λ ≈ 20 mm). Use ≤ 1.5 mm near connectors.
- Fence offset: ≥ 0.5 mm from the nearest pair edge. Keep it symmetric on both sides of each pair.
- Remove any floating copper islands. Every pour fragment needs ≥ 2 stitching vias, or delete it.

### 5c. Connector ground anchoring
- **CM B2B connector:** each GND pin adjacent to the PCIe pins gets its own via to L2, ≤ 0.5 mm from the pad.
- **M.2 connector:**
  - Place GND vias at pins 45, 51 and 57 bracketing the lanes.
  - Add a stitch row along the connector body every 1.5 mm.
  - Bond the standoff and mounting screw pad to GND with ≥ 4 vias.

### 5d. Board-level
- Stitch L2↔L4 GND on a 5 mm grid board-wide and on a 2 mm pitch along the board edge near the M.2.
- No L3 power-plane edge running under the PCIe pairs (prevents edge-coupled noise).

---

## 6. REFCLK & Sideband

- **REFCLK:**
  - Treat as a differential pair per section 4, with no layer change if possible.
  - Keep ≥ 1 mm from TX/RX.
  - Termination: follow the module guidance. HCSL source terminations are usually on-module, so add none unless specified.
- **PERST#:** 3.3 V domain. Add a level shift if the module uses 1.8 V I/O (check Radxa pin voltage). Add a 10 kΩ pull-down if the host leaves it floating at boot.
- **CLKREQ#:** open-drain. 10 kΩ pull-up to 3.3 V.
- **PEWAKE#:** open-drain. 10 kΩ pull-up to 3.3 V.

---

## 7. M.2 Power (affects return integrity)

- **3.3 V rail:** ≥ 2.5 A capable (NVMe peaks around 2–3 A). Fed by a wide pour or plane on L3. Use ≥ 6 vias at the connector supply pins.
- **Decoupling at the connector:** 2× 22 µF + 4× 100 nF. Place each 100 nF with its GND via ≤ 1 mm from the pad.
- **Switcher placement:** keep the regulator ≥ 10 mm from the PCIe pairs. Its switching loop must not overlap the PCIe corridor on any layer.

---

## 8. Cycle 19 Verification Checklist

1. ☐ Fab impedance quote matches 85/90 Ω with a coupon; geometry updated from the fab calculator.
2. ☐ DRC net-class rules entered (width/gap, skew ≤ 0.127 mm, clearance).
3. ☐ TX/RX direction crossed correctly; AC cap location verified against the specific Radxa module datasheet.
4. ☐ Zero split-plane crossings (overlay L1 pairs on L2 in the viewer).
5. ☐ Every signal via has ≤ 0.8 mm GND return vias; every pour island stitched.
6. ☐ Fence pitch ≤ 2 mm audited.
7. ☐ Optional: export the channel for 2D field-solver / S-parameter check. Target insertion loss better than −3 dB @ 2.5 GHz and return loss better than −10 dB.
8. ☐ Bring-up: confirm `lspci` link at 5 GT/s x1 and run a sustained `fio` read with AER error counters at 0.

If you share the Cycle 18 stackup and any review findings, I can tighten the geometry numbers and flag deltas specifically.