# M.2 NVMe (PCIe Gen2 x1) Routing and Return-Path Design for a Radxa CM4-Form-Factor Carrier

I don't have your project files or what "Cycle 28" contains. This is a complete baseline package you can diff against your current revision. Confirm the module-specific items (marked **⚠ verify**) against the Radxa CM3/CM4-compatible module datasheet and your fab's stackup.

---

## 1. Interface Summary

| Item | Value |
|---|---|
| Link | PCIe Gen2, 5 GT/s, x1 (CM4-compatible connector exposes 1 lane) |
| Nyquist frequency | 2.5 GHz. Design bandwidth to about 7.5 GHz (3rd harmonic). |
| Signals | PCIe_TX±, PCIe_RX±, PCIe_CLK± (100 MHz HCSL), PERST#, CLKREQ#, (optional) WAKE#, M.2 3.3 V |
| M.2 connector | M-key, 2242/2280. Lane 0 on pins 41/43 (PERn0/PERp0, host RX) and 47/49 (PETn0/PETp0, host TX). REFCLK on 53/55. |

---

## 2. Differential Impedance Targets

| Net class | Target Z_diff | Tolerance | Notes |
|---|---|---|---|
| PCIE_TX / PCIE_RX | **90 Ω** | ±10 % | Matches CM4 carrier guidelines. 85 Ω (PCIe CEM) is also acceptable. Pick one and keep it consistent. |
| PCIE_REFCLK | 90–100 Ω | ±10 % | HCSL. Use 90 Ω to share the rule. |
| Single-ended reference | ~50 Ω | ±10 % | Informational |

### Example Stackup (JLC-style 4-layer, 1.6 mm, 7628 prepreg, εr ≈ 4.4)

| Layer | Use |
|---|---|
| L1 | Signal (PCIe pairs routed here) |
| L2 | **Solid GND**, the reference for L1 |
| L3 | Power (3V3 pour) / low-speed signals |
| L4 | Signal / GND pour |

- **L1 microstrip, 90 Ω:** approximately **W = 0.20 mm, S = 0.20 mm** over 0.21 mm prepreg. This is a starting point only. Re-run in your fab's impedance calculator and order controlled impedance.
- **6-layer option:** route on L3 stripline between GND L2 and L4 for better isolation. This needs extra via transitions.

---

## 3. Routing Rules

| Parameter | Rule |
|---|---|
| Intra-pair skew (P vs N) | ≤ **0.127 mm (5 mil)**. Match near the mismatch source (at the bend), not at the end. |
| Inter-pair matching (TX vs RX) | Not required. They are independent lanes/directions. |
| REFCLK vs data | No matching needed (common-clock architecture). |
| Max length, module connector → M.2 | Target ≤ **100 mm**, hard limit about 150 mm on FR-4. |
| Pair-to-pair spacing (TX↔RX) | ≥ **3W**, ideally ≥ 5W (≥ 1.0 mm). |
| Pair to other signals | ≥ **5W** (≥ 1.0 mm). Keep away from SDIO, Ethernet, and switching nodes by ≥ 3 mm. |
| Pair to board edge / plane edge | ≥ 1.5 mm, and keep the reference plane extending ≥ 3W past the pair. |
| Bends | 45° or arcs only, no 90°. Keep the pair coupled through bends. |
| Serpentine (skew tuning) | Amplitude ≤ 2S, segment length ≥ 3W, spacing between bumps ≥ 4S. |
| Layer changes | Max **2 vias per signal** end-to-end, ideally 0 on L1-only routing. |
| Uncoupled length | ≤ 3 mm total (pin breakout only). |
| Over splits/voids | **Never.** Continuous L2 GND under the full pair length. |
| Polarity | P/N swap is allowed by PCIe (auto polarity inversion). TX↔RX must not be swapped. |

### AC Coupling Capacitors
- PCIe requires 100 nF caps on the **TX side of each transmitter**.
  - **Host TX → SSD RX:** the cap is required somewhere. The CM4-compatible module may already include it. **⚠ verify.** If it is not on-module, place 0201/0402 100 nF X7R caps on PETp0/PETn0 near the module connector.
  - **SSD TX → host RX:** the caps are on the M.2 SSD itself. Do **not** add caps on the carrier.
- Place the two caps symmetrically, side by side, with no stagger.
- **Cut out L2 GND under the cap pads** (pad-sized void, ~0.1 mm oversize). This compensates the impedance dip from the wide pads. Keep L3 solid so the field references L3 there.

---

## 4. High-Speed Return Path and Via Stitching

### 4.1 Signal-Via Return Vias (most critical)
Wherever a pair changes layer, the return current must change reference plane too.

- Place **2 GND vias per differential via pair**, one on each side, symmetric about the pair axis.
- Distance from signal via center to GND via center: ≤ **0.8 mm** (closer is better, ~0.6 mm typical with 0.3/0.6 mm vias).
- If the reference changes GND→3V3 (avoid this), add a **100 nF stitching cap GND↔3V3** within 1 mm instead. Avoid this case entirely by routing on L1/L2-GND only.
- Antipad: oval antipad covering both signal vias (~0.25 mm clearance), not two separate circles. Tune it in a solver if possible.
- Back-drilling is not needed for a 1.6 mm board at 5 GT/s. Stub resonance is far above 7.5 GHz if the transition is L1→L4.

### 4.2 Guard / Fence Stitching Along the Route
- λ in FR-4 at 7.5 GHz ≈ 20 mm. Stitching pitch should be ≤ λ/8–λ/10, so use **≤ 2.5 mm pitch**.
- If you pour L1 copper beside the pairs, keep it ≥ 3W (≥ 0.6 mm) from the pair edge. Stitch the pour to L2 at ≤ 2.5 mm pitch and **remove any floating or unstitched slivers** (antenna risk).
- Alternatively, leave no L1 pour near the pairs. This is often cleaner on 4-layer boards.

### 4.3 Connector Regions
- **Module connector (Hirose DF40):** place a row of GND vias adjacent to the PCIe pin breakout, with at least one GND via per PCIe pin pair within 1 mm. Tie all connector GND pins with short, direct vias.
- **M.2 connector:** stitch GND vias at every GND pin (pins 39, 45, 51, 57). Add a via fence on both sides of the pair fan-out at ≤ 2 mm pitch.
- **M.2 standoff/mounting screw:** connect to GND with ≥ 4 stitching vias.

### 4.4 Plane Integrity
- No traces on L2 under the PCIe corridor.
- No 3V3 or other splits in L2.
- If L3 is a 3V3 pour, add GND↔3V3 decoupling (100 nF + 10 nF) every ~10 mm along the corridor to give an AC return near layer transitions.
- Add board-wide GND stitching on a ~5 mm grid in open areas.

---

## 5. Sideband and Power

| Signal | Rule |
|---|---|
| PERST# | 3.3 V open-drain/push-pull from the module. Route away from pairs (≥ 1 mm). Add an optional 10 k pull-down so the SSD stays in reset during boot. |
| CLKREQ# | 10 k pull-up to 3V3. Tie to the module CLKREQ or to GND if unused (forces clock always-on). **⚠ verify** module behavior. |
| REFCLK | Same rules as the data pairs (90 Ω, ≤ 5 mil skew). Keep ≥ 1 mm from TX/RX. |
| M.2 3V3 | Up to **2.5 A** peak for NVMe. Use a dedicated buck or load switch with ≥ 3 A capability. Bulk 2× 22 µF + 4× 100 nF at the connector. Use ≥ 1.0 mm traces or a pour, with multiple vias per layer change. |
| LED/DAS (pin 10) | Optional activity LED via 1 k to 3V3 |

---

## 6. Verification Checklist for This Cycle

1. Fab impedance report confirms 90 Ω ±10 % on L1 coupons.
2. DRC net-class rule: PCIE_* set to W/S per the stackup, with skew ≤ 0.127 mm.
3. Every PCIe signal via has ≥ 2 GND vias within 0.8 mm.
4. L2 is continuous under the full PCIe corridor. Check with plane-only view.
5. AC caps are present on host TX only, with L2 voids under the cap pads.
6. No floating copper near the pairs. All pours are stitched at ≤ 2.5 mm.
7. Bring-up test:
   - `lspci -vv`: check `LnkSta: Speed 5GT/s, Width x1`.
   - `fio` sequential read: expect about 380–420 MB/s, the Gen2 x1 practical ceiling.
   - Check `dmesg` for AER correctable errors under sustained load.

If you share your stackup, the fab you're using, and the current Cycle 28 layout constraints, I can compute exact trace geometry and flag deviations.