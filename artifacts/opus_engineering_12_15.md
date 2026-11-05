# M.2 NVMe (PCIe Gen2 x1) Routing Design: Radxa CM4-Form-Factor Carrier, Cycle 12

**Assumptions to confirm.** I don't have your earlier cycle notes, so this design assumes the following:
- The module is a Radxa CM3 or CM3S (RK3566) in the RPi CM4 form factor, exposing one PCIe 2.0 lane.
- The carrier is a 4-layer board: L1 Sig, L2 GND, L3 PWR, L4 Sig.
- The slot is an M.2 M-key socket supporting 2230, 2242 and 2280 drives.

Adjust if Cycle 12 uses a different module or stackup.

---

## 1. Signal Inventory

| Net group | Pins (CM4 connector → M.2 M-key) | Type |
|---|---|---|
| PCIe_TX_P/N (host → SSD) | CM4 TX → M.2 PERp0/PERn0 (pins 49/47) | 5 GT/s diff |
| PCIe_RX_P/N (SSD → host) | M.2 PETp0/PETn0 (pins 43/41) → CM4 RX | 5 GT/s diff |
| PCIe_CLK_P/N | CM4 CLK → M.2 REFCLKp/n (pins 55/53) | 100 MHz HCSL diff |
| PERST# | → M.2 pin 50 | 3.3 V CMOS |
| CLKREQ# | ↔ M.2 pin 52 (open-drain, pull-up) | 3.3 V OD |
| PEWAKE# | ← M.2 pin 54 (optional, pull-up) | 3.3 V OD |
| 3V3_M2 | M.2 pins 2/4/12/14/16/18/70/72/74 | Power, ≥2.5 A peak |

**Polarity and naming.** On M.2, PET is the SSD's transmitter and PER is its receiver. The host TX must land on PER, and the host RX on PET. Polarity inversion within a pair is tolerated by PCIe link training. Lane swaps are not relevant at x1.

---

## 2. Stackup and Impedance

The recommended stackup is JLC04161H-3313 or an equivalent thin-prepreg 1.6 mm, 4-layer build:

| Layer | Function | Dielectric to next |
|---|---|---|
| L1 | High-speed signals (PCIe preferred here) | 3313 prepreg ≈ 0.10 mm, εr ≈ 4.1 |
| L2 | Solid GND (no splits under PCIe) | Core ≈ 1.27 mm |
| L3 | Power pours, 3V3_M2 / 5V | 3313 prepreg ≈ 0.10 mm |
| L4 | Low-speed signals, secondary HS escape | — |

**Target impedance.** Use 90 Ω ±10 % differential. This is a common carrier compromise between PCIe CEM 85 Ω and the 90–100 Ω tolerated by the RK3566 and RPi CM4 guidance. It is also the value the CM4-ecosystem design guides specify.

| Parameter | L1 over L2 (starting point) | Notes |
|---|---|---|
| Trace width W | ~0.127 mm (5 mil) | Confirm with the fab's impedance calculator or a Polar solver |
| Pair gap S | ~0.15 mm (6 mil) | Edge-coupled microstrip, 1 oz finished copper plus solder mask |
| Zdiff | 90 Ω ±10 % | Order a controlled-impedance coupon |
| REFCLK | Same 90 Ω geometry | HCSL tolerates 85–100 Ω |

Run the field solver with the fab's actual prepreg thickness and mask thickness before freezing these numbers. A 7628 build (≈0.21 mm prepreg) needs roughly W 0.20 mm and S 0.20 mm instead.

---

## 3. Differential Routing Rules

| Rule | Value |
|---|---|
| Intra-pair skew (P vs N) | ≤ 0.127 mm (5 mil) per pair. Compensate within 5 mm of the mismatch source. |
| Inter-pair matching (TX vs RX vs CLK) | Not required; PCIe clock recovery is per-lane. Keep the lengths sensible anyway. |
| Max carrier length, CM4 connector → M.2 | ≤ 100 mm target (Gen2 budget is generous, but the module already consumes part of it) |
| Pair-to-pair spacing (TX ↔ RX) | ≥ 5 × W (≥ 0.65 mm). Prefer opposite sides of the GND-referenced corridor. |
| Spacing to other signals | ≥ 0.5 mm (≈ 4 W). ≥ 1 mm from switching regulators, USB3, Ethernet magnetics and clocks. |
| Spacing to plane-pour edge / board edge | ≥ 0.5 mm to pour on the same layer; ≥ 1.5 mm to board edge |
| Bends | 45° or arcs only; no 90° corners |
| Length-tune serpentines | Amplitude ≤ 2 × S; segment ≥ 3 W; gap between loops ≥ 4 W. Place near the mismatch, not mid-route. |
| Layer changes | ≤ 2 vias per net (≤ 1 pair transition preferred). P and N vias placed symmetrically. |
| Reference | Continuous L2 GND under the entire route, the AC caps, and the connector pads. Never cross a split or slot. |
| Coupling breaks | Uncoupled length ≤ 3 mm total (pad fan-out only) |

### AC Coupling

- **Default:** PCIe requires series 100 nF caps only on each transmitter output.
  - Host TX caps: the RPi CM4 places these on-module. Verify for the Radxa CM3 in its hardware design guide.
  - SSD TX caps: always on the SSD itself.
- **Expected outcome:** normally no caps are needed on the carrier.
- **If the Radxa datasheet shows TX caps absent on-module:**
  - Fit 100 nF X7R 0201 (0402 acceptable) on the TX pair.
  - Place them close to the CM4 connector, symmetrical, with the pad pitch matched to the pair.
  - Void L2 GND under the cap pads (anti-pad ≈ pad + 0.1 mm) to cancel pad capacitance.

### M.2 Connector Fan-out

- Enter pins straight and symmetrically.
- Void L2 under the SMT pads of PET, PER and REFCLK if the pad width exceeds 2 × W. This reduces the capacitive dip.
- Keep the 2242 and 2280 standoff screw pads away from the pairs. Tie the standoffs to GND.

---

## 4. High-Speed Return Path and Via Stitching

**Why stitching matters.** At 5 GT/s, the 2.5 GHz Nyquist frequency and the harmonics up to ~7.5 GHz drive the requirement. In FR4, λ at 7.5 GHz ≈ 19.5 mm. Stitching at ≤ λ/8 therefore gives a pitch of ≤ ~2.5 mm.

| Location | Rule |
|---|---|
| Signal-via return | Every PCIe signal via gets 1 GND via within 0.6 mm (centre-to-centre). Preferred: 2 GND vias, one each side of the pair, in a P-N-G symmetric pattern. |
| Reference-plane change (L1/L2 → L4/L3) | L3 is power. If the route moves to L4, it must reference GND. Either pour L4-adjacent GND or keep PCIe on L1 only. If unavoidable, place a 100 nF stitching cap (0402) L3↔L2 within 1 mm of the transition. Avoid this situation. |
| Route corridor fencing | GND vias along both sides of the pair corridor at 2.5 mm pitch, ≥ 0.5 mm from the nearest trace edge (≥ 4 W). Only where L1 has GND pour; don't pour copper closer than 0.5 mm to the pair. |
| CM4 connector region | Every GND pin of the Hirose DF40 footprint gets its own via to L2. No shared vias between pads. |
| M.2 connector region | All GND pins (3, 5, 11, 17, 23, 29, 35, 41-adjacent, 47-adjacent, 53-adjacent, 57, 71, 73, 75, …) via'd directly to L2. Add a GND via between each diff pair's pads row where geometry permits. |
| Board-wide plane stitching | L1/L4 GND pours to L2 on a ≤ 5 mm grid. Add a perimeter fence at ≤ 2.5 mm pitch for EMI. |
| Via geometry | 0.3 mm drill / 0.5–0.55 mm pad (standard). Anti-pad on L2/L3 for signal vias ≈ 0.85–0.9 mm. Keep the anti-pads of P and N merged into one oblong void, not two circles with a sliver. |
| Via stubs | Through-hole on 1.6 mm with L1→L4 transitions leaves no stub. L1→L3 transitions are disallowed for PCIe. |

---

## 5. Sideband, Clock and Power

- **PERST#:**
  - Drive from the CM4 PCIe reset GPIO.
  - Add a 10 kΩ pull-down so the slot is held in reset while the module boots.
  - Optional 100 pF to GND near M.2 for ESD/noise.
- **CLKREQ#:** 10 kΩ pull-up to 3V3. Tie to the CM4 CLKREQ pin, or to GND if always-on clock is acceptable (simplest for Gen2 x1).
- **PEWAKE#:** 10 kΩ pull-up. Can be left unconnected at the host.
- **REFCLK:**
  - Route as a 90 Ω pair under the same rules.
  - Keep ≥ 1 mm from the TX/RX pairs.
  - No series resistors unless the module datasheet calls for them (HCSL termination is on-module).
- **3V3_M2:**
  - Use a dedicated buck (≥ 3 A), or the 3V3 rail if the budget allows. NVMe drives can pull 2.5 A peaks.
  - Pour on L3 or wide L1/L4 copper (≥ 1.5 mm width, multiple vias).
  - Decoupling at the socket: 1 × 47 µF + 2 × 10 µF + 4 × 100 nF, the 100 nF parts within 3 mm of the 3V3 pins.
  - Keep the buck inductor ≥ 10 mm from the PCIe pairs.
- **LED/DAS (pin 10):** optional activity LED via 330 Ω to 3V3.
- **SUSCLK (pin 68):** leave NC (not needed for NVMe boot).

---

## 6. Cycle 12 Verification Checklist

1. ☐ Fab impedance stackup confirmed. W/S updated from the solver; coupon ordered at 90 Ω ±10 %.
2. ☐ DRC diff-pair rule set: 90 Ω, ±0.127 mm intra-pair, uncoupled ≤ 3 mm.
3. ☐ PET↔host RX and PER↔host TX mapping double-checked against the Radxa CM3 pinout.
4. ☐ AC-cap presence on-module verified. Carrier caps removed or added accordingly.
5. ☐ No PCIe trace crosses an L2 void, slot, or plane edge. Check with the copper-pour visualisation.
6. ☐ Every PCIe signal via has an adjacent GND via (≤ 0.6 mm).
7. ☐ Corridor fence ≤ 2.5 mm pitch; board stitching ≤ 5 mm grid.
8. ☐ Pad voids under the M.2 HS pads and any AC caps.
9. ☐ 3V3_M2 IR drop < 3 % at 2.5 A; decoupling placed.
10. ☐ Bring-up test:
    - `lspci -vv` shows LnkSta 5GT/s x1.
    - AER error counters stay at zero after a 30-minute `fio` run.
    - Repeat across 2230 and 2280 drives.

If you share the Cycle 11 stackup or layout review notes, I can tighten the W/S numbers and flag specific geometry issues.