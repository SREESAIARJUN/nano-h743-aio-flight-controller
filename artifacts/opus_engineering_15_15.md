# M.2 NVMe (PCIe Gen2 x1) Routing Specification: Radxa CM4-Form-Factor Expansion Board, Cycle 15

**Scope:** One PCIe 2.0 x1 link (5 GT/s) from the CM4-format module connector (Hirose DF40 pair) to an M.2 Key-M socket (2230/2242/2280), plus REFCLK, sideband signals and return-path stitching.

**Assumption:** a 4-layer, 1.6 mm board with an inner GND reference. If your Cycle 14 stackup differs, rerun the impedance numbers with your fab's calculator. Fab-specific parameters are marked **[verify]**.

---

## 1. Electrical Targets

| Parameter | Target | Notes |
|---|---|---|
| Differential impedance (TX/RX pairs) | **85 Ω ±10%** | PCIe CEM base. 90 Ω is acceptable if shared with USB pairs. Avoid 100 Ω. |
| Differential impedance (REFCLK) | 85–100 Ω ±10% | HCSL, 100 MHz |
| Single-ended impedance (per leg) | ~45–50 Ω | Coupling will pull this down |
| Data rate / Nyquist | 5 GT/s / 2.5 GHz | Design the return path to at least 7.5 GHz (3rd harmonic) |
| Intra-pair skew | **≤ 5 mil (0.127 mm)** | Match at the mismatch point, not just at the end |
| Inter-pair skew (TX vs RX) | No requirement | Lanes are independent; keep within 1 inch for neatness |
| REFCLK intra-pair skew | ≤ 5 mil | |
| REFCLK vs data skew | No requirement | Common-clock architecture |
| Max carrier trace length | **≤ 100 mm (4 in) target**, 150 mm absolute | Module-side loss consumes part of the budget |
| Insertion loss (carrier only) | < 3 dB at 2.5 GHz | Gen2 total channel budget is about 13 dB |
| Layer transitions per pair | **≤ 2 vias per leg** (ideally 0–1) | |
| AC coupling caps | See §4 | |

---

## 2. Stackup and Geometry (JLC-class 4-layer, 1.6 mm, 7628 prepreg) [verify]

| Layer | Use | Thickness |
|---|---|---|
| L1 | High-speed signals (PCIe) | 1 oz outer copper |
| Prepreg | 7628, εr ≈ 4.4 | ~0.21 mm (8.3 mil) |
| L2 | **Solid GND, no splits under PCIe** | 0.5 oz |
| Core | | ~1.065 mm |
| L3 | Power (3V3 plane for M.2) | 0.5 oz |
| Prepreg | 7628 | ~0.21 mm |
| L4 | Low-speed signals, GND pour | 1 oz |

**Edge-coupled microstrip on L1 over L2 (starting values):**

| Target | Trace width W | Gap S | Pitch to other pairs |
|---|---|---|---|
| 85 Ω diff | **0.20 mm (7.9 mil)** | **0.13 mm (5.1 mil)** | ≥ 3W edge-to-edge (0.6 mm); ≥ 5W next to clocks or switchers |
| 90 Ω diff | 0.18 mm | 0.15 mm | same |
| 100 Ω REFCLK (optional) | 0.15 mm | 0.18 mm | same |

- Add an **impedance-controlled coupon** and specify "85 Ω diff ±10% on L1, ref L2" in the fab notes.
- If you add GND pour on L1 next to the pairs (coplanar), keep pour clearance **≥ 3W (≥ 0.6 mm)**. Closer pour lowers impedance, so recalculate as CPWG if you go tighter.

---

## 3. Topology and Pin Map

```
CM4 module (DF40)                         M.2 Key-M socket
 PCIE_TX_P/N  ───────────────────────────► PERp0 / PERn0  (pins 43 / 41)
 PCIE_RX_P/N  ◄─────────────────────────── PETp0 / PETn0  (pins 49 / 47)
 PCIE_CLK_P/N ───────────────────────────► REFCLKp / REFCLKn (pins 55 / 53)
 PCIE_nRST    ───[3V3 level, RC opt.]─────► PERST#  (pin 50)
 PCIE_CLK_nREQ◄───────────────────────────  CLKREQ# (pin 52, open-drain, pull-up on carrier)
                                           PEDET (pin 69): NC or GND per SSD type
 3V3 (≥3 A rail) ─────────────────────────► pins 2,4,12,14,16,18,70,72,74
```

**Naming:** host TX → SSD **PER** (SSD receive); SSD **PET** → host RX. Getting this crossed is the most common error. Cross-check it in the schematic review.

**Polarity:** P/N inversion within a pair is tolerated by PCIe link training. You may swap P/N to avoid a crossover, but document it. **Never swap TX and RX.**

**Radxa module variant [verify]:** confirm which DF40 pins carry PCIe on your module. Radxa CM3 (RK3566) and CM5 follow the RPi CM4 pinout for PCIe Gen2 x1, but check the current Radxa datasheet. Also check whether the module's PCIe clock request and reset are 3.3 V tolerant.

---

## 4. AC Coupling Capacitors

- **Host TX → SSD:** the RPi CM4 places TX AC caps on the module, and Radxa CM-format modules are believed to do the same **[verify against the module schematic]**.
  - If the caps are present on the module, **do not add caps on the carrier.**
  - If they are absent, place **100 nF X7R, 0201 or 0402**, near the transmitter (module connector side), within 10 mm.
- **SSD TX → host RX:** the M.2 SSD provides its own TX caps. **Do not add caps on the carrier.**
- If you do place caps:
  - Place both caps of the pair side by side and symmetric.
  - Void L2 under the pads only (an antipad roughly the pad size plus 0.1 mm) to cancel the pad's excess capacitance.
- Optional DNP footprints cost little and let you fix a Cycle 15 surprise without a respin.

---

## 5. Routing Rules

1. **Route on L1 only** from the DF40 to the M.2 socket wherever possible. Both connectors are SMT on top, so zero vias are achievable. This is the preferred Cycle 15 layout.
2. **Reference plane:** L2 GND must be continuous under the full length plus 3W on each side. No splits, slots or via-antipad chains crossing under a pair.
3. **Bends:** use 45° or arcs only. Keep corner skew compensation within 15 mm of the bend that causes it, using small serpentine bumps:
   - amplitude ≤ 2× gap
   - bump length ≥ 3W
   - put the compensation on the short leg
4. **Pin breakout:** uncouple for no more than 3 mm at the DF40 and M.2 pads. Neck down if needed, but keep the pair symmetric.
5. **Spacing:**
   - ≥ 0.6 mm (3W) between the TX and RX pairs. Use 5W if they run in parallel for more than 10 mm.
   - ≥ 1.0 mm from REFCLK to the data pairs.
   - ≥ 2 mm from any switcher node, its inductor, or SD/HDMI/USB lines.
6. **Board edge:** keep ≥ 1 mm from the board edge and ≥ 0.5 mm from the L2 plane edge (spec ≥ 20H).
7. **No stubs:** no test points on the data pairs. If probing is needed, use in-line 0 Ω-free pads only. REFCLK test points must be on-pad.
8. **M.2 socket pads:** on 0.5 mm pitch, enter the pads straight. If impedance discontinuity is a concern, void L2 under the M.2 signal pads (TDR-verified designs often skip this at Gen2).
9. **Length matching:** done in KiCad/Altium per pair, tolerance 0.127 mm. Include via length if any vias exist (use the actual stackup depth).

---

## 6. Return-Path Via Stitching

### 6.1 At any signal layer transition (if a via is unavoidable)
- Place **2 GND return vias per differential via pair**, one on each side, placed symmetrically.
- Keep them **≤ 0.75 mm (30 mil) center-to-center** from the signal vias.
- Signal vias:
  - drill 0.3 mm, pad 0.5 mm
  - antipad 0.8–0.9 mm diameter, **merged oblong antipad** around both P/N vias
  - spacing P-to-N via of about 0.8–1.0 mm
- Tie both reference planes (e.g. L2 GND ↔ L4 GND pour) with these vias. If the reference changes from GND to the 3V3 plane, add **a 100 nF stitching cap within 1.5 mm** of the transition. Avoid GND→PWR reference changes altogether if possible.
- Backdrill is not needed: the stub is under 1.6 mm, so its resonance is well above 15 GHz.

### 6.2 Along coplanar-flanked routes (if GND pour on L1)
- Use a **via fence pitch ≤ 2.5 mm** along both pour edges facing the pairs.
- Derivation: the 3rd harmonic is 7.5 GHz, and λ in FR4 (εeff ≈ 3.4) is about 21.7 mm. λ/8 gives ~2.7 mm, which rounds to 2.5 mm.
- Fence vias: 0.3 mm drill, 0.6 mm pad, ≥ 0.6 mm from the pair edge.
- Remove any isolated pour slivers shorter than 5 mm or not stitched at both ends (antenna risk).

### 6.3 Connector regions
- **DF40 connector:** place GND vias at every module GND pin adjacent to the PCIe pins, within 0.5 mm, via-in-pad or dog-bone. Add an extra row of 4–6 GND vias along the DF40 edge near the PCIe pin cluster.
- **M.2 socket:**
  - Give every GND pin near the PCIe group (pins 39, 45, 51, 57) its own via, ≤ 0.5 mm from the pad.
  - Add ≥ 2 GND vias at the socket mounting/shell tabs.
  - Stitch the standoff/screw pad to GND with ≥ 4 vias.
- **AC caps (if used):** place 1 GND via on each side of the cap pair.

### 6.4 Global stitching
- Use a GND stitching grid of **≤ 5 mm pitch** across the PCIe region, and ≤ 10 mm board-wide.
- Under the M.2 SSD body, keep L1 as GND pour (no routing) and stitch it at 5 mm. This also helps SSD heat spreading.

---

## 7. REFCLK and Sideband

- **REFCLK:** HCSL, 85–100 Ω diff, length ≤ 150 mm.
  - Series 0 Ω / 33 Ω footprints are optional. CM4-class modules usually terminate internally **[verify]**.
  - Keep the 1 mm spacing rule from §5.
  - Use stitching rules identical to the data pairs.
- **PERST#:** add a 10 kΩ pull-down if the module does not drive it during boot. Optionally add a 100 nF/10 kΩ RC to delay deassertion by at least 100 ms after 3V3 is stable.
- **CLKREQ#:** 10 kΩ pull-up to 3V3. Some SSDs need CLKREQ# grounded if the host ignores it. Add a DNP 0 Ω to GND as an option.
- **3V3 for M.2:** route as a plane or a pour of at least 1.5 mm.
  - Budget 3 A peak (Gen2 x1 SSDs draw about 2.5 A at worst).
  - Place bulk 47 µF plus 10 µF, and 4 × 100 nF close to pins 2/4 and 70/72/74.
  - Use a load switch with soft start (e.g. TPS22918) to prevent module brownout on hot insertion.

---

## 8. Cycle 15 Verification Checklist

- [ ] Fab note: 85 Ω ±10% diff, L1 ref L2, with test coupon
- [ ] DRC net class "PCIE_DIFF": W 0.20 / S 0.13, clearance 0.6 mm, max uncoupled 3 mm
- [ ] Length-match report: each pair ≤ 0.127 mm skew
- [ ] Zero or ≤ 2 vias per leg, each with 2 return vias ≤ 0.75 mm away
- [ ] Ratsnest/plane check: no L2 voids or splits under PCIe nets (overlay review)
- [ ] TX/RX naming cross-check (host TX → PER, PET → host RX)
- [ ] AC-cap ownership confirmed against the module schematic
- [ ] PEDET, PERST#, CLKREQ# options populated or DNP as intended
- [ ] Bring-up test: run `lspci -vv` and confirm `LnkSta: Speed 5GT/s, Width x1`. Check that the AER error counters stay at zero after a 30-minute `fio` run.
- [ ] Optional: TDR on the coupon; target 85 Ω ±8.5 Ω

If you share the Cycle 14 stackup or the module variant (CM3, CM3I or CM5), I can recompute the exact W/S, confirm the pin numbers, and tighten the loss budget.