# M.2 NVMe (PCIe Gen2 x1) Routing and Return-Path Design: Radxa CM4-Form-Factor Carrier, Cycle 10

## 0. Scope and Assumptions

| Item | Assumption (verify against your module) |
|---|---|
| Host | CM4-form-factor module (Radxa CM3 / CM3S or RPi CM4). PCIe **Gen2 x1, 5 GT/s** on the 100-pin connector (J2/"high-speed" side) |
| Device | M.2 2280/2242, **M-key**, NVMe, lane 0 only |
| Stackup | 4-layer, 1.6 mm, JLC04161H-7628 class: L1 Sig / L2 GND / L3 PWR / L4 Sig, prepreg ≈0.21 mm, εr ≈4.4 |
| Signals | PETx0± (host TX), PERx0± (host RX), REFCLK± (100 MHz HCSL), PERST#, CLKREQ#, PEWAKE# |

**Key frequency numbers:**
- Nyquist is 2.5 GHz.
- Design the return path to cover at least the 3rd harmonic (7.5 GHz).
- λ in FR4 at 7.5 GHz is about 19 mm, so λ/10 ≈ 1.9 mm. This drives the stitching pitch.

---

## 1. Differential Pair Parameters

### 1.1 Impedance Targets

| Net class | Zdiff | Tolerance | Notes |
|---|---|---|---|
| PCIE_TX0, PCIE_RX0 | **90 Ω** | ±10% | CM4 datasheet target. 85 Ω (PCIe CEM) is also acceptable. Pick one and keep it end-to-end. |
| PCIE_REFCLK | **90 Ω** (100 Ω ok) | ±10% | HCSL, no termination needed on the carrier if the module drives it terminated (check module datasheet) |
| Single-ended sideband | 50 Ω nominal | ±15% | PERST#, CLKREQ#, PEWAKE# (not critical) |

### 1.2 Geometry (L1 microstrip over L2 GND, H ≈ 0.21 mm)

| Parameter | Value | Notes |
|---|---|---|
| Trace width W | **0.20 mm** | Starting point. **Confirm with the fab's impedance calculator** and order controlled impedance. |
| Intra-pair gap S | **0.18 mm** | Yields ≈90 Ω edge-coupled |
| Pair-to-pair spacing (TX↔RX) | ≥ **5W ≈ 1.0 mm** | Or separate them with a GND pour plus via fence |
| Pair to other signals | ≥ **0.75 mm (≈3.5H)** | Clocks/USB/HDMI ≥ 1.5 mm |
| Pair to coplanar GND pour | ≥ **0.5 mm** | Keeps it microstrip-dominant. Closer pour lowers Z; recompute as CPWG if < 0.3 mm. |
| Pair to board edge / plane edge | ≥ **1.0 mm** | The reference plane must extend ≥ 3W beyond the pair on both sides |

If you route on L4 (over L3 PWR), the L3 plane must be solid and AC-stitched. Prefer **all-L1 routing**.

### 1.3 Length and Skew Rules

| Rule | Value |
|---|---|
| Intra-pair skew (P vs N) | **≤ 0.127 mm (5 mil)**. Compensate within 5 mm of the mismatch source using small serpentine bumps (amplitude ≤ 2S, length ≥ 3W). |
| Inter-pair (TX vs RX vs REFCLK) | **No matching required.** Lanes are independent and REFCLK is common-clock tolerant. |
| Max carrier trace length, TX/RX | **≤ 100 mm** (target < 50 mm). The CM4 module has its own internal routing; keep total loss < ~-8 dB @ 2.5 GHz for the carrier portion. |
| Max length, REFCLK | ≤ 100 mm |
| Bends | 45° or arc (R ≥ 3W). **No 90° corners.** |
| Necking at connector pins | Allowed for ≤ 2.5 mm, with uniform symmetric breakout |

### 1.4 AC Coupling

| Path | Caps on carrier? |
|---|---|
| Host TX → SSD RX (PETx) | **No.** The CM4-class module has TX caps on-module. **Verify in the Radxa CM3 hardware design guide.** If absent, add 100 nF 0201/0402 caps placed near the module, footprints symmetric, pads ≤ trace width + 0.1 mm, with a GND cutout on L2 under the pads (one pad-size void to reduce capacitive dip). |
| SSD TX → Host RX (PERx) | **No.** M.2 SSDs place AC caps on the card. |

### 1.5 Polarity and Pin Map (M.2 M-key)

| Signal | M.2 pin | Module side |
|---|---|---|
| PERp0 / PERn0 (SSD TX → host RX) | 47 / 49 | PCIE_RX_P/N |
| PETp0 / PETn0 (host TX → SSD RX) | 41 / 43 | PCIE_TX_P/N |
| REFCLKp / REFCLKn | 55 / 53 | PCIE_CLK_P/N |
| PERST# | 50 | PCIE_nRST (3.3 V, add a 10k pull-down so the SSD stays in reset while the module boots) |
| CLKREQ# | 52 | PCIE_CLK_nREQ (open-drain, 10k pull-up to 3.3 V) |
| PEWAKE# | 54 | Optional, pull-up to 3.3 V |

- P/N polarity inversion is legal in PCIe link training. **Do not rely on it**: route without swaps for clean debug.
- Never swap TX/RX naming. Host TX connects to **PET** (named from the host's perspective in M.2).

---

## 2. High-Speed Return Path and Via Stitching

### 2.1 Reference Plane Rules
1. L2 GND is **continuous** under the full length of every pair, plus ≥ 3W either side. **No splits, slots, or antipad chains** crossing under the pairs.
2. The pairs must not cross L3 power-split boundaries if any segment references L3.
3. Mounting-hole keepouts (M.2 standoff) must not intrude into the pair reference region.

### 2.2 Signal-Via Transitions (if a layer change is unavoidable)
- **Maximum 2 vias per signal** (1 transition) across the whole path.
- Signal via: 0.3 mm drill / 0.5 mm pad. Antipad on the planes: **0.8–0.9 mm diameter**, or a single oblong antipad around both vias of the pair.
- Pair vias are placed symmetrically, with via pitch ≈ 0.8–1.0 mm.
- **Return-path vias:** 2 GND vias per pair transition (1 is the minimum), placed **≤ 0.6 mm** (center-to-center ≤ 1 mm) from the signal vias, symmetric on the pair's axis.
- If the transition is L1(GND ref) → L4(PWR ref), add **2 × 100 nF 0201 stitching caps** L3↔GND within 2 mm of the transition, in addition to the GND vias.

### 2.3 Via Fence / Coplanar Stitching
| Location | Rule |
|---|---|
| Along TX/RX pairs (when a GND pour is beside them) | GND via row on each side, **pitch ≤ 2.0 mm** (λ/10 @ 7.5 GHz), offset ≥ 0.5 mm from the pair edge |
| Between TX and RX pairs | Single centered via fence, pitch ≤ 2.0 mm (crosstalk isolation, NEXT) |
| Around REFCLK | Fence both sides, pitch ≤ 2.5 mm |
| Pour edges / isolated copper | No floating copper. Every pour island has ≥ 2 vias, and no unstitched peninsula longer than 2 mm. |
| Via size | 0.3 mm drill / 0.5–0.6 mm pad (standard JLC) |

### 2.4 Connector Transitions

**Module connector (DF40 100-pin):**
- Tie every adjacent GND pin directly to L2 with **one via per GND pad**, within 0.5 mm of the pad.
- Ensure the GND pins flanking the PCIe pins have the shortest possible via path. These are the return path for the pair as it enters the board.

**M.2 connector:**
- Every GND pin adjacent to pins 41–55 gets its own via within 0.5 mm.
- Add 2–4 extra GND stitching vias between the connector body and the pair breakouts.
- Place the L2 plane fully under the connector pad field, with no void except standard pad antipads.

**M.2 standoff / mounting nut:** bond to GND with ≥ 4 vias around the pad. This provides a low-impedance chassis/shield path for the SSD ground.

### 2.5 Board-Level Stitching
- A perimeter GND via ring at ≤ 5 mm pitch around the M.2 region and around the board edge (EMI containment).
- A grid of L1/L4 GND pours tied to L2 at a ≤ 5 mm pitch in open areas.

---

## 3. Power Integrity for the SSD (affects the return path)
- **3.3 V rail at ≥ 3 A peak** (NVMe write bursts reach 2.5 A+). Use a dedicated buck or a high-current LDO if the module's 3.3 V output is limited. The CM4 3V3 output is **not** sufficient for most NVMe drives.
- Bulk capacitance at the M.2 power pins: 1 × 47 µF + 2 × 10 µF + 4 × 100 nF, located ≤ 3 mm from pins 2/4/70/72/74.
- Power-pin vias: ≥ 2 vias per pin group (0.3 mm), with a wide L3 pour.

---

## 4. DRC / Constraint Setup (KiCad/Altium net classes)

```
NetClass PCIE_DIFF   : W=0.20  Gap=0.18  Clear=0.75  Zdiff=90  SkewMax=0.127mm  ViaMax=2
NetClass PCIE_REFCLK : W=0.20  Gap=0.18  Clear=0.75  Zdiff=90  SkewMax=0.127mm
NetClass PCIE_SB     : W=0.20  Clear=0.20
Rule: PCIE_DIFF ↔ any other net clearance ≥ 0.75 mm (≥1.0 mm to TX/RX of other pair)
Rule: GND pour → PCIE_DIFF clearance 0.5 mm
```

---

## 5. Cycle 10 Verification Checklist

| # | Check | Pass criterion |
|---|---|---|
| 1 | Stackup confirmed with fab, impedance-controlled order | Fab report 90 Ω ±10% |
| 2 | Intra-pair skew report | ≤ 0.127 mm for all 3 pairs |
| 3 | Plane-crossing check (L2 under all pairs) | 0 crossings / voids |
| 4 | Signal via count | ≤ 2 per net; each transition has ≥ 2 GND vias ≤ 1 mm |
| 5 | Fence pitch | ≤ 2.0 mm along TX/RX |
| 6 | AC caps | Matches module guide (none on carrier for CM4-class) |
| 7 | Pinout | PET→41/43, PER→47/49, REFCLK→55/53, PERST#=50 |
| 8 | 3.3 V SSD rail | ≥ 3 A, ripple < 50 mVpp under load |
| 9 | Bring-up | `lspci` shows link at **5 GT/s x1**; `fio` sequential test stable 10 min; no AER errors in `dmesg` |
| 10 | Optional SI | TDR on coupon 90 Ω ±10%; S21 of carrier path > -5 dB @ 2.5 GHz |

---

### Deliverable Summary

- Route **TX, RX, and REFCLK on L1 only**, at **90 Ω (0.20/0.18 mm)** over solid L2 GND.
- Hold skew to **≤ 5 mil** and keep each pair **< 100 mm**.
- Add **GND via fences at ≤ 2 mm pitch** and **2 return vias at ≤ 1 mm** for any transition.
- Give every connector GND pin its own via.
- Place **no carrier AC caps** (confirm against the Radxa CM3 design guide).
- Provide a **dedicated ≥ 3 A 3.3 V rail** for the SSD.