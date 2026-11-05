# M.2 NVMe (PCIe Gen2 x1) Routing and Return-Path Stitching Spec: Radxa CM4-Class Carrier, Cycle 11

**Scope:** A single PCIe Gen2 x1 link (5 GT/s) from the compute module's board-to-board connector to an M.2 M-key socket (2242/2280 SSD).

**Assumptions to verify against your Cycle 10 baseline:**
- **Stackup:** a 4-layer, 1.6 mm stackup. Section 1 also gives a 6-layer option.
- **Signal list:** the module exposes PCIe TX, RX, REFCLK, PERST#, CLKREQ# and WAKE#.
- **Pin map:** confirm the exact pin mapping and on-module AC-cap population against the specific Radxa module datasheet (CM3, CM5 or CM4-compatible). It differs between Radxa and Raspberry Pi modules.

---

## 1. Stackup and Impedance Targets

| Parameter | Target | Notes |
|---|---|---|
| Differential impedance (TX/RX) | **85 Ω ±10 %** | PCIe base spec value. 90 Ω is acceptable if the fab can't hit 85; avoid 100 Ω. |
| Differential impedance (REFCLK, HCSL) | **85–100 Ω ±10 %** | Use the same geometry as the data pairs for simplicity. |
| Single-ended reference | ~42–50 Ω | Falls out of the differential geometry. |
| Reference plane | **Solid GND, adjacent layer** | Never reference PWR or a split plane. |

### Option A: 4-layer, 1.6 mm (e.g., JLC 7628-type)

Layers: L1 Sig / L2 GND / L3 PWR / L4 Sig. Prepreg L1–L2 ≈ 0.21 mm, εr ≈ 4.4.

Starting geometry for 85 Ω microstrip on L1:
- **W = 0.28 mm, S = 0.20 mm** (≈ 11 / 8 mil).
- Confirm with the fab's impedance calculator and order **controlled-impedance with coupons**.

Routing layers: route PCIe **only on L1**. L4 is referenced to the PWR plane, so it is unsuitable.

### Option B: 6-layer (recommended if Cycle 11 allows a cost increase)

Layers: L1 Sig / L2 GND / L3 Sig / L4 Sig / L5 GND / L6 Sig.

Use L1 (microstrip) or L3 (stripline, between GND L2 and the L4/GND reference). Stripline lowers EMI and crosstalk.

---

## 2. Differential Pair Routing Rules

| Rule | Value |
|---|---|
| Intra-pair skew (P vs N) | **≤ 0.13 mm (5 mil)** |
| Skew compensation location | Within 5 mm of the mismatch source (pin escape or bend). Use small serpentine bumps, amplitude ≤ 2×S, length ≥ 3×W. |
| Inter-pair matching (TX vs RX) | **Not required.** Lanes are independent and the clock is embedded. |
| REFCLK skew | ≤ 0.25 mm intra-pair. No matching to data required. |
| Max total length, connector → M.2 pin | **≤ 100 mm recommended.** Gen2 budget allows ~150 mm on FR-4, but keep margin for the module-side trace. |
| Pair-to-pair spacing | **≥ 3×W** (≈ 0.85 mm). Use ≥ 5×W between TX and RX pairs to limit NEXT. |
| Spacing to other signals | ≥ 5×W, or ≥ 1.5 mm to clocks, switching regulators and USB 3. |
| Spacing to plane/pour edge | ≥ 3×W from any copper-pour edge or plane split. |
| Bends | 45° or arcs only. No 90° corners. Keep pairs symmetric through bends. |
| Vias per net | **≤ 2** (ideally 0 on 4-layer, routing L1 only). |
| Stubs | None. Test points must be inline pads, not T-stubs. Backdrill or use via-in-pad only if unavoidable. |
| Neck-down at pins | Allowed for ≤ 2 mm only, to escape the B2B connector or M.2 pads. |
| Over gaps/splits | **Prohibited.** Check L2 under the entire route, including under the M.2 socket. |

### AC Coupling Capacitors
- **Host TX → SSD RX:** caps are normally on the compute module. Do **not** add a second set. Verify this per module datasheet.
- **SSD TX → Host RX:** caps are on the M.2 card per the M.2 spec. Place **none** on the carrier.
- **If the module lacks TX caps:**
  - Place 0402 (preferably 0201) **220 nF X7R** caps at the module end.
  - Make the pad pairs symmetric.
  - Use a GND-plane cutout under the pads, 1×–2× pad size, on L2, to reduce capacitive dip. On 4-layer, only cut L2 if L3 is GND; otherwise skip the cutout.

### Polarity and Direction
M.2 pin naming is from the **host's perspective**:
- **PETp0/n0** (pins 49/47) = host transmit → connect to the module's **PCIe_TX**.
- **PERp0/n0** (pins 43/41) = host receive → connect to the module's **PCIe_RX**.
- **REFCLKp/n** = pins 55/53.

Swapping TX and RX is the single most common bring-up failure, so cross-check direction in the schematic review. P/N inversion within a pair is tolerated by link training, but avoid it anyway.

---

## 3. High-Speed Return Path and Via Stitching

### 3.1 Signal-Via Return Vias (Layer Transitions)
Every layer change of a pair must carry its return current to the new reference plane:
- Place **2 GND vias per differential via pair**, one on each side, symmetric.
- Keep them **≤ 1.0 mm center-to-center** from the signal vias (target 0.8 mm).
- If the two reference planes are different nets (e.g., GND ↔ PWR on 4-layer), add a **100 nF 0402 stitching cap within 1 mm** of the transition. Better still, avoid the transition entirely.
- Signal via: 0.3 mm drill / 0.55–0.6 mm pad. Use antipad ≈ 0.9–1.0 mm and a shared oval antipad for the pair, sized for ~85 Ω.

### 3.2 Guard Stitching Along the Route
Gen2 has 2.5 GHz Nyquist; consider content up to ~7.5 GHz (3rd harmonic). In FR-4, λ ≈ 20 mm at 7.5 GHz, which sets the pitch limits below.

| Item | Value |
|---|---|
| Stitching via pitch along the pair corridor | **≤ 2.5 mm** (≈ λ/8 at 7.5 GHz). Use ≤ 2 mm near the connectors. |
| Stitching via offset from pair edge | ≥ 3×W (≈ 0.85 mm), so impedance is unaffected. |
| Coplanar GND pour on L1 next to pairs | Optional. If used, keep ≥ 3×W clearance, stitch at ≤ 2.5 mm, and **remove all floating or unstitched slivers**. |
| L2-GND ↔ L3/L4 pour stitching, board-wide | Grid ≤ 5 mm under and around the PCIe region. |

### 3.3 Connector Regions
- **B2B module connector:**
  - Tie every GND pin adjacent to the PCIe pins directly to L2 with its own via, placed ≤ 0.5 mm from the pad.
  - Do not share one via between multiple GND pins.
- **M.2 socket:**
  - Via every GND pin (pins 39, 45, 51, 57, etc. along the PCIe group) to L2 with a dedicated via.
  - Add a stitching via row at ≤ 2 mm pitch across the socket mouth.
  - Ensure L2 is solid under the socket body; no PWR islands intrude.
- **Mounting standoff:**
  - Connect the M.2 standoff (2242/2280 positions) to GND with 4+ vias.
  - This gives the SSD a low-impedance chassis/return reference.

### 3.4 Plane Integrity Checks
1. Run an L2 continuity check under all PCIe nets and verify zero crossings of voids, antipad chains or splits.
2. Check antipad chains under the M.2 and B2B connectors. Make sure overlapping antipads don't create a slot under the pair; resize or stagger them if needed.
3. On 4-layer, keep the 3.3 V M.2 supply routing on L3/L4, away from the region below the L1 PCIe pairs.

---

## 4. Sideband and Power Signals

| Signal | Rule |
|---|---|
| PERST# (pin 50) | Module GPIO/PERST, 3.3 V. Add a 10 k pull-down if the module doesn't drive it at reset. Keep it away from the pairs. |
| CLKREQ# (pin 52) | Open-drain, with a 10 k pull-up to 3.3 V. Tie to GND if the module refclk is always-on and the module has no CLKREQ input. |
| PEWAKE# (pin 54) | Open-drain, with a 10 k pull-up. Can be left unconnected if unused. |
| 3.3 V supply (pins 2/4/12/14/16/18/70/72/74) | 3 A capable rail. Place 22 µF + 4×100 nF at the socket. Fit a load switch with soft-start to limit inrush (~1–2 A). |
| SUSCLK (pin 68) | NC, or 32.768 kHz if available. |
| DAS/DSS LED (pin 10) | Optional activity LED. |

---

## 5. Verification Checklist (Cycle 11 Sign-off)

**Design rules**
- [ ] Net classes created: `PCIE_DP` (85 Ω), `PCIE_REFCLK` (85–100 Ω), with DRC width and gap enforced.
- [ ] Intra-pair skew report ≤ 0.13 mm (data) and ≤ 0.25 mm (refclk).
- [ ] Every PCIe via has ≥ 2 return GND vias within 1 mm.
- [ ] No reference-plane crossing (screenshot L2 under the overlay for review).

**Topology and connectivity**
- [ ] TX/RX direction verified against the module datasheet and M.2 host-perspective naming.
- [ ] AC caps exist exactly once per direction: module side for host TX, card side for host RX.

**Fabrication**
- [ ] Fab notes specify the controlled impedance, a test coupon, and a ±10 % tolerance.

**Bring-up**
- [ ] `lspci -vv` shows `LnkSta: Speed 5GT/s, Width x1`.
- [ ] AER errors are clean under a sustained `fio` load.

---

**Deltas to confirm from earlier cycles:** if Cycle 10 used 90 Ω or 100 Ω geometry, update the net classes and re-run impedance with the fab before release. If the module datasheet lists a different max carrier trace length, use the stricter of that value and 100 mm.