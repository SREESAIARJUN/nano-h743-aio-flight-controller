# M.2 NVMe (PCIe Gen2 x1) Routing and Return-Path Design: Radxa CM4-Form-Factor Carrier

**Scope note:** I don't have access to your internal "Cycle 22" files, such as the stackup, netlist or prior review notes. This is a complete design spec built on stated assumptions. Swap in your fab's actual stackup numbers and the pin/cap details from the module datasheet you're using (Radxa CM3, CM3S, CM5, or Raspberry Pi CM4) before freezing.

---

## 1. Interface Summary

| Item | Value |
|---|---|
| Link | PCIe Gen2, x1 lane, 5 GT/s (8b/10b), Nyquist 2.5 GHz |
| Bandwidth of concern | DC to ~7.5 GHz (3rd harmonic) |
| Diff pairs | PCIe_TX± (CM to SSD), PCIe_RX± (SSD to CM), PCIe_REFCLK± (100 MHz HCSL) |
| Sideband | PERST#, CLKREQ#, PEWAKE#, plus SUSCLK (optional) and PEDET (optional) |
| Connector | M.2 Key M socket, 2242/2280 mount positions |
| Power | 3.3 V at ≥3 A capable rail at the socket |

---

## 2. Stackup Assumption (4-layer, 1.6 mm, 1 oz outer)

| Layer | Use | Notes |
|---|---|---|
| L1 | Signal: all PCIe pairs | Microstrip referenced to L2 |
| (Prepreg) | ~0.10 mm, Er ≈ 4.1 (e.g. 3313-class) | |
| L2 | **Solid GND**, unbroken under all PCIe routes | |
| (Core) | ~1.2 mm | |
| L3 | Power (3V3, 5V pours) | |
| (Prepreg) | ~0.10 mm | |
| L4 | Signal / GND pour | Use only for unavoidable PCIe escapes |

On a 6-layer board, put PCIe on L1 over L2 GND as well, and keep L3/L4 stripline as a fallback with GND on both sides.

---

## 3. Differential Routing Parameters

| Parameter | Target | Notes |
|---|---|---|
| Differential impedance | **85 Ω ±10%** | PCIe CEM target. 90 Ω ±10% is also acceptable and matches many CM-module datasheets. Pick one and keep the whole channel consistent. |
| Single-ended impedance | ~42–45 Ω | Falls out of the diff geometry |
| Trace W / S (L1, 0.1 mm prepreg) | **~0.15 mm / 0.20 mm (starting point)** | Final values must come from the fab's impedance calculator, with coupon-controlled impedance ordered. |
| Pair-to-pair spacing | ≥ 3×S (≥0.6 mm), ≥5×H preferred | TX-to-RX spacing matters most. Never interleave them. |
| Spacing to other signals | ≥ 0.75 mm (≥5H) | Keep SDIO, USB and HDMI clear |
| Spacing to plane/pour edge | ≥ 3× trace width | Avoid edge coupling to L1 copper pour |
| Intra-pair skew (P vs N) | **≤ 0.127 mm (5 mil)** | Compensate within ~2 mm of the mismatch source (bend or pad), not at the far end |
| Lane-to-lane matching | N/A | x1 only. TX and RX are independent. |
| REFCLK intra-pair skew | ≤ 0.127 mm | REFCLK-to-data length matching is not required |
| Max carrier trace length | ≤ 100 mm recommended, ≤ 150 mm hard | Keeps channel loss comfortably inside the Gen2 budget with the module's own routing |
| Max vias per signal | ≤ 2 per net (1 transition) | 0 is ideal: route entirely on L1 |
| Bends | 45° or arcs, no 90° | Use symmetrical serpentine with amplitude ≤ 2×S and segment length ≥ 3×W |
| Breakout neck-down | Allowed ≤ 3 mm at connector pins | Keep the pair coupled through the neck-down |

### AC Coupling Capacitors

- **Host TX (CM to SSD):** Check the module datasheet. RPi CM4 and Radxa CM3-class modules typically place TX caps on-module. If they are absent, place 220 nF 0201/0402 caps on the carrier near the module connector.
- **SSD TX (into host RX):** Caps are on the SSD per the M.2 spec. **Do not** add caps on the carrier.
- **Cap footprint compensation:** Void L2 GND directly under the 0402 cap pads (pad-sized cutout), with L3 as the new reference. Alternatively, use 0201 parts and skip the void.
- Place the P and N caps side-by-side and symmetrical. Never stagger them.

### Connector Interface (M.2 Socket and Module B2B)

- Enter the pins straight. Keep the pair symmetric right up to the pads.
- Optionally void L2 under the M.2 signal pads (SMT pad capacitance compensation). Check with TDR or a field solver first; at Gen2 this is usually optional.
- Pin mapping: socket PETp0/PETn0 is the **SSD transmitter**, so it connects to the host RX. Verify that the polarity and TX/RX crossover are labelled from the host perspective in the schematic. This is a classic re-spin bug. PCIe tolerates P/N polarity inversion, but do not rely on it for REFCLK.

---

## 4. High-Speed Return Path and Via Stitching

### 4.1 Reference Plane Rules

1. L2 is solid GND under every PCIe trace, including the full width of the pair plus ≥3W on each side. No splits, slots or anti-pad chains crossing beneath.
2. Do not route over L2 voids created by other via fields, such as the B2B connector's pin field. Check the anti-pad merge in the via fields under the module connectors.
3. Do not route PCIe over the board edge or within 1 mm of it.

### 4.2 Signal Via Transitions (if any)

| Rule | Value |
|---|---|
| Signal via | 0.25–0.3 mm drill, 0.45–0.5 mm pad |
| Anti-pad on planes | ~0.8–0.9 mm diameter. A shared oblong anti-pad around both P/N vias is fine. |
| Via pitch (P to N) | Same as pair pitch, roughly 0.7–1.0 mm |
| **GND return vias** | **2 per diff-pair transition** (1 minimum), placed symmetrically, within **≤ 1.0 mm** (≤0.6 mm preferred) of the signal vias |
| Transition preference | L1 to L4 (full-through, no stub). Avoid L1 to L3 on a 1.6 mm board, which leaves a ~1.4 mm stub. |
| Reference change GND to PWR | Avoid. If unavoidable, add a 100 nF (0402) stitching cap from PWR to GND within 1 mm of the transition. |

**Pattern (top view):** `G  P  N  G` in a line, or the G vias at ±45° of the pair centreline. Keep it symmetric so both legs see the same return.

### 4.3 Ground Via Fencing and Stitching

- **L1 coplanar pour:** If L1 has GND pour near the pairs, keep ≥3W clearance and stitch the pour edge to L2 every **≤ 2.5 mm**. λ at 7.5 GHz in FR-4 is about 22 mm, so this gives roughly λ/10 or better. Never leave floating or poorly stitched copper slivers near pairs; remove them.
- **Fence along PCIe channel:** One row of GND vias on each side of the TX/RX bundle at 2–3 mm pitch, ≥1 mm from the nearest trace edge. Also run a fence between TX and RX when they are closer than 1 mm over a long run.
- **Connector stitching:**
  - M.2 socket: at least 1 GND via per GND pin, placed directly at the pad. Add extra GND vias at the socket shell/tabs.
  - Module B2B connector: GND via on every GND pin adjacent to PCIe pins, with no shared vias between pins.
- **Board-level:** Stitch L1/L4 GND pours to L2 on a ~5 mm grid across the board, plus an edge fence at ≤ 3 mm.
- **REFCLK:** Same rules. Route REFCLK away from TX/RX, separated by a GND fence or ≥5H.

---

## 5. Sideband and Power

| Signal | Rule |
|---|---|
| PERST# | Host-driven, 10 k pull-up/down per module guidance. Route away from REFCLK. |
| CLKREQ# | Open-drain, 10 k pull-up to 3V3. Can tie low if the module doesn't support CLKREQ. |
| PEWAKE# | Open-drain, 10 k pull-up |
| 3V3 to socket | ≥ 1.5 mm pour or plane. Bulk 22–47 µF plus 2× 10 µF plus 4× 100 nF at the socket pins. Load switch with soft-start recommended (SSD inrush). |
| Rail sizing | ≥3 A capable. NVMe peaks of 2.5 A+ on writes are common. |

---

## 6. DRC / Constraint Manager Rules (copy into the CAD tool)

```
NetClass PCIE_DIFF: PCIE_TX_P/N, PCIE_RX_P/N, PCIE_CLK_P/N
  Layer L1: W=0.150 mm, Gap=0.200 mm   (fab-verified for 85 Ω)
  Layer L4: W=0.150 mm, Gap=0.200 mm   (if symmetric stackup)
  Diff impedance: 85 Ω ±10%
  Intra-pair max skew: 0.127 mm
  Max length (carrier): 100 mm (warn) / 150 mm (error)
  Max vias per net: 2
  Clearance to other nets: 0.75 mm
  Clearance pair-to-pair: 0.60 mm
  Uncoupled length max: 3.0 mm
```

---

## 7. Verification Checklist

- [ ] Fab impedance coupons ordered (85 Ω diff, L1 and L4)
- [ ] TX/RX crossover verified, host vs. M.2 naming
- [ ] AC caps: present exactly once per direction
- [ ] No pair crosses an L2 split or anti-pad chain (run a plane-overlay check)
- [ ] Every layer transition has ≥1 (ideally 2) GND vias within 1 mm
- [ ] Skew ≤ 0.127 mm, compensated near the source
- [ ] Pour slivers removed. Pour edges stitched at ≤ 2.5 mm.
- [ ] 3V3 socket rail tested at ≥3 A with soft-start
- [ ] Bring-up: `lspci -vv` shows `LnkSta: Speed 5GT/s, Width x1`. Run `fio` sustained-write for 30 min, then check AER/dmesg for correctable errors.

---

If you share the actual Cycle 22 stackup (layer count, dielectric thicknesses, Er) and which module (Radxa CM3, CM5, or Pi CM4) is the target, I can compute exact W/S values and the per-module length budget.