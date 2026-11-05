# M.2 NVMe (PCIe Gen2 x1) Routing and Return-Path Design: Radxa CM4-Form-Factor Carrier

> **Assumptions:** I don't have your "Cycle 23" design history, so this spec is self-contained. It assumes:
> - The compute module is a Radxa CM-series board in the CM4 form factor (e.g., Radxa CM3 / RK3566) exposing **one PCIe 2.x lane** on the Hirose DF40 board-to-board connectors.
> - The carrier has a **Key-M M.2 2230/2242/2280 socket**.
>
> Confirm pin mapping and on-module AC caps against the Radxa module schematic before layout.

---

## 1. Interface Summary

| Signal group | Nets | Type | Notes |
|---|---|---|---|
| Host TX → SSD RX | PCIE_TX_P/N | Diff, AC-coupled | Caps on carrier unless the module already has them |
| SSD TX → Host RX | PCIE_RX_P/N | Diff | The SSD card has its own TX caps. Do **not** add caps on the carrier. |
| Reference clock | PCIE_CLK_P/N | HCSL 100 MHz diff | Polarity swap is **not** allowed |
| Sideband | PERST#, CLKREQ#, PEWAKE# | 3.3 V CMOS, open-drain for CLKREQ#/WAKE# | 10 k pull-ups to 3.3 V |
| Power | 3V3_M2 | 3.3 V, ≥2.5 A peak (budget 3 A) | Dedicated pour/plane |

**Signal facts:**
- Gen2 runs at 5 GT/s, so the Nyquist frequency is **2.5 GHz**.
- Design the channel for clean behaviour to roughly **7.5 GHz** (third harmonic).

---

## 2. Stackup

### Option A: 4-layer, 1.6 mm, thin prepreg (recommended minimum)

| Layer | Use | Dielectric to next |
|---|---|---|
| L1 | PCIe pairs, components | ~0.10 mm prepreg (e.g., 3313, εr ≈ 4.1) |
| L2 | **Solid GND**, unbroken under all PCIe routes | ~1.26 mm core |
| L3 | Power (3V3_M2, 5V), slow signals | ~0.10 mm prepreg |
| L4 | Slow signals, GND pour | — |

### Option B: 6-layer (if the board is dense)

The layer order is **S1 / GND / S2 / PWR / GND / S3**. Route PCIe on S1 (microstrip) or S2 (stripline between GND and PWR). Stripline is preferred only if PWR is stitched to GND with decoupling caps along the route.

> Avoid 4-layer stackups with thick 7628 prepreg (~0.2 mm). They force ~0.3 mm traces that won't escape the DF40 or M.2 pads cleanly.

---

## 3. Differential Routing Parameters

| Parameter | Target | Notes |
|---|---|---|
| Differential impedance | **85 Ω ±10%** | PCIe/M.2 base spec. 90 Ω is acceptable if the fab can't hit 85 Ω. Don't use 100 Ω. |
| Single-ended impedance | ~42–50 Ω | Result of the geometry |
| Trace W / S (Option A, L1 over L2) | **≈0.14 mm / 0.13 mm** (starting point) | **Get final values from the fab's impedance calculator** and order controlled impedance |
| RefClk impedance | 85–100 Ω diff | Same geometry as data pairs is fine |
| Intra-pair skew (P vs N) | **≤ 0.13 mm (5 mil)** | Compensate within ~2 mm of the mismatch source using small bumps, not one big serpentine |
| Inter-pair skew (TX vs RX) | Not required | Lanes are independently deskewed |
| RefClk vs data | Not required | Common-clock architecture |
| Max carrier length (DF40 → M.2 pad) | **≤ 100 mm**, target < 60 mm | Module traces add more length, so keep the carrier short |
| Pair-to-pair spacing | **≥ 3× trace-to-plane height**, prefer ≥ 0.5 mm | Separate TX and RX pairs especially |
| Pair to other signals | ≥ 5× height (≥ 0.6 mm) | Keep SDIO, Ethernet, USB, and switching nodes ≥ 2 mm away |
| Distance to plane edge / board edge | ≥ 1 mm from any GND plane edge | Don't route over voids |
| Corners | 45° or arcs | No 90° corners |
| Layer changes | **≤ 2 vias per net**, ideally 0 | Route L1 end-to-end where possible |
| Polarity | Data P/N may be swapped (link polarity inversion) | Never swap RefClk P/N |

### AC Coupling (host TX only)

- **Value:** 220 nF, 0201 (or 0402), X7R/X5R. The PCIe Gen2 spec allows 75–265 nF.
- **Placement:** symmetric placement within ~10 mm of the DF40 connector, with both caps side by side.
- **Pad voiding:** void the L2 GND under the cap pads, sized to the pad footprint plus ~0.1 mm. This reduces the capacitive impedance dip at the pads.
- **Check the module first:** if the Radxa module already places TX caps, DNP these and fit 0 Ω links. Better still, route straight through.

### Connector Breakouts

- **DF40 pins:** neck down only inside the pin field, with breakout length ≤ 3 mm.
- **M.2 Key-M:** enter the pads straight, with no stubs.
- **Pad voiding:** void L2 under the M.2 signal pads to offset pad capacitance (~0.2 mm larger than the pads).

---

## 4. High-Speed Return Path and Via Stitching

### 4.1 Reference Plane Integrity
- L2 GND must be **continuous** beneath every PCIe pair from the DF40 to the M.2 socket. Don't allow splits, slots, or via-antipad chains that merge into a slot.
- Check for antipad "Swiss-cheesing" under the DF40 field. Clearance shapes must not merge across the pair's path.

### 4.2 Signal-Via Return Vias (only if a layer change is unavoidable)
- Place **2 GND vias per differential via pair**, one on each side, symmetric to the pair axis.
- Keep their center-to-center distance from the signal vias at **≤ 0.8 mm** (≤ 1 mm max).
- Size the antipads at ~0.25 mm larger radius than the pad. Make the pair a single oblong ("stadium") antipad and avoid separate overlapping circles.
- On a 4-layer board, a transition L1 → L4 references L2 GND then L3 PWR. Prefer to avoid this transition entirely. If it's unavoidable, put a **100 nF stitching cap (3V3 ↔ GND)** within 2 mm of the transition. Keep it as close as the GND return vias if possible.
- Through-hole via stubs at 1.6 mm are tolerable at Gen2, so back-drilling is not needed.

### 4.3 Route-Edge Via Fence (top-layer GND pour)
If L1 has GND copper pour next to the pairs:
- **Clearance from pour to pair:** ≥ 3× trace width (~0.45 mm). Otherwise the impedance shifts to coplanar values.
- **Stitching pitch:** **≤ 2.5 mm** along both sides of the routing channel. This comes from λ/20 at 2.5 GHz in FR4 (λ ≈ 60 mm, so ≈ 3 mm), rounded down for 3rd-harmonic margin.
- **No floating copper:** remove any pour slivers or islands not tied by at least 2 vias. They resonate.

### 4.4 Connector Ground Stitching
- **DF40 (module side):** every GND pin in and adjacent to the PCIe pin group gets its **own via to L2** (≤ 0.5 mm from pad). Don't share one via between two pins.
- **M.2 socket:** each GND pad flanking PETp0/n and PERp0/n gets an individual via to L2. Also give the socket's mechanical/shell tabs ≥ 2 vias each.
- **Board-edge stitching:** use a GND via row at ≤ 5 mm pitch around the board perimeter, tying L1/L2/L4 pours to reduce edge radiation.

### 4.5 Power Delivery (affects return currents)
- **3V3_M2:** use a dedicated L3 region. Place a 22 µF + 10 µF bulk cluster plus 4 × 100 nF at the M.2 power pins.
- **Load switch:** add one (e.g., TPS22918-class) with soft start, and gate it with module GPIO if hot-power control is needed.
- **Regulator placement:** keep any buck regulator's switch node and inductor **≥ 10 mm** from the PCIe channel and the M.2 socket, on the opposite side of L2 if possible.

---

## 5. Reference Clock and Sideband

- **RefClk:** route as an 85–100 Ω pair with the same rules as the data pairs, plus a GND fence.
  - Termination is not needed on the carrier, since the module source and the card handle HCSL termination. Confirm against the module datasheet.
  - Keep the RefClk ≥ 1 mm from the data pairs.
- **PERST#:** driven by the module. Add a 10 k pull-down or pull-up per the module requirement (usually active-low with a pull-up). Add a 100 pF filter at the socket if the run is long.
- **CLKREQ#:** 10 k pull-up to 3.3 V. If the module doesn't support clock power management, tie it so the clock is always on.
- **PEDET (M.2 pin 69):** leave unconnected or pull up per NVMe vs SATA detection needs.
- **SUSCLK:** optional 32.768 kHz. Leave it NC if unused.

---

## 6. Verification Checklist

1. ☐ Fab-confirmed stackup and impedance table (85 Ω diff on L1). Order an impedance coupon and TDR report.
2. ☐ Intra-pair skew ≤ 0.13 mm, including module-side length if Radxa publishes it.
3. ☐ Zero layer changes, or each change has 2 GND return vias ≤ 0.8 mm away (plus a stitching cap if the reference changes).
4. ☐ Continuous L2 under all pairs. Run the DRC "plane split crossing" check.
5. ☐ AC caps only on host TX, symmetric, with voided reference under the pads.
6. ☐ RefClk polarity correct; data polarity swap noted, if used.
7. ☐ GND via per connector GND pin; fence pitch ≤ 2.5 mm; no floating copper.
8. ☐ 3V3_M2 sized for 3 A, with bulk plus local decoupling at the socket.
9. ☐ Optional: extract S-parameters (e.g., OpenEMS or a SiWave-type tool). Check insertion loss **> −3 dB at 2.5 GHz** for the carrier segment, and return loss **< −10 dB up to 2.5 GHz**.
10. ☐ Bring-up checks:
    - `lspci -vv` shows **LnkSta: Speed 5GT/s, Width x1**.
    - Run `fio` sequential reads (expect ~350–400 MB/s ceiling for Gen2 x1).
    - Check AER error counters stay at zero under load.

---

If you share your actual fab stackup, board outline, or the Cycle 22 issues you're addressing (e.g., link training at Gen1 only, AER errors), I can tighten the trace geometry and pinpoint the likely return-path culprits.