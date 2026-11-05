# M.2 NVMe (PCIe Gen2 x1) Routing and Return-Path Design: Radxa CM4 Expansion Board, Cycle 17

**Scope.** This covers the PCIe link from the CM4 board-to-board connectors to an M.2 M-key socket (2230/2242/2280).

**Assumptions to verify before layout.** Confirm the following against the current Radxa CM4 datasheet and reference carrier schematic:
- **Link width and speed:** PCIe Gen2 x1, 5 GT/s, 8b/10b encoding.
- **Pinout:** connector pin assignments for the PCIe pairs.
- **AC-coupling caps:** whether the host TX caps are already fitted on the module.
- **Polarity inversion:** whether lane polarity inversion is supported.

---

## 1. Link Budget and Electrical Targets

| Parameter | Target | Notes |
|---|---|---|
| Data rate / Nyquist | 5 GT/s / 2.5 GHz | Design the channel to roughly 5 GHz (2× Nyquist) |
| Differential impedance (data) | **85 Ω ±10%** | PCIe CEM/M.2 recommendation. 90–100 Ω is tolerable at Gen2 if the fab can't hit 85 Ω |
| Differential impedance (REFCLK) | 85–100 Ω ±10% | HCSL |
| Single-ended (reference) | ~45–50 Ω | |
| Carrier insertion-loss budget | ≤ 4 dB @ 2.5 GHz | Leaves margin for the module and SSD |
| Max trace length (carrier) | ≤ 100 mm recommended, ≤ 150 mm absolute | Keep short. Place the M.2 socket close to the CM4 connector |
| Return loss | ≤ −10 dB to 2.5 GHz | Mostly set by vias and the connector |

Standard FR-4 loses roughly 0.45–0.6 dB/inch at 2.5 GHz for 4–5 mil traces. A 100 mm route therefore costs about 2–2.4 dB plus about 0.3 dB of vias, which is comfortable.

---

## 2. Stackup

**Recommended: 6-layer, 1.6 mm.** This gives a solid ground under both outer layers and room for power planes.

| Layer | Use |
|---|---|
| L1 | Signal: PCIe pairs, components |
| L2 | **GND (solid, unbroken under PCIe)** |
| L3 | Signal / low-speed |
| L4 | Power (3V3, 5V) |
| L5 | **GND** |
| L6 | Signal / components |

**Budget option: 4-layer (SIG / GND / PWR / SIG).**
- Route all PCIe on L1 over the L2 ground.
- Do not route PCIe on L4 over the power plane unless you stitch capacitors at every reference change. Avoid that entirely if possible.

**Starting geometries.** Use these as inputs to your fab's impedance calculator, not as final values.

| Stackup (L1→L2 dielectric) | Diff target | Width / Gap (approx.) |
|---|---|---|
| ~0.10 mm prepreg (e.g. 3313/2116 class, εr ≈ 4.1) | 85 Ω | 0.13 mm / 0.15 mm |
| ~0.10 mm prepreg | 100 Ω | 0.10 mm / 0.18 mm |
| ~0.21 mm prepreg (7628 class, εr ≈ 4.4) | 85 Ω | ~0.25 mm / 0.20 mm |
| ~0.21 mm prepreg | 100 Ω | ~0.18 mm / 0.20 mm |

**Action:**
- Order with **controlled impedance specified on the fab notes**: "L1 diff pairs 85 Ω ±10% ref L2".
- Use the fab's exact widths and record them in the netclass.

---

## 3. Netclass: `PCIE_GEN2`

| Rule | Value |
|---|---|
| Nets | PCIE_TX_P/N (host→SSD), PCIE_RX_P/N (SSD→host), PCIE_REFCLK_P/N |
| Intra-pair skew (P vs N) | **≤ 0.13 mm (5 mil)**, matched near the mismatch source |
| Inter-pair skew (TX vs RX) | No requirement (independent lanes) |
| REFCLK to data length match | Not required (common-clock architecture) |
| Pair-to-pair spacing | ≥ 3× trace width (prefer ≥ 0.5 mm, or 5H) |
| Spacing to other signals | ≥ 0.5 mm; ≥ 1.0 mm to clocks, SMPS switch nodes, USB3/HDMI |
| Spacing to plane/board edge | ≥ 1.0 mm from the board edge; reference plane must extend ≥ 3W beyond the pair |
| Max vias per lane | **2** (ideally 0 on the 6-layer board: route on L1 end to end) |
| Bend style | 45° or arcs; no 90° corners |
| Serpentine (skew tune) | Amplitude ≤ 2× gap, segment length ≥ 3× width, placed at the end where the skew originates |
| Pad/stub | No stubs, no test points on data lines (use AC-cap pads if a probe point is needed) |
| Fiber weave | Rotate the route about 10° or use spread glass (1067/1078/1086) for long straight runs |

### AC-Coupling Capacitors
- **Host TX → M.2 RX (PETp0/PETn0 at the socket): fit 100 nF (75–265 nF allowed), 0201 or 0402, X7R.** Omit these if the CM4 module already has them; verify this.
- **SSD TX → host RX: no caps on the carrier.** The M.2 card carries its own TX caps.
- Placement rules:
  - Place the P/N caps symmetrically, side by side.
  - Place them close to the M.2 socket or the CM4 connector, never mid-route with stubs.
  - **Void L2 ground under the cap pads** (pad size plus about 0.1 mm) to cancel the pad's excess capacitance and hold impedance, but only if L3 is GND or a quiet plane on the 6-layer board.

### Polarity
- Swapping P/N within a pair is allowed if the module PHY supports polarity inversion; Gen2 links negotiate it automatically.
- Prefer a straight-through route anyway.
- **Never swap TX and RX.**

---

## 4. Return Path and Via Stitching

At 2.5 GHz the return current flows directly under the pair on L2. Every discontinuity in that path is the main risk, more than loss.

### 4.1 Signal-via transitions (if a layer change is unavoidable)
- **Place two GND stitching vias per differential via pair**, one on each side, symmetrically. Keep the center-to-center distance from signal via to GND via **≤ 1.0 mm (target 0.7–0.8 mm)**.
- When switching from the L2 reference to the L5 reference, the GND vias tie L2↔L5 so the return current has a path.
- If the reference changes between GND and PWR (4-layer board), add a **10 nF 0402 stitching cap** from PWR to GND within 1 mm of the via pair. This is a fallback only.
- **Signal via:** 0.25–0.3 mm drill, 0.45–0.5 mm pad.
- **Anti-pad:** oval/oblong around both vias together, about 0.9–1.0 mm × 1.6–1.8 mm, on all planes.
- **Stubs:** route L1→L6, or L1→L3 with back-drilling. At Gen2 a 1.6 mm stub is tolerable, but a full-through via used L1→L6 has no stub, so prefer it.

### 4.2 Connector transitions
**CM4 board-to-board connector (Hirose DF40-class):**
- Use the connector's GND pins that flank the PCIe pins. Drop a GND via at **each GND pad within 0.5 mm**, with no shared vias.
- Fan out the pair directly on L1 with no neck-down longer than 1 mm.

**M.2 socket:**
- Tie every GND pin adjacent to PET/PER/REFCLK straight to L2 with its own via.
- Void L2 under the PCIe signal pads of the M.2 edge-connector footprint (pad + 0.1 mm) only if the fab impedance model shows a dip greater than 10%. Otherwise keep it solid.

### 4.3 Along-route stitching (coplanar ground / fences)
- If L1 has ground pour, keep it **≥ 3W (≥ 0.5 mm) from the pair**. Otherwise the pour shifts impedance; remove the pour near the pairs if you can't hold that clearance.
- **Stitch the L1 pour to L2 every ≤ 3 mm (λ/20 at 5 GHz ≈ 3 mm in FR-4)** along the PCIe corridor, in rows on both sides of the routing channel.
- Trim isolated copper islands near the pairs. Floating copper resonates.

### 4.4 Plane integrity checks
- **No splits, slots, or antipad chains under the pairs on L2.** Watch for via fields from the CM4 connector that merge antipads into a slot. Ensure web copper ≥ 0.15 mm between antipads.
- No PCIe pair may cross a plane edge. Keep the plane edge ≥ 1 mm beyond the pair.

---

## 5. REFCLK and Sideband Signals

| Signal | Rule |
|---|---|
| REFCLK± (100 MHz HCSL) | Diff pair, same impedance class, intra-pair ≤ 0.13 mm. No AC caps. Termination per the module datasheet (usually internal; don't add 49.9 Ω unless specified). Keep ≥ 1 mm from data pairs |
| PERST# | Open-drain/push-pull from the module. 10 kΩ pull-up to 3V3 if the module requires it. Optional 100 nF filter not recommended (it delays reset timing) |
| CLKREQ# | Open-drain, 10 kΩ pull-up to 3V3. If the module lacks CLKREQ#, tie low through 0 Ω (DNP option) |
| PEWAKE# | Optional, 10 kΩ pull-up |
| SUSCLK (32.768 kHz) | Optional; most SSDs don't require it |
| DAS/DSS LED | Optional activity LED via a FET |
| M.2 pin 21/... config | Leave per M-key spec; PEDET (pin 69) NC or GND for NVMe |

---

## 6. M.2 Power

**3V3_M2 rail:**
- Use a dedicated buck or load switch rated **≥ 3 A** (SSD peaks reach about 2.5 A).
- Bulk capacitance at the socket: **2 × 22 µF + 4 × 100 nF**, placed at the 3V3 pins.
- Use a wide pour (≥ 2 mm) or a plane on L4 with **≥ 6 vias** at the socket.

**Separation from PCIe:**
- Keep the SMPS inductor and switch node ≥ 5 mm from PCIe pairs and REFCLK.
- Do not route anything over the switch node on L2. Keep L2 solid ground there.

**Mounting:** M.2 standoff screws to GND for chassis/EMI return.

---

## 7. Cycle 17 Verification Checklist

1. ☐ Fab stackup confirmed; netclass widths/gaps updated to the fab calculator (85 Ω).
2. ☐ All PCIe on L1 over solid L2; via count per lane ≤ 2 (target 0).
3. ☐ Intra-pair skew ≤ 0.13 mm on TX, RX, and REFCLK (DRC report attached).
4. ☐ Every signal-via pair has 2 GND vias ≤ 1.0 mm away.
5. ☐ Corridor ground stitching pitch ≤ 3 mm; no floating copper.
6. ☐ L2 plane check: no slots or merged antipads under the pairs (inspect the plane layer alone).
7. ☐ Host-TX AC caps present (or confirmed on-module), symmetric, with plane void under pads (if applicable).
8. ☐ TX/RX direction verified: module PCIE_TX → M.2 PERp0/PERn0 (pins 41/43) is the host-to-device path. Note that M.2 naming is device-relative; triple-check against the socket pinout.
9. ☐ REFCLK routed; CLKREQ#/PERST# pull-ups placed.
10. ☐ 3V3_M2 rail ≥ 3 A, bulk caps at the socket, SMPS kept away from the corridor.
11. ☐ Optional: 2D field-solver/TDR coupon on the panel for impedance verification.
12. ☐ Bring-up: `lspci -vv` shows `LnkSta: Speed 5GT/s, Width x1`; run `fio` for 10+ min and check `AER` counters (`dmesg | grep -i aer`) for correctable errors.

**Note on item 8.** M.2 signal names are from the SSD's perspective (PETx = SSD transmit). The host TX must connect to the SSD **PER** pins, and the host RX to **PET**. This is the most common NVMe carrier-board error.