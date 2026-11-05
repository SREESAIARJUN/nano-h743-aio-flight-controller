# Pan-Axis Yoke, Bearing Seats and 6.5 mm Slip-Ring Channel: Parametric Geometry

I don't have your earlier cycles, so I can't see your prior motor choice, frame interfaces or fit conventions. Every assumption below is a single named parameter. Override them to match your Cycle 30 baseline. Values marked `VERIFY` came from typical datasheets and must be measured on your actual parts.

---

## 1. Architecture

```
            tilt bearing bores (×2)
        ┌──┐                    ┌──┐
        │  │      YOKE ARMS     │  │ ← wire groove on inner face
        │  │                    │  │
        └──┴────────┬──┬────────┴──┘ ← yoke hub (slip-ring pocket + key + wire exit)
                    │SR│ ← 6.5 mm slip-ring capsule (rotor side turns with yoke)
     ┌──────────────┤  ├──────────────┐ ← base top plate
     │   [6700]═════╡  ╞═════[6700]   │ ← upper bearing (inner race clamped by yoke collar)
     │          lip │  │ lip          │ ← outer-race spacer lip
     │   [6700]═════╡  ╞═════[6700]   │ ← lower bearing
     │         ┌────┴──┴────┐         │ ← rotor coupler (split clamp on spigot, bolts to rotor)
     │         │ BLDC ROTOR │         │
     │         │  (hollow)  │ ← slip-ring stator leads pass down through motor bore
     └─────────┴────────────┴─────────┘ ← motor mount plate (stator fixed)
```

**Load path.** The two 6700 bearings (10×15×4 mm) react all yoke loads: radial, axial and tilting moment. The motor's own tiny bearings only see torque. The bearing span is set by the outer-race lip, which gives a stiff back-to-back-style stance.

**Preload.** Each bearing's inner race is clamped between two parts:
- the upper race by the yoke collar,
- the lower race by the rotor coupler.

To set preload, push the coupler up with a wave washer or 0.05–0.1 mm shim, then lock it with the pinch screw.

**Slip ring.** The capsule body is keyed to the yoke hub and rotates with it. The stator-side lead bundle drops through the hollow spigot and the motor bore into the base.

---

## 2. Axial Stack (z = 0 at lower bearing face)

| Element | z from | z to | Notes |
|---|---|---|---|
| Lower 6700 | 0.0 | 4.0 | outer race seated against lip |
| Spacer lip | 4.0 | 5.0 | ID 12.6 clears rotating inner races |
| Upper 6700 | 5.0 | 9.0 | |
| Yoke collar | 9.0 | 9.5 | Ø11.6, contacts inner race only |
| Yoke hub | 9.5 | 9.5 + hub_h | 0.5 mm running gap to boss top |
| Coupler | −4.0 | 0.0 | relief collar contacts lower inner race only |
| Motor rotor face | −4.5 | | 0.5 mm gap |

---

## 3. OpenSCAD Source

```openscad
// =====================================================================
//  PAN-AXIS YOKE / BEARING SEATS / 6.5 mm SLIP-RING CHANNEL — Cycle 31
//  Units: mm.  Select part with PART.  All fits driven by PROCESS.
// =====================================================================
PART    = "assembly";   // "assembly","yoke","base","coupler","mount_plate"
PROCESS = "FDM";        // "FDM" (crush ribs) | "CNC" (H7/k6-style)
$fn = 96;
eps = 0.01;

// ---------------- Bearings (6700 = 10x15x4) --------------------------
brg_id = 10;  brg_od = 15;  brg_w = 4;
brg_ir_shoulder = 11.6;   // max inner-race land dia (contact only here)
brg_or_shoulder = 13.4;   // min outer-race land dia
lip_t   = 1.0;            // outer-race spacer lip
lip_id  = 12.6;           // must clear rotating inner race

// ---------------- Fit tables ----------------------------------------
seat_clr  = (PROCESS=="FDM") ? 0.15 : 0.000; // housing bore over nominal
rib_int   = 0.20;   // FDM crush-rib interference (radial)
rib_r     = 0.60;
rib_n     = 6;
shaft_fit = (PROCESS=="FDM") ? -0.05 : 0.0;  // spigot OD vs brg_id (- = slip)

// ---------------- Slip ring (6.5 mm capsule) — VERIFY ---------------
sr_d        = 6.5;
sr_len      = 17.5;   // capsule body length
sr_clr      = (PROCESS=="FDM") ? 0.25 : 0.10;
sr_key_w    = 1.6;    // anti-rotation tab (or wire-exit notch) width
sr_key_d    = 1.2;    // tab radial depth
sr_lead_d   = 3.0;    // stator-side wire bundle
sr_rot_exit = 3.0;    // rotor-side wire exit slot width

// ---------------- Spigot (hollow shaft of yoke) ---------------------
spg_od   = brg_id + shaft_fit;
spg_bore = sr_d + sr_clr;            // 6.75 FDM -> 1.6 mm wall
assert((spg_od - spg_bore)/2 >= 1.2, "Spigot wall < 1.2 mm: use Al tube insert");

// ---------------- Motor (GBM2804H-class hollow) — VERIFY ------------
mot_bell_d   = 33;
mot_h        = 14;
mot_bore     = 6.0;   // hollow bore; must pass sr_lead_d
rot_pcd = 12;  rot_hole = 2.2;  rot_n = 4;
sta_pcd = 19;  sta_hole = 2.7;  sta_n = 4;
assert(mot_b