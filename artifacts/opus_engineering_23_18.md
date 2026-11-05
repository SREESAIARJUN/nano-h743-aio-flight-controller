# Pan-Axis Yoke Geometry (Cycle 23): Hollow-Shaft BLDC Seats and 6.5 mm Slip-Ring Channel

I don't have the earlier cycles' locked dimensions, so every vendor-dependent value is a named parameter flagged `VERIFY`. Swap in your datasheet numbers and the `assert()` checks will flag any interference.

---

## 1. Architecture

```
              ┌──── tilt motor seat         idler bearing seat ────┐
              │  (stator BC + wire bore)    (6801, crush ribs)      │
              │                                                     │
         [L ARM]                                               [R ARM]
              └──────────────── CROSSBEAM (wire channel) ──────────┘
                                   │ HUB (split clamp)
              ═══════════ TOWER: 2× 6801 back-to-back, central outer shoulder
                                   │ SPINDLE Ø12 / bore Ø8 (inner-race spacer = preload)
                          ┌────────┴────────┐  spindle flange → motor ROTOR face
                          │  2804H HOLLOW   │  stator → BASE
                          └────────┬────────┘
      BASE PLATE ══════════════════╧══════ slip-ring flange pocket (underside)
                              6.5 mm capsule runs up through motor bore
```

**Load path:**
- The motor bearings carry torque only.
- Tilt and moment loads go through the two 6801 bearings, into the tower, then into the base.
- The slip-ring stator flange is keyed to the base. Its rotor leads exit up through the spindle bore into the yoke.

---

## 2. OpenSCAD Source (`pan_yoke_c23.scad`)

```openscad
// =====================================================================
//  Pan-Axis Yoke Assembly — Cycle 23
//  Units: mm. Z-up. Pan axis = Z. Tilt axis = X at arm height.
// =====================================================================

/* [Render] */
part    = "assembly"; // [assembly, base, tower, spindle, spacer, yoke, section]
explode = 0;          // [0:1:40]
$fn     = 96;

/* [Fit / Printer Tuning] */
press_int   = 0.10;  // diametral interference at crush-rib tips (FDM: 0.08–0.15)
seat_relief = 0.40;  // diametral oversize of bore between ribs
slip_clr    = 0.30;  // diametral clearance around slip-ring body
shaft_clr   = 0.05;  // spindle→bearing ID (light press; ream if printed)
hole_clr    = 0.25;  // screw clearance
chamfer     = 0.6;   // lead-in chamfer on all seats

/* [Bearing 6801 — 12x21x5] */
brg_id = 12; brg_od = 21; brg_w = 5;
brg_shoulder_id = 17;   // outer-race abutment (> inner race OD ~14.6)
brg_mid_t       = 3;    // housing shoulder between the two bearings

/* [Pan Motor — 2804H-class hollow shaft]  VERIFY */
mot_d       = 34;
mot_h       = 16;
mot_bore    = 7.5;  // hollow shaft through-hole
stator_bc   = 19;   stator_m = 2.0;  stator_n = 4;
rotor_bc    = 16;   rotor_m  = 2.0;  rotor_n  = 4;

/* [Tilt Motor (on left arm)]  VERIFY */
tilt_stator_bc = 19; tilt_m = 2.0;