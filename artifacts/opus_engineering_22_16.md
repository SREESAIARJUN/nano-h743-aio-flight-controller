# Pan-Axis Yoke, Hollow-Shaft BLDC Bearing Seats, and 6.5 mm Slip-Ring Channel: Parametric Geometry (Cycle 22)

I don't have the earlier cycles' parameter sheet. Every value below is exposed as a top-level parameter so you can paste in your locked numbers. Defaults assume a **GBM4108H-class hollow-shaft gimbal motor**, a **61808 thin-section support bearing**, a **6.5 mm OD capsule slip ring**, and **688 tilt-axis bearings**. Verify all motor and slip-ring dimensions against your actual datasheets before cutting metal.

---

## 1. Architecture

```
            ┌───────── tilt bearing seats (688) ─────────┐
            │                                              │
            │  YOKE ARM                          YOKE ARM  │
            │      └──────── yoke hub plate ───────┘       │
            │             │  (bolted to rotor)   │         │
            │   skirt ─►  ║  ┌──────────────┐    ║  ◄─ skirt rides 61808 inner race
            │  61808 ═════╬══│  BLDC rotor  │════╬═════ outer race in base cup (floating)
            │   BASE CUP  ║  │  stator      │    ║
            │             ╚══╧══════╤═══════╧════╝  ◄─ stator bolted to base floor
            │                drive tube │ (bonded in hollow shaft)
            │                 ┌─────────┴──┐
            │                 │ 6.5 slip   │  ◄─ channel tower, clamped stator body
            │                 │ ring       │
            │                 └─────┬──────┘
            │                 radial wire exit slot
```

Key decisions:

- **The support bearing takes the yoke's overturning moment.** Without it, the moment would load the motor's tiny internal bearings. The bearing's outer race is **axially floating**: slip fit plus an O-ring groove. This stops it fighting the motor bearings, which avoids a doubly-constrained axis.
- **The slip ring is stacked below the motor, not nested inside the shaft.** Most hollow gimbal-motor bores are 5.5–8 mm, and nesting a 6.5 mm capsule leaves no wall for a drive coupling.
- **A drive tube transmits rotation to the slip-ring rotor.** It is bonded inside the hollow shaft and grips the rotor stub. Never rely on wire-bundle twist to drive the rotor.

---

## 2. OpenSCAD source

```openscad
// ============================================================
//  Pan-Axis Yoke + BLDC Bearing Seats + 6.5mm Slip-Ring Channel
//  Cycle 22 | units: mm | Z-up, pan axis = Z
// ============================================================

/* [Render] */
part    = "assembly";   // [assembly, base, yoke, drive_tube, section]
explode = 0;            // [0:1:40]
$fn     = 128;

/* [Process / Fits] */
process   = "print";    // [print, cnc]
// Printed holes come out undersize; compensation added to all bores
hole_comp = process == "print" ? 0.15 : 0.0;
// Housing bore for floating outer race (H7-ish slip on CNC, snug on print)
fit_housing = process == "print" ?  0.10 : 0.010;   // added to D
// Shaft/skirt OD on rotating inner race (k6-ish interference on CNC)
fit_shaft   = process == "print" ?  0.00 : 0.010;   // added to d
// Tilt bearing seats (outer race, light press)
fit_tilt    = process == "print" ?  0.05 : -0.005;

/* [Motor - VERIFY AGAINST DATASHEET] */
motor_od        = 46.0;  // rotor can OD
motor_h         = 25.0;  // stator base to rotor top face
motor_bore      = 8.0;   // hollow shaft ID (wire passage)
stator_bc       = 25.0;  // stator mount bolt circle
stator_n        = 4;
stator_screw    = 3.0;   // M3
rotor_bc        = 30.0;  // rotor mount bolt circle
rotor_n         = 4;
rotor_screw     = 3.0;
stator_pilot_d  = 12.0;  // centering pilot boss on stator underside
rotor_clear     = 1.0;   // radial clearance rotor can -> skirt

/* [Support bearing (thin section)] */
sb_d = 50;   // 61810: 50x65x7 (default chosen to clear motor_od=46)
sb_D = 65;
sb_B = 7;
sb_shoulder_h = 1.5;     // abutment shoulder radial height (ISO 'da' min)
oring_cs      = 1.0;     // O-ring cross-section for floating outer race

/* [Slip ring - 6.5mm capsule, VERIFY] */
sr_d        = 6.5;
sr_body_l   = 17.0;
sr_rotor_d  = 3.0;       // rotating stub / wire-exit OD on rotor end
sr_clear    = 0.10;      // diametral clearance in channel
sr_clamp_screw = 2.0;    // M2 radial clamp (use nylon-tip / pad!)
sr_exit_w   = 4.0;       // stator wire exit slot width

/* [Drive tube] */
dt_od = motor_bore - 0.2;  // bonded (Loctite 638) in hollow shaft
dt_id = sr_rotor_d + 0.1;  // grips slip-ring rotor stub
dt_engage = 6.0;           // engagement on sr rotor stub

/* [Base] */
base_floor = 5.0;
base_wall  = 3.5;
base_flange_w = 10;      // mounting flange beyond cup
base_mount_n  = 4;
base_mount_screw = 4.0;
tower_wall = 2.5;

/* [Yoke] */
hub_t      = 5.0;        // hub plate thickness
skirt_wall_min = 1.6;
arm_t      = 6.0;
arm_w      = 22.0;
arm_inner_span = 80.0;   // inner face-to-face distance (payload width)
tilt_axis_h    = 70.0;   // tilt axis above hub plate top
tilt_d = 16; tilt_B = 5; tilt_bore_through = 10; // 688 seat
fillet_r = 4.0;

// ---------------- derived ----------------
z_stator  = base_floor;
z_rotor   = base_floor + motor_h;      // yoke hub underside
skirt_id  = motor_od + 2*rotor_clear;
skirt_od  = sb_d + fit_shaft;
skirt_len = sb_B + 0.5;
z_brg_top = z_rotor - 0.3;             // 0.3 running gap under hub plate
z_brg_bot = z_brg_top - sb_B;
cup_od    = sb_D + 2*base_wall;
cup_h     = z_brg_top;
tower_id  = sr_d + sr_clear + hole_comp;
tower_od  = tower_id + 2*tower_wall;
tower_h   = sr_body_l + 3;             // below base floor

// ---------------- sanity checks ----------------
assert((skirt_od - skirt_id)/2 >= skirt_wall_min,
  str("Skirt wall too thin: ", (skirt_od-skirt_id)/2,
      " mm -> choose larger support bearing"));
assert(dt_id < dt_od - 0.8, "Drive tube wall < 