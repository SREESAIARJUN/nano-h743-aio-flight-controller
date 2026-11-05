# Pan-Axis Yoke: Parametric Geometry (Cycle 13)

I don't have the documents from your earlier cycles, so the motor, slip-ring, and bearing values below are explicit assumptions. Every one is a top-level parameter. Replace them with your datasheet values before printing or machining.

---

## 1. Architecture

A 6.5 mm slip ring will not pass through the rotating hollow shaft of a typical 28xx gimbal motor (shaft bore is usually about 5–5.5 mm). The design therefore uses an externally borne rotor:

```
              ┌───── tilt-motor bore (arm tops) ─────┐
              │                                       │
         ARM ─┤        YOKE (rotates)                 ├─ ARM  ← wire channel up one arm
              │   ┌─────────────────────────┐         │
              └───┤  hub: 2× 6700ZZ (DB)    ├─────────┘
                  │  ↑ preload clamp ring   │
                  │  ┌── slip-ring rotor ↑  │
     motor rotor ─┤  │ STANDPIPE (fixed)    │ ← bolted to yoke underside
     motor stator ┤  │  Ø6.5 slip-ring bore │ ← bolted to base
                  └──┴──────────────────────┘
                     BASE (fixed) — stator-side leads exit below
```

- **Standpipe (stationary):** part of the base. It carries the bearing inner races on its OD and the slip-ring capsule in its bore.
- **Yoke hub (rotating):** carries the bearing outer races. The motor rotor bolts to its underside.
- **Motor:** the stock shaft and circlip are removed. The rotor rides on the 6700 pair, which sets the air gap. You need a motor whose stator ID is at least the standpipe OD, or one you can de-shaft. Many 28xx/35xx gimbal motors allow this; check yours.
- **Slip ring:** the stator side stays in the standpipe. The rotor-side leads rise into the yoke hub and route up one arm to the tilt motor.

**Bearing choice: 6700ZZ (10×15×4).** Its 10 mm bore leaves room for a 6.5 mm channel plus a wall. Two are mounted back-to-back (DB) for moment stiffness against the cantilevered arms.

---

## 2. OpenSCAD Source

```openscad
// =====================================================================
//  PAN-AXIS YOKE + BEARING SEATS + 6.5mm SLIP-RING CHANNEL  (Cycle 13)
//  Units: mm.  Set PROCESS and verify all [VERIFY] params vs datasheets.
// =====================================================================

/* [Render] */
PART    = "assembly";   // [assembly, base, yoke, clamp_ring, spacer]
EXPLODE = 12;           // assembly explode distance
$fn     = 96;

/* [Process / Fits] */
PROCESS = "FDM";        // [FDM, CNC]
// press = interference (+ means bore smaller than part)
fit_press_bore  = (PROCESS=="CNC") ? -0.008 : 0.10; // bearing OD seat (bore)
fit_press_shaft = (PROCESS=="CNC") ?  0.005 : -0.06; // bearing ID journal (shaft)
fit_slide       = (PROCESS=="CNC") ?  0.02  : 0.25;  // sliding clearance (diam.)
fit_hole        = (PROCESS=="CNC") ?  0.10  : 0.30;  // clearance holes

/* [Bearing 6700ZZ] */
brg_id = 10;  brg_od = 15;  brg_w = 4;
brg_n  = 2;               // DB pair
brg_gap = 3;              // spacer between outer races (sets moment span)
brg_shoulder = 1.0;       // radial shoulder height (<= race land, keep off seals)

/* [Slip ring  6.5mm capsule]  [VERIFY] */
sr_d      = 6.5;
sr_len    = 17.0;         // body length
sr_lead_d = 3.2;          // bundled lead diameter (6-wire)
sr_rotor_protrude = 3.0;  // rotor stub above body

/* [Motor - de-shafted gimbal motor]  [VERIFY] */
mot_od         = 34.0;
mot_h          = 13.0;
mot_stator_id  = 10.5;    // must exceed standpipe OD + runout margin
stator_pcd = 12.0;  stator_screw = 2.0;  stator_n = 4;
rotor_pcd  = 19.0;  rotor_screw  = 2.0;  rotor_n  = 4;
air_gap_runout_budget = 0.05;  // TIR allowed at magnet gap

/* [Base] */
base_d = 46;  base_t = 4;
base_mount_pcd = 38; base_mount_screw = 3.0;
standpipe_wall_min = 1.4;

/* [Yoke] */
hub_wall    = 3.0;
hub_floor_t = 3.0;
arm_span    = 60;    // inside distance between arms
arm_t       = 5;
arm_w       = 16;
arm_h       = 55;    // hub-floor to tilt-axis
tilt_bore   = 22;    // tilt motor spigot  [VERIFY]
tilt_pcd    = 19; tilt_screw = 2.0;
wire_ch_w   = 4.0;  wire_ch_d = 3.0;   // routed in -X arm
gusset      = 10;

/* [Preload clamp] */
clamp_t = 2.5;  clamp_screw = 2.0;  clamp_n = 3;
preload_shim = 0.05;   // axial crush for DB preload (CNC); FDM: use wave washer

// ---------------------------------------------------------------- derived
sp_od   = brg_id + fit_press_shaft;          // standpipe journal Ø
sp_bore = sr_d + fit_slide;                   // slip-ring channel Ø
sp_wall = (sp_od - sp_bore)/2;
assert(sp_wall >= standpipe_wall_min,
       str("Standpipe wall ", sp_wall, " < min. Use CNC/Al or larger bearing."));
assert(mot_stator_id - sp_od >= 2*air_gap_runout_budget,
       "Stator ID too tight around standpipe.");

brg_stack = brg_n*brg_w + brg_gap;
hub_bore  = brg_od - fit_press_bore;
hub_od    = brg_od + 2*hub_wall;
hub_h     = brg_stack + hub_floor_t;
sp_len    = base_t + mot_h + brg_stack + 1.0;   // journal through stack
yoke_z0   = base_t + mot_h + 