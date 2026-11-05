# Pan-Axis Yoke: Parametric Geometry (Cycle 15)

I don't have your prior cycle files, so the code below uses explicit assumptions. Every one is a top-level parameter, and the hardware dimensions marked with ⚠ in Section 5 must be checked against your datasheets.

---

## 1. Architecture and Load Path

```
            ┌──── tilt bearing seat ────┐
            │                           │
          ARM                          ARM        ← yoke arms (wire channel in +X arm)
            │      ┌─slip ring─┐        │
   ═════════╧══════╡ 6.5 mm    ╞════════╧═════   ← yoke plate (bolted to motor rotor)
   ║ skirt  ┌──────┴───┬───────┴──────┐  skirt ║
   ║[OUTER] │  motor   │ hollow shaft │ [OUTER]║  ← thin-section bearing (618xx)
   ║[INNER] │  rotor   │ (stationary) │ [INNER]║     inner race on base boss
   ╚════════╧══════════╧══════════════╧════════╝  ← base housing (stator mount)
```

- **The motor's own bearings only transmit torque.** The overturning moment from the offset tilt payload goes through a large thin-section bearing (61808 or 61809) that surrounds the motor.
- **Bearing races:**
  - The inner race is stationary, seated on the base boss.
  - The outer race rotates in the yoke skirt.
  - The skirt bore has a step that abuts only the outer race, so the bearing is captured axially when the plate is bolted down.
- **Slip ring placement:**
  - The slip-ring body rotates with the yoke and sits in a center channel above the rotor.
  - Its stationary-side wires drop through the motor's hollow shaft to the base.
  - An anti-rotation coupler keys the slip-ring core to the stationary shaft.

---

## 2. OpenSCAD Source: `pan_yoke_c15.scad`

```openscad
// =====================================================================
//  PAN-AXIS YOKE — Cycle 15
//  Parametric: base housing, thin-section bearing seats,
//  hollow-shaft BLDC interface, 6.5 mm slip-ring channel.
//  Units: mm.  +Z = pan axis.  Origin = base underside, axis centre.
// =====================================================================

/* [Render] */
part   = "assembly";   // [assembly, base, yoke, coupler, section]
fab    = "fdm";        // [fdm, cnc]   selects fit allowances
$fn    = 160;

/* [Motor — hollow-shaft gimbal BLDC (2804H class) ⚠ verify] */
mot_od        = 34.0;   // rotor bell OD
mot_h         = 15.5;   // stator mount face -> rotor top face
mot_stator_bc = 19.0;   // stator bolt circle
mot_stator_n  = 4;
mot_stator_m  = 2.0;    // M2
mot_rotor_bc  = 12.0;   // rotor bolt circle
mot_rotor_n   = 4;
mot_rotor_m   = 2.0;    // M2
mot_rotor_ang = 45;     // angular offset of rotor holes
mot_shaft_od  = 6.0;    // stationary hollow shaft OD (protrudes through cap)
mot_shaft_id  = 4.0;    // wire passage
mot_shaft_proud = 0.8;  // shaft protrusion above rotor face (0 if flush)
mot_bell_clr  = 0.6;    // radial clearance, rotor bell <-> boss bore

/* [Main bearing — thin section 618xx ⚠ verify] */
// 61808: 40x52x7 (CNC Al)   61809: 45x58x7 (recommended for FDM)
brg_id  = (fab=="cnc") ? 40 : 45;
brg_od  = (fab=="cnc") ? 52 : 58;
brg_w   = 7;
brg_shoulder_in  = 2.0; // inner-race abutment height (radial) on boss
brg_shoulder_out = 2.0; // outer-race abutment height (radial) in skirt

/* [Slip ring — 6.5 mm capsule ⚠ verify] */
sr_od        = 6.5;
sr_len       = 16.5;    // body length
sr_core_od   = 2.5;     // stationary-side rotor stub / wire bundle OD
sr_core_len  = 3.0;     // stub protrusion below body
sr_wire_bundle = 3.2;   // rotating-side wire bundle diameter
sr_grub_m    = 3.0;     // M3 radial set-screw (nylon-tipped!)

/* [Yoke] */
plate_t     = 5.0;
skirt_wall  = (fab=="cnc") ? 3.0 : 4.0;
axial_gap   = 0.8;      // boss top <-> plate underside (shim space)
arm_span    = 90.0;     // inner face to inner face
arm_t       = 8.0;      // arm thickness (X)
arm_w       = 22.0;     // arm width (Y)
arm_h       = 70.0;     // plate top -> tilt axis
tilt_brg_id = 15; tilt_brg_od = 24; tilt_brg_w = 5;   // 6802 ⚠
arm_wire_ch = 4.0;      // wire channel diameter inside +X arm
fillet_r    = 4.0;

/* [Base] */
floor_t     = 4.0;
base_flange_od = 80.0;
base_flange_t  = 5.0;
base_mount_bc  = 68.0; base_mount_n = 4; base_mount_m = 3.0;
base_wire_exit_w = 6.0;

/* [Fits — derived, do not edit casually] */
// Diametral allowances (+ = clearance, - = interference)
fit_inner_journal = (fab=="cnc") ? -0.005 : -0.10; // boss OD vs brg ID (k5 / FDM crush)
fit_outer_bore    = (fab=="cnc") ?  0.010 :  0.15; // skirt bore vs brg OD (H7 / FDM)
fit_sr_bore       = (fab=="cnc") ?  0.05  :  0.20;
fit_hole          = (fab=="cnc") ?  0.20  :  0.30;
crush_ribs        = (fab=="fdm");
rib_n = 8; rib_h = 0.25; // FDM crush-rib protrusion (radial)

// ------------------- derived geometry --------------------------------
rotor_top_z   = floor_t + mot_h;
boss_top_z    = rotor_top_z - axial_gap;
brg_bot_z     = boss_top_z - brg_w;
boss_bore_d   = mot_od + 2*mot_bell_clr;
boss_journal_d= brg_id - fit_inner_journal;     // interference => larger
boss_wall     = (brg_id - boss_bore_d)/2;
skirt_bore_d  = brg_od + fit_outer_bore;
skirt_od      = brg_od + 2*skirt_wall;
skirt_bot_z   = brg_bot_z - 1.0;                // 1 mm below bearing
sr_bot_z      = rotor_top_z + plate_t + 1.5;    // body floats above plate
sr_boss_od    = sr_od + 2*3.5;
arm_x         = arm_span/2 + arm_t/2;
tilt_z        = rotor_top_z + plate_t + arm_h;

// ------------------- design-rule checks ------------------------------
min_wall = (fab=="cnc") ? 1.5 : 2.4;
assert(boss_wall >= min_wall,
  str("Boss wall ", boss_wall, " < ", min_wall, ": choose larger bearing"));
assert(brg_od - 2*brg_shoulder_out > brg_id + 2*brg_shoul