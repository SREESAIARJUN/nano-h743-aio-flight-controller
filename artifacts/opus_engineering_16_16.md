# Pan-Axis Yoke, Bearing Seats & 6.5 mm Slip-Ring Channel: Parametric Geometry (Cycle 16)

I don't have the outputs of earlier cycles, so this design states every assumption explicitly as a parameter. Lines marked `// VERIFY` must be measured against your actual parts before you cut metal or print.

---

## 1. Architecture

```
        ┌──── tilt motor ────┐          ← yoke arm tips (tilt axis)
        │                    │
        │      YOKE ARMS     │
        └───────┬──┬─────────┘
                │  │  ← yoke hub (bolts to motor ROTOR)
           ═════╪══╪═════  pan motor (hollow shaft)
                │▓▓│  ← rotating hollow tube (pressed/bonded into yoke hub)
         ┌──────┤▓▓├──────┐
         │ [B1] │▓▓│ [B1] │  ← upper bearing seat  ┐ duplex pair, spaced
         │      │▓▓│      │                        │ for tilt-moment stiffness
         │ [B2] │▓▓│ [B2] │  ← lower bearing seat  ┘
         │    ┌─┴──┴─┐    │
         │    │ SLIP │    │  ← 6.5 mm capsule pocket + anti-rotation key
         │    │ RING │    │
         └────┴──┬───┴────┘  ← base housing (fixed); stator wires exit side slot
```

**Design rationale:**

- **Motor bearings vs. moment loads.** Small gimbal motors have tiny internal bearings that should not carry the yoke's overhung moment. A dedicated duplex bearing pair, spaced by `brg_span`, takes the moment. The motor only supplies torque.
- **Slip-ring placement.** The slip-ring capsule (Ø6.5) lives in the fixed base, not inside the motor. Only its rotor-side wire bundle travels up the hollow tube. This removes the most common failure mode: trying to pass a 6.5 mm body through a ~5 mm motor bore.
- **Slip-ring decoupling.** The slip-ring rotor is driven by its wire bundle plus a soft key. It is never rigidly co-constrained with the shaft, so the slip ring and the bearings cannot fight over concentricity.

---

## 2. OpenSCAD Source (`pan_yoke_c16.scad`)

```openscad
// =====================================================================
//  Pan-Axis Yoke + Bearing Seats + 6.5mm Slip-Ring Channel — Cycle 16
//  Units: mm.  Z+ = up.  Pan axis = Z axis.
// =====================================================================

/* [Process] */
process     = "FDM";        // "FDM" | "CNC"
part        = "assembly";   // "assembly" | "base" | "yoke" | "tube" | "section"
explode     = 0;            // assembly explode distance
$fn         = 128;

/* [Bearings — default 6801-2RS 12x21x5] */
brg_id      = 12.0;
brg_od      = 21.0;
brg_w       = 5.0;
brg_span    = 14.0;   // centre-to-centre spacing of duplex pair
brg_shoulder= 1.6;    // radial shoulder height (must clear inner/outer race only)

/* [Hollow tube (rotating)] */
tube_od     = brg_id;     // nominal, fit applied below
tube_id     = 7.2;        // wire bundle passage
tube_len    = 46.0;

/* [Pan motor — hollow shaft]  VERIFY ALL */
mot_od         = 40.0;    // VERIFY
mot_h          = 16.0;    // VERIFY
mot_bore       = 12.8;    // VERIFY — hollow-shaft clear bore
rotor_pcd      = 25.0;    // VERIFY
rotor_bolt     = 2.5;     // M2.5 VERIFY
stator_pcd     = 25.0;    // VERIFY (often differs from rotor PCD)
stator_bolt    = 3.0;     // M3 VERIFY
stator_bolt_rot= 45;      // angular offset stator vs rotor pattern

/* [Slip ring 6.5mm capsule]  VERIFY */
sr_d        = 6.5;
sr_len      = 17.0;       // VERIFY body length
sr_key_w    = 1.6;        // anti-rotation tab width (or flat)
sr_key_h    = 1.2;
sr_wire_d   = 3.2;        // stator-side bundle exit
sr_rotor_bundle_d = 3.0;  // rotor-side bundle (must < tube_id)

/* [Yoke] */
yoke_span   = 70.0;       // inside distance between arms (camera width + clearance)
arm_t       = 5.0;
arm_w       = 18.0;
arm_h       = 55.0;       // hub top to tilt axis
hub_t       = 6.0;
hub_d       = 44.0;
tilt_bore   = 12.0;       // tilt-motor spigot / bearing seat
tilt_pcd    = 19.0;
tilt_bolt   = 2.5;
fillet_r    = 4.0;

/* [Base housing] */
base_wall   = 3.0;
base_floor  = 3.0;
base_flange = 60.0;
mount_pcd   = 50.0;
mount_bolt  = 3.4;

// ---------------------------------------------------------------------
//  Fits  (H7/k6-equivalent intent; tuned per process)
// ---------------------------------------------------------------------
fdm         = (process == "FDM");
// Outer-race seat bore (housing). FDM: slight oversize + crush ribs.
seat_bore   = fdm ? brg_od + 0.20 : brg_od + 0.000;  // CNC: ream Ø21 H7 (+0/+0.021)
rib_n       = 6;
rib_proud   = 0.25;      // crush rib interference (FDM only)
// Inner race on tube: light interference / locational.
tube_fit_od = fdm ? brg_id - 0.02 : brg_id + 0.005;  // CNC: k6 (+0.001/+0.012)
// Slip-ring pocket: clearance; ring must float radially.
sr_pocket_d = sr_d + (fdm ? 0.35 : 0.15);
mot_clear   = 0.6;       // tube-to-motor-bore radial clearance check

// ---------------------------------------------------------------------
//  Sanity checks — fail loudly
// ---------------------------------------------------------------------
assert(tube_fit_od + 2*mot_clear <= mot_bore,
       "Tube will rub motor hollow shaft: increase mot_bore or reduce bearing ID");
assert(sr_rotor_bundle_d + 0.8 <= tube_id,
       "Rotor wire bundle too fat for tube bore");
assert((tube_od - tube_id)/2 >= 1.8,
       "Tube wall < 1.8mm — too thin for press fit");
assert(brg_span >= brg_w + 3,
       "Bearing spacer too short");
assert(brg_shoulder < (brg_od - brg_id)/2 * 0.5,
       "Shoulder would touch seals/opposite race");

// Derived Z stack (base housing frame, z=0 = base floor underside)
z_sr_bot   = base_floor;
z_sr_top   = z_sr_bot + sr_len;
z_b2       = z_sr_top + 2.0;                 // lower bearing face
z_b1       = z_b2 + brg_span;                // upper bearing lower face
z_base_top = z_b1 + brg_w + 1.0;             // housing top / motor stator seat
z_mot_top  = z_base_top + mot_h;
z_hub      = z_mot_top;                      // yoke hub sits on rotor

// ---------------------------------------------------------------------
//  Primitives
// ---------------------------------------------------------------------
module bolt_circle(pcd, n=4, rot=0) {
    for (i=[0:n-1]) rotate([0,0,rot + i*360/n]) translate([pcd/2,0,0]) children();
}

module bearing_seat(z0) {
    // Pocket + crush ribs (FDM). Subtract this from the housing.
    translate([0,0,z0]) difference() {
        cylinder(d=seat_bore, h=brg_w + 0.01);
        if (fdm)
            for (i=[0:rib_n-1]) rotate([0,0,i*360/rib_n])
                translate([seat_bore/2 - rib_proud, -0.6, -1])
                    cube([rib_proud + 0.5, 1.2, brg_w + 2]);
    }
    // 0.5 x 45° lead-in chamfer (top of seat only)
    translate([0,0,z0 + brg_w - 0.5])
        cylinder(d1=seat_bore, d2=seat_bore + 1.0, h=0.51);
    // Undercut relief at seat floor — stops fillet/ elephant-foot preventing seating
    translate([0,0,z0 - 0.4]) cylinder(d=seat_bore + 0.6, h=0.45);
}

module slip_ring_channel() {
    // Capsule pocket
    translate([0,0,z_sr_bot]) cylinder(d=sr_pocket_d, h=sr_len + 0.2);
    // Anti-rotation key slot (full length, +X side)
    translate([sr_pocket_d/2 - 0.3, -sr_key_w/2 - 0.1, z_sr_bot])
        cube([sr_key_h + 0.5, sr_key_w + 0.2, sr_len + 0.2]);
    // Stator-side wire exit: radial slot through wall at floor level (-X side)
    translate([0,0,z_sr_bot + sr_wire_d/2 + 0.5]) rotate([0,-90,0])
        cylinder(d=sr_wire_d, h=base_flange);
    translate([-base_flange, -sr_wire_d/2, z_sr_bot])
        cube([base_flange, sr_wire_d, sr_wire_d/2 + 0.5]);
    // Rotor-side bundle path up to tube
    translate([0,0,z_sr_top]) cylinder(d=tube_id, h=z_b2 - z_sr_top + 0.1);
    // Insertion/service: floor through-hole
    translate([0,0,-1]) cylinder(d=sr_pocket_d - 1.2, h=base_floor + 2);
}

// ---------------------------------------------------------------------
//  BASE HOUSING (fixed)
// ---------------------------------------------------------------------
module base_housing() {
    hous_od = seat_bore + 2*base_wall + 2;
    difference() {
        union() {
            cylinder(d=base_flange, h=base_floor);
            cylinder(d=hous_od, h=z_base_top);
            // stator landing pad
            translate([0,0,z_base_top - 3]) cylinder(d=max(hous_od, stator_pcd + 8), h=3);
            // gussets
            for (a=[0:90:270]) rotate([0,0,a+45])
                hull() {
                    translate([hous_od/2 - 1, -1.