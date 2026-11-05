# Cycle 7: Pan-Axis Yoke, Hollow-Shaft Bearing Seats, and 6.5 mm Slip-Ring Channel

I don't have the outputs of Cycles 1–6, so **every interface dimension below is a named parameter with a default**. Override the defaults with your locked values. Anything marked `VERIFY` must be checked against the datasheet of the specific part you have.

---

## 1. Architecture (pan stack, bottom → top)

```
          ┌────────── YOKE (U-arms → tilt axis) ──────────┐
          │   wire exit slot ◄── slip-ring ROTOR leads    │
          └───────┬───────────[clamp hub]─────────┬───────┘
                  │  motor ROTOR bolts → yoke     │
           ┌──────┴─────── BLDC (hollow) ─────────┴──┐
           │  motor STATOR bolts → base top face     │
           └──────┬──────────────────────────────────┘
   BASE ┌─────────┤ hollow shaft Ø10 (bore Ø6.9) ├─────────┐
        │ [6800]  │   ║ slip ring Ø6.5 capsule  ║ │ [6800] │ ← duplex bearing pair,
        │ spacer  │   ║                         ║ │ spacer │   outer-race spacer
        │ [6800]  │   ║                         ║ │ [6800] │
        └─────────┴── retaining nut / circlip ───┴─────────┘
                 └─► slip-ring STATOR flange + anti-rotation key → base underside
```

Load paths and design decisions:

- **Thrust and moment load** goes through a pair of 6800-2RS bearings (10×19×5). The pair is spaced to give a long moment arm, so the motor's own bearings carry almost nothing.
- **The shaft is fixed to the yoke** and spins with it. The slip-ring rotor rides inside the shaft bore. The slip-ring stator flange is keyed to the base.
- **The motor bore must clear the shaft.** The code asserts this. A 2804H-class motor (≈6 mm bore) will **fail** the assert, so use a 4108H / 3506H-class motor, or use the alternate layout in §5.

---

## 2. OpenSCAD: Full Parametric Model

```openscad
// ============================================================
//  PAN AXIS – Cycle 7   |  yoke + bearing seats + slip-ring channel
//  Units: mm.  Set PRINTED=true for FDM compensation, false for CNC.
// ============================================================
$fn = 96;

/* [Build] */
PART      = "assembly";   // assembly | base | shaft | yoke | spacer | nut
EXPLODE   = 0;            // 0..30 visual explode
PRINTED   = true;

/* [Slip ring – VERIFY] */
sr_d          = 6.5;   // capsule OD
sr_len        = 17.0;  // body length below flange
sr_flange_d   = 12.5;  // stator flange OD
sr_flange_t   = 1.0;
sr_flange_hole_bc = 9.0;   // 2× M1.6 on this BC (or use key tab)
sr_clear      = 0.20;  // radial clearance → bore = sr_d + 2*clear*... see below
sr_wire_bundle= 3.0;   // rotor lead bundle Ø (6-wire ≈ 2.5–3)

/* [Bearings 6800-2RS] */
brg_d = 10; brg_D = 19; brg_B = 5;
brg_span   = 14;       // centre-to-centre of the two bearings
brg_lip    = 1.0;      // shoulder radial height in housing

/* [Fits] */
// CNC: housing Ø19 K6/M6-ish -> nominal -0.005 ; shaft j6 -> +0.004
// FDM: holes print undersize; tune with a test coupon (see §4)
hole_comp  = PRINTED ? 0.15 : -0.005;   // added to bearing seat Ø
shaft_comp = PRINTED ? -0.05 : 0.004;   // added to shaft seat Ø

/* [Shaft] */
shaft_od    = brg_d;
shaft_bore  = sr_d + 2*sr_clear;      // 6.9
shaft_wall_min = 1.2;
shaft_shoulder_d = 12.5;              // ≤ inner-race land (6800 ≈ 12.2-13)
thread_len  = 4;                      // M10x0.75 fine thread for lock nut

/* [Motor – hollow BLDC, VERIFY] */
mot_od        = 46;     // rotor OD (4108H ≈ 46)
mot_h         = 24;     // overall height excl. shaft
mot_bore      = 12;     // hollow through-bore
mot_stator_bc = 25;  mot_stator_n = 4; mot_stator_screw = 3.0; // M3
mot_rotor_bc  = 19;  mot_rotor_n  = 4; mot_rotor_screw  = 2.5; // M2.5
mot_air_gap   = 0.6;    // axial clearance rotor-to-base

/* [Base] */
base_od   = 40;
base_wall = 3.0;
base_mount_bc = 32; base_mount_n = 3; base_mount_screw = 3.0;

/* [Yoke] */
yoke_span     = 70;   // inner distance between arms (payload width)
yoke_arm_t    = 6;
yoke_arm_w    = 20;
yoke_arm_h    = 55;   // pan-plane to tilt-axis height
yoke_floor_t  = 6;
tilt_brg_D    = 13; tilt_brg_B = 4;   // 683/MR138 for tilt (VERIFY)
hub_len       = 8;    // shaft engagement in yoke hub
hub_clamp_screw = 2.5;

// ---------- derived ----------
housing_len = brg_span + brg_B + 2*base_wall;
spacer_len  = brg_span - brg_B;
assert(shaft_od - shaft_bore >= 2*shaft_wall_min, "Shaft wall too thin for slip ring");
assert(mot_bore >= shaft_od + 0.4, "Motor bore does not clear shaft – use alt layout §5");
assert(shaft_shoulder_d < brg_D - 4, "Shoulder will rub outer race / seal");

// ---------- helpers ----------
module bolt_circle(bc, n, d, h, rot=0) {
    for (i=[0:n-1]) rotate(rot + i*360/n) translate([bc/2,0,0])
        cylinder(d=d, h=h);
}
module hex_nut_trap(m, h) cylinder(d=m*1.85/cos(30), h=h, $fn=6);

// ============================================================
//  BASE: duplex bearing housing + motor stator seat + SR anchor
// ============================================================
module base() {
    seat_d = brg_D + hole_comp;
    difference() {
        union() {
            cylinder(d=base_od, h=housing_len);
            // mounting flange
            cylinder(d=base_mount_bc + 10, h=4);
        }
        // upper bearing seat (open top)
        translate([0,0,housing_len - brg_B - base_wall + 0.001])
            cylinder(d=seat_d, h=brg_B + base_wall);
        // lower bearing seat (open bottom)
        translate([0,0,-0.001]) cylinder(d=seat_d, h=brg_B + base_wall);
        // centre: spacer cavity (same Ø, spacer locates outer races)
        cylinder(d=seat_d, h=housing_len);
        // NOTE: pure through-bore + outer spacer = no machined shoulders
        //       → both seats bored in ONE setup = best coaxiality.

        // motor stator screws (top) w/ nut traps from below
        translate([0,0,housing_len-8])
            bolt_circle(mot_stator_bc, mot_stator_n, mot_stator_screw+0.2, 9, 45);
        translate([0,0,housing_len-8-2.4])
            for(i=[0:mot_stator_n-1]) rotate(45+i*360/mot_stator_n)
              translate([mot_stator_bc/2,0,0]) hex_nut_trap(mot_stator_screw,2.4);
        // base mounting
        translate([0,0,-1]) bolt_circle(base_mount_bc, base_mount_n, base_mount_screw+0.2, 10, 0);
        // slip-ring stator wire exit (radial slot in flange underside)
        translate([0,-1.75,-0.01]) cube([base_od, 3.5, 2.5]);
    }
}

// Lower retainer plate: holds bottom bearing outer race + clocks SR stator
module sr_anchor_plate() {
    difference() {
        cylinder(d=base_mount_bc + 10, h=2.5);
        translate([0,0,-1]) cylinder(d=shaft_shoulder_d + 1, h=5);   // clear rotating nut
        translate([0,0,-1]) bolt_circle(base_mount_bc, base_mount_n, base_mount_screw+0.2, 5);
        // slip-ring flange screws
        translate([0,0,-1]) bolt_circle(sr_flange_hole_bc, 2, 1.7, 5, 90);
    }
}

// ============================================================
//  OUTER-RACE SPACER (sets bearing span; light axial preload)
// ============================================================
module spacer() {
    difference() {
        cylinder(d=brg_D - 0.05, h=spacer_len);
        translate([0,0,-1]) cylinder(d=brg_D - 2*1.6, h=spacer_len+2); // contacts outer race only
    }
}

// ============================================================
//  HOLLOW SHAFT  (yoke hub ↔ upper brg ↔ lower brg ↔ nut)
// ============================================================
shaft_len = hub_len + (mot_h + mot_air_gap) + housing_len + thread_len + 1;
module shaft() {
    seat = shaft_od + shaft_comp;
    difference() {
        union() {
            cylinder(d=seat, h=shaft_len);
            // shoulder that bears on upper inner race
            translate([0,0, thread_len + housing_len - base_wall + 0.5])
                cylinder(d=shaft_shoulder_d, h=1.5);
        }
        // slip-ring channel – full-length through-bore
        translate([0,0,-1]) cylinder(d=shaft_bore, h=shaft_len+2);
        // flat for yoke hub anti-rotation (D-profile)
        translate([seat/2 - 0.8, -seat, shaft_len - hub_len])
            cube([2, 2*seat, hub_len+1]);
        // entry chamfer so slip-ring wires don't skin
        translate([0,0,shaft_len-1.0]) cylinder(d1=shaft_bore, d2=shaft_bore+2, h=1.01);
    }
    // NOTE: thread M10x0.75 over bottom thread_len – cut on lathe, or
    //       for FDM replace with e-clip groove: Ø9.6 x 0.7 wide.
}

module lock_nut() {   // M10x0.75 thin nut placeholder (use DIN 1804 / KM0 if CNC)
    difference() {
        cylinder(d=16, h=3, $fn=6);
        translate([0,0,-1]) cylinder(d=shaft_od, h=5);
    }
}

// ============================================================
//  YOKE
// ============================================================
module yoke() {
    W = yoke_span + 2*yoke_arm_t;
    difference() {
        union() {
            // floor plate
            translate([-W/2, -yoke_arm_w/2, 0]) cube([W, yoke_arm_w, yoke_floor_t]);
            // motor-rotor disc
            cylinder(d=mot_od - 2, h=yoke_floor_t);
            // hub (down into motor bore? no – hub sits above floor)
            cylinder(d=shaft