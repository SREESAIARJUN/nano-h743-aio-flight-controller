# Pan-Axis Yoke: Parametric Geometry (Cycle 20)

I don't have your earlier cycles, so the dimensions below are **typical defaults, marked `VERIFY`**. Replace them with your measured part values. Every downstream feature derives from these parameters, so changing one value propagates through the model.

---

## 1. Design Intent & Axis Stack

```
            TILT AXIS (Y) ── bearing bosses on yoke arms
     ┌───────┐                     ┌───────┐
     │ ◎ arm │                     │ arm ◎ │   ◎ = 6700ZZ seat / tilt motor seat
     │   ║   │← wire groove        │       │
     │   ║   └─────────────────────┘       │
     │   ╚══════╗  HUB  (rotates)          │
     └──────────╫──────────────────────────┘
                ║  ← slip-ring rotor leads exit radially
          ┌─────╫─────┐
          │ 6.5mm SR  │ ← slip-ring body keyed to yoke (rotor side)
          ├───────────┤
          │ BLDC rotor│ ← hollow shaft, SR stator leads pass through
          │ BLDC stat.│
          └───────────┘  BASE (fixed)
```

**Key decisions:**

- **The yoke bolts to the motor rotor bell.** The slip-ring body sits coaxially in the yoke hub, and its stator leads exit downward through the motor's hollow shaft.
- **An upper support bearing (6700ZZ, 10×15×4) is optional.** It goes in a seat above the slip-ring capsule and takes overturning moment off the motor bearings when the tilt payload is offset.
- **Bearing seats use an outer-race-only shoulder**, so the shoulder never rubs the inner race or seals.
- **One switch changes the fit strategy.** `process = "FDM"` adds hole compensation. `process = "CNC"` uses true ISO fits (N7 press / H7 slip).

---

## 2. OpenSCAD Source: `pan_yoke_c20.scad`

```openscad
// =====================================================================
//  PAN-AXIS YOKE + HOLLOW-SHAFT BLDC SEATS + 6.5mm SLIP-RING CHANNEL
//  Cycle 20  |  Units: mm  |  Origin: motor rotor face, Z+ = up
// =====================================================================

/* [Process] */
process     = "FDM";   // "FDM" or "CNC"
fdm_hole_comp = 0.15;  // diametral add for printed holes (calibrate!)
$fn         = 160;
eps         = 0.01;

/* [Pan BLDC motor : VERIFY against datasheet] */
motor_od          = 40.5;   // rotor bell OD
motor_shaft_id    = 7.0;    // hollow shaft through-bore (GM3506-class)
rotor_bolt_bc     = 25.0;   // bolt circle on rotor face  VERIFY
rotor_bolt_n      = 4;
rotor_bolt_d      = 2.5;    // M2.5 VERIFY (some are M3)
rotor_bolt_head_d = 4.8;
rotor_boss_d      = 12.0;   // raised hub on rotor face to clear
rotor_boss_h      = 0.8;

/* [6.5 mm slip ring : VERIFY] */
sr_od          = 6.5;
sr_len         = 14.0;      // capsule body length (no flange variant)
sr_clear       = 0.10;      // radial-ish diametral clearance
sr_wire_bundle = 3.2;       // e.g. 6x 30AWG rotor leads
sr_setscrew    = 2.0;       // M2 nylon-tip anti-rotation
sr_keyed       = true;      // true = set-screw lock, false = adhesive only

/* [Upper support bearing 6700ZZ] */
use_support_brg = true;
brg_od   = 15.0;
brg_id   = 10.0;
brg_w    = 4.0;
brg_shoulder_d = 13.4;      // < outer-race land, > inner race OD (~11.4)
brg_interf_cnc = 0.010;     // N7-ish press for CNC (diametral)
brg_interf_fdm = 0.00;      // printed: rely on comp + Loctite 641

/* [Yoke geometry] */
hub_d        = motor_od + 2;  // slight overhang to shroud motor bell
hub_t        = 6.0;           // hub floor thickness above rotor face
hub_core_h   = sr_len + 2.0;  // central tower enclosing slip ring
arm_span     = 70.0;          // inner face to inner face
arm_t        = 6.0;
arm_w        = 22.0;
tilt_z       = 55.0;          // tilt axis height above rotor face
tilt_boss_d  = 26.0;
tilt_brg_od  = 15.0; tilt_brg_w = 4.0; tilt_brg_shoulder = 13.4;
wire_groove_w = 3.6;
wire_groove_d = 2.4;
fillet_r     = 3.0;

/* [Derived / checks] */
function hole(d) = (process=="FDM") ? d + fdm_hole_comp : d;
function press(d)= (process=="CNC") ? d - brg_interf_cnc : hole(d) - brg_interf_fdm;

sr_bore      = hole(sr_od + sr_clear);
min_wall_sr  = (hub_core_od() - sr_bore)/2;
function hub_core_od() = max(sr_bore + 6, (use_support_brg ? brg_od + 5 : 0));

assert(motor_shaft_id >= sr_od + 0.3 || true,
  "INFO: slip ring does NOT pass through shaft; only stator leads do. OK if intended.");
assert(motor_shaft_id >= sr_wire_bundle + 0.5,
  "FAIL: stator wire bundle will not pass through motor hollow shaft");
assert(brg_shoulder_d > brg_id + 1.2 && brg_shoulder_d < brg_od - 1.0,
  "FAIL: bearing shoulder contacts inner race or misses outer race");
assert(min_wall_sr >= 1.6, "FAIL: slip-ring channel wall < 1.6 mm");
assert(arm_span/2 - arm_t > hub_d/2 - 20, "WARN: check arm/hub blend");
echo(str("SR bore=",sr_bore," | wall=",min_wall_sr,
         " | brg seat=",press(brg_od)," | process=",process));

// =====================================================================
//  REUSABLE NEGATIVE: bearing seat (outer race shoulder + lead-in)
// =====================================================================
module bearing_seat_neg(od, w, shoulder_d, through_d, chamfer=0.4) {
    bore = press(od);
    // seat pocket
    translate([0,0,-eps]) cylinder(d=bore, h=w+eps);
    // lead-in chamfer
    translate([0,0,-eps]) cylinder(d1=bore+2*chamfer, d2=bore, h=chamfer+eps);
    // relief undercut at shoulder corner (prevents corner radius bind)
    translate([0,0,w-0.3]) cylinder(d=bore+0.4, h=0.3+eps);
    // through hole past shoulder
    translate([0,0,w-eps]) cylinder(d=shoulder_d, h=50);
}

// =====================================================================
//  SLIP-RING CHANNEL (negative)
// =====================================================================
module slip_ring_channel_neg() {
    // main capsule bore, entered from below (rotor face side)
    translate([0,0,-eps]) cylinder(d=sr_bore, h=hub_t + hub_core_h);
    // stator-lead pass-through into motor hollow shaft
    translate([0,0,-10]) cylinder(d=hole(motor_shaft_id), h=10+eps);
    // rotor-lead radial exit slot, aligned to +X arm groove
    translate([0,-wire_groove_w/2, hub_t + sr_len - 1])
        cube([hub_d, wire_groove_w, 6]);
    // anti-rotation set screw (radial, at capsule mid-length, -Y side)
    if (sr_keyed)
        translate([0,0,hub_t + sr_len*0.5]) rotate([90,0,0])
            cylinder(d=hole(sr_setscrew)*0.85, h=hub_core_od()); // tap-drill
    // top: support bearing seat
    if (use_support_brg)
        translate([0,0,hub_t + hub_core_h + brg_w]) mirror([0,0,1])
            bearing_seat_neg(brg_od, brg_w, brg_shoulder_d, sr_bore);
}

// =====================================================================
//  MOTOR INTERFACE (negative)
// =====================================================================
module motor_interface_neg() {
    // rotor boss clearance pocket
    translate([0,0,-eps]) cylinder(d=hole(rotor_boss_d+0.4), h=rotor_boss_h+0.2);
    for (i=[0:rotor_bolt_n-1]) rotate(i*360/rotor_bolt_n + 45)
        translate([rotor_bolt_bc/2,0,0]) {
            translate([0,0,-eps]) cylinder(d=hole(rotor_bolt_d+0.2), h=hub_t+1);
            // counterbore from top; screw length = hub_t - cbore + engagement
            translate([0,0,hub_t-2.6]) cylinder(d=hole(rotor_bolt_head_d+0.3), h=20);
        }
}

// =====================================================================
//  POSITIVE BODIES
// =====================================================================
module hub_body() {
    cylinder(d=hub_d, h=hub_t);
    cylinder(d=hub_core_od(), h=hub_t + hub_core_h + (use_support_brg?brg_w+1.5:0));
}

module arm(side) {           // side = +1 / -1 along X
    x0 = side*(arm_span/2 + arm_t/2);
    hull() {
        translate([x0 - arm_t/2, -arm_w/2, 0]) cube([arm_t, arm_w, hub_t]);
        translate([x0, 0, tilt_z]) rotate([0,90,0])
            cylinder(d=tilt_boss_d, h=arm_t, center=true);
    }
    // floor bridge from hub to arm
    hull() {
        cylinder(d=hub_d*0.7, h=hub_t);
        translate([x0 - arm_t/2, -arm_w/2, 0]) cube([arm_t, arm_w, hub_t]);
    }
}

module tilt_seat_neg(side) {
    x_in = side*(arm_span/2);
    translate([x_in,0,tilt_z]) rotate([0, side*90, 0])
        bearing_seat_neg(tilt_brg_od, tilt_brg_w, tilt_brg_shoulder, 0);
}

module wire_groove_neg() {   // inner face of +X arm, hub exit -> tilt bore
    x = arm_span/2;
    translate([x - wire_groove_d, -wire_groove_w/2, hub_t - 1])
        cube([wire_groove_d + eps, wire_groove_w, tilt_z - hub_t]);
    // floor run from hub exit to arm
    translate([hub_core_od()/2 - 1, -wire_groove_w/2, hub_t - wire_groove_d])
        cube([x - hub_core_od()/2 + 1, wire_groove_w, wire_groove_d + eps]);
}

// =====================================================================
//  ASSEMBLY
// =====================================================================
module pan_yoke() {
    difference() {
        union() { hub_body(); arm(+1); arm(-1); }
        slip_ring_channel_neg();
        motor_interface_neg();
        tilt_seat_neg(+1); t