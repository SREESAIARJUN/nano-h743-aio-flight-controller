# Pan-Axis Yoke: Parametric Geometry (Cycle 21)

I don't have the earlier cycles, so I've built this as a fully parametric baseline. Every dimension is a named variable. Values marked `⚠ MEASURE` are typical for a GBM2804H-class hollow-shaft gimbal motor and a 6.5 mm capsule slip ring. Confirm them with calipers before you cut metal.

---

## 1. Architecture

```
            TILT AXIS (Y)
   ┌──────────┐        ┌──────────┐
   │ Arm A    │        │ Arm B    │
   │ tilt-    │        │ idler    │
   │ motor    │        │ bearing  │
   │ stator   │        │ seat     │
   │ pocket   │        │          │
   └────┬─────┘        └─────┬────┘
        │   ┌────────────┐   │
        └───┤  HUB       ├───┘   ← yoke crossbar
            │ ┌────────┐ │
            │ │SLIP    │ │  ← 6.5 mm slip-ring channel (rotor keyed to yoke)
            │ │RING    │ │
            │ └──┬─────┘ │
            └────┼───────┘
       ══════════╪══════════  ← pan motor ROTOR face (bolts into hub underside)
       │  PAN MOTOR (hollow)│
       ══════════╪══════════  ← stator → handle (fixed)
                 │ stator-side wires exit through hollow shaft
            PAN AXIS (Z)
```

Key decisions:

- **The slip ring sits in the yoke hub, above the motor, not inside the hollow shaft.** A 2804H shaft bore (about 5.5 to 6 mm) usually cannot pass a 6.5 mm capsule. Only the stator-side leads go down the shaft. If your shaft bore is at least 7.0 mm, set `slipring_in_shaft = true` and the code relocates the channel.
- **The slip-ring rotor is keyed to the yoke and the stator to the shaft/handle**, so the ring sees exactly the yoke rotation.
- **The yoke's single point of rotation is the pan motor's own bearings.** The yoke is supported only through the rotor bolt circle, so its CG should be kept over the axis (see the echo checks).
- **Tilt axis:** a motor stator pocket in Arm A and an idler bearing seat in Arm B. The bores are line-bored coaxially by construction: both are generated from the same axis variable.

---

## 2. OpenSCAD Source (`pan_yoke_c21.scad`)

```openscad
// =====================================================================
//  PAN-AXIS YOKE — Cycle 21
//  Units: mm.  Z = pan axis.  Y = tilt axis.  Origin = motor rotor face.
// =====================================================================

/* [Process] */
process         = "fdm";      // "fdm" | "sla" | "cnc"
$fn             = 128;

/* [Pan motor — hollow shaft  ⚠ MEASURE] */
pm_rotor_od        = 34.0;    // rotor bell OD
pm_rotor_pilot_d   = 12.0;    // centering boss/recess on rotor face
pm_rotor_pilot_h   = 1.0;
pm_bolt_pcd        = 16.0;    // rotor bolt circle (some use 12/16 slots)
pm_bolt_n          = 4;
pm_bolt_d          = 2.0;     // M2
pm_bolt_head_d     = 3.8;     // M2 SHCS head
pm_bolt_head_h     = 2.0;
pm_shaft_bore      = 5.8;     // hollow shaft ID
pm_shaft_protrude  = 0.8;     // shaft/circlip above rotor face (clearance)

/* [Slip ring 6.5 mm capsule  ⚠ MEASURE] */
sr_od            = 6.5;
sr_len           = 13.0;      // capsule body length
sr_flange_d      = 14.0;      // stator flange (0 if flangeless)
sr_flange_t      = 1.0;
sr_rotor_flat    = 5.6;       // across-flat of rotor anti-rotation key
sr_wire_bundle_d = 3.2;       // rotor-side lead bundle
slipring_in_shaft = false;    // true only if pm_shaft_bore >= sr_od+0.5

/* [Yoke geometry] */
hub_od          = 38.0;
hub_h           = 22.0;       // must enclose slip ring + floor
hub_floor_t     = 3.0;        // material under bolt heads
arm_span        = 72.0;       // inner face-to-face distance (payload width + clearance)
arm_t           = 6.0;        // arm wall thickness (Y)
arm_w           = 24.0;       // arm width (X)
arm_h           = 58.0;       // hub top → tilt axis height
crossbar_h      = 7.0;
fillet_r        = 4.0;        // internal root fillets
tilt_axis_z     = hub_h + arm_h;

/* [Tilt motor pocket — Arm A  ⚠ MEASURE] */
tm_stator_pilot_d = 15.0;
tm_pilot_depth    = 2.0;
tm_bolt_pcd       = 12.0;
tm_bolt_n         = 4;
tm_bolt_d         = 2.0;
tm_wire_slot_w    = 3.5;

/* [Idler bearing — Arm B] */
// Default MR105 (5x10x4). Alternative 685ZZ (5x11x5).
brg_od   = 10.0;
brg_id   = 5.0;
brg_w    = 4.0;
brg_shoulder_d = 7.5;         // ≥ inner-race land? NO — outer shoulder ID
                              // must be < OD and > OD of inner race (≈7.3)
brg_lip_t = 0.8;              // retaining shoulder thickness

// ---------------------------------------------------------------------
//  Process-dependent fit allowances (radial, on DIAMETER)
// ---------------------------------------------------------------------
function press_allow() =      // bearing OD seat
    process=="cnc" ? -0.008 :  // ≈ N7/K6 light interference (machine to K6)
    process=="sla" ?  0.06  :
                      0.12;    // FDM: hole shrink compensation → light press
function slip_allow() =       // locating clearance (pilots, capsules)
    process=="cnc" ?  0.02  :
    process=="sla" ?  0.12  :
                      0.25;
function bolt_clear(d) = d + (process=="cnc" ? 0.2 : 0.4);

// ---------------------------------------------------------------------
//  Assertions — fail fast on impossible geometry
// ---------------------------------------------------------------------
assert(pm_bolt_pcd/2 + pm_bolt_head_d/2 < hub_od/2 - 1.2,
       "Rotor bolt heads break out of hub wall");
assert(pm_bolt_pcd/2 - pm_bolt_head_d/2 > (sr_flange_d+slip_allow())/2 + 0.6
       || slipring_in_shaft,
       "Bolt counterbores intersect slip-ring flange seat — raise floor or shrink flange");
assert(!slipring_in_shaft || pm_shaft_bore >= sr_od + 0.5,
       "Slip ring cannot pass hollow shaft; set slipring_in_shaft=false");
assert(hub_h >= hub_floor_t + sr_len - (slipring_in_shaft?sr_len:0) + 1.5,
       "Hub too short for slip-ring capsule");
assert(brg_shoulder_d < brg_od - 1.0, "Bearing shoulder won't retain outer race");
assert(arm_t >= brg_w + brg_lip_t + 0.5, "Arm too thin for bearing + shoulder");

// ---------------------------------------------------------------------
//  Primitives
// ---------------------------------------------------------------------
module bolt_circle(pcd, n, ang0=45)
    for (i=[0:n-1]) rotate([0,0,ang0+i*360/n]) translate([pcd/2,0,0]) children();

module rounded_box(size, r) {          // XY-rounded block
    hull() for (x=[r,size.x-r], y=[r,size.y-r])
        translate([x,y,0]) cylinder(r=r, h=size.z);
}

// ---------------------------------------------------------------------
//  Solid body
// ---------------------------------------------------------------------
module hub_solid() cylinder(d=hub_od, h=hub_h);

module crossbar_solid() {
    L = arm_span + 2*arm_t;
    translate([-arm_w/2, -L/2, hub_h - crossbar_h])
        rounded_box([arm_w, L, crossbar_h], 3);
}

module arm_solid(side) {            // side = +1 (A, motor) / -1 (B, idler)
    y0 = side>0 ? arm_span/2 : -arm_span/2 - arm_t;
    hull() {
        translate([-arm_w/2, y0, hub_h - crossbar_h]) cube([arm_w, arm_t, 1]);
        translate([0, y0, tilt_axis_z]) rotate([-90,0,0])
            cylinder(d=arm_w, h=arm_t);
    }
}

module root_fillets() {             // concave fillet arm↔crossbar
    for (s=[-1,1]) {
        yf = s*(arm_span/2) - (s>0 ? fillet_r : -fillet_r);
        translate([-arm_w/2, 0, hub_h]) difference() {
            translate([0, s>0 ? arm_span/2-fillet_r : -arm_span/2, 0])
                cube([arm_w, fillet_r, fillet_r]);
            translate([-1, yf, fillet_r]) rotate([0,90,0])
                cylinder(r=fillet_r, h=arm_w+2);
        }
    }
}

// ---------------------------------------------------------------------
//  Cuts
// ---------------------------------------------------------------------
module pan_motor_interface() {
    // pilot recess for rotor centering boss (shared-axis registration)
    translate([0,0,-0.01])
        cylinder(d=pm_rotor_pilot_d + slip_allow(), h=pm_rotor_pilot_h+0.01);
    // shaft/circlip clearance
    translate([0,0,-0.01])
        cylinder(d=pm_shaft_bore+3.0, h=pm_rotor_pilot_h+pm_shaft_protrude+0.3);
    // bolts + counterbores from inside the hub
    bolt_circle(pm_bolt_pcd, pm_bolt_n) {
        translate([0,0,-1]) cylinder(d=bolt_clear(pm_bolt_d), h=hub_h);
        translate([0,0,hub_floor_t])
            cylinder(d=pm_bolt_head_d+0.5, h=hub_h);
    }
}

module slipring_channel() {
    z0 = slipring_in_shaft ? -sr_len : hub_floor_t;
    // capsule bore
    translate([0,0,z0]) cylinder(d=sr_od + slip_allow(), h=sr_len+0.01);
    // stator flange seat — stator is clamped to the SHAFT side, so the
    // flange floats in an oversize counterbore (rotation clearance 0.5 mm radial)
    if (sr_flange_d > 0)
        translate([0,0,z0-0.01]) cylinder(d=sr_flange_d + 1.0, h=sr_flange_t+0.4);
    // rotor anti-rotation key (D-flat) at top of capsule
    translate([0,0,z0+sr_len]) intersection() {
        cylinder(d=sr_od+slip_allow(), h=2.5);
        cube([sr_rotor_flat+slip_allow(), 20, 6], center=true);
    }
    // rotor-lead exit up through crossbar, then a groove toward Arm A
    translate([0,0,z0+sr_len]) cylinder(d=sr_wire_bundle_d+0.6, h=hub_h);
    translate([-(sr_wire_bundle_d+0.6)/2, 0, hub_h - 1.6])
        cube([sr_wire_bundle_d+0.6, arm_span/2, 2]);
    // stator leads continue down the hollow shaft
    translate([0,0,-1]) cylinder(d=pm_shaft_bore, h=z0+1.02);
}

module tilt_motor_pocket() {        // Arm A (+Y), inner face
    translate([0, arm_span/2 - 0.01, tilt_axis_z]) rotate([-90,0,0]) {
        cylinder(d=tm_stator_pilot_d + slip_allow(), h=tm_pilot_depth);
        cylinder(d=6.0, h=arm_t+1);                       // shaft/wire pass
        rotate([0,0,45]) bolt_circle(tm_bolt_pcd, tm_bolt_n)
            cylinder(d=bolt_clear(tm_bolt_d), h=arm_t+1);
    }
    // wire slot down the arm to the crossbar groove
    translate([-tm_wire_slot_w/2, arm_span/2+arm_t-1.8, hub_h])
        cube([tm_wire_slot_w, 2, arm_h]);
}

module idler_bearing_seat() {       //