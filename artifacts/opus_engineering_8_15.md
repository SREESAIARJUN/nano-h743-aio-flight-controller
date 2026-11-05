# Pan-Axis Yoke: Parametric Geometry (Cycle 8)

I don't have your earlier cycles' outputs, so I've set every interface dimension as a named parameter with a stated assumption. Measure your actual motor and slip ring and override the values in the **INTERFACE** block. The `assert()` checks will flag any combination that won't assemble.

---

## 1. Architecture & Load Path

```
        ┌──────────── YOKE ARMS (to tilt axis) ────────────┐
        │                                                   │
        └──────────────┬──── yoke plate ────┬───────────────┘
                       │  bolted to ROTOR   │
               ┌───────┴────────────────────┴───────┐
               │      BLDC MOTOR (hollow shaft)     │  ← stator bolted to base
               └───────┬────────────────────┬───────┘
                ┌──────┤  spindle tube      ├──────┐
                │ ═════╡  (integral w/yoke) ╞═════ │  ← bearing #1 (6700ZZ)
                │      │  ┌──────────────┐  │      │
                │      │  │ SLIP RING    │  │      │  ← Ø6.5 capsule inside spindle
                │ ═════╡  │ (rotor→yoke) │  ╞═════ │  ← bearing #2 (6700ZZ)
                │      └──┤              ├──┘      │
                │   base  └──── stator ──┘ wires   │  ← stator anti-rotation + exit slot
                └──────────────────────────────────┘
```

**Design decisions:**

- **The spindle tube is integral with the yoke** and passes down through the motor's hollow bore. Two widely spaced bearings carry the overturning moment from the yoke arms, so the motor's internal bearings only see torque.
- **The slip ring sits coaxially inside the spindle.**
  - Its rotor leads go up into the yoke through a side window.
  - Its stator is clamped by a grub screw in the base and exits through a radial slot.
- **Bearing choice: 6700ZZ (10×15×4).** A 10 mm spindle around a Ø6.5 slip ring leaves a 1.55 mm wall after clearance. That is fine in aluminium and marginal-but-OK in PETG/CF-nylon. If your motor bore is ≥12.2 mm, switch to 6701 (12×18×4).

---

## 2. OpenSCAD Source (`pan_yoke_c8.scad`)

```openscad
// =====================================================================
//  PAN-AXIS YOKE + BEARING SEATS + Ø6.5 SLIP-RING CHANNEL — Cycle 8
//  Units: mm.  Render parts individually via `part`.
// =====================================================================
part = "assembly";   // "base" | "yoke" | "assembly" | "section"
$fn  = 96;

// ---------------- MANUFACTURING MODE ----------------
fdm        = true;    // false = CNC/lathe nominal fits
hole_comp  = fdm ? 0.15 : 0.0;  // FDM holes print undersize (diametral)
// Bearing seat fits (diametral). Tune with a test coupon!
seat_od_fit   = fdm ? -0.05 : -0.008; // housing: light press (≈ N7 machined)
shaft_fit     = fdm ? -0.10 : +0.004; // spindle: slide/transition (≈ k5)

// ---------------- INTERFACE: MEASURE & OVERRIDE ----------------
// Bearing (6700ZZ default)
brg_id = 10;  brg_od = 15;  brg_w = 4;
brg_shoulder_h = 0.8;     // housing shoulder lip radial height (≤ outer-race land)
brg_spacing    = 14;      // centre-to-centre between bearings (moment arm)

// Motor (placeholder ≈ 4108-class hollow gimbal motor)
mot_d        = 46;    // rotor bell OD
mot_h        = 22;    // total height stator-face → rotor-face
mot_bore_d   = 10.6;  // hollow shaft clear bore
stator_pcd   = 25;  stator_screw = 2.5;  stator_n = 4;
rotor_pcd    = 19;  rotor_screw  = 2.5;  rotor_n  = 4;
mot_air_gap  = 0.6;   // clearance rotor bell ↔ base

// Slip ring capsule (Ø6.5 miniature, e.g. 6-12 wire)
sr_d        = 6.5;
sr_len      = 17.0;  // body length excl. leads
sr_clear    = 0.2;   // diametral clearance in channel
sr_wire_bundle = 3.2;// lead bundle dia (both ends)

// Yoke
yoke_span   = 70;     // inner width between arms (payload/tilt motor)
arm_t       = 5;      // arm thickness
arm_h       = 55;     // arm height above yoke plate
arm_w       = 22;     // arm width
plate_t     = 5;
tilt_bore_d = 10.6;   // tilt-axis hollow bore through arms
tilt_axis_z = 42;     // above plate top
cable_win_w = 4;      // side window for slip-ring rotor leads

// Base
base_wall   = 2.5;
base_floor  = 3;
grub        = 2;      // M2 grub for slip-ring stator clamp
mount_pcd   = 50;  mount_screw = 3;  mount_n = 4;

// ---------------- DERIVED ----------------
spindle_od   = brg_id + shaft_fit;
spindle_id   = sr_d + sr_clear + hole_comp;
spindle_wall = (spindle_od - spindle_id)/2;
seat_d       = brg_od + seat_od_fit + hole_comp;
brg_stack_h  = brg_spacing + brg_w;
housing_od   = max(brg_od + 2*base_wall*2, stator_pcd + stator_screw*3);
base_h       = base_floor + brg_stack_h + 2;   // +2 top shoulder
spindle_len  = mot_h + mot_air_gap + base_h - base_floor + 0.5;

// ---------------- SANITY CHECKS ----------------
assert(spindle_wall >= (fdm ? 1.4 : 0.8),
       str("Spindle wall too thin: ", spindle_wall));
assert(spindle_od + 0.3 <= mot_bore_d,
       str("Spindle (", spindle_od, ") won't pass motor bore (", mot_bore_d, ")"));
assert(sr_len < brg_stack_h + base_h,
       "Slip ring longer than channel — extend base");
assert(brg_shoulder_h < (brg_od-brg_id)/4,
       "Shoulder touches inner race/seal — reduce");
echo(spindle_wall=spindle_wall, housing_od=housing_od, base_h=base_h);

// =====================================================================
//  MODULES
// =====================================================================
module bolt_circle(pcd, n, d, h, z=0, rot=45) {
  for (i=[0:n-1]) rotate([0,0,rot+i*360/n])
    translate([pcd/2,0,z]) cylinder(d=d+hole_comp, h=h);
}

// Bearing seat: pocket with bottom shoulder; `from_top` = open side up
module bearing_seat(z0) {
  translate([0,0,z0]) {
    cylinder(d=seat_d, h=brg_w + 0.01);
    // lead-in chamfer for press
    translate([0,0,brg_w-0.01]) cylinder(d1=seat_d, d2=seat_d+0.8, h=0.4);
  }
  // shoulder relief through-bore (clears inner race)
  translate([0,0,z0-brg_shoulder_h-0.01])
    cylinder(d=brg_od - 2*brg_shoulder_h, h=brg_shoulder_h+0.02);
}

// ----------------------------- BASE ----------------------------------
module base() {
  difference() {
    union() {
      cylinder(d=housing_od, h=base_h);
      // mounting flange
      cylinder(d=mount_pcd + mount_screw*3, h=base_floor);
      // grub-screw boss
      translate([0,0,base_floor+sr_len*0.3])
        rotate([0,90,0]) cylinder(d=6, h=housing_od/2+2);
    }
    // central through-channel below bearings (slip-ring stator side)
    translate([0,0,-1]) cylinder(d=spindle_id, h=base_h+2);
    // inter-bearing cavity (clears spindle + grease)
    translate([0,0,base_floor]) cylinder(d=brg_od-2*brg_shoulder_h, h=brg_stack_h+5);
    // lower bearing seat (shoulder below)
    bearing_seat(base_floor + brg_shoulder_h);
    // upper bearing seat (open at top; inserted from top)
    translate([0,0,base_floor + brg_shoulder_h + brg_spacing])
      cylinder(d=seat_d, h=base_h);
    // spacer ring sits between them -> keeps outer races apart (preload via yoke nut)
    // slip-ring stator wire exit slot (radial, through floor)
    translate([0,-sr_wire_bundle/2,-1]) cube([housing_od, sr_wire_bundle+hole_comp, base_floor+1.5]);
    // grub screw (M2 tap drill 1.6)
    translate([0,0,base_floor+sr_len*0.3]) rotate([0,90,0])
      cylinder(d=1.6, h=housing_od);
    // motor stator screws (from top, into base)
    bolt_circle(stator_pcd, stator_n, stator_screw*0.8, 8, base_h-8); // tap drill
    // mount holes
    bolt_circle(mount_pcd, mount_n, mount_screw, base_floor+2, -1, 0);
  }
}

// ----------------------------- YOKE ----------------------------------
module yoke() {
  plate_l = yoke_span + 2*arm_t;
  difference() {
    union() {
      // plate
      translate([-plate_l/2,-arm_w/2,0]) cube([plate_l, arm_w, plate_t]);
      hull() { cylinder(d=mot_d*0.7, h=plate_t);
               translate([-plate_l/2,-arm_w/2,0]) cube([plate_l,arm_w,plate_t]); }
      // arms with gussets
      for (s=[-1,1]) translate([s*(yoke_span/2+arm_t/2),0,0]) {
        translate([-arm_t/2,-arm_w/2,0]) cube([arm_t, arm_w, plate_t+arm_h]);
        // tilt boss
        translate([0,0,plate_t+tilt_axis_z]) rotate([0,90,0])
          cylinder(d=arm_w, h=arm_t, center=true);
        // gusset
        hull() {
          translate([-arm_t/2,-arm_w/2,plate_t]) cube([arm_t,arm_w,12]);
          translate([-s*8-arm_t/2,-arm_w/2,plate_t]) cube([arm_t,arm_w,0.1]);
        }
      }
      // spindle (down, -Z)
      translate([0,0,-spindle_len]) cylinder(d=spindle_od, h=spindle_len+0.01);
      // spindle root fillet
      translate([0,0,-2]) cylinder(d1=spindle_od, d2=spindle_od+4, h=2);
    }
    // slip-ring channel through spindle
    translate([0,0,-spindle_len-1]) cylinder(d=spindle_id, h=spindle_len+plate_t+2);
    // rotor-lead side window into plate cable groove
    translate([-cable_win_w/2, 0, -1]) cube([cable_win_w, arm_w, plate_t+2]);
    // cable channel along plate to arm (tilt motor feed)
    translate([0,-cable_win_w/2,plate_t-2]) cube([yoke_span/2, cable_win_w, 3]);
    // tilt bores
    for (s=[-1,1]) translate([s*(yoke_span/2+arm_t/2),0,plate_t+tilt_axis_z])
      rotate([0,90,0]) cylinder(d=tilt_bore_d+hole_comp, h=arm_t+2, center=true);
    // rotor screws (counterbored from top)
    bolt_circle(rotor_pcd, rotor_n, rotor_screw, plate_t+2, -1);
    bolt_circle(rotor_pcd, rotor_n, rotor_screw*1.9, 3, plate_t-2);
    // spindle tip: retaining-ring groove (DIN 471, 10