# Pan-Axis Yoke: Parametric Geometry (Cycle 27)

I don't have the earlier cycles' parameter set, so every value below is a named, documented default. Swap in your locked values from previous cycles; asserts will catch fits that break.

---

## 1. Architecture

```
            ┌──────── tilt bearing seat ────────┐
            │  arm                         arm  │
            │                                   │
            └──────┐   ┌─ slip-ring rotor ┐  ┌──┘
                   │   │   driver slot    │  │      ← YOKE (rotates)
          6700ZZ ▓▓│   │                  │  │▓▓    upper bearing
          spacer   │   │  ║ 6.5 capsule ║  │  │      integral shoulder
          6700ZZ ▓▓│   │  ║  (fixed)    ║  │  │▓▓    lower bearing
                   └───┤  ║             ║  ├──┘      ← rotor flange (M-bolts to motor bell)
              ═══ BLDC hollow-shaft motor (stator fixed) ═══
                       │  fixed spindle  │           ← SPINDLE (fixed, bonded/bolted to base)
              ─────────┴─────────────────┴────────── BASE
                          wire exit slot ↓
```

**Load path:** The yoke rides on two preloaded 6700ZZ bearings on a fixed hollow spindle. The motor supplies only torque, so its small internal bearings never carry payload moment, and pan stiffness comes from the 10 mm spindle and the bearing span.

**Slip ring:** The 6.5 mm capsule body is keyed fixed inside the spindle. Its rotor and wire bundle exit the top and are driven by a slot in the yoke cap, so cables stay coaxial with zero wrap.

**Hard requirement:** the motor's hollow bore must be at least the spindle OD plus clearance. An assert enforces this.

---

## 2. OpenSCAD Source

```openscad
// ============================================================
//  PAN-AXIS YOKE  —  Cycle 27
//  Units: mm.  All geometry driven from the parameter block.
// ============================================================
$fn = 96;
PART = "assembly";   // "yoke" | "spindle" | "cap" | "assembly"

// ---------- MANUFACTURING ----------
process      = "fdm";     // "fdm" | "cnc"
// Bearing seat diametral fit (+ = clearance, - = interference)
seat_fit     = (process == "cnc") ? -0.010 : 0.12;  // FDM: print then ream/press
hole_comp    = (process == "fdm") ? 0.20 : 0.05;    // general hole growth
eps          = 0.01;

// ---------- PAN BEARINGS (6700ZZ: 10x15x4) ----------
brg_id = 10;  brg_od = 15;  brg_w = 4;
brg_spacing   = 6;      // axial gap between bearings (= shoulder height)
shoulder_lip  = 1.0;    // radial lip retaining outer race (must clear inner race/shields)

// ---------- FIXED SPINDLE ----------
spindle_od    = brg_id;         // nominal; turn/ream to bearing fit
spindle_len   = 34;             // base face -> top of upper bearing
spindle_flange_d = 26;  spindle_flange_t = 3;

// ---------- SLIP RING (miniature 6.5 mm capsule) ----------
sr_od         = 6.5;
sr_body_len   = 14.0;   // VERIFY against datasheet
sr_clear      = 0.15;   // radial clearance in channel
sr_key_w      = 1.2;    // anti-rotation key (match capsule flat/tab)
sr_key_d      = 0.6;
sr_rotor_tab_w = 2.0;   // rotor driver tab width
wire_slot_w   = 3.0;    // base-side wire exit

// ---------- MOTOR (hollow-shaft gimbal BLDC) ----------
motor_bell_d  = 35;     // e.g. 4108-class; VERIFY
motor_bore    = 11.0;   // hollow shaft ID; VERIFY
motor_h       = 20;
rotor_pcd     = 19.0;   // rotor-side bolt circle; VERIFY
rotor_bolts   = 4;
rotor_bolt_d  = 2.5;    // M2.5
rotor_pilot_d = 12.0;   // centring boss on motor bell (0 = none)
rotor_pilot_h = 1.0;

// ---------- YOKE ----------
hub_wall      = 3.0;
hub_od        = brg_od + 2*hub_wall;
flange_t      = 3.5;
payload_w     = 60;     // inner span between arms
arm_t         = 5;
arm_depth     = 22;
arm_h         = 55;     // flange top -> tilt axis
tilt_brg_od   = 9;  tilt_brg_w = 4;   // 684ZZ default
fillet_r      = 4;

// ---------- DERIVED ----------
hub_h     = flange_t + 2*brg_w + brg_spacing + 1.5;
sr_ch_d   = sr_od + 2*sr_clear + hole_comp;
spindle_id = sr_ch_d;
spindle_wall = (spindle_od - spindle_id)/2;

// ---------- SANITY ----------
assert(spindle_wall >= 1.2,
       str("Spindle wall too thin: ", spindle_wall, " mm"));
assert(motor_bore >= spindle_od + 0.4,
       "Motor hollow bore cannot clear spindle — choose larger-bore motor");
assert(brg_od - 2*shoulder_lip > brg_id + 1.5,
       "Shoulder lip will rub inner race");
assert(rotor_pcd/2 - rotor_bolt_d > brg_od/2 + 0.8,
       "Rotor bolts break into bearing seat");

// ============================================================
module bearing_seat_cut(od, w) {
    // Seat + 0.3 mm entry chamfer for press-in alignment
    cylinder(d = od + seat_fit, h = w + eps);
    translate([0,0,-0.3]) cylinder(d1 = od + seat_fit + 0.6,
                                   d2 = od + seat_fit, h = 0.3 + eps);
}

module rotor_bolt_pattern(d, h) {
    for (i = [0:rotor_bolts-1])
        rotate(i*360/rotor_bolts + 45)
            translate([rotor_pcd/2, 0, 0]) cylinder(d = d, h = h);
}

module yoke() {
    difference() {
        union() {
            // Rotor flange
            cylinder(d = max(motor_bell_d, rotor_pcd + 8), h = flange_t);
            // Bearing hub
            cylinder(d = hub_od, h = hub_h);
            // Crossbar + arms
            arm_span = payload_w + 2*arm_t;
            translate([-arm_span/2, -arm_depth/2, 0])
                cube([arm_span, arm_depth, flange_t]);
            for (s = [-1, 1])
                translate([s*(payload_w/2 + arm_t/2) - arm_t/2, -arm_depth/2, 0])
                    hull() {
                        cube([arm_t, arm_depth, flange_t]);
                        translate([0, arm_depth/2, arm_h])
                            rotate([0,90,0]) cylinder(d = arm_depth, h = arm_t);
                    }
        }
        // --- Lower bearing seat (enters from bottom, motor side)
        translate([0,0,flange_t]) bearing_seat_cut(brg_od, brg_w);
        // Through-bore under lower bearing (clears spindle)
        translate([0,0,-eps]) cylinder(d = brg_id + 1.5, h = hub_h + 1);
        // Shoulder bore between bearings
        translate([0,0,flange_t + brg_w])
            cylinder(d = brg_od - 2*shoulder_lip, h = brg_spacing + eps);
        // --- Upper bearing seat (enters from top)
        translate([0,0,flange_t + brg_w + brg_spacing])
            bearing_seat_cut(brg_od, brg_w + 2);
        // Motor bolts + counterbore from top side of flange
        translate([0,0,-eps]) rotor_bolt_pattern(rotor_bolt_d + hole_comp, flange_t + 1);
        // Motor pilot recess
        if (rotor_pilot_d > 0)
            translate([0,0,-eps]) cylinder(d = rotor_pilot_d + hole_comp,
                                           h = rotor_pilot_h + eps);
        // Tilt-axis bearing seats (outer faces of arms)
        for (s = [-1, 1])
            translate([s*(payload_w/2 + arm_t + eps), 0, arm_h])
                rotate([0, s*-90, 0]) {
                    bearing_seat_cut(tilt_brg_od, tilt_brg_w);
                    cylinder(d = tilt_brg_od - 2, h = arm_t + 1); // through
                }
        // Cable pass from hub up to tilt motor
        translate([payload_w/2 - 1, -2, flange_t])
            cube([arm_t + 2, 4, 20]);
    }
}

module spindle() {
    difference() {
        union() {
            cylinder(d = spindle_flange_d, h = spindle_flange_t);
            cylinder(d = spindle_od, h = spindle_len);
        }
        // Slip-ring channel
        translate([0,0,-eps]) cylinder(d = spindle_id, h = spindle_len + 1);
        // Anti-rotation key groove (top end, capsule length)
        translate([spindle_id