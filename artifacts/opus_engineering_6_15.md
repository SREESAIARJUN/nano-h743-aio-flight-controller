# Cycle 6: Pan-Axis Yoke, Bearing Seats, and 6.5 mm Slip-Ring Channel

I don't have the Cycle 1–5 data, so this design rests on stated assumptions. Every part-specific dimension is a parameter in the first block of the OpenSCAD file. **Measure your actual motor, bearing, and slip ring and edit those values before cutting metal.**

---

## 1. Architecture and Assumptions

```
            TILT AXIS (Y)
   Arm A ─────────────────── Arm B
  [tilt motor seat]     [6700ZZ idler bearing seat]
     │                         │
     │  ← internal wire duct → │
     └──────────┬──────────────┘
           PAN HUB (Z axis)
     ┌──────────┴──────────┐
     │ 6.5 mm slip-ring     │  capsule body rotates WITH yoke
     │ capsule pocket       │  (anti-rotation flat + M2 set screw)
     ├──────────────────────┤
     │ pan motor rotor      │  bolts to hub underside, pilot-located
     │ (hollow shaft)       │  slip-ring rotor wires exit DOWN through
     └──────────────────────┘  the motor's hollow shaft to the fixed base
```

| Item | Assumed default | Parameter |
|---|---|---|
| Pan motor | GBM2804H-class hollow-shaft gimbal motor | `pan_motor_*` |
| Rotor bolt pattern | 4× M2.5 on Ø12 PCD | `pan_motor_pcd`, `pan_motor_screw` |
| Hollow shaft bore | Ø4.0 (wire pass) | `pan_shaft_bore` |
| Slip ring | 6.5 mm OD capsule, 15.5 mm body, 6–12 wires | `sr_*` |
| Tilt idler bearing | 6700ZZ (10×15×4) | `brg_*` |
| Tilt motor | Same 2804 class, stator side bolted to arm A | `tilt_motor_*` |
| Material | 6061-T6 (CNC) or PETG/CF-nylon (FDM) | `process` switch |

**Why the capsule body is in the yoke.** A 6.5 mm capsule won't pass through a 2804 hollow shaft, but its 2–3 mm rotor wire bundle will. Mounting the body in the yoke and grounding the wire bundle in the base keeps the stack short and coaxial.

---

## 2. Fit and Tolerance Strategy

| Feature | Nominal | CNC (6061) | FDM (0.4 nozzle) | Fit intent |
|---|---|---|---|---|
| 6700 bearing seat Ø15 | 15.000 | 15.000 −0.008/−0.018 (N7/M6) | 15.00 + `fdm_hole_comp` (≈+0.15) | Light press / stationary outer ring |
| Bearing seat depth | 4.0 | 4.00 +0.05 | 4.10 | Bearing flush + shoulder retention |
| Bearing shoulder ID | — | ≤ 12.0 (clear inner ring, contacts outer only) | same | Outer race abutment |
| Slip-ring pocket Ø6.5 | 6.5 | 6.52 +0.02 (slip fit) | 6.5 + 0.20 | Hand insertion, no crushing |
| Motor pilot recess | rotor Ø | +0.05 | +0.20 | Concentricity datum |
| Wire duct | ≥ 3.5 × 2.5 | — | — | 12× AWG32 bundle plus margin |

Concentricity is critical: the slip-ring pocket and motor pilot must be machined in the **same setup** (Op 2 below), with a TIR target of ≤ 0.03 mm to the pan axis.

---

## 3. OpenSCAD Source (`pan_yoke_c6.scad`)

```openscad
// =====================================================================
//  Pan-Axis Yoke — Cycle 6
//  Hollow-shaft BLDC pan interface, tilt bearing seats, 6.5mm slip ring
//  Origin: pan axis = Z, motor rotor mating face at Z=0, tilt axis = Y
// =====================================================================
$fn = $preview ? 64 : 160;

/* ---------- PROCESS ---------- */
process        = "FDM";      // "FDM" or "CNC"
fdm_hole_comp  = 0.15;       // radial-equivalent diameter add for printed holes
function H(d) = (process == "FDM") ? d + fdm_hole_comp : d;   // hole comp

/* ---------- PAN MOTOR (hollow shaft) — VERIFY ---------- */
pan_motor_bell_d   = 33.0;   // rotor bell OD for pilot recess
pan_motor_pilot_h  = 1.0;    // pilot recess depth
pan_motor_pcd      = 12.0;   // rotor bolt PCD
pan_motor_n        = 4;
pan_motor_screw    = 2.5;    // M2.5
pan_motor_angle0   = 45;     // bolt pattern clocking
pan_shaft_bore     = 4.0;    // hollow-shaft wire passage

/* ---------- SLIP RING CAPSULE — VERIFY ---------- */
sr_d           = 6.5;
sr_len         = 15.5;       // body length
sr_clear       = (process == "FDM") ? 0.20 : 0.02;
sr_flat_depth  = 0.4;        // anti-rotation flat (pairs with flat filed on capsule or set screw)
sr_wire_exit_d = 3.2;        // rotor bundle exit (down, through motor shaft)
sr_setscrew    = 2.0;        // M2 radial set screw (nylon-tip recommended)

/* ---------- HUB ---------- */
hub_d     = 40.0;
hub_h     = sr_len + 4.0;    // capsule fully enclosed + floor
hub_floor = 2.0;             // material under capsule, above motor face

/* ---------- YOKE ARMS ---------- */
arm_span_inner = 52.0;       // clear width between arms (payload envelope)
arm_t          = 6.0;        // arm thickness (along Y)
arm_w          = 22.0;       // arm width (along X)
tilt_axis_z    = 55.0;       // tilt axis height above Z=0
arm_boss_d     = 26.0;       // boss around tilt seats
base_bridge_h  = 6.0;        // crossbar thickness joining arms at hub

/* ---------- TILT IDLER BEARING: 6700ZZ ---------- */
brg_od    = 15.0;
brg_id    = 10.0;
brg_w     = 4.0;
brg_seat_extra = 0.10;       // depth margin
brg_shoulder_d = 12.5;       // > brg_id+inner ring land; contacts outer race only
brg_press = (process == "CNC") ? -0.013 : fdm_hole_comp; // N7-ish vs FDM comp

/* ---------- TILT MOTOR (stator side on Arm A) — VERIFY ---------- */
tilt_motor_pcd    = 19.0;
tilt_motor_n      = 4;
tilt_motor_screw  = 2.5;
tilt_motor_center = 8.0;     // centre clearance (stator boss / wires)

/* ---------- WIRE DUCT ---------- */
duct_w = 3.6;
duct_h = 2.6;

/* ---------- DERIVED ---------- */
arm_y = arm_span_inner/2 + arm_t/2;          // arm centreline Y
assert(sr_d + 2*sr_clear + 3.0 < hub_d,       "Hub too thin around slip ring");
assert(brg_od + 4.0 <= arm_boss_d,            "Bearing boss wall < 2 mm");
assert(brg_w + brg_seat_extra < arm_t,        "Arm too thin for bearing + shoulder");
assert(pan_motor_pcd/2 + pan_motor_screw > (sr_d/2 + sr_clear + 0.8),
       "Motor bolts intersect slip-ring pocket");
assert(sr_wire_exit_d <= pan_shaft_bore + 0.01, "Wire exit larger than motor hollow shaft");

/* =================== MODULES =================== */

module bolt_circle(pcd, n, a0=0) {
    for (i=[0:n-1]) rotate([0,0,a0 + i*360/n]) translate([pcd/2,0,0]) children();
}

module counterbored(d, head_d, head_h, len) {
    // hole along -Z from top, cbore at top
    translate([0,0,-len]) cylinder(d=H(d+0.2), h=len+0.01);
    translate([0,0,-head_h]) cylinder(d=H(head_d), h=head_h+0.01);
}

// ---- Slip-ring channel: pocket + flat + set screw + exit ----
module slipring_channel() {
    pd = sr_d + 2*sr_clear;
    // capsule pocket opens upward (wires to payload go up/into duct)
    difference() {
        translate([0,0,hub_floor]) cylinder(d=pd, h=sr_len + 0.5);
        // anti-rotation flat (material LEFT in pocket)
        translate([pd/2 - sr_flat_depth, -pd/2, hub_floor-0.1])
            cube([pd, pd, sr_len + 1]);
    }
    // top lead-in chamfer
    translate([0,0,hub_h-0.6]) cylinder(d1=pd, d2=pd+1.2, h=0.61);
    // rotor bundle exit through floor -> motor hollow shaft
    translate([0,0,-1]) cylinder(d=H(sr_wire_exit_d), h=hub_floor+2);
    // radial set screw opposite the flat (tap M2 or heat-set insert)
    translate([0,0,hub_floor + sr_len*0.6]) rotate([0,-90,0])
        cylinder(d=H(sr_setscrew*0.8), h=hub_d);
}

// ---- Pan motor interface on hub underside ----
module pan_motor_interface() {
    // pilot recess for rotor bell (concentricity datum)
    translate([0,0,-0.01]) cylinder(d=H(pan_motor_bell_d)+ (process=="FDM"?0.05:0.05),
                                    h=pan_motor_pilot_h+0.01);
    // rotor bolts, counterbored from hub top OUTSIDE capsule
    bolt_circle(pan_motor_pcd, pan_motor_n, pan_motor_angle0)
        translate([0,0,hub_h]) counterbored(pan_motor_screw, 4.6, hub_h-hub_floor-1, hub_h+1);
}

// ---- Tilt idler bearing seat (bore along Y, opening INWARD) ----
module bearing_seat() {
    seat_d = brg_od + brg_press;
    // seat from inner face
    translate([0,-0.01,0]) rotate([-90,0,0]) cylinder(d=seat_d, h=brg_w + brg_seat_extra + 0.01);
    // entry chamfer 0.3x45
    translate([0,-0.01,0]) rotate([-90,0,0]) cylinder(d1=seat_d+0.6, d2=seat_d, h=0.3);
    // shoulder through-bore (outer race abutment only)
    rotate([-90,0,0]) cylinder(d=H(brg_shoulder_d), h=arm_t+1);
    // extraction notch for bearing removal
    translate([seat_d/2-1.0, 0, -1.0])