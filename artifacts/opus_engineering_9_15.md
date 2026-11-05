# Cycle 9: Pan-Axis Yoke, Hollow-Shaft BLDC Bearing Seats, and 6.5 mm Slip-Ring Channel

I don't have the outputs from Cycles 1–8, so every dimension that depends on them is a **named parameter**. Where I had to choose a value, it's marked `// ASSUME`. Replace those with your locked values from earlier cycles. The geometry recomputes, and `assert()` checks catch interferences.

---

## 1. Architecture (section through the pan axis)

```
            ┌──────────── tilt motor mount ────────────┐
            │  arm                                arm  │
            │   ║                                   ║  │
            └───╨────────── YOKE DECK ──────────────╨──┘   ← bolts to motor rotor (M2 × 4)
                  │ skirt │  ◄─ slip-ring rotor     │ skirt │
          ┌───────┤ (rot) ├─────────────────────────┤ (rot) ├───────┐
          │ 6808  ╞═══════╡   BLDC (hollow shaft)   ╞═══════╡ 6808  │  ← support bearing
          │ seat  │       │   ┌─────────────┐       │       │ seat  │     outer: housing bore
          │       │       │   │   stator    │       │       │       │     inner: yoke skirt
          │       └───────┘   └──┬─────┬────┘       └───────┘       │
          │  HOUSING CUP         │ SR  │  ◄─ 6.5 mm channel         │
          │                      │body │                            │
          └──────────────────────┤flng ├──── wire exit slot ────────┘
                                 └─────┘ anti-rotation tab
```

**Load path:** The yoke deck bolts to the motor rotor, so torque goes in directly. A thin-wall **6808 (40×52×7)** bearing carries the yoke skirt. It takes the overturning moment from the tilt/roll stages, so the motor's own small bearings only see torque. The slip ring runs on-axis through the hollow shaft. Its fixed flange is captured in the housing floor, and its rotating side is keyed to the yoke deck.

---

## 2. OpenSCAD source (`pan_yoke_c9.scad`)

```openscad
// =====================================================================
//  Pan-axis yoke + BLDC bearing seats + 6.5 mm slip-ring channel
//  Cycle 9 — fully parametric. Units: mm.
// =====================================================================
$fn = 128;

// ---------------- Manufacturing mode ----------------
PROCESS = "FDM";            // "FDM" | "CNC"
// Diametral compensation added to bores / subtracted from pins
bore_comp  = (PROCESS == "FDM") ? 0.15 : 0.000;
press_intf = (PROCESS == "FDM") ? 0.10 : 0.015;   // outer-race interference (dia)
slip_clr   = (PROCESS == "FDM") ? 0.25 : 0.020;   // inner-race slip clearance (dia)

// ---------------- Motor (hollow shaft) ----------------
mot_od         = 34.0;   // ASSUME GBM2804H-class
mot_h          = 15.0;   // total incl. rotor cap
mot_hollow_id  = 7.0;    // ASSUME — must pass 6.5 slip ring
stator_bc      = 19.0;   stator_n = 4; stator_hole = 3.2;  // M3
rotor_bc       = 12.0;   rotor_n  = 4; rotor_hole  = 2.2;  // M2
rotor_boss_d   = 16.0;   // raised boss on rotor face (clear it)
rotor_boss_h   = 0.6;

// ---------------- Support bearing (6808) ----------------
brg_id = 40.0;  brg_od = 52.0;  brg_w = 7.0;
brg_da_min = 42.0;   // ISO shoulder: shaft-side min abutment dia
brg_Da_max = 50.0;   // ISO shoulder: housing-side max abutment dia
brg_ra     = 0.3;    // corner radius -> undercut size

// ---------------- Slip ring (6.5 mm capsule) ----------------
sr_d         = 6.5;
sr_len       = 17.0;   // ASSUME body length (6/12-wire capsule)
sr_flange_d  = 16.0;   // ASSUME
sr_flange_t  = 1.5;
sr_flange_bc = 12.0;   sr_flange_hole = 1.8;   // M1.6 ×2
sr_wire_bundle_d = 4.0;    // rotor-side bundle (6×AWG32 + sleeve)
sr_stator_wire_slot_w = 3.0;
sr_clr       = 0.30;   // radial clearance in channel (always loose)

// ---------------- Housing ----------------
hsg_wall    = 3.0;
hsg_floor   = 4.0;
hsg_od      = brg_od + 2*hsg_wall + 2*4;   // +4 for clamp-ring screw land
clamp_bc    = brg_od + 2*hsg_wall + 2.5;
clamp_n     = 6; clamp_hole = 2.2;  clamp_t = 2.0;
base_mount_bc = hsg_od + 10; base_mount_n = 4; base_mount_hole = 3.4;

// ---------------- Yoke ----------------
skirt_wall   = (brg_id - mot_od)/2 - 1.0;  // leaves 1 mm radial to motor
deck_t       = 4.0;
yoke_span    = 70.0;   // ASSUME inner distance between arms (tilt payload)
arm_t        = 5.0;
arm_w        = 22.0;
arm_h        = 55.0;   // ASSUME deck-to-tilt-axis
tilt_mot_bc  = 19.0;  tilt_mot_hole = 3.2; tilt_mot_n = 4;
tilt_hollow  = 8.0;

// ---------------- Derived + sanity checks ----------------
gap_motor_brg   = 1.0;                         // vertical gap
skirt_len       = mot_h + gap_motor_brg - rotor_boss_h;
brg_z0          = hsg_floor + mot_h - brg_w;   // bearing top flush with rotor face
hsg_h           = brg_z0 + brg_w;

assert(mot_hollow_id >= sr_d + 2*sr_clr,
       "Slip ring will not pass through motor hollow shaft");
assert((brg_id - 2*skirt_wall) > mot_od + 1.5,
       "Yoke skirt interferes with motor rotor");
assert(skirt_wall >= 1.6, "Skirt wall too thin — choose a larger bearing");
assert(sr_flange_d < stator_bc - stator_hole,
       "Slip-ring flange collides with stator screws");

// =====================================================================
//  Helpers
// =====================================================================
module bolt_circle(bc, n, d, h, a0=45)
    for (i=[0:n-1]) rotate(a0 + i*360/n)
        translate([bc/2,0,-0.01]) cylinder(d=d, h=h+0.02);

module chamfer_ring(d, c) cylinder(d1=d+2*c, d2=d, h=c);

// ISO-style bearing pocket: press-fit bore, entry chamfer,
// relief undercut at the shoulder, shoulder land ≤ Da_max
module bearing_seat_housing(z) {
    bore = brg_od - press_intf + bore_comp;
    translate([0,0,z]) {
        cylinder(d=bore, h=brg_w + 0.5);               // seat
        translate([0,0,brg_w+0.5-0.01]) cylinder(d=bore+1, h=20); // open above
        translate([0,0,brg_w-0.6]) chamfer_ring(bore, 0.6);       // lead-in
        // shoulder undercut (corner relief so race seats flat)
        translate([0,0,-0.4])
            difference(){ cylinder(d=bore, h=0.41);
                          cylinder(d=brg_Da_max, h=0.41); }
        // through to motor cavity below shoulder
        translate([0,0,-50]) cylinder(d=brg_Da_max, h=50.01);
    }
}

// =====================================================================
//  6.5 mm slip-ring channel (subtracted from housing floor)
// =====================================================================
module slip_ring_channel() {
    // body channel (loose; capsule must never be loaded radially)
    translate([0,0,-sr_len]) cylinder(d=sr_d + 2*sr_clr + bore_comp, h=sr_len + hsg_floor + 1);
    // flange counterbore from underside (body hangs below housing)
    translate([0,0,-0.01]) cylinder(d=sr_flange_d + 0.4 + bore_comp, h=sr_flange_t + 0.2);
    // M1.6 flange screws (self-tap FDM / tap CNC)
    bolt_circle(sr_flange_bc, 2, sr_flange_hole, hsg_floor, a0=0);
    // anti-rotation tab slot (redundant to screws; survives screw loosening)
    translate([sr_flange_d/2 - 0.5, -1.0, -0.01]) cube([2.0, 2.0, sr_flange_t + 0.2]);
    // stator-wire exit: radial slot in floor underside to housing edge
    rotate(90) translate([0, -sr_stator_wire_slot_w/2, -0.01])
        cube([hsg_od, sr_stator_wire_slot_w, 2.0]);
}

// =====================================================================
//  HOUSING (stationary)
// =====================================================================
module housing() {
    difference() {
        union() {
            cylinder(d=hsg_od, h=hsg_h);
            // mounting ears
            for (i=[0:base_mount_n-1]) rotate(45 + i*90)
                hull(){ cylinder(d=12, h=hsg_floor);
                        translate([base_mount_bc/2,0,0]) cylinder(d=9, h=hsg_floor); }
        }
        // motor cavity
        translate([0,0,hsg_floor]) cylinder(d=mot_od + 1.0, h=hsg_h);
        // bearing pocket
        bearing_seat_housing(brg_z0);
        // stator screws (counterbored from below)
        bolt_circle(stator_bc, stator_n, stator_hole, hsg_floor);
        for (i=[0:stator_n-1]) rotate(45+i*360/stator_n)
            translate([stator_bc/2,0,-0.01]) cylinder(d=5.8, h=1.5);
        // motor phase-lead exit (radial notch in cavity wall)
        rotate(180) translate([mot_od/2 - 1, -2.5, hsg_floor])
            cube([hsg_wall + 10, 5, 3]);
        // clamp-ring screw holes
        bolt_circle(clamp_bc, clamp_n, clamp_hole*0.85, 6, a0=