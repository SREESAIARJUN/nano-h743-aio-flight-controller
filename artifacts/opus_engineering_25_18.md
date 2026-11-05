# Pan-Axis Yoke, Bearing Seats & 6.5 mm Slip-Ring Channel: Parametric Geometry

I don't have the earlier cycles' state, so I've assumed the hardware below. Everything is a top-level parameter, so you can substitute your measured values without restructuring.

| Item | Assumed part | Key dims (verify with calipers) |
|---|---|---|
| Pan motor | GBM2804H-100T hollow-shaft gimbal motor | Bell Ø34 × 14 h, stator mount 4×M3 on Ø19, rotor 4×M2.5 on Ø12, bore Ø5.5 |
| Auxiliary bearing | 6808-2RS thin-section | 40 × 52 × 7 |
| Slip ring | 6.5 mm capsule, 6-wire | Body Ø6.5 × 17, flange Ø15 × 1.2, 2× M1.6 on Ø11 |
| Fasteners | M3 / M2.5 socket head | — |

## 1. Architecture

```
            ┌──── yoke arms (tilt axis) ────┐
            │                               │
            └──┐  hub plate (bolts to bell) ┌┘
       skirt ──┤▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓├── skirt
   outer race →[6808]   ┌─ motor bell ─┐   [6808]← outer race
   inner race → boss ║  │   stator     │  ║ boss
   ══════════════════╩══╧══════════════╧══╩═══════ base plate (z=0)
                         ║ slip-ring  ║   ← channel, flange recess,
                         ╚═ capsule ══╝     anti-rotation, wire slot
```

**Load path.** The 6808 bearing sits on a boss that rises from the base and surrounds the motor. The yoke skirt carries the outer race. Tilt-payload moments therefore go through the 6808, not through the motor's tiny internal bearings. The motor only supplies torque.

**Slip ring.** The stator flange is fixed in the base underside. The rotor leads go up through the motor's hollow shaft and are clamped in the yoke hub. A strain-relief boss there drives the slip-ring rotor without loading the motor shaft.

## 2. OpenSCAD source

```openscad
// ===============================================================
//  Pan-axis yoke + bearing seats + 6.5mm slip-ring channel
//  Units: mm. All geometry is driven from the parameter block.
// ===============================================================

/* [Render] */
part = "assembly"; // [assembly, base, yoke, section, fit_coupon]
process = "FDM";   // [FDM, SLA, CNC]
$fn = 128;

/* [Motor - GBM2804H] */
mot_od        = 34.0;   // bell OD
mot_h         = 14.0;   // stator base -> bell top
mot_bore      = 5.5;    // hollow shaft ID
mot_stat_pcd  = 19.0;   // stator mount PCD
mot_stat_d    = 3.0;    // stator screws M3
mot_rot_pcd   = 12.0;   // rotor (bell) mount PCD
mot_rot_d     = 2.5;    // rotor screws M2.5
mot_wire_w    = 6.0;    // phase-lead exit slot width
mot_clr       = 0.6;    // radial air gap bell <-> boss

/* [Bearing - 6808] */
brg_id = 40.0;
brg_od = 52.0;
brg_w  = 7.0;
brg_ir_land = 2.0;      // inner-race shoulder radial height (contact inner race only)
brg_or_land = 2.0;      // outer-race shoulder radial height
brg_chamfer = 0.5;      // lead-in, also clears race corner radius

/* [Slip ring - 6.5mm capsule] */
sr_body_d   = 6.5;
sr_body_l   = 17.0;
sr_fl_d     = 15.0;
sr_fl_t     = 1.2;
sr_fl_pcd   = 11.0;
sr_fl_screw = 1.6;
sr_wire_slot= 3.0;      // stator lead exit slot width
sr_lead_bend_r = 3.0;   // minimum lead bend radius at exit

/* [Base] */
base_t      = 5.0;
base_size   = 70.0;
base_mount_pcd = 58.0;  // 4x M3 to chassis
boss_gap    = 0.5;      // axial gap between bearing top and hub underside

/* [Yoke] */
hub_t       = 5.0;
skirt_wall  = 3.0;
arm_w       = 14.0;
arm_t       = 6.0;
arm_h       = 55.0;     // hub top -> tilt axis
arm_span    = 70.0;     // inside face to inside face
tilt_brg_od = 22.0;     // 608 tilt bearing in arm tip (placeholder)
tilt_brg_w  = 7.0;
strain_boss_d = 10.0;   // slip-ring rotor lead clamp boss

// ---------------- Process-dependent fits (diametral) ----------------
// Calibrate with part="fit_coupon" before the final print.
press_bore = process=="CNC" ? -0.012 : process=="SLA" ? 0.05 : 0.12; // housing bore vs bearing OD (light press / N7-ish)
press_shaft= process=="CNC" ?  0.008 : process=="SLA" ? -0.03 : -0.06; // shaft seat vs bearing ID (k6-ish)
slide      = process=="CNC" ?  0.05  : process=="SLA" ? 0.15 : 0.30;  // slip/locating fit
hole_clr   = process=="CNC" ?  0.20  : 0.35;                            // screw clearance

// Polyhole: compensates for facet chords so printed holes are not undersize
module phole(d, h, center=false)
    cylinder(h=h, r=(d/2)/cos(180/$fn), center=center);

// ---------------- Derived Z stack ----------------
z_brg_bot  = mot_h - brg_w;            // bearing top flush with bell top
z_brg_top  = mot_h;
z_hub_bot  = mot_h + boss_gap;
z_hub_top  = z_hub_bot + hub_t;
boss_id    = mot_od + 2*mot_clr;
assert(boss_id < brg_id - 2.4, "Boss wall < 1.2mm: use 6809 or smaller motor");
assert(mot_bore >= 3.0, "Hollow shaft too small for 6-wire lead bundle");

// ================= BASE =================
module bearing_shaft_seat() {
    // inner-race seat: press OD on top, shoulder below
    seat_d = brg_id + press_shaft;
    difference() {
        union() {
            // shoulder ring (contacts inner race only)
            cylinder(h=z_brg_bot, d=brg_id + 2*brg_ir_land);
            // seat
            translate([0,0,z_brg_bot-0.01])
                cylinder(h=brg_w+0.01 - brg_chamfer, d=seat_d);
            translate([0,0,z_brg_top-brg_chamfer])
                cylinder(h=brg_chamfer, d1=seat_d, d2=seat_d-2*brg_chamfer);
        }
        // motor clearance bore
        translate([0,0,-1]) phole(boss_id, mot_h+2);
        // undercut relief at shoulder corner (bearing corner radius clearance)
        translate([0,0,z_brg_bot-0.4])
            difference() {
                cylinder(h=0.4, d=seat_d+0.8);
                cylinder(h=0.4, d=seat_d-0.4);
            }
    }
}

module slipring_channel() {
    // carved from base underside, coaxial with pan axis
    // 1) flange recess (locating fit on flange OD)
    translate([0,0,-0.01]) phole(sr_fl_d + slide, sr_fl_t + 0.2);
    // 2) body channel (slip fit), runs up to the motor bore
    phole(sr_body_d + slide, base_t + 1);
    // 3) flange screws (tapped M1.6 -> 1.25 pilot, or heat-set)
    for (a=[0,180]) rotate(a) translate([sr_fl_pcd/2,0,0])
        phole(process=="CNC" ? 1.25 : 1.4, 4);
    // 4) stator lead exit slot with bend-radius fillet volume
    hull() {
        translate([sr_fl_d/2 - 1, -sr_wire_slot/2, 0])
            cube([1, sr_wire_slot, sr_fl_t + 0.2]);
        translate([sr_fl_d/2 + sr_lead_bend_r, -sr_wire_slot/2, 0])
            cube([0.01, sr_wire_slot, 0.01]);
    }
    translate([0, -sr_wire_slot/2, 0])
        cube([base_size/2 + 