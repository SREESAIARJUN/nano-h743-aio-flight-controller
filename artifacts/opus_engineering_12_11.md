# Pan-Axis Yoke Assembly: Parametric Geometry (Cycle 12)

I don't have your earlier cycles, so I've built this on explicit assumptions. Every dimension is a named parameter, so you can override any assumption without touching the geometry logic. **Verify all values marked `⚠ VERIFY` against your actual component datasheets before cutting metal or printing.**

---

## 1. Architecture

```
            ┌──────── Tilt-axis bores (arms) ────────┐
            │                                         │
     ┌──────┴──────┐                           ┌──────┴──────┐
     │  Yoke Arm L │                           │  Yoke Arm R │
     │  (cable ch.)│                           │  (cable ch.)│
     └──────┬──────┘                           └──────┬──────┘
            └──────────────┬──────────────────────────┘
                     ┌─────┴─────┐  ← Yoke crossbar / hub
                     │ ROTOR CLAMP│  ← bolts to motor rotor bell
              ═══════╪═══════════╪═══════  ← Upper bearing (6701ZZ)
                     │  hollow   │
                     │  stem Ø12 │  ← slip-ring channel runs through
              ═══════╪═══════════╪═══════  ← Lower bearing (6701ZZ)
                     │  STATOR   │  ← hollow-shaft BLDC stator, fixed to base
                     │ SLIP RING │  ← Ø6.5 capsule, body keyed to base
                     └───────────┘
```

**Load path:** The duplex bearings carry all radial and moment loads from the yoke. The motor only supplies torque, which keeps the motor's own small bearings free of the yoke cantilever moment. This matters for precision: gimbal motor bearings typically have 20–50 µm of play.

**Wire path:** Tilt-axis wires → arm channel → crossbar → down the hollow stem → slip-ring rotor leads. The slip-ring stator leads exit through a radial slot in the base.

---

## 2. Design Assumptions

| Item | Assumed value | Notes |
|---|---|---|
| Motor | Hollow-shaft gimbal BLDC, 2804/3510-class | ⚠ VERIFY bolt pattern and hollow bore |
| Pan bearings | 2× 6701ZZ (12 × 18 × 4 mm) | 12 mm bore passes the Ø6.5 slip ring plus wall |
| Slip ring | Ø6.5 mm capsule, ~6 circuits, with flange | ⚠ VERIFY flange Ø, length, tab |
| Tilt bearings | MR85ZZ (5 × 8 × 2.5 mm) per arm | Optional, set `tilt_bearing=false` for a plain bore |
| Process | `"fdm"`, `"sla"`, or `"cnc"` | Switches fit compensation automatically |

---

## 3. OpenSCAD Source: `pan_yoke_c12.scad`

```openscad
// =====================================================================
//  PAN-AXIS YOKE — Cycle 12
//  Hollow-shaft BLDC + duplex bearing seats + Ø6.5 slip-ring channel
//  Units: mm.  All geometry is driven from the parameter block.
// =====================================================================

/* [Process] */
process = "fdm";            // "fdm" | "sla" | "cnc"

/* [Render] */
part = "assembly";          // "assembly" | "yoke" | "base" | "section"
$fn  = $preview ? 64 : 160;

// ---------------------------------------------------------------------
// FIT COMPENSATION  (diametral, applied to *holes*)
// press  : interference seat for bearing OD
// slide  : locational clearance (bearing ID on stem, slip ring body)
// loose  : wire / free clearance
// ---------------------------------------------------------------------
function fit(kind) =
    process == "cnc" ?
        (kind=="press" ? -0.012 : kind=="slide" ? 0.010 : 0.20) :
    process == "sla" ?
        (kind=="press" ?  0.05  : kind=="slide" ? 0.10  : 0.30) :
    /* fdm */
        (kind=="press" ?  0.10  : kind=="slide" ? 0.20  : 0.40);

// FDM holes print undersize; polyhole-style correction
function hole_d(d, kind) = d + fit(kind);

/* [Pan Bearings — 6701ZZ] */
brg_id   = 12.0;
brg_od   = 18.0;
brg_w    =  4.0;
brg_gap  =  8.0;      // spacer between bearings (wider = more moment stiffness)
brg_shoulder = 1.0;   // radial shoulder height retaining outer race
brg_relief   = 0.4;   // undercut at seat corner (removes printed fillet bind)

/* [Motor — ⚠ VERIFY] */
mot_stator_bc  = 19.0;   // stator mounting bolt circle
mot_stator_n   = 4;
mot_stator_bolt= 2.5;    // M2.5
mot_rotor_bc   = 12.0;   // rotor bell bolt circle
mot_rotor_n    = 4;
mot_rotor_bolt = 2.0;    // M2
mot_od         = 34.0;
mot_h          = 14.0;
mot_bore       = 7.0;    // motor hollow shaft bore

/* [Slip Ring — Ø6.5 capsule, ⚠ VERIFY] */
sr_d        = 6.5;
sr_len      = 17.0;
sr_flange_d = 15.0;
sr_flange_t = 1.2;
sr_flange_bc= 11.0;      // flange screw circle (2× M1.6)
sr_tab_w    = 2.0;       // anti-rotation tab width (rotor side)
sr_lead_d   = 2.8;       // rotor wire bundle diameter

/* [Stem] */
stem_id  = max(hole_d(sr_d, "slide"), mot_bore) + 0.0;
stem_od  = brg_id;                 // bearings ride directly on stem
stem_len = 2*brg_w + brg_gap + 6;

/* [Yoke] */
arm_span      = 70.0;   // inner face-to-face distance between arms
arm_t         =  6.0;
arm_w         = 18.0;
arm_h         = 55.0;   // stem axis to tilt axis
cross_t       =  8.0;
tilt_bore     =  5.0;
tilt_bearing  = true;   // MR85ZZ seat
tilt_brg_od   =  8.0;
tilt_brg_w    =  2.5;
cable_ch_w    =  3.5;
cable_ch_d    =  2.5;
fillet_r      =  3.0;

/* [Base] */
base_h  = 22.0;
base_od = 40.0;

// =====================================================================
// DESIGN-RULE CHECKS
// =====================================================================
assert(stem_id + 2*1.2 <= stem_od,
  str("Stem wall < 1.2 mm: id=", stem_id, " od=", stem_od));
assert(sr_flange_d < mot_stator_bc - mot_stator_bolt,
  "Slip-ring flange collides with stator bolts");
assert(arm_t > tilt_brg_w + 1.5 || !tilt_bearing,
  "Arm too thin for tilt bearing seat");
assert(brg_od + 2*2.5 < mot_od,
  "Bearing housing wall < 2.5 mm within motor footprint");

// =====================================================================
// PRIMITIVES
// =====================================================================
module bearing_seat(od, w, kind="press", relief=brg_relief, shoulder=brg_shoulder) {
    // Pocket with corner relief and a retaining shoulder at the bottom
    d = hole_d(od, kind);
    union() {
        cylinder(d=d, h=w + 0.01);
        // corner relief groove so the race seats fully against the shoulder
        translate([0,0,-relief]) cylinder(d=d - 2*shoulder + 2*relief, h=relief+0.01);
        // chamfer lead-in
        translate([0,0,w-0.5]) cylinder(d1=d, d2=d+1.0, h=0.51);
    }
}

module bolt_circle(bc, n, d, h, phase=45) {
    for (i=[0:n-1]) rotate(phase + i*360/n)
        translate([bc/2,0,0]) cylinder(d=hole_d(d,"loose"), h=h);
}

module rounded_box(size, r) {
    hull() for (x=[r,size.x-r], y=[r,size.y-r])
        translate([x,y,0]) cylinder(r=r, h=size.z);
}

// =====================================================================
// SLIP-RING CHANNEL  (subtracted from base)
// =====================================================================
module slip_ring_channel() {
    // body bore
    cylinder(d=hole_d(sr_d,"slide"), h=sr_len + 1);
    // flange counterbore at bottom (installed from below)
    translate([0,0,-0.01])
        cylinder(d=hole_d(sr_flange_d,"slide"), h=sr_flange_t + 0.2);
    // flange screws
    for (a=[0,180]) rotate(a) translate([sr_flange_bc/2,0,0])
        cylinder(d=1.4, h=6);           // M1.6 tap drill / self-tap
    // stator lead exit slot
    translate([-1.5, 0, -0.01]) cube([3, base_od, 3]);
}

// =====================================================================
// YOKE  (rotates)
// =====================================================================
module stem() {
    difference() {
        union() {
            cylinder(d=stem_od, h=stem_len);
            // bearing spacer collar is a separate part; stem has bearing shoulder:
            translate([0,0,stem_len - 2])
                cylinder(d=brg_id + 2.5, h=2);  // contacts inner race only
        }
        translate([0,0,-1]) cylinder(d=stem_id, h=stem_len+2);
        // slip-ring rotor anti-rotation key
        translate([-hole_d(sr_tab_w,"slide")/2, 0, -1])
            cube([hole_d(sr_tab_w,"slide"), stem_od, 4]);
    }
}

module yoke() {
    span_out = arm_span + 2*arm_t;
    difference() {
        union() {
            // crossbar
            translate([-span_out/2, -arm_w/2, 0])
                rounded_box([span_out, arm_w, cross_t], fillet_r);
            // hub boss mating to rotor bell
            cylinder(d=mot_rotor_bc + mot_rotor_bolt + 6, h=cross_t);
            // arms
            for (s=[-1,1])
                translate([s*(arm_span/2 + arm_t/2) - arm_t/2, -arm_w/2, 0])
                    hull() {
                        cube([arm_t, arm_w, cross_t]);
                        translate([0, arm_w/2, arm_h])
                            rotate([0,90,0]) cylinder(d=arm_w, h=arm_t);
                    }
            // stem hangs below crossbar
            mirror([0,0,1]) stem();
        }
        // central wire passage through hub
        translate([0,0,-stem_len-1]) cylinder(d=stem_id, h=stem_len+cross_t+2);
        // rotor bell bolts (counterbored from top)
        bolt_circle(mot_rotor_bc, mot_rotor_n, mot_rotor_bolt, cross_t+1, 45);
        translate([0,0,cross_t-2])
            bolt_circle(mot_rotor_bc, mot_rotor_n, mot_rotor_bolt+1.8, 3, 45);
        // tilt-axis bores / bearing seats
        for (s=[-1,1]) translate([s*(arm_span/2 + arm_t/2), 0, arm_h]) {
            rotate([0,90,0]) translate([0,0,-arm_t])
                cylinder(d=hole_d(tilt_bore,"loose"), h=2*arm_t);
            if (tilt_bearing)
                rotate([0, s>0 ? -90 : 90, 0]) translate([0,0,-arm_t/2-0.01])
                    bearing_seat(tilt_brg_od, tilt_brg_w, "press", 0.2, 0.6);
        }
        // cable channels: up each arm's outer face, across crossbar top
        for (s=[-1,1]) {
            translate([s*(arm_span/2 + arm_t) - (s>0?cable_ch_d:0), -cable_ch_w/2, cross_t/2])
                cube([cable_ch_d, cable_ch_w, arm_h]);
            translate([0, -cable_ch_w/2, cross_t - cable_ch_d])
                rotate([0,0, s>0?0:180])
                    cube([arm_span/2 + arm_t, cable_ch_w, cable_ch_d+0.01]);
        }
    }
}

// =====================================================================
// BASE  (fixed: houses bearings, stator, slip ring)
// =====================================================================
module base() {
    seat1_z = base_h - brg_w;                 // upper bearing
    seat2_z = seat1_z - brg_gap - brg_w;      // lower bearing
    difference() {
        cylinder(d=base_od, h=base_h);
        // through-bore clearance around stem
        translate([0,0,-1]) cylinder(d=stem_od + 1.0, h=base_h+2);
        // upper bearing seat (open top)
        translate([0,0,seat1_z]) bearing_seat(brg_od, brg_w + 0.01);
        // lower bearing seat (open top: inserted before spacer)
        translate([0,0,seat2_z]) bearing_seat(brg_od, brg_w);
        // spacer bore between seats (outer-race spacer)
        translate([0,0,seat2_z + brg_w])
            cylinder(d=hole_d(brg_od,"slide"), h=brg_gap+0.01);
        // stator bolts
        bolt_circle(mot_stator_bc, mot_stator_n, mot_stator_bolt, base_h+2, 45);
        // slip ring from below
        translate([0,0,-0.01]) slip_ring_channel();
    }
}

// =====================================================================
// OUTPUT
// =====================================================================
if (part == "yoke")      yoke();
if (part == "base")      base();
if (part == "assembly") { base(); translate([0,0,base_h + mot_h + 0.5]) yoke(); }
if (part == "section")  difference() {
    union() { base(); translate([0,0,base_h + mot_h + 0.5]) yoke(); }
    translate([0,-100,-50]) cube(200);
}
```

---

## 4. Tolerance Table

| Feature | Nominal | FDM hole | SLA hole | CNC (ISO fit) |
|---|---