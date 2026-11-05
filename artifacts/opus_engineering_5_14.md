# Cycle 5: Pan-Axis Yoke, Bearing Seats & 6.5 mm Slip-Ring Channel

I don't have the earlier cycles, so this design rests on the assumptions below. **Every value marked `VERIFY` must be measured on your actual parts before you print or machine anything.** All dimensions live in one parameter block, so a change propagates everywhere.

---

## 1. Assumptions

| Item | Assumed part | Key dims (mm) |
|---|---|---|
| Pan motor | 2804-class hollow-shaft gimbal BLDC (e.g. GBM2804H-100T) | Rotor OD 34.0, height 15.5 (with shaft stub), hollow bore ≈ 4.0 for wires only |
| Slip ring | 6.5 mm micro capsule, 6–12 wire | OD 6.5, body length 21.0, rotor tube protrudes 2.5 |
| Moment bearing | 6708ZZ thin-section | 40 × 50 × 6 |
| Yoke | U-yoke carrying the tilt axis | Arm span 70 inner, tilt axis 55 above hub |

### Why a separate 6708 bearing

The internal bearings of a 2804 motor are tiny (≈ 3 × 7 mm) and fail under yoke overturning moment. The design isolates the load paths:

* **Moment and axial load:** carried by the 6708 between the base boss (inner race) and the yoke hub (outer race).
* **Torque only:** the motor rotor drives the yoke through a compliant-tolerance bolt interface.

### Why the slip ring sits below the motor

A 6.5 mm capsule does not fit inside a ~4 mm hollow shaft. So the stack is:

1. The capsule sits coaxially in a channel below the stator, with its body clamped and keyed to the base.
2. Its rotor tube is driven by a coupler on the motor shaft's bottom stub.
3. The rotor-side wires pass up through the hollow shaft to the yoke.

---

## 2. Stack-Up (section view, Z up)

```
   ┌──── yoke arms ────┐
   │                   │
 ══╪═══ yoke hub ══════╪══   ← bolts to motor rotor (top face)
   ║ [6708 outer] ┌──┐ ║      outer race in yoke hub bore
   ║ [6708 inner] │M │ ║      inner race on base boss OD
 ══╩═══ base boss ┤ O├─╩══   ← stator bolts to boss floor
          │       │ T│
          │  ┌────┴──┘        hollow shaft stub
          │  │coupler│
          │  │  SR   │        6.5 mm slip-ring channel
          │  │capsule│        + anti-rotation set-screw
          └──┴───────┴── base plate, wire exit slot
```

---

## 3. OpenSCAD Source (`pan_yoke_c5.scad`)

```openscad
// =====================================================================
//  PAN AXIS – Cycle 5
//  Yoke + 6708 moment-bearing seats + 2804 motor mount + 6.5mm slip ring
//  Units: mm.  Set PROCESS to "FDM" or "CNC" to switch fit tables.
// =====================================================================
$fn = 128;
PROCESS = "FDM";          // "FDM" | "CNC"
PART    = "assembly";     // "base" | "yoke" | "coupler" | "assembly"

// ---------- Motor (2804 class)  — VERIFY ALL ----------
mot_od          = 34.0;
mot_h           = 15.5;   // stator base face -> rotor top face
mot_stator_bc   = 19.0;   // stator bolt circle  VERIFY
mot_stator_n    = 4;
mot_stator_bolt = 2.5;    // M2.5                VERIFY
mot_rotor_bc    = 12.0;   // rotor bolt circle   VERIFY
mot_rotor_n     = 4;
mot_rotor_bolt  = 2.0;    // M2                  VERIFY
mot_spigot_d    = 8.0;    // stator centre boss / pilot VERIFY
mot_shaft_od    = 5.0;    // hollow shaft bottom stub OD VERIFY
mot_shaft_stub  = 2.0;    // stub protrusion below stator face
mot_bore        = 4.0;    // wire passage

// ---------- Moment bearing 6708ZZ ----------
brg_id = 40.0;  brg_od = 50.0;  brg_w = 6.0;
brg_shoulder = 1.2;       // radial shoulder height (inner & outer race only)

// ---------- Slip ring capsule ----------
sr_od    = 6.5;
sr_len   = 21.0;
sr_rot_d = 3.0;           // rotor tube OD  VERIFY
sr_rot_l = 2.5;

// ---------- Fit table ----------
// Values are diametral adjustments added to nominal.
function fit(k) =
  PROCESS == "CNC" ?
    ( k=="brg_housing" ? -0.005 :   // ~M7/K6 light press, 50mm
      k=="brg_shaft"   ? +0.005 :   // ~k5 on 40mm boss
      k=="sr_bore"     ? +0.03  :
      k=="pilot"       ? +0.02  :
      k=="clear"       ? +0.20  : 0 )
  : // FDM (0.4 nozzle, 0.2 layer, PETG/ASA) – tune with test coupon
    ( k=="brg_housing" ? +0.10  :   // + crush ribs give retention
      k=="brg_shaft"   ? -0.08  :
      k=="sr_bore"     ? +0.25  :
      k=="pilot"       ? +0.20  :
      k=="clear"       ? +0.40  : 0 );

// ---------- Base ----------
base_d       = 62;
base_t       = 4.0;       // floor under slip ring
sr_pocket_h  = sr_len + 1.0;
boss_h       = brg_w + 1.5;          // bearing land + lead-in
stator_floor = base_t + sr_pocket_h; // Z of stator seating face
base_mount_bc = 52; base_mount_n = 4; base_mount_bolt = 3.0;

// ---------- Yoke ----------
hub_wall    = 3.5;
hub_od      = brg_od + 2*hub_wall;
gap_axial   = 0.5;        // running gap yoke-hub to base
arm_span    = 70;         // inner distance between arms
arm_t       = 6;
arm_w       = 22;
tilt_z      = 55;         // tilt axis height above hub top
tilt_bore   = 22.0;       // tilt bearing / motor seat (Cycle 6)

// ---------- Derived ----------
boss_top_z  = stator_floor + mot_h - 2.0; // bearing sits near rotor top
rotor_top_z = stator_floor + mot_h;
eps = 0.01;

// =====================================================================
//  Helpers
// =====================================================================
module bolt_circle(bc, n, d, h, z=0, rot=45)
  for (i=[0:n-1]) rotate(rot + i*360/n)
    translate([bc/2,0,z]) cylinder(d=d, h=h);

module chamfer_ring(d, c, z, up=true)   // 45° lead-in cone
  translate([0,0,z]) cylinder(d1= up? d : d+2*c, d2= up? d+2*c : d, h=c+eps);

// Bearing housing bore with optional FDM crush ribs.
module brg_housing_bore(z0) {
  d = brg_od + fit("brg_housing");
  translate([0,0,z0]) difference() {
    cylinder(d=d, h=brg_w+eps);
    if (PROCESS=="FDM")
      for (i=[0:5]) rotate(i*60) translate([d/2,0,0])
        cylinder(d=0.6, h=brg_w, $fn=12);    // 6 ribs, 0.3 mm interference
  }
  chamfer_ring(d, 0.6, z0-0.6, up=false);
}

// =====================================================================
//  BASE: slip-ring channel + stator seat + 6708 inner-race boss
// =====================================================================
module base() {
  boss_od = brg_id + fit("brg_shaft");
  difference() {
    union() {
      // plate + pedestal
      cylinder(d=base_d, h=stator_floor);
      // bearing boss: tube around motor, inner race seats on its OD
      cylinder(d=boss_od, h=boss_top_z);
      // inner-race shoulder (contacts inner race ONLY)
      cylinder(d=brg_id + 2*brg_shoulder, h=boss_top_z - brg_w);
    }
    // motor cavity (clearance around rotor)
    translate([0,0,stator_floor])
      cylinder(d=mot_od + fit("clear") + 1.0, h=mot_h + 5);

    // ---- slip-ring channel ----
    translate([0,0,base_t]) cylinder(d=sr_od + fit("sr_bore"), h=sr_pocket_h+eps);
    chamfer_ring(sr_od + fit("sr_bore"), 0.5, stator_floor-0.5);
    // coupler clearance between capsule top and shaft stub
    translate([0,0,stator_floor-3]) cylinder(d=mot_shaft_od+3, h=3+eps);
    // anti-rotation: radial M3 set screw (nylon tip) at mid capsule
    translate([0,0,base_t + sr_len/2]) rotate([0,90,0])
      cylinder(d=2.5, h=base_d);             // tap M3
    // stator-side wire exit: floor groove + radial slot
    translate([0,-1.75,-eps]) cube([base_d/2+1, 3.5, base_t + 3]);
    // motor phase-wire channel up the cavity wall
    translate([mot_od/2-1, -2.5, stator_floor-eps]) cube([4, 5, mot_h]);

    // ---- stator mount ----
    translate([0,0,stator_floor-1.0])
      cylinder(d=mot_spigot_d + fit("pilot"), h=1.0+eps); // pilot recess
    bolt_circle(mot_stator_bc, mot_stator_n, mot_stator_bolt+fit("clear"),
                stator_floor+1, -eps);
    bolt_circle(mot_stator_bc, mot_stator_n, 5.0, 2.0, -eps); // cap-head c'bore

    // ---- base mounting ----
    bolt_circle(base_mount_bc, base_mount_n, base_mount_bolt+fit("clear"),
                stator_floor+1, -eps, 0);
    // boss top lead-in chamfer
    translate([0,0,boss_top_z-0.6]) difference() {
      cylinder(d=boss_od+2, h=1);
      cylinder(d1=boss_od, d2=boss_od-1.2, h=0.6+eps);
    }
  }
}

// =====================================================================
//  COUPLER: motor shaft stub -> slip-ring rotor tube (torque only)
// =====================================================================
module coupler() {
  h = 3.0;
  difference() {
    cylinder(d=mot_shaft_od + 2.4, h=h);
    translate([0,0,h-mot_shaft_stub]) cylinder(d=mot_shaft_od + 0.10, h=h); // slip-on
    translate([0,0,-eps]) cylinder(d=sr_rot_d + 0.10, h=h);                // rotor tube
    // drive flat: tune to shaft/tube flats, or glue with Loctite 638
    translate([-0.4,-(mot_shaft_od+3)/2,-eps]) cube([0.8, mot_shaft_od+3, 1.2]);
  }
}

// =====================================================================
//  YOKE: hub (6708 outer race + rotor bolt-up) + U-arms
// =====================================================================
module yoke() {
  hub_z0  = boss_top_z - brg_w - gap_axial;   // hub underside
  hub_top = rot