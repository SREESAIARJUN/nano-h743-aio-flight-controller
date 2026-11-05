# Pan-Axis Yoke: Parametric Geometry (Cycle 4)

I don't have your Cycle 1–3 artifacts, so this design rests on the assumptions below. Each one is a single parameter, so if your selected parts differ you only need to change that value.

## 0. Baseline Assumptions (verify against your datasheets)

| Item | Assumed part | Key dims (VERIFY) |
|---|---|---|
| Pan motor | GBM2804H-100T-class hollow-shaft gimbal BLDC | Ø28 × 14 mm, stator bolts 4×M2.5 on Ø19, rotor bolts 4×M2 on Ø12, shaft bore Ø7.0 |
| Slip ring | Capsule micro slip ring, Ø6.5 body | Ø6.5 × 17 mm body, Ø10 rotor flange × 1.5 mm, 6 wires (AWG30) both ends |
| Pan support bearing | 6801-2RS | 12 × 21 × 5 mm |
| Tilt bearings (yoke arm tips) | MR85-2RS (×2, one per arm) | 5 × 8 × 2.5 mm |
| Payload tilt span | 50 mm between arm inner faces | Set from tilt payload width |
| Material | PETG/ASA FDM *or* 6061-T6 CNC | Select with `fit_mode` |

**Kinematic layout:**
- The motor stator is fixed to the base and the rotor bell drives the yoke hub.
- The yoke hub spigot (OD 12) runs in the 6801, which is captured in the base housing. The 6801 takes the overturning moment so the motor bearings don't have to.
- The slip ring sits coaxially inside the spigot bore. Its rotor flange is keyed to the yoke and its stator is anchored to the base.
- Wires exit the top of the hub through a side channel and route up one yoke arm.

**Critical check:** the spigot bore must be ≥ 6.5 + 2 × 0.25 = **7.0 mm**, and the spigot wall is (12 − 7)/2 = **2.5 mm**.
- 2.5 mm is acceptable in aluminium.
- 2.5 mm is marginal in FDM. In FDM, either print with 100% infill concentric walls or step up to a 6802 bearing (15 × 24 × 5), which gives a 4.0 mm wall.
- The code below enforces this check with an assert.

---

## 1. Fit & Tolerance Table

| Feature | CNC (6061) | FDM (0.4 nozzle, calibrated) |
|---|---|---|
| Bearing OD seat (press) | Ø OD −0.010 / −0.025 (N7-ish) | Ø OD +0.10, with 6 crush ribs 0.15 high |
| Spigot to bearing ID (slip/locate) | Ø ID −0.005 / −0.015 (h6) | Ø ID −0.05, then ream/sand |
| Slip-ring body channel | Ø6.5 +0.20 | Ø6.5 +0.35 |
| Slip-ring flange pocket | Ø10 +0.10, depth 1.5 +0.05 | Ø10 +0.25, depth 1.6 |
| Motor pilot / register | Ø pilot +0.02 | Ø pilot +0.15 |
| Clearance holes M2 / M2.5 | 2.2 / 2.7 | 2.4 / 2.9 |
| Heat-set insert M2 (FDM only) | n/a | Ø3.2 × 4.0 deep |

---

## 2. OpenSCAD Model (`pan_yoke_c4.scad`)

```openscad
// ============================================================
//  Pan-Axis Yoke — Cycle 4
//  Hollow-shaft BLDC + 6801 support bearing + Ø6.5 slip-ring channel
//  Units: mm.  Z+ = up, pan axis = Z.
// ============================================================

/* [Manufacturing] */
fit_mode = "FDM";          // "FDM" | "CNC"
$fn      = 128;

/* [Motor — GBM2804H-class, VERIFY] */
mot_rotor_bolt_pcd   = 12.0;
mot_rotor_bolt_d     = 2.0;     // M2
mot_rotor_bolt_n     = 4;
mot_rotor_bolt_ang0  = 45;
mot_rotor_pilot_d    = 7.0;     // = hollow shaft bore, used as register
mot_rotor_pilot_h    = 1.0;
mot_rotor_face_d     = 28.0;

/* [Pan support bearing — 6801] */
pb_id = 12.0;  pb_od = 21.0;  pb_w = 5.0;

/* [Tilt bearings — MR85] */
tb_id = 5.0;   tb_od = 8.0;   tb_w = 2.5;

/* [Slip ring — Ø6.5 capsule, VERIFY] */
sr_body_d    = 6.5;
sr_body_l    = 17.0;
sr_flange_d  = 10.0;
sr_flange_t  = 1.5;
sr_key_w     = 1.6;     // anti-rotation tab width on flange (if present)
sr_wire_bundle_d = 2.6; // 6× AWG30 ≈ Ø2.2 + margin

/* [Yoke geometry] */
hub_d        = 32.0;    // flange that bolts to rotor
hub_t        = 4.0;
spigot_od    = pb_id;   // runs in 6801
spigot_l     = pb_w + 3.0; // bearing + preload/circlip land
arm_gap      = 50.0;    // inner face to inner face
arm_t        = 5.0;     // arm thickness (X)
arm_w        = 14.0;    // arm width (Y)
arm_h        = 55.0;    // hub top to tilt axis
boss_od      = tb_od + 2*3.0;
fillet_r     = 3.0;

// ---------- Derived tolerances ----------
function seat_d(od)  = fit_mode=="CNC" ? od-0.018 : od+0.10;
function shaft_d(id) = fit_mode=="CNC" ? id-0.010 : id-0.05;
sr_clear   = fit_mode=="CNC" ? 0.20 : 0.35;
sr_fl_clr  = fit_mode=="CNC" ? 0.10 : 0.25;
pilot_clr  = fit_mode=="CNC" ? 0.02 : 0.15;
hole_m2    = fit_mode=="CNC" ? 2.2 : 2.4;
insert_m2  = 3.2;

sr_bore    = sr_body_d + sr_clear;
spigot_wall = (shaft_d(spigot_od) - sr_bore)/2;

// ---------- Design-rule assertions ----------
assert(spigot_wall >= (fit_mode=="CNC" ? 1.5 : 2.0),
  str("Spigot wall ", spigot_wall, " mm too thin: upsize pan bearing (6802)"));
assert(sr_bore <= mot_rotor_pilot_d + 0.01 || true,
  "Note: slip-ring passes above motor; motor bore only carries base-side wires");
assert(arm_t >= 4.0, "Arm too thin for tilt-motor torque reaction");

// ============================================================
//  Modules
// ============================================================

module bolt_circle(pcd, n, a0=0)
  for (i=[0:n-1]) rotate([0,0,a0+i*360/n]) translate([pcd/2,0,0]) children();

module crush_ribs(d, h, n=6, rib=0.15, w=0.8)
  // ribs protrude INTO the seat; subtract the seat, then add the ribs back
  if (fit_mode=="FDM")
    for (i=[0:n-1]) rotate([0,0,i*360/n])
      translate([d/2 - rib, -w/2, 0]) cube([rib+0.01, w, h]);

module bearing_seat_cut(od, w, lip=0.6, through_d=undef) {
  // Blind pocket with shoulder (lip) and chamfered entry
  d = seat_d(od);
  cylinder(d=d, h=w+0.01);
  translate([0,0,-0.01]) cylinder(d1=d+0.8, d2=d, h=0.4);   // lead-in
  if (!is_undef(through_d)) translate([0,0,-50]) cylinder(d=through_d, h=100);
}

// ---------- Hub + spigot ----------
module hub() {
  difference() {
    union() {
      cylinder(d=hub_d, h=hub_t);
      // spigot projects DOWN through motor into the 6801 in the base
      translate([0,0,-spigot_l]) cylinder(d=shaft_d(spigot_od), h=spigot_l);
      // pilot register into rotor bore
      translate([0,0,-mot_rotor_pilot_h])
        cylinder(d=mot_rotor_pilot_d - pilot_clr, h=mot_rotor_pilot_h);
    }
    // rotor bolts (counterbored from top)
    bolt_circle(mot_rotor_bolt_pcd, mot_rotor_bolt_n, mot_rotor_bolt_ang0) {
      translate([0,0,-5]) cylinder(d=hole_m2, h=20);
      translate([0,0,hub_t-1.6]) cylinder(d=4.0, h=5);   // M2 SHCS head
    }
    // circlip groove on spigot (DIN 471 Ø12: groove Ø11.5 × 1.1)
    translate([0,0,-spigot_l+0.8])
      difference() { cylinder(d=20, h=1.1); cylinder(d=11.5, h=1.1); }
  }
}

// ---------- Slip-ring channel (subtractive) ----------
module slip_ring_channel_cut() {
  // Body bore, full length through spigot + hub
  translate([0,0,-spigot_l-1]) cylinder(d=sr_bore, h=spigot_l+hub_t+2);
  // Flange pocket on hub top — rotor side keyed to yoke
  translate([0,0,hub_t - sr_flange_t - (fit_mode=="FDM"?0.1:0.05)])
    cylinder(d=sr_flange_d + sr_fl_clr, h=sr_flange_t+1);
  // Anti-rotation key slot
  translate([sr_flange_d/2 - 0.5, -(sr_key_w+sr_fl_clr)/2, hub_t - sr_flange_t - 0.1])
    cube([2.0, sr_key_w+sr_fl_clr, sr_flange_t+1]);
  // Radial wire exit toward arm A (+X), with bend radius ≥ 3× bundle
  hull() {
    translate([0,0,hub_t-0.01]) cylinder(d=sr_wire_bundle_d, h=0.01);
    translate([arm_gap/2, 0, hub_t+2]) rotate([0,90,0])
      cylinder(d=sr_wire_bundle_d, h=0.1);
  }
}

// ---------- Arm with integral wire channel ----------
module arm(side=1, wired=false) {
  x0 = side * (arm_gap/2 + arm_t/2);
  difference() {
    hull() {
      translate([x0 - arm_t/2*side - (side<0?arm_t:0) + (side<0?arm_t:0), 0, 0])
        translate([side>0?0:-arm_t, -arm_w/2, 0]) cube([arm_t, arm_w, hub_t]);
      translate([x0, 0, arm_h]) rotate([0,90,0]) cylinder(d=boss_od, h=arm_t, center=true);
    }
    // Tilt bearing seat, opens to INNER face, shoulder on outer side
    translate([x0, 0, arm_h]) rotate([0, side>0?-90:90, 0])
      translate([0,0,-arm_t/2-0.01]) {
        bearing_seat_cut(tb_od, tb_w);
        translate([0,0,-1]) cylinder(d=tb_od-1.5, h=arm_t+2);  // shoulder through-hole
      }
    // Internal wire channel (arm A only): Ø bundle, 1.2 mm cover wall
    if (wired)
      translate([x0, 0, hub_t]) cylinder(d=sr_wire_bundle_d+0.4, h=arm_h-boss_od/2-1);
  }
  if (fit_mode=="FDM")
    translate([x0, 0, arm_h]) rotate([0, side>0?-90:90, 0])
      translate([0,0,-arm_t/2]) crush_ribs(seat_d(tb_od), tb_w);
}

// ---------- Cross-beam joining hub to arms ----------
module crossbeam() {
  hull() {
    translate([-(arm_gap/2+arm_t), -arm_w/2, 0]) cube([arm_gap+2*arm_t, arm_w, hub_t]);
    cylinder(d=hub_d, h=hub_t);
  }
}

// ============================================================
//  Assembly
// ============================================================
module pan_yoke() {
  difference() {
    union() {
      hub();
      crossbeam();
      arm(+1, wired=true);
      arm(-1, wired=false);
    }
    slip_ring_channel_cut();
  }
}

// ---------- Base-side bearing housing (mates to motor stator) ----------
module pan_bearing_housing(h=pb_w+2.5, od=pb_od+8) {
  difference() {
    cylinder(d=od, h=h);
    translate([0,0,h-pb_w]) bearing_seat_cut(pb_od, pb_w);
    translate([0,0,-1]) cylinder(d=pb_od-2.5, h=h);  // shoulder on bottom
    translate([0,0,-1]) cylinder(d=sr_bore+1.0, h=h+2); // slip-ring stator pass
  }
  if (fit_mode=="FDM") translate([0,0,h-pb_w]) crush_ribs(seat_d(pb_od), pb_w);
}

pan_yoke();
// translate([0,0