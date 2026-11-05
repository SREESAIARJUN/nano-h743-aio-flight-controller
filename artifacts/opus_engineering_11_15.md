# Pan-Axis Yoke: Parametric Geometry (Cycle 11)

I don't have the earlier cycles' context, so every dimension below is a named parameter. Values marked **VERIFY** are typical defaults that you should check against your actual motor and slip-ring datasheets before cutting metal.

---

## 1. Architecture

```
            ┌──────────── BASE / STATOR PLATE (stationary) ────────────┐
            │   slip-ring STATOR flange pocket + anti-rotation key     │
            │   motor STATOR bolt circle                                │
            └───────────────┬──────────────────────────────────────────┘
                            │  hollow stationary SPINDLE (OD = bearing ID)
                    ┌───────┴───────┐
   motor ROTOR ═════╡  YOKE HUB     ╞═════  rotor bolt circle + pilot register
                    │ [brg A]       │  ← upper bearing seat (shoulder below)
                    │   spacer land │
                    │ [brg B]       │  ← lower bearing seat (retaining lip/circlip)
                    └───┬───────┬───┘
                 ARM ───┘       └─── ARM   (tilt bore + wire channels in arms)
```

- **Stationary spindle.** A hollow spindle carries the 6.5 mm slip ring through its bore. The ring's stator flange is keyed to the base, and its rotor leads exit radially into the rotating hub.
- **Duplex bearings.** Two thin-section bearings in the yoke hub, spaced apart, take the moment load off the motor's own small bearings. Default is 6701ZZ (12×18×4).
- **Bearing choice.** 6701 is preferred over 6800 (10×19×5) because the 12 mm bore leaves a 2.6 mm spindle wall around the 6.8 mm channel, versus 1.6 mm with a 6800.
- **Motor attachment.** The motor rotor bolts to the hub top face and is located by a pilot register, so the motor never sets concentricity. The bearings do.

---

## 2. Fit and Tolerance Table

| Feature | Nominal | CNC (Al 6061) | FDM (PETG/PA-CF) | Fit intent |
|---|---|---|---|---|
| Bearing housing bore | 18.000 | **+0.000 / −0.010** (≈N6) | +0.10 (then ream/heat-press) | Light press |
| Spindle OD at bearings | 12.000 | **−0.005 / −0.015** (g6) | −0.05 | Sliding (allows preload shim) |
| Slip-ring channel | 6.5 | **6.70 ±0.02** | 6.90 | Clearance; never clamp the barrel |
| Slip-ring flange pocket | per DS | +0.05 | +0.20 | Clearance + key |
| Rotor pilot register | per motor | H7 | +0.15 | Locating |
| Hub bore runout to pilot | — | ≤0.01 TIR | ≤0.05 | Single setup op |

**Rule:** machine the bearing bores, the shoulder faces, and the rotor pilot in **one chucking** of the hub. That is where pan-axis wobble comes from.

---

## 3. OpenSCAD Source

```openscad
// ============================================================
// PAN-AXIS YOKE + HOLLOW-SHAFT BEARING SEATS + 6.5mm SLIP RING
// Cycle 11  |  units: mm  |  Z=0 = motor-rotor mating face
// ============================================================

/* [Manufacturing] */
MFG          = "CNC";     // "CNC" or "FDM"
$fn          = 128;
part         = "all";     // "all","yoke","spindle","base","section"

/* [Bearings: 6701ZZ default] */
brg_id       = 12;        // spindle seat
brg_od       = 18;
brg_w        = 4;
brg_gap      = 6;         // spacer land between bearings (moment arm)
brg_shoulder = 1.2;       // radial shoulder height (<= ~1/2 outer race thickness)

/* [Slip ring: 6.5mm capsule, VERIFY datasheet] */
sr_d         = 6.5;
sr_len       = 17.0;      // VERIFY body length
sr_fl_d      = 13.0;      // VERIFY stator flange OD
sr_fl_t      = 1.0;       // VERIFY flange thickness
sr_key_w     = 2.0;       // anti-rotation key (flange tab / pin)
sr_wire_slot = [3.0, 2.0];// radial rotor-lead exit [w, h]

/* [Motor: hollow-shaft gimbal motor, VERIFY] */
mot_od        = 34;
rot_bc        = 16;       // rotor bolt circle dia
rot_n         = 4;
rot_screw     = 2.0;      // M2
rot_pilot_d   = 10.0;     // rotor register boss
rot_pilot_h   = 1.0;
stat_bc       = 19;       // stator bolt circle
stat_n        = 4;
stat_screw    = 2.5;      // M2.5
mot_h         = 14;       // stack height (base to rotor face)

/* [Yoke] */
hub_od        = brg_od + 8;
hub_h         = 2*brg_w + brg_gap + 3;   // +3 = retaining lip / circlip land
arm_span      = 70;       // inner distance between arms (payload width + clearance)
arm_t         = 5;
arm_w         = 16;
arm_drop      = 55;       // hub bottom -> tilt axis
tilt_bore     = 8;        // tilt-side bearing/motor pilot
arm_wire_d    = 3.2;      // internal wire channel in each arm
fillet_r      = 4;

/* [Spindle] */
spindle_len   = hub_h + mot_h + 4;   // passes through motor into base
spindle_flange_d = 22;
spindle_flange_t = 3;

// ---------- derived tolerances ----------
function fit(cnc, fdm) = (MFG == "CNC") ? cnc : fdm;
brg_bore_cut = brg_od + fit(-0.005, 0.10);   // housing (press)
spindle_od   = brg_id + fit(-0.010, -0.05);  // shaft (sliding)
sr_channel   = sr_d   + fit( 0.20,  0.40);
sr_fl_cut    = sr_fl_d + fit(0.05,  0.20);
pilot_cut    = rot_pilot_d + fit(0.01, 0.15);
eps = 0.01;

// ---------- sanity checks ----------
assert(spindle_od - sr_channel >= 2*1.5, "Spindle wall < 1.5mm: use larger-bore bearing");
assert(hub_od - brg_od >= 6, "Hub wall too thin around bearings");
assert(rot_bc/2 - rot_screw > rot_pilot_d/2, "Rotor screws collide with pilot");
assert(brg_shoulder < (brg_od-brg_id)/4, "Shoulder will touch inner race/seals");

// ============================================================
module bolt_circle(bc, n, d, h, start=45)
  for (i=[0:n-1]) rotate(start + i*360/n)
    translate([bc/2,0,-eps]) cylinder(d=d, h=h+2*eps);

// --- duplex bearing seats with central shoulder land ---
module bearing_seats_cut() {
  // upper seat (enters from top, seats on land)
  translate([0,0,hub_h-brg_w-1.5]) cylinder(d=brg_bore_cut, h=brg_w+1.5+eps);
  // lower seat (enters from bottom, retained by lip/circlip)
  translate([0,0,-eps]) cylinder(d=brg_bore_cut, h=brg_w+1.5+eps);
  // land bore: clears spindle, provides axial shoulder for both outer races
  cylinder(d=brg_od - 2*brg_shoulder, h=hub_h+eps);
  // grease/relief undercuts at bore corners (CNC tool-radius relief)
  for (z=[1.5+brg_w, hub_h-brg_w-1.5])
    translate([0,0,z-0.3]) cylinder(d=brg_bore_cut+0.6, h=0.3+eps);
}

// --- rotating side of slip ring: lead exit through hub to arm channels ---
module slip_lead_exit_cut() {
  z = hub_h/2;
  for (s=[-1,1]) translate([0,0,z])
    rotate([0,90,s>0?0:180])
      translate([0,-sr_wire_slot[0]/2,-sr_wire_slot[1]/2])
        cube([hub_od, sr_wire_slot[0], sr_wire_slot[1]]);
}

// ============================================================
module yoke() {
  difference() {
    union() {
      // hub
      cylinder(d=hub_od, h=hub_h);
      // rotor mating flange
      translate([0,0,hub_h]) cylinder(d=rot_bc+rot_screw*3, h=2.5);
      // cross-beam
      hull() {
        translate([-arm_span/2-arm_t, -arm_w/2, 0]) cube([arm_span+2*arm_t, arm_w, 0.1]);
        translate([0,0,0]) cylinder(d=hub_od, h=hub_h*0.6);
      }
      translate([-arm_span/2-arm_t, -arm_w/2, -arm_t])
        cube([arm_span+2*arm_t, arm_w, arm_t]);
      // arms
      for (s=[-1,1]) translate([s>0 ? arm_span/2 : -arm_span/2-arm_t, -arm_w/2, -arm_drop-arm_w/2])
        cube([arm_t, arm_w, arm_drop+arm_w/2]);
      // tilt bosses
      for (s=[-1,1]) translate([s*(arm_span/2+arm_t/2),0,-arm_drop])
        rotate([0,90,0]) cylinder(d=arm_w+4, h=arm_t, center=true);
    }
    // bearing seats + spindle clearance
    bearing_seats_cut();
    // rotor pilot register & screws (from top)
    translate([0,0,hub_h+2.5-rot_pilot_h]) cylinder(d=pilot_cut, h=rot_pilot_h+eps);
    translate([0,0,hub_h]) bolt_circle(rot_bc, rot_n, rot_screw+fit(0.1,0.3), 3);
    // counterbores from underside for rotor screw heads
    bolt_circle(rot_bc, rot_n, rot_screw*1.9, hub_h-1.2);
    // slip-ring lead exits
    slip_lead_exit_cut();
    // arm wire channels (hub -> tilt boss)
    for (s=[-1,1]) translate([s*(arm_span/2+arm_t/2), 0, -arm_drop])
      cylinder(d=arm_wire_d, h=arm_drop+hub_h/2);
    // tilt bores
    for (s=[-1,1]) translate([s*(arm_span/2+arm_t/2),0,-arm_drop])
      rotate([0,90,0]) cylinder(d=tilt_bore+fit(0.0,0.15), h=arm_t+2, center=true);
  }
}

// ============================================================
module spindle() {
  // stationary; b