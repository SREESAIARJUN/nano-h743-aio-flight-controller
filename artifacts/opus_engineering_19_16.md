# Pan-Axis Yoke: Parametric Geometry (Cycle 19)

I don't have the earlier cycles' parameter set, so this design is built around stated, overridable assumptions. Every dimension lives in one parameter block, and `assert()` checks reject geometrically impossible combinations.

---

## 1. Design Intent and Assumptions

| Item | Assumed part | Key dims (mm) |
|---|---|---|
| Pan motor | GBM2804H-100T-class hollow-shaft gimbal BLDC | Ø34 OD, hollow bore Ø7.0 (verify), stator M3 on Ø19 / M2.5 on Ø16 rotor pattern |
| Pan support bearing | **6808-2RS** thin-section | 40 × 52 × 7 |
| Tilt bearing (each arm) | **6700ZZ** | 10 × 15 × 4 |
| Slip ring | 6.5 mm capsule (6–12 wire) | Ø6.5 body, Ø18 flange, 2× M2 on Ø13.5, ~20 L |

**Load path.** The motor bearings are not rated for the yoke's overturning moment. A 6808 thin-section bearing therefore sits on a fixed tubular stalk that encloses the motor. The yoke skirt rides on the outer race, so the motor carries torque only.

**Coordinates.** +Z is the pan axis, Z = 0 is the base mounting face, and the tilt axis lies along X.

```
        ┌──[tilt 6700]           [tilt 6700]──┐   ← yoke arms
        │                                     │
        └────────┬─────[ yoke hub ]──────┬────┘
                 │ slip-ring flange (rot)│
          ┌──────┴── 6808 outer race ────┴─────┐  ← yoke skirt
          │  ┌─── 6808 inner race ──────────┐  │
          │  │  stalk  ┌─ motor rotor ─┐    │  │
          │  │         │  hollow Ø7    │    │  │  ← Ø6.8 slip-ring channel
          │  │         └─ stator ──────┘    │  │
══════════╧══╧══════════ base plate ════════╧══╧══ wire exit slot →
```

---

## 2. OpenSCAD Source: `pan_yoke_c19.scad`

```openscad
// =====================================================================
//  Pan-Axis Yoke Assembly — Cycle 19
//  Units: mm   |   Z = pan axis   |   tilt axis = X
// =====================================================================
$fn = 128;

/* ---------- [Process] ---------- */
PROCESS        = "FDM";      // "FDM" | "CNC"
// Radial compensation added to every *bore* (FDM holes print undersize)
bore_comp      = (PROCESS == "FDM") ? 0.12 : 0.0;
// Fit allowances (diametral). Negative = interference.
fit_press_od   = (PROCESS == "FDM") ? -0.05 : -0.012; // outer race in housing
fit_slide_id   = (PROCESS == "FDM") ?  0.00 :  0.008; // inner race on stalk
clear_gen      = 0.30;       // general clearance (diametral)

/* ---------- [Pan bearing 6808] ---------- */
pb_id = 40;  pb_od = 52;  pb_w = 7;
pb_shoulder    = 2.0;        // radial land contacting race face (≤ race thickness)

/* ---------- [Tilt bearing 6700] ---------- */
tb_id = 10;  tb_od = 15;  tb_w = 4;
tb_shoulder    = 1.2;

/* ---------- [Motor GBM2804H-class] ---------- */
mot_od         = 34.0;
mot_h          = 17.0;       // stator base → rotor top
mot_bore       = 7.0;        // hollow shaft ID — MEASURE YOUR UNIT
stator_pcd     = 19.0;  stator_screw = 3.2;   // M3 clearance
rotor_pcd      = 16.0;  rotor_screw  = 2.7;   // M2.5 clearance
mot_spigot_d   = 12.0;       // stator centering boss (0 = none)

/* ---------- [Slip ring 6.5 mm capsule] ---------- */
sr_body_d      = 6.5;
sr_body_l      = 20.0;
sr_flange_d    = 18.0;
sr_flange_t    = 1.5;
sr_flange_pcd  = 13.5;  sr_flange_screw = 2.2;  // M2
sr_bundle_d    = 3.5;        // stator-side wire bundle
sr_chan_d      = sr_body_d + clear_gen;          // 6.8

/* ---------- [Base / stalk] ---------- */
base_d         = 80;
base_t         = 5;
stalk_h        = mot_h + 4;  // stalk top sits above rotor top
stalk_wall     = (pb_id - mot_od)/2 - clear_gen/2;
mount_pcd      = 68;  mount_screw = 3.4; mount_n = 4;
exit_slot_w    = 5.0;        // radial wire exit in base underside
exit_slot_d    = 3.0;

/* ---------- [Yoke] ---------- */
skirt_wall     = 3.5;
skirt_od       = pb_od + 2*skirt_wall;
hub_t          = 5;          // plate bolted to rotor
arm_span       = 70;         // inner face to inner face
arm_t          = 8;          // must be ≥ tb_w + tb_shoulder-face
arm_w          = 22;
arm_h          = 55;         // hub top → tilt axis
tilt_wire_d    = 4.5;        // wire channel through tilt bore

/* ---------- Derived ---------- */
pb_z           = base_t + stalk_h - pb_w;     // bearing bottom face
hub_z          = base_t + mot_h + 0.5;        // hub underside on rotor top
skirt_bot_z    = pb_z - 1.0;

/* ---------- Sanity checks ---------- */
assert(mot_bore >= sr_chan_d,
  str("Motor bore ", mot_bore, " < slip-ring channel ", sr_chan_d,
      " — slip ring will not pass. Use larger-bore motor."));
assert(stalk_wall >= 1.5, "Stalk wall too thin between motor and 6808 ID.");
assert(arm_t >= tb_w + 1.0, "Tilt arm too thin for bearing + shoulder.");
assert(pb_shoulder < (pb_od - pb_id)/2 - 0.5,
  "Pan shoulder would contact the bearing seals/opposite race.");
assert(stalk_h > mot_h, "Stalk must clear motor height.");
assert(hub_z + hub_t > pb_z + pb_w, "Hub must sit above pan bearing.");

// =====================================================================
//  Primitive: bearing seat (bore + shoulder), positioned at origin,
//  race occupies z ∈ [0, w]
// =====================================================================
module bearing_bore(od, w, shoulder, fit, depth_extra = 0.01) {
    // seat
    translate([0,0,-depth_extra])
        cylinder(d = od + fit + 2*bore_comp, h = w + 2*depth_extra);
    // through-relief inside shoulder
    translate([0,0,-50])
        cylinder(d = od - 2*shoulder + 2*bore_comp, h = 100);
    // 0.5×45° lead-in chamfer
    translate([0,0,w - 0.5])
        cylinder(d1 = od + fit + 2*bore_comp,
                 d2 = od + fit + 2*bore_comp + 1.0, h = 0.51);
}

// Undercut so race corner radius never rides on a sharp internal corner
module seat_undercut(od, at_z) {
    translate([0,0,at_z - 0.3])
        cylinder(d = od + 0.6, h = 0.6);
}

module bolt_circle(pcd, d, n, h, start = 0) {
    for (i = [0:n-1]) rotate([0,0,start + i*360/n])
        translate([pcd/2,0,0]) cylinder(d = d, h = h);
}

// =====================================================================
//  BASE + STALK  (fixed)
// =====================================================================
module base() {
    difference() {
        union() {
            cylinder(d = base_d, h = base_t);
            // stalk: OD = 6808 ID (inner race seat)
            cylinder(d = pb_id + fit_slide_id, h = base_t + stalk_h);
        }
        // motor pocket
        translate([0,0,base_t])
            cylinder(d = mot_od + clear_gen*2, h = stalk_h + 1);
        // stator screws + centering spigot
        translate([0,0,-1]) bolt_circle(stator_pcd, stator_screw + 2*bore_comp, 4, 50, 45);
        if (mot_spigot_d > 0)
            translate([0,0,base_t - 1.5])
                cylinder(d = mot_spigot_d + 0.1 + 2*bore_comp, h = 2);
        // M3 counterbores from below
        translate([0,0,-1]) bolt_circle(stator_pcd, 6.0, 4, 2.5, 45);

        // slip-ring stator channel + bundle exit
        translate([0,0,-1]) cylinder(d = sr_chan_d + 2*bore_comp, h = 50);
        // radial exit slot on underside
        translate([0, -exit_slot_w/2, -0.01])
            cube([base_d/2 + 1, exit_slot_w, exit_slot_d]);

        // base mounting holes
        translate([0,0,-1]) bolt_circle(mount_pcd, mount_screw, mount_n, 20);

        // inner race retention: snap-ring groove above bearing (optional)
        translate([0,0,pb_z + pb_w])
            difference() {
                cylinder(d = pb_id + 2, h = 1.1);
                translate([0,0,-1]) cylinder(d = pb_id - 1.4, h = 4);
            }
    }
    // inner-race shoulder ring (lower stop)
    translate([0,0,pb_z - 1.5])
        difference() {
            cylinder(d = pb_id + 2*pb_shoulder, h = 1.5);
            translate([0,0,-1]) cylinder(d = pb_id - 0.1, h = 4);
        }
}

// =====================================================================
//  YOKE  (rotating) — hub + skirt + two tilt arms
// =====================================================================
module tilt_arm(side) {   // side = +1 / -1
    x0 = side > 0 ? arm_span/2 : -arm_span/2 - arm_t;
    difference() {
        hull() {
            translate([x0, -arm_w/2, hub_z]) cube([arm_t, arm_w, hub_t]);
            translate([x0, 0, hub_z + arm_h]) rotate([0,90,0])
                cylinder(d = tb_od + 2*5, h = arm_t);
        }
        // tilt bearing seat, opening toward the inside
        translate([side > 0 ? arm_span/2 : -arm_span/2, 0, hub_z + arm_h])
            rotate([0, side > 0 ? 90 : -90, 0]) {
                bearing_bore(tb_od, tb_w, tb_shoulder, fit_press_od);
                seat_undercut(tb_od, 0);
            }
        // wire channel down the arm → hub
        translate([x0 + arm_t/2, 0, hub_z - 1])
            cylinder(d = tilt_wire_d, h = arm_h + 1);
    }
}

module yoke() {
    difference() {
        union() {
            // hub plate
            translate([0,0,hub_z])
                cylinder(d = skirt_od, h = hub_t);
            // skirt carrying 6808 outer race
            translate([0,0,skirt_bot_z])
                cylinder(d = skirt_od, h = hub_z - skirt_bot_z + 0.01);
            // arm bridge
            translate([-arm_span/2 - arm_t, -arm_w/2, hub_z])
                cube([arm_span + 2*arm_t, arm_w, hub_t]);
        }
        // skirt interior clearance over stalk
        translate([0,0,skirt_bot_z - 1])
            cylinder(d = pb_id + 2*pb_shoulder + clear_gen, h = hub_z - skirt_bot_z + 1);
        // 6808 outer-race seat (press)
        translate([0,0,pb_z]) {
            bearing_bore(pb_od, pb_w, 0.01, fit_press_od);
            seat_undercut(pb_od, pb_w);
        }
        // rotor bolt pattern + head counterbores
        translate([0,0,hub_z - 1]) bolt_circle(