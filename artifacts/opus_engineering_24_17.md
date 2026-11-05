# Pan-Axis Yoke: Parametric Geometry (Cycle 24)

I don't have your earlier cycle files, so this design rests on the assumptions below. Each one is a single parameter, so override any that differ from your locked values.

| Assumption | Default | Parameter |
|---|---|---|
| Slip ring | 6.5 mm OD capsule, ~17 mm long, wires exit both ends | `sr_d`, `sr_len` |
| Pan bearings | 2× 6700ZZ (10×15×4), face-to-face spaced | `brg_*` |
| Stationary axle | 10×7 mm 6061 tube (off-the-shelf, wall 1.5) | `tube_od`, `tube_id` |
| Motor | Hollow-shaft gimbal BLDC; rotor bolts 4× M2.5 on PCD 12 | `rot_*`; **verify against datasheet** |
| Tilt bearings | 684ZZ (4×9×4) in each arm boss | `tb_*` |
| Process | FDM (PETG/PA-CF) or CNC (6061) | `process` |

---

## 1. Architecture and Stack-Up

```
        wire bundle (rotating side) ──► arm channel ──► tilt axis
              │
   ┌──────────┴──────────┐  ← hub cap: retains upper outer race, fork engages SR rotor wires
   │ ▓▓ 6700 (upper) ▓▓  │
   │ │   spacer gap   │  │  ← brg_span: sets moment stiffness
   │ ▓▓ 6700 (lower) ▓▓  │      yoke hub (rotates)
   └──┬──────────────┬───┘  ← rotor flange: bolts to BLDC rotor bell
      │  10×7 TUBE   │      (stationary, pressed into stator / base)
      │ [slip ring]  │      ← SR housing keyed in tube (stationary)
      └──────────────┘
```

The axle tube is stationary. The yoke spins on the tube's OD, and the slip ring lives inside the tube's ID. This keeps the slip ring concentric with the pan axis by construction, so the ring body never carries bearing load.

---

## 2. OpenSCAD Source: `pan_yoke_c24.scad`

```openscad
// ============================================================
//  Pan-Axis Yoke — Cycle 24
//  Hollow-shaft BLDC + duplex 6700 seats + 6.5mm slip-ring channel
//  Units: mm.  All fits driven from the PROCESS block.
// ============================================================

/* [Output] */
part    = "assembly"; // [assembly, yoke, hub_cap, axle_tube, section]
process = "fdm";      // [fdm, cnc]

/* [Slip ring] */
sr_d        = 6.5;
sr_len      = 17.0;
sr_bundle_d = 3.2;    // rotor-side wire bundle incl. sleeving
sr_key_w    = 1.2;    // anti-rotation key slot in tube (stator side)

/* [Axle tube — stationary] */
tube_od  = 10.0;      // = bearing ID (6700)
tube_id  = 7.0;
tube_len = 34.0;

/* [Pan bearings 6700ZZ] */
brg_id   = 10.0;
brg_od   = 15.0;
brg_w    = 4.0;
brg_span = 10.0;      // inner-face gap between bearings
brg_shoulder = 1.0;   // radial outer-race shoulder land

/* [Motor rotor interface] */
rot_flange_d = 28.0;
rot_flange_t = 3.0;
rot_pcd      = 12.0;  // VERIFY vs motor datasheet
rot_n        = 4;
rot_bolt_d   = 2.5;
rot_head_d   = 4.8;
rot_pilot_d  = 0;     // >0 adds a centring spigot if rotor has a recess

/* [Yoke] */
payload_w = 60.0;     // clear span between arms
arm_t     = 5.0;
arm_w     = 14.0;
arm_h     = 55.0;     // plate top -> tilt axis
plate_t   = 4.0;
wire_ch_w = 3.6;      // routing channel
wire_ch_d = 2.4;

/* [Tilt bearings 684ZZ] */
tb_od = 9.0;
tb_w  = 4.0;
tb_boss_d = 15.0;

/* [Hub cap] */
cap_t = 2.0;

/* [Hidden] */
$fn = 128;
eps = 0.01;

// ---------- PROCESS-DEPENDENT FITS --------------------------
// FDM: printed holes come out undersize; values tuned for 0.4 nozzle,
//      re-calibrate with the fit coupon (module fit_coupon) per material.
// CNC: housing N7-ish light press for 6700 in Al.
brg_seat_adj  = (process=="fdm") ?  0.12 : -0.008;  // added to OD
tb_seat_adj   = (process=="fdm") ?  0.10 : -0.006;
run_clear     = (process=="fdm") ?  0.40 :  0.20;   // rotating clearances
sr_clear      = (process=="fdm") ?  0.25 :  0.05;
leadin        = (process=="fdm") ?  0.40 :  0.20;   // 45° chamfer

// ---------- DERIVED -----------------------------------------
hub_h  = 2*brg_w + brg_span;
hub_od = brg_od + 2*3.0;
shoulder_d = brg_od - 2*brg_shoulder;
tilt_z = hub_h + arm_h;

// ---------- SANITY CHECKS -----------------------------------
assert(tube_od == brg_id, "Tube OD must equal bearing ID");
assert(tube_id >= sr_d + 2*sr_clear, "Slip ring will not fit tube bore");
assert(shoulder_d > brg_id + 2.0,
       "Shoulder would rub inner race; reduce brg_shoulder");
assert(rot_pcd/2 - rot_head_d/2 > tube_od/2 + run_clear,
       "Rotor bolt heads foul the axle tube");
assert(rot_pcd/2 + rot_head_d/2 < rot_flange_d/2,
       "Bolt heads overhang flange");
assert(tube_len >= hub_h + cap_t + rot_flange_t + 6,
       "Tube too short for stack + base engagement");
assert(sr_len <= tube_len - 2, "Slip ring longer than tube");

// ============================================================
module chamfer_bore(d, c) {     // 45° lead-in cone for press fits
    cylinder(d1=d+2*c, d2=d, h=c+eps);
}

module bearing_seat(od, w, adj, from_top=false, zbase=0) {
    d = od + adj;
    translate([0,0,zbase]) {
        cylinder(d=d, h=w+eps);
        if (from_top) translate([0,0,w-leadin]) mirror([0,0,1])
            translate([0,0,-leadin]) chamfer_bore(d, leadin);
        else translate([0,0,-eps]) chamfer_bore(d, leadin);
    }
}

// ---------- AXLE TUBE (stationary, buy as 10x7 Al tube) -----
module axle_tube() {
    difference() {
        cylinder(d=tube_od, h=tube_len);
        translate([0,0,-eps]) cylinder(d=tube_id, h=tube_len+2*eps);
        // stator-side anti-rotation key slot (bottom end)
        translate([-sr_key_w/2, 0, -eps])
            cube([sr_key_w, tube_od, 4]);
    }
}

// ---------- YOKE HUB ----------------------------------------
module hub_body() {
    // rotor flange
    translate([0,0,-rot_flange_t]) cylinder(d=rot_flange_d, h=rot_flange_t);
    if (rot_pilot_d > 0)
        translate([0,0,-rot_flange_t-1.0]) cylinder(d=rot_pilot_d, h=1.0+eps);
    cylinder(d=hub_od, h=hub_h);
}

module hub_cuts() {
    // through clearance for rotating hub over stationary tube
    translate([0,0,-rot_flange_t-2])
        cylinder(d=tube_od + 2*run_clear, h=hub_h+rot_flange_t+4);
    // lower seat (shoulder above it)
    bearing_seat(brg_od, brg_w, brg_seat_adj, false, 0);
    // relief between bearings: shoulder land, clears inner races
    translate([0,0,brg_