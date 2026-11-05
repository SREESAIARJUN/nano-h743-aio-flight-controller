# Pan-Axis Yoke: Parametric Geometry (Cycle 18)

I don't have the earlier cycles' context, so some motor, bearing and slip-ring dimensions below are **assumed placeholders**. They are grouped in clearly marked blocks so you can replace them with your Cycle 17 values or datasheet dimensions. Every dependent dimension derives from those blocks, and `assert()` guards catch impossible combinations.

---

## 1. Architecture

```
            ┌──────── tilt bearing seat (both arms)
   ┌──┐                       ┌──┐
   │  │◄── yoke arm           │  │
   │  │                       │  │
   │  └───────┬─────┬─────────┘  │  ◄── yoke bridge
   └──────────┤ SR  ├────────────┘
              │cap- │◄── 6.5 mm slip-ring capsule (in mast top)
     ┌────────┤sule ├────────┐ ◄── pan support bearing (6701, 12×18×4)
     │  hub   │     │        │     outer race in yoke hub, inner on mast
     └──┬─────┤mast ├─────┬──┘ ◄── hub bolts to motor ROTOR
        │ BLDC│(fix)│rotor│
        │stator     │     │  ◄── hollow-shaft gimbal motor
   ═════╧═════╧═════╧═════╧═════ base (mast is fixed here)
```

**Design intent:**

- **Stationary mast tube.** The mast is fixed to the base and passes through the motor's hollow shaft. It never touches the rotating parts except through the bearing.
- **Pan support bearing.** The bearing sits in the yoke hub on top of the mast. It takes the yoke's radial and moment loads, so the small motor bearings only carry torque. This is the main stiffness gain.
- **Slip-ring capsule.**
  - The capsule body is clamped in the mast top and keyed against rotation by a flat.
  - The rotor-side leads exit upward into the yoke.
  - The stator-side leads run down the mast bore and out through the base.
  - No wire ever crosses a rotating gap.

---

## 2. OpenSCAD Source

```openscad
// =====================================================================
//  PAN-AXIS YOKE + BEARING SEATS + 6.5 mm SLIP-RING CHANNEL  (Cycle 18)
//  Units: mm.  Render a part with:  PART = "yoke" | "mast" | "assembly"
// =====================================================================
PART = "assembly";
$fn  = $preview ? 64 : 160;

// ---------------- [A] MOTOR — VERIFY AGAINST DATASHEET ----------------
motor_od          = 46.0;   // rotor bell OD (e.g. 4108H-class)
motor_h           = 25.0;   // stator mount face -> rotor top face
motor_bore        = 13.0;   // hollow shaft clear bore (must pass mast)
rotor_bc          = 16.0;   // rotor bolt-circle diameter
rotor_bolt_n      = 4;
rotor_bolt_d      = 3.0;    // M3 (use 2.0 for M2 patterns)
rotor_boss_d      = 25.0;   // raised boss on rotor top (relief pocket)
rotor_boss_h      = 1.0;

// ---------------- [B] BEARINGS — VERIFY ------------------------------
pan_brg   = [12, 18, 4];    // 6701ZZ  [ID, OD, W]
tilt_brg  = [ 8, 16, 5];    // 688ZZ   [ID, OD, W]

// ---------------- [C] SLIP RING — VERIFY -----------------------------
sr_d          = 6.5;        // capsule body OD
sr_len        = 18.0;       // capsule body length (excl. flange)
sr_flange_d   = 9.5;        // stator-end flange OD (0 = none)
sr_flange_t   = 1.0;
sr_lead_bundle= 3.2;        // rotor-side wire bundle dia (6 ch ~ 3 mm)

// ---------------- [D] PROCESS FITS (diametral) -----------------------
// FDM PETG/ASA, 0.4 nozzle. For CNC alu: press -0.012, slip +0.010
fit_brg_press = 0.10;   // added to OD -> printed holes shrink ~0.15
fit_brg_slip  = 0.20;
fit_sr        = 0.25;   // capsule must slide, then be clamped
fit_bolt      = 0.30;
crush_ribs    = true;   // FDM: 6 ribs give interference w/o cracking
rib_h         = 0.20;   // radial rib height

// ---------------- [E] YOKE GEOMETRY ----------------------------------
payload_w     = 70.0;   // clear width between arms (camera/tilt body)
tilt_axis_h   = 55.0;   // tilt axis height above yoke hub bottom face
arm_t         = 7.0;    // arm thickness (along tilt axis)
arm_w         = 24.0;   // arm width
bridge_t      = 6.0;    // bridge plate thickness
hub_d         = 34.0;
hub_h         = 10.0;   // must contain pan bearing + shoulder
wall_min      = 1.6;    // min printable wall

// ---------------- [F] MAST ----------------------------------------------
mast_od       = pan_brg[0];          // bearing ID sets mast OD
mast_id       = sr_d + fit_sr;       // capsule channel
mast_len      = motor_h + 12 + hub_h; // base engagement 12 mm
mast_flange_d = 30; mast_flange_t = 3;
mast_bc = 22; mast_bolt_n = 3;

// ======================== SANITY CHECKS ===============================
assert(motor_bore >= mast_od + 0.6,
       "Mast will rub the motor hollow shaft: increase motor_bore or shrink bearing");
assert((mast_od - mast_id)/2 >= wall_min,
       "Mast wall too thin around slip-ring capsule");
assert(hub_h >= pan_brg[2] + 2.0, "Hub too short for pan bearing + shoulder");
assert(rotor_bc/2 + rotor_bolt_d < hub_d/2 - wall_min, "Bolt circle breaks hub wall");
assert(rotor_bc/2 - rotor_bolt_d/2 > pan_brg[1]/2 + 0.8,
       "Rotor bolts intersect pan bearing seat");
assert(sr_len <= mast_len, "Slip ring longer than mast");

// =========================== PRIMITIVES ===============================
// Bearing seat: blind counterbore + lead-in chamfer + shoulder through-hole
module bearing_seat(brg, depth_extra=0.2, press=true, thru_d=undef) {
    od   = brg[1] + (press ? fit_brg_press : fit_brg_slip);
    d    = brg[2] + depth_extra;
    sh_d = is_undef(thru_d) ? brg[1] - 2.0 : thru_d;  // shoulder supports outer race only
    difference() {
        union() {
            translate([0,0,-0.01]) cylinder(d=od, h=d+0.01);
            translate([0,0,-0.01]) cylinder(d1=od+1.0, d2=od, h=0.5);   // lead-in
            translate([0,0,d-0.01]) cylinder(d=sh_d, h=50);            // thru
        }
        if (press && crush_ribs)
            for (a=[0:60:359]) rotate(a)
                translate([od/2,0,0]) cylinder(r=rib_h+0.15, h=d, $fn=12);
    }
}

// Slip-ring channel with anti-rotation flat + flange recess + lead exit
module slip_ring_channel(len) {
    d = sr_d + fit_sr;
    difference() {
        cylinder(d=d, h=len);
        // flat: capsules have none; a heat-shrink/CA'd shim or clamp
        // screw bears on it. Flat leaves 0.25 mm key engagement.
        translate([d/2-0.25, -d, -1]) cube([d, 2*d, len+2]);
    }
    if (sr_flange_d > 0)
        translate([0,0,len-sr_flange_t-0.1])
            cylinder(d=sr_flange_d+fit_sr, h=sr_flange_t+0.2);
}

module bolt_circle(bc, n, d, h, cb_d=0, cb_h=0, phase=45) {
    for (i=[0:n-1]) rotate(phase + i*360/n) translate([bc/2,0,0]) {
        translate([0,0,-1]) cylinder(d=d+fit_bolt, h=h+2);
        if (cb_d>0) translate([0,0,h-cb_h]) cylinder(d=cb_d, h=cb_h+1);
    }
}

// ================================ YOKE ================================
arm_x = payload_w/2 + arm_t/2;
yoke_span = payload_w + 2*arm_t;

module yoke() {
    difference() {
        union() {
            cylinder(d=hub_d, h=hub_h);                        // hub
            translate([-yoke_span/2, -arm_w/2, hub_h-bridge_t])
                cube([yoke_span, arm_w, bridge_t]);            // bridge
            for (s=[-1,1]) translate([s*arm_x, 0, 0]) arm();   // arms
            for (s=[-1,1]) hull() {                            // gussets
                translate([s*(payload_w/2+0.01), -arm_w/2, hub_h-0.01])
                    cube([0.01, arm_w, 10]);
                translate([s*(payload_w/2 - 8), -arm_w/2, hub_h-0.01])
                    cube([0.01, arm_w, 0.01]);
            }
        }
        // rotor boss relief + rotor bolts (counterbored from top)
        translate([0,0,-0.01]) cylinder(d=rotor_boss_d+0.6, h=rotor_boss_h+0.2);
        bolt_circle(rotor_bc, rotor_bolt_n, rotor_bolt_d, hub_h,
                    cb_d=5.8, cb_h=hub_h-4.0);
        // pan bearing seat: entered from below, outer race press fit,
        // shoulder above it leaves a clear path for the capsule leads
        translate([0,0,rotor_boss_h+0.2])
            bearing_seat(pan_brg