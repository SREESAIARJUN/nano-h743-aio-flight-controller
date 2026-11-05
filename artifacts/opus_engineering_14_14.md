# Pan-Axis Yoke: Bearing Seats and 6.5 mm Slip-Ring Channel (Cycle 14)

I don't have the earlier cycles, so this design states its baseline assumptions explicitly. Every assumption is a single parameter. Change the parameter and the geometry and checks follow.

---

## 1. Architecture and Stack-Up

```
        ┌──────── tilt axis (Ø bore in arm tops) ────────┐
        │  ARM                                     ARM   │   ← rotating yoke
        │  ║cable channel                              │
        └──╨───────────── YOKE HUB ──────────────────────┘
                  │ rotor bolt circle │  ← motor ROTOR bolts to yoke hub
              ┌───┴───────────────────┴───┐
              │   hollow-shaft BLDC motor  │
              └───┬───────────────────┬───┘
                  │ stator bolt circle │  ← motor STATOR bolts to base housing
        ┌─────────┴───────────────────┴─────────┐
        │  BASE HOUSING  [6800ZZ]               │  ← fixed
        │                [spacer]               │
        │                [6800ZZ]  + retaining  │
        └──────────── shaft exit / wire out ────┘
   Hollow shaft Ø10/Ø8: fixed in yoke hub → through motor bore → bearing pair
   Slip ring Ø6.5 capsule: inside shaft bore. Rotor leads go up, stator leads go down.
```

**Assumptions (all parametric):**

| Item | Default | Notes |
|---|---|---|
| Slip ring | Ø6.5 × 17 mm capsule, 6-wire | Rotor flange or tab keyed to the shaft |
| Hollow shaft | Ø10 OD / Ø8 ID, 6061-T6 or 304 tube | 0.75 mm radial clearance around the slip ring |
| Bearings | 2 × 6800ZZ (10×19×5), DB pair with 3 mm spacer | Gives axial and moment stiffness |
| Motor | 2804-class hollow shaft | **Verify the through-bore is ≥ 10.5 mm.** The code asserts this. |
| Process | FDM PETG/PA-CF *or* CNC 6061 | Selected with `process` |

---

## 2. Fit and Tolerance Table

| Interface | CNC (ISO) | FDM (print-and-ream) |
|---|---|---|
| Bearing OD seat Ø19 | **H7** → 19.000 / +0.021, light press, Loctite 641 | Print Ø18.90, ream to Ø19.00, press with retaining compound |
| Shaft to bearing ID Ø10 | **k6** shaft (+0.001/+0.010) | Same metal shaft |
| Shaft to yoke hub | Ø10 **H7** + 2× M2 set screws on a flat | Ø10.05 + clamp slot + M2.5 pinch bolt |
| Slip ring in shaft bore | Ø8 bore, Ø6.5 body: free slip | Same |
| Bearing shoulder Ø | ≥ 16.5 (6800 ISO abutment) | Same |
| Seat coaxiality | ⌀0.01 between the two seats (one setup) | Ream both seats in one pass |
| Bearing preload | Spacer length controls it. Lap ±0.005. | 3.00 spacer + wave washer |

---

## 3. OpenSCAD: `pan_yoke_c14.scad`

```openscad
// ============================================================
// Pan-axis yoke + bearing housing + slip-ring channel — Cycle 14
// Units: mm. All geometry is driven from the parameters below.
// ============================================================
$fn = 96;

// ---------- process ----------
process   = "FDM";            // "FDM" or "CNC"
hole_comp = (process=="FDM") ? 0.15 : 0.0;   // FDM undersize holes → add comp
ream_stock= (process=="FDM") ? -0.10 : 0.0;  // leave stock for reaming bearing seat

// ---------- slip ring (6.5 mm capsule) ----------
sr_d        = 6.5;
sr_len      = 17.0;
sr_lead_bundle_d = 3.0;     // 6 × AWG30 bundle
sr_tab_w    = 1.6;          // anti-rotation tab (rotor side)
sr_tab_h    = 2.0;

// ---------- hollow shaft ----------
shaft_od    = 10.0;
shaft_id    = 8.0;
shaft_len   = 42.0;

// ---------- bearings (6800ZZ) ----------
brg_d   = 10;  brg_D = 19;  brg_B = 5;
brg_spacer   = 3.0;
brg_shoulder = 16.5;        // ISO abutment diameter
circlip_groove_d = 20.2;    // DIN 472 Ø19 internal circlip groove
circlip_w        = 1.1;

// ---------- motor (2804-class hollow shaft) ----------
mot_od          = 34.0;
mot_h           = 18.0;
mot_bore        = 11.0;     // VERIFY on datasheet
stator_pcd      = 19.0;  stator_screw = 2.0;  // M2
rotor_pcd       = 19.0;  rotor_screw  = 2.0;  // M2
n_screws        = 4;

// ---------- yoke ----------
hub_d        = 40.0;
hub_h        = 8.0;
yoke_span    = 90.0;        // inner distance between arms (camera width + clearance)
arm_t        = 6.0;
arm_w        = 18.0;
arm_h        = 70.0;        // hub top → tilt axis
tilt_bore_d  = 19.0;        // tilt bearing seat (6800) or motor boss
chan_w       = 3.4;         // cable channel width
chan_d       = 2.4;         // channel depth (cover with a 0.8 mm snap lid)
bend_r       = 4.0;         // minimum wire bend radius at corners

// ---------- base housing ----------
base_od      = 36.0;
base_h       = 2*brg_B + brg_spacer + 4.0;
flange_t     = 4.0;
mount_pcd    = 30.0;  mount_screw = 3.0;

// ---------- design-rule checks ----------
assert(mot_bore >= shaft_od + 0.5, "Motor through-bore too small for shaft");
assert(shaft_id - sr_d >= 1.0,     "Slip-ring radial clearance < 0.5 mm");
assert((shaft_od - shaft_id)/2 >= 0.75, "Shaft wall too thin");
assert(chan_w >= sr_lead_bundle_d + 0.3, "Cable channel too narrow");
assert(hub_d/2 - rotor_pcd/2 >= 2*rotor_screw, "Rotor screws too close to hub edge");

function H(d) = d + hole_comp;          // compensated clearance hole

// ============================================================
module bolt_circle(pcd, d, n, h, z0=-1) {
    for (i=[0:n-1]) rotate(45 + i*360/n)
        translate([pcd/2,0,z0]) cylinder(d=H(d)+0.2, h=h);
}

// ---------- hollow shaft ----------
module shaft() {
    difference() {
        cylinder(d=shaft_od, h=shaft_len);
        translate([0,0,-1]) cylinder(d=shaft_id, h=shaft_len+2);
        // flat for set screws (top 8 mm)
        translate([shaft_od/2-0.5, -shaft_od/2, shaft_len-8]) cube([2, shaft_od, 9]);
        // keyway for slip-ring anti-rotation tab
        translate([-sr_tab_w/2, shaft_id/2-0.2, shaft_len-sr_tab_h])
            cube([sr_tab_w+0.1, 1, sr_tab_h+1]);
    }
}

// ---------- base housing: bearing seats ----------
module base_housing() {
    seat_D = brg_D + ream_stock;
    difference() {
        union() {
            cylinder(d=base_od, h=base_h);
            cylinder(d=mount_pcd + 2*mount_screw + 6, h=flange_t);
        }
        // lower bearing seat (from below)
        translate([0,0,-1]) cylinder(d=seat_D, h=brg_B + 1 + 2);
        // shoulder bore between the bearings, sized for the spacer + shaft clearance
        cylinder(d=brg_shoulder, h=base_h+1);
        // upper bearing seat (from above)
        translate([0,0,base_h-brg_B-2]) cylinder(d=seat_D, h=brg_B+3);
        // circlip grooves at both ends
        translate([0,0,1.0])                 cylinder(d=circlip_groove_d, h=circlip_w);
        translate([0,0,base_h-1.0-circlip_w]) cylinder(d=circlip_groove_d, h=circlip_w);
        // stator bolt circle (top face)
        bolt_circle(stator_pcd, stator_screw, n_screws, base_h+2);
        // frame mount bolt circle (flange)
        bolt_circle(mount_pcd, mount_screw, 4, flange_t+2);
        // stator lead exit slot (motor phase wires)
        translate([brg_D/2, -1.5, base_h-3]) cube([base_od, 3, 4]);
    }
}

// ---------- slip-ring rotor-lead path: radial exit, then up the arm ----------
module lead_path(side=1) {
    // radial groove on the hub top from the shaft bore to the arm root
    hull() {
        translate([0,0,hub_h-chan_d]) cylinder(d=chan_w, h=chan_d+1);
        translate([side*(yoke_span/2 + arm_t/2), 0, hub_h-chan_d])
            cylinder(d=chan_w, h=chan_d+1);
    }
    // radiused transition into the vertical arm channel
    translate([side*(yoke_span/2 + arm_t/2), 0, hub_h])
        rotate([90,0,0]) rotate_extrude(angle=360, $fn=48)
            translate([bend_r,0]) square([0.01,0.01]); // placeholder for bend relief
    // vertical channel on the outer face of the arm
    translate([side*(yoke_span/2 + arm_t) - chan_d + 0.01, -chan_w/2, hub_h - chan_d])
        cube([chan_d+0.5, chan_w, arm_h - tilt_bore_d/2 - 3]);
}

// ---------- yoke ----------
module arm(side=1) {
    x = side*(yoke_span/2 + arm_t/2);
    difference() {
        hull() {
            translate([x-arm_t/2, -arm_w/2, 0]) cube([arm_t, arm_w, 1]);
            translate([x, 0, hub_h+arm_h]) rotate([0,90,0])
                cylinder(d=arm_w + 8, h=arm_t, center=true);
        }
        // tilt-axis bearing seat / motor boss
        translate([x, 0, hub_h+arm_h]) rotate([0,90,0])
            cylinder(d=tilt_bore_d + ream_stock, h=arm_t+2, center=true);
    }
}

module yoke() {
    difference() {
        union() {
            cylinder(d=hub_d, h=hub_h);
            // cross-beam, with fillet volume via hull
            hull() {
                translate([-(yoke_span/2+arm_t), -arm_w/2, 0]) cube([yoke_span+2*arm_t, arm_w, hub_h]);
                cylinder(d=hub_d, h=hub_h);
            }
            arm(1); arm(-1);
        }
        // shaft bore (H7 / FDM clamp)
        translate([0,0,-1]) c