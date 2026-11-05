# Pan-Axis Yoke: Parametric Geometry (Cycle 10)

## 0. Assumptions

I don't have the earlier cycles, so every value that depends on them is a named parameter. **Check the items marked ⚠ against your actual parts before printing or machining.**

| Item | Assumed default | Notes |
|---|---|---|
| Pan motor | Hollow-shaft gimbal BLDC, 2804H-class | ⚠ stator/rotor hole PCDs, shaft bore ID |
| Slip ring | 6.5 mm OD capsule, ~6–12 circuits | ⚠ length, wire-exit geometry |
| Support bearing | 61804-2RS (20×32×7) thin-section | Takes yoke moment loads off the motor's small internal bearings |
| Tilt bearing (yoke arms) | MR85ZZ (5×8×2.5) ×2 | ⚠ match the tilt motor/shaft |
| Process | FDM, PETG/PA-CF, 0.4 mm nozzle | Fit offsets are tunable per process (see §3) |

### Load path

```
        tilt bearing ──┐         ┌── tilt bearing
                       │  YOKE   │
                       └────┬────┘
            yoke hub ── 61804 support bearing ── base housing
                            │
            motor rotor bell ← bolted to yoke hub (torque only)
            motor stator     ← bolted to base
            slip ring: housing keyed to BASE, rotor wires driven by YOKE
                       wires route up through the motor's hollow shaft
```

- The support bearing carries radial and moment loads.
- The motor carries only torque.
- The slip ring carries no load. It is driven through a compliant fork, so misalignment cannot side-load it.

---

## 1. OpenSCAD Source: `pan_yoke_c10.scad`

```openscad
// =====================================================================
//  PAN-AXIS YOKE / BASE / SLIP-RING CHANNEL — Cycle 10
//  Units: mm. All fits parametric. Render parts via `part` selector.
// =====================================================================
/* [Part selector] */
part = "assembly"; // [assembly, yoke, base, fit_coupon, section]

/* [Process compensation] */
fit_press   = -0.10;  // bearing OD press (neg = interference after compensation)
fit_slide   =  0.15;  // sliding clearance (slip ring, shafts)
fit_loose   =  0.30;  // wire channels, free clearance
hole_comp   =  0.20;  // FDM small-hole shrink compensation (diameter)
$fn = 128;

/* [Motor — VERIFY ⚠] */
mot_stator_d     = 28.0;  // motor body OD (for clearance pocket)
mot_h            = 14.5;  // total height incl. bell
mot_stator_pcd   = 19.0;  // stator mount bolt circle
mot_rotor_pcd    = 12.0;  // rotor bell bolt circle
mot_bolt_d       = 2.0;   // M2
mot_bolt_n       = 4;
mot_hollow_id    = 5.5;   // hollow shaft bore (wire pass)
mot_spigot_d     = 8.0;   // stator centering boss (0 = none)

/* [Slip ring — VERIFY ⚠] */
sr_od        = 6.5;
sr_len       = 14.0;      // housing body length
sr_wire_bdl  = 3.2;       // rotor-side wire bundle dia (incl. sleeve)
sr_key_w     = 1.6;       // anti-rotation key width (clamp tab)
sr_fork_gap  = 1.2;       // drive-fork slot width for rotor wires

/* [Support bearing 61804] */
brg_id = 20; brg_od = 32; brg_w = 7;
brg_shoulder = 1.5;       // radial shoulder height (land for race)

/* [Tilt bearings MR85] */
tb_id = 5; tb_od = 8; tb_w = 2.5;

/* [Yoke geometry] */
yoke_span      = 70;      // inner distance between arms (payload width)
yoke_arm_t     = 6;       // arm thickness
yoke_arm_w     = 16;      // arm width (along view axis)
yoke_arm_h     = 55;      // pan plane -> tilt axis
yoke_bridge_t  = 6;       // crossbar thickness
yoke_fillet    = 3;
hub_h          = brg_w + 3;

/* [Base] */
base_wall  = 3;
base_floor = 3;

// ---------------- derived ----------------
bridge_len = yoke_span + 2*yoke_arm_t;
hub_od_brg = brg_id - fit_press;          // hub shaft into bearing ID
sr_bore    = sr_od + fit_slide + hole_comp;

// =====================================================================
//  UTILITIES
// =====================================================================
module bolt_circle(pcd, n, d, h, ang0=45) {
  for (i=[0:n-1]) rotate(ang0 + i*360/n)
    translate([pcd/2,0,-0.01]) cylinder(d=d+hole_comp, h=h+0.02, $fn=24);
}

// Bearing seat: press bore + lead-in chamfer + retaining shoulder
module bearing_seat(od, w, shoulder, depth_extra=0.2) {
  d = od + fit_press + hole_comp;
  translate([0,0,-0.01]) {
    cylinder(d=d, h=w + depth_extra);
    cylinder(d1=d+0.8, d2=d, h=0.4);                 // lead-in
    translate([0,0,w+depth_extra-0.01])
      cylinder(d=od - 2*shoulder, h=50);             // shoulder through-bore
  }
  // FDM crush ribs alternative: uncomment to use 6 ribs instead of full press
  // for(i=[0:5]) rotate(i*60) translate([d/2,0,0]) cylinder(d=0.6,h=w);
}

// Slip-ring channel: bore + anti-rotation key + side wire exit
module slipring_channel(len) {
  translate([0,0,-0.01]) {
    cylinder(d=sr_bore, h=len);
    translate([sr_bore/2-0.5, -(sr_key_w+fit_slide)/2, 0])
      cube([1.6, sr_key_w+fit_slide, len]);          // key slot
  }
  // stator wire exit (radial, through floor wall)
  translate([0,0,0]) rotate([0,90,0])
    hull(){ cylinder(d=sr_wire_bdl+fit_loose,h=40);
            translate([-3,0,0]) cylinder(d=sr_wire_bdl+fit_loose,h=40); }
}

// =====================================================================
//  YOKE (rotating part)
// =====================================================================
module yoke() {
  difference() {
    union() {
      // crossbar
      translate([-bridge_len/2, -yoke_arm_w/2, 0])
        rounded_box([bridge_len, yoke_arm_w, yoke_bridge_t], yoke_fillet);
      // arms
      for (s=[-1,1])
        translate([s*(yoke_span/2 + yoke_arm_t/2) - yoke_arm_t/2,
                   -yoke_arm_w/2, 0])
          arm();
      // central disc for motor rotor bolts
      cylinder(d=mot_rotor_pcd + 8, h=yoke_bridge_t);
      // hub spigot (goes DOWN into support bearing ID)
      translate([0,0,-hub_h]) cylinder(d=hub_od_brg, h=hub_h+0.01);
    }
    // hollow core: wire path up from motor shaft
    translate([0,0,-hub_h-1]) cylinder(d=max(mot_hollow_id, sr_wire_bdl)+fit_loose, h=50);
    // drive fork slot: engages slip-ring rotor wire bundle (compliant)
    translate([-sr_fork_gap/2 - fit_slide/2, -10, -hub_h-0.01])
      cube([sr_fork_gap+fit_slide, 20, 4]);
    // rotor-bell bolts, counterbored from top
    bolt_circle(mot_rotor_pcd, mot_bolt_n, mot_bolt_d, 50);
    for (i=[0:mot_bolt_n-1]) rotate(45+i*360/mot_bolt_n)
      translate([mot_rotor_pcd/2,0,yoke_bridge_t-1.6]) cylinder(d=4.2,h=5);
    // wire exit groove to left arm
    translate([0,-1.75,yoke_bridge_t-2]) cube([bridge_len/2, 3.5, 3]);
  }
}

module arm() {
  difference() {
    rounded_box([yoke_arm_t, yoke_arm_w, yoke_arm_h + tb_od], yoke_fillet);
    // tilt bearing seat, opening toward payload (inner face)
    translate([yoke_arm_t/2, yoke_arm_w/2, yoke_arm_h]) rotate([0,90,0])
      translate([0,0,-yoke_arm_t/2]) bearing_seat(tb_od, tb_w, 0.8);
    // internal wire channel up the arm
    translate([yoke_arm_t/2-1.25, yoke_arm_w/2-1.75, 2])
      cube([2.5, 3.5, yoke_arm_h-6]);
  }
}

// =====================================================================
//  BASE (stationary part)
// =====================================================================
base_od = brg_od + 2*base_wall + 2;
base_h  = base_floor + sr_len + 2 + mot_h + brg_w;

module base() {
  difference() {
    cylinder(d=max(base_od, mot_stator_d + 2*base_wall), h=base_h);

    // support bearing seat at top (open upward)
    translate([0,0,base_h - brg_w - 0.2]) mirror([0,0,0])
      bearing_seat(brg_od, brg_w + 0.2, brg_shoulder);

    // motor pocket below bearing
    translate([0,0,base_floor + sr_len + 2])
      cylinder(d=mot_stator_d + 2*fit_loose, h=mot_h + 0.01);

    // motor stator bolts through mounting deck
    translate([0,0,base_floor + sr_len - 1])
      bolt_circle(mot_stator_pcd, mot_bolt_n, mot_bolt_d, 4);
    if (mot_spigot_d>0)
      translate([0,0,base_floor+sr_len]) cylinder(d=mot_spigot_d+fit_slide, h=3);

    // slip-ring channel (housing keyed, stationary)
    translate([0,0,base_floor]) slipring_channel(sr_len + 3);

    // floor mount holes (M3 on 4 corners of 30 PCD)
    bolt_circle(30, 4, 3.2, base_floor);
  }
}

// =====================================================================
//  FIT COUPON — print this FIRST
// =====================================================================
module fit_coupon() {
  offs = [-0.20, -0.10, 0.0, 0.10];
  for (i=[0:len(offs)-1]) translate([i*42,0,0]) difference() {
    cylinder(d=brg_od+6, h=brg_w+1.5);
    translate([0,0,1.5]) cylinder(d=brg_od+offs[i]+hole_comp, h=10);
    translate([0,0,-1]) cylinder(d=brg_od-3, h=5);
    translate([0,-brg_od/2-2.5,brg_w+1]) linear_extrude(1)
      text(str(offs[i]), size=3, halign="center");
  }
  // slip-ring sleeve sweep
  for (i=[0:3]) translate([i*14, 30, 0]) difference() {
    cylinder(d=sr_od+5, h=8);
    translate([0,0,-1]) cylinder(d=sr_od + 0.05*i + 0.05 + hole_comp, h=10);
  }
}

// =====================================================================
module rounded_box(s, r) {
  hull() for (x=[r, s[0]-r], y=[r, s[1]-r])
    translate([x,y,0]) cylinder(r=min(r, s[0]/2-0.01, s[1]/2-0.01), h=s[2], $fn=32);
}

// ---------------- dispatch ----------------
if (part=="yoke") yoke();
else if (part=="base") base();
else if (part=="fit_coupon") fit_coupon();
else if (part=="section") difference(){ union(){ base();
         translate([0,0,base_h+0.5]) yoke(); }
         translate([0,-100,-1]) cube([200,200,300]); }
else { color("SteelBlue") base();
       color("Orange") translate([0,0,base_h+0.5]) yoke(); }
```

---

## 2. FreeCAD Equivalent (Spreadsheet-Driven)

Use this route if you need STEP export, drawings, or FEM.

1. Create a **Spreadsheet** named `P` and give each cell an alias with the same name as the OpenSCAD variables: `brg_od`, `fit_press`, `sr_bore`, and so on.
2. In **PartDesign**, build one Body per part, in this order:
   - `Base`: Pad, then Pockets for the bearing seat, motor cavity, and slip-ring bore. Drive every dimension with an expression, for example `=P.brg_od + P.fit_press + P.hole_comp`.
   - `Yoke`: Pad the crossbar, Pad the arms (sketch symmetric to the YZ plane), Revolve the hub spigot, and Hole features for the bearing seats.
3. Add a **Chamfer** of 0.4 mm on every bearing-seat entry edge.