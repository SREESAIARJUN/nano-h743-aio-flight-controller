# Ergonomic Handle Housing (Cycle 21)

This is a parametric OpenSCAD design for a handheld controller. It has three main features:

- **Grip:** an ergonomic grip with finger grooves.
- **Battery:** an 18650 battery chamber with a screw-retained end cap.
- **Head:** a tilted head with a recessed, chamfered bezel for a 1.3" SH1106/SSD1306 OLED and a dished thumbstick aperture.

The shell splits down the mid-plane into two halves. Each half prints flat on its split face without supports.

## Design Overview

```
        thumbstick (front)        OLED bezel (toward user)
              ◯                      ▭▭▭▭
         ╭──────────────────────────────╮   ← head, top tilted 15°
         ╰──╮                       ╭──╯
            │  ┃ ┃ wire channel     │
     front  │  ┃                    │  rear
   (fingers)( ┃ 18650 battery       │
            ( ┃                     │
            ( ┃                     │
            ╰─┸──── screw cap ─────╯
```

| Part | Print orientation | Notes |
|---|---|---|
| `left_half` | Split face down | Clearance + counterbored screw holes |
| `right_half` | Split face down | Pilot holes for M2.5 self-tappers |
| `bottom_cap` | Flat face down | Holds the battery's negative spring contact |
| `stick_cap_ring` (optional) | Flat | Trim ring for the thumbstick aperture |

## OpenSCAD Source

```openscad
// =====================================================================
//  Ergonomic Handle Housing — Cycle 21
//  Integrated 18650 battery chamber, 1.3" OLED bezel, thumbstick aperture
//  Units: mm.  Tested in OpenSCAD 2021.01+
// =====================================================================

/* [Render] */
part = "assembly"; // [assembly, left_half, right_half, bottom_cap, stick_cap_ring, section]
explode = 0;       // [0:1:30]  assembly explode distance
show_components = true;  // ghost battery / OLED / stick in assembly

/* [Shell] */
wall      = 2.2;
tilt      = 15;      // head top-face tilt (deg, toward user)
$fn       = 64;

/* [Grip profile]  [z, width(x), depth(y)] */
grip_slices = [
    [0,    32, 38],
    [30,   30, 36],
    [70,   34, 40],
    [104,  33, 41]
];
finger_z     = [24, 46, 68];  // finger-groove centres
finger_r     = 12;
finger_depth = 1.6;           // how far grooves bite into the grip

/* [Head] */
head_w   = 50;
head_d   = 66;
head_h   = 30;
head_r   = 7;
head_pos = [0, 6, 122];   // centre of head block

/* [Battery: 18650] */
batt_d       = 18.6;
batt_len     = 65.5;
batt_clear   = 0.8;
contact_room = 6;         // spring + button contacts
tube_wall    = 1.6;
bore_d       = batt_d + batt_clear;
bore_len     = batt_len + contact_room;

/* [Bottom cap] */
cap_w        = 26;
cap_d        = 32;
cap_t        = 3;
cap_screw_y  = -13;       // M3 screw retains cap
plug_len     = 4;

/* [OLED 1.3in SH1106 module] */
oled_pcb       = [35.5, 33.8, 1.6];
oled_glass     = [35.0, 23.5, 1.5];
oled_glass_off = 1.2;          // glass centre offset toward pin header (+y local)
oled_window    = [30.5, 16.0]; // visible area + margin
oled_hole_sp   = [30.4, 28.4]; // M2 mounting hole pattern
oled_y         = -13;          // position on head top (local y, - = toward user)
bezel_chamfer  = 1.4;

/* [Thumbstick (Alps RKJXV-class mini analog)] */
stick_body     = [16.6, 16.6, 11];
stick_y        = 18;
stick_aperture = 21;      // through-hole; allows full tilt of 20mm cap
stick_dish_d   = 30;      // thumb dish
stick_dish_dep = 1.5;
stick_screw_sp = 22;      // 2x M2 posts, diagonal

/* [Fasteners] */
split_screw_d  = 2.9;     // M2.5 clearance
split_pilot_d  = 2.1;     // M2.5 self-tap pilot
split_head_d   = 5.2;
boss_d         = 7;
pin_d          = 1.85;    // 1.75 filament alignment pins
pin_len        = 6;

// Boss positions on split plane (y, z)
split_bosses = [
    [  0,  86],     // top of grip, above battery
    [-22, 113],     // head rear
    [ 26, 113],     // head front
    [ 13,  20]      // front low grip (outside battery tube)
];
pins = [[-10, 100], [10, 50], [-14, 40]];

eps = 0.01;

// =====================================================================
//  Primitive helpers
// =====================================================================
module rounded_box(s, r) {
    r2 = max(min(r, min(s)/2 - eps), 0.3);
    hull()
        for (x = [-1, 1], y = [-1, 1], z = [-1, 1])
            translate([x*(s[0]/2 - r2), y*(s[1]/2 - r2), z*(s[2]/2 - r2)])
                sphere(r = r2, $fn = 24);
}

module ellipse_slice(w, d, h = eps) {
    scale([w, d, 1]) cylinder(d = 1, h = h, $fn = 72);
}

// Head local frame: origin at centre of tilted top face, +z = outward normal
module head_frame() {
    translate(head_pos) rotate([-tilt, 0, 0]) translate([0, 0, head_h/2]) children();
}

// =====================================================================
//  Solid volumes (o = inward offset; o=0 outer skin, o=wall inner cavity)
// =====================================================================
module grip_solid(o = 0) {
    difference() {
        union() {
            for (i = [0 : len(grip_slices) - 2]) hull() {
                a = grip_slices[i];
                b = grip_slices[i+1];
                translate([0, 0, max(a[0], o)])
                    ellipse_slice(a[1] - 2*o, a[2] - 2*o);
                translate([0, 0, b[0]])
                    ellipse_slice(b[1] - 2*o, b[2] - 2*o);
            }
        }
        // finger grooves on front (+y)
        for (z = finger_z) {
            dy = grip_d_at(z)/2 + finger_r - finger_depth;
            translate([0, dy, z]) rotate([0, 90, 0])
                cylinder(r = finger_r + o, h = 60, center = true);
        }
    }
}

// linear interpolation of grip depth for groove placement
function grip_d_at(z) =
    let(i = max([for (k = [0 : len(grip_slices) - 2]) if (grip_slices[k][0] <= z) k]),
        a = grip_slices[i], b = grip_slices[i+1],
        t = (z - a[0]) / (b[0] - a[0]))
    a[2] + t * (b[2] - a[2]);

module head_solid(o = 0) {
    hull() {
        translate([0, 0, grip_slices[len(grip_slices)-1][0] - 6])
            ellipse_slice(grip_slices[len(grip_slices)-1][1] - 2*o,
                          grip_slices[len(grip_slices)-1][2] - 2*o);
        translate(head_pos) rotate([-t