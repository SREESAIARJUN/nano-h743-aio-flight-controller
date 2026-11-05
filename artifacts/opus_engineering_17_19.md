# Ergonomic Handle Housing: OpenSCAD Design (Cycle 17)

This is a parametric, two-piece (left/right split) handle. It includes:

- **Contoured grip** with finger grooves and an elliptical cross-section
- **18650 battery chamber** with a bottom-loading cap and spring pocket
- **1.3" OLED bezel** (SH1106/SSD1306 128×64 module) on the front face, with a recessed glass pocket and a chamfered window
- **Thumbstick aperture** (KY-023 / PS2-style module) on the top face, with a dished thumb surround and mounting standoffs
- Screw bosses, a tongue-and-groove seam, and a USB-C charge port

```openscad
// =====================================================================
//  Ergonomic Handle Housing  -  Cycle 17
//  Battery: 18650 | Display: 1.3" OLED 128x64 | Input: 2-axis thumbstick
//  Units: mm. Print halves flat on their split face (x = 0), no supports
//  except the USB-C slot bridge (short span, bridges fine).
// =====================================================================

/* [Render] */
part      = "assembly";   // [assembly, left, right, cap, all_plated]
explode   = 12;           // gap between halves in assembly view
show_ghosts = true;       // show battery / OLED / stick reference models
$fn = 64;

/* [Shell] */
wall      = 2.4;
clr       = 0.25;         // general fit clearance

/* [Grip profile]  [z, half-width X, half-depth Y]  (finger grooves) */
grip = [
    [  0, 17.0, 15.0],
    [  8, 18.0, 16.0],
    [ 22, 16.5, 15.0],   // groove - pinky
    [ 34, 18.0, 16.5],
    [ 46, 16.5, 15.0],   // groove - ring
    [ 58, 18.5, 16.5],
    [ 70, 17.0, 15.5],   // groove - middle
    [ 82, 19.0, 17.0],
    [ 96, 19.5, 17.0]
];

/* [Head] */
head_w   = 46;   // X
head_d   = 36;   // Y
head_z0  = 100;
head_z1  = 150;
head_r   = 7;    // corner radius

/* [Battery 18650] */
bat_d     = 18.6;
bat_l     = 65.5;
bat_clr   = 0.8;
spring_l  = 6;     // spring / contact allowance
bat_z0    = 6;     // bottom of chamber (top of cap plug)
tube_wall = 1.6;

/* [OLED 1.3in module] */
oled_pcb   = [35.4, 33.5, 1.6];   // W x H x T
oled_glass = [34.5, 23.0, 1.5];
oled_view  = [30.0, 15.5];        // visible window
oled_glass_off = 1.5;             // glass centre above PCB centre (Z)
oled_z     = 117;                 // centre of glass on front face
oled_tilt  = 8;                   // deg, leans screen back toward eye line

/* [Thumbstick] */
stick_pcb    = [34, 26, 1.6];
stick_hole_d = 28;                // aperture (cap Ø ~25 + travel)
stick_y      = 2;                 // offset toward rear
stick_holes  = [[-13.5, -9.5], [13.5, -9.5], [-13.5, 9.5], [13.5, 9.5]]; // verify vs your module
stick_pcb_drop = 9;               // PCB distance below outer top surface
dish_d       = 40;
dish_depth   = 1.6;

/* [Fasteners] */
screw_d      = 2.6;   // M2.5 self-tap pilot  (use 2.9 clearance side)
screw_clear  = 2.9;
screw_head_d = 5.2;
boss_d       = 6.5;
boss_pts = [          // [y, z] in split plane
    [ 0,   92],
    [14,  110],
    [14,  128],
    [-14.5, 143],
    [ 0,   12]        // below battery, beside cap plug
];

/* [Charge port] */
usb_slot = [9.6, 3.8];
usb_z    = 108;

// =====================================================================
//  Outer form, inset by o (o = 0 -> outer skin, o = wall -> cavity)
// =====================================================================
module ell_disc(rx, ry, o) {
    scale([1, (ry - o) / (rx - o), 1])
        cylinder(r = rx - o, h = 0.01);
}

module grip_body(o = 0) {
    for (i = [0 : len(grip) - 2])
        hull() {
            translate([0, 0, grip[i][0]])     ell_disc(grip[i][1],   grip[i][2],   o);
            translate([0, 0, grip[i+1][0]])   ell_disc(grip[i+1][1], grip[i+1][2], o);
        }
}

module head_body(o = 0) {
    r = head_r;
    hull()
        for (x = [-1, 1], y = [-1, 1], z = [0, 1])
            translate([x * (head_w/2 - r),
                       y * (head_d/2 - r),
                       z ? head_z1 - r : head_z0 + r])
                sphere(r = r - o);
}

module transition(o = 0) {
    g = grip[len(grip) - 1];
    hull() {
        translate([0, 0, g[0] - 6]) ell_disc(g[1], g[2], o);
        intersection() {
            head_body(o);
            translate([-50, -50, head_z0]) cube([100, 100, 14]);
        }
    }
}

module body(o = 0) {
    union() { grip_body(o); head_body(o); transition(o); }
}

// =====================================================================
//  Feature geometry
// =====================================================================
module oled_frame() {   // local frame: origin on front skin, +Z = into housing
    translate([0, -head_d/2, oled_z])
        rotate([-90 + oled_tilt, 0, 0])
            children();
}

module oled_cuts() {
    oled_frame() {
        // chamfered viewing window
        hull() {
            translate([0, 0, -0.5])  cube([oled_view[0] + 4, oled_view[1] + 4, 0.1], center = true);
            translate([0, 0, wall - 0.9]) cube([oled_view[0], oled_view[1], 0.1], center = true);
        }
        translate([0, 0, wall/2]) cube([oled_view[0], oled_view[1], wall * 2], center = true);
        // glass pocket (leaves 0.9 mm bezel skin)
        translate([-oled_glass[0]/2 - clr, -oled_glass[1]/2 - clr, wall - 0.9])
            cube([oled_glass[0] + 2*clr, oled_glass[1] + 2*clr, 10]);
        // PCB clearance
        translate([-oled_pcb[0]/2 - clr, -oled_pcb[1]/2 - clr - oled_glass_off,
                   wall - 0.9 + oled_glass[2]])
            cube([oled_pcb[0] + 2*clr, oled_pcb[1] + 2*clr, 10]);
    }
}

module oled_bezel_lip() {   // raised frame around window
    oled_frame()
        difference() {
            translate([0, 0, -0.6]) linear_extrude(1.2, center = true)
                offset(r = 2) square([oled_view[0] + 4, oled_view[1] + 4], center = true);
            translate([0, 0, -1]) cube([oled_view[0] + 4.2, oled_view[1] + 4.2, 4], center = true);
        }
}

module oled_retainers() {   // snap tabs that hold PCB against glass
    oled_frame()
        for (sx = [-1, 1])