# Ergonomic Handle Housing: OpenSCAD Design (Cycle 8)

This is a parametric, print-ready OpenSCAD model. The handle is split vertically into left and right shells, so each half prints flat on its split face with no supports inside the grip. A separate bottom cap closes the battery bay.

**Assumed hardware** (all dimensions are parameters, so you can swap parts):

- **Display:** 1.3" OLED, SH1106 128×64, 4-pin I²C, PCB 35.4 × 33.5 mm
- **Thumbstick:** KY-023 / PS2-style module, PCB 26 × 34 mm, mounting holes 20.5 × 26.5 mm
- **Battery:** 18650 cell (Ø18.6 × 65–69 mm) with spring and button contacts
- **Charging:** USB-C charger board (TP4056-C style), exposed at the rear of the head

```openscad
// =====================================================================
//  Ergonomic Handheld Controller Housing  —  Cycle 8
//  1.3" OLED bezel | analog thumbstick aperture | 18650 battery grip
//  Units: mm.  Render with F6, then export each `part` separately.
// =====================================================================

/* [Output] */
part = "assembly"; // [assembly, left, right, cap, exploded]

/* [Quality] */
$fn = 64;
slices = 24;                 // grip profile resolution

/* [Shell] */
wall      = 2.4;             // 6 perimeters @ 0.4 nozzle
clearance = 0.25;            // fit tolerance for mating parts

/* [Grip] */
grip_h    = 105;
grip_w    = 34;              // side-to-side (X)
grip_d    = 40;              // front-to-back (Y)
swell     = 0.12;            // palm swell factor
groove_z  = [30, 51, 72];    // finger groove heights
groove_r  = 9;
groove_depth = 2.0;

/* [Head] */
head_w    = 54;
head_d    = 72;
head_h    = 30;
head_r    = 8;               // corner radius
head_tilt = 18;              // face tilt toward user (deg)
head_z    = grip_h - 10;
head_y    = -4;

/* [Battery 18650] */
bat_d       = 18.8;
bat_l       = 70;            // cell + spring compression
bat_y       = -3;            // offset toward front
cradle_wall = 1.6;
bat_z0      = 4;

/* [OLED 1.3in SH1106] */
oled_pcb     = [35.6, 33.8, 1.6];
oled_glass   = [35.0, 24.0, 1.8];   // glass panel footprint
oled_view    = [30.5, 16.0];        // visible window
oled_view_dy = -1.2;                // window offset on glass
oled_holes   = [30.4, 28.4];        // M2 hole pitch
oled_y       = 15;                  // centre on head face (rear zone)
bezel_chamfer= 1.6;

/* [Thumbstick KY-023] */
stick_ap     = 23;           // aperture for gimbal travel (+/-25 deg)
stick_dish_r = 20;           // ergonomic dish sphere radius
stick_dish_d = 1.8;          // dish depth
stick_pcb    = [26, 34];
stick_holes  = [20.5, 26.5];
stick_y      = -18;          // centre on head face (front zone)
stick_top_to_pcb = 17;       // gimbal top -> PCB underside

/* [Fasteners M3 self-tap] */
boss_d   = 7;
pilot_d  = 2.5;
clear_d  = 3.4;
head_cb_d= 6.2;

/* [Ports] */
usbc = [9.6, 3.8];           // USB-C opening

// =====================================================================
//  Profile functions
// =====================================================================
function gw(z) = grip_w * (0.9 + swell * sin(180 * z / grip_h));
function gd(z) = grip_d * (0.92 + swell * 0.8 * sin(180 * z / grip_h));

// =====================================================================
//  Primitive helpers
// =====================================================================
module ell_slice(z, w, d)
    translate([0, 0, z]) scale([w/2, d/2, 1]) cylinder(r = 1, h = 0.01);

module rbox(size, r) {               // centred XY, base at z=0
    hull() for (x = [-1, 1], y = [-1, 1], z = [r, size.z - r])
        translate([x*(size.x/2 - r), y*(size.y/2 - r), z]) sphere(r);
}

module head_xf()
    translate([0, head_y, head_z]) rotate([-head_tilt, 0, 0]) children();

module on_face()                     // origin on outer head top surface
    head_xf() translate([0, 0, head_h]) children();

// =====================================================================
//  Grip solids
// =====================================================================
module finger_grooves(extra = 0)
    for (z = groove_z)
        translate([0, -gd(z)/2 - groove_r + groove_depth, z])
            rotate([0, 90, 0]) cylinder(r = groove_r + extra, h = 80, center = true);

module grip_outer()
    difference() {
        for (i = [0 : slices - 1]) {
            z0 = grip_h * i / slices;  z1 = grip_h * (i + 1) / slices;
            hull() { ell_slice(z0, gw(z0), gd(z0)); ell_slice(z1, gw(z1), gd(z1)); }
        }
        finger_grooves();
    }

module grip_inner()
    difference() {
        for (i = [0 : slices - 1]) {
            z0 = grip_h * i / slices - (i == 0 ? 1 : 0);
            z1 = grip_h * (i + 1) / slices + (i == slices - 1 ? 8 : 0);
            hull() {
                ell_slice(z0, gw(max(z0,0)) - 2*wall, gd(max(z0,0)) - 2*wall);
                ell_slice(z1, gw(min(z1,grip_h)) - 2*wall, gd(min(z1,grip_h)) - 2*wall);
            }
        }
        finger_grooves(wall);        // keeps wall thickness behind grooves
    }

// =====================================================================
//  Head solids
// =====================================================================
module head_outer() head_xf() rbox([head_w, head_d, head_h], head_r);
module head_inner()
    head_xf() translate([0, 0, wall])
        rbox([head_w - 2*wall, head_d - 2*wall, head_h - 2*wall], head_r - wall);

module body_outer() union() { grip_outer(); head_outer(); }
module body_inner() union() { grip_inner(); head_inner(); }

// =====================================================================
//  OLED bezel
// =====================================================================
module oled_cut()
    on_face() translate([0, oled_y, 0]) {
        // chamfered viewing window through the face
        hull() {
            translate([0, oled_view_dy, -wall - 0.01])
                linear_extrude(0.01) square(oled_view, center = true);
            translate([0, oled_view_dy, 0.01])
                linear_extrude(0.01)
                    square([oled_view.x + 2*bezel_chamfer,
                            oled_view.y + 2*bezel_chamfer], center = true);
        }
        // glass pocket from inside (leaves a 0.8 mm bezel lip)
        translate([0, 0, -wall - 5 + (wall - 0.8)])
            linear_extrude(5)
                square([oled_glass.x + 2*clearance, oled_glass.y + 2*clearance], center = true);
    }

module oled_mount()                  // standoffs + retention pegs
    on_face() translate([0, oled_y, -wall]) {
        so_h = (wall - 0.8) + oled_glass.z - (wall - 0.8) + 0.2;   // seats PCB on glass
        for (x = [-1, 1], y = [-1, 1])
            translate([x*oled_holes.x/2, y*oled_holes.y/2, -oled_glass.z - 0.2]) {
                cylinder(d = 5, h = oled_glass.z + 0.2 + 0.01);
                translate([0, 0, -oled_pcb.z - 0.6])          // M2 locating peg
                    cylinder(d = 1.8, h = oled_pcb.z + 0.6);
            }
    }

// =====================================================================
//  Thumbstick aperture
// =====================================================================
module stick_cut()
    on_face() translate([0, stick_y, 0]) {
        // ergonomic dish around stick
        translate([0