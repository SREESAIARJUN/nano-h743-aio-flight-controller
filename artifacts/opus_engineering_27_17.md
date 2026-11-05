# Ergonomic Handle Housing: Cycle 27

This is a parametric OpenSCAD design. It is a split-shell handheld controller with these features:

- **Contoured grip** with finger undulations and a palm swell.
- **Tilted thumb deck** (head).
- **1.3" OLED bezel** for the SH1106/SSD1306 128×64 module, with a chamfered window, glass ledge and PCB pocket.
- **Thumbstick aperture** for the KY-023 / PS2-style module, with a dished chamfer and standoffs.
- **Integrated 18650 battery chamber** with a bayonet end cap, spring recess and wire channel.
- **Left/right clamshell split** with M3 screw bosses, plus a USB-C charge port.

The module dimensions in the code are typical values. Measure your actual parts before printing.

```openscad
// =====================================================================
//  Ergonomic Handheld Controller Housing  —  Cycle 27
//  Features: 18650 battery chamber, 1.3" OLED bezel, thumbstick aperture
//  Units: mm
// =====================================================================

/* [Render] */
part = "assembly"; // [assembly, left, right, cap, print_all]
explode = 0;       // [0:1:40]

/* [Global] */
$fn  = 72;
wall = 2.4;          // shell wall thickness
tol  = 0.25;         // fit clearance

/* [Grip profile: z, half-width X, half-depth Y, Y-offset] */
// Undulating radii create finger rests; the swell at the top fills the palm
grip = [
    [  0, 17.0, 21.0, 0.0],
    [ 12, 18.0, 23.0, 1.0],
    [ 30, 16.3, 21.0, 2.5],   // finger groove 1
    [ 48, 17.6, 22.8, 3.0],
    [ 66, 16.3, 21.0, 3.0],   // finger groove 2
    [ 86, 18.2, 23.2, 2.0],
    [104, 19.5, 24.5, 0.0]    // palm swell / neck
];

/* [Head / thumb deck] */
head_W    = 52;     // width (X)
head_L    = 92;     // length (Y)
head_H    = 24;     // thickness
head_r    = 7;      // corner radius
head_tilt = 18;     // deck tilt toward the user (deg)
head_pos  = [0, -14, 122];

/* [1.3" OLED module (SH1106 128x64)] */
oled_pcb    = [36.0, 34.5, 1.6];
oled_glass  = [35.4, 23.5, 1.6];
oled_view   = [30.5, 16.0];    // visible window
oled_vofs   = 1.5;             // window offset from glass centre (Y)
oled_y      = 20;              // position on deck (local Y)
bezel_rise  = 1.2;             // raised bezel lip
bezel_w     = 3.0;

/* [Thumbstick (KY-023 / PS2 type)] */
stick_ap     = 27;             // aperture diameter at surface
stick_ap_in  = 22;             // aperture diameter at inner face
stick_pcb    = [34, 26.5];
stick_holes  = [26.6, 20.4];   // mounting hole spacing
stick_stand  = 9.5;            // standoff length below deck inner face
stick_y      = -22;

/* [Battery: 18650] */
cell_d    = 18.6;
cell_L    = 65.5;
spring_L  = 4.5;
tube_id   = cell_d + 0.8;
tube_wall = 1.6;
tube_z0   = 0;
tube_L    = cell_L + spring_L + 3;

/* [Fasteners: M3] */
m3_clear = 3.4;
m3_pilot = 2.6;
m3_head  = 6.0;
boss_d   = 7.5;

// ---------------------------------------------------------------------
//  Primitive helpers
// ---------------------------------------------------------------------
module slice(s, o=0)
    translate([0, s[3], s[0]])
        linear_extrude(0.01)
            scale([max(s[1]-o, 0.5), max(s[2]-o, 0.5)]) circle(1);

module rbox(size, r) {
    r2 = max(r, 0.5);
    hull() for (x=[-1,1], y=[-1,1], z=[-1,1])
        translate([x*(size[0]/2-r2), y*(size[1]/2-r2), z*(size[2]/2-r2)])
            sphere(r2, $fn=32);
}

module head_xf() { translate(head_pos) rotate([-head_tilt, 0, 0]) children(); }

module head_body(o=0)
    head_xf() rbox([head_W-2*o, head_L-2*o, head_H-2*o], head_r-o);

// ---------------------------------------------------------------------
//  Outer / inner volumes (o = inward offset)
// ---------------------------------------------------------------------
module body(o=0) {
    // grip: hull consecutive slices for a smooth lofted surface
    for (i=[0:len(grip)-2]) hull() { slice(grip[i], o); slice(grip[i+1], o); }
    // neck blend: grip top into head
    hull() { slice(grip[len(grip)-1], o); head_body(o); }
    head_body(o);
}

// ---------------------------------------------------------------------
//  Internal features
// ---------------------------------------------------------------------
module battery_tube() {
    translate([0, 0, tube_z0])
        cylinder(d=tube_id+2*tube_wall, h=tube_L+tube_wall);
}

module battery_void() {
    translate([0, 0, tube_z0-1]) cylinder(d=tube_id, h=tube_L+1);
    // + contact slot and wire channel up into the head
    translate([0, 0, tube_z0+tube_L-1]) {
        translate([-4, -1.5, 0]) cube([8, 3, tube_wall+2]);
        cylinder(d=5, h=40);
    }
}

// Bayonet L-slots inside the tube mouth
module bayonet_slots() {
    for (a=[0,180]) rotate([0,0,a]) {
        translate([tube_id/2-0.5, -1.75, -1]) cube([1.6, 3.5, 7]);  // vertical entry
        translate([0,0,4]) rotate([0,0,-1])
            rotate_extrude(angle=-35, $fn=96)
                translate([tube_id/2-0.5, 0]) square([1.6, 2.4]);   // locking arc
    }
}

// Boss locations in the YZ plane (screw axis = X, split plane x = 0)
grip_bosses = [[-15, 22], [16, 20], [-15, 90], [16, 92]];
function head_boss_pts() = [for (y=[-head_L/2+9, head_L/2-9], s=[-1]) [y, 0]];

module boss_cyls(d) {
    for (p=grip_bosses) translate([0, p[0], p[1]]) rotate([0,90,0])
        cylinder(d=d, h=60, center=true);
    head_xf() for (y=[-head_L/2+8, head_L/2-8, 0])
        translate([0, y, -head_H/2+wall+4]) rotate([0,90,0])
            cylinder(d=d, h=60, center=true);
}

module bosses() intersection() { boss_cyls(boss_d); body(0.01); }

// Thumbstick standoffs hanging from the deck underside
module stick_standoffs() head_xf()
    translate([0, stick_y, head_H/2-wall])
        for (x=[-1,1], y=[-1,1])
            translate([x*stick_holes[1]/2, y*stick_holes[0]/2, -stick_stand])
                difference() {
                    cylinder(d=6, h=stick_stand+0.5);
                    translate([0,0,-1]) cylinder(d=m3_pilot-0.4, h=stick_stand); // M2.5 self-tap
                }

// OLED retention clips / ledge frame under the deck
module oled_frame() head_xf()
    translate([0, oled_y, head_H/2-wall-2.2])
        difference() {
            translate([0,0,0.1]) cube([oled_pcb[0]+4, oled_pcb[1]+4, 4.4], center=true);
            cube([oled_pcb[0]+2*tol, oled_pcb[1]+2*tol, 6], center=true);
        }

// ---------------------------------------------------------------------
//  Cut features
// ---------------------------------------------------------------------
module oled_cuts() head_xf() translate([0, oled_y, head_H/2]) {
    // visible window with 45° chamfer (zero-parallax look)
    translate([0, oled_vofs, 0])
        hull() {
            translate([0,0,-wall-0.1]) cube([oled_view[0], oled_view[1], 0.01], center=true);
            translate([0,0, bezel_rise+0.01])
                cube([oled_view[0]+2*(wall+bezel_rise), oled_view[1]+2*(wall+bezel_rise), 0.01], center=true);
        }
    // glass ledge pocket (from inside)
    translate([0,0,-wall-oled_glass[2]/2+0.8])
        cube([oled_glass[0]+2*tol, oled_glass[1]+2*tol, oled_glass[2]+1.6], center=true);
}

module oled_bezel_add() head_xf() translate([0, oled_y+oled_vofs, head_H/2-0.5])
    hull() {
        cube([oled_view[0]+2*bezel_w+6, oled_view[1]+2*bezel_w+6, 1], center=true);
        translate([0,0,bezel_rise])
            cube([oled_view[0]+2*bezel_w, oled_view[1]+2*bezel_w, 1], center=true);
    }

module stick_cuts() head_xf() translate([0, stick_y, head_H/2]) {
    