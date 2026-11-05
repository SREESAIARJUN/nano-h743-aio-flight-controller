# Ergonomic Handle Housing – Cycle 11

This is a parametric OpenSCAD design for a handheld controller. It has four main parts:

- **Contoured grip** with three finger grooves.
- **Integrated 18650 battery chamber** in the grip, closed by a screw-on end cap.
- **Tilted head** carrying a **1.3" OLED bezel** (SH1106/SSD1306 128×64 module) and a **thumbstick aperture** (KY-023 / PS2-style analog stick).

The shell splits into left and right halves at the X=0 plane, so each half prints flat on its split face with no supports.

```openscad
// =====================================================================
//  Ergonomic Handle Housing — Cycle 11
//  Battery: 18650 (in grip) | Display: 1.3" OLED | Input: analog stick
//  Split: left/right halves on X=0 plane, joined with M3 screws + pins
// =====================================================================

/* [Render] */
part = "assembly"; // [assembly, left, right, cap, all_print]
explode = 0;       // [0:1:30]

/* [Global] */
$fn  = 64;
wall = 2.2;        // shell wall thickness
tol  = 0.25;       // general fit clearance

/* [Grip] [z, width(X), depth(Y)] — ergonomic stations */
grip_st = [
    [  0, 34, 30],
    [ 15, 36, 34],
    [ 45, 40, 37],   // palm swell
    [ 75, 38, 34],
    [100, 36, 32],
    [112, 42, 40]    // flare into head
];
groove_z  = [28, 50, 72];   // finger groove centers (front, +Y)
groove_r  = 9;
groove_dp = 3.2;            // groove depth into surface

/* [Head] */
head_z0   = 100;    // head bottom
head_top  = 140;    // top surface height at y=0
head_w    = 50;
head_y0   = -30;    // rear (user side)
head_y1   = 32;     // front
head_rr   = 7;      // corner radius
tilt      = 15;     // top face tilt toward user (deg)

/* [Battery 18650] */
bat_d     = 18.6;
bat_len   = 65 + 6;   // cell + spring allowance
bat_wall  = 1.6;
bat_z0    = 3;        // above cap plug

/* [OLED 1.3in module] */
oled_pcb   = [36.0, 34.5, 1.6];
oled_glass = [35.4, 24.0, 1.6];   // glass outline
oled_view  = [30.5, 15.5];        // visible window
oled_view_off = 1.5;              // window offset toward pin header
oled_pos_y = -12;                 // on top face (user side)

/* [Thumbstick KY-023] */
stick_hole  = 25;     // aperture for cap travel
stick_pcb   = [26.5, 34.5];
stick_pos_y = 17;     // on top face (front, under thumb)
stick_stand = 9;      // standoff height below top surface

/* [Fasteners] */
m3_clear  = 3.3;
m3_head   = 6.0;
insert_d  = 4.0;      // M3 heat-set insert hole
boss_d    = 7.5;
pin_d     = 1.85;     // 1.75 filament alignment pins

// ---------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------
function top_z(y) = head_top + y * tan(tilt);

module on_top(y)   // local frame on tilted top surface
    translate([0, y, top_z(y)]) rotate([tilt, 0, 0]) children();

module ellipse(w, d) scale([w/2, d/2]) circle(r = 1);

// ---------------------------------------------------------------------
// Outer / inner body shapes (d = inset)
// ---------------------------------------------------------------------
module grip_shape(d = 0) {
    n = len(grip_st);
    for (i = [0 : n - 2]) hull() {
        for (s = [grip_st[i], grip_st[i+1]])
            translate([0, 0, s[0] + (s[0] == 0 ? -0.01 : 0)])
                linear_extrude(0.01) ellipse(s[1] - 2*d, s[2] - 2*d);
    }
}

module finger_grooves(d = 0) {
    for (z = groove_z) {
        s = lookup_depth(z);
        translate([0, s/2 + groove_r - groove_dp, z])
            rotate([0, 90, 0])
                cylinder(r = groove_r + d, h = 60, center = true);
    }
}
// interpolate grip depth at height z
function lookup_depth(z) =
    lookup(z, [for (s = grip_st) [s[0], s[2]]]);

module head_shape(d = 0) {
    r = head_rr - d;
    hull()
        for (x = [-1, 1], y = [head_y0 + head_rr, head_y1 - head_rr]) {
            translate([x*(head_w/2 - head_rr), y, head_z0 + head_rr])
                sphere(r);
            translate([x*(head_w/2 - head_rr), y, top_z(y) - head_rr])
                sphere(r);
        }
}

module body_outer() {
    difference() {
        union() { grip_shape(0); head_shape(0); }
        finger_grooves(0);
    }
}

module body_inner() {
    difference() {
        union() {
            grip_shape(wall);
            head_shape(wall);
            translate([0,0,-1]) linear_extrude(1.02)   // open bottom
                ellipse(grip_st[0][1]-2*wall, grip_st[0][2]-2*wall);
        }
        finger_grooves(wall);
    }
}

// ---------------------------------------------------------------------
// Battery chamber (tube + ribs + cap bosses)
// ---------------------------------------------------------------------
cap_screw_x = 12.5;

module battery_structure() {
    difference() {
        union() {
            translate([0,0,bat_z0])
                cylinder(d = bat_d + 2*tol + 2*bat_wall, h = bat_len);
            // ribs tying tube to the shell
            intersection() {
                grip_shape(0.1);
                union() {
                    translate([-0.8, -30, bat_z0 + 10]) cube([1.6, 60, bat_len - 20]);
                    translate([-30, -0.8, bat_z0 + 10]) cube([60, 1.6, bat_len - 20]);
                }
            }
            // cap screw bosses
            intersection() {
                grip_shape(0.1);
                for (x = [-1, 1]) translate([x*cap_screw_x, 0, 0])
                    cylinder(d = 6.5, h = 12);
            }
        }
        // cell bore (open at bottom for insertion)
        translate([0,0,-1]) cylinder(d = bat_d + 2*tol, h = bat_z0 + bat_len + 0.01);
        // wire exit at top of tube toward head
        translate([0,0,bat_z0 + bat_len - 1]) cylinder(d = 5, h = 5);
        // cap screw pilot holes (M3 self-tap 2.6 / or inserts)
        for (x = [-1, 1]) translate([x*cap_screw_x, 0, -1])
            cylinder(d = 2.6, h = 11);
    }
}

// ---------------------------------------------------------------------
// OLED bezel
// ---------------------------------------------------------------------
module oled_cut() {
    on_top(oled_pos_y) {
        // visible window with 45° chamfered bezel
        translate([0, oled_view_off, -wall - 0.01])
            hull() {
                linear_extrude(0.01) square(oled_view, center = true);
                translate([0,0,wall + 0.02])
                    linear_extrude(0.01)
                        square([oled_view[0] + 2*wall, oled_view[1] + 2*wall], center = true);
            }
        // glass pocket (from inside)
        translate([0, 0, -wall - oled_glass[2] + 0.8])
            cube([oled_glass[0] + 2*tol, oled_glass[1] + 2*tol, 2*oled_glass[2]], center = true);
        // PCB/body clearance
        translate([0, 0, -wall - 12])
            cube([oled_pcb[0] + 2*tol, oled_pcb[1] + 2*tol, 12.6 - 0.8], center = false ? 0 : true);
    }
}

module oled_raised_rim() {   // subtle raised lip around window
    on_top(oled_pos_y) translate([0, oled_view_off, -0.5])
        difference() {
            linear_extrude(1.3, scale = 0.97)
                offset(r = 2) square([oled_view[0] + 2*wall + 2, oled_view[1] + 2*wall + 2], center = true);
            translate([0,0,-1]) linear_extrude(4)
                square([oled_view[0] + 2*wall, oled_view[1] + 2*wall], center = true);
        }
}

module oled_mounts()