# Ergonomic Handheld Controller Housing: Cycle 3

This is a parametric OpenSCAD design with these parts:
- **Body:** a sculpted, chain-hulled grip with a finger-swell profile and a widened head.
- **Display:** a raised, chamfered bezel for a 1.3" SH1106/SSD1306 128×64 OLED module, with a recessed glass pocket.
- **Thumbstick:** a flared, countersunk aperture for a KY-023/PS2-style analog stick, with mounting standoffs.
- **Battery:** an integrated 18650 chamber with spring-contact slots and a removable, screw-retained bottom door.
- **Assembly:** the shell splits into top and bottom halves along a horizontal parting line. The halves align with a tongue lip and join with M2.5 heat-set inserts. There is a USB-C charge port at the rear.

### Cycle 3 changes (vs. Cycle 2)
1. The inner cavity is end-clamped, so the nose and tail no longer punch through.
2. The OLED moved 4 mm aft and the head was widened to prevent a PCB clash with the thumbstick board.
3. The battery cradle ribs are now trimmed by the door opening, so the cell can actually be inserted.
4. A tongue-and-groove parting lip replaces the pin alignment, which reduces seam step.
5. Every part has a print-orientation export via `part=`.

```openscad
// =====================================================================
//  Ergonomic Handheld Controller Housing  -  Cycle 3
//  OLED 1.3" (SH1106/SSD1306 128x64) | KY-023 thumbstick | 18650 cell
//  Units: mm.  Set `part` then F6 -> export STL.
// =====================================================================

/* [Render] */
part = "assembly"; // [assembly, top, bottom, door, exploded]
$fn  = 48;

/* [Shell] */
wall      = 2.2;
clear     = 0.20;    // FDM fit clearance
lip_h     = 2.5;     // parting tongue height
lip_w     = 1.2;

/* [OLED 1.3in module] */
oled_y        = 54;          // centre along body
oled_win      = [30.5, 16];  // visible window (X,Y)
oled_glass    = [35.0, 24.0, 1.6];
oled_pcb      = [35.6, 33.8, 1.6];
oled_holes    = [30.6, 28.6];// hole spacing
bezel_top_z   = 18.0;
bezel_cover   = 1.2;         // plastic above glass

/* [Thumbstick KY-023] */
stick_y       = 20;
stick_ap_d    = 23;          // aperture at surface
stick_ap_d2   = 30;          // top countersink
stick_pcb_z   = 2.5;         // PCB top-face height
stick_holes   = [26.0, 20.5];

/* [Battery 18650] */
bat_d     = 18.6;
bat_len   = 70;              // incl. springs
bat_y0    = 88;
bat_z     = -3.5;            // cell axis height

/* [Fasteners] */
insert_d  = 3.6;  insert_h = 4.5;   // M2.5 heat-set
screw_d   = 2.8;  head_d   = 5.2;
boss_d    = 7.0;
bosses    = [[0,8],[0,80],[-15,76],[15,76],[0,162]];  // [x,y] (top-down)

/* [USB-C] */
usb_slot  = [9.6, 3.6];
usb_z     = 4;

// ---------------- Ergonomic profile ----------------
// [y, width(X), height(Z)]
S = [
  [  0, 14, 12],   // nose
  [  6, 34, 26],
  [ 18, 44, 34],   // thumb deck
  [ 48, 46, 34],   // display head
  [ 66, 43, 32],
  [ 80, 36, 29],   // neck / index rest
  [ 98, 40, 32],   // finger swell 1
  [117, 37, 30],
  [136, 40, 32],   // finger swell 2
  [156, 35, 28],
  [166, 16, 12]    // tail
];
N = len(S);
L = S[N-1][0];

module station(i, o) {
  s = S[i];
  y = (i==0) ? s[0]+o : (i==N-1) ? s[0]-o : s[0];
  translate([0, y, 0])
    scale([max(s[1]/2-o, .5), 1, max(s[2]/2-o, .5)])
      rotate([90,0,0]) cylinder(r=1, h=0.01, center=true, $fn=72);
}
module body(o=0) {
  for (i=[0:N-2]) hull() { station(i,o); station(i+1,o); }
}

// ---------------- helpers ----------------
module rrect(sz, r, h) {
  linear_extrude(h) offset(r) square([sz[0]-2*r, sz[1]-2*r], center=true);
}
BIG = 400;
module above(z=0) translate([-BIG/2,-BIG/2,z]) cube([BIG,BIG,BIG/2]);
module below(z=0) translate([-BIG/2,-BIG/2,z-BIG/2]) cube([BIG,BIG,BIG/2]);

// ---------------- Feature: OLED bezel ----------------
module bezel_add() {
  translate([0, oled_y, 0])
    hull() {
      translate([0,0,8])            rrect([oled_pcb[0]+8, oled_pcb[1]-4], 4, 0.1);
      translate([0,0,bezel_top_z-.1]) rrect([oled_glass[0]+6, oled_glass[1]+6], 3, 0.1);
    }
}
module bezel_cut() {
  gz = bezel_top_z - bezel_cover - oled_glass[2];
  translate([0, oled_y, 0]) {
    // chamfered viewing window
    hull() {
      translate([0,0,gz+oled_glass[2]-.01]) rrect(oled_win, 1, .01);
      translate([0,0,bezel_top_z+.01])     rrect(oled_win+[3,3], 1.5, .01);
    }
    // glass pocket
    translate([0,0,gz]) rrect([oled_glass[0]+.4, oled_glass[1]+.4], .5, oled_glass[2]+.01);
    // PCB / connector clearance underneath
    translate([0,0,-1]) rrect([oled_pcb[0]+.6, oled_pcb[1]+.6], .5, gz+1.01);
  }
}
module oled_posts() {   // heat-stake pins locate PCB against glass
  gz = bezel_top_z - bezel_cover - oled_gl