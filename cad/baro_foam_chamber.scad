// Nano H743 AIO - DPS310 (U4) barometer foam chamber / shroud cap   (Revision 12)
//
// Single source of truth for the baro chamber geometry.  analysis/baro_acoustic_analysis.py
// parses the top-level "name = expr;" assignments below (do not move them into modules,
// keep one assignment per line) and evaluates them; derived values are computed there
// from these same expressions, so no number is duplicated.
//
// Frames
//   KiCad board frame : x right, y DOWN, origin at board top-left corner (kicad_pcb).
//   SCAD world frame  : X = kicad x, Y = board_size - kicad y (Y up/north), Z = 0 on the
//                       PCB TOP copper/mask surface, +Z away from the board.
//   Cap local frame   : origin at the cap footprint centre on the PCB, +X east (toward the
//                       board edge x = 36), +Y north.  The vent wall is the EAST wall.
//
// Parts (select with -D 'part="..."'):
//   "cap"        printed shroud (SLA high-temperature resin, see notes), bonded to the PCB
//   "foam_block" pre-cut foam block (cut template, free state, i.e. before compression)
//   "air"        chamber air volume (cavity minus U4 body/passives/ledges) - analysis only
//   "vent_air"   vent labyrinth air volume - analysis only
//   "assembly"   preview: board corner, MOSFET courtyards, grommet keep-out, U4, cap, foam
//
// Material / process notes
//   Cap: SLA high-temperature resin (HDT @0.45 MPa >= 120 C, e.g. Formlabs High Temp or
//        Loctite 3D 3955) because the adjacent Q11 high-side FET runs ~87-110 C at rating
//        and the 0.35 mm vent labyrinth is below MJF/FDM resolution.  Not TPU95A: FDM cannot
//        hold a 0.35 mm closed channel, and TPU creeps at board temperature.
//        The cap must be OPAQUE (black-pigmented resin, or the translucent amber High Temp
//        resin painted matt black after the hydrophobic dip): with the 0.8 mm air gap the
//        U4 lid port sees the cavity walls directly (light shielding, see analysis).
//        Post-process: IPA flush of the labyrinth (syringe), UV cure, hydrophobic
//        fluoropolymer dip (contact angle > 100 deg) so water is REPELLED from the channel.
//   Bond: continuous bead of neutral-cure (alkoxy) RTV or UV acrylic on the foot.  NO acetoxy
//        RTV (acetic acid outgassing attacks the DPS310).  The bead must be leak-free: any
//        leak is a parallel vent that bypasses the labyrinth (see analysis).
//   Foam: open-cell polyether PU, black (carbon-black, opaque), 45 PPI, 28 kg/m^3,
//        flow resistivity ~4 kPa*s/m^2 (swept 2..8 kPa*s/m^2 in the analysis).

/* [Selection] */
part = "assembly";          // "cap" | "foam_block" | "air" | "vent_air" | "assembly"
$fn = 48;                   // facets for round features (grommet / holes)

/* [Board, from tools/gen_power_stage_pcb.py / kicad_pcb] */
board_size = 36.0;          // mm, square outline
board_t = 1.614;            // mm, JLC061612-1080 stackup
mount_pitch = 30.5;         // mm, M3 hole pattern
grommet_keepout_r = 3.25;   // mm, MountingHole_3.2mm_M3_Grommet_Keepout courtyard radius
stack_clear = 0.25;         // mm, extra radial clearance cap <-> grommet/standoff/nut
stack_free_h = 8.5;         // mm, integration spec: free height above PCB top to the next stack board
stack_clear_z = 0.5;        // mm, vertical clearance cap roof <-> next board (vibration / grommet sag)

/* [Neighbouring top-side courtyards, from kicad_pcb (KiCad frame, centre + size)] */
q4_cx_k = 26.45;            // mm, Q4 courtyard centre (N edge)
q4_cy_k = 5.675;
q4_w = 5.2;                 // mm, courtyard X extent
q4_l = 7.45;                // mm, courtyard Y extent
q11_cx_k = 30.325;          // mm, Q11 courtyard centre (E edge, high-side FET next to the cap)
q11_cy_k = 15.65;
q11_w = 7.45;
q11_l = 5.2;
court_h = 1.0;              // mm, preview height of the courtyard blocks
court_clear = 0.2;          // mm, min cap-to-courtyard gap (bond bead squeeze-out)

/* [U4 site (reservation - the PCB currently holds only the power stage)] */
cap_cx_k = 32.55;           // mm, cap footprint centre, KiCad x  (NE corner, top face F.Cu)
cap_cy_k = 9.55;            // mm, cap footprint centre, KiCad y
u4_dx = -0.98;              // mm, U4 centre offset from cap centre, local +X (east)
u4_dy = 0.0;                // mm, U4 centre offset from cap centre, local +Y (north)

/* [DPS310 U4, Infineon LGA-8 2.0 x 2.5 x 1.0] */
u4_w = 2.0;                 // mm, package X
u4_l = 2.5;                 // mm, package Y
u4_h = 1.0;                 // mm, package height (lid with vent hole on top)
pass_w = 0.5;               // mm, 0402 passive X extent (rotated: long axis along Y)
pass_l = 1.0;               // mm, 0402 passive Y extent (C24/C25/C26 inside the chamber)
pass_h = 0.55;              // mm, 0402 passive height (max)
n_pass = 3;                 // count of 0402 decoupling caps inside the chamber

/* [Cap shell] */
cap_w = 6.2;                // mm, outer footprint X
cap_l = 6.5;                // mm, outer footprint Y
cap_h = 8.0;                // mm, outer height above PCB top (needs >= 8.5 mm free stack height)
wall_t = 0.8;               // mm, N/S/W wall thickness
vent_wall_t = 2.1;          // mm, east wall thickness (holds the two-plane labyrinth)
roof_t = 0.8;               // mm, roof thickness
corner_r = 0.6;             // mm, outer vertical edge radius
foot_relief_h = 0.15;       // mm, glue-bead relief under the wall (0 = flat foot)

/* [Foam support and sensor air gap] */
air_gap = 0.8;              // mm, clear air between U4 lid and foam underside (foam never touches)
ledge_w = 0.4;              // mm, foam ledge depth from N/S inner walls
ledge_h = 0.4;              // mm, ledge rib height
foam_fit = 0.2;             // mm, foam oversize per side (compression retention), X/Y
foam_fit_z = 0.2;           // mm, foam oversize in Z (compressed by the roof)
foam_ppi = 45;              // pores per inch
foam_density = 28;          // kg/m^3
foam_sigma = 4000;          // Pa*s/m^2 flow resistivity, nominal (analysis sweeps x0.5..x2)
foam_solid_density = 1100;  // kg/m^3 PU polymer (porosity = 1 - density/solid density)

/* [Materials for the analysis] */
resin_hdt = 120;            // C, cap resin heat deflection temperature @ 0.45 MPa (minimum)
coat_theta = 100;           // deg, water contact angle of the fluoropolymer-dipped channel (minimum)

/* [Vent labyrinth: square-channel zig-zag in two interleaved planes inside the east wall] */
// Lane k runs in plane (k % 2) and alternates direction in Y; at each lane end a short
// X crossover joins the two planes.  Consecutive lanes therefore sit in different planes,
// so the vertical web rule applies between lanes k and k+2 (2 * vent_pitch apart).
vent_a = 0.35;              // mm, channel square side (SLA, +-vent_tol)
vent_tol = 0.03;            // mm, channel size tolerance (analysis corner)
vent_lanes = 17;            // number of sloped lanes (odd -> ports at opposite Y ends)
vent_plane_sep = 0.85;      // mm, X distance between the two channel planes (centre-centre)
vent_skin = 0.5;            // mm, min solid skin around channel (to roof / corners / foot)
vent_z_in = 6.8;            // mm, inner port centre height (top of foam region)
vent_z_out = 0.6;           // mm, outer port centre height (lowest point -> self-draining)

/* [Derived - evaluated identically by the analysis] */
cav_x0 = -cap_w / 2 + wall_t;                 // mm, cavity west face (local)
cav_x1 = cap_w / 2 - vent_wall_t;             // mm, cavity east face
cav_y0 = -cap_l / 2 + wall_t;                 // mm
cav_y1 = cap_l / 2 - wall_t;                  // mm
cav_h = cap_h - roof_t;                       // mm, cavity height (PCB to roof)
foam_z0 = u4_h + air_gap;                     // mm, foam underside
foam_t = cav_h - foam_z0;                     // mm, installed foam thickness
vent_xc = cav_x1 + vent_wall_t / 2;           // mm, labyrinth mid-plane X (between the planes)
vent_xa = vent_xc - vent_plane_sep / 2;       // mm, plane A (inner) X
vent_xb = vent_xc + vent_plane_sep / 2;       // mm, plane B (outer) X
vent_y_end = cap_l / 2 - corner_r - vent_skin - vent_a / 2;   // mm, lane end |Y|
vent_pitch = (vent_z_in - vent_z_out) / vent_lanes;           // mm, vertical drop per lane
vent_web = 2 * vent_pitch - vent_a;           // mm, solid web between lanes k, k+2 (same plane)
vent_xweb = vent_plane_sep - vent_a;          // mm, solid web between the two planes
vent_skin_x = vent_xa - vent_a / 2 - cav_x1;  // mm, skin to the cavity face (= outer skin)
vent_lane_len = sqrt(pow(2 * vent_y_end, 2) + pow(vent_pitch, 2));  // mm
vent_slope_deg = atan(vent_pitch / (2 * vent_y_end));        // deg, descent of each lane
vent_stub_in = vent_xa - cav_x1;              // mm, plane A -> cavity face
vent_stub_out = cap_w / 2 - (vent_lanes % 2 == 1 ? vent_xa : vent_xb);   // mm, last plane -> outer face
vent_n_cross = vent_lanes - 1;                // count of X crossovers between lanes
vent_len = vent_lanes * vent_lane_len + vent_n_cross * vent_plane_sep + vent_stub_in + vent_stub_out;   // mm, centreline
u4_x0 = u4_dx - u4_w / 2;                     // mm
u4_x1 = u4_dx + u4_w / 2;                     // mm
cap_x0_k = cap_cx_k - cap_w / 2;              // mm, KiCad footprint extents
cap_x1_k = cap_cx_k + cap_w / 2;
cap_y0_k = cap_cy_k - cap_l / 2;
cap_y1_k = cap_cy_k + cap_l / 2;

/* [Sanity asserts] */
assert(u4_x1 + 0.25 + pass_w <= cav_x1 - 0.2, "0402 column east of U4 must clear the vent wall by 0.2 mm");
assert(cav_y1 - cav_y0 >= u4_l + 1.0, "cavity too short for U4");
assert(u4_x0 >= cav_x0 + 0.3 && u4_x1 <= cav_x1 - 0.3, "U4 must clear the cavity walls by 0.3 mm");
assert(abs(u4_dy) + u4_l / 2 <= cav_y1 - ledge_w - 0.2, "U4 under a ledge");
assert(air_gap >= 0.5, "foam-to-sensor air gap must be >= 0.5 mm");
assert(foam_t >= 3.0, "foam thinner than 3 mm");
assert(vent_web >= 0.35, "labyrinth web between lanes too thin to print");
assert(vent_z_in + vent_a / 2 <= cav_h - 0.2, "inner port must open below the roof");
assert(vent_z_in - vent_a / 2 >= foam_z0 + 0.5, "inner port must open into the foam region");
assert(vent_z_out - vent_a / 2 >= foot_relief_h + 0.2, "outer port too close to the foot");
assert(vent_z_in + vent_a / 2 + vent_skin <= cap_h, "labyrinth breaks through the roof");
assert(vent_lanes % 2 == 1, "use an odd lane count so the ports are at opposite ends");
assert(vent_slope_deg >= 5, "lanes must descend >= 5 deg toward the outer port (drain)");
assert(vent_xweb >= 0.4, "web between labyrinth planes < 0.4 mm");
assert(vent_skin_x >= 0.4, "labyrinth skin to cavity / outside < 0.4 mm");
assert(abs(cap_w / 2 - vent_xb - vent_a / 2 - vent_skin_x) < 1e-6, "labyrinth not centred in the vent wall");
assert(wall_t >= 0.6 && roof_t >= 0.6, "SLA wall minimum 0.6 mm");
assert(cap_x1_k <= board_size - 0.3, "cap overhangs the east board edge");
assert(cap_y0_k >= 0.3, "cap overhangs the north board edge");
// grommet keep-out of the NE hole (KiCad (board-2.75, 2.75)): nearest point of the footprint
hole_k = (board_size - mount_pitch) / 2;      // mm, hole inset from edges (2.75)
assert(sqrt(pow(max(cap_x0_k - (board_size - hole_k), 0, (board_size - hole_k) - cap_x1_k), 2)
          + pow(max(cap_y0_k - hole_k, 0, hole_k - cap_y1_k), 2))
       >= grommet_keepout_r + stack_clear, "cap intrudes into the grommet / standoff keep-out");
assert(cap_x0_k >= q4_cx_k + q4_w / 2 + court_clear || cap_y0_k >= q4_cy_k + q4_l / 2 + court_clear,
       "cap overlaps the Q4 courtyard");
assert(cap_y1_k <= q11_cy_k - q11_l / 2 - court_clear, "cap overlaps the Q11 courtyard");
assert(cap_h + stack_clear_z <= stack_free_h, "cap taller than the free stack height");

echo(str("BAROCAP cav=", cav_x1 - cav_x0, "x", cav_y1 - cav_y0, "x", cav_h,
         " foam_t=", foam_t, " vent_len=", vent_len, " vent_pitch=", vent_pitch,
         " vent_slope_deg=", vent_slope_deg));

// ------------------------------------------------------------------------------------------
module rounded_box(w, l, h, r) {
    hull() for (sx = [-1, 1], sy = [-1, 1])
        translate([sx * (w / 2 - r), sy * (l / 2 - r), 0]) cylinder(r = r, h = h);
}

// labyrinth centreline: lane k goes from vent_pt0(k) to vent_pt1(k) in plane (k % 2),
// descending vent_pitch; lane 0 starts at the inner port end (Y = +vent_y_end, plane A).
function vent_px(k) = (k % 2 == 0) ? vent_xa : vent_xb;
function vent_py(k) = ((k % 2 == 0) ? 1 : -1) * vent_y_end;
function vent_pt0(k) = [vent_px(k), vent_py(k), vent_z_in - k * vent_pitch];
function vent_pt1(k) = [vent_px(k), -vent_py(k), vent_z_in - (k + 1) * vent_pitch];

module seg(p, q, a) {
    // square-section bar of side a from p to q (p, q at the same X)
    d = q - p;
    len = norm(d);
    ang = atan2(d[2], d[1]);
    translate(p) rotate([ang, 0, 0]) translate([-a / 2, 0, -a / 2]) cube([a, len, a]);
    translate(p - [a / 2, a / 2, a / 2]) cube([a, a, a]);   // joint
    translate(q - [a / 2, a / 2, a / 2]) cube([a, a, a]);   // joint
}

module vent_channel(a = vent_a) {
    for (k = [0 : vent_lanes - 1]) {
        seg(vent_pt0(k), vent_pt1(k), a);
        if (k < vent_lanes - 1) {   // X crossover to the next plane at this lane end
            e = vent_pt1(k);
            translate([min(vent_xa, vent_xb) - a / 2, e[1] - a / 2, e[2] - a / 2])
                cube([vent_plane_sep + a, a, a]);
        }
    }
    // inner stub: plane A -> cavity (west face) at the lane-0 start
    p0 = vent_pt0(0);
    translate([cav_x1 - 0.05, p0[1] - a / 2, p0[2] - a / 2]) cube([vent_xa - cav_x1 + 0.05 + a / 2, a, a]);
    // outer stub: last plane -> outside (east face) at the last vertex
    pe = vent_pt1(vent_lanes - 1);
    translate([pe[0] - a / 2, pe[1] - a / 2, pe[2] - a / 2]) cube([cap_w / 2 - pe[0] + a / 2 + 0.05, a, a]);
}

module ledges() {
    for (sy = [-1, 1])
        translate([cav_x0, sy > 0 ? cav_y1 - ledge_w : cav_y0, foam_z0 - ledge_h])
            cube([cav_x1 - cav_x0, ledge_w, ledge_h]);
}

module cavity() {
    translate([cav_x0, cav_y0, -0.01]) cube([cav_x1 - cav_x0, cav_y1 - cav_y0, cav_h + 0.01]);
}

module cap() {
    difference() {
        rounded_box(cap_w, cap_l, cap_h, corner_r);
        cavity();
        vent_channel();
        // glue-bead relief groove along the foot centreline (keeps a bead captive)
        if (foot_relief_h > 0)
            difference() {
                translate([0, 0, -0.01]) rounded_box(cap_w - 0.3, cap_l - 0.3, foot_relief_h + 0.01, corner_r);
                translate([0, 0, -0.02]) rounded_box(cap_w - 2 * wall_t + 0.3, cap_l - 2 * wall_t + 0.3,
                                                     foot_relief_h + 0.03, 0.1);
            }
    }
    ledges();
}

module u4_body() { translate([u4_x0, u4_dy - u4_l / 2, 0]) cube([u4_w, u4_l, u4_h]); }

module passives() {
    // C24/C25/C26 0402 east of U4, stacked in Y
    for (i = [0 : n_pass - 1])
        translate([u4_x1 + 0.25, -(n_pass * pass_l + (n_pass - 1) * 0.3) / 2 + i * (pass_l + 0.3), 0])
            cube([pass_w, pass_l, pass_h]);
}

module foam_block(free_state = true) {
    fx = (cav_x1 - cav_x0) + (free_state ? 2 * foam_fit : 0);
    fy = (cav_y1 - cav_y0) + (free_state ? 2 * foam_fit : 0);
    fz = foam_t + (free_state ? foam_fit_z : 0);
    if (free_state) cube([fx, fy, fz]);
    else translate([cav_x0, cav_y0, foam_z0]) cube([fx, fy, fz]);
}

module air() {
    difference() {
        translate([cav_x0, cav_y0, 0]) cube([cav_x1 - cav_x0, cav_y1 - cav_y0, cav_h]);
        u4_body();
        passives();
        ledges();
    }
}

// world placement of the cap local frame (on the top face)
module at_site() { translate([cap_cx_k, board_size - cap_cy_k, 0]) children(); }

module assembly() {
    // board corner (NE quadrant) below Z = 0
    color("darkgreen", 0.6) translate([18, 18, -board_t]) cube([18, 18, board_t]);
    // NE grommet keep-out
    color("red", 0.3) translate([board_size - (board_size - mount_pitch) / 2, board_size - (board_size - mount_pitch) / 2, 0])
        cylinder(r = grommet_keepout_r, h = 3);
    // Q4 (N edge) and Q11 (E edge) courtyards, top face (from kicad_pcb)
    color("gray", 0.5) {
        translate([q4_cx_k - q4_w / 2, board_size - (q4_cy_k + q4_l / 2), 0]) cube([q4_w, q4_l, court_h]);
        translate([q11_cx_k - q11_w / 2, board_size - (q11_cy_k + q11_l / 2), 0]) cube([q11_w, q11_l, court_h]);
    }
    at_site() {
        color("silver") u4_body();
        color("tan") passives();
        color("dimgray", 0.9) foam_block(false);
        color("white", 0.35) cap();
        color("blue") vent_channel();
    }
}

if (part == "cap") cap();
else if (part == "foam_block") foam_block(true);
else if (part == "air") air();
else if (part == "vent_air") vent_channel();
else assembly();
