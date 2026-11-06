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
//   "septum"     internal ePTFE septum die-cut (bonded under the ledge ring, above U4)
//   "air"        chamber air volume (cavity minus U4 body/passives/ledges/septum) - analysis only
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
//        fluoropolymer dip (secondary barrier only - see water notes below).
//   Vent membrane (Rev 12): die-cut oleophobic ePTFE / PET-nonwoven adhesive vent laminate
//        bonded in the seat recess on the EAST outer face, covering the plenum into which the
//        labyrinth outer port opens.  Class: Gore automotive-electronics high-airflow
//        adhesive vent laminate (LM9xx: 65.5 l/h/cm^2 @ 70 mbar, WEP >= 600 mbar / 30 s,
//        -40..+150 C, gore-auto-electronics-adhesive-datasheet-us.pdf 2025-09) or Donaldson
//        Tetratex ePTFE equivalent.  Procurement spec = mem_airflow (typ, analysis sweeps
//        x0.5..x2 on resistance), mem_wep_min, mem_t_max below.  The membrane is the WATER
//        barrier.  The labyrinth behind it is the light baffle and a mechanical guard if the
//        membrane is punctured; it is NOT self-draining (capillary pinning >> gravity at
//        a = 0.35 mm), so a breached membrane means replace the cap and dry it out.
//        ePTFE laminates are white/translucent: light shielding relies on the labyrinth.
//   Bond: continuous bead of neutral-cure (alkoxy) RTV or UV acrylic on the foot.  NO acetoxy
//        RTV (acetic acid outgassing attacks the DPS310).  The bead must be leak-free: any
//        leak is a parallel vent that bypasses the membrane and labyrinth (see analysis).
//   Foam (Rev 12): open-cell MELAMINE foam, black-dyed (opaque), 10 kg/m^3, fine cell
//        (~0.15 mm), flow resistivity ~10 kPa*s/m^2 (BASF Basotect class, >= 10 kPa*s/m^2),
//        continuous service to ~150 C.  Polyether PU (~100 C) was dropped because the cap
//        foot reaches ~111 C in the burst scenarios.  Die-cut, blown clean (friable edges).

/* [Selection] */
part = "assembly";          // "cap" | "foam_block" | "membrane" | "septum" | "air" | "vent_air" | "assembly"
$fn = 48;                   // facets for round features (grommet / holes)

/* [Board, from tools/gen_power_stage_pcb.py / kicad_pcb] */
board_size = 36.0;          // mm, square outline
board_t = 1.614;            // mm, JLC061612-1080 stackup
mount_pitch = 30.5;         // mm, M3 hole pattern
grommet_keepout_r = 3.25;   // mm, MountingHole_3.2mm_M3_Grommet_Keepout courtyard radius
stack_clear = 0.25;         // mm, extra radial clearance cap <-> grommet/standoff/nut
stack_free_h = 8.5;         // mm, integration spec: free height above PCB top to the next stack board
stack_clear_z = 0.5;        // mm, vertical clearance cap roof <-> next board (vibration / grommet sag)
edge_clear_min = 0.3;       // mm, min cap footprint to board outline (depanel / router tolerance)

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
pass_gap = 0.25;            // mm, U4 body to the 0402 column (X)
pass_pitch_gap = 0.3;       // mm, gap between stacked 0402s (Y)
pass_wall_clear = 0.2;      // mm, min 0402 column to vent-wall cavity face
u4_wall_clear = 0.3;        // mm, min U4 body to cavity walls

/* [Cap shell] */
cap_w = 6.2;                // mm, outer footprint X
cap_l = 6.5;                // mm, outer footprint Y
cap_h = 8.0;                // mm, outer height above PCB top (needs >= 8.5 mm free stack height)
wall_t = 0.8;               // mm, N/S/W wall thickness
vent_wall_t = 2.1;          // mm, east wall thickness (labyrinth plane + plenum + membrane seat)
roof_t = 0.8;               // mm, roof thickness
corner_r = 0.6;             // mm, outer vertical edge radius
foot_relief_h = 0.15;       // mm, glue-bead relief under the wall (0 = flat foot)
sla_wall_min = 0.6;         // mm, SLA minimum wall

/* [Foam support and sensor air gap] */
air_gap = 0.6;              // mm, clear air between U4 lid and the septum underside
air_gap_min = 0.5;          // mm, min sensor air gap (septum sag / particle clearance)
ledge_w = 0.4;              // mm, ledge-ring depth from the N/S inner walls
ledge_w_x = 0.3;            // mm, ledge-ring depth from the E/W inner walls
ledge_h = 0.4;              // mm, ledge-ring height (septum below, foam above)
foam_squeeze_max = 0.3;     // max transient squeeze of the foam passing the ledge ring at insertion

/* [Internal septum: second RC stage between the foam region and the sensor gap] */
// Same ePTFE/PET laminate as the vent membrane (one part number, two die-cuts): the
// backing makes it stiff enough that diaphragm motion does not bypass its flow resistance.
// Bonded (laminate adhesive) to the UNDERSIDE of the ledge ring from the open cap bottom,
// after the foam is inserted.  Resistance swept with mem_r_lo..mem_r_hi (covers a partial
// edge leak of the narrow adhesive land).
sep_clear = 0.05;           // mm, septum die-cut clearance to the cavity walls per side
sep_land_min = 0.25;        // mm, min adhesive land on the ledge ring
foam_fit = 0.2;             // mm, foam oversize per side (compression retention), X/Y
foam_fit_z = 0.2;           // mm, foam oversize in Z (compressed by the roof)
foam_t_min = 3.0;           // mm, min installed foam thickness
foam_pore = 0.15;           // mm, mean cell size (melamine)
foam_density = 10;          // kg/m^3 (black melamine, 9..11)
foam_sigma = 10000;         // Pa*s/m^2 flow resistivity, nominal (analysis sweeps x0.5..x2)
foam_solid_density = 1510;  // kg/m^3 melamine-formaldehyde resin (porosity = 1 - density/solid density)
foam_t_max = 150;           // C, continuous service temperature (melamine)

/* [Materials for the analysis] */
resin_hdt = 120;            // C, cap resin heat deflection temperature @ 0.45 MPa (minimum)
coat_theta_adv = 110;       // deg, advancing water contact angle, fluoropolymer-dipped resin (min)
coat_theta_rec = 90;        // deg, receding water contact angle (hysteresis -> pinning)

/* [Vent labyrinth: single-plane serpentine inside the east wall] */
// Lanes are horizontal (along Y) in the plane X = vent_xa and are joined by vertical risers
// at alternating Y ends.  Lane 0 starts at the inner port (Y = +vent_y_end, Z = vent_z_in);
// the last lane ends at Y = vent_y_out where the outer stub runs +X into the plenum.
vent_a = 0.35;              // mm, channel square side (SLA, +-vent_tol)
vent_tol = 0.03;            // mm, channel size tolerance (analysis corner)
vent_lanes = 5;             // number of lanes
vent_pitch = 0.8;           // mm, vertical lane pitch (riser length)
vent_web_min = 0.35;        // mm, min printable solid web between channels
vent_skin_in = 0.45;        // mm, skin between the cavity face and the channel plane
vent_skin = 0.5;            // mm, min solid skin around channel (to roof / corners)
vent_z_in = 6.8;            // mm, inner port centre height (top of foam region)
vent_y_out = 0.0;           // mm, outer stub Y (centre of the plenum)

/* [Vent membrane + plenum + seat on the EAST outer face] */
mem_w = 5.0;                // mm, membrane die-cut outer size along Y (incl. adhesive ring)
mem_h = 7.0;                // mm, membrane outer size along Z
mem_zc = 4.0;               // mm, membrane centre height above PCB top
mem_ring = 1.0;             // mm, adhesive ring width (seal land) all round
mem_t = 0.31;               // mm, laminate thickness incl. adhesive (Gore VE8 / LM9xx class)
mem_seat_d = 0.35;          // mm, seat recess depth (membrane sits 0.04 mm below the face)
mem_plenum_d = 0.4;         // mm, plenum depth behind the active membrane area
mem_edge_min = 0.1;         // mm, min seat to the rounded-corner tangent / foot / roof
mem_airflow = 65.5;         // l/h/cm^2 typical @ mem_dp_ref (Gore LM9xx-class datasheet value)
mem_dp_ref = 7000;          // Pa, airflow test pressure (70 mbar)
mem_t_ref = 23;             // C, airflow test temperature (viscosity reference)
mem_r_lo = 0.5;             // resistance sweep factor, low  (high-airflow lot)
mem_r_hi = 2.0;             // resistance sweep factor, high (datasheet-minimum-airflow lot)
mem_wep_min = 60000;        // Pa, water entry pressure (min, >= 600 mbar / 30 s)
mem_t_max = 150;            // C, laminate + adhesive max temperature
mem_e_eff = 50e6;           // Pa, effective bending modulus of the ePTFE/PET laminate (low bound)
mem_rho_s = 0.12;           // kg/m^2, laminate areal density
mem_nu = 0.4;               // Poisson ratio (laminate)

/* [Derived - evaluated identically by the analysis] */
cav_x0 = -cap_w / 2 + wall_t;                 // mm, cavity west face (local)
cav_x1 = cap_w / 2 - vent_wall_t;             // mm, cavity east face
cav_y0 = -cap_l / 2 + wall_t;                 // mm
cav_y1 = cap_l / 2 - wall_t;                  // mm
cav_h = cap_h - roof_t;                       // mm, cavity height (PCB to roof)
sep_z0 = u4_h + air_gap;                      // mm, septum underside
sep_z1 = sep_z0 + mem_t;                      // mm, septum top = ledge-ring underside
foam_z0 = sep_z1 + ledge_h;                   // mm, foam underside (ledge-ring top)
sep_w = cav_x1 - cav_x0 - 2 * sep_clear;      // mm, septum die-cut X
sep_l = cav_y1 - cav_y0 - 2 * sep_clear;      // mm, septum die-cut Y
sep_pw = cav_x1 - cav_x0 - 2 * ledge_w_x;     // mm, septum active area (ledge-ring opening) X
sep_pl = cav_y1 - cav_y0 - 2 * ledge_w;       // mm, Y
sep_area = sep_pw * sep_pl;                   // mm^2
foam_t = cav_h - foam_z0;                     // mm, installed foam thickness
vent_xa = cav_x1 + vent_skin_in + vent_a / 2; // mm, channel plane X
vent_y_end = cap_l / 2 - corner_r - vent_skin - vent_a / 2;   // mm, lane end |Y|
vent_z_out = vent_z_in - (vent_lanes - 1) * vent_pitch;       // mm, outer stub centre height
vent_web = vent_pitch - vent_a;               // mm, solid web between adjacent lanes
vent_y_last0 = (vent_lanes % 2 == 1) ? vent_y_end : -vent_y_end;   // mm, last lane start Y
vent_lane_len = 2 * vent_y_end;               // mm, full lane
vent_last_len = abs(vent_y_last0 - vent_y_out);   // mm, last (partial) lane to the outer stub
mem_x_face = cap_w / 2;                       // mm, outer east face
mem_x_seat = mem_x_face - mem_seat_d;         // mm, seat floor (membrane adhesive land)
mem_x_floor = mem_x_seat - mem_plenum_d;      // mm, plenum floor (outer stub opens here)
mem_pw = mem_w - 2 * mem_ring;                // mm, plenum = active membrane area, Y
mem_ph = mem_h - 2 * mem_ring;                // mm, plenum, Z
mem_area = mem_pw * mem_ph;                   // mm^2, active membrane area
mem_r = mem_dp_ref / (mem_airflow * 1e-3 / 3600 / 1e-4);     // Pa*s/m, specific airflow resistance @ mem_t_ref
vent_web_out = mem_x_floor - (vent_xa + vent_a / 2);         // mm, web channel plane -> plenum floor
vent_stub_in = vent_xa - cav_x1;              // mm, cavity face -> channel plane
vent_stub_out = mem_x_floor - vent_xa;        // mm, channel plane -> plenum floor
vent_len = (vent_lanes - 1) * (vent_lane_len + vent_pitch) + vent_last_len + vent_stub_in + vent_stub_out;   // mm, centreline
u4_x0 = u4_dx - u4_w / 2;                     // mm
u4_x1 = u4_dx + u4_w / 2;                     // mm
cap_x0_k = cap_cx_k - cap_w / 2;              // mm, KiCad footprint extents
cap_x1_k = cap_cx_k + cap_w / 2;
cap_y0_k = cap_cy_k - cap_l / 2;
cap_y1_k = cap_cy_k + cap_l / 2;

/* [Sanity asserts] */
assert(u4_x1 + pass_gap + pass_w <= cav_x1 - pass_wall_clear, "0402 column east of U4 must clear the vent wall");
assert(cav_y1 - cav_y0 >= u4_l + 1.0, "cavity too short for U4");
assert(u4_x0 >= cav_x0 + u4_wall_clear && u4_x1 <= cav_x1 - u4_wall_clear, "U4 must clear the cavity walls");
assert(abs(u4_dy) + u4_l / 2 <= cav_y1 - ledge_w - 0.2, "U4 under a ledge");
assert(air_gap >= air_gap_min, "septum-to-sensor air gap below air_gap_min");
assert(min(ledge_w, ledge_w_x) - sep_clear >= sep_land_min, "septum adhesive land on the ledge ring too narrow");
assert(1 - sep_pw / (cav_x1 - cav_x0 + 2 * foam_fit) <= foam_squeeze_max
       && 1 - sep_pl / (cav_y1 - cav_y0 + 2 * foam_fit) <= foam_squeeze_max, "foam squeezed too hard passing the ledge ring");
assert(foam_t >= foam_t_min, "foam thinner than foam_t_min");
assert(vent_web >= vent_web_min, "labyrinth web between lanes too thin to print");
assert(vent_web_out >= vent_web_min, "web between channel plane and plenum floor too thin");
assert(vent_skin_in >= vent_web_min, "labyrinth skin to the cavity too thin");
assert(vent_z_in + vent_a / 2 <= cav_h - 0.2, "inner port must open below the roof");
assert(vent_z_in - vent_a / 2 >= foam_z0 + 0.5, "inner port must open into the foam region");
assert(vent_z_in + vent_a / 2 + vent_skin <= cap_h, "labyrinth breaks through the roof");
assert(vent_z_out - vent_a / 2 >= foot_relief_h + vent_skin, "labyrinth too close to the foot");
assert(vent_y_end + vent_a / 2 <= cav_y1, "lane ends must stay within the cavity Y span (inner port opens into the cavity)");
assert(vent_lanes >= 3, "need >= 3 lanes (light baffle: >= 6 right-angle turns)");
assert(abs(vent_y_out) + vent_a / 2 <= mem_pw / 2 - 0.2, "outer stub must open inside the plenum (Y)");
assert(vent_z_out - vent_a / 2 >= mem_zc - mem_ph / 2 + 0.2 && vent_z_out + vent_a / 2 <= mem_zc + mem_ph / 2 - 0.2,
       "outer stub must open inside the plenum (Z)");
assert(mem_w / 2 <= cap_l / 2 - corner_r - mem_edge_min, "membrane seat runs onto the rounded corners");
assert(mem_zc - mem_h / 2 >= foot_relief_h + mem_edge_min + 0.2, "membrane seat too close to the foot");
assert(mem_zc + mem_h / 2 <= cap_h - mem_edge_min - 0.2, "membrane seat too close to the roof");
assert(mem_ring >= 1.0, "adhesive ring < 1 mm (vendor minimum seal land)");
assert(mem_seat_d >= mem_t + 0.02, "membrane must sit below the outer face");
assert(mem_pw > 0 && mem_ph > 0, "membrane ring leaves no active area");
assert(wall_t >= sla_wall_min && roof_t >= sla_wall_min, "SLA wall minimum");
assert(cap_x1_k <= board_size - edge_clear_min, "cap overhangs the east board edge");
assert(cap_y0_k >= edge_clear_min, "cap overhangs the north board edge");
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
         " foam_t=", foam_t, " vent_len=", vent_len, " vent_z_out=", vent_z_out,
         " mem_area=", mem_area, " mem_r=", mem_r, " sep_area=", sep_area));

// ------------------------------------------------------------------------------------------
module rounded_box(w, l, h, r) {
    hull() for (sx = [-1, 1], sy = [-1, 1])
        translate([sx * (w / 2 - r), sy * (l / 2 - r), 0]) cylinder(r = r, h = h);
}

// labyrinth centreline vertices (all in the plane X = vent_xa), inner port first
function vent_ys(k) = ((k % 2 == 0) ? 1 : -1) * vent_y_end;          // lane k start Y
function vent_zk(k) = vent_z_in - k * vent_pitch;                     // lane k Z
function vent_path() = concat(
    [for (k = [0 : vent_lanes - 2]) each [[vent_xa, vent_ys(k), vent_zk(k)], [vent_xa, -vent_ys(k), vent_zk(k)]]],
    [[vent_xa, vent_ys(vent_lanes - 1), vent_zk(vent_lanes - 1)], [vent_xa, vent_y_out, vent_zk(vent_lanes - 1)]]);

module bar(p, q, a) {
    // square-section box of side a around the axis-aligned segment p -> q (incl. a/2 end caps)
    hull() { translate(p) cube(a, center = true); translate(q) cube(a, center = true); }
}

module vent_channel(a = vent_a) {
    pts = vent_path();
    for (i = [0 : len(pts) - 2]) bar(pts[i], pts[i + 1], a);
    // inner stub: cavity face -> lane-0 start
    bar([cav_x1 - 0.05 + a / 2, pts[0][1], pts[0][2]], pts[0], a);
    // outer stub: last vertex -> plenum floor (+0.05 overlap into the plenum)
    pe = pts[len(pts) - 1];
    bar(pe, [mem_x_floor + 0.05 - a / 2, pe[1], pe[2]], a);
}

module membrane_seat() {
    // seat (adhesive land) and plenum (active area) recesses in the east face
    translate([mem_x_seat, -mem_w / 2, mem_zc - mem_h / 2]) cube([mem_seat_d + 0.05, mem_w, mem_h]);
    translate([mem_x_floor, -mem_pw / 2, mem_zc - mem_ph / 2]) cube([mem_plenum_d + 0.01, mem_pw, mem_ph]);
}

module membrane() {
    translate([mem_x_seat, -mem_w / 2, mem_zc - mem_h / 2]) cube([mem_t, mem_w, mem_h]);
}

module ledges() {
    // closed ledge ring: foam rests on top, septum bonded underneath
    difference() {
        translate([cav_x0, cav_y0, sep_z1]) cube([cav_x1 - cav_x0, cav_y1 - cav_y0, ledge_h]);
        translate([cav_x0 + ledge_w_x, cav_y0 + ledge_w, sep_z1 - 0.01]) cube([sep_pw, sep_pl, ledge_h + 0.02]);
    }
}

module septum() {
    translate([cav_x0 + sep_clear, cav_y0 + sep_clear, sep_z0]) cube([sep_w, sep_l, mem_t]);
}

module cavity() {
    translate([cav_x0, cav_y0, -0.01]) cube([cav_x1 - cav_x0, cav_y1 - cav_y0, cav_h + 0.01]);
}

module cap() {
    difference() {
        rounded_box(cap_w, cap_l, cap_h, corner_r);
        cavity();
        vent_channel();
        membrane_seat();
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
        translate([u4_x1 + pass_gap, -(n_pass * pass_l + (n_pass - 1) * pass_pitch_gap) / 2 + i * (pass_l + pass_pitch_gap), 0])
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
        septum();
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
        color("white", 0.8) membrane();
        color("white", 0.8) septum();
    }
}

if (part == "cap") cap();
else if (part == "foam_block") foam_block(true);
else if (part == "membrane") membrane();
else if (part == "septum") septum();
else if (part == "air") air();
else if (part == "vent_air") vent_channel();
else assembly();
