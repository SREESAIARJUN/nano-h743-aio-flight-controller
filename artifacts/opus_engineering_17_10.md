# Pan-Axis Yoke Assembly: Parametric Geometry (Cycle 17)

I don't have the outputs of Cycles 1–16, so this design states its baseline assumptions explicitly. Every dimension is a named parameter. If your earlier cycles fixed different values (motor model, bearing size, tilt span), change the parameter block and the geometry will regenerate consistently.

---

## 1. Baseline Assumptions (override as needed)

| Item | Assumed part | Key dims | Verify by |
|---|---|---|---|
| Pan motor | GBM2804-class hollow-shaft gimbal BLDC | Ø28 stator base, M2.5 ×4 on Ø19 PCD (stator), M2 ×4 on Ø12 PCD (rotor), hollow bore Ø6.9 | Calipers on your batch, since PCDs vary by vendor |
| Slip ring | 6.5 mm capsule micro slip ring, 6-wire | Body Ø6.5 × 18.5 L, flange Ø15 × 1.0 with 2× M1.6 on Ø11.5 | Datasheet + calipers |
| Pan bearing | 6800-2RS (thin section) | 10 × 19 × 5 | Standard |
| Tilt motor | Second GBM2804 on yoke arm | Same as pan | – |
| Material | PETG/PA-CF FDM *or* 6061-T6 CNC | `MFG` switch selects fit set | – |

**Load path.** The base holds the pan-motor stator. The rotor drives a hollow spindle that carries the yoke. The 6800 bearing, preloaded against a shoulder, takes the radial and overturning moment, so the motor's internal bearings don't carry the full yoke load. The slip-ring body (stator side) is keyed to the base. Its rotor wires exit upward through the spindle bore into the yoke.

---

## 2. Fit and Tolerance Table

| Feature | Nominal | FDM (as-printed, then reamed) | CNC 6061 | Fit class |
|---|---|---|---|---|
| 6800 housing bore | Ø19.000 | Ø19.15 print → ream Ø19.00 | Ø19.000 −0.009/−0.030 | N7 (light press) |
| 6800 shaft seat | Ø10.000 | Ø9.85 print → sand to slide | Ø10.000 0/−0.009 | h6/k5 sliding |
| Slip-ring channel | Ø6.5 | Ø6.85 | Ø6.60 +0.05/0 | Clearance (body must not be loaded) |
| Spindle wire bore | ≥ Ø4.0 | Ø4.5 | Ø4.5 | Clearance |
| Motor pilot | Ø(hub) | +0.20 | +0.02/0 | Locating |
| Bearing shoulder height | ≥ 1/3 race | 1.2 | 1.0 | Per ISO 15 abutment |

**Concentricity budget** (slip ring vs. pan axis): ≤ 0.10 mm TIR CNC, ≤ 0.25 mm FDM. Beyond that, the capsule's internal brushes see side load and wear fast. If you exceed it, use the compliant coupling option (`SR_FLEX=true`), which replaces the rigid rotor clamp with a slotted-tab drive.

---

## 3. OpenSCAD Source

```openscad
// ============================================================
//  PAN-AXIS YOKE + BEARING SEATS + 6.5mm SLIP-RING CHANNEL
//  Cycle 17  -  fully parametric
// ============================================================
$fn = 128;

// ---------- manufacturing mode ----------
MFG = "FDM";            // "FDM" or "CNC"
fdm = (MFG == "FDM");

// ---------- fits (resolved from mode) ----------
BRG_HOUSING_ALLOW = fdm ? 0.15 : -0.015;  // + = oversize bore
BRG_SHAFT_ALLOW   = fdm ? -0.15 : -0.005;
SR_CLEAR          = fdm ? 0.35 : 0.10;
PILOT_CLEAR       = fdm ? 0.20 : 0.02;
HOLE_CLEAR        = fdm ? 0.30 : 0.15;

// ---------- pan motor (GBM2804-class) ----------
M_D          = 28.0;    // stator base OD
M_H          = 18.0;    // overall height
M_BORE       = 6.9;     // hollow shaft ID
M_STATOR_PCD = 19.0;  M_STATOR_SCREW = 2.5;
M_ROTOR_PCD  = 12.0;  M_ROTOR_SCREW  = 2.0;
M_ROTOR_PILOT= 8.0;     // rotor-face centering boss dia

// ---------- bearing 6800 ----------
B_ID = 10.0; B_OD = 19.0; B_W = 5.0;
B_SHOULDER   = fdm ? 1.2 : 1.0;   // radial abutment
B_PRELOAD_GAP= 0.10;              // axial gap closed by clamp nut/washer

// ---------- slip ring ----------
SR_D      = 6.5;   SR_L      = 18.5;
SR_FL_D   = 15.0;  SR_FL_T   = 1.0;
SR_FL_PCD = 11.5;  SR_FL_SCREW = 1.6;
SR_WIRE_BORE = 4.5;
SR_FLEX   = false;  // compliant rotor drive

// ---------- spindle ----------
SP_OD   = B_ID + BRG_SHAFT_ALLOW;
SP_ID   = SR_D + SR_CLEAR;
SP_LEN  = B_W + 6.0;
SP_FLANGE_D = 24.0; SP_FLANGE_T = 3.0;

// ---------- yoke ----------
TILT_SPAN    = 70.0;   // inner clearance between arms
ARM_T        = 6.0;
ARM_W        = 32.0;
ARM_H        = 55.0;   // base-plate top to tilt axis
YOKE_BASE_T  = 5.0;
FILLET_R     = 4.0;
TILT_MOTOR_D = 28.0; TILT_PCD = 19.0; TILT_SCREW = 2.5;
WIRE_SLOT_W  = 5.0;    // cable channel up the arm

// ---------- base housing ----------
BASE_OD = 40.0; BASE_H = M_H + B_W + 8.0;

// ============================================================
//  HELPERS
// ============================================================
module bolt_circle(pcd, n, d, h, phase=45) {
    for (i=[0:n-1]) rotate([0,0,phase+i*360/n])
        translate([pcd/2,0,-0.01]) cylinder(d=d, h=h+0.02);
}
module rounded_box(size, r) {
    hull() for (x=[r,size[0]-r], y=[r,size[1]-r])
        translate([x,y,0]) cylinder(r=r, h=size[2]);
}

// ============================================================
//  BEARING SEAT (housing side), reusable
// ============================================================
module bearing_seat_cut(depth=B_W) {
    // bore
    cylinder(d=B_OD + BRG_HOUSING_ALLOW, h=depth + B_PRELOAD_GAP);
    // abutment shoulder: clearance for inner race / seals
    translate([0,0,-B_SHOULDER])
        cylinder(d=B_OD - 2*B_SHOULDER - 1.5, h=B_SHOULDER+0.01);
    // install chamfer
    translate([0,0,depth+B_PRELOAD_GAP-0.6])
        cylinder(d1=B_OD+BRG_HOUSING_ALLOW, d2=B_OD+1.4, h=0.61);
}

// ============================================================
//  SLIP-RING CHANNEL (stator-side pocket + anti-rotation)
// ============================================================
module slipring_channel_cut() {
    // body channel
    cylinder(d=SR_D + SR_CLEAR, h=SR_L + 1);
    // flange counterbore (stator side, at bottom)
    translate([0,0,-SR_FL_T-0.05])
        cylinder(d=SR_FL_D + SR_CLEAR, h=SR_FL_T+0.1);
    // flange screws (also anti-rotation)
    translate([0,0,-SR_FL_T-6])
        bolt_circle(SR_FL_PCD, 2, SR_FL_SCREW - 0.25, 6, phase=0); // tap drill
    // stator lead exit, radial slot
    translate([0,-1.5,-SR_FL_T-3]) cube([BASE_OD, 3, 3]);
}

// ============================================================
//  BASE HOUSING (fixed)
// ============================================================
module base_housing() {
    difference() {
        cylinder(d=BASE_OD, h=BASE_H);
        // motor stator pocket from top
        translate([0,0,BASE_H - M_H - 0.01])
            cylinder(d=M_D + PILOT_CLEAR*2, h=M_H+0.02);
        // stator screws
        translate([0,0,BASE_H - M_H - 4])
            bolt_circle(M_STATOR_PCD, 4, M_STATOR_SCREW+HOLE_CLEAR, 4.1);
        // slip ring, coaxial, seated below motor
        translate([0,0,BASE_H - M_H - SR_L])
            slipring_channel_cut();
        // mounting holes to chassis
        bolt_circle(32, 4, 3.4, 4, phase=0);
    }
}

// ============================================================
//  SPINDLE (rotates with motor rotor, carries yoke)
// ============================================================
module spindle() {
    difference() {
        union() {
            // flange bolts to motor rotor
            cylinder(d=SP_FLANGE_D, h=SP_FLANGE_T);
            // rotor pilot boss
            translate([0,0,-1.5])
                cylinder(d=M_ROTOR_PILOT - PILOT_CLEAR, h=1.5);
            // bearing journal
            translate([0,0,SP_FLANGE_T]) cylinder(d=SP_OD, h=SP_LEN);
        }
        // through bore for slip-ring rotor wires
        translate([0,0,-2]) cylinder(d=SP_ID, h=SP_LEN+SP_FLANGE_T+4);
        // rotor screws
        bolt_circle(M_ROTOR_PCD, 4, M_ROTOR_SCREW+HOLE_CLEAR, SP_FLANGE_T);
        // M10x0.75 thread relief / clamp-nut zone at top (cosmetic groove)
        translate([0,0,SP_FLANGE_T+SP_LEN-3])
            difference(){ cylinder(d=SP_OD+1,h=0.6); cylinder(d=SP_OD-0.6,h=0.6); }
        // slip-ring rotor drive
        if (SR_FLEX) {
            translate([-0.6,-SP_OD,SP_FLANGE_T+SP_LEN-4])
                cube([1.2, SP_OD*2, 4.1]);      // slot accepts wire-bundle tab
        }
    }
}

// ============================================================
//  YOKE (U-frame, bearing housing integrated in base plate)
// ============================================================
module yoke() {
    W  = TILT_SPAN + 2*ARM_T;
    difference() {
        union() {
            // base plate
            translate([-W/2, -ARM_W/2, 0])
                rounded_box([W, ARM_W, YOKE_BASE_T], FILLET_R);
            // bearing boss
            cylinder(d=B_OD + 8, h=YOKE_BASE_T + B_W - 1);
            // arms
            for (s=[-1,1]) translate([s*(TILT_SPAN/2 + ARM_T/2) - ARM_T/2, -ARM_W/2, 0])
                hull() {
                    cube([ARM_T, ARM_W, YOKE_BASE_T]);
                    translate([0, ARM_W/2, ARM_H]) rotate([0,90,0])
                        cylinder(d=ARM_W, h=ARM_T);
                }
            // inner root fillets
            for (s=[-1,1]) translate([s*(TILT_SPAN/2) - (s>0?FILLET_R:0), -ARM_W/2, YOKE_BASE_T])
                difference() {
                    cube([FILLET_R, ARM_W, FILLET_R]);
                    translate([s>0?0:FILLET_R, -0.1, FILLET_R]) rotate([-90,0,0])
                        cylinder(r=FILLET_R, h=ARM_W+0.2);
                }
        }
        // bearing seat (bearing outer race in yoke; spindle in inner race)
        translate([0,0,B_SHOULDER]) bearing_seat_cut();
        // through bore for wires
        translate([0,0,-1]) cylinder(d=SP_ID + 1.0, h=50);
        // tilt motor mount on +X arm (outside face)
        translate([TILT_SPAN/2 - 0.01, 0, ARM_H]) rotate([0,90,0]) {
            cylinder(d=M_ROTOR_PILOT + PILOT_CLEAR, h=ARM_T+0.02);
            bolt_circle(TILT_PCD, 4, TILT_SCREW + HOLE_CLEAR, ARM_T);
        }
        // idler bearing seat on -X arm (6800 again, inside face)
        translate([-TILT_SPAN/2 + 0.01, 0, ARM_H]) rotate([0,-90,0])
            bearing_seat_cut(depth=B_W);
        // wire channel up +X arm (inside face)
        translate([TILT_SPAN/2 - 2.0, -WIRE_SLOT_W/2, YOKE_BASE_T])
            cube([2.01, WIRE_SLOT_W, ARM_H - YOKE_BASE_T]);
        // wire passage from hub to arm, under the plate
        translate([0, -WIRE_SLOT_W/2, -0.01])
            cube([TILT_SPAN/2, WIRE_SLOT_W, 2.0]);
        // lightening pockets
        for (s=[-1,1]) translate([s*TILT_SPAN/4, 0, -0.01])
            cylinder(d=10, h=YOKE_BASE_T - 1.5);
    }
}

// ============================================================
//  ASSEMBLY / EXPORT SELECTOR
// ============================================================
PART = "assembly";   // "base" | "spindle" | "yoke" | "assembly"

if (PART=="