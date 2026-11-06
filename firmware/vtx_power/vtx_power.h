// Nano H743 AIO - digital VTX 9 V / 12 V 2 A BEC supervisor + TPS62933 plant model (Revision 13.0)
// Sreesai Arjun Engineering
//
// Host-testable reference for U6 (TI TPS62933DRLR, SOT-583-8, LCSC C3200405,
// 500 kHz with RT floating) feeding the +VTX rail of the 6-pin SH1.0 digital
// VTX connector J1 (pin 1) and the J2 solder pads.
//   VTX_PWR     PD10 GPIO(81) RELAY2 -> R39 1k -> BEC_EN -> U6 EN (internal
//               pull-up: MCU high-Z = BEC ON, hwdef "OUTPUT HIGH", RELAY2_DEFAULT 1)
//   VTX_12V_SEL PD11 GPIO(82) RELAY3 -> R37 1k -> Q1 2N7002 gate, R38 100k
//               gate pull-down (MCU high-Z = Q1 off = 9 V, RELAY3_DEFAULT 0)
//   FB          R34 102k (+VTX->FB), R35 10.0k (FB->GND), R36 26.7k FB->Q1 drain
//               Vout = 0.8 V * (1 + R34 / Rbot): Q1 off 8.96 V, Q1 on 12.02 V
//   VTX_VSENSE  PC5 ADC12_INP8 <- R40 10k / R41 1k (x11.0 = BATT2_VOLT_MULT), C43 100 nF
//
// Supervisor policy (identical to the ArduPilot Lua interlock, spec D2):
//   - 12 V only when the user requests it AND VBAT >= 13.6 V continuously for
//     2 s AND the BEC sense (battery 2) agrees, i.e. the rail is verified in
//     its window. 13.6 V = 3.4 V/cell 4S, above a full 3S pack (12.6 V), so
//     12 V is refused below 4S.
//   - drop to 9 V when VBAT < 13.0 V continuously for 0.3 s (or the request
//     goes away, or VBAT becomes invalid, or the rail faults at 12 V).
//   - VTX_PWR is NEVER driven low while armed; an off request while armed is
//     deferred until disarm.
//   - power-on order: VTX_12V_SEL low (9 V) BEFORE EN high; 12 V only after the
//     9 V rail has been ADC-verified.
//   - rail check: VTX_VSENSE * 11.0 against a window after a 10 ms soft-start
//     blank (tSS = 4.8 ms typ, 5.9 ms with ISS min), dropout-aware (see
//     rail_window()), 250 ms over-voltage blank after a 12 -> 9 V step (OVP
//     decay, see Tps62933Plant).
//   - fault persistence 300 ms; auto-retry (EN power cycle) ONLY when disarmed,
//     bounded retries, then latch. Over-voltage at 9 V (FB network / Q1 short)
//     latches the rail OFF when disarmed (HDZero Race V3 is 12 V max).
//
// What it reproduces from ArduPilot:
//   - relay semantics of AP_Relay (AP_Relay.cpp init() 317-355: RELAYn_DEFAULT
//     drives the pin at boot) and the Lua API used by the applet
//     (AP_Scripting/docs/docs.lua: relay:on/off 3076-3080, battery:voltage 3642,
//     arming:is_armed 3686, millis 87, gcs:send_text 3037).
// What it adds:
//   - a deterministic TPS62933 plant (FB divider incl. Q1 Rds_on and leakage,
//     SS ramp, Dmax / foldback dropout with Rds_on + DCR, OVP 115 % / 5 % hyst.,
//     UVP 65 % hiccup, EN / UVLO thresholds) so the policy is tested closed-loop.
//
// TPS62933 numbers: TI SLUSEA4D (Aug 2022) s8.5 Electrical Characteristics
// (p.6-7), s9.3.4 Eq.1 (p.19), s9.3.7 Eq.6 (p.21-22), s9.3.9 Eq.8/10 (p.22-23),
// s9.3.11 (p.24), s9.3.12 (p.24-25).
//
// Allocation-free, no exceptions, fixed-size state.
#pragma once

#include <stdint.h>
#include <stddef.h>

namespace nano_vtx {

// ---------------------------------------------------------------------------
// Board + TPS62933 constants
// ---------------------------------------------------------------------------
namespace hw {
// feedback network (schematic refdes, 1 % 0402)
static constexpr float R34_TOP_OHM   = 102000.0f;
static constexpr float R35_BOT_OHM   = 10000.0f;
static constexpr float R36_SW_OHM    = 26700.0f;
static constexpr float Q1_RDS_ON_OHM = 10.0f;      // 2N7002 C8545 at Vgs 3.27 V (5 R at 10 V, datasheet); x2 margin
// sense divider (BATT2_VOLT_MULT 11.0, ardupilot/defaults.parm)
static constexpr float R40_TOP_OHM   = 10000.0f;
static constexpr float R41_BOT_OHM   = 1000.0f;
static constexpr float SENSE_MULT    = (R40_TOP_OHM + R41_BOT_OHM) / R41_BOT_OHM;   // 11.0
// gate drive: 3.3 V GPIO through R37 1k into R38 100k
static constexpr float GPIO_HIGH_V   = 3.3f;
static constexpr float R37_GATE_OHM  = 1000.0f;
static constexpr float R38_PD_OHM    = 100000.0f;
// TPS62933 SLUSEA4D s8.5
static constexpr float VREF_TYP      = 0.800f;     // VFB typ
static constexpr float VREF_MIN      = 0.784f;     // VFB TJ -40..150 C
static constexpr float VREF_MAX      = 0.816f;
static constexpr float IFB_LEAK_MAX  = 0.15e-6f;   // IFB at VFB 0.8 V
static constexpr float ISS_TYP       = 5.5e-6f;    // ISS 4.5 / 5.5 / 6.5 uA
static constexpr float ISS_MIN       = 4.5e-6f;
static constexpr float ISS_MAX       = 6.5e-6f;
static constexpr float C38_SS_F      = 33e-9f;     // C38 33 nF (DS s10.2.2.4: ~5 ms)
static constexpr float FSW_HZ        = 500e3f;     // RT floating, 450..550 kHz
static constexpr float TOFF_MIN_S    = 140e-9f;    // tOFF_MIN typ
static constexpr float TON_MAX_S     = 7e-6f;      // tON_MAX typ (foldback limit)
static constexpr float RDS_HS_OHM    = 0.076f;     // RDSON_HS 25 C, VBST-SW 5 V
static constexpr float RDS_LS_OHM    = 0.032f;     // RDSON_LS 25 C
static constexpr float L1_DCR_OHM    = 0.057f;     // DMBJ PNLS6045-100M C2849537
static constexpr float IHS_LIMIT_MIN = 4.2f;       // IHS_LIMIT 4.2 / 5 / 5.8 A
static constexpr float ILS_LIMIT_MIN = 2.9f;       // ILS_LIMIT 2.9 / 3.8 / 4.5 A
static constexpr float OVP_RATIO     = 1.15f;      // VOVP 112/115/118 %
static constexpr float OVP_HYST      = 0.05f;      // VOVP hysteresis 5 %
static constexpr float UVP_RATIO     = 0.65f;      // VUVP 65 %
static constexpr float HICCUP_ON_S   = 256e-6f;    // thiccup_ON
static constexpr float HICCUP_OFF_TSS = 10.5f;     // thiccup_OFF = 10.5 x tSS
static constexpr float EN_RISE_MAX_V = 1.28f;      // VEN_RISE 1.21 typ / 1.28 max
static constexpr float EN_FALL_MIN_V = 1.10f;      // VEN_FALL 1.10 min / 1.17 typ
static constexpr float UVLO_RISE_V   = 3.6f;       // VIN_UVLO rising typ
static constexpr float UVLO_FALL_V   = 3.3f;       // VIN_UVLO falling typ
}

// ---------------------------------------------------------------------------
// Pure helpers (all SI units)
// ---------------------------------------------------------------------------
// Effective bottom resistor: R35 alone (Q1 off) or R35 || (R36 + Rds_on) (Q1 on).
float rbot_effective(bool sel12, float rds_on = hw::Q1_RDS_ON_OHM);
// DS Eq.1 rearranged: Vout = Vref * (1 + Rtop / Rbot) + Ileak * Rtop
// (Ileak = FB pin leakage + Q1 off-state leakage flowing out of the FB node).
float vout_setpoint(bool sel12, float vref = hw::VREF_TYP, float ileak_a = 0.0f,
                    float rds_on = hw::Q1_RDS_ON_OHM);
// FB voltage for a given Vout (Q1 state) - used for the OVP/UVP ratios.
float fb_voltage(float vout, bool sel12, float rds_on = hw::Q1_RDS_ON_OHM);
// DS Eq.6: tSS = Css * Vref / Iss
float soft_start_s(float css_f = hw::C38_SS_F, float iss_a = hw::ISS_TYP, float vref = hw::VREF_TYP);
// DS Eq.8: Dmax = 1 - tOFF_MIN * fsw (fixed frequency, no foldback)
float dmax_fixed(float fsw_hz = hw::FSW_HZ, float toff_s = hw::TOFF_MIN_S);
// Foldback limit: tON_MAX / (tON_MAX + tOFF_MIN) (DS s9.3.9, ~0.98)
float dmax_foldback();
// CCM dropout with conduction losses (research rev13_bec_vtx.md s2):
//   Vout_max = D*Vin - I*(DCR + D*Rhs + (1-D)*Rls)
float dropout_vout_max(float vin, float iout, float d, float rhs = hw::RDS_HS_OHM,
                       float rls = hw::RDS_LS_OHM, float dcr = hw::L1_DCR_OHM);
// Inverse: minimum Vin to hold vout at iout.
float dropout_vin_min(float vout, float iout, float d, float rhs = hw::RDS_HS_OHM,
                      float rls = hw::RDS_LS_OHM, float dcr = hw::L1_DCR_OHM);
// Q1 gate voltage when VTX_12V_SEL drives high (R37/R38 divider): 3.267 V.
float q1_gate_drive_v();

// ---------------------------------------------------------------------------
// Abstract hardware hooks
// ---------------------------------------------------------------------------
// On the FC these are relay:on/off(1) (RELAY2 = VTX_PWR) and relay:on/off(2)
// (RELAY3 = VTX_12V_SEL); AP_Relay writes hal.gpio (AP_Relay.cpp 520-541).
class VtxPins {
public:
    virtual ~VtxPins() = default;
    virtual void set_enable(bool on) = 0;     // VTX_PWR: true = EN high (BEC on)
    virtual void set_select_12v(bool on) = 0; // VTX_12V_SEL: true = Q1 on = 12 V
};

// VTX_VSENSE pin voltage (before the x11 divider ratio). On the FC this is
// battery:voltage(1) / BATT2_VOLT_MULT. Returns false if no fresh sample.
class SenseAdc {
public:
    virtual ~SenseAdc() = default;
    virtual bool read_pin_v(float& v) = 0;
};

enum class Severity : uint8_t { EMERGENCY = 0, ALERT, CRITICAL, ERROR, WARNING, NOTICE, INFO, DEBUG };
class Reporter {
public:
    virtual ~Reporter() = default;
    virtual void text(Severity sev, const char* msg) = 0;   // gcs:send_text
};

// ---------------------------------------------------------------------------
// Supervisor
// ---------------------------------------------------------------------------
struct Config {
    // 12 V interlock (spec D2)
    float v12_enable_vbat = 13.6f;      // V, >= for v12_qualify_ms
    uint32_t v12_qualify_ms = 2000;
    float v12_drop_vbat = 13.0f;        // V, < for v12_drop_ms
    uint32_t v12_drop_ms = 300;
    float vbat_valid_min = 2.0f;        // below -> monitor not connected / invalid
    float vbat_valid_max = 60.0f;
    // rail targets (from the FB divider, typ Vref)
    float target_9v = 0.0f;             // 0 = derive with vout_setpoint(false)
    float target_12v = 0.0f;            // 0 = derive with vout_setpoint(true)
    float sense_mult = hw::SENSE_MULT;  // BATT2_VOLT_MULT
    // window
    float window_tol = 0.06f;           // +/- 6 %
    float dropout_headroom_v = 0.4f;    // spec D3: expected = VBAT - headroom in dropout
    float dropout_dmax = 0.93f;         // DS Eq.8 at 500 kHz (no foldback): worst-case floor
    float dropout_i_max = 2.0f;         // A, rail rating
    float dropout_r_hot = 0.175f;       // DCR 65 mR + HS 110 mR hot (rev13_bec_vtx.md s2, ~1.4x 25 C)
    uint32_t settle_ms = 10;            // after EN on / 9 -> 12 V (tSS 5.9 ms worst + margin)
    uint32_t down_settle_ms = 250;      // OV blank after 12 -> 9 V (OVP decay, ~78 ms no-load @30 uF)
    uint32_t fault_persist_ms = 300;    // out-of-window time before a fault is declared
    // retry (disarmed only)
    uint32_t retry_off_ms = 500;        // EN low time per power cycle
    uint8_t max_retries = 3;
};

enum class RailState : uint8_t { OFF = 0, STARTING, RUNNING, RETRY_OFF, LATCHED_ON, LATCHED_OFF };
const char* to_string(RailState s);

enum class FaultKind : uint8_t { NONE = 0, UNDERVOLTAGE, OVERVOLTAGE, NO_SENSE };
const char* to_string(FaultKind k);

struct Inputs {
    uint32_t now_ms = 0;
    bool armed = false;
    float vbat_v = 0.0f;         // battery:voltage(0)
    bool vbat_valid = false;     // battery:healthy(0)
    bool want_power = true;      // RELAY2 user intent (RELAY2_DEFAULT 1)
    bool want_12v = false;       // VTXB_12V_REQ / RELAY3 user intent
};

struct Window {
    float lo = 0.0f, hi = 0.0f;
    float expected = 0.0f;       // spec D3 "expected" (min(target, VBAT - headroom))
    bool dropout = false;
};

enum Flag : uint32_t {
    F_EN          = 1u << 0,
    F_SEL12       = 1u << 1,
    F_RAIL_OK     = 1u << 2,     // last checked sample in window, past blanking
    F_V12_QUAL    = 1u << 3,     // VBAT qualified for 12 V (>= 13.6 V for 2 s)
    F_V12_LOCKOUT = 1u << 4,     // rail faulted at 12 V; cleared when the request drops
    F_OFF_DEFERRED = 1u << 5,    // off requested while armed
    F_FAULT_ACTIVE = 1u << 6,
    F_LATCHED     = 1u << 7,
};

struct Health {
    uint32_t flags = 0;
    uint32_t updates = 0, sense_reads = 0, sense_missing = 0;
    uint32_t v12_grants = 0, v12_refused = 0, v12_drops_lowbat = 0, v12_drops_request = 0, v12_drops_fault = 0;
    uint32_t faults_uv = 0, faults_ov = 0, faults_nosense = 0, faults_while_armed = 0;
    uint32_t retries = 0, latches = 0, off_deferred = 0, en_low_while_armed = 0;
    float last_rail_v = 0.0f;
    Window last_window;
};

class VtxBecSupervisor {
public:
    VtxBecSupervisor(VtxPins& pins, SenseAdc& adc, Reporter* rep = nullptr, const Config& cfg = Config());

    // Boot: matches hwdef (VTX_PWR HIGH, VTX_12V_SEL LOW) and RELAY2_DEFAULT 1 /
    // RELAY3_DEFAULT 0: drives SEL low, then EN high, starts the settle timer.
    void begin(uint32_t now_ms);
    // Call periodically (Lua applet: 10 Hz). Rate independent: all timing is
    // by timestamp difference (uint32 wrap-safe).
    void update(const Inputs& in);
    // Operator clear (e.g. param write); refused while armed. Restarts at 9 V.
    bool clear_latch(bool armed, uint32_t now_ms);

    // Window for the current selection / VBAT (public for tests + analysis).
    Window rail_window(bool sel12, float vbat_v, bool vbat_valid) const;
    float target(bool sel12) const { return sel12 ? t12_ : t9_; }

    RailState state() const { return state_; }
    FaultKind fault() const { return fault_; }
    bool enabled() const { return en_; }
    bool select_12v() const { return sel12_; }
    const Health& health() const { return h_; }
    const Config& config() const { return cfg_; }

private:
    void drive_enable(bool on, bool armed);
    void drive_select(bool on);
    void start(uint32_t now_ms, uint32_t settle);
    void say(Severity s, const char* fmt, ...) __attribute__((format(printf, 3, 4)));
    void declare_fault(FaultKind k, const Inputs& in);
    static bool elapsed(uint32_t now, uint32_t since, uint32_t ms) { return uint32_t(now - since) >= ms; }
    static bool before(uint32_t now, uint32_t until) { return int32_t(now - until) < 0; }

    VtxPins& pins_;
    SenseAdc& adc_;
    Reporter* rep_;
    Config cfg_;
    Health h_;
    float t9_ = 0.0f, t12_ = 0.0f;
    RailState state_ = RailState::OFF;
    FaultKind fault_ = FaultKind::NONE;
    bool en_ = false, sel12_ = false;
    bool lockout12_ = false;
    bool refused_reported_ = false, fault_reported_ = false;
    bool hi_valid_ = false, lo_valid_ = false, bad_valid_ = false;
    uint32_t hi_since_ = 0, lo_since_ = 0, bad_since_ = 0;
    uint32_t settle_until_ = 0, ov_blank_until_ = 0, state_since_ = 0;
    uint8_t retries_ = 0;
    char msg_[80] = {0};
};

// ---------------------------------------------------------------------------
// TPS62933 plant (closed-loop test model; averaged, CCM / pulse-skip)
// ---------------------------------------------------------------------------
enum class PinDrive : uint8_t { LOW = 0, HIGH, HIGH_Z };

struct PlantParams {
    float vref = hw::VREF_TYP;
    float iss = hw::ISS_TYP;
    float css = hw::C38_SS_F;
    float cout_eff_f = 30e-6f;    // 3 x 22 uF 25 V 1206 at 9-12 V bias, ~45 % (UNVERIFIED DC-bias)
    float dmax = 0.93f;           // 0.93 fixed (Eq.8) or dmax_foldback() ~0.98
    float rhs = hw::RDS_HS_OHM, rls = hw::RDS_LS_OHM, dcr = hw::L1_DCR_OHM;
    float ilim = 0.5f * (hw::IHS_LIMIT_MIN + hw::ILS_LIMIT_MIN);   // DS Eq.11 with min limits: 3.55 A
    float q1_vth = 2.5f;          // 2N7002 C8545 Vgs(th) max
    float q1_rds_on = hw::Q1_RDS_ON_OHM;
    float ileak_fb = 0.0f;        // FB pin leakage, A (<= 0.15 uA)
    float ileak_q1 = 0.0f;        // Q1 off-state D-S leakage, A
    bool q1_short = false;        // fault: Q1 D-S short (stuck at 12 V)
    bool r34_open = false;        // fault: top resistor open (FB -> GND, Vout -> dropout)
    float sense_gain = 1.0f;      // R40/R41 tolerance (1.0 = exact x11)
    float adc_vref = 3.3f;        // H7 ADC full scale for VTX_VSENSE (SCALE(1))
};

class Tps62933Plant {
public:
    explicit Tps62933Plant(const PlantParams& p = PlantParams()) : p_(p) {}
    // Advance by dt (s). vin = VBAT, iload = VTX current (constant-current model),
    // rshort > 0 adds a resistive fault load.
    void step(float dt, float vin, float iload, float rshort = 0.0f);
    void run(float seconds, float vin, float iload, float rshort = 0.0f, float dt = 20e-6f);

    void set_en(PinDrive d) { en_drive_ = d; }
    void set_sel(PinDrive d) { sel_drive_ = d; }

    float vout() const { return vout_; }
    float fb() const;
    bool q1_on() const;
    bool switching() const { return running_ && !hiccup_ && !ovp_; }
    bool running() const { return running_; }
    bool ovp() const { return ovp_; }
    bool hiccup() const { return hiccup_; }
    uint32_t ovp_events() const { return ovp_events_; }
    uint32_t hiccup_events() const { return hiccup_events_; }
    float setpoint() const;
    float sense_pin_v() const;     // VTX_VSENSE incl. 12-bit quantisation
    float en_pin_v() const;
    float ss_v() const { return ss_; }
    PlantParams& params() { return p_; }

private:
    float rbot() const;
    float rtop() const;
    PlantParams p_;
    PinDrive en_drive_ = PinDrive::HIGH_Z, sel_drive_ = PinDrive::HIGH_Z;
    float vout_ = 0.0f, ss_ = 0.0f;
    bool running_ = false, uvlo_ok_ = false, ovp_ = false, hiccup_ = false;
    float uv_time_ = 0.0f, hiccup_time_ = 0.0f;
    uint32_t ovp_events_ = 0, hiccup_events_ = 0;
};

} // namespace nano_vtx
