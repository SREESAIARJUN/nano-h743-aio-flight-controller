// Nano H743 AIO - digital VTX 9 V / 12 V 2 A BEC supervisor + TPS62933 plant model (Revision 13.0)
// Sreesai Arjun Engineering
#include "vtx_power.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace nano_vtx {

// ---------------------------------------------------------------------------
// Pure helpers
// ---------------------------------------------------------------------------
static float parallel(float a, float b) { return a * b / (a + b); }

float rbot_effective(bool sel12, float rds_on)
{
    return sel12 ? parallel(hw::R35_BOT_OHM, hw::R36_SW_OHM + rds_on) : hw::R35_BOT_OHM;
}

float vout_setpoint(bool sel12, float vref, float ileak_a, float rds_on)
{
    // Node equation at FB: (Vout - Vfb)/Rtop = Vfb/Rbot + Ileak, Vfb = Vref.
    return vref * (1.0f + hw::R34_TOP_OHM / rbot_effective(sel12, rds_on)) + ileak_a * hw::R34_TOP_OHM;
}

float fb_voltage(float vout, bool sel12, float rds_on)
{
    const float rb = rbot_effective(sel12, rds_on);
    return vout * rb / (hw::R34_TOP_OHM + rb);
}

float soft_start_s(float css_f, float iss_a, float vref) { return css_f * vref / iss_a; }

float dmax_fixed(float fsw_hz, float toff_s) { return 1.0f - toff_s * fsw_hz; }

float dmax_foldback() { return hw::TON_MAX_S / (hw::TON_MAX_S + hw::TOFF_MIN_S); }

float dropout_vout_max(float vin, float iout, float d, float rhs, float rls, float dcr)
{
    const float v = d * vin - iout * (dcr + d * rhs + (1.0f - d) * rls);
    return v > 0.0f ? v : 0.0f;
}

float dropout_vin_min(float vout, float iout, float d, float rhs, float rls, float dcr)
{
    return (vout + iout * (dcr + d * rhs + (1.0f - d) * rls)) / d;
}

float q1_gate_drive_v()
{
    return hw::GPIO_HIGH_V * hw::R38_PD_OHM / (hw::R37_GATE_OHM + hw::R38_PD_OHM);
}

const char* to_string(RailState s)
{
    switch (s) {
    case RailState::OFF: return "OFF";
    case RailState::STARTING: return "STARTING";
    case RailState::RUNNING: return "RUNNING";
    case RailState::RETRY_OFF: return "RETRY_OFF";
    case RailState::LATCHED_ON: return "LATCHED_ON";
    case RailState::LATCHED_OFF: return "LATCHED_OFF";
    }
    return "?";
}

const char* to_string(FaultKind k)
{
    switch (k) {
    case FaultKind::NONE: return "none";
    case FaultKind::UNDERVOLTAGE: return "undervoltage";
    case FaultKind::OVERVOLTAGE: return "overvoltage";
    case FaultKind::NO_SENSE: return "no sense";
    }
    return "?";
}

// ---------------------------------------------------------------------------
// Supervisor
// ---------------------------------------------------------------------------
VtxBecSupervisor::VtxBecSupervisor(VtxPins& pins, SenseAdc& adc, Reporter* rep, const Config& cfg)
    : pins_(pins), adc_(adc), rep_(rep), cfg_(cfg)
{
    t9_ = cfg_.target_9v > 0.0f ? cfg_.target_9v : vout_setpoint(false);
    t12_ = cfg_.target_12v > 0.0f ? cfg_.target_12v : vout_setpoint(true);
}

void VtxBecSupervisor::say(Severity s, const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg_, sizeof(msg_), fmt, ap);
    va_end(ap);
    if (rep_ != nullptr) {
        rep_->text(s, msg_);
    }
}

void VtxBecSupervisor::drive_enable(bool on, bool armed)
{
    if (!on && armed) {
        // Hard guard: the policy never asks for this; count it if it ever does.
        h_.en_low_while_armed++;
        return;
    }
    en_ = on;
    pins_.set_enable(on);
}

void VtxBecSupervisor::drive_select(bool on)
{
    sel12_ = on;
    pins_.set_select_12v(on);
}

void VtxBecSupervisor::start(uint32_t now_ms, uint32_t settle)
{
    // Sequencing: select 9 V first, then enable (Q1 gate settles in << 1 ms:
    // 1k * Ciss ~ 50 pF = 50 ns), so the rail can never start at 12 V.
    drive_select(false);
    drive_enable(true, false);
    state_ = RailState::STARTING;
    state_since_ = now_ms;
    settle_until_ = now_ms + settle;
    bad_valid_ = false;
}

void VtxBecSupervisor::begin(uint32_t now_ms)
{
    start(now_ms, cfg_.settle_ms);
    say(Severity::INFO, "VTX BEC: on at %.2f V", double(t9_));
}

bool VtxBecSupervisor::clear_latch(bool armed, uint32_t now_ms)
{
    if (armed) {
        return false;
    }
    retries_ = 0;
    fault_ = FaultKind::NONE;
    fault_reported_ = false;
    lockout12_ = false;
    if (state_ == RailState::LATCHED_ON || state_ == RailState::LATCHED_OFF) {
        start(now_ms, cfg_.settle_ms);
        say(Severity::NOTICE, "VTX BEC: latch cleared, restart at 9 V");
    }
    return true;
}

Window VtxBecSupervisor::rail_window(bool sel12, float vbat_v, bool vbat_valid) const
{
    Window w;
    const float t = target(sel12);
    if (!vbat_valid) {
        // Without VBAT dropout cannot be told from a fault: check over-voltage only.
        w.expected = t;
        w.lo = 0.0f;
        w.hi = t * (1.0f + cfg_.window_tol);
        return w;
    }
    // Spec D3 "expected" point: VBAT - headroom when in dropout.
    w.expected = fminf(t, vbat_v - cfg_.dropout_headroom_v);
    w.dropout = (vbat_v - cfg_.dropout_headroom_v) < t;
    // Lower bound: worst-case fixed-frequency dropout at full rated load, hot
    // Rds_on/DCR (a lower bound for any real part, which also has foldback).
    const float floor_v = cfg_.dropout_dmax * vbat_v - cfg_.dropout_i_max * cfg_.dropout_r_hot;
    w.lo = fmaxf(0.0f, fminf(t, floor_v)) * (1.0f - cfg_.window_tol);
    // Upper bound: a buck output can never exceed its input.
    w.hi = fminf(t, vbat_v) * (1.0f + cfg_.window_tol);
    return w;
}

void VtxBecSupervisor::declare_fault(FaultKind k, const Inputs& in)
{
    const uint32_t now = in.now_ms;
    fault_ = k;
    bad_valid_ = false;               // persistence restarts for the next decision
    h_.flags |= F_FAULT_ACTIVE;
    if (k == FaultKind::UNDERVOLTAGE) h_.faults_uv++;
    if (k == FaultKind::OVERVOLTAGE) h_.faults_ov++;
    if (k == FaultKind::NO_SENSE) h_.faults_nosense++;

    if (sel12_) {
        // First response at 12 V: fall back to 9 V (not a power-off, allowed
        // armed), lock 12 V out until the request is withdrawn.
        drive_select(false);
        lockout12_ = true;
        h_.v12_drops_fault++;
        ov_blank_until_ = now + cfg_.down_settle_ms;
        settle_until_ = now + cfg_.settle_ms;
        say(Severity::WARNING, "VTX BEC: %s at 12V (%.2f V), back to 9V", to_string(k), double(h_.last_rail_v));
        return;
    }
    if (k == FaultKind::NO_SENSE) {
        if (!fault_reported_) {
            say(Severity::WARNING, "VTX BEC: no VTX_VSENSE sample, rail unverified");
            fault_reported_ = true;
        }
        return;
    }
    if (in.armed) {
        h_.faults_while_armed++;
        if (!fault_reported_) {
            say(Severity::CRITICAL, "VTX BEC: %s %.2f V (armed: kept on)", to_string(k), double(h_.last_rail_v));
            fault_reported_ = true;
        }
        return;
    }
    if (state_ == RailState::LATCHED_ON || state_ == RailState::LATCHED_OFF) {
        return;
    }
    if (k == FaultKind::OVERVOLTAGE) {
        // 9 V commanded but rail high: FB network or Q1 failure. Retrying
        // cannot fix it and the VTX may be rated 12 V max: latch off (disarmed).
        drive_enable(false, false);
        drive_select(false);
        state_ = RailState::LATCHED_OFF;
        state_since_ = now;
        h_.latches++;
        say(Severity::CRITICAL, "VTX BEC: overvoltage %.2f V, latched OFF", double(h_.last_rail_v));
        return;
    }
    if (retries_ < cfg_.max_retries) {
        retries_++;
        h_.retries++;
        drive_enable(false, false);
        drive_select(false);
        state_ = RailState::RETRY_OFF;
        state_since_ = now;
        say(Severity::WARNING, "VTX BEC: %s %.2f V, power cycle %u/%u", to_string(k), double(h_.last_rail_v),
            unsigned(retries_), unsigned(cfg_.max_retries));
        return;
    }
    state_ = RailState::LATCHED_ON;    // UV: keep EN on, TPS62933 hiccup protects itself
    state_since_ = now;
    h_.latches++;
    say(Severity::CRITICAL, "VTX BEC: %s persists after %u retries, latched", to_string(k), unsigned(retries_));
}

void VtxBecSupervisor::update(const Inputs& in)
{
    const uint32_t now = in.now_ms;
    h_.updates++;

    const bool vbat_ok = in.vbat_valid && isfinite(in.vbat_v) && in.vbat_v >= cfg_.vbat_valid_min &&
                         in.vbat_v <= cfg_.vbat_valid_max;

    // ---- VBAT qualification timers (continuous-condition semantics) ----
    if (vbat_ok && in.vbat_v >= cfg_.v12_enable_vbat) {
        if (!hi_valid_) { hi_valid_ = true; hi_since_ = now; }
    } else {
        hi_valid_ = false;
    }
    if (!vbat_ok || in.vbat_v < cfg_.v12_drop_vbat) {
        if (!lo_valid_) { lo_valid_ = true; lo_since_ = now; }
    } else {
        lo_valid_ = false;
    }
    const bool qualified = hi_valid_ && elapsed(now, hi_since_, cfg_.v12_qualify_ms);
    const bool low_persist = lo_valid_ && elapsed(now, lo_since_, cfg_.v12_drop_ms);

    if (!in.want_12v) {
        lockout12_ = false;
        refused_reported_ = false;
    }

    // ---- power intent ----
    if (!in.want_power) {
        if (in.armed) {
            if (!(h_.flags & F_OFF_DEFERRED)) {
                h_.off_deferred++;
                say(Severity::WARNING, "VTX BEC: off request deferred until disarm");
            }
            h_.flags |= F_OFF_DEFERRED;
        } else if (state_ != RailState::OFF) {
            drive_enable(false, false);
            drive_select(false);
            state_ = RailState::OFF;
            state_since_ = now;
            fault_ = FaultKind::NONE;
            fault_reported_ = false;
            retries_ = 0;
            h_.flags &= ~uint32_t(F_OFF_DEFERRED | F_FAULT_ACTIVE);
            say(Severity::INFO, "VTX BEC: off");
        }
    } else {
        h_.flags &= ~uint32_t(F_OFF_DEFERRED);
        if (state_ == RailState::OFF) {
            start(now, cfg_.settle_ms);
            say(Severity::INFO, "VTX BEC: on at %.2f V", double(t9_));
        }
    }
    if (state_ == RailState::RETRY_OFF) {
        if (in.armed) {
            start(now, cfg_.settle_ms);           // never leave the VTX dark while armed
            say(Severity::WARNING, "VTX BEC: armed during retry, rail re-enabled");
        } else if (elapsed(now, state_since_, cfg_.retry_off_ms)) {
            start(now, cfg_.settle_ms);
        }
    }

    // ---- 12 V interlock ----
    if (sel12_) {
        const bool drop_req = !in.want_12v;
        const bool drop_bat = low_persist;
        if (drop_req || drop_bat) {
            drive_select(false);
            ov_blank_until_ = now + cfg_.down_settle_ms;
            settle_until_ = now + cfg_.settle_ms;
            bad_valid_ = false;
            if (drop_bat) {
                h_.v12_drops_lowbat++;
                say(Severity::WARNING, "VTX BEC: VBAT %.2f V < %.1f V, 12V -> 9V", double(in.vbat_v),
                    double(cfg_.v12_drop_vbat));
            } else {
                h_.v12_drops_request++;
                say(Severity::INFO, "VTX BEC: 12V released, 9V");
            }
        }
    } else if (in.want_12v && en_ && state_ == RailState::RUNNING && !lockout12_) {
        if (qualified && (h_.flags & F_RAIL_OK)) {
            drive_select(true);
            settle_until_ = now + cfg_.settle_ms;
            bad_valid_ = false;
            refused_reported_ = false;
            h_.v12_grants++;
            say(Severity::INFO, "VTX BEC: 12V on (VBAT %.2f V)", double(in.vbat_v));
        } else if (!hi_valid_ && !refused_reported_) {
            h_.v12_refused++;
            refused_reported_ = true;
            say(Severity::WARNING, "VTX BEC: 12V refused, VBAT %.2f V < %.1f V (4S only)", double(in.vbat_v),
                double(cfg_.v12_enable_vbat));
        }
    }

    // ---- rail check ----
    const bool checking = en_ && (state_ == RailState::STARTING || state_ == RailState::RUNNING ||
                                  state_ == RailState::LATCHED_ON);
    if (checking && !before(now, settle_until_)) {
        float pin = 0.0f;
        const bool have = adc_.read_pin_v(pin) && isfinite(pin);
        FaultKind bad = FaultKind::NONE;
        bool ok = false;
        if (have) {
            h_.sense_reads++;
            const float v = pin * cfg_.sense_mult;
            const Window w = rail_window(sel12_, in.vbat_v, vbat_ok);
            h_.last_rail_v = v;
            h_.last_window = w;
            const bool blank_hi = before(now, ov_blank_until_);
            if (v < w.lo) {
                bad = FaultKind::UNDERVOLTAGE;
            } else if (v > w.hi && !blank_hi) {
                bad = FaultKind::OVERVOLTAGE;
            } else {
                ok = (v <= w.hi);         // above window but still decaying: neither OK nor fault
            }
        } else {
            h_.sense_missing++;
            bad = FaultKind::NO_SENSE;
        }
        if (bad == FaultKind::NONE) {
            bad_valid_ = false;
            if (ok) {
                h_.flags |= F_RAIL_OK;
                if (state_ == RailState::STARTING) {
                    state_ = RailState::RUNNING;
                    state_since_ = now;
                }
                if ((h_.flags & F_FAULT_ACTIVE) && state_ == RailState::RUNNING) {
                    h_.flags &= ~uint32_t(F_FAULT_ACTIVE);
                    fault_ = FaultKind::NONE;
                    fault_reported_ = false;
                    say(Severity::NOTICE, "VTX BEC: rail recovered %.2f V", double(h_.last_rail_v));
                }
            } else {
                h_.flags &= ~uint32_t(F_RAIL_OK);
            }
        } else {
            h_.flags &= ~uint32_t(F_RAIL_OK);
            if (!bad_valid_) {
                bad_valid_ = true;
                bad_since_ = now;
            } else if (elapsed(now, bad_since_, cfg_.fault_persist_ms)) {
                declare_fault(bad, in);
            }
        }
    } else if (!checking) {
        h_.flags &= ~uint32_t(F_RAIL_OK);
    }

    // ---- flags ----
    uint32_t f = h_.flags & ~uint32_t(F_EN | F_SEL12 | F_V12_QUAL | F_V12_LOCKOUT | F_LATCHED);
    if (en_) f |= F_EN;
    if (sel12_) f |= F_SEL12;
    if (qualified) f |= F_V12_QUAL;
    if (lockout12_) f |= F_V12_LOCKOUT;
    if (state_ == RailState::LATCHED_ON || state_ == RailState::LATCHED_OFF) f |= F_LATCHED;
    h_.flags = f;
}

// ---------------------------------------------------------------------------
// TPS62933 plant
// ---------------------------------------------------------------------------
bool Tps62933Plant::q1_on() const
{
    if (p_.q1_short) {
        return true;
    }
    const float vg = (sel_drive_ == PinDrive::HIGH) ? q1_gate_drive_v() : 0.0f;
    return vg > p_.q1_vth;
}

float Tps62933Plant::rbot() const
{
    const float rsw = hw::R36_SW_OHM + (p_.q1_short ? 0.0f : p_.q1_rds_on);
    return q1_on() ? parallel(hw::R35_BOT_OHM, rsw) : hw::R35_BOT_OHM;
}

float Tps62933Plant::rtop() const
{
    return p_.r34_open ? 1e12f : hw::R34_TOP_OHM;   // open top resistor: FB pulled to GND
}

float Tps62933Plant::fb() const
{
    const float rt = rtop(), rb = rbot();
    const float leak = p_.ileak_fb + (q1_on() ? 0.0f : p_.ileak_q1);
    const float v = (vout_ / rt - leak) / (1.0f / rt + 1.0f / rb);
    return v > 0.0f ? v : 0.0f;
}

float Tps62933Plant::setpoint() const
{
    const float ref = fminf(p_.vref, ss_);
    const float leak = p_.ileak_fb + (q1_on() ? 0.0f : p_.ileak_q1);
    return ref * (1.0f + rtop() / rbot()) + leak * rtop();
}

float Tps62933Plant::en_pin_v() const
{
    switch (en_drive_) {
    case PinDrive::LOW: return 0.7e-6f * 1000.0f;      // Ip into R39 -> 0.7 mV
    case PinDrive::HIGH: return hw::GPIO_HIGH_V - 0.7e-6f * 1000.0f;
    case PinDrive::HIGH_Z: break;
    }
    // Floating: the internal Ip pull-up (DS s9.3.6) charges EN above VEN_RISE.
    return hw::EN_RISE_MAX_V + 0.5f;
}

float Tps62933Plant::sense_pin_v() const
{
    float v = vout_ * hw::R41_BOT_OHM / (hw::R40_TOP_OHM + hw::R41_BOT_OHM) * p_.sense_gain;
    if (v < 0.0f) v = 0.0f;
    if (v > p_.adc_vref) v = p_.adc_vref;
    const float code = roundf(v / p_.adc_vref * 4095.0f);
    return code * p_.adc_vref / 4095.0f;
}

void Tps62933Plant::step(float dt, float vin, float iload, float rshort)
{
    // UVLO (DS s8.5: 3.6 V rising / 3.3 V falling typ)
    if (vin >= hw::UVLO_RISE_V) uvlo_ok_ = true;
    if (vin < hw::UVLO_FALL_V) uvlo_ok_ = false;
    const bool enable = uvlo_ok_ && en_pin_v() > hw::EN_RISE_MAX_V;
    const bool disable = !uvlo_ok_ || en_pin_v() < hw::EN_FALL_MIN_V;
    if (disable || !enable) {
        running_ = false;
        ss_ = 0.0f;
        hiccup_ = false;
        ovp_ = false;
        uv_time_ = 0.0f;
    } else if (!running_) {
        running_ = true;
        ss_ = 0.0f;
    }
    const float tss = soft_start_s(p_.css, p_.iss, p_.vref);
    if (running_ && hiccup_) {
        hiccup_time_ += dt;
        if (hiccup_time_ >= hw::HICCUP_OFF_TSS * tss) {
            hiccup_ = false;
            ss_ = 0.0f;                      // normal restart with soft start
            uv_time_ = 0.0f;
        }
    }
    if (running_ && !hiccup_) {
        ss_ = fminf(ss_ + p_.iss / p_.css * dt, 3.0f);
    }

    const float v = vout_;
    const float i_short = rshort > 0.0f ? v / rshort : 0.0f;
    const float i_load = v > 0.05f ? iload : 0.0f;
    const float i_bleed = v / (rtop() + rbot()) + v / (hw::R40_TOP_OHM + hw::R41_BOT_OHM);
    const float itot = i_load + i_short + i_bleed;

    // OVP (DS s9.3.11): HS off above 115 %, released below 115 % - 5 %.
    const float fbv = fb();
    if (!ovp_ && fbv > hw::OVP_RATIO * p_.vref) {
        ovp_ = true;
        ovp_events_++;
    } else if (ovp_ && fbv < (hw::OVP_RATIO - hw::OVP_HYST) * p_.vref) {
        ovp_ = false;
    }

    const bool sw = running_ && !hiccup_ && !ovp_;
    const float desired = fminf(setpoint(), dropout_vout_max(vin, i_load + i_short, p_.dmax, p_.rhs, p_.rls, p_.dcr));
    float nv;
    bool ilimited = false;
    if (sw && (v < desired || itot > p_.ilim)) {
        // Source up to the average current limit (DS Eq.11); above it the
        // output collapses (overload / short).
        const float dv = (p_.ilim - itot) / p_.cout_eff_f * dt;
        ilimited = (v + dv) < desired;   // could not reach the set point at the limit
        nv = fminf(desired, v + dv);
    } else if (sw) {
        // Above the set point: pulse skipping, no sink (TPS62933 is not the
        // FCCM "F" variant): load + bleed discharge Cout down to the set point.
        nv = fmaxf(desired, v - itot / p_.cout_eff_f * dt);
    } else {
        nv = v - itot / p_.cout_eff_f * dt;
    }
    vout_ = nv > 0.0f ? nv : 0.0f;

    // UVP hiccup (DS s9.3.12): FB < 65 % with current limit for > 256 us,
    // only after soft start has finished.
    if (sw && ss_ >= p_.vref && fbv < hw::UVP_RATIO * p_.vref && ilimited) {
        uv_time_ += dt;
        if (uv_time_ > hw::HICCUP_ON_S) {
            hiccup_ = true;
            hiccup_time_ = 0.0f;
            hiccup_events_++;
            uv_time_ = 0.0f;
        }
    } else {
        uv_time_ = 0.0f;
    }
}

void Tps62933Plant::run(float seconds, float vin, float iload, float rshort, float dt)
{
    const long n = lroundf(seconds / dt);
    for (long i = 0; i < n; i++) {
        step(dt, vin, iload, rshort);
    }
}

} // namespace nano_vtx
