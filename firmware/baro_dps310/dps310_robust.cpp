// Nano H743 AIO - DPS310 barometer driver core with I2C bus-noise immunity (Revision 12.0)
// Sreesai Arjun Engineering
#include "dps310_robust.h"

#include <math.h>
#include <string.h>

namespace nano_baro {

// ---------------------------------------------------------------------------
// Pure helpers
// ---------------------------------------------------------------------------
int32_t sign_extend(uint32_t v, unsigned bits)
{
    if (bits == 0 || bits > 32) {
        return 0;
    }
    if (bits == 32) {
        return int32_t(int64_t(v) - ((v & 0x80000000u) ? (int64_t(1) << 32) : 0));
    }
    const uint32_t mask = (1u << bits) - 1u;
    const uint32_t sign = 1u << (bits - 1);
    v &= mask;
    // (v ^ sign) - sign: classic branch-free sign extension, all in int64 -> no overflow
    return int32_t(int64_t(v ^ sign) - int64_t(sign));
}

int32_t raw24(const uint8_t* b)
{
    return sign_extend((uint32_t(b[0]) << 16) | (uint32_t(b[1]) << 8) | uint32_t(b[2]), 24);
}

int32_t scale_factor(uint8_t prc)
{
    static const int32_t k[8] = {524288, 1572864, 3670016, 7864320, 253952, 516096, 1040384, 2088960};
    return prc < 8 ? k[prc] : 0;
}

uint32_t measurement_time_us(uint8_t prc)
{
    static const uint32_t t[8] = {3600, 5200, 8400, 14800, 27600, 53200, 104400, 206800};
    return prc < 8 ? t[prc] : 0xFFFFFFFFu;
}

uint32_t background_budget_us(uint8_t pm_rate, uint8_t pm_prc, uint8_t tmp_rate, uint8_t tmp_prc)
{
    if (pm_rate > 7 || tmp_rate > 7 || pm_prc > 7 || tmp_prc > 7) {
        return 0xFFFFFFFFu;
    }
    return (1u << pm_rate) * measurement_time_us(pm_prc) + (1u << tmp_rate) * measurement_time_us(tmp_prc);
}

bool Coefficients::operator==(const Coefficients& o) const
{
    return c0 == o.c0 && c1 == o.c1 && c00 == o.c00 && c10 == o.c10 && c01 == o.c01 &&
           c11 == o.c11 && c20 == o.c20 && c21 == o.c21 && c30 == o.c30;
}

static uint32_t u16be(const uint8_t* b) { return (uint32_t(b[0]) << 8) | b[1]; }

Coefficients parse_coefficients(const uint8_t b[18])
{
    Coefficients c;
    c.c0  = sign_extend((uint32_t(b[0]) << 4) | (uint32_t(b[1]) >> 4), 12);
    c.c1  = sign_extend((uint32_t(b[1] & 0x0F) << 8) | b[2], 12);
    c.c00 = sign_extend((uint32_t(b[3]) << 12) | (uint32_t(b[4]) << 4) | (uint32_t(b[5]) >> 4), 20);
    c.c10 = sign_extend((uint32_t(b[5] & 0x0F) << 16) | (uint32_t(b[6]) << 8) | b[7], 20);
    c.c01 = sign_extend(u16be(&b[8]), 16);
    c.c11 = sign_extend(u16be(&b[10]), 16);
    c.c20 = sign_extend(u16be(&b[12]), 16);
    c.c21 = sign_extend(u16be(&b[14]), 16);
    c.c30 = sign_extend(u16be(&b[16]), 16);
    return c;
}

void encode_coefficients(const Coefficients& c, uint8_t b[18])
{
    const uint32_t c0 = uint32_t(c.c0) & 0xFFFu, c1 = uint32_t(c.c1) & 0xFFFu;
    const uint32_t c00 = uint32_t(c.c00) & 0xFFFFFu, c10 = uint32_t(c.c10) & 0xFFFFFu;
    b[0] = uint8_t(c0 >> 4);
    b[1] = uint8_t(((c0 & 0x0F) << 4) | (c1 >> 8));
    b[2] = uint8_t(c1 & 0xFF);
    b[3] = uint8_t(c00 >> 12);
    b[4] = uint8_t((c00 >> 4) & 0xFF);
    b[5] = uint8_t(((c00 & 0x0F) << 4) | (c10 >> 16));
    b[6] = uint8_t((c10 >> 8) & 0xFF);
    b[7] = uint8_t(c10 & 0xFF);
    const int32_t v16[7] = {c.c01, c.c11, c.c20, c.c21, c.c30, 0, 0};
    for (int i = 0; i < 5; i++) {
        const uint32_t u = uint32_t(v16[i]) & 0xFFFFu;
        b[8 + 2 * i] = uint8_t(u >> 8);
        b[9 + 2 * i] = uint8_t(u & 0xFF);
    }
}

bool coefficients_sane(const uint8_t b[18], const Coefficients& c)
{
    // 1. A floating bus (SDA released, all ones) or SDA stuck low (all zeros)
    //    reads the same pattern twice, so the dual read alone cannot catch it.
    bool all00 = true, allFF = true;
    for (int i = 0; i < 18; i++) {
        all00 = all00 && b[i] == 0x00;
        allFF = allFF && b[i] == 0xFF;
    }
    if (all00 || allFF) {
        return false;
    }
    // 2. Zero temperature or pressure sensitivity is physically impossible.
    if (c.c1 == 0 || c.c10 == 0) {
        return false;
    }
    // 3. Saturated (most-negative) codes in the two primary terms indicate a
    //    shifted / truncated block (e.g. register pointer slipped by a byte).
    if (c.c0 == -2048 || c.c1 == -2048 || c.c00 == -524288 || c.c10 == -524288) {
        return false;
    }
    // 4. The compensated temperature must be a real temperature somewhere in
    //    the raw range the ADC can report at 1x (|Traw_sc| <= 16): T(0) = c0/2
    //    and slope c1 must reach [-40, 85] C for some |Traw_sc| <= 16.
    const float t_lo = 0.5f * float(c.c0) - 16.0f * fabsf(float(c.c1));
    const float t_hi = 0.5f * float(c.c0) + 16.0f * fabsf(float(c.c1));
    if (t_hi < -40.0f || t_lo > 85.0f) {
        return false;
    }
    return true;
}

void compensate(const Coefficients& cal, int32_t UP_, int32_t UT, int32_t kp, int32_t kt,
                float& pressure, float& temperature)
{
    const float temp_scaled  = float(UT) / float(kt);
    const float press_scaled = float(UP_) / float(kp);
    temperature = float(cal.c0) * 0.5f + float(cal.c1) * temp_scaled;
    pressure = float(cal.c00);
    pressure += press_scaled * (float(cal.c10) + press_scaled * (float(cal.c20) + press_scaled * float(cal.c30)));
    pressure += temp_scaled * float(cal.c01);
    pressure += temp_scaled * press_scaled * (float(cal.c11) + press_scaled * float(cal.c21));
}

float max_dp_dh(const Config& c)
{
    const double R = 287.05, g = 9.80665;
    const double rho = double(c.p_max_pa) / (R * (double(c.t_min_c) + 273.15));
    return float(rho * g);
}

const char* to_string(I2cStatus s)
{
    switch (s) {
    case I2cStatus::OK: return "OK";
    case I2cStatus::NACK: return "NACK";
    case I2cStatus::BUS_ERROR: return "BUS_ERROR";
    case I2cStatus::ARB_LOST: return "ARB_LOST";
    case I2cStatus::TIMEOUT: return "TIMEOUT";
    }
    return "?";
}

const char* to_string(Update u)
{
    switch (u) {
    case Update::NEW_SAMPLE: return "NEW_SAMPLE";
    case Update::NO_DATA: return "NO_DATA";
    case Update::REJECTED: return "REJECTED";
    case Update::NOT_RUNNING: return "NOT_RUNNING";
    case Update::BUS_FAULT: return "BUS_FAULT";
    }
    return "?";
}

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------
Dps310::Dps310(I2cTransport& bus, I2cBusGpio* gpio, const Config& cfg)
    : bus_(bus), gpio_(gpio), cfg_(cfg)
{
    kp_ = scale_factor(cfg_.pm_prc);
    kt_ = scale_factor(cfg_.tmp_prc);
    period_us_ = cfg_.pm_rate < 8 ? 1000000u >> cfg_.pm_rate : 1000000u;
}

uint8_t Dps310::prs_cfg_value() const { return uint8_t(((cfg_.pm_rate & 7) << 4) | (cfg_.pm_prc & 0x0F)); }
uint8_t Dps310::tmp_cfg_value() const
{
    return uint8_t((temp_src_ & 0x80) | ((cfg_.tmp_rate & 7) << 4) | (cfg_.tmp_prc & 0x0F));
}
uint8_t Dps310::cfg_reg_value() const
{
    return uint8_t((needs_shift(cfg_.pm_prc) ? CFG_P_SHIFT : 0) | (needs_shift(cfg_.tmp_prc) ? CFG_T_SHIFT : 0));
}

void Dps310::note_failure(I2cStatus st)
{
    switch (st) {
    case I2cStatus::NACK: h_.nack++; break;
    case I2cStatus::BUS_ERROR: h_.bus_error++; break;
    case I2cStatus::ARB_LOST: h_.arb_lost++; break;
    case I2cStatus::TIMEOUT: h_.timeout++; break;
    case I2cStatus::OK: break;
    }
}

// Bounded retry. A TIMEOUT with SDA held low is not retried: a slave is
// mid-byte and only a bus clear helps (same trigger as ArduPilot
// HAL_I2C_CLEAR_ON_TIMEOUT, but here the clear is SDA-monitored and ends in a STOP).
I2cStatus Dps310::rd(uint8_t r, uint8_t* b, size_t n)
{
    I2cStatus st = I2cStatus::TIMEOUT;
    for (uint8_t a = 0; a < cfg_.max_attempts; a++) {
        if (a) h_.retries++;
        st = bus_.read_registers(r, b, n);
        if (st == I2cStatus::OK) {
            h_.xfer_ok++;
            h_.consec_failed = 0;
            return st;
        }
        note_failure(st);
        if (st == I2cStatus::TIMEOUT && gpio_ && !gpio_->read_sda()) {
            h_.sda_stuck_seen++;
            need_recovery_ = true;
            break;
        }
    }
    h_.xfer_failed++;
    h_.consec_failed++;
    if (h_.consec_failed > h_.max_consec_failed) h_.max_consec_failed = h_.consec_failed;
    if (h_.consec_failed >= cfg_.fail_threshold) need_recovery_ = true;
    return st;
}

I2cStatus Dps310::wr(uint8_t r, uint8_t v)
{
    I2cStatus st = I2cStatus::TIMEOUT;
    for (uint8_t a = 0; a < cfg_.max_attempts; a++) {
        if (a) h_.retries++;
        st = bus_.write_register(r, v);
        if (st == I2cStatus::OK) {
            h_.xfer_ok++;
            h_.consec_failed = 0;
            return st;
        }
        note_failure(st);
        if (st == I2cStatus::TIMEOUT && gpio_ && !gpio_->read_sda()) {
            h_.sda_stuck_seen++;
            need_recovery_ = true;
            break;
        }
    }
    h_.xfer_failed++;
    h_.consec_failed++;
    if (h_.consec_failed > h_.max_consec_failed) h_.max_consec_failed = h_.consec_failed;
    if (h_.consec_failed >= cfg_.fail_threshold) need_recovery_ = true;
    return st;
}

// Bus clear, UM10204 rev 7 s3.1.16 (and NXP AN10216 / AN13202 bus-clear notes):
// release SDA, clock SCL up to 9 times until SDA is seen high, then STOP.
//
// The STOP is generated WITHOUT another SCL falling edge. A slave-transmitter
// hung mid-READ changes SDA only on SCL falling edges; the old sequence (SCL
// low, SDA low, SCL high, SDA high) gave it one more falling edge, so a 0 in
// its next bit held SDA low through the "STOP" and the bus stayed hung (e.g.
// remaining bits 0,0,1,0: 2 clocks, SDA released on the '1', SCL falls, the
// next '0' kills the STOP). Here, whenever SCL is high and SDA reads high:
//   SDA low  (SCL high)  = START  (s3.1.4: slaves reset their bit counter on a
//                                   START at any bit position)
//   SDA high (SCL high)  = STOP   -> every slave returns to idle
// Neither edge is an SCL edge, so no slave can shift out a new bit in between.
// Cases covered (all bounded by 9 clocks):
//   - slave-transmitter mid-read, bit 0..7: SDA follows its data; the first
//     '1' bit or, at the latest, the master-ACK slot (SDA released by us =
//     NACK, transmitter stops) gives SDA high: worst case 8 clocks (0x00 at MSB).
//   - slave-receiver in its ACK slot of a write / address byte: one falling
//     edge ends the ACK, SDA high -> START/STOP before the next byte's ACK.
//     (This is why "always clock all 9" is NOT used: a receiver that released
//     after clock 1 would ACK again on clock 9 and SDA would be low again.)
//   - slave-receiver mid data bits: SDA already high, 0 clocks.
// If SDA is low again after the STOP (another device), clocking resumes within
// the same 9-clock budget. SCL held low (stretch > 0.5 ms) -> fail.
// Timing at hp = 5 us: tSU;STA, tHD;STA, tSU;STO, tBUF all >= 4.7 us (Sm).
bool Dps310::recover_bus()
{
    h_.recoveries++;
    h_.flags |= F_RECOVERING;
    need_recovery_ = false;
    h_.consec_failed = 0;
    if (!gpio_) {
        h_.recovery_failed++;
        h_.flags |= F_RECOVERY_FAIL;
        h_.flags &= ~uint32_t(F_RECOVERING);
        return false;
    }
    const uint32_t hp = cfg_.clear_half_period_us;
    auto wait_scl_high = [&]() {
        for (int i = 0; i < 100; i++) {          // 100 x hp = 0.5 ms stretch limit
            if (gpio_->read_scl()) return true;
            gpio_->delay_us(hp);
        }
        return false;
    };
    gpio_->begin_bitbang();
    gpio_->set_sda(true);
    gpio_->set_scl(true);
    gpio_->delay_us(hp);
    bool ok = wait_scl_high();          // SCL held low by someone: cannot clear
    bool idle = false;
    uint32_t clocks = 0;
    while (ok) {
        if (gpio_->read_sda()) {
            // SCL high, SDA high: START then STOP, SCL untouched.
            gpio_->set_sda(false);
            gpio_->delay_us(hp);        // tHD;STA / tSU;STO
            gpio_->set_sda(true);
            gpio_->delay_us(hp);        // tBUF
            if (gpio_->read_sda() && gpio_->read_scl()) { idle = true; break; }
        }
        if (clocks >= 9) break;
        gpio_->set_scl(false);          // SDA stays released: master-ACK slot reads NACK
        gpio_->delay_us(hp);
        gpio_->set_scl(true);
        ok = wait_scl_high();
        gpio_->delay_us(hp);
        clocks++;
    }
    h_.last_clear_clocks = clocks;
    ok = ok && idle;
    gpio_->end_bitbang();
    h_.flags &= ~uint32_t(F_RECOVERING);
    if (!ok) {
        h_.recovery_failed++;
        h_.flags |= F_RECOVERY_FAIL;
        return false;
    }
    h_.flags &= ~uint32_t(F_RECOVERY_FAIL);
    return true;
}

// ArduPilot toggles CS to escape the boot WHOAMI state; on I2C that call is a
// no-op, so the escape here is: re-read, then soft reset and re-read.
bool Dps310::probe()
{
    for (int phase = 0; phase < 2; phase++) {
        bool any_ack = false;
        for (uint8_t i = 0; i < cfg_.probe_reads; i++) {
            uint8_t id = 0;
            if (rd(reg::PRODUCT_ID, &id, 1) == I2cStatus::OK) {
                any_ack = true;
                if (id == PRODUCT_ID_VALUE) {
                    h_.flags |= F_PRESENT;
                    return true;
                }
                h_.whoami_bad++;
            }
            bus_.delay_us(1000);
        }
        if (!any_ack || phase == 1) {
            break;
        }
        h_.quirk_resets++;
        wr(reg::RESET, SOFT_RESET_CMD);
        bus_.delay_us(40000);           // coefficients ready <= 40 ms after reset
    }
    h_.flags &= ~uint32_t(F_PRESENT);
    return false;
}

bool Dps310::read_coefficients(Coefficients& out, uint8_t& srce)
{
    uint8_t prev[18], cur[18];
    bool have_prev = false;
    for (uint8_t i = 0; i < cfg_.coef_max_reads; i++) {
        h_.coef_reads++;
        if (rd(reg::COEF, cur, 18) != I2cStatus::OK) {
            have_prev = false;
            if (need_recovery_) return false;
            continue;
        }
        if (have_prev) {
            if (memcmp(prev, cur, 18) == 0) {
                const Coefficients c = parse_coefficients(cur);
                if (!coefficients_sane(cur, c)) {
                    h_.coef_insane++;
                    return false;
                }
                // COEF_SRCE: read twice as well, bit 7 only
                uint8_t s1 = 0, s2 = 0;
                if (rd(reg::COEF_SRCE, &s1, 1) != I2cStatus::OK ||
                    rd(reg::COEF_SRCE, &s2, 1) != I2cStatus::OK || ((s1 ^ s2) & 0x80)) {
                    h_.coef_mismatch++;
                    return false;
                }
                out = c;
                srce = uint8_t(s1 & 0x80);
                return true;
            }
            h_.coef_mismatch++;
        }
        memcpy(prev, cur, 18);
        have_prev = true;
    }
    return false;
}

bool Dps310::apply_temperature_fix()
{
    // Order from ArduPilot set_config_registers() / Infineon DpsClass.cpp correctTemp()
    static const uint8_t seq[5][2] = {{reg::FIX_A, 0xA5}, {reg::FIX_B, 0x96}, {reg::FIX_C, 0x02},
                                      {reg::FIX_A, 0x00}, {reg::FIX_B, 0x00}};
    for (int attempt = 0; attempt < 2; attempt++) {
        bool ok = true;
        for (int i = 0; i < 5 && ok; i++) {
            ok = wr(seq[i][0], seq[i][1]) == I2cStatus::OK;
        }
        if (ok) return true;
        if (need_recovery_) return false;
    }
    return false;
}

bool Dps310::config_matches(const uint8_t c[4]) const
{
    return c[0] == prs_cfg_value() && c[1] == tmp_cfg_value() &&
           (c[2] & MEAS_CTRL_MASK) == MEAS_CTRL_BG_PT && c[3] == cfg_reg_value();
}

// Write in standby, read back as one burst, compare all bits, then start
// background mode and read that back too. Up to 3 attempts.
bool Dps310::write_config_verified()
{
    for (int attempt = 0; attempt < 3; attempt++) {
        h_.config_writes++;
        bool ok = wr(reg::MEAS_CFG, 0x00) == I2cStatus::OK &&            // standby
                  wr(reg::CFG_REG, cfg_reg_value()) == I2cStatus::OK &&   // shifts first
                  wr(reg::PRS_CFG, prs_cfg_value()) == I2cStatus::OK &&
                  wr(reg::TMP_CFG, tmp_cfg_value()) == I2cStatus::OK;
        if (ok) ok = apply_temperature_fix();
        uint8_t rb[4] = {0, 0, 0, 0};
        if (ok) ok = rd(reg::PRS_CFG, rb, 4) == I2cStatus::OK;
        if (ok && (rb[0] != prs_cfg_value() || rb[1] != tmp_cfg_value() || rb[3] != cfg_reg_value())) {
            h_.config_readback_fail++;
            continue;
        }
        if (ok) ok = wr(reg::MEAS_CFG, MEAS_CTRL_BG_PT) == I2cStatus::OK;
        if (ok) ok = rd(reg::PRS_CFG, rb, 4) == I2cStatus::OK;
        if (ok && config_matches(rb)) {
            h_.flags |= F_CONFIG_OK;
            return true;
        }
        if (ok) h_.config_readback_fail++;
        if (need_recovery_) break;
    }
    h_.flags &= ~uint32_t(F_CONFIG_OK);
    return false;
}

bool Dps310::init(uint32_t now_us)
{
    h_.inits++;
    h_.flags &= ~uint32_t(F_RUNNING | F_CONFIG_OK | F_HEALTHY);
    state_ = DriverState::UNINIT;
    auto fail = [&]() {
        h_.init_failed++;
        state_ = DriverState::HOLDOFF;
        holdoff_until_ = now_us + cfg_.recovery_holdoff_us;
        return false;
    };
    const bool shift_ok = cfg_.pm_prc < 8 && cfg_.tmp_prc < 8 && cfg_.pm_rate < 8 && cfg_.tmp_rate < 8;
    if (!shift_ok || background_budget_us(cfg_.pm_rate, cfg_.pm_prc, cfg_.tmp_rate, cfg_.tmp_prc) >= 1000000u) {
        h_.flags |= F_BUDGET_FAIL;
        return fail();
    }
    if (!probe()) return fail();
    // SENSOR_RDY and COEF_RDY, up to 100 ms
    bool ready = false;
    for (int i = 0; i < 20 && !ready; i++) {
        uint8_t m = 0;
        if (rd(reg::MEAS_CFG, &m, 1) == I2cStatus::OK &&
            (m & (MEAS_COEF_RDY | MEAS_SENSOR_RDY)) == (MEAS_COEF_RDY | MEAS_SENSOR_RDY)) {
            ready = true;
        } else {
            if (need_recovery_) return fail();
            bus_.delay_us(5000);
        }
    }
    if (!ready) {
        h_.coef_not_ready++;
        return fail();
    }
    Coefficients c;
    uint8_t srce = 0;
    if (!read_coefficients(c, srce)) {
        if (!have_coef_) h_.flags &= ~uint32_t(F_COEF_VALID);
        return fail();
    }
    if (have_coef_ && (c != coef_ || srce != temp_src_)) {
        // NVM cannot change: a self-consistent different block means the
        // previous or the new read is wrong. Refuse to run on either.
        h_.flags |= F_COEF_CHANGED;
        h_.flags &= ~uint32_t(F_COEF_VALID);
        h_.coef_mismatch++;
        return fail();
    }
    coef_ = c;
    temp_src_ = srce;
    have_coef_ = true;
    h_.flags |= F_COEF_VALID;
    if (!write_config_verified()) return fail();
    state_ = DriverState::RUNNING;
    h_.flags |= F_RUNNING;
    pend_n_ = 0;
    return true;
}

float Dps310::gate_pa(uint32_t dt_us) const
{
    return cfg_.max_vz_mps * max_dp_dh(cfg_) * (float(dt_us) * 1e-6f) + cfg_.gate_margin_pa;
}

Update Dps310::plausibility(float p, float t, uint32_t now)
{
    h_.samples++;
    if (!(p >= cfg_.p_min_pa && p <= cfg_.p_max_pa)) {   // also rejects NaN
        h_.rej_range_p++;
        return Update::REJECTED;
    }
    if (!(t >= cfg_.t_min_c && t <= cfg_.t_max_c)) {
        h_.rej_range_t++;
        return Update::REJECTED;
    }
    bool accept = false;
    if (ref_valid_ && fabsf(p - ref_p_) <= gate_pa(now - ref_t_)) {
        accept = true;
    } else {
        if (ref_valid_) h_.rej_rate++;
        // candidate new level: consistent with the previous out-of-gate sample?
        if (pend_n_ > 0 && fabsf(p - pend_p_) <= gate_pa(now - pend_t_)) {
            pend_n_++;
        } else {
            pend_n_ = 1;
        }
        pend_p_ = p;
        pend_t_ = now;
        if (pend_n_ >= cfg_.reanchor_samples) {
            if (ref_valid_) h_.reanchors++;
            accept = true;
        }
    }
    if (!accept) {
        return Update::REJECTED;
    }
    pend_n_ = 0;
    ref_valid_ = true;
    ref_p_ = p;
    ref_t_ = now;
    last_.pressure_pa = p;
    last_.temperature_c = t;
    last_.t_us = now;
    last_.valid = true;
    h_.accepted++;
    return Update::NEW_SAMPLE;
}

Update Dps310::update(uint32_t now)
{
    if (last_.valid && now - last_.t_us > 4 * period_us_) {
        h_.flags &= ~uint32_t(F_HEALTHY);
    }
    if (state_ == DriverState::HOLDOFF) {
        if (int32_t(now - holdoff_until_) < 0) {
            return Update::NOT_RUNNING;
        }
        state_ = DriverState::UNINIT;
    }
    if (need_recovery_) {
        if (!recover_bus()) {
            state_ = DriverState::HOLDOFF;
            holdoff_until_ = now + cfg_.recovery_holdoff_us;
            return Update::BUS_FAULT;
        }
        state_ = DriverState::UNINIT;     // re-verify coefficients and config
    }
    if (state_ == DriverState::UNINIT) {
        if (!init(now)) {
            return Update::NOT_RUNNING;
        }
    }

    // status poll = config read-back on every sample (4 bytes, ~0.1 ms at 400 kHz)
    uint8_t c4[4];
    if (rd(reg::PRS_CFG, c4, 4) != I2cStatus::OK) {
        return Update::BUS_FAULT;
    }
    h_.polls++;
    if (!config_matches(c4)) {
        // confirm with a second read so a bit error on the poll is not a reconfig
        if (rd(reg::PRS_CFG, c4, 4) != I2cStatus::OK) {
            return Update::BUS_FAULT;
        }
        if (!config_matches(c4)) {
            h_.config_lost++;
            h_.flags &= ~uint32_t(F_CONFIG_OK | F_HEALTHY);
            if (init(now)) {
                h_.config_repairs++;
            }
            return Update::NO_DATA;
        }
    }
    if (!(c4[2] & MEAS_PRS_RDY)) {
        h_.not_ready++;
        return Update::NO_DATA;
    }
    uint8_t d[6];
    if (rd(reg::PSR_B2, d, 6) != I2cStatus::OK) {
        return Update::BUS_FAULT;
    }
    bool all00 = true, allFF = true;
    for (int i = 0; i < 6; i++) {
        all00 = all00 && d[i] == 0;
        allFF = allFF && d[i] == 0xFF;
    }
    if (all00 || allFF) {
        h_.samples++;
        h_.rej_pattern++;
        return Update::REJECTED;
    }
    float p = 0, t = 0;
    compensate(coef_, raw24(&d[0]), raw24(&d[3]), kp_, kt_, p, t);
    const Update u = plausibility(p, t, now);
    if (u == Update::NEW_SAMPLE) {
        h_.flags |= F_HEALTHY;
    }
    return u;
}

} // namespace nano_baro
