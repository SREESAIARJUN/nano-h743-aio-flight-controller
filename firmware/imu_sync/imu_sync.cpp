// Nano H743 AIO - dual ICM-42688-P synchronisation reference (Revision 11.0)
// Sreesai Arjun Engineering
#include "imu_sync.h"

#include <cmath>

namespace nano_imu_sync {

// ===========================================================================
// a) Register plan
// ===========================================================================
uint32_t normalise_backend_rate(uint32_t requested_hz)
{
    switch (requested_hz) {
    case 2000: case 4000: case 8000: return requested_hz;
    default: return 1000;   // ArduPilot switch default: "1KHz"
    }
}

uint8_t odr_code_for_rate(uint32_t backend_rate_hz)
{
    switch (normalise_backend_rate(backend_rate_hz)) {
    case 2000: return 0x05;
    case 4000: return 0x04;
    case 8000: return 0x03;
    default:   return 0x06;
    }
}

// ICM-42688 / ICM-40609 / IIM-42652 branch of set_filter_and_scaling();
// bandwidths cross-checked against DS-000347 s5.3 AAF table (6/36/10 = 258 Hz,
// 12/144/8 = 536 Hz, 21/440/6 = 997 Hz, accel 5/25/10 = 213 Hz).  The
// "1KHz, 348Hz AAF" comment in upstream's switch default is stale: the 1 kHz
// coefficients stay 6/36/10 (= 258 Hz).
AafCoeffs gyro_aaf_for_rate(uint32_t backend_rate_hz)
{
    switch (normalise_backend_rate(backend_rate_hz)) {
    case 2000: return AafCoeffs{12, 144, 8, 536};
    case 4000:
    case 8000: return AafCoeffs{21, 440, 6, 997};
    default:   return AafCoeffs{6, 36, 10, 258};
    }
}

AafCoeffs accel_aaf() { return AafCoeffs{5, 25, 10, 213}; }

double odr_hz_with_clkin(uint32_t nominal_odr_hz, double clkin_hz)
{
    return double(nominal_odr_hz) * clkin_hz / double(CLKIN_REF_HZ);
}

bool RegPlan::write(uint8_t bank, uint8_t addr, uint8_t value, uint16_t delay_us)
{
    return modify(bank, addr, 0xFFU, value, delay_us);
}

bool RegPlan::modify(uint8_t bank, uint8_t addr, uint8_t clear_mask, uint8_t set_bits, uint16_t delay_us)
{
    if (count >= MAX_OPS) {
        overflow = true;
        return false;
    }
    op[count++] = RegOp{bank, addr, clear_mask, set_bits, delay_us};
    return true;
}

const RegOp* RegPlan::find(uint8_t bank, uint8_t addr) const
{
    for (size_t i = count; i > 0; i--) {
        if (op[i - 1].bank == bank && op[i - 1].addr == addr) return &op[i - 1];
    }
    return nullptr;
}

void build_config_plan(const PlanConfig& cfg, RegPlan& out)
{
    out.clear();
    const uint8_t   odr = odr_code_for_rate(cfg.backend_rate_hz);
    const AafCoeffs g   = gyro_aaf_for_rate(cfg.backend_rate_hz);
    const AafCoeffs a   = accel_aaf();

    // "disable gyro and accel as per 12.9 in the ICM-42688 docs"
    out.write(0, reg::PWR_MGMT0, 0x00);
    out.write(0, reg::GYRO_CONFIG0, odr);    // FS_SEL 000 = +/-2000 dps
    out.write(0, reg::ACCEL_CONFIG0, odr);   // FS_SEL 000 = +/-16 g
    // gyro AAF + notch enabled (clear AAF_DIS | NF_DIS), coefficients
    out.modify(1, reg::GYRO_CONFIG_STATIC2, 0x03, 0x00);
    out.write(1, reg::GYRO_CONFIG_STATIC3, g.delt);
    out.write(1, reg::GYRO_CONFIG_STATIC4, uint8_t(g.deltsqr & 0xFFU));
    out.write(1, reg::GYRO_CONFIG_STATIC5, uint8_t(((g.bitshift << 4) & 0xF0) | ((g.deltsqr >> 8) & 0x0F)));
    // accel AAF (DELT << 1, bit0 ACCEL_AAF_DIS = 0)
    out.write(2, reg::ACCEL_CONFIG_STATIC2, uint8_t(a.delt << 1));
    out.write(2, reg::ACCEL_CONFIG_STATIC3, uint8_t(a.deltsqr & 0xFFU));
    out.write(2, reg::ACCEL_CONFIG_STATIC4, uint8_t(((a.bitshift << 4) & 0xF0) | ((a.deltsqr >> 8) & 0x0F)));

    if (cfg.clock == ClockMode::RtcClkin) {
        // pin 9 -> CLKIN first (upstream order), then RTC_MODE
        out.modify(1, reg::INTF_CONFIG5, PIN9_FUNCTION_MASK, PIN9_FUNCTION_CLKIN);
        out.modify(0, reg::INTF_CONFIG1, uint8_t(INTF_CONFIG1_AFSR_MASK | INTF_CONFIG1_RTC_MODE),
                   uint8_t(INTF_CONFIG1_AFSR_DISABLE | INTF_CONFIG1_RTC_MODE));
    } else {
        out.modify(0, reg::INTF_CONFIG1, INTF_CONFIG1_AFSR_MASK, INTF_CONFIG1_AFSR_DISABLE);
    }
    out.write(0, reg::PWR_MGMT0, PWR_MGMT0_GYRO_ACCEL_LN, PWR_ON_DELAY_US);
}

void build_fallback_plan(RegPlan& out)
{
    out.clear();
    out.write(0, reg::PWR_MGMT0, 0x00);
    // back to the internal clock (CLKSEL untouched = 01 PLL/RC auto), AFSR fix kept
    out.modify(0, reg::INTF_CONFIG1, uint8_t(INTF_CONFIG1_AFSR_MASK | INTF_CONFIG1_RTC_MODE),
               INTF_CONFIG1_AFSR_DISABLE);
    // pin 9 stays wired to Y2 OUT: INT2 must be open-drain before it becomes an output
    out.modify(0, reg::INT_CONFIG, INT_CONFIG_INT2_PUSH_PULL, 0x00);
    out.modify(1, reg::INTF_CONFIG5, PIN9_FUNCTION_MASK, PIN9_FUNCTION_INT2);
    out.write(0, reg::PWR_MGMT0, PWR_MGMT0_GYRO_ACCEL_LN, PWR_ON_DELAY_US);
}

uint32_t execute_plan(const RegPlan& plan, const RegisterBus& bus)
{
    uint32_t xfers = 0;
    for (size_t i = 0; i < plan.count; i++) {
        const RegOp& o = plan.op[i];
        if (o.bank != 0) { bus.write(bus.ctx, reg::BANK_SEL, o.bank); xfers++; }
        if (o.is_write()) {
            bus.write(bus.ctx, o.addr, o.set_bits);
            xfers++;
        } else {
            const uint8_t v = bus.read(bus.ctx, o.addr);
            bus.write(bus.ctx, o.addr, o.apply(v));
            xfers += 2;
        }
        if (o.bank != 0) { bus.write(bus.ctx, reg::BANK_SEL, 0); xfers++; }
        if (o.delay_us && bus.delay_us) bus.delay_us(bus.ctx, o.delay_us);
    }
    return xfers;
}

// ===========================================================================
// b) FIFO timestamp
// ===========================================================================
// s12.7: RTC -> 32.768/30 us = 32768/30 ns per count; internal -> 32000/30 ns.
static uint64_t tick_num_ns(ClockMode mode) { return mode == ClockMode::RtcClkin ? 32768U : 32000U; }
static constexpr uint64_t TICK_DEN = 30U;

uint64_t tmst_ticks_to_ns(uint64_t ticks, ClockMode mode, TmstRes res)
{
    const uint64_t mul = tick_num_ns(mode) * (res == TmstRes::Res16us ? 16U : 1U);
    return (ticks * mul + TICK_DEN / 2U) / TICK_DEN;
}

uint32_t tmst_ticks_per_sample_x256(double odr_hz, ClockMode mode, TmstRes res)
{
    const double ns_per_tick = double(tick_num_ns(mode)) / double(TICK_DEN) * (res == TmstRes::Res16us ? 16.0 : 1.0);
    return uint32_t(std::lround(1e9 / odr_hz / ns_per_tick * 256.0));
}

TimestampUnwrapper::Result TimestampUnwrapper::update(uint16_t raw)
{
    Result r{};
    if (!_have) {
        _have = true;
        _last = raw;
        _acc = raw;
        r.unwrapped = _acc;
        r.first = true;
        return r;
    }
    const uint16_t d = uint16_t(raw - _last);
    if (raw < _last) _wraps++;
    _last = raw;
    _acc += d;
    r.unwrapped = _acc;
    r.delta = d;
    r.duplicate = (d == 0);
    if (_expected_x256 != 0 && d != 0) {
        const uint64_t d256 = uint64_t(d) << 8;
        const uint64_t k = (d256 + _expected_x256 / 2U) / _expected_x256;   // nearest multiple
        // tolerance: 5 % of one period, at least 2 counts (s12.7 shows +/-0.5 count toggling)
        uint64_t tol = _expected_x256 / 20U;
        if (tol < 512U) tol = 512U;
        const uint64_t target = k * _expected_x256;
        const uint64_t err = d256 > target ? d256 - target : target - d256;
        r.irregular = (k == 0) || (err > tol);
        r.missed = (k >= 1 && !r.irregular) ? uint16_t(k - 1U) : 0;
    }
    return r;
}

// ===========================================================================
// c) Dual-IMU pairing
// ===========================================================================
void DualImuPairer::reset()
{
    const PairerConfig cfg = _cfg;
    *this = DualImuPairer(cfg);
}

bool DualImuPairer::push(uint8_t imu, const ImuSample& s)
{
    Queue& q = _q[imu & 1U];
    bool ok = true;
    if (q.n == QUEUE) {
        q.pop();
        _overflow[imu & 1U]++;
        ok = false;
    }
    q.buf[(q.head + q.n) % QUEUE] = s;
    q.n++;
    return ok;
}

static int64_t abs_diff(uint64_t a, uint64_t b) { return a > b ? int64_t(a - b) : int64_t(b - a); }

bool DualImuPairer::pop_pair(PairedSample& out)
{
    const uint64_t half = _cfg.odr_period_ns / 2U;
    Queue& q1 = _q[0];
    Queue& q2 = _q[1];
    while (q1.n && q2.n) {
        const ImuSample& s1 = q1.at(0);
        // IMU2 samples older than the window can never pair with s1 or later
        while (q2.n && q2.at(0).t_ns + half < s1.t_ns) { q2.pop(); _orphans[1]++; }
        if (!q2.n) return false;
        if (q2.at(0).t_ns > s1.t_ns + half) {   // nothing for s1 in its window
            q1.pop(); _orphans[0]++;
            continue;
        }
        // nearest neighbour: a later IMU2 sample may be closer (jitter / drift)
        if (q2.n >= 2 && abs_diff(q2.at(1).t_ns, s1.t_ns) < abs_diff(q2.at(0).t_ns, s1.t_ns)) {
            q2.pop(); _orphans[1]++;
        }
        out.imu1 = s1;
        out.imu2 = q2.at(0);
        out.offset_ns = int64_t(out.imu2.t_ns) - int64_t(out.imu1.t_ns);
        q1.pop();
        q2.pop();
        _pairs++;
        account(out.offset_ns, out.imu1.t_ns);
        return true;
    }
    return false;
}

static double wrap_period(double v, double T)
{
    // into (-T/2, +T/2]
    double w = std::fmod(v, T);
    if (w > T / 2) w -= T;
    if (w <= -T / 2) w += T;
    return w;
}

void DualImuPairer::account(int64_t offset_ns, uint64_t t1_ns)
{
    const double T = double(_cfg.odr_period_ns);
    double u = double(offset_ns);
    if (_have_prev) {
        // keep the offset continuous across the +/-T/2 pairing boundary
        u += T * std::nearbyint((_prev_u - u) / T);
    }
    _have_prev = true;
    _prev_u = u;

    if (_n == 0) {
        _x0_ns = t1_ns;
        _y0 = u;
    }
    const double x = double(t1_ns - _x0_ns) * 1e-9;   // s
    const double y = u - _y0;                           // ns
    _sx += x; _sy += y; _sxx += x * x; _sxy += x * y;
    _n++;
    if (_n >= _cfg.window_pairs) {
        const double n = double(_n);
        const double den = n * _sxx - _sx * _sx;
        const double slope_ns_per_s = den > 0 ? (n * _sxy - _sx * _sy) / den : 0.0;
        _drift_ppm = slope_ns_per_s * 1e-3;   // (ns/s) / 1e9 * 1e6
        _phase_ns = wrap_period(_y0 + _sy / n, T);
        _windows++;
        _state = std::fabs(_drift_ppm) <= _cfg.drift_threshold_ppm ? SyncState::Shared : SyncState::NotShared;
        _n = 0;
        _sx = _sy = _sxx = _sxy = 0.0;
        _prev_u = wrap_period(_prev_u, T);   // re-base so the unwrapped value stays bounded
    }
}

// ===========================================================================
// d) CLKIN watchdog
// ===========================================================================
// Tolerance budget for tol_ppm = 500 (default):
//   Y2 frequency stability            +/-30 ppm  (SX2M32.768KM20F30TNN, LCSC C2901603)
//   MCU reference (24 MHz HSE crystal) +/-50 ppm  ASSUMED (part tolerance+temp+aging
//                                                  not verified for this board)
//   => worst-case true reading error   +/-80 ppm; 500 ppm is >6x margin so a
//      healthy clock never trips, while every physical fault mode is far
//      outside: wrong 32.000 kHz part (-23 400 ppm), ringing double-count
//      (+1e6 ppm), divider / half-rate (-5e5 ppm), stopped/OE-low (Dead).
//   Plus the 1-edge counting quantisation of the window, 1e6 / expected_edges
//   (10 ms -> 3052 ppm, 100 ms -> 305 ppm), added so short windows can't
//   false-trip.
// Only OUT_OF_RANGE and DEAD trigger the fallback: inside 31-50 kHz both IMUs
// still run synchronously from the same clock (ODR scales by f/32 kHz and
// ArduPilot's _update_sensor_rate() tracks it), so dropping to the unsynchronised
// internal clocks would make things worse.  OFF_NOMINAL is a warning.
ClockWatchdog::Report ClockWatchdog::update(uint32_t rising_edges, uint32_t window_us)
{
    Report r{};
    r.status = _status;
    r.raw = ClockStatus::Unknown;
    r.fallback_latched = _latched;
    if (window_us < _cfg.min_window_us || window_us == 0) {
        return r;   // window too short to judge: no state change
    }
    const double expected = double(_cfg.nominal_hz) * double(window_us) * 1e-6;
    r.freq_hz = double(rising_edges) * 1e6 / double(window_us);
    r.err_ppm = (r.freq_hz - double(_cfg.nominal_hz)) / double(_cfg.nominal_hz) * 1e6;
    r.allowed_ppm = double(_cfg.tol_ppm) + 1e6 / expected;

    if (r.freq_hz < double(_cfg.dead_below_hz)) {
        r.raw = ClockStatus::Dead;
    } else if (r.freq_hz < double(_cfg.min_hz) || r.freq_hz > double(_cfg.max_hz)) {
        r.raw = ClockStatus::OutOfRange;
    } else if (std::fabs(r.err_ppm) > r.allowed_ppm) {
        r.raw = ClockStatus::OffNominal;
    } else {
        r.raw = ClockStatus::Valid;
    }

    const bool bad = (r.raw == ClockStatus::Dead || r.raw == ClockStatus::OutOfRange);
    const bool status_bad = (_status == ClockStatus::Dead || _status == ClockStatus::OutOfRange);
    if (bad) {
        _good = 0;
        if (_bad < 255) _bad++;
        if (_bad >= _cfg.fail_windows) {
            _status = r.raw;
        }
    } else {
        _bad = 0;
        if (_good < 255) _good++;
        if (!status_bad || _good >= _cfg.recover_windows) {
            _status = r.raw;   // Unknown/Valid/OffNominal move freely
        }
    }
    const bool now_bad = (_status == ClockStatus::Dead || _status == ClockStatus::OutOfRange);
    if (now_bad && !_latched) {
        _latched = true;
        r.apply_fallback = true;
    }
    r.status = _status;
    r.fallback_latched = _latched;
    return r;
}

// ===========================================================================
// e) Consistency voter
// ===========================================================================
void rotate_pitch_180(const float in[3], float out[3])
{
    out[0] = -in[0];
    out[1] =  in[1];
    out[2] = -in[2];
}

void ConsistencyVoter::reset()
{
    const VoterConfig cfg = _cfg;
    *this = ConsistencyVoter(cfg);
}

VoteResult ConsistencyVoter::update(const PairedSample& p)
{
    VoteResult r{};
    float g2[3], a2[3];
    rotate_pitch_180(p.imu2.gyro, g2);
    rotate_pitch_180(p.imu2.accel, a2);
    for (int i = 0; i < 3; i++) {
        _sum[i]     += p.imu1.gyro[i] - g2[i];
        _sum[3 + i] += p.imu1.accel[i] - a2[i];
    }
    // per-IMU evidence: bit-identical gyro output (AFSR-style stuck) and clipping
    const ImuSample* s[2] = {&p.imu1, &p.imu2};
    for (int k = 0; k < 2; k++) {
        bool same = _have_last;
        bool sat = false;
        for (int i = 0; i < 3; i++) {
            if (s[k]->gyro[i] != _last_gyro[k][i]) same = false;
            _last_gyro[k][i] = s[k]->gyro[i];
            if (std::fabs(s[k]->gyro[i]) >= _cfg.gyro_sat_rad_s ||
                std::fabs(s[k]->accel[i]) >= _cfg.accel_sat_m_s2) sat = true;
        }
        _stuck[k] = same ? uint16_t(_stuck[k] + (_stuck[k] < 0xFFFF ? 1 : 0)) : 0;
        if (_stuck[k] + 1U >= _cfg.stuck_samples) _stuck_flag[k] = true;
        if (sat) _sat_flag[k] = true;
    }
    _have_last = true;

    if (++_k < (_cfg.average_n ? _cfg.average_n : 1)) {
        r.fault_mask = _fault;
        r.suspect = _suspect;
        return r;
    }
    const float inv = 1.0f / float(_k);
    r.evaluated = true;
    for (int i = 0; i < 6; i++) {
        r.diff[i] = _sum[i] * inv;
        const float thr = i < 3 ? _cfg.gyro_thresh_rad_s : _cfg.accel_thresh_m_s2;
        const uint8_t bit = uint8_t(1U << i);
        if (std::fabs(r.diff[i]) > thr) {
            r.exceed_mask = uint8_t(r.exceed_mask | bit);
            _ok[i] = 0;
            if (_exc[i] < 0xFFFF) _exc[i]++;
            if (_exc[i] >= _cfg.persist_blocks) _fault = uint8_t(_fault | bit);
        } else {
            _exc[i] = 0;
            if (_ok[i] < 0xFFFF) _ok[i]++;
            if (_ok[i] >= _cfg.clear_blocks) _fault = uint8_t(_fault & ~bit);
        }
        _sum[i] = 0.0f;
    }
    _k = 0;

    if (!_fault) {
        _suspect = Suspect::None;
    } else if (_suspect == Suspect::None || _suspect == Suspect::Unknown) {
        // two sensors cannot out-vote each other: only name one with evidence
        const bool e1 = _stuck_flag[0] || _sat_flag[0];
        const bool e2 = _stuck_flag[1] || _sat_flag[1];
        _suspect = (e1 && !e2) ? Suspect::Imu1 : (e2 && !e1) ? Suspect::Imu2 : Suspect::Unknown;
    }
    for (int k = 0; k < 2; k++) { _stuck_flag[k] = false; _sat_flag[k] = false; }
    r.fault_mask = _fault;
    r.suspect = _suspect;
    return r;
}

}  // namespace nano_imu_sync
