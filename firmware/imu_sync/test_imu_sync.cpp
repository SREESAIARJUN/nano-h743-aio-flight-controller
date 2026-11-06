// Nano H743 AIO - dual ICM-42688-P synchronisation regression tests (Revision 11.0)
// Build + run:  make -C firmware/imu_sync test
#include "imu_sync.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

using namespace nano_imu_sync;

static int g_fail = 0;
static int g_pass = 0;
#define CHECK(cond, ...) do { if (cond) { g_pass++; } else { g_fail++; \
    std::printf("  [FAIL] %s:%d ", __FILE__, __LINE__); std::printf(__VA_ARGS__); std::printf("\n"); } } while (0)

// deterministic LCG so results are reproducible across hosts
static uint32_t g_rng = 0x42688B11u;
static double urand()   // [0, 1)
{
    g_rng = g_rng * 1664525u + 1013904223u;
    return double(g_rng >> 8) / 16777216.0;
}
static double nrand()   // ~N(0,1) (Irwin-Hall, 12 uniforms)
{
    double s = 0;
    for (int i = 0; i < 12; i++) s += urand();
    return s - 6.0;
}

// ---------------------------------------------------------------------------
// ICM-42688-P register-file model (banks 0..4, BANK_SEL at 0x76 in every bank)
// ---------------------------------------------------------------------------
struct RegModel {
    uint8_t  bank = 0;
    uint8_t  r[5][256];
    uint32_t writes = 0, reads = 0;
    uint32_t delay_total_us = 0;
    uint8_t  log_bank[128], log_addr[128], log_val[128];
    uint32_t nlog = 0;
    RegModel()
    {
        std::memset(r, 0, sizeof(r));
        // DS-000347 reset values used by the plans
        r[0][reg::INTF_CONFIG1]  = 0x91;
        r[0][reg::GYRO_CONFIG0]  = 0x06;
        r[0][reg::ACCEL_CONFIG0] = 0x06;
        r[0][reg::TMST_CONFIG]   = 0x23;
        r[1][reg::GYRO_CONFIG_STATIC2] = 0xA3;   // arbitrary: proves only bits 1:0 are cleared
        r[1][reg::INTF_CONFIG5] = 0x00;
    }
};
static uint8_t m_read(void* c, uint8_t a)
{
    RegModel* m = static_cast<RegModel*>(c);
    m->reads++;
    return a == reg::BANK_SEL ? m->bank : m->r[m->bank][a];
}
static void m_write(void* c, uint8_t a, uint8_t v)
{
    RegModel* m = static_cast<RegModel*>(c);
    m->writes++;
    if (a == reg::BANK_SEL) {
        m->bank = uint8_t(v & 0x07);
    } else {
        m->r[m->bank][a] = v;
        if (m->nlog < 128) {
            m->log_bank[m->nlog] = m->bank; m->log_addr[m->nlog] = a; m->log_val[m->nlog] = v; m->nlog++;
        }
    }
}
static void m_delay(void* c, uint32_t us) { static_cast<RegModel*>(c)->delay_total_us += us; }
static RegisterBus bus_for(RegModel& m) { return RegisterBus{&m, m_read, m_write, m_delay}; }

// Literal transcription of upstream AP_InertialSensor_Invensensev3::set_filter_and_scaling()
// for inv3_type == ICM42688 (and, with force_rtc, the IIM42652 RTC branch applied to it).
struct ApRef {
    RegModel& m;
    void register_write(uint8_t a, uint8_t v) { m_write(&m, a, v); }
    uint8_t register_read(uint8_t a) { return m_read(&m, a); }
    void register_write_bank(uint8_t bank, uint8_t a, uint8_t v)
    { register_write(reg::BANK_SEL, bank); register_write(a, v); register_write(reg::BANK_SEL, 0); }
    uint8_t register_read_bank(uint8_t bank, uint8_t a)
    { register_write(reg::BANK_SEL, bank); uint8_t v = register_read(a); register_write(reg::BANK_SEL, 0); return v; }
    void run(uint32_t backend_rate_hz, bool force_rtc)
    {
        uint8_t odr_config = 0x06;
        uint8_t aaf_delt = 6, accel_aaf_delt = 5;
        uint16_t aaf_deltsqr = 36, accel_aaf_deltsqr = 25;
        uint8_t aaf_bitshift = 10, accel_aaf_bitshift = 10;
        switch (backend_rate_hz) {
        case 2000: odr_config = 0x05; aaf_delt = 12; aaf_deltsqr = 144; aaf_bitshift = 8; break;
        case 4000: odr_config = 0x04; aaf_delt = 21; aaf_deltsqr = 440; aaf_bitshift = 6; break;
        case 8000: odr_config = 0x03; aaf_delt = 21; aaf_deltsqr = 440; aaf_bitshift = 6; break;
        default: break;
        }
        register_write(0x4E, 0x00);
        register_write(0x4F, odr_config);
        register_write(0x50, odr_config);
        uint8_t aaf_enable = register_read_bank(1, 0x0B);
        register_write_bank(1, 0x0B, uint8_t(aaf_enable & ~0x03));
        register_write_bank(1, 0x0C, aaf_delt);
        register_write_bank(1, 0x0D, uint8_t(aaf_deltsqr & 0xFF));
        register_write_bank(1, 0x0E, uint8_t(((aaf_bitshift << 4) & 0xF0) | ((aaf_deltsqr >> 8) & 0x0F)));
        register_write_bank(2, 0x03, uint8_t(accel_aaf_delt << 1));
        register_write_bank(2, 0x04, uint8_t(accel_aaf_deltsqr & 0xFF));
        register_write_bank(2, 0x05, uint8_t(((accel_aaf_bitshift << 4) & 0xF0) | ((accel_aaf_deltsqr >> 8) & 0x0F)));
        const uint8_t v = register_read(0x4D);
        uint8_t intf_config1 = uint8_t((v & 0x3F) | 0x40);
        if (force_rtc) {
            intf_config1 |= 0x04;
            const uint8_t intf_config5 = register_read_bank(1, 0x7B);
            register_write_bank(1, 0x7B, uint8_t((intf_config5 & ~0x06) | 0x04));
        }
        register_write(0x4D, intf_config1);
        register_write(0x4E, 0x0F);
    }
};

static bool same_regs(const RegModel& a, const RegModel& b)
{
    return std::memcmp(a.r, b.r, sizeof(a.r)) == 0 && a.bank == b.bank;
}

// ---------------------------------------------------------------------------
static void test_tables()
{
    std::printf("T1 ODR codes / AAF table == ArduPilot ICM-42688 branch, CLKIN ODR scaling\n");
    const uint32_t rates[] = {1000, 2000, 4000, 8000};
    const uint8_t  odr[]   = {0x06, 0x05, 0x04, 0x03};
    const uint8_t  delt[]  = {6, 12, 21, 21};
    const uint16_t dsq[]   = {36, 144, 440, 440};
    const uint8_t  bsh[]   = {10, 8, 6, 6};
    const uint16_t bw[]    = {258, 536, 997, 997};
    for (int i = 0; i < 4; i++) {
        const AafCoeffs g = gyro_aaf_for_rate(rates[i]);
        CHECK(odr_code_for_rate(rates[i]) == odr[i], "ODR code %u Hz", unsigned(rates[i]));
        CHECK(g.delt == delt[i] && g.deltsqr == dsq[i] && g.bitshift == bsh[i] && g.bw_hz == bw[i],
              "gyro AAF %u Hz: %u/%u/%u", unsigned(rates[i]), g.delt, g.deltsqr, g.bitshift);
        // DS-000347 s5.3 AAF table rows (258 Hz: 6/36/10, 536 Hz: 12/144/8, 997 Hz: 21/440/6 - the
        // datasheet itself lists 440, not 21^2 = 441, and ArduPilot copies it)
        CHECK(std::abs(int(g.deltsqr) - int(g.delt) * int(g.delt)) <= 1, "DELTSQR ~ DELT^2 at %u Hz", unsigned(rates[i]));
    }
    const AafCoeffs a = accel_aaf();
    CHECK(a.delt == 5 && a.deltsqr == 25 && a.bitshift == 10 && a.bw_hz == 213, "accel AAF 5/25/10 (213 Hz)");
    CHECK(normalise_backend_rate(3000) == 1000 && odr_code_for_rate(16000) == 0x06,
          "unsupported rates fall back to 1 kHz config (upstream switch default)");
    CHECK(std::fabs(odr_hz_with_clkin(8000, 32768.0) - 8192.0) < 1e-9, "8 kHz x 32.768/32 = 8192 Hz");
    CHECK(std::fabs(odr_hz_with_clkin(1000, 31000.0) - 968.75) < 1e-9 &&
          std::fabs(odr_hz_with_clkin(1000, 50000.0) - 1562.5) < 1e-9, "CLKIN range end points scale ODR");
    // the +2.4 % shift is inside ArduPilot's +/-5 % post-convergence tracking window
    CHECK(8192.0 / 8000.0 - 1.0 < 0.05, "RTC ODR shift %.2f %% < 5 %%", (8192.0 / 8000.0 - 1.0) * 100.0);
    // Y2 worst case +/-30 ppm keeps ODR within 0.25 Hz of 8192
    CHECK(std::fabs(odr_hz_with_clkin(8000, 32768.0 * (1 + 30e-6)) - 8192.0) < 0.25, "30 ppm -> <0.25 Hz ODR error");
}

static void test_config_plan()
{
    std::printf("T2 register plan vs. upstream set_filter_and_scaling() on a register model\n");
    const uint32_t rates[] = {1000, 2000, 4000, 8000, 3000};
    for (uint32_t rate : rates) {
        for (int rtc = 0; rtc < 2; rtc++) {
            RegModel ref, mine;
            ApRef ap{ref};
            ap.run(rate, rtc != 0);
            RegPlan plan;
            build_config_plan(PlanConfig{rate, rtc ? ClockMode::RtcClkin : ClockMode::Internal}, plan);
            CHECK(!plan.overflow, "plan overflow");
            execute_plan(plan, bus_for(mine));
            CHECK(same_regs(ref, mine), "final register file identical to upstream (rate %u, rtc %d)", unsigned(rate), rtc);
            // identical order of non-BANK_SEL data writes
            bool order_ok = ref.nlog == mine.nlog;
            for (uint32_t i = 0; order_ok && i < ref.nlog; i++) {
                order_ok = ref.log_bank[i] == mine.log_bank[i] && ref.log_addr[i] == mine.log_addr[i] &&
                           ref.log_val[i] == mine.log_val[i];
            }
            CHECK(order_ok, "write order/value sequence matches upstream (rate %u, rtc %d, %u vs %u writes)",
                  unsigned(rate), rtc, unsigned(ref.nlog), unsigned(mine.nlog));
            CHECK(mine.bank == 0, "BANK_SEL left at 0");
            CHECK(mine.delay_total_us == 300, "300 us after PWR_MGMT0 = 0x0F");
        }
    }
    // explicit bit checks from reset values
    RegModel m;
    RegPlan plan;
    build_config_plan(PlanConfig{8000, ClockMode::RtcClkin}, plan);
    const uint32_t xfers = execute_plan(plan, bus_for(m));
    CHECK(plan.op[0].addr == reg::PWR_MGMT0 && plan.op[0].set_bits == 0 && plan.op[0].is_write(), "sensors off first");
    CHECK(m.r[0][reg::INTF_CONFIG1] == 0x55, "INTF_CONFIG1 0x91 -> 0x55 (AFSR fix 0x40 | RTC 0x04 | CLKSEL 01), got 0x%02X",
          m.r[0][reg::INTF_CONFIG1]);
    CHECK(m.r[1][reg::INTF_CONFIG5] == 0x04, "PIN9_FUNCTION = CLKIN (0b10)");
    CHECK(m.r[0][reg::GYRO_CONFIG0] == 0x03 && m.r[0][reg::ACCEL_CONFIG0] == 0x03, "8 kHz ODR code, 2000 dps / 16 g");
    CHECK(m.r[1][reg::GYRO_CONFIG_STATIC2] == 0xA0, "only AAF_DIS/NF_DIS cleared (0xA3 -> 0x%02X)", m.r[1][reg::GYRO_CONFIG_STATIC2]);
    CHECK(m.r[1][reg::GYRO_CONFIG_STATIC3] == 21 && m.r[1][reg::GYRO_CONFIG_STATIC4] == (440 & 0xFF) &&
          m.r[1][reg::GYRO_CONFIG_STATIC5] == 0x61, "gyro AAF 21/440/6 -> 0x15 0xB8 0x61");
    CHECK(m.r[2][reg::ACCEL_CONFIG_STATIC2] == 0x0A && m.r[2][reg::ACCEL_CONFIG_STATIC3] == 25 &&
          m.r[2][reg::ACCEL_CONFIG_STATIC4] == 0xA0, "accel AAF 5/25/10 -> 0x0A 0x19 0xA0");
    CHECK(m.r[0][reg::PWR_MGMT0] == 0x0F, "gyro+accel low-noise on");
    // CLKIN pin function must be set before RTC_MODE (upstream order)
    int i5 = -1, i1 = -1;
    for (int i = 0; i < plan.count; i++) {
        if (plan.op[i].bank == 1 && plan.op[i].addr == reg::INTF_CONFIG5) i5 = i;
        if (plan.op[i].bank == 0 && plan.op[i].addr == reg::INTF_CONFIG1) i1 = i;
    }
    CHECK(i5 >= 0 && i1 > i5, "PIN9=CLKIN (op %d) precedes RTC_MODE (op %d)", i5, i1);
    std::printf("    8 kHz RTC plan: %u ops, %u SPI transfers\n", unsigned(plan.count), unsigned(xfers));
    // reserved bits of INTF_CONFIG5 and INTF_CONFIG1[3] preserved
    RegModel m2;
    m2.r[1][reg::INTF_CONFIG5] = 0xF9 | 0x02;   // reserved bits set + FSYNC
    m2.r[0][reg::INTF_CONFIG1] = 0x98;          // ACCEL_LP_CLK_SEL=1, CLKSEL=00
    execute_plan(plan, bus_for(m2));
    CHECK(m2.r[1][reg::INTF_CONFIG5] == 0xFD, "INTF_CONFIG5 reserved bits kept, 0xFB -> 0x%02X", m2.r[1][reg::INTF_CONFIG5]);
    CHECK(m2.r[0][reg::INTF_CONFIG1] == 0x5C, "INTF_CONFIG1 0x98 -> 0x5C, got 0x%02X", m2.r[0][reg::INTF_CONFIG1]);
    // plan capacity guard
    RegPlan full;
    for (size_t k = 0; k < RegPlan::MAX_OPS; k++) full.write(0, 0x10, 0);
    CHECK(!full.write(0, 0x11, 0) && full.overflow && full.count == RegPlan::MAX_OPS, "overflow reported, no write past end");
}

static void test_fallback_plan()
{
    std::printf("T3 loss-of-clock fallback plan\n");
    RegModel m;
    RegPlan cfg;
    build_config_plan(PlanConfig{8000, ClockMode::RtcClkin}, cfg);
    execute_plan(cfg, bus_for(m));
    m.r[0][reg::INT_CONFIG] = 0x13;   // INT1 push-pull active-high, INT2 push-pull (worst case)
    RegModel before = m;
    RegPlan fb;
    build_fallback_plan(fb);
    execute_plan(fb, bus_for(m));
    CHECK((m.r[0][reg::INTF_CONFIG1] & INTF_CONFIG1_RTC_MODE) == 0, "RTC_MODE cleared");
    CHECK(m.r[0][reg::INTF_CONFIG1] == 0x51, "AFSR fix + CLKSEL kept (0x%02X)", m.r[0][reg::INTF_CONFIG1]);
    CHECK((m.r[1][reg::INTF_CONFIG5] & PIN9_FUNCTION_MASK) == PIN9_FUNCTION_INT2, "pin 9 back to INT2");
    CHECK(m.r[0][reg::INT_CONFIG] == 0x03, "INT2 forced open-drain, INT1 config untouched (0x%02X)", m.r[0][reg::INT_CONFIG]);
    CHECK(m.r[0][reg::PWR_MGMT0] == 0x0F && fb.op[0].addr == reg::PWR_MGMT0 && fb.op[0].set_bits == 0,
          "sensors off during the clock switch, back on at the end");
    // RTC clear must happen before pin 9 stops being CLKIN; open-drain before INT2
    int i_rtc = -1, i_od = -1, i_pin = -1;
    for (int i = 0; i < fb.count; i++) {
        if (fb.op[i].addr == reg::INTF_CONFIG1 && fb.op[i].bank == 0) i_rtc = i;
        if (fb.op[i].addr == reg::INT_CONFIG && fb.op[i].bank == 0) i_od = i;
        if (fb.op[i].addr == reg::INTF_CONFIG5 && fb.op[i].bank == 1) i_pin = i;
    }
    CHECK(i_rtc >= 0 && i_od > i_rtc && i_pin > i_od, "order: RTC off (%d) -> INT2 open-drain (%d) -> PIN9=INT2 (%d)",
          i_rtc, i_od, i_pin);
    // nothing else changed: AAF / ODR registers untouched
    bool others = true;
    for (int b = 0; b < 5; b++)
        for (int a = 0; a < 256; a++) {
            if ((b == 0 && (a == reg::INTF_CONFIG1 || a == reg::INT_CONFIG || a == reg::PWR_MGMT0)) ||
                (b == 1 && a == reg::INTF_CONFIG5)) continue;
            if (before.r[b][a] != m.r[b][a]) others = false;
        }
    CHECK(others, "fallback touches only INTF_CONFIG1, INT_CONFIG, INTF_CONFIG5, PWR_MGMT0");
    // fallback result == internal-clock config from the same state (except INT_CONFIG / INTF_CONFIG5 already default)
    RegModel internal;
    RegPlan ic;
    build_config_plan(PlanConfig{8000, ClockMode::Internal}, ic);
    execute_plan(ic, bus_for(internal));
    CHECK(internal.r[0][reg::INTF_CONFIG1] == m.r[0][reg::INTF_CONFIG1], "fallback INTF_CONFIG1 == non-RTC upstream value");
}

static void test_timestamp()
{
    std::printf("T4 FIFO timestamp scaling (DS-000347 s12.7) and 16-bit unwrap\n");
    // s12.7 worked examples: 915.5 counts (RTC) and 937.5 counts (internal) == 1000 us
    // (915.5 x 32.768/30 = 999.97 us: the datasheet example is itself rounded to 1000 us)
    CHECK(std::llabs(int64_t(tmst_ticks_to_ns(1831, ClockMode::RtcClkin, TmstRes::Res1us)) - 2000000) < 100,
          "RTC: 1831 counts = 2 x 1000 us (%llu ns)", (unsigned long long)tmst_ticks_to_ns(1831, ClockMode::RtcClkin, TmstRes::Res1us));
    CHECK(tmst_ticks_to_ns(1875, ClockMode::Internal, TmstRes::Res1us) == 2000000, "internal: 1875 counts = 2 x 1000 us");
    CHECK(tmst_ticks_to_ns(30, ClockMode::RtcClkin, TmstRes::Res16us) == 524288, "TMST_RES=1 x16");
    const uint32_t e_rtc = tmst_ticks_per_sample_x256(8192.0, ClockMode::RtcClkin, TmstRes::Res1us);
    const uint32_t e_int = tmst_ticks_per_sample_x256(8000.0, ClockMode::Internal, TmstRes::Res1us);
    CHECK(std::fabs(e_rtc / 256.0 - 111.758) < 0.01, "RTC 8192 Hz: %.3f counts/sample", e_rtc / 256.0);
    CHECK(std::fabs(e_int / 256.0 - 117.188) < 0.01, "internal 8000 Hz: %.3f counts/sample", e_int / 256.0);

    // 200 000 samples (~24 s, ~341 wraps) with fractional period and an arbitrary start value
    TimestampUnwrapper uw(e_rtc);
    const double per = 1e9 / 8192.0 / (32768.0 / 30.0);   // counts per sample
    const uint64_t start = 65000;
    bool mono = true, exact = true, clean = true;
    uint64_t prev = 0;
    for (uint32_t k = 0; k < 200000; k++) {
        const uint64_t truth = start + uint64_t(std::floor(double(k) * per));
        const TimestampUnwrapper::Result r = uw.update(uint16_t(truth & 0xFFFF));
        if (k && r.unwrapped <= prev) mono = false;
        if (r.unwrapped != truth) exact = false;
        if (k && (r.missed || r.duplicate || r.irregular)) clean = false;
        prev = r.unwrapped;
    }
    CHECK(mono && exact, "unwrapped == true 64-bit count across wraps");
    CHECK(clean, "no false gap / duplicate flags on a clean 8192 Hz stream");
    CHECK(uw.wraps() == uint32_t((start + uint64_t(199999 * per)) >> 16), "wrap count %u", unsigned(uw.wraps()));
    const uint64_t ns = tmst_ticks_to_ns(uw.value() - start, ClockMode::RtcClkin, TmstRes::Res1us);
    CHECK(std::fabs(double(ns) - 199999.0 * 1e9 / 8192.0) < 2000.0, "converted span %.6f s", double(ns) * 1e-9);

    // fault injection: wrap 0xFFC0 -> 0x0030, a 3-sample gap, a duplicate, garbage
    TimestampUnwrapper u2(e_rtc);
    u2.update(0xFFC0);
    TimestampUnwrapper::Result r = u2.update(uint16_t(0xFFC0 + 112));
    CHECK(r.unwrapped == 0xFFC0u + 112u && r.delta == 112 && !r.irregular && u2.wraps() == 1, "wrap past 0xFFFF");
    r = u2.update(uint16_t(0xFFC0 + 112 + 4 * 112));
    CHECK(r.missed == 3 && !r.irregular, "3 missed samples detected (missed=%u)", r.missed);
    r = u2.update(uint16_t(0xFFC0 + 112 + 4 * 112));
    CHECK(r.duplicate, "duplicate (stalled) timestamp flagged");
    r = u2.update(uint16_t(0xFFC0 + 112 + 4 * 112 + 50));
    CHECK(r.irregular, "half-period delta flagged irregular");
}

// ---------------------------------------------------------------------------
// stream generation on the MCU time base (DRDY EXTI timestamps, ns)
// ---------------------------------------------------------------------------
struct Stream {
    double   t0_ns;
    double   period_ns;
    double   jitter_ns;   // uniform +/-
    uint64_t k = 0;
    uint64_t next_t() const { return uint64_t(t0_ns + double(k) * period_ns); }
    ImuSample make(const float gyro[3], const float accel[3])
    {
        ImuSample s;
        s.t_ns = uint64_t(t0_ns + double(k) * period_ns + (urand() * 2.0 - 1.0) * jitter_ns);
        for (int i = 0; i < 3; i++) { s.gyro[i] = gyro[i]; s.accel[i] = accel[i]; }
        k++;
        return s;
    }
};

struct PairRun {
    uint64_t pairs = 0;
    double   max_abs_off_err = 0;
};

// run two streams through the pairer in time order until t_end
static PairRun run_pair(DualImuPairer& p, Stream& a, Stream& b, double t_end_ns, double expect_off_ns = NAN)
{
    PairRun res;
    const float z[3] = {0, 0, 0};
    PairedSample ps;
    while (true) {
        const uint64_t ta = a.next_t(), tb = b.next_t();
        if (double(ta) > t_end_ns && double(tb) > t_end_ns) break;
        if (ta <= tb) p.push(0, a.make(z, z)); else p.push(1, b.make(z, z));
        while (p.pop_pair(ps)) {
            res.pairs++;
            if (!std::isnan(expect_off_ns)) {
                const double e = std::fabs(double(ps.offset_ns) - expect_off_ns);
                if (e > res.max_abs_off_err) res.max_abs_off_err = e;
            }
        }
    }
    return res;
}

static void test_pairing_shared()
{
    std::printf("T5 pairing on a shared CLKIN: constant phase, zero drift\n");
    const double T = 1e9 / 8192.0;
    const double offs[] = {37000.0, -45000.0, 59000.0, 0.0};
    for (double off : offs) {
        DualImuPairer p;
        // both IMUs on Y2 running at +27 ppm: the common error must not show as drift
        Stream a{1e6, T / (1 + 27e-6), 500.0};
        Stream b{1e6 + off, T / (1 + 27e-6), 500.0};
        const PairRun r = run_pair(p, a, b, 3.2e9, off);
        CHECK(p.windows() >= 3, "3 regression windows (%u)", unsigned(p.windows()));
        CHECK(p.state() == SyncState::Shared, "offset %.0f ns: state Shared", off);
        CHECK(std::fabs(p.drift_ppm()) < 0.5, "offset %.0f ns: drift %.4f ppm ~ 0", off, p.drift_ppm());
        CHECK(std::fabs(p.phase_offset_ns() - off) < 100.0, "phase estimate %.1f ns vs %.0f", p.phase_offset_ns(), off);
        CHECK(r.max_abs_off_err <= 1001.0, "every pair within jitter of the phase (max err %.0f ns)", r.max_abs_off_err);
        CHECK(p.orphans(0) == 0 && p.orphans(1) <= 1 && p.overflow(0) == 0 && p.overflow(1) == 0,
              "no orphans/overflow (%llu, %llu)", (unsigned long long)p.orphans(0), (unsigned long long)p.orphans(1));
        CHECK(r.pairs >= 26200, "pairs %llu for 3.2 s", (unsigned long long)r.pairs);
    }
}

static void test_pairing_drift()
{
    std::printf("T6 drift detection: clock not shared / RTC inactive\n");
    const double T = 1e9 / 8192.0;
    const double ppms[] = {200.0, -60.0, 20.0, -15.0};
    for (double ppm : ppms) {
        DualImuPairer p;
        Stream a{0.0, T, 500.0};
        // +ppm: IMU2 period longer (IMU2 clock slow) -> offset grows
        Stream b{20000.0, T * (1 + ppm * 1e-6), 500.0};
        const PairRun r = run_pair(p, a, b, 4.1e9);
        CHECK(p.state() == SyncState::NotShared, "%+.0f ppm -> NotShared", ppm);
        CHECK(std::fabs(p.drift_ppm() - ppm) < 0.05 * std::fabs(ppm) + 0.3, "measured %.3f ppm vs injected %+.0f",
              p.drift_ppm(), ppm);
        CHECK(r.pairs > 33000, "pairing continues through the +/-T/2 slip (%llu pairs)", (unsigned long long)r.pairs);
    }
    // internal RC oscillators after fallback: +0.4 % vs -0.3 % at nominal 8 kHz
    PairerConfig c;
    c.odr_period_ns = 125000;
    c.window_pairs = 4000;
    DualImuPairer p(c);
    Stream a{0.0, 125000.0 / 1.004, 500.0};
    Stream b{0.0, 125000.0 / 0.997, 500.0};
    run_pair(p, a, b, 2.2e9);
    const double expect = (1.004 / 0.997 - 1.0) * 1e6;   // relative period error of IMU2
    CHECK(p.state() == SyncState::NotShared && std::fabs(p.drift_ppm() - expect) < 0.01 * expect,
          "RC clocks: %.0f ppm (expect %.0f)", p.drift_ppm(), expect);
    CHECK(p.orphans(0) > 0, "faster IMU1 leaves orphans (%llu)", (unsigned long long)p.orphans(0));
}

static void test_pairing_from_fifo_timestamps()
{
    std::printf("T7 FIFO timestamps -> unwrap -> ns -> pairing (shared CLKIN, different epochs)\n");
    const double cnt_per = 1e9 / 8192.0 / (32768.0 / 30.0);
    TimestampUnwrapper u1(tmst_ticks_per_sample_x256(8192.0, ClockMode::RtcClkin, TmstRes::Res1us));
    TimestampUnwrapper u2(tmst_ticks_per_sample_x256(8192.0, ClockMode::RtcClkin, TmstRes::Res1us));
    // IMU counters started 41.3 ms apart; IMU2 sample instants lag IMU1 by 23 us.
    // Epoch alignment (MCU anchor from the first DRDY EXTI pair) maps both into MCU ns.
    const uint64_t c1_0 = 60000, c2_0 = 12345;
    const double anchor1_ns = 5.0e6, anchor2_ns = 5.0e6 + 23000.0;
    DualImuPairer p;
    const float z[3] = {0, 0, 0};
    PairedSample ps;
    uint64_t pairs = 0;
    for (uint32_t k = 0; k < 3 * 8192 + 100; k++) {
        const uint64_t c1 = c1_0 + uint64_t(std::floor(double(k) * cnt_per));
        const uint64_t c2 = c2_0 + uint64_t(std::floor(double(k) * cnt_per + 0.37));
        const uint64_t w1 = u1.update(uint16_t(c1 & 0xFFFF)).unwrapped - c1_0;
        const uint64_t w2 = u2.update(uint16_t(c2 & 0xFFFF)).unwrapped - c2_0;
        ImuSample s1{}, s2{};
        s1.t_ns = uint64_t(anchor1_ns) + tmst_ticks_to_ns(w1, ClockMode::RtcClkin, TmstRes::Res1us);
        s2.t_ns = uint64_t(anchor2_ns) + tmst_ticks_to_ns(w2, ClockMode::RtcClkin, TmstRes::Res1us);
        (void)z;
        p.push(0, s1);
        p.push(1, s2);
        while (p.pop_pair(ps)) pairs++;
    }
    CHECK(u1.wraps() > 20 && u2.wraps() > 20, "both counters wrapped (%u, %u)", unsigned(u1.wraps()), unsigned(u2.wraps()));
    CHECK(p.state() == SyncState::Shared && std::fabs(p.drift_ppm()) < 1.0, "Shared, drift %.3f ppm", p.drift_ppm());
    CHECK(std::fabs(p.phase_offset_ns() - 23000.0) < 1500.0, "phase %.0f ns ~ 23 us", p.phase_offset_ns());
    CHECK(pairs >= 3 * 8192, "pairs %llu", (unsigned long long)pairs);
}

static void test_watchdog()
{
    std::printf("T8 CLKIN watchdog (PD12 edge count)\n");
    ClockWatchdog wd;
    const uint32_t W = 10000;   // 10 ms windows: 327.68 edges nominal
    ClockWatchdog::Report r = wd.update(328, W);
    CHECK(r.raw == ClockStatus::Valid && r.status == ClockStatus::Valid && !r.apply_fallback, "nominal -> Valid");
    CHECK(std::fabs(r.allowed_ppm - (500.0 + 1e6 / 327.68)) < 1e-6, "allowed %.0f ppm = 500 + 1-edge quantisation", r.allowed_ppm);
    // +/-30 ppm oscillator never trips even with +/-1 edge quantisation
    int false_trips = 0;
    for (int i = 0; i < 1000; i++) {
        const double f = 32768.0 * (1 + (urand() * 60.0 - 30.0) * 1e-6) * (1 + (urand() * 100.0 - 50.0) * 1e-6);
        const uint32_t edges = uint32_t(f * W * 1e-6 + urand());
        if (wd.update(edges, W).raw != ClockStatus::Valid) false_trips++;
    }
    CHECK(false_trips == 0, "1000 windows of in-spec clock: %d false trips", false_trips);
    // short window ignored
    r = wd.update(0, 500);
    CHECK(r.raw == ClockStatus::Unknown && r.status == ClockStatus::Valid, "sub-1 ms window ignored");
    // single dead window: debounced, no fallback
    r = wd.update(0, W);
    CHECK(r.raw == ClockStatus::Dead && r.status == ClockStatus::Valid && !r.apply_fallback, "1 dead window debounced");
    r = wd.update(328, W);
    CHECK(r.status == ClockStatus::Valid, "recovered glitch");
    // 32.000 kHz part: in range, off nominal -> warn, no fallback
    r = wd.update(320, W);
    CHECK(r.raw == ClockStatus::OffNominal && r.status == ClockStatus::OffNominal && !r.apply_fallback,
          "32.000 kHz: OffNominal (%.0f ppm), no fallback", r.err_ppm);
    wd.update(328, W);
    // half rate (divider / missed edges): out of range
    r = wd.update(164, W);
    CHECK(r.raw == ClockStatus::OutOfRange && !r.apply_fallback, "16.4 kHz window 1: not yet");
    r = wd.update(164, W);
    CHECK(r.status == ClockStatus::OutOfRange && r.apply_fallback && r.fallback_latched, "16.4 kHz x2 -> fallback");
    r = wd.update(164, W);
    CHECK(!r.apply_fallback && r.fallback_latched, "fallback asserted exactly once");
    // ringing double-count: 65.5 kHz is out of range too
    ClockWatchdog wd2;
    wd2.update(655, W);
    r = wd2.update(655, W);
    CHECK(r.status == ClockStatus::OutOfRange && r.apply_fallback, "double-counting (65.5 kHz) -> fallback");
    // recovery hysteresis on status, latch persists
    for (int i = 0; i < 9; i++) r = wd2.update(328, W);
    CHECK(r.status == ClockStatus::OutOfRange, "status held for 9 good windows");
    r = wd2.update(328, W);
    CHECK(r.status == ClockStatus::Valid && r.fallback_latched && !r.apply_fallback, "Valid after 10, fallback stays latched");
    wd2.clear_latch_on_ground();
    CHECK(!wd2.fallback_latched(), "ground clear");
    // boundary: 31 kHz / 50 kHz inclusive
    ClockWatchdog wd3;
    CHECK(wd3.update(310, W).raw == ClockStatus::OffNominal, "31.0 kHz in range");
    CHECK(wd3.update(309, W).raw == ClockStatus::OutOfRange, "30.9 kHz out of range");
    CHECK(wd3.update(500, W).raw == ClockStatus::OffNominal && wd3.update(501, W).raw == ClockStatus::OutOfRange,
          "50.0 kHz edge");
    CHECK(wd3.update(5, W).raw == ClockStatus::Dead, "500 Hz of noise edges = Dead");
}

// clock death mid-stream: both IMUs stall (common mode), watchdog fires,
// fallback plan executed on both register models, IMUs resume on RC clocks
static void test_clock_death_integration()
{
    std::printf("T9 clock death mid-stream -> watchdog -> fallback on both IMUs\n");
    RegModel imu[2];
    RegPlan cfg;
    build_config_plan(PlanConfig{8000, ClockMode::RtcClkin}, cfg);
    for (auto& m : imu) execute_plan(cfg, bus_for(m));
    auto rtc_active = [&](const RegModel& m) {
        return (m.r[0][reg::INTF_CONFIG1] & INTF_CONFIG1_RTC_MODE) &&
               (m.r[1][reg::INTF_CONFIG5] & PIN9_FUNCTION_MASK) == PIN9_FUNCTION_CLKIN;
    };
    CHECK(rtc_active(imu[0]) && rtc_active(imu[1]), "both IMUs in RTC mode");

    ClockWatchdog wd;
    DualImuPairer p;
    const double T = 1e9 / 8192.0;
    Stream a{0.0, T, 300.0}, b{31000.0, T, 300.0};
    const double t_death = 1.237e9;
    int fallback_at_ms = -1;
    const float z[3] = {0, 0, 0};
    PairedSample ps;
    for (int win = 0; win < 400; win++) {   // 10 ms windows, 4 s
        const double w0 = win * 1e7, w1 = w0 + 1e7;
        const bool clk_alive = (w1 <= t_death);
        double alive_ns = clk_alive ? 1e7 : (w0 < t_death ? t_death - w0 : 0.0);
        const uint32_t edges = uint32_t(alive_ns * 1e-9 * 32768.0 * (1 + 12e-6));
        bool rtc = rtc_active(imu[0]);
        // IMU sample production in this window
        const double end = rtc ? std::fmin(w1, t_death) : w1;
        while (double(a.next_t()) < end || double(b.next_t()) < end) {
            if (a.next_t() <= b.next_t()) p.push(0, a.make(z, z)); else p.push(1, b.make(z, z));
            while (p.pop_pair(ps)) {}
        }
        if (rtc && w1 > t_death) {   // stalled IMUs: nothing more until fallback
            a.t0_ns = w1; a.k = 0; b.t0_ns = w1; b.k = 0;
        }
        const ClockWatchdog::Report r = wd.update(edges, 10000);
        if (r.apply_fallback) {
            fallback_at_ms = int(w1 / 1e6);
            RegPlan fb;
            build_fallback_plan(fb);
            for (auto& m : imu) execute_plan(fb, bus_for(m));
            // internal clocks: IMU1 +0.4 %, IMU2 -0.3 % (PLL mode +/-1 % spec), restart after 300 us
            a = Stream{w1 + 300e3, 125000.0 / 1.004, 300.0};
            b = Stream{w1 + 300e3 + 9000.0, 125000.0 / 0.997, 300.0};
            PairerConfig c; c.odr_period_ns = 125000; c.window_pairs = 4000;
            p = DualImuPairer(c);
        }
    }
    CHECK(fallback_at_ms > 0, "fallback asserted");
    const double latency_ms = fallback_at_ms - t_death / 1e6;
    CHECK(latency_ms > 0 && latency_ms <= 30.0, "detection latency %.1f ms (<= 3 windows)", latency_ms);
    CHECK(!rtc_active(imu[0]) && !rtc_active(imu[1]), "both IMUs back on internal clock");
    CHECK(wd.status() == ClockStatus::Dead, "watchdog reports Dead");
    CHECK(p.windows() >= 3 && p.state() == SyncState::NotShared,
          "after fallback the pairer correctly reports clocks not shared (%.0f ppm)", p.drift_ppm());
}

static void test_voter()
{
    std::printf("T10 cross-IMU consistency voter (IMU2 ROTATION_PITCH_180)\n");
    const float v[3] = {1.0f, 2.0f, 3.0f};
    float o[3];
    rotate_pitch_180(v, o);
    CHECK(o[0] == -1.0f && o[1] == 2.0f && o[2] == -3.0f, "pitch 180: x->-x, z->-z");

    // board motion: rotating + accelerating; IMU2 reports it in its own (pitched 180) frame
    auto make_pair = [](double t, float gb[3], float ab[3], PairedSample& ps, float noise_g, float noise_a) {
        const float gx = float(0.8 * std::sin(6.0 * t)), gy = float(-1.5 * std::cos(4.0 * t)), gz = float(2.2 * std::sin(2.0 * t));
        const float ax = float(3.0 * std::sin(5.0 * t)), ay = float(-1.0), az = float(-9.80665 + 2.0 * std::cos(3.0 * t));
        const float g[3] = {gx, gy, gz}, a[3] = {ax, ay, az};
        for (int i = 0; i < 3; i++) {
            ps.imu1.gyro[i] = g[i] + float(nrand()) * noise_g;
            ps.imu1.accel[i] = a[i] + float(nrand()) * noise_a;
        }
        float g2[3], a2[3];
        for (int i = 0; i < 3; i++) { g2[i] = g[i] + gb[i] + float(nrand()) * noise_g; a2[i] = a[i] + ab[i] + float(nrand()) * noise_a; }
        // board -> IMU2 sensor frame (pitch 180 is its own inverse)
        rotate_pitch_180(g2, ps.imu2.gyro);
        rotate_pitch_180(a2, ps.imu2.accel);
        ps.imu1.t_ns = uint64_t(t * 1e9);
        ps.imu2.t_ns = ps.imu1.t_ns + 20000;
        ps.offset_ns = 20000;
    };
    // vibration-level noise per sample: 0.05 rad/s, 1.5 m/s^2 (averaged over 8 before voting)
    const float ng = 0.05f, na = 1.5f;
    float zb[3] = {0, 0, 0};
    {
        ConsistencyVoter vt;
        uint8_t any = 0;
        PairedSample ps;
        for (int k = 0; k < 8192 * 2; k++) {
            make_pair(k / 8192.0, zb, zb, ps, ng, na);
            const VoteResult r = vt.update(ps);
            any = uint8_t(any | r.fault_mask);
        }
        CHECK(any == 0, "healthy rotated pair: no faults over 2 s (mask 0x%02X)", any);
    }
    {
        // forgetting the rotation must be caught (gyro x/z, accel z ~ 2 g)
        ConsistencyVoter vt;
        PairedSample ps;
        VoteResult r{};
        for (int k = 0; k < 8192; k++) {
            make_pair(k / 8192.0, zb, zb, ps, ng, na);
            for (int i = 0; i < 3; i++) { ps.imu2.gyro[i] = ps.imu1.gyro[i]; ps.imu2.accel[i] = ps.imu1.accel[i]; }
            r = vt.update(ps);
        }
        CHECK((r.fault_mask & (VOTE_ACCEL_Z)) && !(r.fault_mask & VOTE_GYRO_Y) && !(r.fault_mask & VOTE_ACCEL_Y),
              "unrotated IMU2 flags the flipped axes only (mask 0x%02X)", r.fault_mask);
    }
    {
        // gyro Y bias on IMU2: 0.2 rad/s
        ConsistencyVoter vt;
        PairedSample ps;
        float gb[3] = {0, 0.2f, 0};
        VoteResult r{};
        int first_fault_block = -1, blocks = 0;
        for (int k = 0; k < 2048; k++) {
            make_pair(k / 8192.0, gb, zb, ps, ng, na);
            r = vt.update(ps);
            if (r.evaluated) blocks++;
            if (r.fault_mask && first_fault_block < 0) first_fault_block = blocks;
        }
        CHECK(r.fault_mask == VOTE_GYRO_Y, "gyro-Y bias isolated to bit gyro Y (0x%02X)", r.fault_mask);
        CHECK(first_fault_block == 8, "persistence: fault on block %d (8 blocks = 64 samples = 7.8 ms)", first_fault_block);
        CHECK(r.suspect == Suspect::Unknown, "two-way bias disagreement: suspect Unknown (no evidence)");
        CHECK(r.diff[1] < -0.15f && r.diff[1] > -0.25f, "diff gyro y %.3f rad/s", double(r.diff[1]));
        // clears after clear_blocks
        int clear_blocks = 0;
        for (int k = 0; k < 8 * 40 && vt.fault_mask(); k++) {
            make_pair(k / 8192.0, zb, zb, ps, ng, na);
            if (vt.update(ps).evaluated) clear_blocks++;
        }
        CHECK(vt.fault_mask() == 0 && clear_blocks == 32, "fault clears after %d good blocks", clear_blocks);
    }
    {
        // transient spike (5 blocks) is below persistence
        ConsistencyVoter vt;
        PairedSample ps;
        float ab[3] = {4.0f, 0, 0};
        uint8_t any = 0;
        for (int k = 0; k < 4096; k++) {
            make_pair(k / 8192.0, zb, (k >= 800 && k < 840) ? ab : zb, ps, ng, na);
            any = uint8_t(any | vt.update(ps).fault_mask);
        }
        CHECK(any == 0, "40-sample accel transient not latched");
    }
    {
        // IMU1 gyro stuck (AFSR-style constant output) during motion
        ConsistencyVoter vt;
        PairedSample ps;
        VoteResult r{};
        float stuck[3] = {0, 0, 0};
        for (int k = 0; k < 4096; k++) {
            make_pair(k / 8192.0, zb, zb, ps, ng, na);
            if (k == 1000) for (int i = 0; i < 3; i++) stuck[i] = ps.imu1.gyro[i];
            if (k >= 1000) for (int i = 0; i < 3; i++) ps.imu1.gyro[i] = stuck[i];
            r = vt.update(ps);
        }
        CHECK(r.fault_mask & (VOTE_GYRO_X | VOTE_GYRO_Y | VOTE_GYRO_Z), "stuck gyro disagrees (0x%02X)", r.fault_mask);
        CHECK(r.suspect == Suspect::Imu1, "stuck IMU1 named as suspect (%d)", int(r.suspect));
    }
    {
        // IMU2 accel clipping at 16 g on a hard impact; IMU1 (different mounting) reads 18 g
        ConsistencyVoter vt;
        PairedSample ps;
        VoteResult r{};
        for (int k = 0; k < 1024; k++) {
            make_pair(k / 8192.0, zb, zb, ps, ng, na);
            ps.imu1.accel[0] = 176.5f;
            ps.imu2.accel[0] = -156.9f;   // sensor frame (x flipped) -> board +156.9 clipped
            r = vt.update(ps);
        }
        CHECK(r.fault_mask & VOTE_ACCEL_X, "clipping disagreement on accel X");
        CHECK(r.suspect == Suspect::Unknown || r.suspect == Suspect::Imu2, "suspect %d", int(r.suspect));
        // IMU1 was also above saturation threshold in this synthetic case -> both have evidence
        CHECK(r.suspect == Suspect::Unknown, "both saturated -> Unknown");
        ConsistencyVoter vt2;
        for (int k = 0; k < 1024; k++) {
            make_pair(k / 8192.0, zb, zb, ps, ng, na);
            ps.imu1.accel[0] = 150.0f;    // just under clip on IMU1
            ps.imu2.accel[0] = -156.9f;   // clipped on IMU2
            ps.imu1.accel[0] += 0.0f;
            r = vt2.update(ps);
        }
        CHECK((r.fault_mask & VOTE_ACCEL_X) && r.suspect == Suspect::Imu2, "only IMU2 clipped -> suspect IMU2 (mask 0x%02X, s %d)",
              r.fault_mask, int(r.suspect));
    }
}

int main()
{
    std::printf("Nano H743 AIO dual ICM-42688-P synchronisation tests (Rev 11.0)\n");
    test_tables();
    test_config_plan();
    test_fallback_plan();
    test_timestamp();
    test_pairing_shared();
    test_pairing_drift();
    test_pairing_from_fifo_timestamps();
    test_watchdog();
    test_clock_death_integration();
    test_voter();
    std::printf("\nRESULT: %s (%d checks, %d failures)\n", g_fail ? "FAIL" : "PASS", g_pass + g_fail, g_fail);
    return g_fail ? 1 : 0;
}
