// Nano H743 AIO - bidirectional DShot telemetry regression tests (Revision 10.0)
// Build + run:  make -C firmware/bdshot_telem test
#include "bdshot_telem.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

using namespace nano_bdshot;

static int g_fail = 0;
static int g_pass = 0;
#define CHECK(cond, ...) do { if (cond) { g_pass++; } else { g_fail++; \
    std::printf("  [FAIL] %s:%d ", __FILE__, __LINE__); std::printf(__VA_ARGS__); std::printf("\n"); } } while (0)

// deterministic LCG so results are reproducible across hosts
static uint32_t g_rng = 0x743A10C0u;
static int32_t rnd_pm(int32_t span)   // uniform in [-span, +span]
{
    g_rng = g_rng * 1664525u + 1013904223u;
    if (span == 0) return 0;
    return int32_t((g_rng >> 8) % uint32_t(2 * span + 1)) - span;
}

static uint32_t roundtrip(uint16_t payload, uint32_t tpb_x256, int32_t jitter, uint16_t t0 = 1000)
{
    uint16_t edges[GCR_TELEMETRY_BIT_LEN];
    const uint32_t n = encode_gcr_edges(payload, t0, tpb_x256, edges, GCR_TELEMETRY_BIT_LEN);
    for (uint32_t i = 1; i < n; i++) {   // timestamp noise on every edge but the reference
        edges[i] = uint16_t(int32_t(edges[i]) + rnd_pm(jitter));
    }
    return decode_gcr_edges(edges, n);
}

static void test_constants()
{
    std::printf("T1 board timing constants\n");
    CHECK(GCR_BITRATE == 750000u, "GCR bit rate %u", unsigned(GCR_BITRATE));
    CHECK(TELEM_PSC == 19u, "TIM1 telemetry prescaler %u (expected 19)", unsigned(TELEM_PSC));
    // 240 MHz / (PSC+1) / 16 must equal the GCR bit rate exactly (no drift term)
    CHECK(TIM1_CLOCK_HZ / (TELEM_PSC + 1u) == GCR_BITRATE * TELEM_IC_SAMPLE,
          "IC sampling is exact: %u Hz", unsigned(TIM1_CLOCK_HZ / (TELEM_PSC + 1u)));
}

static void test_gcr_exhaustive()
{
    std::printf("T2 GCR round trip, all 4096 payloads, nominal timing\n");
    int bad = 0;
    for (uint32_t p = 0; p < 4096; p++) {
        if (roundtrip(uint16_t(p), 16u * 256u, 0) != p) bad++;
    }
    CHECK(bad == 0, "%d payloads failed nominal round trip", bad);

    // the IC counter is reset to 0 per receive window and never wraps inside
    // one (5.46 ms period vs a 28 us reply); late-start frames still decode
    bad = 0;
    for (uint32_t p = 0; p < 4096; p += 7) {
        if (roundtrip(uint16_t(p), 16u * 256u, 0, 64000) != p) bad++;
    }
    CHECK(bad == 0, "%d payloads failed with a late (t0 = 64000) start", bad);
}

static void test_clock_tolerance()
{
    std::printf("T3 ESC clock tolerance (per-interval rounding, no accumulation)\n");
    // longest run between GCR transitions is 3 bit cells; decoding fails when
    // 3 * |err| * 16 ticks >= 8 ticks  ->  |err| < 16.7 %
    int worst_ok_pct = 0;
    for (int pct = 0; pct <= 20; pct++) {
        bool all = true;
        for (int sgn = -1; sgn <= 1; sgn += 2) {
            const uint32_t tpb = uint32_t(lround(16.0 * 256.0 * (1.0 + sgn * pct / 100.0)));
            for (uint32_t p = 0; p < 4096 && all; p++) {
                if (roundtrip(uint16_t(p), tpb, 0) != p) all = false;
            }
        }
        if (!all) break;
        worst_ok_pct = pct;
    }
    std::printf("      all 4096 payloads decode up to +/-%d %% bit-clock error\n", worst_ok_pct);
    CHECK(worst_ok_pct >= 10, "clock tolerance only +/-%d %% (AM32 HSI is +/-1..2 %%)", worst_ok_pct);
    CHECK(worst_ok_pct < 17, "tolerance %d %% exceeds the 3-cell theoretical bound - model error", worst_ok_pct);
}

static void test_jitter()
{
    std::printf("T4 edge jitter / noise immunity\n");
    // +/-3 ticks (250 ns at 83.3 ns/tick) on every edge: intervals see at most
    // +/-6 ticks, inside the +/-8 tick rounding margin.
    int bad = 0, undetected = 0, trials = 0;
    for (int rep = 0; rep < 4; rep++) {
        for (uint32_t p = 0; p < 4096; p++) {
            const uint32_t r = roundtrip(uint16_t(p), 16u * 256u, 3);
            trials++;
            if (r != p) { bad++; if (r != INVALID_ERPM) undetected++; }
        }
    }
    CHECK(bad == 0, "+/-3 tick jitter: %d/%d frames lost", bad, trials);

    bad = undetected = trials = 0;
    for (int rep = 0; rep < 8; rep++) {
        for (uint32_t p = 0; p < 4096; p++) {
            const uint32_t r = roundtrip(uint16_t(p), 16u * 256u, 9);
            trials++;
            if (r != p) { bad++; if (r != INVALID_ERPM) undetected++; }
        }
    }
    const double rej = 100.0 * (bad - undetected) / trials, und = 100.0 * undetected / trials;
    std::printf("      +/-9 tick jitter: %.2f %% frames rejected, %.3f %% undetected corruption\n", rej, und);
    // A 4-bit CRC lets ~1/16 of random corruptions through; require the
    // GCR length check + CRC to keep undetected errors well under that.
    CHECK(bad == 0 || double(undetected) / bad < 1.0 / 16.0 + 0.02,
          "undetected/rejected ratio %.3f too high", bad ? double(undetected) / bad : 0.0);

    // dropped edge (glitch filtered out): must almost always be rejected
    int dropped_bad = 0, dropped_und = 0, dropped_n = 0;
    for (uint32_t p = 0; p < 4096; p += 3) {
        uint16_t e[GCR_TELEMETRY_BIT_LEN];
        uint32_t n = encode_gcr_edges(uint16_t(p), 100, 16u * 256u, e, GCR_TELEMETRY_BIT_LEN);
        const uint32_t k = 1 + (p % (n - 1));
        std::memmove(&e[k], &e[k + 1], (n - k - 1) * sizeof(e[0]));
        const uint32_t r = decode_gcr_edges(e, n - 1);
        dropped_n++;
        if (r != p) { dropped_bad++; if (r != INVALID_ERPM) dropped_und++; }
    }
    // Some dropped edges are harmless: the corrupted 5-bit symbol is an
    // invalid GCR code that the table maps to 0, which equals the true nibble
    // (0x19 -> 0).  Those decode to the correct payload and are not errors.
    std::printf("      missing-edge frames: %d/%d rejected, %d recovered exactly, %d undetected\n",
                dropped_bad - dropped_und, dropped_n, dropped_n - dropped_bad, dropped_und);
    CHECK(double(dropped_und) / dropped_n < 1.0 / 16.0 + 0.02, "missing-edge undetected rate %.3f", double(dropped_und) / dropped_n);

    CHECK(decode_gcr_edges(nullptr, 0) == INVALID_ERPM, "empty capture -> INVALID");
    uint16_t one = 5;
    CHECK(decode_gcr_edges(&one, 1) == INVALID_ERPM, "single edge -> INVALID");
}

static void test_payload_decode()
{
    std::printf("T5 eRPM / EDT payload decode (ArduPilot arithmetic)\n");
    // 40 000 RPM on a 14-pole motor = 280 000 eRPM = 214.29 us period
    const uint16_t p = encode_erpm_period(280000);
    CHECK(p == 214, "encode 280k eRPM -> 0x%03x (expected e0 m214)", p);
    Frame f = decode_payload(p, true);
    CHECK(f.type == FrameType::ERPM && f.erpm_x100 == 2804, "erpm_x100 %u", f.erpm_x100);
    CHECK(erpm_x100_to_rpm(f.erpm_x100, 14) == 40057, "rpm %u", unsigned(erpm_x100_to_rpm(f.erpm_x100, 14)));

    f = decode_payload(ZERO_ERPM, true);
    CHECK(f.type == FrameType::ERPM && f.erpm_x100 == 0, "ZERO_ERPM -> stopped");
    CHECK(decode_payload(INVALID_ERPM, true).type == FrameType::Invalid, "INVALID passthrough");
    CHECK(decode_payload(0x000, false).type == FrameType::ZeroPeriod, "period 0: CRC-valid, no RPM");

    // EDT temperature 80 C: e=001, bit8=0, value 80
    const uint16_t temp = uint16_t((1u << 9) | 80u);
    f = decode_payload(temp, true);
    CHECK(f.type == FrameType::EDT_Temperature && f.value == 80, "EDT temp type %d value %u", int(f.type), f.value);
    // the same frame with EDT disabled (SERVO_DSHOT_ESC 1) is read as 160 us
    f = decode_payload(temp, false);
    CHECK(f.type == FrameType::ERPM && f.erpm_x100 == 3750, "EDT-off misread erpm_x100 %u", f.erpm_x100);
    // voltage 16.8 V = 67 * 0.25
    f = decode_payload(uint16_t((2u << 9) | 67u), true);
    CHECK(f.type == FrameType::EDT_Voltage && f.value == 67, "EDT voltage");
    f = decode_payload(uint16_t((7u << 9) | 0x12u), true);
    CHECK(f.type == FrameType::EDT_Status, "EDT v2 status");
    // expo 0 with bit8 clear is an un-normalised eRPM, never EDT
    f = decode_payload(0x0C8, true);
    CHECK(f.type == FrameType::ERPM, "e=0 small period stays eRPM");
}

static void test_quantisation()
{
    std::printf("T6 eRPM quantisation over the motor envelope (14P, 1 500..42 000 RPM)\n");
    double worst = 0.0;
    uint32_t worst_rpm = 0;
    for (uint32_t rpm = 1500; rpm <= 42000; rpm += 50) {
        const uint32_t erpm = rpm * 7;
        const Frame f = decode_payload(encode_erpm_period(erpm), true);
        const double got = double(erpm_x100_to_rpm(f.erpm_x100, 14));
        const double err = std::fabs(got - rpm) / rpm;
        if (err > worst) { worst = err; worst_rpm = rpm; }
    }
    std::printf("      worst-case speed error %.3f %% at %u RPM (%.2f Hz)\n",
                worst * 100.0, unsigned(worst_rpm), worst * worst_rpm / 60.0);
    // INS_HNTCH_BW 40 Hz: speed error must stay well inside +/-BW/4 of the
    // 1st harmonic at top speed (700 Hz) -> 10 Hz -> 1.4 %
    CHECK(worst < 0.014, "quantisation error %.3f %% exceeds notch budget", worst * 100.0);
    // slow end: the format must still represent hover-idle (~1 500 RPM)
    CHECK(encode_erpm_period(1500 * 7) != ZERO_ERPM, "idle speed representable");
}

static void test_round_robin()
{
    std::printf("T7 TIM1 round-robin freshness (1 capture slot per frame, 4 motors)\n");
    MotorTelemetryMonitor mon(4, 3);
    const Frame good = decode_payload(encode_erpm_period(140000), true);   // 20k RPM, 429 us
    Frame bad{FrameType::Invalid, 0, 0};
    for (int i = 0; i < 4; i++) mon.on_frame(good);
    bool all = true;
    for (int m = 0; m < 4; m++) all = all && mon.fresh(uint8_t(m));
    CHECK(all, "all motors fresh after one full rotation");
    // motor 2 goes silent; others keep answering
    for (int i = 0; i < 4 * 4; i++) {
        mon.on_frame((i % 4) == 2 ? bad : good);
    }
    CHECK(mon.fresh(0) && mon.fresh(1) && mon.fresh(3), "healthy motors stay fresh");
    CHECK(!mon.fresh(2), "silent motor 2 stale after >3 own slots");
    CHECK(mon.health(2).consecutive_bad == 4, "motor 2 consecutive_bad %u", mon.health(2).consecutive_bad);
    // m2: 1 clean + 4 bad -> 100*4/(1+5) = 66.7 % (ArduPilot formula)
    CHECK(std::fabs(mon.error_rate_pct(2) - 66.666f) < 0.01f && mon.error_rate_pct(0) < 1.0f,
          "error rates m2 %.1f %% m0 %.1f %%", mon.error_rate_pct(2), mon.error_rate_pct(0));
    // EDT frames count as clean but do not refresh speed
    MotorTelemetryMonitor m2(4, 1);
    for (int i = 0; i < 4; i++) m2.on_frame(good);
    Frame edt{FrameType::EDT_Temperature, 0, 60};
    for (int i = 0; i < 8; i++) m2.on_frame(edt);
    CHECK(!m2.fresh(0) && m2.health(0).good_frames == 3, "EDT-only stream is clean but not fresh");
    // plausibility: a CRC-valid 9 us period decodes (wrapped) to 1131 x100 eRPM
    MotorTelemetryMonitor m3(4, 3, 100);
    const Frame wrapped = decode_payload(9, true);
    CHECK(wrapped.type == FrameType::ERPM && wrapped.erpm_x100 == 1131, "9 us period wraps in ArduPilot arithmetic");
    m3.on_frame(wrapped);
    CHECK(!m3.fresh(0) && m3.health(0).implausible_frames == 1 && m3.health(0).good_frames == 1,
          "9 us period rejected as implausible, still CRC-clean");
    MotorTelemetryMonitor m4(4, 3, 100);
    m4.on_frame(decode_payload(ZERO_ERPM, true));
    CHECK(m4.fresh(0), "stopped motor (ZERO_ERPM) is fresh, not implausible");
}

static void test_ap_error_attribution()
{
    std::printf("T8 ArduPilot RECV_FAILED error attribution (upstream defect + patch)\n");
    for (int patched = 0; patched <= 1; patched++) {
        ApErrorCounterModel ap(4, patched != 0, 0);
        for (int i = 0; i < 400; i++) {
            const int motor = i % 4;
            ap.on_frame(motor != 1, true);   // ESC2 (index 1) never answers
        }
        std::printf("      %s: errors m0..m3 = %u %u %u %u\n", patched ? "patched  " : "upstream ",
                    unsigned(ap.errors(0)), unsigned(ap.errors(1)), unsigned(ap.errors(2)), unsigned(ap.errors(3)));
        if (patched) {
            CHECK(ap.errors(1) == 100 && ap.errors(2) == 0, "patched: dead ESC2 charged to itself");
        } else {
            CHECK(ap.errors(1) == 0 && ap.errors(2) == 100, "upstream: dead ESC2 charged to ESC3 (defect reproduced)");
        }
    }
    // long outage with the 5 s window (1600 Hz frames -> 8000 frames): the
    // patched counter is bounded by the window instead of wrapping its uint16
    ApErrorCounterModel lp(4, true, 8000);
    uint16_t peak = 0;
    for (int i = 0; i < 400000; i++) {
        lp.on_frame((i % 4) != 1, true);
        if (lp.errors(1) > peak) peak = lp.errors(1);
    }
    CHECK(peak <= 2001 && peak >= 1999, "patched: silent ESC2 errors bounded by 5 s window (peak %u)", unsigned(peak));
}

static void test_notch_plan()
{
    std::printf("T9 notch plan, INS_HNTCH_FREQ 80, ATT 40, HMNCS 3, gyro 2 kHz (OPTS 6)\n");
    const float hz[4] = {15.0f, 50.0f, 300.0f, 500.0f};
    NotchPlan plan;
    plan_notches(hz, 4, 0x3, 80.0f, 2000.0f, 40.0f, plan);
    CHECK(plan.count == 8, "8 notch slots (4 motors x 2 harmonics), got %u", plan.count);
    CHECK(!plan.slot[0].enabled, "15 Hz < 0.25*min -> disabled");
    // 50 Hz: half-way from disable (20) to min (80) in LINEAR A: A = 1 + (0.1-1)*0.5 = 0.55
    // -> 10.4 dB, not 20 dB
    CHECK(plan.slot[1].enabled && std::fabs(plan.slot[1].center_hz - 80.0f) < 1e-3f &&
          std::fabs(plan.slot[1].depth_db - 10.385f) < 0.01f, "50 Hz -> 80 Hz centre, 10.4 dB (A interpolated), got %.2f",
          plan.slot[1].depth_db);
    CHECK(std::fabs(plan.slot[2].depth_db - 40.0f) < 0.01f, "full depth 40 dB above min");
    NotchPlan big;
    plan_notches(hz, 4, 0xFFFF, 80.0f, 8000.0f, 40.0f, big);
    CHECK(big.count == 54, "16 harmonics x 4 motors capped at HAL_HNF_MAX_FILTERS 54 (got %u)", big.count);
    CHECK(plan.slot[2].enabled && plan.slot[2].center_hz == 300.0f, "300 Hz h1");
    CHECK(plan.slot[6].enabled && plan.slot[6].center_hz == 600.0f, "300 Hz h2 = 600");
    CHECK(!plan.slot[7].enabled, "500 Hz h2 = 1000 Hz >= 960 Hz Nyquist guard -> disabled");
    CHECK(plan.slot[5].enabled && plan.slot[5].center_hz == 100.0f, "50 Hz h2 = 100 Hz above min");
}

int main()
{
    std::printf("Nano H743 AIO bidirectional DShot600 telemetry tests (Rev 10.0)\n");
    test_constants();
    test_gcr_exhaustive();
    test_clock_tolerance();
    test_jitter();
    test_payload_decode();
    test_quantisation();
    test_round_robin();
    test_ap_error_attribution();
    test_notch_plan();
    std::printf("\nRESULT: %s (%d checks, %d failures)\n", g_fail ? "FAIL" : "PASS", g_pass + g_fail, g_fail);
    return g_fail ? 1 : 0;
}
