// Nano H743 AIO - VTX BEC supervisor regression tests (Revision 13.0)
// Build + run:  make -C firmware/vtx_power test
//
// The supervisor runs closed-loop against Tps62933Plant: an averaged model of
// U6 with the R34/R35/R36 + Q1 feedback network, SS ramp from C38, Dmax 0.93
// (Eq.8, 500 kHz) or foldback dropout with Rds_on + L1 DCR, OVP 115 % / 5 %
// hysteresis, UVP 65 % hiccup, EN pull-up and UVLO. The ADC hook reads the
// plant's VTX_VSENSE through the R40/R41 divider with 12-bit quantisation.
// Faults injected: output short, Q1 D-S short (stuck 12 V), R34 open (rail to
// dropout), missing ADC samples, VBAT invalid, VBAT sag / 2S..4S packs, arming
// at every state, uint32 millisecond wrap. Deterministic LCG for the soak.
#include "vtx_power.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

using namespace nano_vtx;

static int g_fail = 0;
static int g_pass = 0;
#define CHECK(cond, ...) do { if (cond) { g_pass++; } else { g_fail++; \
    std::printf("  [FAIL] %s:%d ", __FILE__, __LINE__); std::printf(__VA_ARGS__); std::printf("\n"); } } while (0)

static uint32_t g_rng = 0x13A7B3C5u;
static float urand()
{
    g_rng = g_rng * 1664525u + 1013904223u;
    return float(g_rng >> 8) / 16777216.0f;
}

static bool near(float a, float b, float tol) { return std::fabs(a - b) <= tol; }

// ---------------------------------------------------------------------------
// Rig: plant + hooks + supervisor
// ---------------------------------------------------------------------------
struct Pins : VtxPins {
    Tps62933Plant* plant = nullptr;
    bool en = false, sel = false;
    char log[64] = {0};
    size_t nlog = 0;
    void note(char c) { if (nlog + 1 < sizeof(log)) { log[nlog++] = c; log[nlog] = 0; } }
    void set_enable(bool on) override
    {
        en = on;
        note(on ? 'E' : 'e');
        plant->set_en(on ? PinDrive::HIGH : PinDrive::LOW);
    }
    void set_select_12v(bool on) override
    {
        sel = on;
        note(on ? 'S' : 's');
        plant->set_sel(on ? PinDrive::HIGH : PinDrive::LOW);
    }
};

struct Adc : SenseAdc {
    Tps62933Plant* plant = nullptr;
    bool missing = false;
    bool read_pin_v(float& v) override
    {
        if (missing) return false;
        v = plant->sense_pin_v();
        return true;
    }
};

struct Rep : Reporter {
    int n = 0, crit = 0, warn = 0;
    char last[96] = {0};
    bool quiet = true;
    void text(Severity s, const char* msg) override
    {
        n++;
        if (s <= Severity::CRITICAL) crit++;
        if (s == Severity::WARNING) warn++;
        std::snprintf(last, sizeof(last), "%s", msg);
        if (!quiet) std::printf("      gcs: %s\n", msg);
    }
};

struct Rig {
    Tps62933Plant plant;
    Pins pins;
    Adc adc;
    Rep rep;
    VtxBecSupervisor sup;
    Inputs in;
    float iload = 0.5f, rshort = 0.0f;
    float plant_dt = 20e-6f;
    uint32_t armed_en_drops = 0;     // EN falling edges seen while armed
    float max_vout = 0.0f;

    explicit Rig(const Config& cfg = Config(), const PlantParams& pp = PlantParams(), uint32_t t0 = 0)
        : plant(pp), sup(pins, adc, &rep, cfg)
    {
        pins.plant = &plant;
        adc.plant = &plant;
        in.now_ms = t0;
        in.vbat_v = 16.0f;
        in.vbat_valid = true;
        in.want_power = true;
    }
    void boot()
    {
        plant.run(0.02f, in.vbat_v, iload, rshort, plant_dt);   // bootloader: pins high-Z
        sup.begin(in.now_ms);
    }
    // advance ms milliseconds of plant time, then one supervisor update
    void tick(uint32_t ms)
    {
        const long n = lroundf(float(ms) * 1e-3f / plant_dt);
        for (long i = 0; i < n; i++) {
            plant.step(plant_dt, in.vbat_v, iload, rshort);
            if (plant.vout() > max_vout) max_vout = plant.vout();
        }
        in.now_ms += ms;
        const bool en_before = pins.en;
        sup.update(in);
        if (in.armed && en_before && !pins.en) armed_en_drops++;
    }
    void run_ms(uint32_t total, uint32_t step_ms = 100)
    {
        for (uint32_t t = 0; t < total; t += step_ms) tick(step_ms);
    }
};

// ---------------------------------------------------------------------------
static void test_divider()
{
    std::printf("T1 FB divider R34/R35/R36 + Q1, Vref corners, leakage, OVP/UVP ratios\n");
    CHECK(near(rbot_effective(false), 10000.0f, 0.01f), "rbot 9V %f", double(rbot_effective(false)));
    CHECK(near(rbot_effective(true, 0.0f), 7275.5f, 1.0f), "rbot 12V %f", double(rbot_effective(true, 0.0f)));
    const float v9 = vout_setpoint(false), v12 = vout_setpoint(true);
    std::printf("      9 V set %.4f V, 12 V set %.4f V (Q1 Rds 10 R)\n", double(v9), double(v12));
    CHECK(near(v9, 8.96f, 0.005f), "9V %f", double(v9));
    CHECK(near(v12, 12.02f, 0.01f), "12V %f", double(v12));
    const float v9lo = vout_setpoint(false, hw::VREF_MIN), v9hi = vout_setpoint(false, hw::VREF_MAX);
    const float v12lo = vout_setpoint(true, hw::VREF_MIN), v12hi = vout_setpoint(true, hw::VREF_MAX);
    std::printf("      Vref 784..816 mV: 9 V %.3f..%.3f, 12 V %.3f..%.3f\n", double(v9lo), double(v9hi),
                double(v12lo), double(v12hi));
    CHECK(near(v9lo, 8.781f, 0.01f) && near(v9hi, 9.139f, 0.01f), "9V corners");
    CHECK(v12hi > 12.0f && v12hi < 12.3f, "12V max %f exceeds HDZero 12 V max -> 9 V default is mandatory", double(v12hi));
    const float dleak_fb = vout_setpoint(false, hw::VREF_TYP, hw::IFB_LEAK_MAX) - v9;
    const float dleak_q1 = vout_setpoint(false, hw::VREF_TYP, 1e-6f) - v9;
    CHECK(near(dleak_fb, 0.0153f, 0.001f), "IFB 0.15 uA -> %f V", double(dleak_fb));
    CHECK(near(dleak_q1, 0.102f, 0.002f), "Q1 1 uA leakage -> %f V", double(dleak_q1));
    CHECK(q1_gate_drive_v() > 3.26f && q1_gate_drive_v() > 2.5f, "gate %f V vs Vth max 2.5 V", double(q1_gate_drive_v()));
    const float ovp_ratio = fb_voltage(v12, false) / hw::VREF_TYP;
    const float uvp_ratio = fb_voltage(v9, true) / hw::VREF_TYP;
    std::printf("      12->9 V step: FB %.1f %% (OVP 112..118 %%); 9->12 V step: FB %.1f %% (UVP 65 %%)\n",
                double(ovp_ratio * 100.0f), double(uvp_ratio * 100.0f));
    CHECK(ovp_ratio > 1.18f, "12->9 always trips OVP: %f", double(ovp_ratio));
    CHECK(uvp_ratio > hw::UVP_RATIO + 0.05f, "9->12 never trips UVP: %f", double(uvp_ratio));
    CHECK(near(hw::SENSE_MULT, 11.0f, 1e-6f) && near(v12 / hw::SENSE_MULT, 1.093f, 0.002f), "sense x11, 12 V -> %f V",
          double(v12 / hw::SENSE_MULT));
}

static void test_soft_start()
{
    std::printf("T2 soft start C38 33 nF (DS Eq.6), plant ramp, 10 ms blank\n");
    const float tss = soft_start_s(), tss_slow = soft_start_s(hw::C38_SS_F, hw::ISS_MIN),
                tss_fast = soft_start_s(hw::C38_SS_F, hw::ISS_MAX);
    std::printf("      tSS %.2f ms typ (%.2f..%.2f ms over ISS 6.5..4.5 uA)\n", double(tss * 1e3f),
                double(tss_fast * 1e3f), double(tss_slow * 1e3f));
    CHECK(near(tss, 4.8e-3f, 0.05e-3f), "tss %f", double(tss));
    CHECK(tss_slow < float(Config().settle_ms) * 1e-3f, "slow tss %f < settle", double(tss_slow));
    Tps62933Plant p;
    p.set_sel(PinDrive::LOW);
    p.set_en(PinDrive::HIGH);
    p.run(tss * 0.5f, 16.8f, 0.5f);
    const float half = p.vout();
    p.run(tss * 0.5f + 1.2e-3f, 16.8f, 0.5f);
    const float done = p.vout();
    std::printf("      at tSS/2 %.3f V, at tSS+1.2 ms %.3f V, OVP events %u\n", double(half), double(done),
                unsigned(p.ovp_events()));
    CHECK(near(half, 4.48f, 0.45f), "half %f", double(half));
    CHECK(near(done, 8.96f, 0.01f), "done %f", double(done));
    CHECK(p.ovp_events() == 0 && p.hiccup_events() == 0, "clean start");
}

static void test_dropout()
{
    std::printf("T3 dropout: Dmax (Eq.8) 0.93 / foldback, Vin_min, 4S-only 12 V\n");
    const float d = dmax_fixed(), dfb = dmax_foldback(), d12 = dmax_fixed(1.2e6f);
    std::printf("      Dmax 500 kHz %.3f, foldback %.4f, 1.2 MHz %.3f\n", double(d), double(dfb), double(d12));
    CHECK(near(d, 0.93f, 1e-4f), "dmax %f", double(d));
    CHECK(near(dfb, 0.9804f, 0.001f), "foldback %f", double(dfb));
    CHECK(near(d12, 0.832f, 1e-3f), "1.2 MHz %f", double(d12));
    const float vmin12 = dropout_vin_min(vout_setpoint(true), 2.0f, d);
    const float vmin12f = dropout_vin_min(vout_setpoint(true), 2.0f, dfb);
    const float vmin9 = dropout_vin_min(vout_setpoint(false), 2.0f, d);
    std::printf("      Vin_min @2 A: 12 V %.2f V (fixed) / %.2f V (foldback), 9 V %.2f V\n", double(vmin12),
                double(vmin12f), double(vmin9));
    CHECK(vmin12 > 13.1f && vmin12 < 13.3f, "vmin12 %f", double(vmin12));
    CHECK(vmin12f > 12.4f && vmin12f < 12.65f, "vmin12 foldback %f", double(vmin12f));
    CHECK(vmin9 > 9.85f && vmin9 < 10.05f, "vmin9 %f", double(vmin9));
    const Config c;
    CHECK(c.v12_enable_vbat > vmin12 + 0.35f, "13.6 V entry has %f V margin over fixed-Dmax dropout",
          double(c.v12_enable_vbat - vmin12));
    CHECK(c.v12_enable_vbat > 3.0f * 4.2f, "3S full (12.6 V) can never qualify for 12 V");
    const float at_drop = dropout_vout_max(c.v12_drop_vbat, 2.0f, d);
    CHECK(at_drop > vout_setpoint(true) * (1.0f - c.window_tol), "at 13.0 V / 2 A the rail is %f V (in window)",
          double(at_drop));
    Tps62933Plant p;
    p.set_sel(PinDrive::LOW);
    p.set_en(PinDrive::HIGH);
    p.run(0.02f, 7.4f, 1.0f);
    std::printf("      2S nominal 7.4 V, 1 A, 9 V mode: plant %.3f V (pass-through, below DJI 7.4 V min)\n",
                double(p.vout()));
    CHECK(near(p.vout(), dropout_vout_max(7.4f, 1.0f, 0.93f), 0.01f) && p.vout() < 7.4f, "2S %f", double(p.vout()));
}

static void test_ovp_transition()
{
    std::printf("T4 OVP on 12 -> 9 V (DS s9.3.11), no UVP on 9 -> 12 V\n");
    PlantParams pp;
    Tps62933Plant p(pp);
    p.set_sel(PinDrive::HIGH);
    p.set_en(PinDrive::HIGH);
    p.run(0.02f, 16.8f, 0.0f);
    CHECK(near(p.vout(), 12.02f, 0.02f), "at 12 V %f", double(p.vout()));
    p.set_sel(PinDrive::LOW);
    float t = 0.0f, t_release = -1.0f, t_window = -1.0f;
    const float dt = 20e-6f;
    while (t < 0.5f) {
        p.step(dt, 16.8f, 0.0f);
        t += dt;
        if (t_release < 0.0f && !p.ovp()) t_release = t;
        if (t_window < 0.0f && p.vout() <= vout_setpoint(false) * 1.06f) t_window = t;
    }
    // no-load bleed: FB divider 112k || sense 11k
    const float rbl = 1.0f / (1.0f / (hw::R34_TOP_OHM + hw::R35_BOT_OHM) + 1.0f / (hw::R40_TOP_OHM + hw::R41_BOT_OHM));
    const float tau = rbl * pp.cout_eff_f;
    const float t_rel_pred = tau * std::log(12.02f / ((hw::OVP_RATIO - hw::OVP_HYST) * (1.0f + 102.0f / 10.0f) * 0.8f));
    std::printf("      no load, Cout %.0f uF: OVP events %u, release %.1f ms (pred %.1f), <= +6 %% after %.1f ms\n",
                double(pp.cout_eff_f * 1e6f), unsigned(p.ovp_events()), double(t_release * 1e3f),
                double(t_rel_pred * 1e3f), double(t_window * 1e3f));
    CHECK(p.ovp_events() == 1, "ovp events %u", unsigned(p.ovp_events()));
    CHECK(t_release > 0.0f && near(t_release, t_rel_pred, 0.1f * t_rel_pred), "release %f", double(t_release));
    CHECK(t_window > 0.0f && t_window * 1e3f < float(Config().down_settle_ms), "window %f < down blank", double(t_window));
    CHECK(near(p.vout(), 8.96f, 0.03f), "settles to 9 V: %f", double(p.vout()));
    // with a VTX load the decay is fast
    Tps62933Plant q(pp);
    q.set_sel(PinDrive::HIGH);
    q.set_en(PinDrive::HIGH);
    q.run(0.02f, 16.8f, 1.0f);
    q.set_sel(PinDrive::LOW);
    q.run(1e-3f, 16.8f, 1.0f);
    CHECK(q.vout() < 9.5f && !q.ovp(), "1 A load: %f V after 1 ms", double(q.vout()));
    // 9 -> 12 V: FB 74.6 % > UVP 65 %
    q.set_sel(PinDrive::HIGH);
    q.run(1e-3f, 16.8f, 1.0f);
    CHECK(near(q.vout(), 12.02f, 0.02f) && q.hiccup_events() == 0 && q.ovp_events() == 1, "9->12 %f hiccup %u",
          double(q.vout()), unsigned(q.hiccup_events()));
}

static void test_boot()
{
    std::printf("T5 boot state (high-Z: EN pull-up ON, gate pull-down 9 V), SEL before EN\n");
    Rig r;
    r.plant.run(0.02f, 16.0f, 0.5f);
    CHECK(r.plant.running() && near(r.plant.vout(), 8.96f, 0.02f), "bootloader rail %f", double(r.plant.vout()));
    CHECK(!r.plant.q1_on(), "Q1 off with gate high-Z");
    r.sup.begin(0);
    CHECK(std::strcmp(r.pins.log, "sE") == 0, "order %s", r.pins.log);
    r.tick(5);
    CHECK(r.sup.state() == RailState::STARTING, "blank before 10 ms: %s", to_string(r.sup.state()));
    r.tick(10);
    CHECK(r.sup.state() == RailState::RUNNING && (r.sup.health().flags & F_RAIL_OK), "running %s",
          to_string(r.sup.state()));
    // cold start through begin() (EN low before, then soft start)
    Rig c;
    c.plant.set_en(PinDrive::LOW);
    c.plant.run(0.01f, 16.0f, 0.5f);
    CHECK(c.plant.vout() < 0.01f, "EN low -> off");
    c.sup.begin(0);
    c.run_ms(100, 10);
    CHECK(c.sup.state() == RailState::RUNNING && c.sup.health().faults_uv == 0, "cold start no fault");
}

static void test_interlock()
{
    std::printf("T6 12 V interlock: >= 13.6 V for 2 s, < 13.0 V for 0.3 s, 4S only\n");
    {
        Rig r;
        r.in.vbat_v = 16.0f;
        r.in.want_12v = true;
        r.boot();
        uint32_t grant = 0;
        for (int i = 0; i < 40 && !grant; i++) {
            r.tick(100);
            if (r.sup.select_12v()) grant = r.in.now_ms;
        }
        std::printf("      request at boot, VBAT 16.0 V: 12 V granted at %u ms\n", unsigned(grant));
        CHECK(grant >= 2000 && grant <= 2100, "grant %u", unsigned(grant));
        r.tick(100);
        CHECK(near(r.plant.vout(), 12.02f, 0.03f) && (r.sup.health().flags & F_RAIL_OK), "rail %f",
              double(r.plant.vout()));
        // sag 0.2 s below 13.0 -> stays
        r.in.vbat_v = 12.9f;
        r.tick(100);
        r.tick(100);
        r.in.vbat_v = 14.0f;
        r.tick(100);
        CHECK(r.sup.select_12v(), "0.2 s sag keeps 12 V");
        // sag 0.3 s -> 9 V
        r.in.vbat_v = 12.9f;
        uint32_t t0 = r.in.now_ms + 100, tdrop = 0;
        for (int i = 0; i < 10 && !tdrop; i++) {
            r.tick(100);
            if (!r.sup.select_12v()) tdrop = r.in.now_ms;
        }
        CHECK(tdrop - t0 == 300, "drop after %u ms", unsigned(tdrop - t0));
        CHECK(r.sup.health().v12_drops_lowbat == 1 && std::strstr(r.rep.last, "12V -> 9V") != nullptr, "msg %s",
              r.rep.last);
        // recovery needs the full 2 s again
        r.in.vbat_v = 14.0f;
        r.run_ms(1900);
        CHECK(!r.sup.select_12v(), "no re-grant before 2 s");
        r.run_ms(300);
        CHECK(r.sup.select_12v(), "re-grant after 2 s");
        // request withdrawn -> immediate 9 V
        r.in.want_12v = false;
        r.tick(100);
        CHECK(!r.sup.select_12v() && r.sup.health().v12_drops_request == 1, "release");
        CHECK(r.sup.health().faults_ov == 0 && r.sup.health().faults_uv == 0, "no faults through 12/9 cycles");
    }
    {
        Rig r;
        r.in.vbat_v = 12.6f;   // 3S full
        r.in.want_12v = true;
        r.boot();
        r.run_ms(10000);
        CHECK(!r.sup.select_12v() && r.sup.health().v12_refused == 1, "3S refused (%u)",
              unsigned(r.sup.health().v12_refused));
        CHECK(std::strstr(r.rep.last, "refused") != nullptr, "msg %s", r.rep.last);
    }
    {
        Rig r;   // dip in the qualification window restarts it
        r.in.vbat_v = 15.0f;
        r.in.want_12v = true;
        r.boot();
        r.run_ms(1500);
        r.in.vbat_v = 13.59f;
        r.tick(100);
        r.in.vbat_v = 13.6f;   // exactly the threshold qualifies (>=)
        r.run_ms(1900);
        CHECK(!r.sup.select_12v(), "dip restarted qualification");
        r.run_ms(200);
        CHECK(r.sup.select_12v(), "13.6 V exactly qualifies");
        r.in.vbat_v = 13.0f;   // exactly 13.0 is not < 13.0
        r.run_ms(2000);
        CHECK(r.sup.select_12v(), "13.0 V exactly holds 12 V");
        r.in.vbat_valid = false;
        r.run_ms(400);
        CHECK(!r.sup.select_12v(), "invalid VBAT drops 12 V");
    }
}

static void test_armed()
{
    std::printf("T7 never VTX_PWR off while armed; deferred off; arm during retry\n");
    Rig r;
    r.boot();
    r.run_ms(500);
    r.in.armed = true;
    r.in.want_power = false;
    r.run_ms(3000);
    CHECK(r.pins.en && r.sup.health().off_deferred == 1 && (r.sup.health().flags & F_OFF_DEFERRED), "deferred");
    r.in.armed = false;
    r.tick(100);
    CHECK(!r.pins.en && r.sup.state() == RailState::OFF, "off after disarm");
    r.in.want_power = true;
    r.run_ms(200);
    CHECK(r.pins.en && r.sup.state() == RailState::RUNNING && !r.pins.sel, "back on at 9 V");
    // short while disarmed -> retry off, then arm -> immediate re-enable
    r.rshort = 0.3f;
    uint32_t guard = 0;
    while (r.sup.state() != RailState::RETRY_OFF && guard++ < 50) r.tick(100);
    CHECK(r.sup.state() == RailState::RETRY_OFF && !r.pins.en, "in retry");
    r.in.armed = true;
    r.tick(100);
    CHECK(r.pins.en && r.sup.state() == RailState::STARTING, "armed -> re-enabled (%s)", to_string(r.sup.state()));
    r.run_ms(5000);
    CHECK(r.pins.en && r.armed_en_drops == 0 && r.sup.health().en_low_while_armed == 0, "never low while armed");
    CHECK(r.sup.health().faults_while_armed >= 1 && r.rep.crit >= 1, "armed fault reported");
}

static void test_window_sweep()
{
    std::printf("T8 dropout-aware window vs plant: VBAT 6..17 V, 0..2 A, Dmax 0.93/foldback, 25 C/hot\n");
    const Config cfg;
    Pins pins;
    Adc adc;
    Tps62933Plant dummy;
    pins.plant = &dummy;
    adc.plant = &dummy;
    VtxBecSupervisor sup(pins, adc, nullptr, cfg);
    int cases = 0, bad = 0;
    float worst_lo = 1e9f;
    const float loads[4] = {0.0f, 0.5f, 1.0f, 2.0f};
    for (int dm = 0; dm < 2; dm++) {
        for (int hot = 0; hot < 2; hot++) {
            for (int sel = 0; sel < 2; sel++) {
                for (int li = 0; li < 4; li++) {
                    for (int k = 0; k <= 110; k++) {
                        const float vbat = 6.0f + 0.1f * float(k);
                        if (sel && vbat < cfg.v12_drop_vbat - 0.3f) continue;   // 12 V only exists >= ~13 V
                        PlantParams pp;
                        pp.dmax = dm ? dmax_foldback() : 0.93f;
                        if (hot) { pp.rhs = 0.110f; pp.rls = 0.045f; pp.dcr = 0.065f; }
                        Tps62933Plant p(pp);
                        p.set_sel(sel ? PinDrive::HIGH : PinDrive::LOW);
                        p.set_en(PinDrive::HIGH);
                        p.run(0.012f, vbat, loads[li], 0.0f, 40e-6f);
                        const float v = p.sense_pin_v() * cfg.sense_mult;
                        const Window w = sup.rail_window(sel != 0, vbat, true);
                        cases++;
                        if (v < w.lo || v > w.hi) {
                            bad++;
                            if (bad < 4) std::printf("      out: vbat %.1f sel %d I %.1f v %.3f [%.3f %.3f]\n",
                                                     double(vbat), sel, double(loads[li]), double(v), double(w.lo), double(w.hi));
                        }
                        const float m = (v - w.lo) / (w.lo > 0.0f ? w.lo : 1.0f);
                        if (m < worst_lo) worst_lo = m;
                    }
                }
            }
        }
    }
    std::printf("      %d steady states, %d outside window, worst low margin %.2f %%\n", cases, bad,
                double(worst_lo * 100.0f));
    CHECK(cases > 1500 && bad == 0, "false faults %d / %d", bad, cases);
    // Spec D3 single-point rule (expected = VBAT - 0.4 V, +/-6 %) at the knee,
    // 9 V mode, 2 A, Dmax 0.93, hot: plant output vs expected.
    PlantParams pp;
    pp.rhs = 0.110f; pp.rls = 0.045f; pp.dcr = 0.065f;
    Tps62933Plant p(pp);
    p.set_sel(PinDrive::LOW);
    p.set_en(PinDrive::HIGH);
    p.run(0.012f, 9.36f, 2.0f);
    const Window w = sup.rail_window(false, 9.36f, true);
    const float dev = p.vout() / w.expected - 1.0f;
    std::printf("      spec point rule at VBAT 9.36 V / 2 A / hot: %.3f V vs expected %.3f V = %.2f %% (limit -6 %%)\n",
                double(p.vout()), double(w.expected), double(dev * 100.0f));
    CHECK(dev < -0.06f && p.vout() >= w.lo, "single-point rule would false-fault here, band does not");
    CHECK(sup.rail_window(false, 9.0f, true).dropout && !sup.rail_window(false, 16.8f, true).dropout, "dropout flag");
    const Window wi = sup.rail_window(false, 0.0f, false);
    CHECK(wi.lo == 0.0f && near(wi.hi, 8.96f * 1.06f, 0.01f), "invalid VBAT -> OV-only window");
}

static void test_faults()
{
    std::printf("T9 faults: short (hiccup) retries, Q1 short / R34 open OV latch, 12 V fault, no sense\n");
    {
        Rig r;
        r.boot();
        r.run_ms(300);
        r.rshort = 0.3f;
        uint32_t t0 = r.in.now_ms, t_first = 0;
        uint32_t off_ms = 0;
        for (int i = 0; i < 200; i++) {
            r.tick(50);
            if (!r.pins.en) off_ms += 50;
            if (!t_first && r.sup.health().retries) t_first = r.in.now_ms;
        }
        std::printf("      short 0.3 R: first retry after %u ms, retries %u, EN-low %u ms, hiccups %u, state %s\n",
                    unsigned(t_first - t0), unsigned(r.sup.health().retries), unsigned(off_ms),
                    unsigned(r.plant.hiccup_events()), to_string(r.sup.state()));
        CHECK(r.plant.hiccup_events() > 0, "plant hiccup");
        CHECK(t_first - t0 >= 300 && t_first - t0 <= 400, "persistence %u", unsigned(t_first - t0));
        CHECK(r.sup.health().retries == 3 && r.sup.state() == RailState::LATCHED_ON && r.pins.en, "latched on");
        CHECK(off_ms >= 3 * 500 && off_ms <= 3 * 550, "EN low %u ms", unsigned(off_ms));
        CHECK(!r.sup.clear_latch(true, r.in.now_ms), "clear refused armed");
        r.rshort = 0.0f;
        CHECK(r.sup.clear_latch(false, r.in.now_ms), "clear");
        r.run_ms(500);
        CHECK(r.sup.state() == RailState::RUNNING && r.sup.fault() == FaultKind::NONE, "recovered %s",
              to_string(r.sup.state()));
    }
    {
        Rig r;   // transient short < persistence: no action
        r.boot();
        r.run_ms(300);
        r.rshort = 0.3f;
        r.run_ms(200);
        r.rshort = 0.0f;
        r.run_ms(500);
        CHECK(r.sup.health().retries == 0 && r.sup.state() == RailState::RUNNING, "200 ms glitch ignored");
    }
    {
        PlantParams pp;
        pp.q1_short = true;   // stuck 12 V
        Rig r(Config(), pp);
        r.in.armed = true;
        r.boot();
        r.run_ms(2000);
        CHECK(r.pins.en && r.sup.health().faults_ov >= 1 && r.armed_en_drops == 0, "Q1 short armed: on, OV %u",
              unsigned(r.sup.health().faults_ov));
        const int msgs = r.rep.n;
        r.run_ms(3000);
        CHECK(r.rep.n == msgs, "no message spam while armed (%d -> %d)", msgs, r.rep.n);
        r.in.armed = false;
        r.run_ms(1000);
        CHECK(!r.pins.en && r.sup.state() == RailState::LATCHED_OFF, "disarm -> latched off (%s)",
              to_string(r.sup.state()));
    }
    {
        PlantParams pp;
        pp.r34_open = true;   // FB top resistor open: FB at GND, output runs to dropout
        Rig r(Config(), pp);
        r.in.vbat_v = 16.8f;
        r.boot();
        r.run_ms(1000);
        std::printf("      R34 open @16.8 V: peak %.2f V, state %s\n", double(r.max_vout), to_string(r.sup.state()));
        CHECK(r.sup.state() == RailState::LATCHED_OFF && !r.pins.en && r.plant.vout() < 9.0f, "OV latch");
    }
    {
        Rig r;   // fault at 12 V: back to 9 V + lockout; retries untouched
        r.in.want_12v = true;
        r.boot();
        r.run_ms(2500);
        CHECK(r.sup.select_12v(), "at 12 V");
        r.iload = 4.5f;      // overload above the 3.55 A average limit
        r.run_ms(600);
        CHECK(!r.sup.select_12v() && r.sup.health().v12_drops_fault == 1 && r.sup.health().retries == 0,
              "12 V fault -> 9 V (drops %u retries %u)", unsigned(r.sup.health().v12_drops_fault),
              unsigned(r.sup.health().retries));
        r.iload = 0.5f;
        r.run_ms(3000);
        CHECK(!r.sup.select_12v() && (r.sup.health().flags & F_V12_LOCKOUT), "locked out");
        r.in.want_12v = false;
        r.tick(100);
        r.in.want_12v = true;
        r.run_ms(300);
        CHECK(r.sup.select_12v(), "toggle clears lockout");
    }
    {
        Rig r;   // ADC missing: never power-cycles, drops 12 V
        r.in.want_12v = true;
        r.boot();
        r.run_ms(2500);
        r.adc.missing = true;
        r.run_ms(1000);
        CHECK(r.pins.en && !r.sup.select_12v() && r.sup.health().faults_nosense >= 1 && r.sup.health().retries == 0,
              "no sense handled");
    }
}

static void test_down_blank()
{
    std::printf("T10 12 -> 9 V OV blank: no-load decay with large Cout\n");
    for (int blank = 0; blank < 2; blank++) {
        Config c;
        if (!blank) c.down_settle_ms = 0;
        PlantParams pp;
        pp.cout_eff_f = 200e-6f;   // 30 uF on-board + VTX input caps, VTX drawing nothing
        Rig r(c, pp);
        r.iload = 0.0f;
        r.in.want_12v = true;
        r.boot();
        r.run_ms(2500);
        r.in.want_12v = false;
        r.run_ms(1500);
        std::printf("      down blank %u ms: OV faults %u, state %s, rail %.2f V\n", unsigned(c.down_settle_ms),
                    unsigned(r.sup.health().faults_ov), to_string(r.sup.state()), double(r.plant.vout()));
        if (blank) {
            CHECK(r.sup.health().faults_ov == 0 && r.sup.state() == RailState::RUNNING, "blank prevents false OV");
        } else {
            CHECK(r.sup.health().faults_ov >= 1, "without blank a false OV latch happens (justifies 250 ms)");
        }
    }
}

static void test_wrap()
{
    std::printf("T11 uint32 millisecond wrap\n");
    Rig r(Config(), PlantParams(), 0xFFFFFF00u);
    r.in.want_12v = true;
    r.boot();
    const uint32_t start = r.in.now_ms;
    uint32_t grant = 0;
    for (int i = 0; i < 40 && !grant; i++) {
        r.tick(100);
        if (r.sup.select_12v()) grant = r.in.now_ms;
    }
    CHECK(grant < start && uint32_t(grant - start) >= 2000 && uint32_t(grant - start) <= 2100, "grant across wrap %u",
          unsigned(grant - start));
    CHECK(r.sup.health().faults_uv == 0 && r.sup.health().faults_ov == 0, "no spurious faults across wrap");
}

static void test_soak()
{
    std::printf("T12 soak: 600 s random packs/sag/load/arm/requests, invariants\n");
    Rig r;
    r.plant_dt = 50e-6f;
    r.in.vbat_v = 16.0f;
    r.boot();
    float pack = 16.0f;
    bool sag = false;
    uint32_t sag_left = 0;
    // shadow of the policy timers for invariant checks
    uint32_t hi_since = 0, lo_since = 0;
    bool hi = false, lo = false, prev_sel = false;
    int viol_grant = 0, viol_hold = 0;
    for (int i = 0; i < 6000; i++) {
        const float u = urand();
        if (u < 0.002f) {
            const int cells = 2 + int(urand() * 3.0f);
            pack = float(cells) * (3.3f + 0.9f * urand());
        }
        if (!sag && urand() < 0.01f) { sag = true; sag_left = 1 + uint32_t(urand() * 8.0f); }
        float v = pack;
        if (sag) { v -= 0.3f + 1.2f * urand(); if (--sag_left == 0) sag = false; }
        r.in.vbat_v = v;
        if (urand() < 0.01f) r.iload = 2.0f * urand();
        if (urand() < 0.005f) r.in.armed = !r.in.armed;
        if (urand() < 0.004f) r.in.want_12v = !r.in.want_12v;
        if (urand() < 0.002f) r.in.want_power = !r.in.want_power;
        r.tick(100);
        const uint32_t now = r.in.now_ms;
        if (v >= 13.6f) { if (!hi) { hi = true; hi_since = now; } } else hi = false;
        if (v < 13.0f) { if (!lo) { lo = true; lo_since = now; } } else lo = false;
        const bool sel = r.sup.select_12v();
        if (sel && !prev_sel && !(hi && now - hi_since >= 2000)) viol_grant++;
        if (sel && lo && now - lo_since >= 300) viol_hold++;
        prev_sel = sel;
    }
    const Health& h = r.sup.health();
    std::printf("      grants %u, drops lowbat/request %u/%u, refused %u, deferred off %u, faults uv/ov %u/%u, "
                "peak %.2f V\n", unsigned(h.v12_grants), unsigned(h.v12_drops_lowbat), unsigned(h.v12_drops_request),
                unsigned(h.v12_refused), unsigned(h.off_deferred), unsigned(h.faults_uv), unsigned(h.faults_ov),
                double(r.max_vout));
    CHECK(h.v12_grants > 5 && h.v12_drops_lowbat > 0, "coverage grants %u drops %u", unsigned(h.v12_grants),
          unsigned(h.v12_drops_lowbat));
    CHECK(viol_grant == 0, "12 V granted without 2 s >= 13.6 V: %d", viol_grant);
    CHECK(viol_hold == 0, "12 V held with VBAT < 13.0 V for >= 0.3 s: %d", viol_hold);
    CHECK(r.armed_en_drops == 0 && h.en_low_while_armed == 0, "EN low while armed");
    CHECK(h.faults_uv == 0 && h.faults_ov == 0 && h.retries == 0, "healthy plant -> zero faults");
    CHECK(r.max_vout < vout_setpoint(true) * 1.02f && r.plant.hiccup_events() == 0, "no overshoot/hiccup");
}

int main()
{
    std::printf("Nano H743 AIO VTX BEC supervisor tests (Rev 13.0)\n");
    test_divider();
    test_soft_start();
    test_dropout();
    test_ovp_transition();
    test_boot();
    test_interlock();
    test_armed();
    test_window_sweep();
    test_faults();
    test_down_blank();
    test_wrap();
    test_soak();
    std::printf("\nRESULT: %s (%d checks, %d failures)\n", g_fail ? "FAIL" : "PASS", g_pass + g_fail, g_fail);
    return g_fail ? 1 : 0;
}
