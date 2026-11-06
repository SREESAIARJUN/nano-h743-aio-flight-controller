// Nano H743 AIO - AT7456E OSD driver regression tests (Revision 13.0)
// Build + run:  make -C firmware/osd_at7456e test
//
// SimAt7456e is a register-level AT7456E behind a simulated SPI2 with a
// nanosecond timeline: every byte takes 8 / SCLK, a 16-bit command takes
// effect at the end of its second byte, read data is shifted out during the
// second byte. Modelled datasheet behaviour (AT7456E-S402-V1.1):
//   - CMM=0x50 NVM read: STAT[5]=1 for nvm_read_ns (22 us min / 30 us typ,
//     p.36); the shadow RAM is loaded at the END, so CMDO reads before that
//     return the previous shadow contents; CMAH/CMAL/CMDI/CMM/VM0 writes while
//     STAT[5]=1 are ignored (p.21 note 1, p.22, p.29).
//   - CMM=0xA0 NVM write: busy 5 ms typ (p.29, p.36), per-character wear count.
//   - VM0[3]=0 takes display_off_ns to take effect (p.36 "0~18us" min, 30 us
//     typ); CMAH/CMAL/CMDI/CMM writes are ignored while the OSD is still on
//     (p.29).
//   - VM0[1] soft reset 40 us (p.36), DMM[2] clear 40 us with display-memory
//     writes ignored meanwhile (p.21 note 2, p.34).
//   - display memory auto-increment terminated by DMDI = 0xFF (p.35).
//     NOTE: DS p.35 calls auto-increment writes "8-bit"; the simulator decodes
//     the (DMDI, char) 16-bit pairs ArduPilot sends, which is how MAX7456-class
//     parts behave with AP_OSD_MAX7456 in the field (open item: bench-verify
//     on AT7456E silicon).
//   - STAT[2:0] from the video input: PAL 0x01, NTSC 0x02, LOS 0x04, and the
//     "nothing detected" 0x00 state the AP comment attributes to AB7456.
#include "osd_at7456e.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

using namespace nano_osd;

static int g_fail = 0;
static int g_pass = 0;
#define CHECK(cond, ...) do { if (cond) { g_pass++; } else { g_fail++; \
    std::printf("  [FAIL] %s:%d ", __FILE__, __LINE__); std::printf(__VA_ARGS__); std::printf("\n"); } } while (0)

static uint32_t g_rng = 0x7456E013u;
static uint32_t rnd()
{
    g_rng = g_rng * 1664525u + 1013904223u;
    return g_rng >> 8;
}

// ---------------------------------------------------------------------------
// Device simulator
// ---------------------------------------------------------------------------
struct SimAt7456e : SpiDevice {
    // configuration
    uint32_t sclk_hz = 5000000;
    uint32_t nvm_read_ns = timing::NVM_READ_TYP_NS;
    uint32_t nvm_write_ns = timing::NVM_WRITE_TYP_NS;
    uint32_t display_off_ns = timing::DISPLAY_OFF_TYP_NS;
    uint32_t reset_ns = timing::SOFT_RESET_TYP_NS;
    uint32_t clear_ns = timing::CLEAR_TYP_NS;
    uint32_t cs_setup_ns = 100;       // /CS fall to first SCLK
    uint32_t cs_gap_ns = 2000;        // HAL overhead between transfers
    bool present = true;
    uint8_t video = STAT_PAL;
    // state
    uint64_t t = 0;
    uint8_t regs[128];
    uint8_t shadow[64];
    uint8_t nvm[512][64];
    uint32_t wear[512];
    uint8_t disp[512];
    uint16_t cursor = 0;
    enum Op { NONE, NVM_READ, NVM_WRITE } op = NONE;
    uint64_t op_end = 0;
    uint16_t op_char = 0;
    bool resetting = false, clearing = false, off_pending = false;
    uint64_t reset_end = 0, clear_end = 0, off_at = 0;
    // counters
    uint32_t ignored = 0, stale_cmdo = 0, nvm_reads = 0, nvm_writes = 0, timing_violations = 0;
    uint32_t transfers = 0, bytes = 0;

    SimAt7456e()
    {
        memset(regs, 0, sizeof(regs));
        memset(shadow, 0, sizeof(shadow));
        memset(nvm, 0, sizeof(nvm));
        memset(wear, 0, sizeof(wear));
        memset(disp, 0, sizeof(disp));
    }
    uint64_t byte_ns() const { return 8000000000ull / sclk_hz; }
    bool busy(uint64_t now) const { return op != NONE && now < op_end; }
    bool osd_on(uint64_t now) const { return (regs[reg::VM0] & VM0_OSD_ENABLE) || (off_pending && now < off_at); }
    uint16_t addr() const { return uint16_t(((regs[reg::DMAH] & 1u) << 8) | regs[reg::DMAL]); }
    uint16_t cmchar() const { return uint16_t(regs[reg::CMAH] | ((regs[reg::CMAL] & 0x40u) << 2)); }

    void advance(uint64_t now)
    {
        if (op != NONE && now >= op_end) {
            if (op == NVM_READ) {
                memcpy(shadow, nvm[op_char], 64);
            } else {
                memcpy(nvm[op_char], shadow, CHAR_BYTES);
                wear[op_char]++;
                nvm_writes++;
            }
            regs[reg::CMM] = 0;
            op = NONE;
        }
        if (resetting && now >= reset_end) {
            resetting = false;
            memset(regs, 0, sizeof(regs));
            memset(disp, 0, sizeof(disp));
            off_pending = false;
        }
        if (clearing && now >= clear_end) {
            clearing = false;
            memset(disp, 0, sizeof(disp));
            regs[reg::DMM] &= uint8_t(~DMM_CLEAR_DISPLAY);
        }
        if (off_pending && now >= off_at) off_pending = false;
    }
    void wr(uint8_t r, uint8_t v, uint64_t now)
    {
        advance(now);
        if (resetting) { ignored++; return; }
        switch (r) {
        case reg::VM0:
            if (busy(now) || clearing) { ignored++; return; }
            if (v & VM0_SOFT_RESET) {
                resetting = true;
                reset_end = now + reset_ns;
                regs[reg::VM0] = VM0_SOFT_RESET;
                return;
            }
            if ((regs[reg::VM0] & VM0_OSD_ENABLE) && !(v & VM0_OSD_ENABLE)) {
                off_pending = true;
                off_at = now + display_off_ns;
            }
            regs[reg::VM0] = v;
            return;
        case reg::CMAH: case reg::CMAL: case reg::CMDI:
            if (busy(now) || osd_on(now)) { ignored++; return; }
            regs[r] = v;
            if (r == reg::CMDI) shadow[regs[reg::CMAL] & 0x3F] = v;
            return;
        case reg::CMM:
            if (busy(now) || osd_on(now)) { ignored++; return; }
            if ((v & 0xF0) == CMM_READ_NVR) {
                op = NVM_READ; op_end = now + nvm_read_ns; op_char = cmchar(); nvm_reads++;
            } else if ((v & 0xF0) == CMM_WRITE_NVR) {
                op = NVM_WRITE; op_end = now + nvm_write_ns; op_char = cmchar();
            }
            regs[reg::CMM] = v;
            return;
        case reg::DMM:
            if (clearing) { ignored++; return; }
            regs[reg::DMM] = v;
            if (v & DMM_CLEAR_DISPLAY) { clearing = true; clear_end = now + clear_ns; }
            if (v & DMM_AUTOINCREMENT) cursor = addr();
            return;
        case reg::DMAH: case reg::DMAL:
            if (clearing) { ignored++; return; }
            regs[r] = v;
            return;
        case reg::DMDI:
            if (clearing) { ignored++; return; }
            if (regs[reg::DMM] & DMM_AUTOINCREMENT) {
                if (v == DMDI_AUTOINC_END) {
                    regs[reg::DMM] &= uint8_t(~DMM_AUTOINCREMENT);
                } else {
                    disp[cursor & 511u] = v;
                    cursor++;
                }
            } else {
                disp[addr()] = v;
            }
            return;
        default:
            regs[r & 0x7F] = v;
            return;
        }
    }
    uint8_t rd(uint8_t a, uint64_t now)
    {
        advance(now);
        if ((a & 0xF0) == reg::STAT) {
            return uint8_t(video | (busy(now) ? STAT_NVR_BUSY : 0) | (resetting ? STAT_RESET_BUSY : 0));
        }
        if ((a & 0xF0) == reg::DMDO) return disp[addr()];
        if ((a & 0xF0) == reg::CMDO) {
            if (busy(now)) stale_cmdo++;
            return shadow[regs[reg::CMAL] & 0x3F];
        }
        return regs[a & 0x7F];
    }
    void process(const uint8_t* tx, uint8_t* rx, size_t len)
    {
        transfers++;
        bytes += uint32_t(len);
        if (sclk_hz > 1000000000u / timing::TCP_MIN_NS) timing_violations++;
        const uint64_t b = byte_ns();
        const uint64_t t0 = t + cs_setup_ns;
        for (size_t i = 0; i + 1 < len; i += 2) {
            const uint8_t a = tx[i], d = tx[i + 1];
            if (!present) {
                if (rx) { rx[i] = 0xFF; rx[i + 1] = 0xFF; }
                continue;
            }
            if (a & reg::READ) {
                const uint8_t v = rd(a, t0 + (i + 1) * b);
                if (rx) { rx[i] = 0; rx[i + 1] = v; }
            } else {
                wr(a, d, t0 + (i + 2) * b);
                if (rx) { rx[i] = 0; rx[i + 1] = 0; }
            }
        }
        t = t0 + len * b + cs_gap_ns;
    }
    bool transfer(const uint8_t* tx, size_t len) override { process(tx, nullptr, len); return true; }
    bool transfer_fullduplex(uint8_t* buf, size_t len) override { process(buf, buf, len); return true; }
    void delay_us(uint32_t us) override { t += uint64_t(us) * 1000u; }

    void program(const uint8_t* font)
    {
        for (int c = 0; c < 256; c++) memcpy(nvm[c], font + c * CHAR_BYTES, CHAR_BYTES);
    }
    bool nvm_equals(const uint8_t* font) const
    {
        for (int c = 0; c < 256; c++) if (memcmp(nvm[c], font + c * CHAR_BYTES, CHAR_BYTES) != 0) return false;
        return true;
    }
    uint32_t total_wear() const { uint32_t s = 0; for (int c = 0; c < 512; c++) s += wear[c]; return s; }
};

// ---------------------------------------------------------------------------
// Fonts
// ---------------------------------------------------------------------------
static uint8_t g_font_distinct[FONT_BYTES];
static uint8_t g_font_border[FONT_BYTES];

static void make_fonts()
{
    // distinct: byte 0 of char k is odd, byte 53 of char k-1 is even, so the
    // stale value upstream reads first can never match.
    for (uint32_t c = 0; c < 256; c++) {
        uint8_t* d = g_font_distinct + c * CHAR_BYTES;
        for (uint32_t x = 0; x < CHAR_BYTES; x++) d[x] = uint8_t(rnd());
        d[0] = uint8_t(2u * c + 1u);
        d[53] = uint8_t(2u * c + 2u);
    }
    // border: like real OSD fonts (font0.bin) - top and bottom rows transparent
    // (0x55, DS p.29 CMDI "01 = transparent") except glyphs that touch the top.
    for (uint32_t c = 0; c < 256; c++) {
        uint8_t* d = g_font_border + c * CHAR_BYTES;
        for (uint32_t x = 0; x < CHAR_BYTES; x++) d[x] = (x < 6 || x >= 48) ? 0x55 : uint8_t(rnd());
        if (c % 7 == 3) d[1] = 0x95;      // glyph pixel in row 0
        if (c % 11 == 5) d[0] = 0x65;
    }
}

// Independent timeline model of upstream's single burst (no simulator code):
// returns the number of characters whose check fails when NVM already holds
// `font`, for SCLK sclk and read latency tread.
static int predict_upstream_rewrites(const uint8_t* font, uint32_t sclk, uint32_t tread, uint32_t cs_setup)
{
    const double b = 8e9 / double(sclk);
    int bad = 0;
    for (int k = 0; k < 256; k++) {
        const uint8_t* cur = font + k * CHAR_BYTES;
        uint8_t prev_shadow[64] = {0};
        if (k > 0) memcpy(prev_shadow, font + (k - 1) * CHAR_BYTES, CHAR_BYTES);
        int cmal = k > 0 ? 53 : 0;
        const double done = cs_setup + 6 * b + tread;
        for (int x = 0; x < CHAR_BYTES; x++) {
            const double tw = cs_setup + (6 + 4 * x + 2) * b;
            if (tw >= done) cmal = x;
            const double tr = cs_setup + (6 + 4 * x + 3) * b;
            const uint8_t v = tr >= done ? cur[cmal] : prev_shadow[cmal];
            if (v != cur[x]) { bad++; break; }
        }
    }
    return bad;
}

// ---------------------------------------------------------------------------
static void test_constants()
{
    std::printf("T1 register map, VM1/RB values, STAT decode truth table (AP :34-116)\n");
    CHECK(reg::STAT == 0xA0 && reg::CMDO == 0xC0 && reg::DMDO == 0xB0 && reg::OSDBL == 0x6C, "read addresses");
    CHECK(reg::CMM == 0x08 && reg::CMAH == 0x09 && reg::CMAL == 0x0A && reg::CMDI == 0x0B, "char memory regs");
    CHECK(VM1_VALUE == 0x4C && RB_VALUE == 0x01, "VM1 0x%02X RB 0x%02X", VM1_VALUE, RB_VALUE);
    CHECK(FONT_BYTES == 13824 && FRAME_BUDGET == 480 && FRAME_MAX_BYTES == 494, "sizes");
    int pal = 0, ntsc_alt = 0;
    for (uint8_t s = 0; s < 8; s++) { pal += vin_is_pal(s); ntsc_alt += vin_is_ntsc_alt(s); }
    CHECK(pal == 2 && ntsc_alt == 2, "truth table pal %d ntsc_alt %d", pal, ntsc_alt);
    CHECK(vin_is_ntsc_alt(0x00) && !vin_is_pal(0x00) && vin_is_ntsc_alt(0x02) && !vin_is_ntsc_alt(0x04),
          "STAT 0 = NTSC quirk, LOS = neither");
}

static void test_spi_clock()
{
    std::printf("T2 SPI2 clock (SPIDevice.cpp:259-280), DS p.4 tCP/tCH/tCL, upstream NVM gap\n");
    const SpiClock c8 = derive_spi_clock(80000000u, 8000000u);
    const SpiClock c10 = derive_spi_clock(80000000u, 10000000u);
    const SpiClock c999 = derive_spi_clock(80000000u, 9990000u);
    const SpiClock c40 = derive_spi_clock(80000000u, 40000000u);
    const SpiClock clow = derive_spi_clock(80000000u, 100000u);
    std::printf("      8*MHZ -> %u Hz (MBR %u, /%u) tCP %.0f ns margin %.0f ns; 10*MHZ -> %u Hz margin %.0f ns\n",
                unsigned(c8.sclk_hz), unsigned(c8.mbr), 2u << c8.mbr, double(c8.tcp_ns), double(c8.margin_tcp_ns),
                unsigned(c10.sclk_hz), double(c10.margin_tcp_ns));
    CHECK(c8.sclk_hz == 5000000u && c8.mbr == 3 && c8.ok && !c8.zero_margin, "8 MHz request");
    CHECK(c8.tch_ns >= 2.0f * float(timing::TCH_MIN_NS) && c8.margin_tcp_ns >= 100.0f, "5 MHz margins");
    CHECK(c10.sclk_hz == 10000000u && c10.mbr == 2 && c10.ok && c10.zero_margin, "10 MHz at the limit");
    CHECK(c999.sclk_hz == 5000000u, "9.99 MHz -> 5 MHz");
    CHECK(c40.sclk_hz == 40000000u && !c40.ok, "40 MHz violates tCP");
    CHECK(clow.mbr == 7 && clow.sclk_hz == 312500u, "MBR capped at 7");
    const uint32_t g5 = upstream_nvm_read_gap_ns(5000000u), g10 = upstream_nvm_read_gap_ns(10000000u);
    std::printf("      upstream CMM -> first CMDO: %.1f us @5 MHz, %.1f us @10 MHz (AT7456E needs >= 22 us)\n",
                double(g5) / 1000.0, double(g10) / 1000.0);
    CHECK(g5 == 4800u && g10 == 2400u, "gap");
    CHECK(g5 < timing::NVM_READ_MIN_NS && g5 > timing::MAX7456_NVM_READ_NS, "fails AT7456E, passes MAX7456");
    const double frame_us = double(FRAME_MAX_BYTES) * 8.0 / 5.0;
    std::printf("      worst flush %u bytes = %.1f us @5 MHz (%.2f %% of the 100 ms OSD period)\n",
                unsigned(FRAME_MAX_BYTES), frame_us, frame_us / 1000.0);
    CHECK(frame_us < 1000.0, "flush < 1 ms");
}

static void test_probe()
{
    std::printf("T3 probe: VM0 soft reset (40 us typ) + 1 ms wait, too-short wait, absent device\n");
    SimAt7456e s;
    At7456e d(s);
    CHECK(d.probe(), "present");
    SimAt7456e s2;
    Config c;
    c.probe_wait_us = 20;
    At7456e d2(s2, c);
    CHECK(!d2.probe(), "20 us wait < 28 us min reset: VM0 still 0x02");
    SimAt7456e s3;
    s3.present = false;
    At7456e d3(s3);
    CHECK(!d3.probe(), "absent (MISO 0xFF)");
}

static void test_upstream_vs_delayed()
{
    std::printf("T4 NVM read latency: upstream single burst vs delayed sequence (font already in NVM)\n");
    const uint32_t clocks[2] = {5000000u, 10000000u};
    const uint32_t lat[2] = {timing::NVM_READ_MIN_NS, timing::NVM_READ_TYP_NS};
    for (int ci = 0; ci < 2; ci++) {
        for (int li = 0; li < 2; li++) {
            SimAt7456e su;
            su.sclk_hz = clocks[ci];
            su.nvm_read_ns = lat[li];
            su.program(g_font_distinct);
            At7456e up(su, Config::upstream());
            FontStats fu;
            const bool oku = up.update_font(g_font_distinct, FONT_BYTES, &fu);
            SimAt7456e sd;
            sd.sclk_hz = clocks[ci];
            sd.nvm_read_ns = lat[li];
            sd.program(g_font_distinct);
            At7456e dl(sd, Config::robust());
            FontStats fd;
            const bool okd = dl.update_font(g_font_distinct, FONT_BYTES, &fd);
            std::printf("      %2u MHz, tNVR %2u us: upstream rewrote %3u (stale CMDO %5u, NVM wear %3u, %.2f s); "
                        "delayed rewrote %u (stale %u, %.1f ms)\n", unsigned(clocks[ci] / 1000000u),
                        unsigned(lat[li] / 1000u), unsigned(fu.rewritten), unsigned(su.stale_cmdo),
                        unsigned(su.total_wear()), double(su.t) * 1e-9, unsigned(fd.rewritten), unsigned(sd.stale_cmdo),
                        double(sd.t) * 1e-6);
            CHECK(oku && fu.rewritten == 256 && su.total_wear() == 256 && su.stale_cmdo > 0, "upstream re-flashes all 256");
            CHECK(double(su.t) * 1e-9 > 256 * 0.015, "upstream boot >= 3.84 s");
            CHECK(okd && fd.rewritten == 0 && sd.total_wear() == 0 && sd.stale_cmdo == 0, "delayed: zero rewrites");
            CHECK(su.nvm_equals(g_font_distinct) && sd.nvm_equals(g_font_distinct), "NVM intact");
            CHECK(double(sd.t) * 1e-6 < 50.0, "delayed check of 256 chars < 50 ms (%.1f)", double(sd.t) * 1e-6);
        }
    }
}

static void test_font_dependence()
{
    std::printf("T5 upstream penalty is font-dependent (stale window vs transparent glyph borders)\n");
    int n5 = -1, n10 = -1;
    for (int ci = 0; ci < 2; ci++) {
        const uint32_t clk = ci ? 10000000u : 5000000u;
        SimAt7456e s;
        s.sclk_hz = clk;
        s.program(g_font_border);
        At7456e up(s, Config::upstream());
        FontStats f;
        up.update_font(g_font_border, FONT_BYTES, &f);
        const int pred = predict_upstream_rewrites(g_font_border, clk, timing::NVM_READ_TYP_NS, s.cs_setup_ns);
        std::printf("      border font @%u MHz: upstream rewrote %u, independent timeline model %d\n",
                    unsigned(clk / 1000000u), unsigned(f.rewritten), pred);
        CHECK(int(f.rewritten) == pred && pred > 0 && pred < 256, "sim == model");
        (ci ? n10 : n5) = int(f.rewritten);
    }
    CHECK(n10 >= n5, "faster SCLK -> longer stale window (%d vs %d)", n10, n5);
    CHECK(predict_upstream_rewrites(g_font_distinct, 5000000u, timing::NVM_READ_MIN_NS, 100) == 256, "model: distinct = 256");
    // informational: the real ArduPilot font0.bin if the reference tree is present
    const char* path = "/tmp/nano_rev11_imu_wt/autopilot/libraries/AP_OSD/fonts/font0.bin";
    static uint8_t real[FONT_BYTES];
    FILE* fp = std::fopen(path, "rb");
    if (fp != nullptr) {
        const size_t n = std::fread(real, 1, sizeof(real), fp);
        std::fclose(fp);
        if (n == FONT_BYTES) {
            std::printf("      (info) AP font0.bin, upstream rewrites per boot: 5 MHz %d/%d, 10 MHz %d/%d (tNVR 22/30 us)\n",
                        predict_upstream_rewrites(real, 5000000u, 22000u, 100), predict_upstream_rewrites(real, 5000000u, 30000u, 100),
                        predict_upstream_rewrites(real, 10000000u, 22000u, 100), predict_upstream_rewrites(real, 10000000u, 30000u, 100));
        }
    } else {
        std::printf("      (info) font0.bin not found, real-font figure skipped\n");
    }
}

static void test_latency_sweep()
{
    std::printf("T6 latency sweep: MAX7456 0.5 us, out-of-spec 60 us part, STAT[5] poll\n");
    {
        SimAt7456e s;
        s.nvm_read_ns = timing::MAX7456_NVM_READ_NS;
        s.program(g_font_distinct);
        At7456e up(s, Config::upstream());
        FontStats f;
        up.update_font(g_font_distinct, FONT_BYTES, &f);
        CHECK(f.rewritten == 0, "MAX7456 timing: upstream fine (%u)", unsigned(f.rewritten));
    }
    {
        SimAt7456e s;
        s.nvm_read_ns = 60000;
        s.program(g_font_distinct);
        At7456e d(s);
        FontStats f;
        d.update_font(g_font_distinct, FONT_BYTES, &f);
        CHECK(f.rewritten == 0 && f.busy_waits > 0, "60 us part: poll absorbs (%u waits)", unsigned(f.busy_waits));
        SimAt7456e s2;
        s2.nvm_read_ns = 60000;
        s2.program(g_font_distinct);
        Config c;
        c.nvm_read_busy_polls = 0;
        At7456e d2(s2, c);
        FontStats f2;
        d2.update_font(g_font_distinct, FONT_BYTES, &f2);
        CHECK(f2.verify_failed > 0 || f2.rewritten > 0, "fixed delay alone fails on a 60 us part");
    }
}

static void test_display_enabled()
{
    std::printf("T7 font check with the OSD enabled (flush-time font change): VM0[3] off latency\n");
    for (int mode = 0; mode < 2; mode++) {
        SimAt7456e s;
        s.program(g_font_distinct);
        s.regs[reg::VM0] = VM0_MODE_PAL | VM0_OSD_ENABLE;
        // previous shadow content from another char
        memcpy(s.shadow, s.nvm[200], 64);
        At7456e d(s, mode ? Config::robust() : Config::upstream());
        const bool ok = d.check_font_char(100, g_font_distinct + 100 * CHAR_BYTES);
        std::printf("      %s: check %s, ignored commands %u\n", mode ? "delayed " : "upstream", ok ? "match" : "MISMATCH",
                    unsigned(s.ignored));
        if (mode) {
            CHECK(ok && s.ignored == 0, "robust waits for display off");
        } else {
            CHECK(!ok && s.ignored >= 2, "upstream CMAH/CMM ignored while display still on");
        }
    }
}

static void test_programming()
{
    std::printf("T8 blank NVM programming, verify, endurance, write timeout, bad size\n");
    SimAt7456e s;
    At7456e d(s);
    FontStats f;
    CHECK(d.update_font(g_font_distinct, FONT_BYTES, &f) && f.rewritten == 256 && f.verify_failed == 0, "program 256");
    CHECK(s.nvm_equals(g_font_distinct), "NVM == font");
    uint32_t maxw = 0;
    for (int c = 0; c < 256; c++) if (s.wear[c] > maxw) maxw = s.wear[c];
    CHECK(maxw == 1, "one write per char (max %u)", unsigned(maxw));
    std::printf("      first boot: %u writes, %u STAT polls, %.2f s\n", unsigned(f.rewritten), unsigned(f.nvm_write_polls),
                double(s.t) * 1e-9);
    FontStats f2;
    CHECK(d.update_font(g_font_distinct, FONT_BYTES, &f2) && f2.rewritten == 0 && s.total_wear() == 256, "second boot");
    SimAt7456e st;
    st.nvm_write_ns = 2000000000u;    // stuck busy
    At7456e ds(st);
    FontStats f3;
    const uint64_t t0 = st.t;
    CHECK(!ds.update_font_char(7, g_font_distinct + 7 * CHAR_BYTES, &f3) && f3.write_timeouts == 1, "timeout");
    CHECK(double(st.t - t0) * 1e-9 < 0.06, "bounded %.3f s (upstream bound 150 s)", double(st.t - t0) * 1e-9);
    CHECK(!d.update_font(g_font_distinct, FONT_BYTES - 1, nullptr), "bad font size rejected (AP :170)");
}

struct OsdRig {
    SimAt7456e s;
    At7456e d;
    uint32_t now = 0;
    explicit OsdRig(const Config& c = Config()) : d(s, c) { d.probe(); }
    void run_to(uint32_t t_ms)
    {
        while (now < t_ms) {
            now += 100;
            d.flush(now);
        }
    }
};

static void test_format()
{
    std::printf("T9 PAL/NTSC/LOS mirror of check_reinit()/reinit() incl. quirks\n");
    OsdRig r;
    r.run_to(1400);
    CHECK(!r.d.initialized() && r.d.frame_stats().bytes_max == 0, "no reinit before 1500 ms");
    r.run_to(1500);
    CHECK(r.d.initialized() && r.d.format() == VideoFormat::PAL && r.d.rows() == ROWS_PAL, "PAL at 1500 ms");
    CHECK(r.s.regs[reg::VM0] == 0x48 && r.s.regs[reg::VM1] == 0x4C && r.s.regs[reg::RB0 + 15] == 0x01, "registers");
    // PAL -> NTSC: STAT polled only every > 1000 ms, debounce needs a second poll
    r.run_to(5000);
    r.s.video = STAT_NTSC;
    const uint32_t t_sw = r.now;
    while (r.d.format() == VideoFormat::PAL && r.now < t_sw + 5000) r.run_to(r.now + 100);
    const uint32_t lat = r.now - t_sw;
    std::printf("      PAL -> NTSC switch latency %u ms (nominal debounce 100 ms)\n", unsigned(lat));
    CHECK(r.d.format() == VideoFormat::NTSC && r.d.rows() == ROWS_NTSC && r.s.regs[reg::VM0] == 0x08, "NTSC");
    CHECK(lat > 1000 && lat <= 2300, "effective debounce 1..2.3 s: %u", unsigned(lat));
    // LOS: no switch, ever
    r.s.video = STAT_LOS;
    const uint32_t reinits = r.d.frame_stats().reinits;
    r.run_to(r.now + 10000);
    CHECK(r.d.format() == VideoFormat::NTSC && r.d.frame_stats().reinits == reinits, "LOS holds format");
    // STAT == 0 (no LOS, no PAL, no NTSC) is NTSC (AB7456 quirk): from PAL it switches
    OsdRig q;
    q.run_to(3000);
    q.s.video = 0x00;
    q.run_to(q.now + 3000);
    CHECK(q.d.format() == VideoFormat::NTSC, "STAT 0 -> NTSC");
    // LOS at reinit -> NTSC (only VIN_IS_PAL selects PAL)
    OsdRig l;
    l.s.video = STAT_LOS;
    l.run_to(1600);
    CHECK(l.d.initialized() && l.d.format() == VideoFormat::NTSC, "LOS at boot -> NTSC");
    // stale debounce timestamp: one glitch arms video_detect_time forever
    OsdRig g;
    g.run_to(3000);
    g.s.video = STAT_NTSC;
    g.run_to(g.now + 1100);              // exactly one STAT poll sees NTSC
    g.s.video = STAT_PAL;
    const uint32_t r0 = g.d.frame_stats().reinits;
    g.run_to(g.now + 20000);
    CHECK(g.d.format() == VideoFormat::PAL && g.d.frame_stats().reinits == r0, "glitch alone: no switch");
    g.s.video = STAT_NTSC;
    const uint32_t t1 = g.now;
    while (g.d.format() == VideoFormat::PAL && g.now < t1 + 3000) g.run_to(g.now + 100);
    std::printf("      second single glitch 20 s later switches after %u ms (stale video_detect_time, no debounce)\n",
                unsigned(g.now - t1));
    CHECK(g.d.format() == VideoFormat::NTSC && g.now - t1 <= 1100, "quirk mirrored");
    // VM0 read-back mismatch (OSD brown-out / reset) -> immediate reinit
    OsdRig b;
    b.run_to(3000);
    b.s.regs[reg::VM0] = 0;
    const uint32_t rb = b.d.frame_stats().reinits;
    b.run_to(b.now + 100);
    CHECK(b.d.frame_stats().reinits == rb + 1 && b.s.regs[reg::VM0] == 0x48, "VM0 stall check");
}

static bool display_matches(const OsdRig& r)
{
    for (uint8_t y = 0; y < r.d.rows(); y++)
        for (uint8_t x = 0; x < COLUMNS; x++)
            if (r.s.disp[y * COLUMNS + x] != r.d.frame_char(x, y)) return false;
    return true;
}

static int converge(OsdRig& r, uint32_t* max_bytes)
{
    int n = 0;
    for (int i = 0; i < 20; i++) {
        r.now += 100;
        r.d.flush(r.now);
        if (r.d.frame_stats().bytes_last == 0) break;
        if (max_bytes && r.d.frame_stats().bytes_last > *max_bytes) *max_bytes = r.d.frame_stats().bytes_last;
        n++;
    }
    return n;
}

static void test_frame()
{
    std::printf("T10 frame diff, 480-byte budget, auto-increment runs, 0xFF, NTSC rows, clear race\n");
    OsdRig r;
    r.run_to(1500);
    uint32_t mb = 0;
    const int n0 = converge(r, &mb);
    CHECK(display_matches(r) && n0 <= 3, "initial full redraw in %d flushes", n0);
    r.d.write(3, 2, "HELLO");
    r.now += 100;
    r.d.flush(r.now);
    CHECK(r.d.frame_stats().bytes_last == 20 && display_matches(r), "5-char run = 20 bytes (%u)",
          unsigned(r.d.frame_stats().bytes_last));
    r.now += 100;
    r.d.flush(r.now);
    CHECK(r.d.frame_stats().bytes_last == 0, "nothing dirty -> no transfer");
    // full random redraw
    for (uint8_t y = 0; y < ROWS_PAL; y++)
        for (uint8_t x = 0; x < COLUMNS; x++) r.d.set_char(x, y, uint8_t(1 + rnd() % 250));
    mb = 0;
    const int n1 = converge(r, &mb);
    std::printf("      full PAL redraw: %d flushes, max %u bytes/flush\n", n1, unsigned(mb));
    CHECK(n1 == 3 && mb <= FRAME_MAX_BYTES && display_matches(r), "full redraw");
    // alternating (no runs): 6 bytes per char
    for (uint8_t y = 0; y < ROWS_PAL; y++)
        for (uint8_t x = 0; x < COLUMNS; x += 2) r.d.set_char(x, y, uint8_t(r.d.frame_char(x, y) ^ 0x01));
    r.now += 100;
    r.d.flush(r.now);
    std::printf("      checkerboard: %u chars in %u bytes\n", unsigned(r.d.frame_stats().chars_last),
                unsigned(r.d.frame_stats().bytes_last));
    CHECK(r.d.frame_stats().chars_last == 80 && r.d.frame_stats().bytes_last == 480, "80 chars x 6 bytes");
    const int n2 = converge(r, nullptr);
    CHECK(n2 == 2 && display_matches(r), "remaining 160 in 2 more flushes");
    // random soak
    uint32_t worst = 0;
    bool all_ok = true;
    for (int it = 0; it < 300; it++) {
        const int k = int(rnd() % 200);
        for (int j = 0; j < k; j++) r.d.set_char(uint8_t(rnd() % COLUMNS), uint8_t(rnd() % ROWS_PAL), uint8_t(rnd() % 256));
        converge(r, &worst);
        all_ok = all_ok && display_matches(r);
    }
    std::printf("      300 random frames (incl. 0xFF): worst %u bytes, buffer peak %u / %u\n", unsigned(worst),
                unsigned(r.d.last_buffer_peak()), unsigned(SPI_BUFFER));
    CHECK(all_ok, "display == frame after every convergence");
    CHECK(worst <= FRAME_MAX_BYTES && r.d.last_buffer_peak() <= SPI_BUFFER, "budget");
    // 0xFF inside a run: upstream mirror corrupts, robust does not
    for (int mode = 0; mode < 2; mode++) {
        OsdRig u(mode ? Config::robust() : Config::upstream());
        u.run_to(1500);
        converge(u, nullptr);
        u.d.write(0, 5, "ABCDEFGH");
        u.d.set_char(3, 5, 0xFF);
        converge(u, nullptr);
        if (mode) {
            CHECK(display_matches(u), "robust: 0xFF written outside auto-increment");
        } else {
            CHECK(!display_matches(u), "upstream: 0xFF ends the run, rest of the run lands on one address");
        }
    }
    // NTSC: only 13 rows ever written
    OsdRig nt;
    nt.s.video = STAT_NTSC;
    nt.run_to(1500);
    for (uint8_t y = 0; y < ROWS_PAL; y++) nt.d.write(0, y, "NTSCNTSCNTSCNTSCNTSCNTSCNTSCNT");
    converge(nt, nullptr);
    bool beyond = false;
    for (int p = ROWS_NTSC * COLUMNS; p < ROWS_PAL * COLUMNS; p++) beyond = beyond || nt.s.disp[p] != 0;
    CHECK(nt.d.rows() == ROWS_NTSC && !beyond && display_matches(nt), "NTSC 13 rows");
    // DMM clear race right after reinit
    for (int mode = 0; mode < 2; mode++) {
        OsdRig c(mode ? Config::robust() : Config::upstream());
        c.d.write(0, 0, "ARDUPILOT");
        c.run_to(1500);
        const uint32_t ign = c.s.ignored;
        converge(c, nullptr);
        std::printf("      %s: writes ignored during DMM clear %u, display %s\n", mode ? "robust  " : "upstream",
                    unsigned(ign), display_matches(c) ? "correct" : "MISSING CHARS");
        if (mode) {
            CHECK(ign == 0 && display_matches(c) && c.d.frame_stats().clear_polls == 0, "robust waits for the clear");
        } else {
            CHECK(ign > 0 && !display_matches(c), "upstream loses the first characters (shadow already updated)");
        }
    }
}

static void test_timing_violation()
{
    std::printf("T11 simulator flags SCLK above 10 MHz\n");
    SimAt7456e s;
    s.sclk_hz = 20000000u;
    At7456e d(s);
    d.probe();
    CHECK(s.timing_violations > 0, "20 MHz flagged");
    SimAt7456e s5;
    At7456e d5(s5);
    d5.probe();
    CHECK(s5.timing_violations == 0, "5 MHz clean");
}

int main()
{
    std::printf("Nano H743 AIO AT7456E OSD driver tests (Rev 13.0)\n");
    make_fonts();
    test_constants();
    test_spi_clock();
    test_probe();
    test_upstream_vs_delayed();
    test_font_dependence();
    test_latency_sweep();
    test_display_enabled();
    test_programming();
    test_format();
    test_frame();
    test_timing_violation();
    std::printf("\nRESULT: %s (%d checks, %d failures)\n", g_fail ? "FAIL" : "PASS", g_pass + g_fail, g_fail);
    return g_fail ? 1 : 0;
}
