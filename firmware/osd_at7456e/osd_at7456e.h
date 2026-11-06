// Nano H743 AIO - AT7456E analog OSD register-level driver reference (Revision 13.0)
// Sreesai Arjun Engineering
//
// Host-testable reference for U5 (Hangzhou Zhongke AT7456E, HTSSOP-28-EP,
// LCSC C82351) on SPI2: PB13 SCK -> R30 33R, PB15 MOSI -> R31 33R, PB14 MISO
// <- R33 33R, PB12 OSD1_CS -> R32 33R. hwdef "SPIDEV osd SPI2 DEVID4 OSD1_CS
// MODE0 8*MHZ 8*MHZ": SPI2 kernel clock PLL1_Q = 80 MHz, so ArduPilot's divider
// search gives 80 / 16 = 5.000 MHz (tCP 200 ns vs 100 ns min, DS p.4).
// All supplies on +3V3_OSD (FB5), /RESET pull-up only (R29 10k), 27 MHz Y3.
//
// What it reproduces from ArduPilot AP_OSD_MAX7456.cpp (line numbers of the
// unmodified file in the reference tree):
//   - register map and bit values (:34-116), VM1 = 50/50 blink, blink time 3,
//     28 % background = 0x4C (:67-86, :341), row brightness 0x01 (:126-132);
//   - init(): VM0 = 0x02 soft reset, 1 ms, VM0 must read 0x00 (:143-156);
//   - update_font(): 13824-byte font, per char check then rewrite (:158-190);
//     check_font_char(): ONE full-duplex burst VM0=0, CMAH, CMM=0x50, then
//     54 x (CMAL, CMDO) with data at rx[9 + 4x] (:193-218);
//     update_font_char(): one write burst + STAT[5] poll every 15 ms, up to
//     10000 polls (:220-247, MAX_NVM_WAIT :123);
//   - check_reinit()/reinit() incl. the quirks (:275-356): VM0 read-back on
//     every flush, STAT once per > 1000 ms, "!LOS && !PAL" = NTSC
//     (VIN_IS_NTSC_ALT :101-105), 100 ms debounce timer that is only evaluated
//     at the 1 s checks and is never cleared except by LOS, no reinit before
//     millis() 1500, reinit selects PAL only on VIN_IS_PAL (LOS -> NTSC);
//   - transfer_frame() (:384-441): shadow-frame diff, DMAH/DMAL + DMM
//     auto-increment runs exited with DMDI=0xFF, DMM=0, 512-byte buffer and the
//     `buffer_offset >= spi_buffer_size - 32` (480-byte) budget whose `break`
//     only leaves the column loop.
// What it adds (Config::robust(), see test_osd_at7456e.cpp T4-T8, T10):
//   - AT7456E NVM read latency: CMM=0x50 needs >= 22 us (typ 30 us) before the
//     shadow RAM is valid (DS p.36 table, p.29 CMM, p.34 "该过程需要30us");
//     MAX7456 needs 0.5 us. Upstream's first CMDO is 3 bytes after CMM
//     (4.8 us at 5 MHz), and CMAL writes while STAT[5]=1 are ignored (DS p.29),
//     so the first CMDO reads return stale shadow RAM and the char is
//     re-flashed. Fix: separate transfer, 40 us delay, STAT[5] poll, then read.
//   - VM0[3]=0 takes 0..18 us (typ 30 us) to take effect and CMM is ignored
//     while the display is enabled (DS p.36, p.29): 40 us after VM0=0.
//   - read-back verify after every NVM write, bounded 1 ms STAT[5] poll
//     (50 ms cap instead of 10000 x 15 ms = 150 s).
//   - after DMM[2]=1 (clear, 40 us typ, DS p.34/p.36) wait and poll DMM[2]=0
//     before the first frame write: display-memory writes during the clear are
//     ignored (DS p.21 note 2), and upstream starts transfer_frame() right
//     after HOS/VOS, with the shadow already updated -> characters missing
//     until they next change.
//   - a frame character 0xFF is written outside auto-increment mode
//     (DS p.35: CA = 0xFF terminates auto-increment; upstream would drop the
//     character and smear the rest of the run onto one address).
//
// Allocation-free, no exceptions, fixed-size state.
#pragma once

#include <stdint.h>
#include <stddef.h>

namespace nano_osd {

// ---------------------------------------------------------------------------
// AT7456E register map (AT7456E-S402-V1.1 p.21 register table; identical
// addresses to MAX7456, AP_OSD_MAX7456.cpp:34-51)
// ---------------------------------------------------------------------------
namespace reg {
static constexpr uint8_t VM0   = 0x00;
static constexpr uint8_t VM1   = 0x01;
static constexpr uint8_t HOS   = 0x02;
static constexpr uint8_t VOS   = 0x03;
static constexpr uint8_t DMM   = 0x04;
static constexpr uint8_t DMAH  = 0x05;
static constexpr uint8_t DMAL  = 0x06;
static constexpr uint8_t DMDI  = 0x07;
static constexpr uint8_t CMM   = 0x08;
static constexpr uint8_t CMAH  = 0x09;
static constexpr uint8_t CMAL  = 0x0A;
static constexpr uint8_t CMDI  = 0x0B;
static constexpr uint8_t OSDM  = 0x0C;
static constexpr uint8_t RB0   = 0x10;   // RB0..RB15 = 0x10..0x1F
static constexpr uint8_t OSDBL = 0x6C;
static constexpr uint8_t READ  = 0x80;   // read address = write address | 0x80
static constexpr uint8_t STAT  = 0xA0;   // read only (AxH)
static constexpr uint8_t DMDO  = 0xB0;   // read only (BxH)
static constexpr uint8_t CMDO  = 0xC0;   // read only (CxH)
}
// VM0 (DS p.22, AP :54-60)
static constexpr uint8_t VM0_VIDEO_BUFFER_DISABLE = 0x01;
static constexpr uint8_t VM0_SOFT_RESET  = 0x02;
static constexpr uint8_t VM0_VSYNC_NEXT  = 0x04;
static constexpr uint8_t VM0_OSD_ENABLE  = 0x08;
static constexpr uint8_t VM0_MODE_PAL    = 0x40;
static constexpr uint8_t VM0_MODE_NTSC   = 0x00;
static constexpr uint8_t VM0_MODE_MASK   = 0x40;
// VM1 = BLINK_DUTY_CYCLE_50_50 | BLINK_TIME_3 | BACKGROUND_BRIGHTNESS_28 (AP :67-86, :341)
static constexpr uint8_t VM1_VALUE       = 0x00 | 0x0C | (0x04 << 4);
// RBn = (BLACKBRIGHTNESS << 2) | WHITEBRIGHTNESS (AP :126-132)
static constexpr uint8_t RB_VALUE        = (0x00 << 2) | 0x01;
// STAT (DS p.32, AP :89-92)
static constexpr uint8_t STAT_PAL        = 0x01;
static constexpr uint8_t STAT_NTSC       = 0x02;
static constexpr uint8_t STAT_LOS        = 0x04;
static constexpr uint8_t STAT_NVR_BUSY   = 0x20;
static constexpr uint8_t STAT_RESET_BUSY = 0x40;
// CMM (DS p.28-29, AP :108-109)
static constexpr uint8_t CMM_WRITE_NVR   = 0xA0;
static constexpr uint8_t CMM_READ_NVR    = 0x50;
// DMM (DS p.25, AP :112-116)
static constexpr uint8_t DMM_AUTOINCREMENT = 0x01;
static constexpr uint8_t DMM_CLEAR_DISPLAY = 0x04;
static constexpr uint8_t DMDI_AUTOINC_END  = 0xFF;   // DS p.35 step 6

// geometry (AP_OSD_MAX7456.h:91-94, .cpp:30-31)
static constexpr uint8_t  COLUMNS      = 30;
static constexpr uint8_t  ROWS_PAL     = 16;
static constexpr uint8_t  ROWS_NTSC    = 13;
static constexpr uint16_t CHAR_BYTES   = 54;     // 12 x 18 px, 2 bit/px (NVM_RAM_SIZE)
static constexpr uint16_t FONT_CHARS   = 256;
static constexpr uint32_t FONT_BYTES   = uint32_t(CHAR_BYTES) * FONT_CHARS;   // 13824
static constexpr uint16_t SPI_BUFFER   = 512;    // spi_buffer_size
static constexpr uint16_t FRAME_BUDGET = SPI_BUFFER - 32;   // 480: stop adding chars at this offset
// worst-case flush size: last check at offset 478, then exit run (4) +
// DMAH/DMAL (4) + DMM (2) + DMDI (2), then the final exit (4) = 494
static constexpr uint16_t FRAME_MAX_BYTES = (FRAME_BUDGET - 2) + 4 + 4 + 2 + 2 + 4;

// AT7456E command completion times (DS p.36 table; MAX7456 column for comparison)
namespace timing {
static constexpr uint32_t NVM_READ_MIN_NS     = 22000;   // CMM=0x50 -> STAT[5]=0, min
static constexpr uint32_t NVM_READ_TYP_NS     = 30000;   // typ (also DS p.29, p.34)
static constexpr uint32_t MAX7456_NVM_READ_NS = 500;
static constexpr uint32_t DISPLAY_OFF_MAX_MIN_NS = 18000;   // VM0[3]=0: "0 ~ 18us" min column
static constexpr uint32_t DISPLAY_OFF_TYP_NS  = 30000;   // typ
static constexpr uint32_t SOFT_RESET_MIN_NS   = 28000;   // VM0[1]=1: 28 us min, 40 typ
static constexpr uint32_t SOFT_RESET_TYP_NS   = 40000;
static constexpr uint32_t CLEAR_TYP_NS        = 40000;   // DMM[2]=1
static constexpr uint32_t NVM_WRITE_TYP_NS    = 5000000; // CMM=0xA0: 5 ms typ (tNVW 3.4/4.2 ms, p.4)
static constexpr uint32_t TCP_MIN_NS          = 100;     // SCLK period (DS p.4)
static constexpr uint32_t TCH_MIN_NS          = 40;
static constexpr uint32_t TCL_MIN_NS          = 40;
// driver delays (DS p.36 note 3: fixed delays shall use the typical values)
static constexpr uint32_t NVM_READ_DELAY_US   = 40;      // >= 30 us typ + 33 %
static constexpr uint32_t DISPLAY_OFF_DELAY_US = 40;
static constexpr uint32_t PROBE_RESET_WAIT_US = 1000;    // AP init() hal.scheduler->delay(1)
static constexpr uint32_t CLEAR_WAIT_US       = 40;      // DMM[2] clear, typ
}

// ---------------------------------------------------------------------------
// SPI clock derivation (AP_HAL_ChibiOS/SPIDevice.cpp:259-280
// derive_freq_flag_bus: start at kernel/2, halve while > request, max 7 steps)
// ---------------------------------------------------------------------------
struct SpiClock {
    uint32_t sclk_hz = 0;
    uint8_t mbr = 0;              // SPI_CFG1.MBR: divisor 2^(mbr+1)
    float tcp_ns = 0, tch_ns = 0, tcl_ns = 0;   // 50 % duty (even STM32 divisor)
    float margin_tcp_ns = 0, margin_tch_ns = 0;
    bool ok = false;              // all >= DS p.4 minimums
    bool zero_margin = false;     // exactly at tCP min
};
SpiClock derive_spi_clock(uint32_t kernel_hz, uint32_t request_hz);
// Time from the end of the CMM=0x50 command byte pair to the first CMDO data
// byte in upstream's single burst (3 bytes of SCLK), in ns.
uint32_t upstream_nvm_read_gap_ns(uint32_t sclk_hz);

// STAT decode exactly as AP :94-105
inline bool stat_is_los(uint8_t s) { return (s & STAT_LOS) != 0; }
inline bool vin_is_pal(uint8_t s) { return !stat_is_los(s) && (s & STAT_PAL); }
inline bool vin_is_ntsc_alt(uint8_t s) { return !stat_is_los(s) && !(s & STAT_PAL); }

// ---------------------------------------------------------------------------
// Abstract hardware hook: one call = one /CS assertion (AP_HAL::SPIDevice)
// ---------------------------------------------------------------------------
class SpiDevice {
public:
    virtual ~SpiDevice() = default;
    virtual bool transfer(const uint8_t* tx, size_t len) = 0;          // TX only
    virtual bool transfer_fullduplex(uint8_t* buf, size_t len) = 0;    // buf: TX in, RX out
    virtual void delay_us(uint32_t us) = 0;
};

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------
struct Config {
    bool delayed_nvm_read = true;     // false = upstream single burst
    uint32_t nvm_read_delay_us = timing::NVM_READ_DELAY_US;
    uint32_t display_off_delay_us = timing::DISPLAY_OFF_DELAY_US;
    uint8_t nvm_read_busy_polls = 10; // extra STAT[5] polls, 10 us apart (0 = fixed delay only)
    uint32_t nvm_write_poll_us = 1000;
    uint16_t nvm_write_max_polls = 50;
    bool verify_after_write = true;
    bool escape_ff = true;            // write char 0xFF outside auto-increment
    uint32_t probe_wait_us = timing::PROBE_RESET_WAIT_US;
    uint32_t clear_wait_us = timing::CLEAR_WAIT_US;   // + up to 10 DMM[2] polls, 10 us apart

    static Config robust() { return Config(); }
    static Config upstream()
    {
        Config c;
        c.delayed_nvm_read = false;
        c.nvm_read_delay_us = 0;
        c.display_off_delay_us = 0;
        c.nvm_read_busy_polls = 0;
        c.nvm_write_poll_us = 15000;  // hal.scheduler->delay(15)
        c.nvm_write_max_polls = 10000;
        c.verify_after_write = false;
        c.escape_ff = false;
        c.clear_wait_us = 0;
        return c;
    }
};

struct FontStats {
    uint16_t checked = 0, mismatched = 0, rewritten = 0;
    uint16_t verify_failed = 0, write_timeouts = 0, busy_waits = 0;
    uint32_t nvm_write_polls = 0;
    bool ok = false;
};

enum class VideoFormat : uint8_t { PAL = 0, NTSC };

struct FrameStats {
    uint32_t flushes = 0, bytes_last = 0, bytes_max = 0, chars_last = 0, truncated = 0;
    uint32_t reinits = 0, vm0_mismatch = 0, stat_checks = 0, clear_polls = 0;
};

class At7456e {
public:
    explicit At7456e(SpiDevice& dev, const Config& cfg = Config());

    bool probe();
    // font must be FONT_BYTES (AP :170); false on size error, write timeout,
    // or verify failure.
    bool update_font(const uint8_t* font, uint32_t len, FontStats* st = nullptr);
    bool check_font_char(uint8_t chr, const uint8_t* data, FontStats* st = nullptr);
    bool update_font_char(uint8_t chr, const uint8_t* data, FontStats* st = nullptr);

    // AP flush(): check_reinit() + transfer_frame() (font / offset paths are
    // separate calls here)
    void flush(uint32_t now_ms);
    void check_reinit(uint32_t now_ms);
    void reinit(uint32_t now_ms);
    void set_offsets(int16_t h, int16_t v);   // constrain 0..63 / 0..31 (AP :345-348)

    void clear();                                   // frame = ' ' (AP :451-455)
    void write(uint8_t x, uint8_t y, const char* text);   // AP :457-466
    void set_char(uint8_t x, uint8_t y, uint8_t c);
    uint8_t frame_char(uint8_t x, uint8_t y) const { return frame_[y][x]; }
    // returns bytes sent (0 = nothing dirty / not initialised)
    uint32_t transfer_frame();

    bool initialized() const { return initialized_; }
    VideoFormat format() const { return format_; }
    uint8_t rows() const { return video_lines_; }
    uint8_t video_signal_reg() const { return video_signal_reg_; }
    const FrameStats& frame_stats() const { return fs_; }
    const Config& config() const { return cfg_; }
    uint32_t last_buffer_peak() const { return buf_peak_; }

private:
    void add_cmd(uint8_t r, uint8_t v);
    bool is_dirty(uint8_t x, uint8_t y) const;
    bool write_register(uint8_t r, uint8_t v);
    bool read_register(uint8_t r, uint8_t& v);

    SpiDevice& dev_;
    Config cfg_;
    uint8_t buffer_[SPI_BUFFER];
    uint32_t off_ = 0, buf_peak_ = 0;
    uint8_t frame_[ROWS_PAL][COLUMNS];
    uint8_t shadow_[ROWS_PAL][COLUMNS];
    uint8_t video_signal_reg_ = VM0_MODE_PAL | VM0_OSD_ENABLE;   // AP ctor :139
    uint8_t video_lines_ = ROWS_PAL;
    VideoFormat format_ = VideoFormat::PAL;
    bool initialized_ = false;
    uint32_t last_signal_check_ = 0, video_detect_time_ = 0;
    int8_t hos_ = 0, vos_ = 0;
    FrameStats fs_;
};

} // namespace nano_osd
