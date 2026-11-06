// Nano H743 AIO - AT7456E analog OSD register-level driver reference (Revision 13.0)
// Sreesai Arjun Engineering
#include "osd_at7456e.h"

#include <string.h>

namespace nano_osd {

// ---------------------------------------------------------------------------
// SPI clock
// ---------------------------------------------------------------------------
SpiClock derive_spi_clock(uint32_t kernel_hz, uint32_t request_hz)
{
    SpiClock c;
    uint32_t f = kernel_hz / 2;
    uint8_t i = 0;
    while (f > request_hz && i < 7) {
        f >>= 1;
        i++;
    }
    c.sclk_hz = f;
    c.mbr = i;
    if (f == 0) {
        return c;
    }
    c.tcp_ns = 1e9f / float(f);
    c.tch_ns = c.tcp_ns * 0.5f;
    c.tcl_ns = c.tcp_ns * 0.5f;
    c.margin_tcp_ns = c.tcp_ns - float(timing::TCP_MIN_NS);
    c.margin_tch_ns = c.tch_ns - float(timing::TCH_MIN_NS);
    c.ok = c.margin_tcp_ns >= -0.01f && c.margin_tch_ns >= -0.01f && (c.tcl_ns - float(timing::TCL_MIN_NS)) >= -0.01f;
    c.zero_margin = c.ok && c.margin_tcp_ns < 0.5f;
    return c;
}

uint32_t upstream_nvm_read_gap_ns(uint32_t sclk_hz)
{
    // bytes 0..5 = VM0, CMAH, CMM pairs; CMM latched at the end of byte 5,
    // first CMDO data shifted out during byte 9: bytes 6, 7, 8 in between.
    return uint32_t(uint64_t(3u * 8u) * 1000000000ull / sclk_hz);
}

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------
At7456e::At7456e(SpiDevice& dev, const Config& cfg) : dev_(dev), cfg_(cfg)
{
    memset(buffer_, 0, sizeof(buffer_));
    memset(frame_, 0, sizeof(frame_));
    memset(shadow_, 0, sizeof(shadow_));
}

void At7456e::add_cmd(uint8_t r, uint8_t v)
{
    // AP buffer_add_cmd (:266-272): silently drops when full
    if (off_ < uint32_t(SPI_BUFFER) - 1) {
        buffer_[off_++] = r;
        buffer_[off_++] = v;
    }
    if (off_ > buf_peak_) {
        buf_peak_ = off_;
    }
}

bool At7456e::write_register(uint8_t r, uint8_t v)
{
    const uint8_t b[2] = {r, v};
    return dev_.transfer(b, 2);
}

bool At7456e::read_register(uint8_t r, uint8_t& v)
{
    uint8_t b[2] = {r, 0xFF};
    const bool ok = dev_.transfer_fullduplex(b, 2);
    v = b[1];
    return ok;
}

bool At7456e::probe()
{
    uint8_t status = 0xFF;
    write_register(reg::VM0, VM0_SOFT_RESET);
    dev_.delay_us(cfg_.probe_wait_us);
    if (!read_register(reg::VM0 | reg::READ, status)) {
        return false;
    }
    return status == 0;
}

bool At7456e::check_font_char(uint8_t chr, const uint8_t* data, FontStats* st)
{
    if (st) st->checked++;
    off_ = 0;
    if (!cfg_.delayed_nvm_read) {
        // upstream :193-218, one burst
        add_cmd(reg::VM0, 0);
        add_cmd(reg::CMAH, chr);
        add_cmd(reg::CMM, CMM_READ_NVR);
        for (uint16_t x = 0; x < CHAR_BYTES; x++) {
            add_cmd(reg::CMAL, uint8_t(x));
            add_cmd(reg::CMDO, 0xFF);
        }
        dev_.transfer_fullduplex(buffer_, off_);
        uint32_t o = 9;
        for (uint16_t x = 0; x < CHAR_BYTES; x++) {
            if (buffer_[o] != data[x]) {
                if (st) st->mismatched++;
                return false;
            }
            o += 4;
        }
        return true;
    }
    // 1) display off, wait for it to take effect (DS p.34 step 1, p.36)
    write_register(reg::VM0, 0);
    if (cfg_.display_off_delay_us) dev_.delay_us(cfg_.display_off_delay_us);
    // 2) select char, start NVM -> shadow RAM read
    add_cmd(reg::CMAH, chr);
    add_cmd(reg::CMM, CMM_READ_NVR);
    dev_.transfer(buffer_, off_);
    // 3) >= 22 us min / 30 us typ (DS p.36), then confirm STAT[5] = 0
    dev_.delay_us(cfg_.nvm_read_delay_us);
    for (uint8_t i = 0; i < cfg_.nvm_read_busy_polls; i++) {
        uint8_t s = 0xFF;
        read_register(reg::STAT, s);
        if (!(s & STAT_NVR_BUSY)) {
            break;
        }
        if (st) st->busy_waits++;
        dev_.delay_us(10);
    }
    // 4) read the 54 bytes from the now-valid shadow RAM
    off_ = 0;
    for (uint16_t x = 0; x < CHAR_BYTES; x++) {
        add_cmd(reg::CMAL, uint8_t(x));
        add_cmd(reg::CMDO, 0xFF);
    }
    dev_.transfer_fullduplex(buffer_, off_);
    uint32_t o = 3;
    for (uint16_t x = 0; x < CHAR_BYTES; x++) {
        if (buffer_[o] != data[x]) {
            if (st) st->mismatched++;
            return false;
        }
        o += 4;
    }
    return true;
}

bool At7456e::update_font_char(uint8_t chr, const uint8_t* data, FontStats* st)
{
    if (cfg_.display_off_delay_us) {
        write_register(reg::VM0, 0);
        dev_.delay_us(cfg_.display_off_delay_us);
    }
    off_ = 0;
    add_cmd(reg::VM0, 0);
    add_cmd(reg::CMAH, chr);
    for (uint16_t x = 0; x < CHAR_BYTES; x++) {
        add_cmd(reg::CMAL, uint8_t(x));
        add_cmd(reg::CMDI, data[x]);
    }
    add_cmd(reg::CMM, CMM_WRITE_NVR);
    dev_.transfer(buffer_, off_);
    if (st) st->rewritten++;

    uint16_t retry;
    for (retry = 0; retry < cfg_.nvm_write_max_polls; retry++) {
        uint8_t status = 0xFF;
        dev_.delay_us(cfg_.nvm_write_poll_us);
        if (st) st->nvm_write_polls++;
        read_register(reg::STAT, status);
        if ((status & STAT_NVR_BUSY) == 0) {
            break;
        }
    }
    if (retry == cfg_.nvm_write_max_polls) {
        if (st) st->write_timeouts++;
        return false;
    }
    if (cfg_.verify_after_write && cfg_.delayed_nvm_read) {
        if (!check_font_char(chr, data, nullptr)) {
            if (st) st->verify_failed++;
            return false;
        }
    }
    return true;
}

bool At7456e::update_font(const uint8_t* font, uint32_t len, FontStats* st)
{
    FontStats local;
    FontStats& s = st ? *st : local;
    s = FontStats();
    if (font == nullptr || len != FONT_BYTES) {
        return false;
    }
    for (uint16_t chr = 0; chr < FONT_CHARS; chr++) {
        const uint8_t* d = font + uint32_t(chr) * CHAR_BYTES;
        if (!check_font_char(uint8_t(chr), d, &s)) {
            if (!update_font_char(uint8_t(chr), d, &s)) {
                return false;
            }
        }
    }
    s.ok = true;
    return true;
}

// ---------------------------------------------------------------------------
// Format detection (mirror of AP :275-356)
// ---------------------------------------------------------------------------
void At7456e::check_reinit(uint32_t now_ms)
{
    uint8_t check = 0xFF;
    read_register(reg::VM0 | reg::READ, check);
    if (check != video_signal_reg_) {
        fs_.vm0_mismatch++;
        reinit(now_ms);
    } else if ((now_ms - last_signal_check_) > 1000u) {   // VIDEO_SIGNAL_CHECK_INTERVAL_MS
        uint8_t sense = 0;
        fs_.stat_checks++;
        read_register(reg::STAT, sense);
        if (sense & STAT_LOS) {
            video_detect_time_ = 0;
        } else {
            const bool mode_pal = (video_signal_reg_ & VM0_MODE_MASK) == VM0_MODE_PAL;
            if ((vin_is_pal(sense) && !mode_pal) || (vin_is_ntsc_alt(sense) && mode_pal)) {
                if (video_detect_time_) {
                    if (now_ms - video_detect_time_ > 100u) {     // VIDEO_SIGNAL_DEBOUNCE_MS
                        reinit(now_ms);
                    }
                } else {
                    video_detect_time_ = now_ms;
                }
            }
        }
        last_signal_check_ = now_ms;
    }
}

void At7456e::reinit(uint32_t now_ms)
{
    if (now_ms < 1500u) {
        return;   // camera power-up guard (AP :318-320)
    }
    uint8_t sense = 0;
    read_register(reg::STAT, sense);
    if (vin_is_pal(sense)) {
        video_signal_reg_ = VM0_MODE_PAL | VM0_OSD_ENABLE;
        video_lines_ = ROWS_PAL;
        format_ = VideoFormat::PAL;
    } else {
        video_signal_reg_ = VM0_MODE_NTSC | VM0_OSD_ENABLE;
        video_lines_ = ROWS_NTSC;
        format_ = VideoFormat::NTSC;
    }
    for (uint8_t x = 0; x < ROWS_PAL; x++) {
        write_register(uint8_t(reg::RB0 + x), RB_VALUE);
    }
    write_register(reg::VM0, video_signal_reg_);
    write_register(reg::VM1, VM1_VALUE);
    write_register(reg::DMM, DMM_CLEAR_DISPLAY);
    if (cfg_.clear_wait_us) {
        dev_.delay_us(cfg_.clear_wait_us);
        for (uint8_t i = 0; i < 10; i++) {
            uint8_t dmm = 0xFF;
            read_register(reg::DMM | reg::READ, dmm);
            if (!(dmm & DMM_CLEAR_DISPLAY)) {
                break;
            }
            fs_.clear_polls++;
            dev_.delay_us(10);
        }
    }
    write_register(reg::HOS, uint8_t(hos_));
    write_register(reg::VOS, uint8_t(vos_));
    memset(shadow_, 0xFF, sizeof(shadow_));
    initialized_ = true;
    fs_.reinits++;
}

void At7456e::set_offsets(int16_t h, int16_t v)
{
    hos_ = int8_t(h < 0 ? 0 : (h > 63 ? 63 : h));
    vos_ = int8_t(v < 0 ? 0 : (v > 31 ? 31 : v));
    write_register(reg::HOS, uint8_t(hos_));
    write_register(reg::VOS, uint8_t(vos_));
}

void At7456e::flush(uint32_t now_ms)
{
    check_reinit(now_ms);
    transfer_frame();
}

// ---------------------------------------------------------------------------
// Frame
// ---------------------------------------------------------------------------
void At7456e::clear() { memset(frame_, ' ', sizeof(frame_)); }

void At7456e::write(uint8_t x, uint8_t y, const char* text)
{
    if (y >= ROWS_PAL || text == nullptr) {
        return;
    }
    while (x < COLUMNS && *text != 0) {
        frame_[y][x] = uint8_t(*text);
        ++text;
        ++x;
    }
}

void At7456e::set_char(uint8_t x, uint8_t y, uint8_t c)
{
    if (y < ROWS_PAL && x < COLUMNS) {
        frame_[y][x] = c;
    }
}

bool At7456e::is_dirty(uint8_t x, uint8_t y) const
{
    if (y >= video_lines_ || x >= COLUMNS) {
        return false;
    }
    return frame_[y][x] != shadow_[y][x];
}

uint32_t At7456e::transfer_frame()
{
    uint16_t previous_pos = UINT16_MAX - 1;
    bool autoinc = false;
    if (!initialized_) {
        return 0;
    }
    off_ = 0;
    uint32_t chars = 0;
    bool truncated = false;
    for (uint8_t y = 0; y < video_lines_; y++) {
        for (uint8_t x = 0; x < COLUMNS; x++) {
            if (!is_dirty(x, y)) {
                continue;
            }
            if (off_ >= FRAME_BUDGET) {
                truncated = true;
                break;            // upstream: leaves the column loop only
            }
            shadow_[y][x] = frame_[y][x];
            const uint8_t chr = frame_[y][x];
            const uint16_t pos = uint16_t(y * COLUMNS + x);
            const bool position_changed = uint16_t(previous_pos + 1) != pos;
            const bool ff = cfg_.escape_ff && chr == DMDI_AUTOINC_END;

            if ((position_changed || ff) && autoinc) {
                add_cmd(reg::DMDI, DMDI_AUTOINC_END);
                add_cmd(reg::DMM, 0);
                autoinc = false;
            }
            if (!autoinc) {
                add_cmd(reg::DMAH, uint8_t(pos >> 8));
                add_cmd(reg::DMAL, uint8_t(pos & 0xFF));
            }
            if (!autoinc && !ff && is_dirty(uint8_t(x + 1), y)) {
                add_cmd(reg::DMM, DMM_AUTOINCREMENT);
                autoinc = true;
            }
            add_cmd(reg::DMDI, chr);
            previous_pos = pos;
            chars++;
        }
    }
    if (autoinc) {
        add_cmd(reg::DMDI, DMDI_AUTOINC_END);
        add_cmd(reg::DMM, 0);
    }
    const uint32_t sent = off_;
    if (sent > 0) {
        dev_.transfer(buffer_, sent);
    }
    fs_.flushes++;
    fs_.bytes_last = sent;
    fs_.chars_last = chars;
    if (sent > fs_.bytes_max) fs_.bytes_max = sent;
    if (truncated) fs_.truncated++;
    return sent;
}

} // namespace nano_osd
