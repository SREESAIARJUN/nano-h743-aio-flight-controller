// Nano H743 AIO - bidirectional DShot600 telemetry processing (Revision 10.0)
// See bdshot_telem.h for scope.  Every function that mirrors ArduPilot names
// the upstream function it reproduces; keep them bit-exact.
#include "bdshot_telem.h"

namespace nano_bdshot {

namespace {

// Cortex-M7 LSL-by-register semantics: only Rs[7:0] is used and any amount
// >= 32 yields 0.  ArduPilot's GCR decoder relies on this implicitly.
inline uint32_t lsl_m7(uint32_t x, uint32_t n)
{
    n &= 0xFFU;
    return n >= 32U ? 0U : (x << n);
}

// nibble -> 5-bit GCR code (DShot / BLHeli_32 / AM32 table)
constexpr uint8_t GCR_ENCODE[16] = {
    0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17,
    0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F,
};

// 5-bit GCR code -> nibble (verbatim from RCOutput::bdshot_decode_telemetry_packet;
// invalid codes decode to 0 and are caught by the CRC)
constexpr uint32_t GCR_DECODE[32] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 10, 11, 0, 13, 14, 15,
    0, 0, 2, 3, 0, 5, 6, 7, 0, 0, 8, 1, 0, 4, 12, 0 };

} // namespace

uint16_t telemetry_frame_with_crc(uint16_t payload12)
{
    payload12 &= 0x0FFFU;
    const uint16_t crc = (~(payload12 ^ (payload12 >> 4) ^ (payload12 >> 8))) & 0x0FU;
    return uint16_t((payload12 << 4) | crc);
}

// == RCOutput::bdshot_decode_telemetry_packet (ArduPilot 4.7.x, non-F1 path)
uint32_t decode_gcr_edges(const uint16_t* buffer, uint32_t count)
{
    if (buffer == nullptr || count == 0U) {
        return INVALID_ERPM;
    }
    uint32_t value = 0;
    uint32_t bits = 0;
    uint32_t len;
    uint16_t old_value = buffer[0];

    for (uint32_t i = 1; i <= count; i++) {
        if (i < count) {
            // ArduPilot: dmar_int_t (int32) difference of uint32 CCR snapshots,
            // then (diff + 8U) / 16 in UNSIGNED arithmetic.  The capture
            // counter is reset to 0 before every receive window and wraps
            // only after 5.46 ms, so timestamps are monotonic in hardware; a
            // reversed interval of -1..-8 ticks (impossible on the real timer,
            // reachable only by synthetic input) yields len 0, i.e. the edge
            // is ignored - reproduced here for bit-exactness.
            const int32_t diff = int32_t(buffer[i]) - int32_t(old_value);
            if (bits >= GCR_FRAME_BITS) {
                break;
            }
            len = (uint32_t(diff) + TELEM_IC_SAMPLE / 2U) / TELEM_IC_SAMPLE;
        } else {
            // final run: whatever is left of the 21-bit frame.  When bits is
            // already 21 this is 0 and ArduPilot shifts by (0 - 1); on the M7
            // that ORs in nothing, which lsl_m7() reproduces without UB.
            len = GCR_FRAME_BITS - bits;
        }
        value = lsl_m7(value, len);
        value |= lsl_m7(1U, len - 1U);
        old_value = (i < count) ? buffer[i] : old_value;
        bits += len;
    }

    if (bits != GCR_FRAME_BITS) {
        return INVALID_ERPM;
    }

    uint32_t decoded = GCR_DECODE[value & 0x1FU];
    decoded |= GCR_DECODE[(value >> 5U) & 0x1FU] << 4U;
    decoded |= GCR_DECODE[(value >> 10U) & 0x1FU] << 8U;
    decoded |= GCR_DECODE[(value >> 15U) & 0x1FU] << 12U;

    uint32_t csum = decoded;
    csum = csum ^ (csum >> 8U);
    csum = csum ^ (csum >> 4U);
    if ((csum & 0xFU) != 0xFU) {
        return INVALID_ERPM;
    }
    return decoded >> 4;
}

uint32_t encode_gcr_edges(uint16_t payload12, uint16_t t0, uint32_t ticks_per_bit_x256,
                          uint16_t* out, uint32_t max_out)
{
    const uint16_t frame = telemetry_frame_with_crc(payload12);
    uint32_t gcr20 = 0;
    for (int n = 3; n >= 0; n--) {
        gcr20 = (gcr20 << 5) | GCR_ENCODE[(frame >> (4 * n)) & 0xFU];
    }
    // Line coding: the 21-bit word (start '1' + 20 GCR bits) has a '1' at
    // every bit cell that begins with a level transition.  The receiver only
    // sees edge timestamps; bit k (MSB first, k=0 is the start edge) begins at
    // t0 + k * bit_time.
    const uint32_t word = (1U << 20) | gcr20;
    uint32_t n_out = 0;
    for (uint32_t k = 0; k < GCR_FRAME_BITS && n_out < max_out; k++) {
        if (word & (1U << (20U - k))) {
            const uint32_t t = (uint32_t(k) * ticks_per_bit_x256 + 128U) >> 8;
            out[n_out++] = uint16_t(t0 + t);
        }
    }
    return n_out;
}

// == RCOutput::bdshot_decode_telemetry_from_erpm (value handling, not the
//    AP_ESC_Telem side effects)
Frame decode_payload(uint32_t payload12, bool edt_enabled)
{
    Frame f{FrameType::Invalid, 0, 0};
    if (payload12 == INVALID_ERPM) {
        return f;
    }
    const uint16_t encoded = uint16_t(payload12);
    const uint8_t expo = uint8_t((encoded >> 9U) & 0x7U);
    const uint16_t value = uint16_t(encoded & 0x1FFU);

    if (!(value & 0x100U) && edt_enabled) {
        f.value = uint16_t(value & 0xFFU);
        switch (expo) {
        case 0b001: f.type = FrameType::EDT_Temperature; return f;
        case 0b010: f.type = FrameType::EDT_Voltage;     return f;
        case 0b011: f.type = FrameType::EDT_Current;     return f;
        case 0b100: f.type = FrameType::EDT_Debug1;      return f;
        case 0b101: f.type = FrameType::EDT_Debug2;      return f;
        case 0b110: f.type = FrameType::EDT_Stress;      return f;
        case 0b111: f.type = FrameType::EDT_Status;      return f;
        default: break;   // expo 0: eRPM with an un-normalised mantissa
        }
        f.value = 0;
    }

    uint16_t erpm = uint16_t(value << expo);   // period in us
    if (erpm == 0U) {
        return f;   // Invalid
    }
    // ArduPilot stores into a uint16_t: periods < 10 us (> 6M eRPM, physically
    // impossible) wrap.  Reproduced deliberately; MotorTelemetryMonitor's
    // plausibility gate is what rejects them on this board.
    erpm = uint16_t((1000000U * 60U / 100U + erpm / 2U) / erpm);
    if (encoded == ZERO_ERPM) {
        erpm = 0;
    }
    if (erpm >= INVALID_ERPM) {
        return f;
    }
    f.type = FrameType::ERPM;
    f.erpm_x100 = erpm;
    return f;
}

uint32_t erpm_x100_to_rpm(uint16_t erpm_x100, uint8_t motor_poles)
{
    if (motor_poles == 0U) {
        return 0;
    }
    return uint32_t(erpm_x100) * 200U / motor_poles;
}

uint16_t encode_erpm_period(uint32_t erpm)
{
    if (erpm == 0U) {
        return ZERO_ERPM;
    }
    uint32_t period_us = (60000000U + erpm / 2U) / erpm;
    uint32_t e = 0;
    while (period_us > 0x1FFU && e < 7U) {
        period_us >>= 1;   // AM32 / BLHeli_32 truncate
        e++;
    }
    if (period_us > 0x1FFU) {
        return ZERO_ERPM;  // slower than the format can express: report stopped
    }
    return uint16_t((e << 9) | period_us);
}

// ---------------------------------------------------------------------------
MotorTelemetryMonitor::MotorTelemetryMonitor(uint8_t motors_in_group, uint8_t stale_slots)
    : frame_(0), slot_(0),
      n_(motors_in_group == 0 || motors_in_group > NUM_MOTORS ? NUM_MOTORS : motors_in_group),
      stale_slots_(stale_slots == 0 ? 1 : stale_slots)
{
    for (auto& h : m_) {
        h = MotorHealth{0, 0, 0, 0, 0, false};
    }
}

uint8_t MotorTelemetryMonitor::on_frame(const Frame& f)
{
    const uint8_t motor = slot_;
    MotorHealth& h = m_[motor];
    if (f.type == FrameType::Invalid) {
        h.bad_frames++;
        if (h.consecutive_bad < 0xFFU) {
            h.consecutive_bad++;
        }
    } else {
        // EDT frames are valid GCR/CRC frames: ArduPilot counts them as
        // clean (erpm_clean_frames) even though they carry no speed.
        h.good_frames++;
        h.consecutive_bad = 0;
        if (f.type == FrameType::ERPM) {
            h.erpm_x100 = f.erpm_x100;
            h.last_good_frame = frame_;
            h.ever_valid = true;
        }
    }
    frame_++;
    slot_ = uint8_t((slot_ + 1U) % n_);
    return motor;
}

bool MotorTelemetryMonitor::fresh(uint8_t motor) const
{
    if (motor >= n_ || !m_[motor].ever_valid) {
        return false;
    }
    // frames since this motor's last good eRPM, in units of its own slots
    const uint32_t age_frames = frame_ - m_[motor].last_good_frame;
    return age_frames <= uint32_t(stale_slots_) * n_;
}

float MotorTelemetryMonitor::error_rate_pct(uint8_t motor) const
{
    const MotorHealth& h = m_[motor];
    return 100.0f * float(h.bad_frames) / float(1U + h.bad_frames + h.good_frames);
}

// ---------------------------------------------------------------------------
ApErrorCounterModel::ApErrorCounterModel(uint8_t motors_in_group, bool patched)
    : curr_(0), n_(motors_in_group == 0 || motors_in_group > 4 ? 4 : motors_in_group), patched_(patched)
{
    for (uint8_t i = 0; i < 4; i++) {
        err_[i] = 0;
        clean_[i] = 0;
    }
}

void ApErrorCounterModel::on_frame(bool got_edges, bool crc_ok)
{
    // DMA-complete ISR: prev = curr; curr = next   (bdshot_receive_pulses_DMAR)
    const uint8_t prev = curr_;
    curr_ = uint8_t((curr_ + 1U) % n_);
    // next bdshot_send_pulse(): book the outcome
    if (got_edges) {
        if (crc_ok) {
            clean_[prev]++;
        } else {
            err_[prev]++;
        }
    } else {
        err_[patched_ ? prev : curr_]++;
    }
}

// ---------------------------------------------------------------------------
void plan_notches(const float* motor_hz, uint8_t n_motors, uint8_t harmonics_mask,
                  float min_hz, float gyro_rate_hz, NotchPlan& out)
{
    out.count = 0;
    const float nyquist = gyro_rate_hz * 0.48f;           // HARMONIC_NYQUIST_CUTOFF
    const float disable_hz = min_hz * 0.25f;               // NOTCHFILTER_ATTENUATION_CUTOFF
    // order: f1h1, f2h1, ..., f1h2, ... (HarmonicNotchFilter::update)
    for (uint8_t h = 0; h < 4; h++) {
        if (!(harmonics_mask & (1U << h))) {
            continue;
        }
        for (uint8_t m = 0; m < n_motors && out.count < NotchPlan::MAX; m++) {
            NotchSlot& s = out.slot[out.count++];
            float fc = motor_hz[m];
            if (fc < 0.0f) fc = 0.0f;
            if (fc > nyquist) fc = nyquist;
            fc *= float(h + 1);
            s.center_hz = 0.0f;
            s.attenuation_scale = 0.0f;
            s.enabled = false;
            if (fc >= nyquist) {
                continue;                       // above Nyquist guard
            }
            if (fc < disable_hz) {
                continue;                       // too far below min: disabled
            }
            float scale = 1.0f;
            if (fc < min_hz) {
                // A fades linearly from full (at min) to none (at disable_hz)
                scale = (fc - disable_hz) / (min_hz - disable_hz);
            }
            s.center_hz = fc < min_hz ? min_hz : fc;
            s.attenuation_scale = scale;
            s.enabled = true;
        }
    }
}

} // namespace nano_bdshot
