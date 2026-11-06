// Nano H743 AIO - bidirectional DShot600 telemetry processing (Revision 10.0)
// Sreesai Arjun Engineering
//
// Host-testable reference implementation of the ESC -> FC telemetry path that
// ArduPilot runs on this board (libraries/AP_HAL_ChibiOS/RCOutput_bdshot.cpp,
// AP_ESC_Telem, Filter/HarmonicNotchFilter).  It exists for three reasons:
//
//   1. Bit-exact regression model.  The GCR decoder, eRPM/EDT decoder and the
//      eRPM -> mechanical RPM conversion reproduce ArduPilot 4.7.x integer
//      arithmetic exactly, so tests/test_bdshot_telem.cpp can prove what the
//      flight code will report for a given wire waveform on THIS board's
//      timer configuration (TIM1, 240 MHz kernel clock, TELEM_IC_SAMPLE 16).
//   2. Board-level health policy.  ArduPilot only exposes a 5 s windowed error
//      percentage; MotorTelemetryMonitor adds staleness tracking that
//      understands the TIM1 round-robin (one motor captured per DShot frame)
//      so bench/ground-test tooling can flag a dead ESC within 3 of its own
//      capture slots (12 DShot frames on TIM1), plus a physical-plausibility
//      gate ArduPilot does not have.
//   3. Notch planning.  NotchPlanner reproduces the HarmonicNotchFilter
//      min-frequency / fade-out / disable rules for INS_HNTCH_MODE 3 with
//      INS_HNTCH_OPTS 6 so analysis/bdshot_timing_analysis.py and the tests
//      can check the defaults.parm values against real motor speeds.
//
// No dynamic allocation, no exceptions, no floating point in the decode path
// (matches the ISR-side constraints of RCOutput_bdshot.cpp).
#pragma once

#include <stdint.h>
#include <stddef.h>

namespace nano_bdshot {

// ---------------------------------------------------------------------------
// Board timing constants (derived in analysis/bdshot_timing_analysis.py)
// ---------------------------------------------------------------------------
// TIM1 sits on APB2: SYSCLK 480 MHz -> HCLK 240 MHz -> PCLK2 120 MHz, timer x2.
static constexpr uint32_t TIM1_CLOCK_HZ       = 240000000UL;
static constexpr uint32_t DSHOT600_BITRATE    = 600000UL;
// GCR reply runs at 5/4 of the command bit rate (RCOutput::bdshot_get_output_rate_hz).
static constexpr uint32_t GCR_BITRATE         = DSHOT600_BITRATE * 5U / 4U;   // 750 kbit/s
// ArduPilot samples each GCR bit with TELEM_IC_SAMPLE timer ticks.
static constexpr uint32_t TELEM_IC_SAMPLE     = 16U;
// telempsc = lrintf((clock / gcr_rate + 0.01) / 16) - 1  -> 240e6/750e3 = 320 / 16 = 20 -> PSC 19
static constexpr uint32_t TELEM_PSC           = (TIM1_CLOCK_HZ / GCR_BITRATE + TELEM_IC_SAMPLE / 2U) / TELEM_IC_SAMPLE - 1U;
static constexpr uint32_t GCR_TELEMETRY_BIT_LEN = 22U;   // max captured edges (RCOutput.h)
static constexpr uint32_t GCR_FRAME_BITS      = 21U;     // start bit + 20 GCR bits

static constexpr uint16_t INVALID_ERPM = 0xFFFFU;   // RCOutput::INVALID_ERPM
static constexpr uint16_t ZERO_ERPM    = 0x0FFFU;   // e=7, m=511: "motor stopped"

// ---------------------------------------------------------------------------
// Layer 1: GCR edge-timestamp decode  (== RCOutput::bdshot_decode_telemetry_packet)
// ---------------------------------------------------------------------------
// buffer : TIM1 CCR snapshots of successive edges (both polarities), 16-bit
//          counter reset to 0 at the start of each receive window,
//          prescaled by TELEM_PSC.  count = number of edges captured.
// return : 12-bit payload (eee m mmmm mmmm) with the CRC nibble removed, or
//          INVALID_ERPM on length/GCR/CRC failure.
// Difference from ArduPilot (documented, intentional): ArduPilot evaluates
// `1U << (len - 1U)` with len == 0 when the captured edges already account
// for all 21 bits (UB in C++, harmless on Cortex-M7 because LSL by >= 32
// yields 0).  This implementation guards that case and produces the same
// result the M7 does.
uint32_t decode_gcr_edges(const uint16_t* buffer, uint32_t count);

// Inverse used by the tests and the bench pattern generator: encodes a 12-bit
// payload (CRC appended) into edge timestamps at nominal timing, starting at
// t0 ticks, with `ticks_per_bit` (16 nominal) ticks per GCR bit.  Returns the
// number of edges written (<= GCR_TELEMETRY_BIT_LEN).
uint32_t encode_gcr_edges(uint16_t payload12, uint16_t t0, uint32_t ticks_per_bit_x256,
                          uint16_t* out, uint32_t max_out);

// DShot 4-bit CRC for telemetry (inverted XOR of the three payload nibbles).
uint16_t telemetry_frame_with_crc(uint16_t payload12);

// ---------------------------------------------------------------------------
// Layer 2: payload decode  (== RCOutput::bdshot_decode_telemetry_from_erpm)
// ---------------------------------------------------------------------------
enum class FrameType : uint8_t {
    Invalid = 0,     // GCR length / CRC failure
    ZeroPeriod,      // CRC-valid but period field decodes to 0: ArduPilot
                     // counts it as a CLEAN frame yet publishes no RPM
    ERPM,            // eRPM sample (erpm_x100 valid; 0 means stopped)
    EDT_Temperature, // value = deg C
    EDT_Voltage,     // value = 0.25 V units
    EDT_Current,     // value = A
    EDT_Debug1,
    EDT_Debug2,
    EDT_Stress,      // EDT v2
    EDT_Status,      // EDT v2
};

struct Frame {
    FrameType type;
    uint16_t  erpm_x100;   // eRPM / 100, ArduPilot's _bdshot.erpm[] unit
    uint16_t  value;       // EDT raw 8-bit value; for ERPM the period in us
};

// edt_enabled == (SERVO_DSHOT_ESC is 3 or 4).  When false, EDT-shaped frames
// are interpreted as eRPM, exactly as ArduPilot does.
Frame decode_payload(uint32_t payload12, bool edt_enabled);

// Mechanical RPM as ArduPilot reports it: erpm_x100 * 200 / poles (integer).
// SERVO_BLH_POLES = 14 on this board (12N14P 2207/2306 class motors).
uint32_t erpm_x100_to_rpm(uint16_t erpm_x100, uint8_t motor_poles);

// Period field the ESC must send for a given eRPM (inverse of the above, used
// for quantisation analysis).  Returns 12-bit eee mmmmmmmmm, normalised the
// way AM32 / BLHeli_32 normalise it (largest mantissa that fits 9 bits).
uint16_t encode_erpm_period(uint32_t erpm);

// ---------------------------------------------------------------------------
// Layer 3: per-motor health with TIM1 round-robin awareness
// ---------------------------------------------------------------------------
// ArduPilot captures exactly ONE motor per DShot frame per timer group
// (pwm_group::bdshot.curr_telem_chan; bdshot_find_next_ic_channel()).  All
// four motors are on TIM1, so each motor is sampled every 4th frame.  A motor
// whose last good frame is older than `stale_frames` of ITS OWN capture slots
// is reported stale.
struct MotorHealth {
    uint32_t good_frames;        // CRC-valid (ArduPilot erpm_clean_frames)
    uint32_t bad_frames;         // GCR/CRC failure (ArduPilot erpm_errors)
    uint32_t implausible_frames; // CRC-valid eRPM faster than the motor can spin
    uint32_t last_good_frame;   // global frame index of the last good capture
    uint16_t erpm_x100;
    uint8_t  consecutive_bad;
    bool     ever_valid;
};

class MotorTelemetryMonitor {
public:
    static constexpr uint8_t NUM_MOTORS = 4;

    // min_period_us: shortest physically possible commutation period.  This
    // board: 42 000 RPM x 7 pole pairs = 294 000 eRPM -> 204 us; default 100 us
    // (2x margin).  A CRC-valid eRPM frame below it (4-bit CRC lets ~1/16 of
    // corrupted frames through, test T4) is booked as implausible and does not
    // refresh the motor.  ArduPilot has no such gate; 0 disables it.
    explicit MotorTelemetryMonitor(uint8_t motors_in_group = NUM_MOTORS,
                                   uint8_t stale_slots = 3,
                                   uint16_t min_period_us = 100);

    // Called once per DShot frame with the outcome of the capture of the motor
    // that owned this frame's slot.  Returns that motor index.
    uint8_t on_frame(const Frame& f);

    // True if the motor has produced a good eRPM frame within stale_slots of
    // its own capture opportunities.
    bool fresh(uint8_t motor) const;

    // Error rate in percent over the lifetime of the monitor, computed with
    // ArduPilot's formula 100*err/(1+err+clean) on ArduPilot's definitions of
    // err (GCR/CRC failure) and clean (CRC-valid, incl. EDT and zero-period).
    float error_rate_pct(uint8_t motor) const;

    const MotorHealth& health(uint8_t motor) const { return m_[motor]; }
    uint32_t frame_index() const { return frame_; }

private:
    MotorHealth m_[NUM_MOTORS];
    uint32_t frame_;
    uint8_t  slot_;
    uint8_t  n_;
    uint8_t  stale_slots_;
    uint16_t min_period_us_;
};

// ---------------------------------------------------------------------------
// Layer 3b: model of ArduPilot's own per-channel error counters
// ---------------------------------------------------------------------------
// RCOutput::bdshot_finish_dshot_gcr_transaction() (DMA-complete / timeout
// ISR) rotates curr_telem_chan BEFORE the receive outcome is booked.
// RCOutput::bdshot_prepare_for_next_pulse() then books
//   RECV_COMPLETE (edges seen) -> clean/erpm_errors[chan[prev_telem_chan]],
//                                 plus the 5 s statistics reset   (correct)
//   RECV_FAILED (no edges)     -> erpm_errors[curr_telem_chan]     (WRONG:
//                                 the NEXT motor; also a group-local index,
//                                 which only matters when chan[] != {0..3})
// so a completely silent ESC is charged to its neighbour.  This is a host
// model of that counter logic (uint16 counters, 5 s window in frames), used by
// test T8 to demonstrate the defect and the behaviour of
// ardupilot/patches/0001-bdshot-book-recv-failed-on-prev-telem-chan.patch,
// which also applies the 5 s reset on the RECV_FAILED path.
class ApErrorCounterModel {
public:
    // window_frames: DShot frames in ArduPilot's 5 s stats window (0 = never)
    ApErrorCounterModel(uint8_t motors_in_group, bool patched, uint32_t window_frames = 0);
    // one DShot frame; got_edges == false means the ESC never answered.
    void on_frame(bool got_edges, bool crc_ok);
    uint16_t errors(uint8_t motor) const { return err_[motor]; }
    uint16_t clean(uint8_t motor) const { return clean_[motor]; }
private:
    void maybe_reset(uint8_t motor);
    uint16_t err_[4];      // RCOutput.h: uint16_t erpm_errors[]
    uint16_t clean_[4];
    uint32_t last_reset_[4];
    uint32_t frame_;
    uint32_t window_;
    uint8_t  curr_;
    uint8_t  n_;
    bool     patched_;
};

// ---------------------------------------------------------------------------
// Layer 4: harmonic-notch frequency planning (INS_HNTCH_MODE 3, OPTS bit1)
// ---------------------------------------------------------------------------
// Mirrors HarmonicNotchFilter<T>::set_center_frequency() for the case
// TreatLowAsMin (OPTS bit5) == 0:
//   f < 0.25 * min            -> notch disabled
//   0.25*min <= f < min       -> centred at min, attenuation faded linearly
//   f >= min                  -> centred at f
// where f is the HARMONIC frequency h*f0 and min is NOT scaled by h (it is
// only scaled when TreatLowAsMin is set), and
//   h*f0 >= 0.48 * gyro_rate  -> that harmonic is disabled (Nyquist guard).
struct NotchSlot {
    float center_hz;
    float depth_fraction;   // fraction of the way from A=1 (0) to A0 (1): ArduPilot
                            // interpolates the LINEAR gain A, not dB
    float depth_db;         // effective notch depth = -40*log10(A)
    bool  enabled;
};

struct NotchPlan {
    static constexpr uint8_t MAX = 54;      // HAL_HNF_MAX_FILTERS on H7
    NotchSlot slot[MAX];
    uint8_t   count;
};

// motor_hz[] are per-motor mechanical frequencies (rpm/60), 0 = no data.
// harmonics_mask = INS_HNTCH_HMNCS (16 harmonics), min_hz = INS_HNTCH_FREQ *
// INS_HNTCH_FM_RAT, att_db = INS_HNTCH_ATT.  Single notches only (OPTS bits 0/4/6
// clear).  Slots beyond HAL_HNF_MAX_FILTERS are dropped, as in ArduPilot.
void plan_notches(const float* motor_hz, uint8_t n_motors, uint16_t harmonics_mask,
                  float min_hz, float gyro_rate_hz, float att_db, NotchPlan& out);

} // namespace nano_bdshot
