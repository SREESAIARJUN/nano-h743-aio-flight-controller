// Nano H743 AIO - dual ICM-42688-P synchronisation reference (Revision 11.0)
// Sreesai Arjun Engineering
//
// Host-testable reference model for the Rev 11 dual-IMU clocking scheme:
//   U2 (IMU1, SPI1) and U3 (IMU2, SPI4) are both clocked from Y2, a 32.768 kHz
//   +/-30 ppm CMOS oscillator, on pin 9 (INT2/FSYNC/CLKIN) through R14/R15 33R.
//   R16 1k taps the same net to PD12 (IMU_CLKIN_MON) so the MCU can see it.
//
// Five blocks, all allocation-free, no exceptions, fixed-size state:
//   a) Register plan builder: ordered (bank, addr, clear, set) list that
//      reproduces ArduPilot AP_InertialSensor_Invensensev3::set_filter_and_scaling()
//      for the ICM-42688 bit-for-bit, plus RTC/CLKIN enable (which upstream only
//      does for IIM-42652) and a loss-of-clock fallback plan.
//   b) 16-bit FIFO timestamp unwrapper + tick -> ns scaling (DS-000347 s12.7).
//   c) Dual-IMU sample pairer: nearest-neighbour pairing within +/-T/2 on a
//      COMMON time base, constant-phase estimate, relative drift (ppm) ->
//      "clock shared" / "clock not shared or RTC inactive".
//   d) CLKIN watchdog on the PD12 edge count: VALID / OFF_NOMINAL /
//      OUT_OF_RANGE / DEAD with debounce and a latched fallback decision.
//   e) Cross-IMU consistency voter (IMU2 ROTATION_PITCH_180 -> board frame).
//
// Datasheet references are to TDK DS-000347 rev 1.2 (04/19/2020), fetched from
// https://www.cdiweb.com/datasheets/invensense/ds-000347-icm-42688-p-v1.2.pdf
#pragma once

#include <stdint.h>
#include <stddef.h>

namespace nano_imu_sync {

// ---------------------------------------------------------------------------
// ICM-42688-P registers used here (DS-000347 s13/s14/s15, ArduPilot INV3REG_*)
// ---------------------------------------------------------------------------
namespace reg {
// bank 0
static constexpr uint8_t INT_CONFIG    = 0x14;   // bit4 INT2_DRIVE_CIRCUIT (1 = push-pull), reset 0x00
static constexpr uint8_t INTF_CONFIG1  = 0x4D;   // bit2 RTC_MODE, bits1:0 CLKSEL, reset 0x91
static constexpr uint8_t PWR_MGMT0     = 0x4E;
static constexpr uint8_t GYRO_CONFIG0  = 0x4F;   // [7:5] FS_SEL, [3:0] ODR
static constexpr uint8_t ACCEL_CONFIG0 = 0x50;
static constexpr uint8_t TMST_CONFIG   = 0x54;   // bit3 TMST_RES (0 = 1 us, 1 = 16 us)
static constexpr uint8_t BANK_SEL      = 0x76;
// bank 1
static constexpr uint8_t GYRO_CONFIG_STATIC2 = 0x0B;   // bit1 GYRO_AAF_DIS, bit0 GYRO_NF_DIS
static constexpr uint8_t GYRO_CONFIG_STATIC3 = 0x0C;   // GYRO_AAF_DELT
static constexpr uint8_t GYRO_CONFIG_STATIC4 = 0x0D;   // GYRO_AAF_DELTSQR[7:0]
static constexpr uint8_t GYRO_CONFIG_STATIC5 = 0x0E;   // BITSHIFT[7:4] | DELTSQR[11:8]
static constexpr uint8_t INTF_CONFIG5        = 0x7B;   // bits2:1 PIN9_FUNCTION
// bank 2
static constexpr uint8_t ACCEL_CONFIG_STATIC2 = 0x03;  // ACCEL_AAF_DELT[6:1] | ACCEL_AAF_DIS[0]
static constexpr uint8_t ACCEL_CONFIG_STATIC3 = 0x04;
static constexpr uint8_t ACCEL_CONFIG_STATIC4 = 0x05;
}  // namespace reg

static constexpr uint8_t INTF_CONFIG1_RTC_MODE     = 0x04;
// ArduPilot "stuck gyro" (AFSR) fix: (v & 0x3F) | 0x40  == clear 0xC0, set 0x40
static constexpr uint8_t INTF_CONFIG1_AFSR_MASK    = 0xC0;
static constexpr uint8_t INTF_CONFIG1_AFSR_DISABLE = 0x40;
static constexpr uint8_t PIN9_FUNCTION_MASK  = 0x06;
static constexpr uint8_t PIN9_FUNCTION_INT2  = 0x00;
static constexpr uint8_t PIN9_FUNCTION_FSYNC = 0x02;
static constexpr uint8_t PIN9_FUNCTION_CLKIN = 0x04;   // 0b10 << 1 (upstream writes | 0x04)
static constexpr uint8_t INT_CONFIG_INT2_PUSH_PULL = 0x10;
static constexpr uint8_t PWR_MGMT0_GYRO_ACCEL_LN   = 0x0F;
static constexpr uint16_t PWR_ON_DELAY_US          = 300;   // ArduPilot delay after PWR_MGMT0 = 0x0F

// CLKIN (DS-000347 Table 4 "External clock source": 31 / 32 typ / 50 kHz)
static constexpr uint32_t CLKIN_MIN_HZ     = 31000U;
static constexpr uint32_t CLKIN_MAX_HZ     = 50000U;
static constexpr uint32_t CLKIN_REF_HZ     = 32000U;   // ODR table is specified at 32 kHz (s12.5)
static constexpr uint32_t CLKIN_NOMINAL_HZ = 32768U;   // Y2 SX2M32.768KM20F30TNN
static constexpr uint32_t CLKIN_OSC_PPM    = 30U;      // Y2 frequency stability (LCSC C2901603)

// ---------------------------------------------------------------------------
// a) Register plan
// ---------------------------------------------------------------------------
struct AafCoeffs {
    uint8_t  delt;
    uint16_t deltsqr;
    uint8_t  bitshift;
    uint16_t bw_hz;   // -3 dB bandwidth quoted in the ArduPilot comment at 32 kHz CLK
};

// ArduPilot ICM-42688 mapping: only 2000/4000/8000 are special-cased, anything
// else uses the 1 kHz configuration (switch default in set_filter_and_scaling).
uint32_t normalise_backend_rate(uint32_t requested_hz);
uint8_t  odr_code_for_rate(uint32_t backend_rate_hz);      // GYRO/ACCEL_CONFIG0 value
AafCoeffs gyro_aaf_for_rate(uint32_t backend_rate_hz);
AafCoeffs accel_aaf();
// ODR and digital-filter corner scale with f_CLKIN / 32 kHz in RTC mode (s12.5).
double odr_hz_with_clkin(uint32_t nominal_odr_hz, double clkin_hz);

struct RegOp {
    uint8_t  bank;
    uint8_t  addr;
    uint8_t  clear_mask;   // 0xFF == plain write of set_bits
    uint8_t  set_bits;
    uint16_t delay_us;     // wait after the op
    bool    is_write() const { return clear_mask == 0xFFU; }
    uint8_t apply(uint8_t old) const { return uint8_t((old & uint8_t(~clear_mask)) | set_bits); }
};

struct RegPlan {
    static constexpr size_t MAX_OPS = 24;
    RegOp   op[MAX_OPS];
    uint8_t count = 0;
    bool    overflow = false;
    void clear() { count = 0; overflow = false; }
    bool write(uint8_t bank, uint8_t addr, uint8_t value, uint16_t delay_us = 0);
    bool modify(uint8_t bank, uint8_t addr, uint8_t clear_mask, uint8_t set_bits, uint16_t delay_us = 0);
    // last op touching (bank, addr), nullptr if none
    const RegOp* find(uint8_t bank, uint8_t addr) const;
};

enum class ClockMode : uint8_t { Internal = 0, RtcClkin = 1 };

struct PlanConfig {
    uint32_t  backend_rate_hz = 8000;   // INS_FAST_SAMPLE rate request
    ClockMode clock = ClockMode::RtcClkin;
};

// Full ICM-42688 configuration sequence, same order as ArduPilot:
//   PWR_MGMT0=0, GYRO_CONFIG0, ACCEL_CONFIG0, gyro AAF (bank1), accel AAF
//   (bank2), [RTC: INTF_CONFIG5 PIN9=CLKIN], INTF_CONFIG1, PWR_MGMT0=0x0F, 300 us.
void build_config_plan(const PlanConfig& cfg, RegPlan& out);

// Loss-of-clock fallback: sensors off, INTF_CONFIG1 RTC_MODE=0 (AFSR fix
// kept), INT_CONFIG INT2 forced open-drain (pin 9 is still wired to Y2 OUT
// through 33R - a push-pull INT2 would fight the oscillator), PIN9=INT2,
// sensors on.  Caller must FIFO-reset afterwards (ArduPilot fifo_reset()).
void build_fallback_plan(RegPlan& out);

// Minimal register bus so a plan can be executed on hardware or a model.
struct RegisterBus {
    void*   ctx;
    uint8_t (*read)(void* ctx, uint8_t addr);
    void    (*write)(void* ctx, uint8_t addr, uint8_t value);
    void    (*delay_us)(void* ctx, uint32_t us);
};
// Executes the plan; for bank != 0 ops it brackets with BANK_SEL=bank ... BANK_SEL=0
// (ArduPilot register_write_bank semantics).  Returns the number of SPI transfers.
uint32_t execute_plan(const RegPlan& plan, const RegisterBus& bus);

// ---------------------------------------------------------------------------
// b) FIFO timestamp
// ---------------------------------------------------------------------------
enum class TmstRes : uint8_t { Res1us = 0, Res16us = 1 };

// DS-000347 s12.7 "FIFO timestamp interval scaling": raw FIFO timestamp counts
// must be scaled by 32.768/30 us (RTC_MODE=1, PIN9=CLKIN) or 32/30 us
// (internal clock) per count; x16 with TMST_RES=1.  NOTE: this is NOT one CLKIN
// period (30.5 us) - the counter is ~1.09 us/count.
uint64_t tmst_ticks_to_ns(uint64_t ticks, ClockMode mode, TmstRes res);
// expected counts per ODR period in 1/256 count units (e.g. RTC 8192 Hz -> 111.76)
uint32_t tmst_ticks_per_sample_x256(double odr_hz, ClockMode mode, TmstRes res);

class TimestampUnwrapper {
public:
    struct Result {
        uint64_t unwrapped;   // monotonic tick count since first sample (+ first raw)
        uint16_t delta;       // modulo-2^16 delta from previous sample
        uint16_t missed;      // samples apparently skipped (delta ~ (k+1) * expected)
        bool     first;
        bool     duplicate;   // delta == 0
        bool     irregular;   // delta not within tolerance of an integer multiple
    };
    // expected_x256 = 0 disables gap/irregularity checks
    explicit TimestampUnwrapper(uint32_t expected_x256 = 0) : _expected_x256(expected_x256) {}
    void     reset() { _have = false; _acc = 0; _last = 0; _wraps = 0; }
    Result   update(uint16_t raw);
    uint64_t value() const { return _acc; }
    uint32_t wraps() const { return _wraps; }
private:
    uint32_t _expected_x256;
    bool     _have = false;
    uint16_t _last = 0;
    uint64_t _acc = 0;
    uint32_t _wraps = 0;
};

// ---------------------------------------------------------------------------
// c) Dual-IMU pairing
// ---------------------------------------------------------------------------
// t_ns MUST be on a time base common to both IMUs (MCU time of the DRDY EXTI
// edge on PC4/PE1, or FIFO timestamps mapped into MCU time).  Two IMUs each
// timestamping with their OWN oscillator would hide a drift between them.
struct ImuSample {
    uint64_t t_ns;
    float    gyro[3];    // rad/s, sensor frame
    float    accel[3];   // m/s^2, sensor frame
};

struct PairedSample {
    ImuSample imu1;
    ImuSample imu2;
    int64_t   offset_ns;   // t2 - t1, in (-T/2, +T/2]
};

enum class SyncState : uint8_t { Acquiring = 0, Shared = 1, NotShared = 2 };

struct PairerConfig {
    uint32_t odr_period_ns       = 122070;   // 1e9 / 8192 Hz (8 kHz x 32.768/32)
    uint32_t window_pairs        = 8192;     // regression window (~1 s at 8192 Hz)
    double   drift_threshold_ppm = 10.0;     // shared CLKIN => 0 ppm exactly
};

class DualImuPairer {
public:
    static constexpr size_t QUEUE = 64;
    explicit DualImuPairer(const PairerConfig& cfg = PairerConfig()) : _cfg(cfg) {}
    void reset();
    // imu = 0 (IMU1) or 1 (IMU2).  Returns false if the queue was full (the
    // oldest sample was dropped and counted in overflow()).
    bool push(uint8_t imu, const ImuSample& s);
    bool pop_pair(PairedSample& out);

    SyncState state() const { return _state; }
    double    drift_ppm() const { return _drift_ppm; }        // last completed window
    double    phase_offset_ns() const { return _phase_ns; }   // last completed window, wrapped
    uint32_t  windows() const { return _windows; }
    uint64_t  pairs() const { return _pairs; }
    uint64_t  orphans(uint8_t imu) const { return _orphans[imu & 1U]; }
    uint64_t  overflow(uint8_t imu) const { return _overflow[imu & 1U]; }
private:
    struct Queue {
        ImuSample buf[QUEUE];
        size_t head = 0, n = 0;
        const ImuSample& at(size_t i) const { return buf[(head + i) % QUEUE]; }
        void pop() { head = (head + 1) % QUEUE; n--; }
    };
    void account(int64_t offset_ns, uint64_t t1_ns);
    PairerConfig _cfg;
    Queue    _q[2];
    uint64_t _pairs = 0, _orphans[2] = {0, 0}, _overflow[2] = {0, 0};
    // continuous (unwrapped) offset for regression
    bool     _have_prev = false;
    double   _prev_u = 0.0;
    // window sums (x in s relative to window start, y in ns)
    uint64_t _x0_ns = 0;
    double   _y0 = 0.0;
    uint32_t _n = 0;
    double   _sx = 0, _sy = 0, _sxx = 0, _sxy = 0;
    double   _drift_ppm = 0.0, _phase_ns = 0.0;
    uint32_t _windows = 0;
    SyncState _state = SyncState::Acquiring;
};

// ---------------------------------------------------------------------------
// d) CLKIN watchdog (PD12 edge count over a window)
// ---------------------------------------------------------------------------
enum class ClockStatus : uint8_t {
    Unknown = 0,
    Valid,        // within nominal +/- allowed_ppm
    OffNominal,   // inside the IMU's 31-50 kHz range but outside ppm tolerance: warn only
    OutOfRange,   // outside 31-50 kHz (but toggling): IMU ODR invalid -> fallback
    Dead,         // < DEAD_BELOW_HZ: oscillator stopped / stuck / OE low -> fallback
};

struct ClockWatchdogConfig {
    uint32_t nominal_hz      = CLKIN_NOMINAL_HZ;
    uint32_t tol_ppm         = 500;     // see imu_sync.cpp for the budget
    uint32_t min_hz          = CLKIN_MIN_HZ;
    uint32_t max_hz          = CLKIN_MAX_HZ;
    uint32_t dead_below_hz   = 1000;
    uint32_t min_window_us   = 1000;    // shorter windows are ignored
    uint8_t  fail_windows    = 2;       // consecutive bad windows to assert a fault
    uint8_t  recover_windows = 10;      // consecutive good windows to report Valid again
};

class ClockWatchdog {
public:
    struct Report {
        ClockStatus raw;          // this window only
        ClockStatus status;       // debounced
        double      freq_hz;
        double      err_ppm;
        double      allowed_ppm;  // tol_ppm + 1-edge quantisation of this window
        bool        apply_fallback;   // true exactly once, on the window the fault is confirmed
        bool        fallback_latched; // stays true: RTC is never re-enabled in flight
    };
    explicit ClockWatchdog(const ClockWatchdogConfig& cfg = ClockWatchdogConfig()) : _cfg(cfg) {}
    Report update(uint32_t rising_edges, uint32_t window_us);
    ClockStatus status() const { return _status; }
    bool fallback_latched() const { return _latched; }
    // Clearing the latch is a ground-only action (re-init both IMUs, FIFO reset).
    void clear_latch_on_ground() { _latched = false; }
private:
    ClockWatchdogConfig _cfg;
    ClockStatus _status = ClockStatus::Unknown;
    ClockStatus _last_bad = ClockStatus::Unknown;
    uint8_t _bad = 0, _good = 0;
    bool _latched = false;
};

// ---------------------------------------------------------------------------
// e) Cross-IMU consistency voter
// ---------------------------------------------------------------------------
// ArduPilot ROTATION_PITCH_180: (x, y, z) -> (-x, y, -z)
void rotate_pitch_180(const float in[3], float out[3]);

enum class Suspect : uint8_t { None = 0, Imu1, Imu2, Unknown };

// fault/exceed mask bits
static constexpr uint8_t VOTE_GYRO_X  = 0x01, VOTE_GYRO_Y  = 0x02, VOTE_GYRO_Z  = 0x04;
static constexpr uint8_t VOTE_ACCEL_X = 0x08, VOTE_ACCEL_Y = 0x10, VOTE_ACCEL_Z = 0x20;

struct VoterConfig {
    float    gyro_thresh_rad_s   = 0.0872665f;  // 5 deg/s  (AP_Arming ins_gyros_consistent)
    float    accel_thresh_m_s2   = 0.75f;       // AP_ARMING_ACCEL_ERROR_THRESHOLD
    uint8_t  average_n           = 8;           // pairs averaged per comparison (8192 -> 1024 Hz)
    uint16_t persist_blocks      = 8;           // consecutive exceeding blocks -> fault
    uint16_t clear_blocks        = 32;          // consecutive passing blocks -> clear
    float    gyro_sat_rad_s      = 34.8f;       // ~0.997 x 2000 dps full scale
    float    accel_sat_m_s2      = 156.0f;      // ~0.995 x 16 g full scale
    uint16_t stuck_samples       = 16;          // bit-identical gyro for >= 2 ms at 8 kHz
};

struct VoteResult {
    bool    evaluated;     // a block completed on this call
    uint8_t exceed_mask;   // this block
    uint8_t fault_mask;    // persisted
    Suspect suspect;
    float   diff[6];       // board-frame IMU1 - IMU2, gyro xyz then accel xyz
};

class ConsistencyVoter {
public:
    explicit ConsistencyVoter(const VoterConfig& cfg = VoterConfig()) : _cfg(cfg) {}
    void reset();
    VoteResult update(const PairedSample& p);   // IMU2 data in its sensor frame
    uint8_t fault_mask() const { return _fault; }
private:
    VoterConfig _cfg;
    float    _sum[6] = {0, 0, 0, 0, 0, 0};
    uint8_t  _k = 0;
    uint16_t _exc[6] = {0, 0, 0, 0, 0, 0};
    uint16_t _ok[6]  = {0, 0, 0, 0, 0, 0};
    uint8_t  _fault = 0;
    // per-IMU health evidence within the current block
    float    _last_gyro[2][3] = {{0, 0, 0}, {0, 0, 0}};
    uint16_t _stuck[2] = {0, 0};
    bool     _stuck_flag[2] = {false, false};
    bool     _sat_flag[2] = {false, false};
    bool     _have_last = false;
    Suspect  _suspect = Suspect::None;
};

}  // namespace nano_imu_sync
