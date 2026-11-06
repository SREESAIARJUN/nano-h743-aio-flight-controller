// Nano H743 AIO - DPS310 barometer driver core with I2C bus-noise immunity (Revision 12.0)
// Sreesai Arjun Engineering
//
// Host-testable reference for U4 (Infineon DPS310, LGA-8 2.0 x 2.5 mm) on I2C1
// (PB6 SCL / PB7 SDA, 400 kHz Fast-mode, 7-bit address 0x76: CSB = VDDIO, SDO = GND).
// The bus is shared with the external compass on the GPS cable, so the driver
// assumes the bus WILL see bit errors, NACK bursts, arbitration loss and a
// slave holding SDA low, and that U4 can brown out (FB4-filtered +3V3_BARO)
// and lose its volatile configuration.
//
// What it reproduces from ArduPilot AP_Baro_DPS280.cpp (DPS310 path):
//   - PRODUCT_ID 0x0D == 0x10 probe, 18-byte coefficient block at 0x10,
//     12/20/16-bit coefficient sign extension, COEF_SRCE (0x28 bit 7) copied
//     into TMP_CFG bit 7, the undocumented temperature-fix write sequence
//     0x0E=A5, 0x0F=96, 0x62=02, 0x0E=00, 0x0F=00, background mode MEAS_CFG=7,
//     the datasheet compensation polynomial.
// What it adds (where upstream is fragile, see test_dps310_robust.cpp T12):
//   - two independent coefficient reads that must match bit-exactly, COEF_RDY
//     and SENSOR_RDY gating, sanity limits; upstream reads the block once.
//   - the boot WHOAMI quirk: upstream toggles CS, which is a no-op on I2C
//     (AP_HAL::Device::set_chip_select() returns false). Here: re-read, then
//     soft reset (RESET=0x09) and re-probe.
//   - scale factors kP/kT selected from the configured oversampling (upstream
//     hard-codes 253952 = 16x) and a background-mode measurement-time budget
//     check (upstream 32 Hz x16 P + 32 Hz x16 T = 1766 ms/s > 1000 ms/s).
//   - config read-back on every poll (PRS_CFG/TMP_CFG/MEAS_CFG/CFG_REG are
//     one 4-byte burst at 0x06), double-read confirmation, immediate repair.
//   - bounded per-transaction retries, consecutive-failure counter, a UM10204
//     s3.1.16 bus clear via an abstract GPIO hook (<= 9 SCL pulses with SDA
//     released while watching SDA; as soon as SDA reads high with SCL high, a
//     START+STOP is made on SDA alone so a slave-transmitter hung mid-read gets
//     no extra SCL falling edge to shift out a 0 that would kill the STOP; the
//     released master-ACK slot is a NACK that ends a read, and stopping at the
//     first SDA-high avoids re-entering a receiver's next ACK slot), then full
//     re-init including coefficient re-verify.
//   - range / rate-of-change / single-sample-spike plausibility with a
//     re-anchor rule so a real step is followed in a bounded number of samples.
//
// Allocation-free, no exceptions, fixed-size state.
#pragma once

#include <stdint.h>
#include <stddef.h>

namespace nano_baro {

// ---------------------------------------------------------------------------
// DPS310 register map (Infineon DPS310 datasheet v1.1 s7)
// ---------------------------------------------------------------------------
namespace reg {
static constexpr uint8_t PSR_B2    = 0x00;   // 24-bit pressure, MSB first
static constexpr uint8_t TMP_B2    = 0x03;   // 24-bit temperature, MSB first
static constexpr uint8_t PRS_CFG   = 0x06;   // [6:4] PM_RATE, [3:0] PM_PRC
static constexpr uint8_t TMP_CFG   = 0x07;   // [7] TMP_EXT, [6:4] TMP_RATE, [3:0] TMP_PRC
static constexpr uint8_t MEAS_CFG  = 0x08;   // [7] COEF_RDY [6] SENSOR_RDY [5] TMP_RDY [4] PRS_RDY [2:0] MEAS_CTRL
static constexpr uint8_t CFG_REG   = 0x09;   // [3] T_SHIFT [2] P_SHIFT [1] FIFO_EN
static constexpr uint8_t INT_STS   = 0x0A;
static constexpr uint8_t FIFO_STS  = 0x0B;
static constexpr uint8_t RESET     = 0x0C;   // [3:0] = 1001 -> soft reset
static constexpr uint8_t PRODUCT_ID = 0x0D;  // REV_ID[7:4] | PROD_ID[3:0] = 0x10
static constexpr uint8_t COEF      = 0x10;   // 18 bytes 0x10..0x21
static constexpr uint8_t COEF_SRCE = 0x28;   // [7] TMP_COEF_SRCE
// undocumented temperature-fix registers (ArduPilot / Infineon DpsClass.cpp)
static constexpr uint8_t FIX_A     = 0x0E;
static constexpr uint8_t FIX_B     = 0x0F;
static constexpr uint8_t FIX_C     = 0x62;
}
static constexpr uint8_t PRODUCT_ID_VALUE = 0x10;
static constexpr uint8_t SOFT_RESET_CMD   = 0x09;
static constexpr uint8_t MEAS_COEF_RDY    = 0x80;
static constexpr uint8_t MEAS_SENSOR_RDY  = 0x40;
static constexpr uint8_t MEAS_TMP_RDY     = 0x20;
static constexpr uint8_t MEAS_PRS_RDY     = 0x10;
static constexpr uint8_t MEAS_CTRL_MASK   = 0x07;
static constexpr uint8_t MEAS_CTRL_BG_PT  = 0x07;   // continuous pressure + temperature
static constexpr uint8_t CFG_T_SHIFT      = 0x08;
static constexpr uint8_t CFG_P_SHIFT      = 0x04;
static constexpr uint8_t I2C_ADDR_SDO_GND = 0x76;

// ---------------------------------------------------------------------------
// Pure helpers
// ---------------------------------------------------------------------------
// Two's-complement sign extension of the low `bits` bits of v (1..32). No UB.
int32_t sign_extend(uint32_t v, unsigned bits);
// 24-bit MSB-first two's complement
int32_t raw24(const uint8_t* b);
// Compensation scale factor kP/kT for oversampling code 0..7 (1x..128x), datasheet Table 9.
int32_t scale_factor(uint8_t prc);
// Single measurement time in microseconds for oversampling code (datasheet Table 16).
uint32_t measurement_time_us(uint8_t prc);
// Shift bit is mandatory when oversampling > 8x (code > 3).
inline bool needs_shift(uint8_t prc) { return prc > 3; }
// Background-mode budget: sum(rate * t_meas) in microseconds per second.
uint32_t background_budget_us(uint8_t pm_rate, uint8_t pm_prc, uint8_t tmp_rate, uint8_t tmp_prc);

struct Coefficients {
    int32_t c0 = 0, c1 = 0;              // 12 bit
    int32_t c00 = 0, c10 = 0;            // 20 bit
    int32_t c01 = 0, c11 = 0, c20 = 0, c21 = 0, c30 = 0;   // 16 bit
    bool operator==(const Coefficients& o) const;
    bool operator!=(const Coefficients& o) const { return !(*this == o); }
};
Coefficients parse_coefficients(const uint8_t b[18]);
void encode_coefficients(const Coefficients& c, uint8_t b[18]);   // inverse, for simulators
// Sanity limits on a parsed block (see .cpp for the rationale of each).
bool coefficients_sane(const uint8_t b[18], const Coefficients& c);

// Datasheet s4.9: Traw_sc = Traw/kT, Praw_sc = Praw/kP,
//   T = c0/2 + c1*Traw_sc
//   P = c00 + Praw_sc*(c10 + Praw_sc*(c20 + Praw_sc*c30)) + Traw_sc*c01 + Traw_sc*Praw_sc*(c11 + Praw_sc*c21)
// Evaluated in float in the same order as ArduPilot calculate_PT().
void compensate(const Coefficients& c, int32_t praw, int32_t traw, int32_t kp, int32_t kt,
                float& pressure_pa, float& temperature_c);

// ---------------------------------------------------------------------------
// Abstract hardware hooks
// ---------------------------------------------------------------------------
enum class I2cStatus : uint8_t { OK = 0, NACK, BUS_ERROR, ARB_LOST, TIMEOUT };
const char* to_string(I2cStatus s);

// One I2C transaction to the DPS310 address. read_registers = START, addr+W,
// reg, rSTART, addr+R, len bytes, STOP. No retries inside: policy lives in Dps310.
class I2cTransport {
public:
    virtual ~I2cTransport() = default;
    virtual I2cStatus read_registers(uint8_t first_reg, uint8_t* buf, size_t len) = 0;
    virtual I2cStatus write_register(uint8_t reg, uint8_t value) = 0;
    virtual void delay_us(uint32_t us) = 0;
};

// Open-drain line control for bus clear. On STM32H7: begin() switches PB6/PB7
// to GPIO open-drain output (both released), end() returns them to AF4 and
// resets the I2C1 peripheral (PE=0, 3 APB cycles, PE=1).
class I2cBusGpio {
public:
    virtual ~I2cBusGpio() = default;
    virtual void begin_bitbang() = 0;
    virtual void end_bitbang() = 0;
    virtual void set_scl(bool released) = 0;   // false = drive low
    virtual void set_sda(bool released) = 0;
    virtual bool read_scl() = 0;
    virtual bool read_sda() = 0;
    virtual void delay_us(uint32_t us) = 0;
};

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------
struct Config {
    // measurement plan (codes as in the registers)
    uint8_t pm_rate  = 5;   // 32 Hz
    uint8_t pm_prc   = 4;   // 16x, 27.6 ms, kP = 253952, P_SHIFT
    uint8_t tmp_rate = 2;   // 4 Hz
    uint8_t tmp_prc  = 0;   // 1x, 3.6 ms, kT = 524288
    // transaction policy
    uint8_t  max_attempts      = 3;     // 1 + 2 retries (same as ArduPilot I2CDevice _retries = 2)
    uint8_t  fail_threshold    = 3;     // consecutive failed transactions -> bus recovery
    uint8_t  coef_max_reads    = 6;     // sliding window, any 2 consecutive identical reads accepted
    uint8_t  probe_reads       = 4;     // PRODUCT_ID reads before the soft-reset quirk path
    uint32_t recovery_holdoff_us = 100000;
    uint32_t clear_half_period_us = 5;  // 100 kHz bit-bang clock (Sm, insensitive to bus C)
    // plausibility
    float p_min_pa = 30000.0f, p_max_pa = 120000.0f;     // 300 .. 1200 hPa
    float t_min_c  = -45.0f,   t_max_c  = 90.0f;         // DPS310 -40..85 C + margin
    float max_vz_mps = 40.0f;                            // fastest credible climb/dive
    float gate_margin_pa = 8.0f;                         // noise + prop-wash allowance
    uint8_t reanchor_samples = 3;                        // consistent out-of-gate samples to re-anchor
};

// dP/dh upper bound used by the rate gate: rho_max * g with rho_max at
// p_max_pa and t_min_c (dry air). 120 kPa, -45 C -> 17.97 Pa/m.
float max_dp_dh(const Config& c);

enum Flag : uint32_t {
    F_PRESENT        = 1u << 0,
    F_COEF_VALID     = 1u << 1,
    F_CONFIG_OK      = 1u << 2,
    F_RUNNING        = 1u << 3,
    F_RECOVERING     = 1u << 4,
    F_RECOVERY_FAIL  = 1u << 5,
    F_HEALTHY        = 1u << 6,   // RUNNING and last sample accepted within 4 periods
    F_COEF_CHANGED   = 1u << 7,   // re-verify after recovery read different (but self-consistent) coefs
    F_BUDGET_FAIL    = 1u << 8,   // Config violates background-mode budget or shift rule
};

struct Health {
    uint32_t flags = 0;
    // transport
    uint32_t xfer_ok = 0, nack = 0, bus_error = 0, arb_lost = 0, timeout = 0;
    uint32_t retries = 0, xfer_failed = 0, consec_failed = 0, max_consec_failed = 0;
    // recovery
    uint32_t recoveries = 0, recovery_failed = 0, last_clear_clocks = 0, sda_stuck_seen = 0;
    // init / integrity
    uint32_t inits = 0, init_failed = 0, whoami_bad = 0, quirk_resets = 0;
    uint32_t coef_reads = 0, coef_mismatch = 0, coef_insane = 0, coef_not_ready = 0;
    uint32_t config_writes = 0, config_readback_fail = 0, config_lost = 0, config_repairs = 0;
    // samples
    uint32_t polls = 0, not_ready = 0, samples = 0, accepted = 0;
    uint32_t rej_range_p = 0, rej_range_t = 0, rej_rate = 0, rej_pattern = 0, reanchors = 0;
};

struct Sample {
    float pressure_pa = 0;
    float temperature_c = 0;
    uint32_t t_us = 0;
    bool valid = false;
};

enum class Update : uint8_t { NEW_SAMPLE, NO_DATA, REJECTED, NOT_RUNNING, BUS_FAULT };
const char* to_string(Update u);

enum class DriverState : uint8_t { UNINIT, RUNNING, HOLDOFF };

class Dps310 {
public:
    Dps310(I2cTransport& bus, I2cBusGpio* gpio, const Config& cfg = Config());

    // Full bring-up: probe (+boot quirk), coefficients (dual read), COEF_SRCE,
    // temperature fix, config + read-back, background mode.
    bool init(uint32_t now_us = 0);
    // Call at >= 2x the pressure rate. Handles recovery / re-init internally.
    Update update(uint32_t now_us);

    const Health& health() const { return h_; }
    const Sample& last() const { return last_; }
    const Coefficients& coefficients() const { return coef_; }
    DriverState state() const { return state_; }
    int32_t kp() const { return kp_; }
    int32_t kt() const { return kt_; }
    // Expected register images after configuration.
    uint8_t prs_cfg_value() const;
    uint8_t tmp_cfg_value() const;
    uint8_t cfg_reg_value() const;
    // Bus clear (public so the HAL can call it on a TIMEOUT with SDA low).
    // <= 9 SCL clocks with SDA released until SDA reads high, then START+STOP
    // with SCL held high (no SCL edge between the observation and the STOP).
    // Worst case: slave-transmitter at the MSB of 0x00 -> 8 clocks; slave in a
    // write/address ACK slot -> 1 clock; receiver mid-byte -> 0 clocks.
    // Returns true only if the bus is idle (SDA and SCL high) after the STOP.
    // health().last_clear_clocks = SCL pulses generated.
    bool recover_bus();

private:
    I2cStatus rd(uint8_t r, uint8_t* b, size_t n);
    I2cStatus wr(uint8_t r, uint8_t v);
    void note_failure(I2cStatus st);
    bool probe();
    bool read_coefficients(Coefficients& out, uint8_t& srce);
    bool apply_temperature_fix();
    bool write_config_verified();
    bool config_matches(const uint8_t cfg4[4]) const;
    Update plausibility(float p, float t, uint32_t now_us);
    float gate_pa(uint32_t dt_us) const;

    I2cTransport& bus_;
    I2cBusGpio* gpio_;
    Config cfg_;
    Health h_;
    Sample last_;
    Coefficients coef_;
    bool have_coef_ = false;
    uint8_t temp_src_ = 0;
    int32_t kp_ = 0, kt_ = 0;
    DriverState state_ = DriverState::UNINIT;
    bool need_recovery_ = false;
    uint32_t holdoff_until_ = 0;
    uint32_t period_us_ = 0;
    // plausibility filter
    bool ref_valid_ = false;
    float ref_p_ = 0;
    uint32_t ref_t_ = 0;
    uint8_t pend_n_ = 0;
    float pend_p_ = 0;
    uint32_t pend_t_ = 0;
    bool reanchor_needs_verify_ = false;
};

} // namespace nano_baro
