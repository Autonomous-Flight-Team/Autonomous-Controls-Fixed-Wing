#ifndef ACTUATORS_H_
#define ACTUATORS_H_

#include "demand_select.h"
#include "flight_state.h"
#include "mixer.h"
#include "output_channel.h"
#include "pilot_input.h"
#include "pwm_out.h"
#include "surface_demand.h"
#include "system_state.h"
#include "throttle_gate.h"
#include <cstdint>

// The output pipeline: state machines + inputs -> µs per channel.
// Pure (no pins): main writes the frame with pwm_out::write_us().
// It holds no armed/mode state; everything follows from SystemState,
// FlightState and manual_active each loop.

struct ActuatorConfig {
    ChannelConfig channels[pwm_out::kMaxChannels]{};
    uint8_t num_channels = 0;
    MixerConfig mixer{}; // one row per channel, same order
    EscConfig esc{};
};

// Exactly one throttle channel, every channel valid, one mixer row per
// channel, ESC stop value = throttle safe_us.
bool actuator_config_valid(const ActuatorConfig& cfg);

// Bench setpoints as plain data (filled from BenchSetpoints by main).
struct BenchFrame {
    uint16_t us[pwm_out::kMaxChannels] = {};
    uint8_t set_mask = 0; // bit ch = us[ch] is set; unset -> trim
};

struct ActuatorInputs {
    SystemState system = SystemState::kBoot;
    FlightState flight = FlightState::kOnGround;
    bool manual_active = false;
    const PilotInput* rc = nullptr; // latest frame (may be stale)
    bool rc_fresh = false;
    const SurfaceDemand* control = nullptr; // nullptr until control/ exists
    BenchFrame bench{};
};

struct OutputFrame {
    uint16_t us[pwm_out::kMaxChannels] = {};
    uint8_t num_channels = 0;
    OutputSource source = OutputSource::kSafe;
    ThrottleCmd throttle{}; // what the throttle channel was told
};

// For comms telemetry (ServoOutputSample -> SERVO_OUTPUT_RAW): copies the
// µs actually sent. Returns the count (at most cap).
uint8_t output_telemetry_us(const OutputFrame& f, uint16_t* us, uint8_t cap);

class Actuators {
  public:
    // cfg must pass actuator_config_valid(); otherwise every update()
    // returns safe values.
    explicit Actuators(const ActuatorConfig& cfg);

    Actuators(const Actuators&) = delete;
    Actuators& operator=(const Actuators&) = delete;

    // Call once per loop, after the FSM updates.
    void update(const ActuatorInputs& in, uint32_t now_ms, OutputFrame* out);

    // Safe values for pwm_out::begin().
    void safe_us(uint16_t* out) const;

    bool config_ok() const { return m_ok; }
    // For PREFLIGHT: config valid and the ESC has seen stop for arm_hold_ms.
    bool preflight_ok(uint32_t now_ms) const;
    int8_t throttle_channel() const { return m_thr; }

  private:
    void fill_safe(OutputFrame* out) const;
    void fill_mirror(const ActuatorInputs& in, OutputFrame* out) const;
    void fill_bench(const ActuatorInputs& in, OutputFrame* out) const;
    void fill_control(const ActuatorInputs& in, uint32_t now_ms,
                      OutputFrame* out);
    uint16_t stop_us() const;

    ActuatorConfig m_cfg;
    OutputChannel m_ch[pwm_out::kMaxChannels];
    EscArmTracker m_esc_arm{};
    int8_t m_thr = -1; // throttle channel index
    bool m_ok = false;
};

#endif // ACTUATORS_H_
