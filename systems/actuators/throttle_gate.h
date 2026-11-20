#ifndef THROTTLE_GATE_H_
#define THROTTLE_GATE_H_

#include "flight_state.h"
#include "output_channel.h"
#include "system_state.h"
#include <cstdint>

// Throttle authority, read only from the state machines. There is no armed
// state here: arming belongs to SystemStateFSM.

// Protocol-neutral motor command. Backends (PWM now, DShot later) translate.
struct ThrottleCmd {
    bool armed = false;    // false = send the protocol's "stop"
    float throttle = 0.0f; // [0, 1]
};

// Values are logged: never renumber, only add.
enum class EscProtocol : uint8_t {
    kPwm = 0, // 50 Hz servo-style pulse; only one implemented
};

struct EscConfig {
    EscProtocol protocol = EscProtocol::kPwm;
    bool bidirectional = false;  // 3D ESC: midpoint = stop (forward only here)
    uint32_t arm_hold_ms = 3000; // "stop" this long before the ESC is ready
    bool needs_range_cal = true; // PWM ESCs learn endpoints; DShot doesn't
};

// The FSMs' own rules: armed, and the flight state lets the motor run.
// ON_GROUND keeps the motor locked unless the system is ACTIVE;
// LANDED and REJECTED_TAKEOFF cut it.
bool throttle_allowed(SystemState sys, FlightState flight);

ThrottleCmd gate_throttle(float demand, SystemState sys, FlightState flight);

// PWM-family translation, using the throttle channel's min/max.
uint16_t esc_stop_us(const EscConfig& esc, const ChannelConfig& ch);
uint16_t esc_us(const EscConfig& esc, const ChannelConfig& ch,
                const ThrottleCmd& cmd);

// The channel's safe_us must be the ESC's stop value.
bool esc_config_valid(const EscConfig& esc, const ChannelConfig& ch);

// Tracks whether the ESC has seen "stop" for arm_hold_ms (feeds preflight).
class EscArmTracker {
  public:
    // Call every loop with the µs actually sent to the ESC.
    void update(uint16_t sent_us, uint16_t stop_us, uint32_t now_ms);
    bool ready(const EscConfig& esc, uint32_t now_ms) const;

  private:
    bool m_at_stop = false;
    uint32_t m_stop_since_ms = 0;
};

#endif // THROTTLE_GATE_H_
