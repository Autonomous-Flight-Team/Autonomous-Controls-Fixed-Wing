#ifndef MOTOR_MONITOR_H_
#define MOTOR_MONITOR_H_

#include <cstdint>

// Checks the motor answers the throttle, from whatever feedback exists.
// Only produces the raw FaultId::kMotorEsc bit; FaultManager debounces and
// escalates, the FSMs respond.

// PLACEHOLDER thresholds: set from bench and flight measurements.
struct MotorMonitorParams {
    float throttle_on = 0.2f;      // above this, the motor must draw current
    float current_min_a = 2.0f;    // battery current expected above throttle_on
    float current_idle_max_a = 4.0f; // at zero throttle (avionics + servos)
    uint32_t response_ms = 500;    // time allowed for current to rise
};

struct MotorMonitorInputs {
    float throttle = 0.0f;      // gated command actually sent, [0, 1]
    bool current_valid = false; // PM02 sample usable
    float current_a = 0.0f;     // total battery current
    bool rpm_valid = false;     // reserved: DShot telemetry
    float rpm = 0.0f;
};

class MotorMonitor {
  public:
    explicit MotorMonitor(const MotorMonitorParams& p = MotorMonitorParams{})
        : m_params(p) {}

    // Returns the raw fault bit for this loop: true = condition present.
    bool update(const MotorMonitorInputs& in, uint32_t now_ms);

  private:
    MotorMonitorParams m_params;
    bool m_waiting = false;    // throttle above throttle_on, no current yet
    uint32_t m_wait_start_ms = 0;
};

#endif // MOTOR_MONITOR_H_
