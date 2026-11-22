#include "motor_monitor.h"

bool MotorMonitor::update(const MotorMonitorInputs& in, uint32_t now_ms) {
    if (!in.current_valid) {
        // No feedback: nothing to judge. A missing PM02 is its own problem.
        m_waiting = false;
        return false;
    }
    const bool commanded = in.throttle > m_params.throttle_on;
    if (!commanded) {
        m_waiting = false;
        // Current with the throttle at zero: ESC or wiring fault.
        return in.throttle <= 0.0f &&
               in.current_a > m_params.current_idle_max_a;
    }
    if (in.current_a >= m_params.current_min_a) {
        m_waiting = false;
        return false;
    }
    if (!m_waiting) {
        m_waiting = true;
        m_wait_start_ms = now_ms;
    }
    return (now_ms - m_wait_start_ms) >= m_params.response_ms;
}
