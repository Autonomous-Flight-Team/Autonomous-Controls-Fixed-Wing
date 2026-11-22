#include "throttle_gate.h"

bool throttle_allowed(SystemState sys, FlightState flight) {
    if (!is_armed(sys)) {
        return false;
    }
    switch (flight) {
    case FlightState::kOnGround:
        return sys == SystemState::kActive;
    case FlightState::kLanded:
    case FlightState::kRejectedTakeoff:
        return false;
    case FlightState::kTakeoffRoll:
    case FlightState::kRotate:
    case FlightState::kClimb:
    case FlightState::kCruise:
    case FlightState::kLoiter:
    case FlightState::kRtl:
    case FlightState::kDescent:
    case FlightState::kLanding:
    case FlightState::kFailsafeLoiter:
    case FlightState::kEmergencyLand:
        return true;
    }
    return false;
}

ThrottleCmd gate_throttle(float demand, SystemState sys, FlightState flight) {
    ThrottleCmd cmd;
    cmd.armed = throttle_allowed(sys, flight);
    if (!cmd.armed) {
        return cmd; // throttle stays 0
    }
    if (demand < 0.0f) {
        demand = 0.0f;
    } else if (demand > 1.0f) {
        demand = 1.0f;
    }
    cmd.throttle = demand;
    return cmd;
}

uint16_t esc_stop_us(const EscConfig& esc, const ChannelConfig& ch) {
    if (esc.bidirectional) {
        return static_cast<uint16_t>((ch.min_us + ch.max_us) / 2);
    }
    return ch.min_us;
}

uint16_t esc_us(const EscConfig& esc, const ChannelConfig& ch,
                const ThrottleCmd& cmd) {
    const uint16_t stop = esc_stop_us(esc, ch);
    if (!cmd.armed) {
        return stop;
    }
    float t = cmd.throttle;
    if (t < 0.0f) {
        t = 0.0f;
    } else if (t > 1.0f) {
        t = 1.0f;
    }
    const float span = static_cast<float>(ch.max_us) - stop;
    return static_cast<uint16_t>(stop + t * span + 0.5f);
}

bool esc_config_valid(const EscConfig& esc, const ChannelConfig& ch) {
    if (ch.kind != ChannelKind::kThrottle || ch.min_us >= ch.max_us) {
        return false;
    }
    return ch.safe_us == esc_stop_us(esc, ch);
}

void EscArmTracker::update(uint16_t sent_us, uint16_t stop_us,
                           uint32_t now_ms) {
    const bool at_stop = sent_us == stop_us;
    if (at_stop && !m_at_stop) {
        m_stop_since_ms = now_ms;
    }
    m_at_stop = at_stop;
}

bool EscArmTracker::ready(const EscConfig& esc, uint32_t now_ms) const {
    return m_at_stop && (now_ms - m_stop_since_ms) >= esc.arm_hold_ms;
}
