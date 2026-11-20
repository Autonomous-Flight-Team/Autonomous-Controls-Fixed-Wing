#include "actuators.h"
#include "pilot.h"

namespace {

int8_t find_throttle(const ActuatorConfig& cfg) {
    int8_t found = -1;
    for (uint8_t i = 0; i < cfg.num_channels; ++i) {
        if (cfg.channels[i].kind != ChannelKind::kThrottle) {
            continue;
        }
        if (found >= 0) {
            return -2; // more than one
        }
        found = static_cast<int8_t>(i);
    }
    return found;
}

} // namespace

bool actuator_config_valid(const ActuatorConfig& cfg) {
    if (cfg.num_channels == 0 || cfg.num_channels > pwm_out::kMaxChannels) {
        return false;
    }
    if (cfg.mixer.num_rows != cfg.num_channels) {
        return false;
    }
    for (uint8_t i = 0; i < cfg.num_channels; ++i) {
        if (!channel_config_valid(cfg.channels[i])) {
            return false;
        }
    }
    const int8_t thr = find_throttle(cfg);
    if (thr < 0) {
        return false;
    }
    return esc_config_valid(cfg.esc, cfg.channels[thr]);
}

Actuators::Actuators(const ActuatorConfig& cfg) : m_cfg(cfg) {
    m_ok = actuator_config_valid(cfg);
    m_thr = m_ok ? find_throttle(cfg) : -1;
    if (!m_ok) {
        m_cfg.num_channels = 0; // nothing to drive with a bad config
    }
    for (uint8_t i = 0; i < m_cfg.num_channels; ++i) {
        m_ch[i] = OutputChannel(m_cfg.channels[i]);
    }
}

uint16_t Actuators::stop_us() const {
    return esc_stop_us(m_cfg.esc, m_cfg.channels[m_thr]);
}

void Actuators::safe_us(uint16_t* out) const {
    for (uint8_t i = 0; i < m_cfg.num_channels; ++i) {
        out[i] = m_cfg.channels[i].safe_us;
    }
}

bool Actuators::preflight_ok(uint32_t now_ms) const {
    return m_ok && m_esc_arm.ready(m_cfg.esc, now_ms);
}

void Actuators::fill_safe(OutputFrame* out) const {
    safe_us(out->us);
    out->throttle = ThrottleCmd{};
}

void Actuators::fill_mirror(const ActuatorInputs& in,
                            OutputFrame* out) const {
    fill_safe(out); // channels the receiver doesn't cover stay safe
    if (in.rc == nullptr) {
        return;
    }
    const bool armed = is_armed(in.system);
    for (uint8_t i = 0; i < m_cfg.num_channels; ++i) {
        uint16_t us = 0;
        const int8_t rc_ch = m_cfg.channels[i].rc_channel;
        if (!mirror_us(*in.rc, in.rc_fresh, rc_ch, &us)) {
            continue;
        }
        if (static_cast<int8_t>(i) == m_thr && !armed) {
            continue; // disarmed: throttle stays at stop
        }
        out->us[i] = us;
    }
    if (armed) {
        // The pilot has the motor; report what we are mirroring.
        out->throttle.armed = out->us[m_thr] != stop_us();
        out->throttle.throttle = m_ch[m_thr].to_norm(out->us[m_thr]);
    }
}

void Actuators::fill_bench(const ActuatorInputs& in, OutputFrame* out) const {
    fill_safe(out);
    for (uint8_t i = 0; i < m_cfg.num_channels; ++i) {
        if (static_cast<int8_t>(i) == m_thr) {
            continue; // BENCH: motor locked
        }
        const bool set = (in.bench.set_mask & (1u << i)) != 0;
        out->us[i] = set ? in.bench.us[i] : m_cfg.channels[i].trim_us;
    }
}

void Actuators::fill_control(const ActuatorInputs& in, uint32_t now_ms,
                             OutputFrame* out) {
    if (in.control == nullptr) {
        fill_safe(out); // no control laws yet: nothing to fly with
        return;
    }
    float mixed[pwm_out::kMaxChannels] = {};
    mix(m_cfg.mixer, *in.control, mixed);
    for (uint8_t i = 0; i < m_cfg.num_channels; ++i) {
        if (static_cast<int8_t>(i) == m_thr) {
            continue;
        }
        out->us[i] = m_ch[i].update(mixed[i], now_ms);
    }
    out->throttle = gate_throttle(mixed[m_thr], in.system, in.flight);
    out->us[m_thr] = esc_us(m_cfg.esc, m_cfg.channels[m_thr], out->throttle);
}

void Actuators::update(const ActuatorInputs& in, uint32_t now_ms,
                       OutputFrame* out) {
    out->num_channels = m_cfg.num_channels;
    if (!m_ok) {
        out->source = OutputSource::kSafe;
        out->throttle = ThrottleCmd{};
        return;
    }
    out->source = select_source(in.system, in.manual_active);
    switch (out->source) {
    case OutputSource::kSafe:
        fill_safe(out);
        break;
    case OutputSource::kMirror:
        fill_mirror(in, out);
        break;
    case OutputSource::kBench:
        fill_bench(in, out);
        break;
    case OutputSource::kControl:
        fill_control(in, now_ms, out);
        break;
    }
    if (out->source != OutputSource::kControl) {
        // Start the rate limiters from what was actually sent, so a later
        // handover to control/ doesn't jump.
        for (uint8_t i = 0; i < m_cfg.num_channels; ++i) {
            m_ch[i].sync_us(out->us[i], now_ms);
        }
    }
    m_esc_arm.update(out->us[m_thr], stop_us(), now_ms);
}
