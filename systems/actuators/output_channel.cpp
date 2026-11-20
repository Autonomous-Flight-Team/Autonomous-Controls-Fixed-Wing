#include "output_channel.h"

namespace {

float clampf(float v, float lo, float hi) {
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

uint16_t round_us(float us) {
    if (us <= 0.0f) {
        return 0;
    }
    return static_cast<uint16_t>(us + 0.5f);
}

float span(uint16_t from, uint16_t to) {
    return static_cast<float>(to) - static_cast<float>(from);
}

} // namespace

bool channel_config_valid(const ChannelConfig& cfg) {
    if (cfg.min_us >= cfg.max_us) {
        return false;
    }
    if (cfg.safe_us < cfg.min_us || cfg.safe_us > cfg.max_us) {
        return false;
    }
    if (cfg.kind == ChannelKind::kSurface) {
        return cfg.trim_us > cfg.min_us && cfg.trim_us < cfg.max_us;
    }
    return true;
}

uint16_t OutputChannel::to_us(float cmd) const {
    const float trim = static_cast<float>(m_cfg.trim_us);
    const float min = static_cast<float>(m_cfg.min_us);
    if (m_cfg.kind == ChannelKind::kThrottle) {
        float t = clampf(cmd, 0.0f, 1.0f);
        if (m_cfg.reversed) {
            t = 1.0f - t;
        }
        return round_us(min + t * span(m_cfg.min_us, m_cfg.max_us));
    }
    float s = clampf(cmd, -1.0f, 1.0f);
    if (m_cfg.reversed) {
        s = -s;
    }
    if (s >= 0.0f) {
        return round_us(trim + s * span(m_cfg.trim_us, m_cfg.max_us));
    }
    return round_us(trim + s * span(m_cfg.min_us, m_cfg.trim_us));
}

float OutputChannel::to_norm(uint16_t us) const {
    const float u = static_cast<float>(us);
    if (m_cfg.kind == ChannelKind::kThrottle) {
        const float range = span(m_cfg.min_us, m_cfg.max_us);
        float t = range > 0.0f ? (u - m_cfg.min_us) / range : 0.0f;
        t = clampf(t, 0.0f, 1.0f);
        return m_cfg.reversed ? 1.0f - t : t;
    }
    const float up = span(m_cfg.trim_us, m_cfg.max_us);
    const float down = span(m_cfg.min_us, m_cfg.trim_us);
    float s = 0.0f;
    if (u >= m_cfg.trim_us) {
        s = up > 0.0f ? (u - m_cfg.trim_us) / up : 0.0f;
    } else {
        s = down > 0.0f ? (u - m_cfg.trim_us) / down : 0.0f;
    }
    s = clampf(s, -1.0f, 1.0f);
    return m_cfg.reversed ? -s : s;
}

float OutputChannel::rate_limit(float cmd, uint32_t now_ms) {
    if (m_cfg.max_rate_per_s <= 0.0f || !m_has_last) {
        return cmd;
    }
    const uint32_t dt_ms = now_ms - m_last_ms; // wrap-safe
    const float step =
        m_cfg.max_rate_per_s * static_cast<float>(dt_ms) / 1000.0f;
    return clampf(cmd, m_last_norm - step, m_last_norm + step);
}

uint16_t OutputChannel::update(float cmd, uint32_t now_ms) {
    const bool thr = m_cfg.kind == ChannelKind::kThrottle;
    float c = thr ? clampf(cmd, 0.0f, 1.0f) : clampf(cmd, -1.0f, 1.0f);
    c = rate_limit(c, now_ms);
    m_last_norm = c;
    m_last_ms = now_ms;
    m_has_last = true;
    return to_us(c);
}

void OutputChannel::sync_us(uint16_t us, uint32_t now_ms) {
    m_last_norm = to_norm(us);
    m_last_ms = now_ms;
    m_has_last = true;
}
