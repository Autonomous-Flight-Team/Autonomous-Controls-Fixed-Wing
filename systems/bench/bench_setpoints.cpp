#include "bench_setpoints.h"

namespace {

// Triangle over one cycle: 0 -> +1 -> -1 -> 0.
float sweep_shape(uint32_t elapsed_ms, uint32_t period_ms) {
    const float p = static_cast<float>(elapsed_ms) / period_ms;
    if (p < 0.25f) {
        return 4.0f * p;
    }
    if (p < 0.75f) {
        return 1.0f - 4.0f * (p - 0.25f);
    }
    return -1.0f + 4.0f * (p - 0.75f);
}

uint8_t ch_bit(uint8_t channel) { return static_cast<uint8_t>(1u << channel); }

} // namespace

BenchSetpoints::BenchSetpoints(const ChannelConfig* channels, uint8_t n)
    : m_channels(channels),
      m_n(n > pwm_out::kMaxChannels ? pwm_out::kMaxChannels : n) {}

bool BenchSetpoints::is_surface(uint8_t channel) const {
    return channel < m_n &&
           m_channels[channel].kind == ChannelKind::kSurface;
}

bool BenchSetpoints::set_us(uint8_t channel, uint16_t us, uint32_t now_ms) {
    (void)now_ms;
    if (!is_surface(channel)) {
        return false;
    }
    const ChannelConfig& c = m_channels[channel];
    if (us < c.min_us || us > c.max_us) {
        return false;
    }
    m_us[channel] = us;
    m_set_mask = static_cast<uint8_t>(m_set_mask | ch_bit(channel));
    m_sweep_mask = static_cast<uint8_t>(m_sweep_mask & ~ch_bit(channel));
    return true;
}

void BenchSetpoints::center_all() {
    m_set_mask = 0;
    m_sweep_mask = 0;
}

bool BenchSetpoints::sweep(int8_t channel, uint32_t now_ms) {
    if (channel >= 0 && !is_surface(static_cast<uint8_t>(channel))) {
        return false;
    }
    for (uint8_t i = 0; i < m_n; ++i) {
        const bool pick = channel < 0 || i == static_cast<uint8_t>(channel);
        if (pick && is_surface(i)) {
            m_sweep_mask = static_cast<uint8_t>(m_sweep_mask | ch_bit(i));
            m_sweep_start_ms[i] = now_ms;
        }
    }
    update(now_ms);
    return true;
}

void BenchSetpoints::update(uint32_t now_ms) {
    for (uint8_t i = 0; i < m_n; ++i) {
        if ((m_sweep_mask & ch_bit(i)) == 0) {
            continue;
        }
        const uint32_t elapsed = now_ms - m_sweep_start_ms[i];
        if (elapsed >= kSweepMs) {
            m_sweep_mask = static_cast<uint8_t>(m_sweep_mask & ~ch_bit(i));
            m_set_mask = static_cast<uint8_t>(m_set_mask & ~ch_bit(i)); // trim
            continue;
        }
        const OutputChannel ch(m_channels[i]);
        m_us[i] = ch.to_us(sweep_shape(elapsed, kSweepMs));
        m_set_mask = static_cast<uint8_t>(m_set_mask | ch_bit(i));
    }
}

bool BenchSetpoints::get_us(uint8_t channel, uint16_t* us) const {
    if (channel >= m_n || us == nullptr) {
        return false;
    }
    const bool set = (m_set_mask & ch_bit(channel)) != 0;
    *us = set ? m_us[channel] : m_channels[channel].trim_us;
    return set;
}

BenchFrame BenchSetpoints::frame() const {
    BenchFrame f;
    f.set_mask = m_set_mask;
    for (uint8_t i = 0; i < m_n; ++i) {
        f.us[i] = m_us[i];
    }
    return f;
}
