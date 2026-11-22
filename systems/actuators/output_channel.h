#ifndef OUTPUT_CHANNEL_H_
#define OUTPUT_CHANNEL_H_

#include <cstdint>

// One servo or ESC output. Pure: no hardware, builds on native.
// pwm_out writes the µs this produces; nothing else here touches pins.

// Values are logged: never renumber, only add.
enum class ChannelKind : uint8_t {
    kSurface = 0,  // [-1, 1] around trim
    kThrottle = 1, // [0, 1] from min_us
};

// Found on the bench; PLACEHOLDER values until then.
struct ChannelConfig {
    const char* name = "";
    ChannelKind kind = ChannelKind::kSurface;
    uint16_t min_us = 1000;
    uint16_t trim_us = 1500; // surfaces only; need not be centered
    uint16_t max_us = 2000;
    bool reversed = false;
    float max_rate_per_s = 0.0f; // normalized units/s; 0 = unlimited
    uint16_t safe_us = 1500;     // disarmed / fault value (throttle: stop)
    int8_t rc_channel = -1;      // PilotInput::raw_us index to mirror; -1 none
};

// True if the config is usable: min < max, trim and safe_us inside range.
bool channel_config_valid(const ChannelConfig& cfg);

class OutputChannel {
  public:
    OutputChannel() = default;
    explicit OutputChannel(const ChannelConfig& cfg) : m_cfg(cfg) {}

    // Normalized command -> µs, clamped and rate limited.
    // Surfaces: -1 -> min_us, 0 -> trim_us, +1 -> max_us (before reverse).
    // Throttle: 0 -> min_us, 1 -> max_us.
    uint16_t update(float cmd, uint32_t now_ms);

    // Mapping only: no rate limit, no state change.
    uint16_t to_us(float cmd) const;
    float to_norm(uint16_t us) const; // inverse of to_us, clamped

    // Something else drove this output (mirror, bench, safe): start the
    // rate limiter from there so the next update() doesn't jump.
    void sync_us(uint16_t us, uint32_t now_ms);

    uint16_t safe_us() const { return m_cfg.safe_us; }
    const ChannelConfig& config() const { return m_cfg; }

  private:
    float rate_limit(float cmd, uint32_t now_ms);

    ChannelConfig m_cfg{};
    float m_last_norm = 0.0f;
    uint32_t m_last_ms = 0;
    bool m_has_last = false; // false: no previous output to limit from
};

#endif // OUTPUT_CHANNEL_H_
