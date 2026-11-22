// Native backend for pwm_out: records writes so tests can check them.
#ifndef GNC_HARDWARE_BUILD

#include "pwm_out.h"

namespace pwm_out {
namespace {

uint16_t g_last_us[kMaxChannels] = {};
uint16_t g_safe_us[kMaxChannels] = {};
uint8_t g_n = 0;
bool g_started = false;
uint32_t g_writes = 0;

} // namespace

bool begin(const uint8_t* pins, uint8_t n, const uint16_t* safe_us) {
    (void)pins;
    if (n == 0 || n > kMaxChannels || safe_us == nullptr) {
        return false;
    }
    g_n = n;
    g_started = true;
    for (uint8_t ch = 0; ch < n; ++ch) {
        g_safe_us[ch] = hard_clamp(safe_us[ch]);
        write_us(ch, g_safe_us[ch]);
    }
    return true;
}

void write_us(uint8_t ch, uint16_t us) {
    if (!g_started || ch >= g_n) {
        return;
    }
    g_last_us[ch] = hard_clamp(us);
    ++g_writes;
}

void write_all_safe() {
    for (uint8_t ch = 0; ch < g_n; ++ch) {
        write_us(ch, g_safe_us[ch]);
    }
}

bool started() { return g_started; }
uint8_t num_channels() { return g_n; }

uint16_t fake_last_us(uint8_t ch) {
    return ch < kMaxChannels ? g_last_us[ch] : 0;
}

uint32_t fake_write_count() { return g_writes; }

void fake_reset() {
    for (uint8_t ch = 0; ch < kMaxChannels; ++ch) {
        g_last_us[ch] = 0;
        g_safe_us[ch] = 0;
    }
    g_n = 0;
    g_started = false;
    g_writes = 0;
}

} // namespace pwm_out

#endif // GNC_HARDWARE_BUILD
