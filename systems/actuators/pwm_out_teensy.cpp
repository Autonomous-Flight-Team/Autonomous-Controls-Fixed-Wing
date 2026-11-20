// Teensy 4.1 backend for pwm_out: hardware FlexPWM/QuadTimer at 50 Hz, so
// pulse widths don't depend on loop timing (the Servo library is
// interrupt-driven and can jitter).
// Pins on the same timer submodule share a frequency: all channels run at
// kFrameHz, so this only matters if a backend ever needs a faster ESC rate.
#ifdef GNC_HARDWARE_BUILD

#include "pwm_out.h"
#include <Arduino.h>

namespace pwm_out {
namespace {

constexpr uint8_t kResolutionBits = 15;
constexpr uint32_t kCounts = 1UL << kResolutionBits;
constexpr uint32_t kPeriodUs = 1000000UL / kFrameHz;

uint8_t g_pins[kMaxChannels] = {};
uint16_t g_safe_us[kMaxChannels] = {};
uint8_t g_n = 0;
bool g_started = false;

uint32_t us_to_counts(uint16_t us) {
    return (static_cast<uint32_t>(us) * kCounts + kPeriodUs / 2) / kPeriodUs;
}

} // namespace

bool begin(const uint8_t* pins, uint8_t n, const uint16_t* safe_us) {
    if (n == 0 || n > kMaxChannels || pins == nullptr || safe_us == nullptr) {
        return false;
    }
    analogWriteResolution(kResolutionBits);
    g_n = n;
    for (uint8_t ch = 0; ch < n; ++ch) {
        g_pins[ch] = pins[ch];
        g_safe_us[ch] = hard_clamp(safe_us[ch]);
        analogWriteFrequency(g_pins[ch], static_cast<float>(kFrameHz));
    }
    g_started = true;
    write_all_safe();
    return true;
}

void write_us(uint8_t ch, uint16_t us) {
    if (!g_started || ch >= g_n) {
        return;
    }
    analogWrite(g_pins[ch], static_cast<int>(us_to_counts(hard_clamp(us))));
}

void write_all_safe() {
    for (uint8_t ch = 0; ch < g_n; ++ch) {
        write_us(ch, g_safe_us[ch]);
    }
}

bool started() { return g_started; }
uint8_t num_channels() { return g_n; }

} // namespace pwm_out

#endif // GNC_HARDWARE_BUILD
