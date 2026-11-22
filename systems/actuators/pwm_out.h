#ifndef PWM_OUT_H_
#define PWM_OUT_H_

#include <cstdint>

// Pulse outputs for servos and PWM ESCs. The only hardware code in actuators.
// Two backends, chosen at link time (no virtual functions):
//   pwm_out_teensy.cpp  GNC_HARDWARE_BUILD: FlexPWM at 50 Hz
//   pwm_out_fake.cpp    native: records the last µs per channel for tests
namespace pwm_out {

constexpr uint8_t kMaxChannels = 8;
constexpr uint16_t kHardMinUs = 900; // last line of defense against bad config
constexpr uint16_t kHardMaxUs = 2100;
constexpr uint32_t kFrameHz = 50;

// Starts every channel and writes safe_us before returning, so outputs never
// float. False if n is 0 or over kMaxChannels.
bool begin(const uint8_t* pins, uint8_t n, const uint16_t* safe_us);

// Clamped to [kHardMinUs, kHardMaxUs]. Ignored before begin() or if ch >= n.
void write_us(uint8_t ch, uint16_t us);

// Every channel back to the safe_us given to begin().
void write_all_safe();

bool started();
uint8_t num_channels();

constexpr uint16_t hard_clamp(uint16_t us) {
    return us < kHardMinUs ? kHardMinUs : (us > kHardMaxUs ? kHardMaxUs : us);
}

#ifndef GNC_HARDWARE_BUILD
// Fake backend only (native tests).
uint16_t fake_last_us(uint8_t ch); // 0 if never written
uint32_t fake_write_count();
void fake_reset();
#endif

} // namespace pwm_out

#endif // PWM_OUT_H_
