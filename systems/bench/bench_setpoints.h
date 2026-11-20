#ifndef BENCH_SETPOINTS_H_
#define BENCH_SETPOINTS_H_

#include "actuators.h"
#include "output_channel.h"
#include "pwm_out.h"
#include <cstdint>

// Surface setpoints for BENCH. One object in main, written by the USB
// console and by MAVLink DO_SET_SERVO (through StateBridge), read by
// Actuators only while the system state is BENCH. The throttle channel is
// never accepted: BENCH keeps the motor locked.
class BenchSetpoints {
  public:
    // channels must outlive this object (normally the vehicle config).
    BenchSetpoints(const ChannelConfig* channels, uint8_t n);

    // Range-checked against the channel's min/max. Cancels a sweep on it.
    bool set_us(uint8_t channel, uint16_t us, uint32_t now_ms);

    // MAVLink DO_SET_SERVO numbers outputs from 1: servo 1 = channel 0.
    // Called by the BenchServoSink adapter in main (comms' interface).
    // Comms has already rejected malformed values and only calls this in
    // BENCH; limits are checked here. false -> DENIED to the ground.
    bool set_servo_1based(uint8_t servo, uint16_t us);

    // Every channel back to trim; stops sweeps.
    void center_all();

    // One slow trim -> max -> min -> trim cycle. channel -1 = all surfaces.
    bool sweep(int8_t channel, uint32_t now_ms);

    // Advances sweeps; call once per loop.
    void update(uint32_t now_ms);

    bool get_us(uint8_t channel, uint16_t* us) const;
    BenchFrame frame() const;

    uint8_t num_channels() const { return m_n; }
    const ChannelConfig& channel(uint8_t i) const { return m_channels[i]; }

    static constexpr uint32_t kSweepMs = 4000;

  private:
    bool is_surface(uint8_t channel) const;

    const ChannelConfig* m_channels;
    uint8_t m_n;
    uint16_t m_us[pwm_out::kMaxChannels] = {};
    uint8_t m_set_mask = 0;
    uint8_t m_sweep_mask = 0;
    uint32_t m_sweep_start_ms[pwm_out::kMaxChannels] = {};
};

#endif // BENCH_SETPOINTS_H_
