// Actuator layer tests: output channels, pwm_out (fake backend), mixer,
// throttle gate / ESC mapping and the motor monitor.
// Run: pio test -e native -f test_actuators

#include "mixer.h"
#include "motor_monitor.h"
#include "output_channel.h"
#include "pwm_out.h"
#include "throttle_gate.h"
#include <unity.h>

namespace {

ChannelConfig surface(uint16_t min, uint16_t trim, uint16_t max) {
    ChannelConfig c;
    c.name = "surf";
    c.min_us = min;
    c.trim_us = trim;
    c.max_us = max;
    c.safe_us = trim;
    return c;
}

ChannelConfig throttle_ch() {
    ChannelConfig c;
    c.name = "thr";
    c.kind = ChannelKind::kThrottle;
    c.min_us = 1000;
    c.max_us = 2000;
    c.safe_us = 1000;
    return c;
}

const SystemState kAllSystem[] = {
    SystemState::kBoot,     SystemState::kPreflight, SystemState::kStandby,
    SystemState::kFaultGround, SystemState::kBench,  SystemState::kActive,
    SystemState::kDegraded, SystemState::kCritical,  SystemState::kEmergency,
    SystemState::kInAirReset,
};

const FlightState kAllFlight[] = {
    FlightState::kOnGround, FlightState::kTakeoffRoll, FlightState::kRotate,
    FlightState::kLanded,   FlightState::kClimb,       FlightState::kCruise,
    FlightState::kLoiter,   FlightState::kRtl,         FlightState::kDescent,
    FlightState::kLanding,  FlightState::kRejectedTakeoff,
    FlightState::kFailsafeLoiter, FlightState::kEmergencyLand,
};

} // namespace

void setUp() { pwm_out::fake_reset(); }
void tearDown() {}

// ---------------- OutputChannel ----------------

void test_surface_maps_asymmetric_around_trim() {
    OutputChannel ch(surface(1100, 1450, 1900));
    TEST_ASSERT_EQUAL_UINT16(1100, ch.to_us(-1.0f));
    TEST_ASSERT_EQUAL_UINT16(1450, ch.to_us(0.0f));
    TEST_ASSERT_EQUAL_UINT16(1900, ch.to_us(1.0f));
    TEST_ASSERT_EQUAL_UINT16(1675, ch.to_us(0.5f));  // 1450 + 0.5 * 450
    TEST_ASSERT_EQUAL_UINT16(1275, ch.to_us(-0.5f)); // 1450 - 0.5 * 350
}

void test_surface_clamps_out_of_range() {
    OutputChannel ch(surface(1000, 1500, 2000));
    TEST_ASSERT_EQUAL_UINT16(2000, ch.to_us(3.0f));
    TEST_ASSERT_EQUAL_UINT16(1000, ch.to_us(-3.0f));
}

void test_surface_reversed() {
    ChannelConfig c = surface(1000, 1500, 2000);
    c.reversed = true;
    OutputChannel ch(c);
    TEST_ASSERT_EQUAL_UINT16(1000, ch.to_us(1.0f));
    TEST_ASSERT_EQUAL_UINT16(2000, ch.to_us(-1.0f));
}

void test_throttle_maps_from_min() {
    OutputChannel ch(throttle_ch());
    TEST_ASSERT_EQUAL_UINT16(1000, ch.to_us(0.0f));
    TEST_ASSERT_EQUAL_UINT16(1500, ch.to_us(0.5f));
    TEST_ASSERT_EQUAL_UINT16(2000, ch.to_us(1.0f));
    TEST_ASSERT_EQUAL_UINT16(1000, ch.to_us(-0.5f));
}

void test_to_norm_inverts_to_us() {
    OutputChannel ch(surface(1100, 1450, 1900));
    for (float v = -1.0f; v <= 1.0f; v += 0.25f) {
        TEST_ASSERT_FLOAT_WITHIN(0.01f, v, ch.to_norm(ch.to_us(v)));
    }
}

void test_rate_limit() {
    ChannelConfig c = surface(1000, 1500, 2000);
    c.max_rate_per_s = 2.0f; // full travel (-1 -> 1) in 1 s
    OutputChannel ch(c);
    TEST_ASSERT_EQUAL_UINT16(1500, ch.update(0.0f, 0));
    // 100 ms later: at most 0.2 of travel
    TEST_ASSERT_EQUAL_UINT16(1600, ch.update(1.0f, 100));
    TEST_ASSERT_EQUAL_UINT16(1700, ch.update(1.0f, 200));
}

void test_rate_limit_wraps_with_millis() {
    ChannelConfig c = surface(1000, 1500, 2000);
    c.max_rate_per_s = 2.0f;
    OutputChannel ch(c);
    ch.update(0.0f, 0xFFFFFFF0u);
    TEST_ASSERT_EQUAL_UINT16(1600, ch.update(1.0f, 0xFFFFFFF0u + 100u));
}

void test_sync_us_makes_handover_bumpless() {
    ChannelConfig c = surface(1000, 1500, 2000);
    c.max_rate_per_s = 1.0f;
    OutputChannel ch(c);
    ch.sync_us(1800, 0); // pilot was holding 0.6
    // Controller asks for 0: moves from 1800, not from trim
    TEST_ASSERT_EQUAL_UINT16(1750, ch.update(0.0f, 100));
}

void test_channel_config_validation() {
    TEST_ASSERT_TRUE(channel_config_valid(surface(1000, 1500, 2000)));
    TEST_ASSERT_FALSE(channel_config_valid(surface(2000, 1500, 1000)));
    TEST_ASSERT_FALSE(channel_config_valid(surface(1000, 1000, 2000)));
    ChannelConfig bad_safe = surface(1000, 1500, 2000);
    bad_safe.safe_us = 2100;
    TEST_ASSERT_FALSE(channel_config_valid(bad_safe));
    TEST_ASSERT_TRUE(channel_config_valid(throttle_ch()));
}

// ---------------- pwm_out (fake backend) ----------------

void test_pwm_begin_writes_safe_values() {
    const uint8_t pins[] = {2, 3};
    const uint16_t safe[] = {1500, 1000};
    TEST_ASSERT_TRUE(pwm_out::begin(pins, 2, safe));
    TEST_ASSERT_EQUAL_UINT16(1500, pwm_out::fake_last_us(0));
    TEST_ASSERT_EQUAL_UINT16(1000, pwm_out::fake_last_us(1));
}

void test_pwm_hard_clamp_and_bounds() {
    const uint8_t pins[] = {2};
    const uint16_t safe[] = {1500};
    pwm_out::begin(pins, 1, safe);
    pwm_out::write_us(0, 3000);
    TEST_ASSERT_EQUAL_UINT16(pwm_out::kHardMaxUs, pwm_out::fake_last_us(0));
    pwm_out::write_us(0, 100);
    TEST_ASSERT_EQUAL_UINT16(pwm_out::kHardMinUs, pwm_out::fake_last_us(0));
    const uint32_t writes = pwm_out::fake_write_count();
    pwm_out::write_us(5, 1500); // out of range: ignored
    TEST_ASSERT_EQUAL_UINT32(writes, pwm_out::fake_write_count());
}

void test_pwm_ignored_before_begin_and_bad_begin() {
    pwm_out::write_us(0, 1500);
    TEST_ASSERT_EQUAL_UINT32(0, pwm_out::fake_write_count());
    const uint16_t safe[pwm_out::kMaxChannels + 1] = {};
    TEST_ASSERT_FALSE(pwm_out::begin(nullptr, 0, safe));
    TEST_ASSERT_FALSE(
        pwm_out::begin(nullptr, pwm_out::kMaxChannels + 1, safe));
}

void test_pwm_write_all_safe() {
    const uint8_t pins[] = {2, 3};
    const uint16_t safe[] = {1500, 1000};
    pwm_out::begin(pins, 2, safe);
    pwm_out::write_us(0, 1900);
    pwm_out::write_us(1, 1800);
    pwm_out::write_all_safe();
    TEST_ASSERT_EQUAL_UINT16(1500, pwm_out::fake_last_us(0));
    TEST_ASSERT_EQUAL_UINT16(1000, pwm_out::fake_last_us(1));
}

// ---------------- Mixer ----------------

void test_conventional_mixer() {
    const MixerConfig m = conventional_mixer();
    SurfaceDemand d;
    d.roll = 0.5f;
    d.pitch = -0.25f;
    d.yaw = 0.1f;
    d.throttle = 0.7f;
    d.flaps = 1.0f;
    float out[pwm_out::kMaxChannels] = {};
    mix(m, d, out);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, out[0]);
    TEST_ASSERT_EQUAL_FLOAT(-0.5f, out[1]);
    TEST_ASSERT_EQUAL_FLOAT(-0.25f, out[2]);
    TEST_ASSERT_EQUAL_FLOAT(0.1f, out[3]);
    TEST_ASSERT_EQUAL_FLOAT(0.7f, out[4]);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, out[5]);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, out[6]);
}

void test_vtail_is_just_a_table() {
    MixerConfig m;
    m.num_rows = 2;
    m.rows[0].pitch = 0.5f;
    m.rows[0].yaw = 0.5f;
    m.rows[1].pitch = 0.5f;
    m.rows[1].yaw = -0.5f;
    SurfaceDemand d;
    d.pitch = 1.0f;
    d.yaw = 1.0f;
    float out[2] = {};
    mix(m, d, out);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, out[0]);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out[1]);
}

// ---------------- Throttle gate ----------------

// Every SystemState x FlightState pair against the FSMs' rules.
void test_throttle_allowed_table() {
    for (SystemState s : kAllSystem) {
        for (FlightState f : kAllFlight) {
            bool expect = is_armed(s);
            if (f == FlightState::kOnGround) {
                expect = s == SystemState::kActive;
            } else if (f == FlightState::kLanded ||
                       f == FlightState::kRejectedTakeoff) {
                expect = false;
            }
            TEST_ASSERT_EQUAL_MESSAGE(expect, throttle_allowed(s, f),
                                      to_string(s));
        }
    }
}

void test_bench_never_gets_throttle() {
    for (FlightState f : kAllFlight) {
        const ThrottleCmd c = gate_throttle(1.0f, SystemState::kBench, f);
        TEST_ASSERT_FALSE(c.armed);
        TEST_ASSERT_EQUAL_FLOAT(0.0f, c.throttle);
    }
}

void test_gate_clamps_demand() {
    const ThrottleCmd hi =
        gate_throttle(2.0f, SystemState::kActive, FlightState::kCruise);
    TEST_ASSERT_TRUE(hi.armed);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, hi.throttle);
    const ThrottleCmd lo =
        gate_throttle(-1.0f, SystemState::kActive, FlightState::kCruise);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, lo.throttle);
}

void test_esc_pwm_mapping() {
    const EscConfig esc;
    const ChannelConfig ch = throttle_ch();
    ThrottleCmd cmd;
    cmd.armed = false;
    cmd.throttle = 1.0f;
    TEST_ASSERT_EQUAL_UINT16(1000, esc_us(esc, ch, cmd)); // disarmed: stop
    cmd.armed = true;
    TEST_ASSERT_EQUAL_UINT16(2000, esc_us(esc, ch, cmd));
    cmd.throttle = 0.25f;
    TEST_ASSERT_EQUAL_UINT16(1250, esc_us(esc, ch, cmd));
}

void test_bidirectional_esc_stops_at_midpoint() {
    EscConfig esc;
    esc.bidirectional = true;
    ChannelConfig ch = throttle_ch();
    TEST_ASSERT_EQUAL_UINT16(1500, esc_stop_us(esc, ch));
    ThrottleCmd cmd; // disarmed
    TEST_ASSERT_EQUAL_UINT16(1500, esc_us(esc, ch, cmd));
    // safe_us at min would mean full reverse on a 3D ESC: invalid
    TEST_ASSERT_FALSE(esc_config_valid(esc, ch));
    ch.safe_us = 1500;
    TEST_ASSERT_TRUE(esc_config_valid(esc, ch));
}

void test_esc_arm_tracker() {
    EscConfig esc;
    esc.arm_hold_ms = 1000;
    EscArmTracker t;
    t.update(1000, 1000, 0);
    TEST_ASSERT_FALSE(t.ready(esc, 500));
    t.update(1000, 1000, 1000);
    TEST_ASSERT_TRUE(t.ready(esc, 1000));
    t.update(1200, 1000, 1100); // left stop: restart the hold
    TEST_ASSERT_FALSE(t.ready(esc, 1100));
    t.update(1000, 1000, 1200);
    TEST_ASSERT_FALSE(t.ready(esc, 2100));
    TEST_ASSERT_TRUE(t.ready(esc, 2200));
}

// ---------------- Motor monitor ----------------

void test_motor_no_current_response_raises_after_window() {
    MotorMonitor m;
    MotorMonitorInputs in;
    in.current_valid = true;
    in.throttle = 0.5f;
    in.current_a = 0.5f;
    TEST_ASSERT_FALSE(m.update(in, 0));
    TEST_ASSERT_FALSE(m.update(in, 499));
    TEST_ASSERT_TRUE(m.update(in, 500));
    in.current_a = 10.0f; // motor answered
    TEST_ASSERT_FALSE(m.update(in, 520));
}

void test_motor_current_at_zero_throttle_raises() {
    MotorMonitor m;
    MotorMonitorInputs in;
    in.current_valid = true;
    in.current_a = 1.0f; // avionics + servos
    TEST_ASSERT_FALSE(m.update(in, 0));
    in.current_a = 8.0f;
    TEST_ASSERT_TRUE(m.update(in, 20));
}

void test_motor_no_feedback_no_fault() {
    MotorMonitor m;
    MotorMonitorInputs in;
    in.throttle = 1.0f;
    TEST_ASSERT_FALSE(m.update(in, 0));
    TEST_ASSERT_FALSE(m.update(in, 5000));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_surface_maps_asymmetric_around_trim);
    RUN_TEST(test_surface_clamps_out_of_range);
    RUN_TEST(test_surface_reversed);
    RUN_TEST(test_throttle_maps_from_min);
    RUN_TEST(test_to_norm_inverts_to_us);
    RUN_TEST(test_rate_limit);
    RUN_TEST(test_rate_limit_wraps_with_millis);
    RUN_TEST(test_sync_us_makes_handover_bumpless);
    RUN_TEST(test_channel_config_validation);
    RUN_TEST(test_pwm_begin_writes_safe_values);
    RUN_TEST(test_pwm_hard_clamp_and_bounds);
    RUN_TEST(test_pwm_ignored_before_begin_and_bad_begin);
    RUN_TEST(test_pwm_write_all_safe);
    RUN_TEST(test_conventional_mixer);
    RUN_TEST(test_vtail_is_just_a_table);
    RUN_TEST(test_throttle_allowed_table);
    RUN_TEST(test_bench_never_gets_throttle);
    RUN_TEST(test_gate_clamps_demand);
    RUN_TEST(test_esc_pwm_mapping);
    RUN_TEST(test_bidirectional_esc_stops_at_midpoint);
    RUN_TEST(test_esc_arm_tracker);
    RUN_TEST(test_motor_no_current_response_raises_after_window);
    RUN_TEST(test_motor_current_at_zero_throttle_raises);
    RUN_TEST(test_motor_no_feedback_no_fault);
    return UNITY_END();
}
