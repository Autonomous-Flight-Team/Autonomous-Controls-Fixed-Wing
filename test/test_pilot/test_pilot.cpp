// Pilot layer tests: CRSF conversion, kRcLost raw bit, freshness, switches
// and mirroring. rc_link doesn't exist yet: inputs are built by hand.
// Run: pio test -e native -f test_pilot

#include "pilot.h"
#include <unity.h>

namespace {

PilotInput frame(uint32_t stamp_ms) {
    PilotInput in;
    in.num_raw = 8;
    for (uint8_t i = 0; i < 8; ++i) {
        in.raw_us[i] = static_cast<uint16_t>(1100 + 100 * i);
    }
    in.stamp_ms = stamp_ms;
    return in;
}

RcLinkStatus link_ok(uint32_t last_frame_ms) {
    RcLinkStatus s;
    s.frames_seen = true;
    s.link_quality = 100;
    s.last_frame_ms = last_frame_ms;
    return s;
}

} // namespace

void setUp() {}
void tearDown() {}

void test_crsf_anchor_points() {
    TEST_ASSERT_EQUAL_UINT16(988, crsf_ticks_to_us(172));
    TEST_ASSERT_EQUAL_UINT16(1500, crsf_ticks_to_us(992));
    TEST_ASSERT_EQUAL_UINT16(2012, crsf_ticks_to_us(1811));
}

void test_crsf_monotonic() {
    uint16_t prev = crsf_ticks_to_us(172);
    for (uint16_t t = 173; t <= 1811; ++t) {
        const uint16_t us = crsf_ticks_to_us(t);
        TEST_ASSERT_TRUE(us >= prev);
        prev = us;
    }
}

void test_rc_lost_before_first_frame() {
    const PilotParams p;
    const RcLinkStatus never;
    TEST_ASSERT_TRUE(rc_lost_raw(never, p, 0));
}

void test_rc_lost_on_timeout() {
    const PilotParams p; // 100 ms
    const RcLinkStatus s = link_ok(1000);
    TEST_ASSERT_FALSE(rc_lost_raw(s, p, 1100));
    TEST_ASSERT_TRUE(rc_lost_raw(s, p, 1101));
}

void test_rc_lost_on_link_quality() {
    PilotParams p;
    RcLinkStatus s = link_ok(1000);
    s.link_quality = 0;
    TEST_ASSERT_TRUE(rc_lost_raw(s, p, 1000));
    p.min_link_quality = 30;
    s.link_quality = 30;
    TEST_ASSERT_TRUE(rc_lost_raw(s, p, 1000));
    s.link_quality = 31;
    TEST_ASSERT_FALSE(rc_lost_raw(s, p, 1000));
}

void test_rc_lost_wraps_with_millis() {
    const PilotParams p;
    const RcLinkStatus s = link_ok(0xFFFFFFC0u);
    TEST_ASSERT_FALSE(rc_lost_raw(s, p, 0x10u)); // 80 ms after the wrap
}

void test_fresh() {
    const PilotParams p;
    const PilotInput in = frame(500);
    TEST_ASSERT_TRUE(pilot_fresh(in, p, 600));
    TEST_ASSERT_FALSE(pilot_fresh(in, p, 601));
    const PilotInput empty;
    TEST_ASSERT_FALSE(pilot_fresh(empty, p, 0));
}

void test_bypass_switch_sets_manual_only_when_fresh() {
    PilotInput in = frame(0);
    TEST_ASSERT_FALSE(manual_active(in, true));
    in.switches = 1u << static_cast<uint8_t>(PilotSwitch::kBypass);
    TEST_ASSERT_TRUE(switch_on(in, PilotSwitch::kBypass));
    TEST_ASSERT_FALSE(switch_on(in, PilotSwitch::kAutonomy));
    TEST_ASSERT_TRUE(manual_active(in, true));
    TEST_ASSERT_FALSE(manual_active(in, false));
}

void test_mirror_us() {
    const PilotInput in = frame(0);
    uint16_t us = 0;
    TEST_ASSERT_TRUE(mirror_us(in, true, 2, &us));
    TEST_ASSERT_EQUAL_UINT16(1300, us);
    TEST_ASSERT_FALSE(mirror_us(in, false, 2, &us)); // stale
    TEST_ASSERT_FALSE(mirror_us(in, true, -1, &us)); // not mapped
    TEST_ASSERT_FALSE(mirror_us(in, true, 8, &us));  // beyond num_raw
    TEST_ASSERT_FALSE(mirror_us(in, true, 2, nullptr));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_crsf_anchor_points);
    RUN_TEST(test_crsf_monotonic);
    RUN_TEST(test_rc_lost_before_first_frame);
    RUN_TEST(test_rc_lost_on_timeout);
    RUN_TEST(test_rc_lost_on_link_quality);
    RUN_TEST(test_rc_lost_wraps_with_millis);
    RUN_TEST(test_fresh);
    RUN_TEST(test_bypass_switch_sets_manual_only_when_fresh);
    RUN_TEST(test_mirror_us);
    return UNITY_END();
}
