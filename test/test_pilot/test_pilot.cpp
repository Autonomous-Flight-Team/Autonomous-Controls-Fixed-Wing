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
    s.stats_seen = true;
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

// Values from ExpressLRS's own CRSF_to_US() (crsf_protocol.h fmap),
// including ticks where a 0.625 slope is 1 µs off (173, 1000, 1500) and
// the extended-limits range outside 172..1811.
void test_crsf_matches_expresslrs() {
    const uint16_t ticks[] = {0,   100, 172,  173,  500,  992,
                              1000, 1500, 1811, 1984, 2047};
    const uint16_t us[] = {881,  944,  988,  989,  1193, 1500,
                           1505, 1818, 2012, 2120, 2159};
    for (size_t i = 0; i < sizeof(ticks) / sizeof(ticks[0]); ++i) {
        TEST_ASSERT_EQUAL_UINT16(us[i], crsf_ticks_to_us(ticks[i]));
    }
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

// Before the first stats frame LQ is unknown: frame age alone decides.
void test_lq_ignored_until_stats_seen() {
    PilotParams p;
    p.min_link_quality = 30;
    RcLinkStatus s = link_ok(1000);
    s.stats_seen = false;
    s.link_quality = 0;
    TEST_ASSERT_FALSE(rc_lost_raw(s, p, 1000));
    TEST_ASSERT_TRUE(rc_lost_raw(s, p, 1101)); // still lost on frame age
    s.stats_seen = true;
    TEST_ASSERT_TRUE(rc_lost_raw(s, p, 1000)); // now LQ 0 counts
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

void test_link_quality_for_telemetry() {
    RcLinkStatus s;
    TEST_ASSERT_EQUAL_UINT8(kLinkQualityUnknown, rc_link_quality_pct(s));
    s.frames_seen = true; // RC frames but no stats yet: still unknown
    TEST_ASSERT_EQUAL_UINT8(kLinkQualityUnknown, rc_link_quality_pct(s));
    s.stats_seen = true;
    s.link_quality = 87;
    TEST_ASSERT_EQUAL_UINT8(87, rc_link_quality_pct(s));
    s.link_quality = 0; // seen, now lost: a real 0, not unknown
    TEST_ASSERT_EQUAL_UINT8(0, rc_link_quality_pct(s));
    s.link_quality = 140;
    TEST_ASSERT_EQUAL_UINT8(100, rc_link_quality_pct(s));
}

void test_rc_telemetry_us() {
    const PilotInput in = frame(0);
    uint16_t us[18] = {};
    TEST_ASSERT_EQUAL_UINT8(8, rc_telemetry_us(in, true, us, 18));
    TEST_ASSERT_EQUAL_UINT16(1100, us[0]);
    TEST_ASSERT_EQUAL_UINT16(1800, us[7]);
    TEST_ASSERT_EQUAL_UINT8(4, rc_telemetry_us(in, true, us, 4));
    TEST_ASSERT_EQUAL_UINT8(0, rc_telemetry_us(in, false, us, 18));
    TEST_ASSERT_EQUAL_UINT8(0, rc_telemetry_us(in, true, nullptr, 18));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_crsf_anchor_points);
    RUN_TEST(test_crsf_matches_expresslrs);
    RUN_TEST(test_crsf_monotonic);
    RUN_TEST(test_rc_lost_before_first_frame);
    RUN_TEST(test_rc_lost_on_timeout);
    RUN_TEST(test_rc_lost_on_link_quality);
    RUN_TEST(test_lq_ignored_until_stats_seen);
    RUN_TEST(test_rc_lost_wraps_with_millis);
    RUN_TEST(test_fresh);
    RUN_TEST(test_bypass_switch_sets_manual_only_when_fresh);
    RUN_TEST(test_mirror_us);
    RUN_TEST(test_link_quality_for_telemetry);
    RUN_TEST(test_rc_telemetry_us);
    return UNITY_END();
}
