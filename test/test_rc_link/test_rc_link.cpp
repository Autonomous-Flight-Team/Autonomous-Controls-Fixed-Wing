// rc_link tests: CRSF framing, CRC, channel unpacking, link statistics,
// resync after noise, and the PilotInput / RcLinkStatus it produces.
// Frames are built here and fed through an in-memory UART.
// Run: pio test -e native -f test_rc_link

#include "byte_stream.h"
#include "pilot.h"
#include "rc_link.h"
#include <unity.h>
#include <vector>

namespace {

PipeRing g_rx, g_unused;
MemoryByteStream g_uart(g_rx, g_unused); // rc_link reads g_rx

void push(const std::vector<uint8_t>& bytes) {
    for (uint8_t b : bytes) {
        g_rx.push(b);
    }
}

std::vector<uint8_t> frame(uint8_t type, const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> body{type};
    body.insert(body.end(), payload.begin(), payload.end());
    std::vector<uint8_t> f{kCrsfSync, static_cast<uint8_t>(body.size() + 1)};
    f.insert(f.end(), body.begin(), body.end());
    f.push_back(crsf_crc8(body.data(), static_cast<uint8_t>(body.size())));
    return f;
}

// Pack 16 channels of 11-bit ticks, little-endian, as the receiver does.
std::vector<uint8_t> rc_frame(const uint16_t ticks[kMaxRcChannels]) {
    std::vector<uint8_t> p(kCrsfRcPayloadLen, 0);
    for (uint16_t bit = 0; bit < 16 * 11; ++bit) {
        if (ticks[bit / 11] & (1u << (bit % 11))) {
            p[bit / 8] |= static_cast<uint8_t>(1u << (bit % 8));
        }
    }
    return frame(kCrsfTypeRcChannels, p);
}

std::vector<uint8_t> centered_frame() {
    uint16_t t[kMaxRcChannels];
    for (auto& v : t) {
        v = 992;
    }
    return rc_frame(t);
}

std::vector<uint8_t> link_stats(uint8_t rssi1, uint8_t rssi2, uint8_t lq,
                                uint8_t antenna) {
    const std::vector<uint8_t> p{rssi1, rssi2, lq, 10, antenna, 4, 3, 60,
                                 100, 8};
    return frame(kCrsfTypeLinkStats, p);
}

} // namespace

void setUp() {
    g_rx.clear();
    rc_link_init(g_uart);
}

void tearDown() {}

void test_crc8_dvb_s2() {
    // CRC-8/DVB-S2 check value for "123456789".
    const uint8_t s[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    TEST_ASSERT_EQUAL_HEX8(0xBC, crsf_crc8(s, sizeof(s)));
}

void test_unpack_round_trip() {
    uint16_t want[kMaxRcChannels];
    for (uint8_t i = 0; i < kMaxRcChannels; ++i) {
        want[i] = static_cast<uint16_t>(172 + i * 109);
    }
    want[15] = 2047; // all 11 bits
    std::vector<uint8_t> f = rc_frame(want);
    uint16_t got[kMaxRcChannels];
    crsf_unpack_channels(&f[3], got);
    TEST_ASSERT_EQUAL_UINT16_ARRAY(want, got, kMaxRcChannels);
}

void test_nothing_before_first_frame() {
    PilotInput in;
    rc_link_update(10);
    TEST_ASSERT_FALSE(rc_link_latest(&in));
    TEST_ASSERT_FALSE(rc_link_status().frames_seen);
    TEST_ASSERT_TRUE(rc_lost_raw(rc_link_status(), PilotParams(), 10));
}

void test_channels_become_pilot_input() {
    uint16_t t[kMaxRcChannels];
    for (auto& v : t) {
        v = 992;
    }
    t[0] = 1811; // roll full right
    t[1] = 172;  // pitch full down
    t[2] = 172;  // throttle idle
    t[3] = 992;  // yaw centered
    t[6] = 1811; // CH7 bypass on
    push(rc_frame(t));
    rc_link_update(1234);

    PilotInput in;
    TEST_ASSERT_TRUE(rc_link_latest(&in));
    TEST_ASSERT_EQUAL(16, in.num_raw);
    TEST_ASSERT_EQUAL_UINT16(2012, in.raw_us[0]);
    TEST_ASSERT_EQUAL_UINT16(988, in.raw_us[1]);
    TEST_ASSERT_EQUAL_UINT16(1500, in.raw_us[3]);
    for (uint8_t i = 0; i < kMaxRcChannels; ++i) { // same math as pilot.h
        TEST_ASSERT_EQUAL_UINT16(crsf_ticks_to_us(t[i]), in.raw_us[i]);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, in.roll);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -1.0f, in.pitch);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, in.yaw);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, in.throttle);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, in.flaps); // unmapped by default
    TEST_ASSERT_TRUE(switch_on(in, PilotSwitch::kBypass));
    TEST_ASSERT_FALSE(switch_on(in, PilotSwitch::kAutonomy));
    TEST_ASSERT_EQUAL_UINT32(1234, in.stamp_ms);
    TEST_ASSERT_TRUE(manual_active(in, pilot_fresh(in, PilotParams(), 1240)));

    const RcLinkStatus& s = rc_link_status();
    TEST_ASSERT_TRUE(s.frames_seen);
    TEST_ASSERT_EQUAL_UINT32(1234, s.last_frame_ms);
    TEST_ASSERT_EQUAL_UINT32(0, s.bad_frames);
}

void test_switch_threshold_and_mid_position() {
    uint16_t t[kMaxRcChannels];
    for (auto& v : t) {
        v = 992; // 1500 µs: a 3-position switch in the middle reads off
    }
    push(rc_frame(t));
    rc_link_update(1);
    PilotInput in;
    rc_link_latest(&in);
    TEST_ASSERT_FALSE(switch_on(in, PilotSwitch::kBypass));
}

void test_config_remaps_channels() {
    RcLinkConfig cfg;
    cfg.throttle_ch = 0; // TAER
    cfg.roll_ch = 1;
    cfg.flaps_ch = 5;
    cfg.bypass_ch = -1;
    rc_link_init(g_uart, cfg);
    uint16_t t[kMaxRcChannels];
    for (auto& v : t) {
        v = 1811;
    }
    t[1] = 992;
    push(rc_frame(t));
    rc_link_update(1);
    PilotInput in;
    rc_link_latest(&in);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, in.throttle);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, in.roll);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, in.flaps);
    TEST_ASSERT_EQUAL_UINT16(0, in.switches); // bypass unmapped
}

void test_link_stats() {
    push(centered_frame());
    push(link_stats(70, 85, 96, 0)); // antenna 1 active
    rc_link_update(5);
    TEST_ASSERT_EQUAL_UINT8(96, rc_link_status().link_quality);
    TEST_ASSERT_EQUAL_INT8(-70, rc_link_status().rssi_dbm);
    TEST_ASSERT_EQUAL_UINT8(96, rc_link_quality_pct(rc_link_status()));
    push(link_stats(70, 85, 40, 1)); // antenna 2 active
    rc_link_update(6);
    TEST_ASSERT_EQUAL_INT8(-85, rc_link_status().rssi_dbm);
    TEST_ASSERT_EQUAL_UINT8(40, rc_link_status().link_quality);
    TEST_ASSERT_FALSE(rc_lost_raw(rc_link_status(), PilotParams(), 7));
}

// The ER8's first RC frames arrive before its first stats frame (~100 ms):
// that window must not read as RC lost, and LQ reports unknown.
void test_rc_frames_before_stats_are_not_lost() {
    push(centered_frame());
    rc_link_update(5);
    TEST_ASSERT_TRUE(rc_link_status().frames_seen);
    TEST_ASSERT_FALSE(rc_link_status().stats_seen);
    TEST_ASSERT_FALSE(rc_lost_raw(rc_link_status(), PilotParams(), 5));
    TEST_ASSERT_EQUAL_UINT8(kLinkQualityUnknown,
                            rc_link_quality_pct(rc_link_status()));
    push(link_stats(60, 60, 0, 0)); // receiver reports LQ 0: now it counts
    rc_link_update(6);
    TEST_ASSERT_TRUE(rc_link_status().stats_seen);
    TEST_ASSERT_TRUE(rc_lost_raw(rc_link_status(), PilotParams(), 6));
}

void test_link_stats_alone_do_not_count_as_frames() {
    push(link_stats(60, 60, 100, 0));
    rc_link_update(5);
    TEST_ASSERT_FALSE(rc_link_status().frames_seen);
    PilotInput in;
    TEST_ASSERT_FALSE(rc_link_latest(&in));
}

void test_frames_stop_means_rc_lost() {
    push(centered_frame());
    push(link_stats(60, 60, 100, 0));
    rc_link_update(100);
    PilotParams p;
    TEST_ASSERT_FALSE(rc_lost_raw(rc_link_status(), p, 150));
    rc_link_update(300); // receiver went quiet (ELRS failsafe)
    TEST_ASSERT_TRUE(rc_lost_raw(rc_link_status(), p, 300));
    PilotInput in;
    TEST_ASSERT_TRUE(rc_link_latest(&in)); // last frame is kept...
    TEST_ASSERT_FALSE(pilot_fresh(in, p, 300)); // ...but stale
}

void test_bad_crc_dropped_and_counted() {
    std::vector<uint8_t> f = centered_frame();
    f.back() ^= 0xFF;
    push(f);
    rc_link_update(1);
    PilotInput in;
    TEST_ASSERT_FALSE(rc_link_latest(&in));
    TEST_ASSERT_EQUAL_UINT32(1, rc_link_status().bad_frames);
    push(centered_frame()); // the next good frame still parses
    rc_link_update(2);
    TEST_ASSERT_TRUE(rc_link_latest(&in));
}

void test_resync_after_noise_and_split_frames() {
    // Junk with fake sync bytes and a bad length before a real frame.
    push({0x00, 0xC8, 0xFF, 0x12, 0xC8, 0x01, 0x55});
    std::vector<uint8_t> f = centered_frame();
    // The frame arrives in two pieces across loops.
    push(std::vector<uint8_t>(f.begin(), f.begin() + 10));
    rc_link_update(1);
    PilotInput in;
    TEST_ASSERT_FALSE(rc_link_latest(&in));
    push(std::vector<uint8_t>(f.begin() + 10, f.end()));
    rc_link_update(2);
    TEST_ASSERT_TRUE(rc_link_latest(&in));
    TEST_ASSERT_EQUAL_UINT16(1500, in.raw_us[0]);
    TEST_ASSERT_EQUAL_UINT32(2, in.stamp_ms);
}

void test_other_frame_types_ignored() {
    push(frame(0x29, {1, 2, 3, 4})); // device info etc.
    push(centered_frame());
    rc_link_update(1);
    PilotInput in;
    TEST_ASSERT_TRUE(rc_link_latest(&in));
    TEST_ASSERT_EQUAL_UINT32(0, rc_link_status().bad_frames);
}

void test_burst_keeps_newest_frame() {
    // Many frames in one loop (500 Hz packet rate, 20 ms loop): the newest
    // one wins and nothing is lost to the 64-byte assembly buffer.
    uint16_t t[kMaxRcChannels];
    for (uint16_t k = 0; k < 10; ++k) {
        for (auto& v : t) {
            v = static_cast<uint16_t>(500 + k);
        }
        push(rc_frame(t));
        push(link_stats(60, 60, 100, 0));
    }
    rc_link_update(20);
    PilotInput in;
    rc_link_latest(&in);
    TEST_ASSERT_EQUAL_UINT16(crsf_ticks_to_us(509), in.raw_us[0]);
    TEST_ASSERT_EQUAL_UINT32(0, rc_link_status().bad_frames);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_crc8_dvb_s2);
    RUN_TEST(test_unpack_round_trip);
    RUN_TEST(test_nothing_before_first_frame);
    RUN_TEST(test_channels_become_pilot_input);
    RUN_TEST(test_switch_threshold_and_mid_position);
    RUN_TEST(test_config_remaps_channels);
    RUN_TEST(test_link_stats);
    RUN_TEST(test_rc_frames_before_stats_are_not_lost);
    RUN_TEST(test_link_stats_alone_do_not_count_as_frames);
    RUN_TEST(test_frames_stop_means_rc_lost);
    RUN_TEST(test_bad_crc_dropped_and_counted);
    RUN_TEST(test_resync_after_noise_and_split_frames);
    RUN_TEST(test_other_frame_types_ignored);
    RUN_TEST(test_burst_keeps_newest_frame);
    return UNITY_END();
}
