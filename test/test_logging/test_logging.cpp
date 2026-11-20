#include <unity.h>
#include <vector>
#include <string.h>
#include <Arduino.h> // stub: g_millis
#include <SdFat.h>   // stub: g_sd (records what the logger writes)
#include "logging_subsystem.h"
#include "logging_interface.h"

typedef std::vector<uint8_t> Bytes;

// ---- helpers ---------------------------------------------------------------

// Build the packet the firmware is expected to produce (independent of firmware code).
static Bytes make_packet(uint8_t sys, uint8_t sub, uint8_t level, uint32_t ts, const Bytes &payload) {
    Bytes p = {0xFE, 0xED, sys, sub, level,
               (uint8_t)(ts >> 24), (uint8_t)(ts >> 16), (uint8_t)(ts >> 8), (uint8_t)ts,
               (uint8_t)payload.size()};
    p.insert(p.end(), payload.begin(), payload.end());
    p.push_back(0xDE);
    p.push_back(0xAD);
    return p;
}

static Bytes flush_all() {
    write_to_sd(true);
    return g_sd.data;
}

static void assert_bytes(const Bytes &expected, const Bytes &actual) {
    TEST_ASSERT_EQUAL_UINT32(expected.size(), actual.size());
    if (!expected.empty())
        TEST_ASSERT_EQUAL_UINT8_ARRAY(expected.data(), actual.data(), expected.size());
}

// 10-byte header + 20-byte payload + 2-byte end marker = 32 bytes; 4096 / 32 = 128 packets
static const uint8_t P20 = 20;
static bool log_n_byte_payload(uint8_t n) {
    uint8_t payload[64];
    memset(payload, 0x55, sizeof(payload));
    return log_message(SENSOR, SUB_IMU, LOG_INFO, payload, n);
}

void setUp(void) {
    g_sd.reset();
    g_millis = 0x01020304;
    TEST_ASSERT_TRUE(initialize_sd());
}
void tearDown(void) {}

// ---- initialization --------------------------------------------------------

void test_init_opens_log_file_in_append_mode(void) {
    TEST_ASSERT_EQUAL_INT(1, g_sd.open_calls);
    TEST_ASSERT_EQUAL_STRING("datalog.bin", g_sd.filename.c_str());
    TEST_ASSERT_EQUAL_INT(O_WRONLY | O_CREAT | O_AT_END, g_sd.open_flags);
}

void test_init_fails_when_sd_begin_fails(void) {
    g_sd.reset();
    g_sd.fail_begin = true;
    TEST_ASSERT_FALSE(initialize_sd());
    TEST_ASSERT_EQUAL_INT(0, g_sd.open_calls);
}

void test_init_fails_when_file_cannot_open(void) {
    g_sd.reset();
    g_sd.fail_open = true;
    TEST_ASSERT_FALSE(initialize_sd());
}

void test_init_clears_buffer_and_drop_counter(void) {
    for (int i = 0; i < 129; i++)
        log_n_byte_payload(P20); // 129th is dropped
    TEST_ASSERT_EQUAL_UINT32(1, logging_dropped());

    g_sd.reset();
    TEST_ASSERT_TRUE(initialize_sd());
    TEST_ASSERT_EQUAL_UINT32(0, logging_dropped());
    assert_bytes(Bytes(), flush_all()); // old data is gone
}

// ---- packet encoding -------------------------------------------------------

void test_imu_packet_matches_golden_bytes(void) {
    // Same vector is checked by the Python decoder tests.
    ImuLogger imu;
    imu_raw_t s = {1, 255, -3};
    TEST_ASSERT_TRUE(imu.loggable(SENSOR, SUB_IMU, &s));

    Bytes expected = {0xFE, 0xED, 0x00, 0x01, 0x00, 0x01, 0x02, 0x03, 0x04, 0x0C,
                      0x00, 0x00, 0x00, 0x01,
                      0x00, 0x00, 0x00, 0xFF,
                      0xFF, 0xFF, 0xFF, 0xFD,
                      0xDE, 0xAD};
    assert_bytes(expected, flush_all());
}

void test_baro_logger_encodes_negative_value_big_endian(void) {
    BaroLogger baro;
    baro_raw_t s = {-2};
    TEST_ASSERT_TRUE(baro.loggable(SENSOR, SUB_BARO, &s));
    assert_bytes(make_packet(SENSOR, SUB_BARO, LOG_INFO, 0x01020304, {0xFF, 0xFF, 0xFF, 0xFE}), flush_all());
}

void test_imu_logger_encodes_multibyte_values_big_endian(void) {
    ImuLogger imu;
    imu_raw_t s = {0x01020304, (int)0x80000000, 0x7FFFFFFF};
    TEST_ASSERT_TRUE(imu.loggable(SENSOR, SUB_IMU, &s));
    Bytes payload = {0x01, 0x02, 0x03, 0x04, 0x80, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xFF};
    assert_bytes(make_packet(SENSOR, SUB_IMU, LOG_INFO, 0x01020304, payload), flush_all());
}

void test_timestamp_is_big_endian(void) {
    g_millis = 0xA1B2C3D4;
    uint8_t b = 1;
    TEST_ASSERT_TRUE(log_message(SENSOR, SUB_IMU, LOG_INFO, &b, 1));
    Bytes out = flush_all();
    TEST_ASSERT_EQUAL_HEX8(0xA1, out[5]);
    TEST_ASSERT_EQUAL_HEX8(0xB2, out[6]);
    TEST_ASSERT_EQUAL_HEX8(0xC3, out[7]);
    TEST_ASSERT_EQUAL_HEX8(0xD4, out[8]);
}

void test_system_subsystem_and_level_are_encoded(void) {
    uint8_t b = 9;
    TEST_ASSERT_TRUE(log_message(0x12, 0x34, LOG_WARN, &b, 1));
    assert_bytes(make_packet(0x12, 0x34, 0x0F, 0x01020304, {9}), flush_all());
}

void test_empty_payload_with_null_pointer_is_valid(void) {
    TEST_ASSERT_TRUE(log_message(SENSOR, SUB_IMU, LOG_ERROR, nullptr, 0));
    Bytes out = flush_all();
    TEST_ASSERT_EQUAL_UINT32(12, out.size());
    assert_bytes(make_packet(SENSOR, SUB_IMU, LOG_ERROR, 0x01020304, {}), out);
}

void test_max_payload_accepted_and_oversize_rejected(void) {
    TEST_ASSERT_TRUE(log_n_byte_payload(64));
    TEST_ASSERT_FALSE(log_n_byte_payload(65));
    TEST_ASSERT_FALSE(log_n_byte_payload(255));
    TEST_ASSERT_EQUAL_UINT32(10 + 64 + 2, flush_all().size()); // only the valid one
}

void test_null_payload_with_nonzero_length_rejected(void) {
    TEST_ASSERT_FALSE(log_message(SENSOR, SUB_IMU, LOG_INFO, nullptr, 4));
    assert_bytes(Bytes(), flush_all());
}

void test_marker_bytes_inside_payload_are_preserved(void) {
    uint8_t payload[] = {0xFE, 0xED, 0xDE, 0xAD};
    TEST_ASSERT_TRUE(log_message(SENSOR, SUB_IMU, LOG_INFO, payload, sizeof(payload)));
    Bytes out = flush_all();
    TEST_ASSERT_EQUAL_UINT32(16, out.size());
    assert_bytes(make_packet(SENSOR, SUB_IMU, LOG_INFO, 0x01020304, {0xFE, 0xED, 0xDE, 0xAD}), out);
}

void test_packets_are_written_back_to_back_in_order(void) {
    uint8_t a = 1, b = 2;
    g_millis = 100;
    TEST_ASSERT_TRUE(log_message(SENSOR, SUB_IMU, LOG_INFO, &a, 1));
    g_millis = 200;
    TEST_ASSERT_TRUE(log_message(SENSOR, SUB_BARO, LOG_INFO, &b, 1));

    Bytes expected = make_packet(SENSOR, SUB_IMU, LOG_INFO, 100, {1});
    Bytes second = make_packet(SENSOR, SUB_BARO, LOG_INFO, 200, {2});
    expected.insert(expected.end(), second.begin(), second.end());
    assert_bytes(expected, flush_all());
}

// ---- buffer full / drops ---------------------------------------------------

void test_full_buffer_drops_packet_and_counts_it(void) {
    for (int i = 0; i < 128; i++)
        TEST_ASSERT_TRUE(log_n_byte_payload(P20));
    TEST_ASSERT_FALSE(log_n_byte_payload(P20));
    TEST_ASSERT_FALSE(log_n_byte_payload(P20));
    TEST_ASSERT_EQUAL_UINT32(2, logging_dropped());
    TEST_ASSERT_EQUAL_UINT32(4096, flush_all().size());
}

void test_no_partial_packet_written_when_space_is_short(void) {
    // 33-byte packets: 124 fit (4092 bytes), 4 bytes left, 125th must be rejected whole.
    for (int i = 0; i < 124; i++)
        TEST_ASSERT_TRUE(log_n_byte_payload(21));
    TEST_ASSERT_FALSE(log_n_byte_payload(21));
    TEST_ASSERT_EQUAL_UINT32(1, logging_dropped());

    Bytes out = flush_all();
    TEST_ASSERT_EQUAL_UINT32(124 * 33, out.size());
    for (size_t i = 0; i < out.size(); i += 33) { // every packet intact
        TEST_ASSERT_EQUAL_HEX8(0xFE, out[i]);
        TEST_ASSERT_EQUAL_HEX8(0xED, out[i + 1]);
        TEST_ASSERT_EQUAL_HEX8(0xDE, out[i + 31]);
        TEST_ASSERT_EQUAL_HEX8(0xAD, out[i + 32]);
    }
}

void test_logging_works_again_after_flush(void) {
    for (int i = 0; i < 128; i++)
        log_n_byte_payload(P20);
    TEST_ASSERT_FALSE(log_n_byte_payload(P20));
    flush_all();
    TEST_ASSERT_TRUE(log_n_byte_payload(P20));
}

// ---- writing to the SD card ------------------------------------------------

void test_nothing_written_below_one_sector(void) {
    log_n_byte_payload(P20);
    TEST_ASSERT_TRUE(write_to_sd());
    TEST_ASSERT_EQUAL_UINT32(0, g_sd.data.size());
    TEST_ASSERT_EQUAL_UINT32(0, g_sd.write_sizes.size());
}

void test_only_whole_512_byte_sectors_written_without_force(void) {
    for (int i = 0; i < 20; i++)
        log_n_byte_payload(P20); // 640 bytes queued
    TEST_ASSERT_TRUE(write_to_sd());
    TEST_ASSERT_EQUAL_UINT32(512, g_sd.data.size());
    TEST_ASSERT_EQUAL_UINT32(1, g_sd.write_sizes.size());
    TEST_ASSERT_EQUAL_UINT32(512, g_sd.write_sizes[0]);

    TEST_ASSERT_TRUE(write_to_sd(true)); // remaining 128 bytes
    TEST_ASSERT_EQUAL_UINT32(640, g_sd.data.size());
}

void test_force_writes_partial_data_and_syncs(void) {
    log_n_byte_payload(P20);
    int syncs = g_sd.sync_calls;
    TEST_ASSERT_TRUE(write_to_sd(true));
    TEST_ASSERT_EQUAL_UINT32(32, g_sd.data.size());
    TEST_ASSERT_TRUE(g_sd.sync_calls > syncs);
}

void test_data_survives_ring_wraparound_in_order(void) {
    Bytes expected;
    for (int i = 0; i < 400; i++) { // 400 * 28 B > 4096 B buffer
        Bytes payload(16, (uint8_t)i);
        payload[0] = (uint8_t)(i >> 8);
        g_millis = 1000 + i;
        TEST_ASSERT_TRUE(log_message(SENSOR, SUB_IMU, LOG_INFO, payload.data(), 16));
        Bytes p = make_packet(SENSOR, SUB_IMU, LOG_INFO, 1000 + i, payload);
        expected.insert(expected.end(), p.begin(), p.end());
        if (i % 7 == 0)
            write_to_sd(true); // irregular drain
    }
    write_to_sd(true);
    TEST_ASSERT_EQUAL_UINT32(0, logging_dropped());
    assert_bytes(expected, g_sd.data);
}

void test_write_failure_is_reported(void) {
    log_n_byte_payload(P20);
    g_sd.fail_write = true;
    TEST_ASSERT_FALSE(write_to_sd(true));
}

void test_sync_happens_at_most_once_per_second(void) {
    const uint32_t T = 0x70000000;
    g_millis = T;
    write_to_sd(); // establish baseline
    int base = g_sd.sync_calls;

    g_millis = T + 999;
    write_to_sd();
    TEST_ASSERT_EQUAL_INT(base, g_sd.sync_calls); // too soon

    g_millis = T + 1000;
    write_to_sd();
    TEST_ASSERT_EQUAL_INT(base + 1, g_sd.sync_calls);
}

void test_close_flushes_everything_then_closes(void) {
    log_n_byte_payload(P20);
    logging_close();
    TEST_ASSERT_EQUAL_UINT32(32, g_sd.data.size());
    TEST_ASSERT_EQUAL_INT(1, g_sd.close_calls);
}

void test_write_to_sd_fails_when_file_is_closed(void) {
    logging_close();
    TEST_ASSERT_FALSE(write_to_sd());
    TEST_ASSERT_FALSE(write_to_sd(true));
}

// ---- additional SD write / sync edge cases ---------------------------------

void test_exactly_one_sector_written_without_force(void) {
    for (int i = 0; i < 16; i++) {
        log_n_byte_payload(P20); // 16 * 32 = 512 bytes queued
    }
    TEST_ASSERT_TRUE(write_to_sd());
    TEST_ASSERT_EQUAL_UINT32(1, g_sd.write_sizes.size());
    TEST_ASSERT_EQUAL_UINT32(512, g_sd.write_sizes[0]);
    TEST_ASSERT_EQUAL_UINT32(512, g_sd.data.size());
}

void test_multiple_sectors_written_in_one_call(void) {
    for (int i = 0; i < 50; i++) {
        log_n_byte_payload(P20); // 50 * 32 = 1600 bytes queued
    }
    TEST_ASSERT_TRUE(write_to_sd());
    TEST_ASSERT_EQUAL_UINT32(3, g_sd.write_sizes.size()); // 3 * 512 = 1536
    for (size_t i = 0; i < g_sd.write_sizes.size(); i++) {
        TEST_ASSERT_EQUAL_UINT32(512, g_sd.write_sizes[i]);
    }
    TEST_ASSERT_EQUAL_UINT32(1536, g_sd.data.size()); // 64 bytes left queued

    TEST_ASSERT_TRUE(write_to_sd(true));
    TEST_ASSERT_EQUAL_UINT32(1600, g_sd.data.size());
}

void test_force_on_empty_buffer_succeeds_and_syncs(void) {
    int syncs = g_sd.sync_calls;
    TEST_ASSERT_TRUE(write_to_sd(true));
    TEST_ASSERT_EQUAL_UINT32(0, g_sd.data.size());
    TEST_ASSERT_EQUAL_UINT32(0, g_sd.write_sizes.size());
    TEST_ASSERT_EQUAL_INT(syncs + 1, g_sd.sync_calls);
}

void test_sync_timer_survives_millis_wraparound(void) {
    const uint32_t T = 0xFFFFFF00; // millis() wraps ~49.7 days after boot
    g_millis = T;
    write_to_sd(true); // forced sync sets the baseline
    int base = g_sd.sync_calls;

    g_millis = T + 999; // wraps past 0
    write_to_sd();
    TEST_ASSERT_EQUAL_INT(base, g_sd.sync_calls); // too soon

    g_millis = T + 1000;
    write_to_sd();
    TEST_ASSERT_EQUAL_INT(base + 1, g_sd.sync_calls);
}

void test_logging_recovers_after_write_failure(void) {
    log_n_byte_payload(P20);
    g_sd.fail_write = true;
    TEST_ASSERT_FALSE(write_to_sd(true));

    g_sd.fail_write = false;
    g_millis = 0x0A0B0C0D;
    uint8_t b = 7;
    TEST_ASSERT_TRUE(log_message(SENSOR, SUB_BARO, LOG_WARN, &b, 1));
    TEST_ASSERT_TRUE(write_to_sd(true));

    // The new packet must reach the card intact.
    Bytes expected = make_packet(SENSOR, SUB_BARO, LOG_WARN, 0x0A0B0C0D, {7});
    TEST_ASSERT_TRUE(g_sd.data.size() >= expected.size());
    Bytes tail(g_sd.data.end() - expected.size(), g_sd.data.end());
    assert_bytes(expected, tail);
}

void test_failed_write_counts_lost_bytes(void) {
    log_n_byte_payload(P20); // 32 bytes
    g_sd.fail_write = true;
    TEST_ASSERT_FALSE(write_to_sd(true));
    TEST_ASSERT_EQUAL_UINT32(32, logging_lost_bytes());
    TEST_ASSERT_EQUAL_UINT32(0, logging_dropped()); // buffer-full drops are separate
}

void test_failed_sector_write_counts_whole_sector(void) {
    for (int i = 0; i < 20; i++) {
        log_n_byte_payload(P20); // 640 bytes queued
    }
    g_sd.fail_write = true;
    TEST_ASSERT_FALSE(write_to_sd());
    TEST_ASSERT_EQUAL_UINT32(512, logging_lost_bytes()); // stops after first failure

    g_sd.fail_write = false;
    TEST_ASSERT_TRUE(write_to_sd(true)); // remaining 128 bytes still get written
    TEST_ASSERT_EQUAL_UINT32(128, g_sd.data.size());
    TEST_ASSERT_EQUAL_UINT32(512, logging_lost_bytes());
}

void test_successful_writes_lose_nothing(void) {
    for (int i = 0; i < 50; i++) {
        log_n_byte_payload(P20);
    }
    TEST_ASSERT_TRUE(write_to_sd(true));
    TEST_ASSERT_EQUAL_UINT32(0, logging_lost_bytes());
}

void test_init_clears_lost_byte_count(void) {
    log_n_byte_payload(P20);
    g_sd.fail_write = true;
    write_to_sd(true);
    TEST_ASSERT_TRUE(logging_lost_bytes() > 0);

    g_sd.reset();
    TEST_ASSERT_TRUE(initialize_sd());
    TEST_ASSERT_EQUAL_UINT32(0, logging_lost_bytes());
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_init_opens_log_file_in_append_mode);
    RUN_TEST(test_init_fails_when_sd_begin_fails);
    RUN_TEST(test_init_fails_when_file_cannot_open);
    RUN_TEST(test_init_clears_buffer_and_drop_counter);
    RUN_TEST(test_imu_packet_matches_golden_bytes);
    RUN_TEST(test_baro_logger_encodes_negative_value_big_endian);
    RUN_TEST(test_imu_logger_encodes_multibyte_values_big_endian);
    RUN_TEST(test_timestamp_is_big_endian);
    RUN_TEST(test_system_subsystem_and_level_are_encoded);
    RUN_TEST(test_empty_payload_with_null_pointer_is_valid);
    RUN_TEST(test_max_payload_accepted_and_oversize_rejected);
    RUN_TEST(test_null_payload_with_nonzero_length_rejected);
    RUN_TEST(test_marker_bytes_inside_payload_are_preserved);
    RUN_TEST(test_packets_are_written_back_to_back_in_order);
    RUN_TEST(test_full_buffer_drops_packet_and_counts_it);
    RUN_TEST(test_no_partial_packet_written_when_space_is_short);
    RUN_TEST(test_logging_works_again_after_flush);
    RUN_TEST(test_nothing_written_below_one_sector);
    RUN_TEST(test_only_whole_512_byte_sectors_written_without_force);
    RUN_TEST(test_force_writes_partial_data_and_syncs);
    RUN_TEST(test_data_survives_ring_wraparound_in_order);
    RUN_TEST(test_write_failure_is_reported);
    RUN_TEST(test_sync_happens_at_most_once_per_second);
    RUN_TEST(test_close_flushes_everything_then_closes);
    RUN_TEST(test_write_to_sd_fails_when_file_is_closed);
    RUN_TEST(test_exactly_one_sector_written_without_force);
    RUN_TEST(test_multiple_sectors_written_in_one_call);
    RUN_TEST(test_force_on_empty_buffer_succeeds_and_syncs);
    RUN_TEST(test_sync_timer_survives_millis_wraparound);
    RUN_TEST(test_logging_recovers_after_write_failure);
    RUN_TEST(test_failed_write_counts_lost_bytes);
    RUN_TEST(test_failed_sector_write_counts_whole_sector);
    RUN_TEST(test_successful_writes_lose_nothing);
    RUN_TEST(test_init_clears_lost_byte_count);
    return UNITY_END();
}
