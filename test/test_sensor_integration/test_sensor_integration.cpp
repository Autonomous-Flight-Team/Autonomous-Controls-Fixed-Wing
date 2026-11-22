// Sensors -> SensorManager -> logging -> SD, with dummy sensors and the SD
// stub. Checks rates, packet layout and fault handling end to end.
#include <unity.h>
#include <map>
#include <vector>

#include <Arduino.h> // stub: g_millis
#include <SdFat.h>   // stub: g_sd
#include "logging_subsystem.h"
#include "sensor_suite.h"

using sensors::sensor_status;

static sensors::SensorSuite suite;

struct Packet {
    uint8_t sys, sub, len;
};

// Splits the SD stream into packets, checking every frame on the way.
static std::vector<Packet> packets() {
    write_to_sd(true);
    const std::vector<uint8_t> &d = g_sd.data;
    std::vector<Packet> out;
    size_t i = 0;
    while (i < d.size()) {
        TEST_ASSERT_TRUE(i + 12 <= d.size());
        TEST_ASSERT_EQUAL_HEX8(0xFE, d[i]);
        TEST_ASSERT_EQUAL_HEX8(0xED, d[i + 1]);
        Packet p = {d[i + 2], d[i + 3], d[i + 9]};
        size_t end = i + 10 + p.len;
        TEST_ASSERT_TRUE(end + 2 <= d.size());
        TEST_ASSERT_EQUAL_HEX8(0xDE, d[end]);
        TEST_ASSERT_EQUAL_HEX8(0xAD, d[end + 1]);
        out.push_back(p);
        i = end + 2;
    }
    return out;
}

static std::map<uint8_t, int> count_by_sub(const std::vector<Packet> &ps) {
    std::map<uint8_t, int> n;
    for (const Packet &p : ps) {
        n[p.sub]++;
    }
    return n;
}

// Runs the loop for `us` microseconds of simulated time in 100 us steps,
// flushing like the firmware does.
static uint32_t t_us = 0;
static void run_for(uint32_t us, bool log_enabled = true) {
    for (uint32_t end = t_us + us; t_us < end; t_us += 100) {
        g_millis = t_us / 1000;
        suite.update(t_us, log_enabled);
        write_to_sd();
    }
}

void setUp(void) {
    g_sd.reset();
    g_millis = 0;
    t_us = 0;
    TEST_ASSERT_TRUE(initialize_sd());
    suite = sensors::SensorSuite(); // fresh sensors, counters and timers
}
void tearDown(void) {}

void test_native_build_uses_dummy_sensors(void) {
    TEST_ASSERT_EQUAL_INT(0, GOOSE_SENSORS_ARE_REAL);
}

void test_begin_logs_trust_limits_first(void) {
    TEST_ASSERT_TRUE(suite.begin(true));
    std::vector<Packet> ps = packets();
    TEST_ASSERT_EQUAL_UINT32(1, ps.size());
    TEST_ASSERT_EQUAL_HEX8(SENSOR, ps[0].sys);
    TEST_ASSERT_EQUAL_HEX8(SUB_SENSOR_TRUST, ps[0].sub);
    TEST_ASSERT_EQUAL_UINT8(sensors::TRUST_PAYLOAD_LEN, ps[0].len);
}

void test_one_second_logs_each_sensor_at_its_rate(void) {
    suite.begin(false);
    run_for(1000000);
    std::map<uint8_t, int> n = count_by_sub(packets());
    TEST_ASSERT_EQUAL_INT(CALL_IMU, n[SUB_IMU]);
    TEST_ASSERT_EQUAL_INT(CALL_BARO, n[SUB_BARO]);
    TEST_ASSERT_EQUAL_INT(CALL_GPS, n[SUB_GPS]);
    TEST_ASSERT_EQUAL_INT(CALL_PITOT, n[SUB_PITOT]);
    TEST_ASSERT_EQUAL_UINT32(0, logging_dropped());
}

void test_every_packet_has_its_sensors_payload_size(void) {
    suite.begin(true);
    run_for(200000);
    for (const Packet &p : packets()) {
        uint8_t want = 0;
        switch (p.sub) {
        case SUB_IMU: want = sensors::IMU_PAYLOAD_LEN; break;
        case SUB_BARO: want = sensors::BARO_PAYLOAD_LEN; break;
        case SUB_GPS: want = sensors::GPS_PAYLOAD_LEN; break;
        case SUB_PITOT: want = sensors::PITOT_PAYLOAD_LEN; break;
        case SUB_SENSOR_TRUST: want = sensors::TRUST_PAYLOAD_LEN; break;
        default: TEST_FAIL_MESSAGE("unknown subsystem in log");
        }
        TEST_ASSERT_EQUAL_UINT8(want, p.len);
    }
}

void test_logging_disabled_still_reads_but_writes_nothing(void) {
    suite.begin(false);
    run_for(100000, false);
    TEST_ASSERT_EQUAL_UINT32(0, packets().size());
    TEST_ASSERT_EQUAL_UINT32(10, suite.imu_counts.reads);
    TEST_ASSERT_EQUAL_UINT32(0, suite.imu_counts.logged);
}

void test_failing_sensor_is_not_logged_and_degrades(void) {
    suite.begin(false);
    suite.imu.set_failing(true);
    run_for(1000000);
    std::map<uint8_t, int> n = count_by_sub(packets());
    TEST_ASSERT_EQUAL_INT(0, n[SUB_IMU]);
    TEST_ASSERT_EQUAL_INT(CALL_BARO, n[SUB_BARO]); // others unaffected
    TEST_ASSERT_EQUAL_UINT32(CALL_IMU, suite.imu_counts.stale);
    TEST_ASSERT(suite.imu.getStatus() == sensor_status::FAILED);
    TEST_ASSERT_FALSE(suite.all_usable());
}

void test_failed_sensor_recovers_and_logs_again(void) {
    suite.begin(false);
    suite.imu.set_failing(true);
    run_for(1000000);
    suite.imu.set_failing(false);
    g_sd.reset();
    TEST_ASSERT_TRUE(initialize_sd());
    run_for(1000000);
    TEST_ASSERT(suite.imu.getStatus() == sensor_status::NOMINAL);
    TEST_ASSERT_EQUAL_INT(CALL_IMU, count_by_sub(packets())[SUB_IMU]);
    TEST_ASSERT_TRUE(suite.all_usable());
}

void test_sensor_that_fails_to_start_is_failed_not_fatal(void) {
    suite.pitot.set_fail_begin(true);
    TEST_ASSERT_FALSE(suite.begin(false));
    TEST_ASSERT(suite.pitot.getStatus() == sensor_status::FAILED);
    TEST_ASSERT(suite.imu.getStatus() == sensor_status::NOMINAL);
    run_for(100000);
    TEST_ASSERT_EQUAL_INT(10, count_by_sub(packets())[SUB_IMU]);
}

void test_dummy_data_changes_every_sample(void) {
    suite.begin(false);
    sensors::BaroData a = suite.baro.get();
    sensors::BaroData b = suite.baro.get();
    TEST_ASSERT_TRUE(a.pressure_Pa != b.pressure_Pa);
}

// ---- RateGate ---------------------------------------------------------------

void test_rate_gate_fires_immediately_then_every_period(void) {
    RateGate g(100); // 10 ms
    TEST_ASSERT_TRUE(g.due(5000));
    TEST_ASSERT_FALSE(g.due(14999));
    TEST_ASSERT_TRUE(g.due(15000));
}

void test_rate_gate_skips_missed_ticks_instead_of_bursting(void) {
    RateGate g(100);
    g.due(0);
    TEST_ASSERT_TRUE(g.due(55000)); // 5 periods late: one tick, not five
    TEST_ASSERT_FALSE(g.due(56000));
    TEST_ASSERT_TRUE(g.due(65000));
}

void test_rate_gate_survives_micros_wraparound(void) {
    RateGate g(100);
    g.due(0xFFFFF000u);
    TEST_ASSERT_FALSE(g.due(0xFFFFFFF0u));
    TEST_ASSERT_TRUE(g.due(0xFFFFF000u + 10000u)); // wrapped past zero
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_native_build_uses_dummy_sensors);
    RUN_TEST(test_begin_logs_trust_limits_first);
    RUN_TEST(test_one_second_logs_each_sensor_at_its_rate);
    RUN_TEST(test_every_packet_has_its_sensors_payload_size);
    RUN_TEST(test_logging_disabled_still_reads_but_writes_nothing);
    RUN_TEST(test_failing_sensor_is_not_logged_and_degrades);
    RUN_TEST(test_failed_sensor_recovers_and_logs_again);
    RUN_TEST(test_sensor_that_fails_to_start_is_failed_not_fatal);
    RUN_TEST(test_dummy_data_changes_every_sample);
    RUN_TEST(test_rate_gate_fires_immediately_then_every_period);
    RUN_TEST(test_rate_gate_skips_missed_ticks_instead_of_bursting);
    RUN_TEST(test_rate_gate_survives_micros_wraparound);
    return UNITY_END();
}
