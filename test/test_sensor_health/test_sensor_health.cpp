// SensorHealth state machine: NOMINAL <-> DEGRADED <-> FAILED.
#include <unity.h>

#include "SensorHealth.h"

using sensors::SensorHealth;
using sensors::sensor_status;

// fail 3 -> DEGRADED, fail 4 more -> FAILED,
// 2 good from DEGRADED -> NOMINAL, 5 good from FAILED -> DEGRADED
static const sensors::sensor_trust_limits LIMITS = {3, 4, 2, 5};

static SensorHealth h(LIMITS);

static void reads(bool ok, int n) {
    for (int i = 0; i < n; i++) {
        h.update_sensor_freshness(ok);
    }
}

void setUp(void) {
    h = SensorHealth(LIMITS);
    h.begin(true);
}
void tearDown(void) {}

void test_starts_uninitialized_and_ignores_reads(void) {
    SensorHealth fresh_h(LIMITS);
    TEST_ASSERT(fresh_h.get_status() == sensor_status::UNINITIALIZED);
    fresh_h.update_sensor_freshness(false);
    TEST_ASSERT(fresh_h.get_status() == sensor_status::UNINITIALIZED);
}

void test_begin_sets_nominal_or_failed(void) {
    TEST_ASSERT(h.get_status() == sensor_status::NOMINAL);
    TEST_ASSERT_FALSE(h.begin(false));
    TEST_ASSERT(h.get_status() == sensor_status::FAILED);
}

void test_degrades_after_fail_threshold(void) {
    reads(false, 2);
    TEST_ASSERT(h.get_status() == sensor_status::NOMINAL);
    reads(false, 1);
    TEST_ASSERT(h.get_status() == sensor_status::DEGRADED);
}

void test_a_success_resets_the_failure_streak(void) {
    reads(false, 2);
    reads(true, 1);
    reads(false, 2);
    TEST_ASSERT(h.get_status() == sensor_status::NOMINAL);
}

void test_fails_after_full_fail_threshold_while_degraded(void) {
    reads(false, 3);
    reads(false, 3);
    TEST_ASSERT(h.get_status() == sensor_status::DEGRADED);
    reads(false, 1);
    TEST_ASSERT(h.get_status() == sensor_status::FAILED);
}

void test_recovers_from_degraded(void) {
    reads(false, 3);
    reads(true, 1);
    TEST_ASSERT(h.get_status() == sensor_status::DEGRADED);
    reads(true, 1);
    TEST_ASSERT(h.get_status() == sensor_status::NOMINAL);
}

void test_failed_recovers_one_step_at_a_time(void) {
    h.begin(false);
    reads(true, 4);
    TEST_ASSERT(h.get_status() == sensor_status::FAILED);
    reads(true, 1);
    TEST_ASSERT(h.get_status() == sensor_status::DEGRADED);
    reads(true, 2);
    TEST_ASSERT(h.get_status() == sensor_status::NOMINAL);
}

void test_failed_stays_failed_while_failing(void) {
    h.begin(false);
    reads(false, 50);
    TEST_ASSERT(h.get_status() == sensor_status::FAILED);
}

void test_degrade_forces_degraded(void) {
    h.degrade();
    TEST_ASSERT(h.get_status() == sensor_status::DEGRADED);
}

void test_fresh_tracks_last_read(void) {
    TEST_ASSERT_FALSE(h.fresh());
    h.on_success();
    TEST_ASSERT_TRUE(h.fresh());
    h.on_failure();
    TEST_ASSERT_FALSE(h.fresh());
}

void test_counters_saturate_instead_of_wrapping(void) {
    sensors::sensor_trust_limits never = {255, 255, 255, 255};
    h.set_limits(never);
    reads(false, 300); // would wrap a uint8_t back to 44
    TEST_ASSERT(h.get_status() == sensor_status::DEGRADED);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_starts_uninitialized_and_ignores_reads);
    RUN_TEST(test_begin_sets_nominal_or_failed);
    RUN_TEST(test_degrades_after_fail_threshold);
    RUN_TEST(test_a_success_resets_the_failure_streak);
    RUN_TEST(test_fails_after_full_fail_threshold_while_degraded);
    RUN_TEST(test_recovers_from_degraded);
    RUN_TEST(test_failed_recovers_one_step_at_a_time);
    RUN_TEST(test_failed_stays_failed_while_failing);
    RUN_TEST(test_degrade_forces_degraded);
    RUN_TEST(test_fresh_tracks_last_read);
    RUN_TEST(test_counters_saturate_instead_of_wrapping);
    return UNITY_END();
}
