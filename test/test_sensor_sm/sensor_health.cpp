#include <unity.h>
#include <vector>
#include <string.h>
#include <Arduino.h> // stub: g_millis
#include <SdFat.h>   // stub: g_sd (records what the logger writes)
#include "SensorHealth.h"
#include "SensorInterface.h"
#include "SensorStatus.h"

#define test_small_fail_threshold 10
#define test_full_fail_threshold 10
#define test_small_recover_threshold 10
#define test_full_recover_threshold 10

typedef std::vector<uint8_t> Bytes;

// ---- initialization --------------------------------------------------------

void test_sensorhealth_init(void)
{
    sensors::SensorHealth sens_hp = sensors::SensorHealth(test_small_fail_threshold, 
                                                            test_full_fail_threshold, 
                                                            test_small_recover_threshold, 
                                                            test_full_recover_threshold);
    TEST_ASSERT_TRUE(sens_hp.get_status() == sensors::sensor_status::UNINITIALIZED);
    sens_hp.begin(true);
    TEST_ASSERT_TRUE(sens_hp.get_status() == sensors::sensor_status::NOMINAL);
}

void test_sensorhealth_failinit(void) {
    sensors::SensorHealth sens_hp = sensors::SensorHealth(test_small_fail_threshold, 
                                                            test_full_fail_threshold, 
                                                            test_small_recover_threshold, 
                                                            test_full_recover_threshold);
    TEST_ASSERT_TRUE(sens_hp.get_status() == sensors::sensor_status::UNINITIALIZED);
    sens_hp.begin(false);
    TEST_ASSERT_TRUE(sens_hp.get_status() == sensors::sensor_status::DEGRADED);
}

void test_sensorhealth_degrade(void) {
    sensors::SensorHealth sens_hp = sensors::SensorHealth(test_small_fail_threshold, 
                                                            test_full_fail_threshold, 
                                                            test_small_recover_threshold, 
                                                            test_full_recover_threshold);
    TEST_ASSERT_TRUE(sens_hp.get_status() == sensors::sensor_status::UNINITIALIZED);
    sens_hp.begin(true);
    TEST_ASSERT_TRUE(sens_hp.get_status() == sensors::sensor_status::NOMINAL);
    sens_hp.degrade();
    TEST_ASSERT_TRUE(sens_hp.get_status() == sensors::sensor_status::DEGRADED);
}

void test_sensorhealth_10fails(void) {
    sensors::SensorHealth sens_hp = sensors::SensorHealth(test_small_fail_threshold, 
                                                            test_full_fail_threshold, 
                                                            test_small_recover_threshold, 
                                                            test_full_recover_threshold);
    sens_hp.begin(true);
    sens_hp.degrade();
    for(int i = 0; i < test_small_fail_threshold + 1; i++) {
        sens_hp.update_sensor_freshness(false);
    }
    TEST_ASSERT_TRUE(sens_hp.get_status() == sensors::sensor_status::DEGRADED);
}

void test_sensorhealth_20fails(void) {
    sensors::SensorHealth sens_hp = sensors::SensorHealth(test_small_fail_threshold, 
                                                            test_full_fail_threshold, 
                                                            test_small_recover_threshold, 
                                                            test_full_recover_threshold);
    sens_hp.begin(true);
    sens_hp.degrade();
    for(int i = 0; i < test_small_fail_threshold + test_full_fail_threshold + 2; i++) {
        sens_hp.update_sensor_freshness(false);
    }
    TEST_ASSERT_TRUE(sens_hp.get_status() == sensors::sensor_status::FAILED);
}

void test_sensorhealth_10fails_then_recover(void) {
    sensors::SensorHealth sens_hp = sensors::SensorHealth(test_small_fail_threshold, 
                                                            test_full_fail_threshold, 
                                                            test_small_recover_threshold, 
                                                            test_full_recover_threshold);
    sens_hp.begin(true);
    sens_hp.degrade();
    for(int i = 0; i < test_small_fail_threshold + 1; i++) {
        sens_hp.update_sensor_freshness(false);
    }
    TEST_ASSERT_TRUE(sens_hp.get_status() == sensors::sensor_status::DEGRADED);
    for(int i = 0; i < test_small_recover_threshold + 1; i++) {
        sens_hp.update_sensor_freshness(true);
    }
    TEST_ASSERT_TRUE(sens_hp.get_status() == sensors::sensor_status::NOMINAL);
}

void test_sensorhealth_20fails_then_recover(void) {
    sensors::SensorHealth sens_hp = sensors::SensorHealth(test_small_fail_threshold, 
                                                            test_full_fail_threshold, 
                                                            test_small_recover_threshold, 
                                                            test_full_recover_threshold);
    sens_hp.begin(true);
    sens_hp.degrade();
    for(int i = 0; i < test_small_fail_threshold + test_full_fail_threshold + 2; i++) {
        sens_hp.update_sensor_freshness(false);
    }
    TEST_ASSERT_TRUE(sens_hp.get_status() == sensors::sensor_status::FAILED);
    for(int i = 0; i < test_small_recover_threshold + test_full_recover_threshold + 2; i++) {
        sens_hp.update_sensor_freshness(true);
    }
    TEST_ASSERT_TRUE(sens_hp.get_status() == sensors::sensor_status::NOMINAL);
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_sensorhealth_init);
    RUN_TEST(test_sensorhealth_failinit);
    RUN_TEST(test_sensorhealth_degrade);
    RUN_TEST(test_sensorhealth_10fails);
    RUN_TEST(test_sensorhealth_20fails);
    RUN_TEST(test_sensorhealth_10fails_then_recover);
    RUN_TEST(test_sensorhealth_20fails_then_recover);
    return UNITY_END();
}