#include <Arduino.h>
#include "../systems/Logging_Subsystem/logging_subsystem.h"
#include "../systems/Logging_Subsystem/logging_interface.h"

static ImuLogger imu_logger;
static elapsedMicros imu_timer, baro_timer, report_timer;

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000)
    {
    }
    if (!initialize_sd())
        while (true)
        {
        } // halt; handle however you prefer
}

void loop()
{
    if (imu_timer >= 1000000UL / CALL_IMU)
    {
        imu_timer -= 1000000UL / CALL_IMU;
        imu_raw_t imu = {1, 2, 3}; // replace with real sensor read
        imu_logger.loggable(SENSOR, SUB_IMU, &imu);
    }

    if (baro_timer >= 1000000UL / CALL_BARO)
    {
        baro_timer -= 1000000UL / CALL_BARO;
        baro_raw_t baro = {1013};
        log_message(SENSOR, SUB_BARO, LOG_INFO, &baro, sizeof(baro));
    }

    write_to_sd();

    if (report_timer >= 5000000UL)
    {
        report_timer = 0;
        Serial.printf("dropped packets: %lu\n", (unsigned long)logging_dropped());
    }
}