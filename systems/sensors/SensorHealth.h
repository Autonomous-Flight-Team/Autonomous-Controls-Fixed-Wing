#pragma once

#include <cstdint>
#include "SensorStatus.h"
#include "logging_interface.h"

namespace sensors
{
    struct sensor_trust_limits
    {
        std::uint8_t fail_threshold;                  // Failure count before we mark a sensor as degraded
        std::uint8_t full_fail_threshold;             // Failure count before we mark a degraded sensor as failed
        std::uint8_t success_threshold_from_degraded; // Success count before we mark a degraded sensor as nominal
        std::uint8_t success_threshold_from_failed;   // Success count before we mark a degraded sensor as nominal
    };

    class SensorHealth : public logging
    {
    public:
        // Initializes and starts the sensor
        // timer - The pointer to the timer
        // return - Whether the begin succeeded
        bool begin(bool start_succeeded);
        // Updates the sensor freshness, should be called after every read
        // read_succeed - Whether the read of the sensor succeeded
        // timer - The pointer to the timer
        void update_sensor_freshness(bool read_succeeded);
        // Updates the sensor status as degraded
        void degrade();
        // Gets the sensor status
        // return - The status of the sensor
        sensor_status get_status();
        // Updates the sensor for a failed read
        void on_success();
        // Updates the sensor for a successful read
        void on_failure();

        bool loggable(uint8_t system_code, uint8_t subsystem_code, const void *structPtr) const override;

    private:
        sensor_trust_limits m_cfg;
        sensor_status m_status{sensor_status::UNINITIALIZED};
        std::uint8_t num_successes{0};
        std::uint8_t num_failures{0};
        std::uint32_t last_sample{0};
        bool is_fresh;
    };
}
