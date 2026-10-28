#pragma once

#include <cstdint>
#include "SensorStatus.h"

namespace sensors {
    struct sensor_trust_limits {
        std::uint8_t fail_threshold; //Failure count before we mark a sensor as degraded
        std::uint8_t full_fail_threshold; //Failure count before we mark a degraded sensor as failed
        std::uint8_t success_threshold_from_degraded; //Success count before we mark a degraded sensor as nominal
        std::uint8_t success_threshold_from_failed; //Success count before we mark a degraded sensor as nominal
    };
    
    class SensorHealth {
        //Initializes and starts the sensor
        //timer - The pointer to the timer
        //return - Whether the begin succeeded
        public: bool begin(bool start_succeeded);
        //Updates the sensor freshness, should be called after every read
        //read_succeed - Whether the read of the sensor succeeded
        //timer - The pointer to the timer
        public: void update_sensor_freshness(bool read_succeeded);
        //Updates the sensor status as degraded
        public: void degrade();
        //Gets the sensor status
        //return - The status of the sensor
        public: sensor_status get_status();
        //Updates the sensor for a failed read
        private: void on_success();
        //Updates the sensor for a successful read
        private: void on_failure();
        
        private: sensor_trust_limits m_cfg;
        private: sensor_status m_status{sensor_status::UNINITIALIZED};
        private: std::uint8_t num_successes{0};
        private: std::uint8_t num_failures{0};
        private: std::uint32_t last_sample{0};
        private: bool is_fresh;
    };
}