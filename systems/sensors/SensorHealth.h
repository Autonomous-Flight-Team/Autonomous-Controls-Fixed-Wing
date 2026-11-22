#ifndef GOOSE_SENSORS_SENSOR_HEALTH_H_
#define GOOSE_SENSORS_SENSOR_HEALTH_H_

#include <cstdint>
#include "SensorStatus.h"
#include "sensor_packets.h"
#include "logging_interface.h"

namespace sensors
{
    // Default trust limits; override per sensor with set_limits().
    constexpr sensor_trust_limits DEFAULT_TRUST_LIMITS = {3, 10, 5, 20};

    // Tracks one sensor's health from its read results.
    //   NOMINAL  --fail_threshold failures in a row-->       DEGRADED
    //   DEGRADED --full_fail_threshold failures in a row-->  FAILED
    //   DEGRADED --success_threshold_from_degraded in a row--> NOMINAL
    //   FAILED   --success_threshold_from_failed in a row-->   DEGRADED
    // Hardware-free, so it is unit tested natively (test_sensor_health).
    class SensorHealth : public logging
    {
    public:
        explicit SensorHealth(
            const sensor_trust_limits &limits = DEFAULT_TRUST_LIMITS);

        // Starts tracking. NOMINAL if the sensor started, else FAILED.
        // return - start_succeeded, passed through
        bool begin(bool start_succeeded);
        // Call after every read attempt
        // read_succeeded - whether the read produced new, valid data
        void update_sensor_freshness(bool read_succeeded);
        // Same as update_sensor_freshness(true)
        void on_success();
        // Same as update_sensor_freshness(false)
        void on_failure();
        // Forces DEGRADED (e.g. another check found the data implausible)
        void degrade();
        sensor_status get_status() const;
        // Whether the most recent read produced new data
        bool fresh() const;

        void set_limits(const sensor_trust_limits &limits);
        const sensor_trust_limits &limits() const;

        bool loggable(uint8_t system_code, uint8_t subsystem_code, const void *structPtr) const override;

    private:
        sensor_trust_limits m_cfg;
        sensor_status m_status{sensor_status::UNINITIALIZED};
        std::uint8_t num_successes{0};
        std::uint8_t num_failures{0};
        bool is_fresh{false};
    };
}

#endif // GOOSE_SENSORS_SENSOR_HEALTH_H_
