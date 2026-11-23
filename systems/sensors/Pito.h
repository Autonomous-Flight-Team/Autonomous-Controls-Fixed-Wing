#ifndef PITO_
#define PITO_

#include <cstdint>

#include "ms4525do.h"
#include "SensorHealth.h"
#include "SensorInterface.h"
#include "SensorStatus.h"
#include "logging_interface.h"

#define pito_small_fail_threshold 10
#define pito_full_fail_threshold 10
#define pito_small_recover_threshold 10
#define pito_full_recover_threshold 10

namespace sensors
{

    struct PitoData
    {
        float pressure_Pa; // Pressure in pascals 🧯
        float die_temp_C;  // Temperature in celsius 🌡️
    };

    class Pito : public logging
    {
    public:
        Pito();

        bool begin();

        sensor_status getStatus();

        bool isFresh();

        PitoData get();

        bool loggable(uint8_t system_code, uint8_t subsystem_code, const void *structPtr) const override;

    private:
        bfs::Ms4525do m_sensor;

        PitoData m_stats;

        SensorHealth m_health;
    };
}
#endif