#ifndef GOOSE_SENSORS_GPS_H_
#define GOOSE_SENSORS_GPS_H_

// HARDWARE ONLY (GNC_HARDWARE_BUILD). See systems/README.MD.
#ifndef GNC_HARDWARE_BUILD
#error "Gps.h is hardware-only. Include sensor_suite.h instead."
#endif

#include <cstdint>

#include "ubx.h"
#include <Arduino.h>

#include "SensorHealth.h"
#include "SensorInterface.h"
#include "SensorStatus.h"
#include "sensor_packets.h"
#include "logging_interface.h"

namespace sensors
{


    class Gps : public logging
    {
    public:
        Gps();

        bool begin();

        sensor_status getStatus();

        bool isFresh();

        GpsData get();

        bool loggable(uint8_t system_code, uint8_t subsystem_code, const void *structPtr) const override;

        SensorHealth &health() { return m_health; }

    private:
        GpsData m_stats;

        bfs::Ubx sensor;

        SensorHealth m_health;
    };
}
#endif // GOOSE_SENSORS_GPS_H_
