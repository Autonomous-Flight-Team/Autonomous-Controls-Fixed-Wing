#ifndef GOOSE_SENSORS_PITO_H_
#define GOOSE_SENSORS_PITO_H_

// HARDWARE ONLY (GNC_HARDWARE_BUILD). See systems/README.MD.
#ifndef GNC_HARDWARE_BUILD
#error "Pito.h is hardware-only. Include sensor_suite.h instead."
#endif

#include <cstdint>

#include "ms4525do.h"
#include "SensorHealth.h"
#include "SensorInterface.h"
#include "SensorStatus.h"
#include "sensor_packets.h"
#include "logging_interface.h"

namespace sensors
{


    class Pito : public logging
    {
    public:
        Pito();

        bool begin();

        sensor_status getStatus();

        bool isFresh();

        PitoData get();

        bool loggable(uint8_t system_code, uint8_t subsystem_code, const void *structPtr) const override;

        SensorHealth &health() { return m_health; }

    private:
        bfs::Ms4525do sensor;

        PitoData m_stats;

        SensorHealth m_health;
    };
}
#endif // GOOSE_SENSORS_PITO_H_
