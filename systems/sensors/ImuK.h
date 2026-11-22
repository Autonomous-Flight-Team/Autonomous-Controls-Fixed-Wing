#ifndef GOOSE_SENSORS_IMU_K_H_
#define GOOSE_SENSORS_IMU_K_H_

// HARDWARE ONLY (GNC_HARDWARE_BUILD). See systems/README.MD.
#ifndef GNC_HARDWARE_BUILD
#error "ImuK.h is hardware-only. Include sensor_suite.h instead."
#endif

#include <cstdint>

#include <SparkFunLSM9DS1.h>
#include <Arduino.h>
#include <Wire.h>

#include "SensorHealth.h"
#include "SensorInterface.h"
#include "SensorStatus.h"
#include "sensor_packets.h"
#include "logging_interface.h"

namespace sensors
{

    class ImuK : public logging
    {
    public:
        ImuK();

        bool begin();

        sensor_status getStatus();

        bool isFresh();

        ImuData get();

        bool loggable(uint8_t system_code, uint8_t subsystem_code, const void *structPtr) const override;

        SensorHealth &health() { return m_health; }

    private:
        ImuData m_stats;

        SensorHealth m_health;

        LSM9DS1 sensor;
    };
}
#endif // GOOSE_SENSORS_IMU_K_H_
