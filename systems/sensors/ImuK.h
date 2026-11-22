#ifndef IMU_
#define IMU_

#include <cstdint>

#include <SparkFunLSM9DS1.h>
#include <Arduino.h>
#include <Wire.h>

#include "logging_interface.h"
#include "SensorHealth.h"
#include "SensorInterface.h"
#include "SensorStatus.h"
#include "logging_interface.h"

namespace sensors
{
    struct ImuData
    {
        float accelX; // X-axis Acceleration in m/s^2 🚀
        float accelY; // Y-axis Acceleration in m/s^2 🚀
        float accelZ; // Z-axis Acceleration in m/s^2 🚀
        float gyroX;  // X-axis Angular Velocity in deg/sec 🥏
        float gyroY;  // Y-axis Angular Velocity in deg/sec 🥏
        float gyroZ;  // Z-axis Angular Velocity in deg/sec 🥏
        float magX;   // X-axis mag in gauss 🧲
        float magY;   // Y-axis mag in gauss 🧲
        float magZ;   // Z-axis mag in gauss 🧲
    };

    class ImuK : public logging
    {
    public:
        ImuK();

        bool begin();

        sensor_status getStatus();

        bool isFresh();

        ImuData get();

        bool loggable(uint8_t system_code, uint8_t subsystem_code, const void *structPtr) const override;

    private:
        ImuData m_stats;

        SensorHealth m_health;

        LSM9DS1 sensor;
    };
}
#endif //IMU