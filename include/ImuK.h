#pragma once

#include <cstdint>

#include <SparkFunLSM9DS1.h>
#include <Arduino.h>
#include <Wire.h>

#include "SensorHealth.h"
#include "SensorInterface.h"
#include "SensorStatus.h"

namespace sensors {
    struct ImuData {
        float accelX; //X-axis Acceleration in m/s^2 🚀
        float accelY; //Y-axis Acceleration in m/s^2 🚀
        float accelZ; //Z-axis Acceleration in m/s^2 🚀
        float gyroX; //X-axis Angular Velocity in deg/sec 🥏
        float gyroY; //Y-axis Angular Velocity in deg/sec 🥏
        float gyroZ; //Z-axis Angular Velocity in deg/sec 🥏
        float magX; //X-axis mag in gauss 🧲
        float magY; //Y-axis mag in gauss 🧲
        float magZ; //Z-axis mag in gauss 🧲
    };

    class ImuK {
     public:
        ImuK();

        bool begin();

        sensor_status getStatus();

        bool isFresh();

        ImuData get();

     private:
        ImuData m_stats;

        SensorHealth m_health;

        LSM9DS1 sensor;
    };
}
