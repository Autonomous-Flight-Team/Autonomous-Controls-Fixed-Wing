
#include <cstdint>

#include "ImuK.h"

#include <SparkFunLSM9DS1.h>
#include <Arduino.h>
#include <Wire.h>

#include "SensorHealth.h"
#include "SensorInterface.h"
#include "SensorStatus.h"

namespace sensors
{
    ImuK::ImuK()
    {
        // TODO
    }

    bool ImuK::begin()
    {
        bool ok = sensor.begin();
        m_health.begin(ok);

        return;
    }

    sensor_status ImuK::getStatus()
    {
        return m_health.get_status();
    }

    bool ImuK::isFresh()
    {
        return false; // PLACEHOLDER
    }

    ImuData ImuK::get()
    {
        bool fail = false;
        if (sensor.gyroAvailable())
        {
            sensor.readGyro();
        }
        else
        {
            fail = true;
        }
        if (sensor.accelAvailable())
        {
            sensor.readAccel();
        }
        else
        {
            fail = true;
        }
        if (sensor.magAvailable())
        {
            sensor.readMag();
        }
        else
        {
            fail = true;
        }
        if (fail)
        {
            m_health.update_sensor_freshness(false);
        }
        else
        {
            m_health.update_sensor_freshness(true);
            m_stats.gyroX = sensor.calcGyro(sensor.gx);
            m_stats.gyroY = sensor.calcGyro(sensor.gy);
            m_stats.gyroZ = sensor.calcGyro(sensor.gz);
            m_stats.accelX = sensor.calcGyro(sensor.ax);
            m_stats.accelY = sensor.calcGyro(sensor.ay);
            m_stats.accelZ = sensor.calcGyro(sensor.az);
            m_stats.magX = sensor.calcGyro(sensor.mx);
            m_stats.magY = sensor.calcGyro(sensor.my);
            m_stats.magZ = sensor.calcGyro(sensor.mz);
        }
        return m_stats;
    }

    bool ImuK::loggable(uint8_t system_code, uint8_t subsystem_code, const void *structPtr) const
    {
        const ImuData *imu = static_cast<const ImuData *>(structPtr);
        uint8_t b[36];

        put_f32_be(&b[0], imu->accelX);
        put_f32_be(&b[4], imu->accelY);
        put_f32_be(&b[8], imu->accelZ);
        put_f32_be(&b[12], imu->gyroX);
        put_f32_be(&b[16], imu->gyroY);
        put_f32_be(&b[20], imu->gyroZ);
        put_f32_be(&b[24], imu->magX);
        put_f32_be(&b[28], imu->magY);
        put_f32_be(&b[32], imu->magZ);

        return log_message(system_code, subsystem_code, LOG_INFO, b, sizeof(b));
    }
}
