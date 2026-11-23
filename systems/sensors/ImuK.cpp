
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
    ImuK::ImuK() : m_health(imu_small_fail_threshold,
            imu_full_fail_threshold,
            imu_small_recover_threshold,
            imu_full_recover_threshold)
    {
        // TODO
    }

    bool ImuK::begin()
    {
        bool ok = m_sensor.begin();
        m_health.begin(ok);

        return ok;
    }

    sensor_status ImuK::getStatus()
    {
        return m_health.get_status();
    }

    bool ImuK::isFresh()
    {
        return (m_health.get_status() == sensor_status::FAILED);
    }

    ImuData ImuK::get()
    {
        bool fail = false;
        if (m_sensor.gyroAvailable())
        {
            m_sensor.readGyro();
        }
        else
        {
            fail = true;
        }
        if (m_sensor.accelAvailable())
        {
            m_sensor.readAccel();
        }
        else
        {
            fail = true;
        }
        if (m_sensor.magAvailable())
        {
            m_sensor.readMag();
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
            m_stats.gyroX = m_sensor.calcGyro(m_sensor.gx);
            m_stats.gyroY = m_sensor.calcGyro(m_sensor.gy);
            m_stats.gyroZ = m_sensor.calcGyro(m_sensor.gz);
            //(Converting from gravitaional accel to m/s^2) 👍
            m_stats.accelX = m_sensor.calcAccel(m_sensor.ax) * (9.80665f);
            m_stats.accelY = m_sensor.calcAccel(m_sensor.ay) * (9.80665f);
            m_stats.accelZ = m_sensor.calcAccel(m_sensor.az) * (9.80665f);
            m_stats.magX = m_sensor.calcMag(m_sensor.mx);
            m_stats.magY = m_sensor.calcMag(m_sensor.my);
            m_stats.magZ = m_sensor.calcMag(m_sensor.mz);
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
