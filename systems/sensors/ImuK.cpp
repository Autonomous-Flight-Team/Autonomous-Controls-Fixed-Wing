// HARDWARE ONLY: compiled to nothing unless GNC_HARDWARE_BUILD is defined
// (env:teensy41), so the native build can include all of systems/.
#ifdef GNC_HARDWARE_BUILD

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
    static constexpr float G_MPS2 = 9.80665f;

    ImuK::ImuK()
    {
        // TODO
    }

    bool ImuK::begin()
    {
        bool ok = sensor.begin() != 0; // begin() returns 0 on failure
        m_health.begin(ok);

        return ok;
    }

    sensor_status ImuK::getStatus()
    {
        return m_health.get_status();
    }

    bool ImuK::isFresh()
    {
        return m_health.fresh();
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
            // calcAccel() is in g; ImuData stores m/s^2
            m_stats.accelX = sensor.calcAccel(sensor.ax) * G_MPS2;
            m_stats.accelY = sensor.calcAccel(sensor.ay) * G_MPS2;
            m_stats.accelZ = sensor.calcAccel(sensor.az) * G_MPS2;
            m_stats.magX = sensor.calcMag(sensor.mx);
            m_stats.magY = sensor.calcMag(sensor.my);
            m_stats.magZ = sensor.calcMag(sensor.mz);
        }
        return m_stats;
    }

    bool ImuK::loggable(uint8_t system_code, uint8_t subsystem_code, const void *structPtr) const
    {
        return log_imu(system_code, subsystem_code,
                      *static_cast<const ImuData *>(structPtr));
    }
}

#endif // GNC_HARDWARE_BUILD
