
#include <cstdint>

#include "Barometer.h"

#include "driver_bmp390_shot.h"
#include "SensorHealth.h"
#include "SensorInterface.h"
#include "SensorStatus.h"

namespace sensors
{
    Barometer::Barometer()
    {
    }

    bool Barometer::begin()
    {
        bool ok = (sensor_init(BMP390_INTERFACE_IIC, BMP390_ADDRESS_ADO_LOW, &m_sensor));
        m_health.begin(ok);
        return ok;
    }

    sensor_status Barometer::getStatus()
    {
        return m_health.get_status();
    }

    bool Barometer::isFresh()
    {
        return false; // PLACEHOLDER
    }

    BaroData Barometer::get()
    {
        if (bmp390_basic_read(&(m_stats.temp_C), &(m_stats.pressure_Pa), &m_sensor) == 1)
        {
            m_health.update_sensor_freshness(false);
        }
        else
        {
            m_health.update_sensor_freshness(true);
        }
        return m_stats;
    }

    bool Barometer::loggable(uint8_t system_code, uint8_t subsystem_code, const void *structPtr) const
    {
        const BaroData *baro = static_cast<const BaroData *>(structPtr);
        uint8_t b[8];
        put_f32_be(&b[0], baro->temp_C);
        put_f32_be(&b[4], baro->pressure_Pa);
        return log_message(system_code, subsystem_code, LOG_INFO, b, sizeof(b));
    }

}
