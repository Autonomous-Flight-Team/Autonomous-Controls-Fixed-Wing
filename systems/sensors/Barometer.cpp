
#include <cstdint>

#include "Barometer.h"

#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include "SensorHealth.h"
#include "SensorInterface.h"
#include "SensorStatus.h"

namespace sensors
{
    Barometer::Barometer() : m_health(baro_small_fail_threshold,
            baro_full_fail_threshold,
            baro_small_recover_threshold,
            baro_full_recover_threshold)
    {
        
    }

    bool Barometer::begin()
    {
        Wire.begin();
        bool ok = !(m_sensor.begin_I2C(0x77));
        m_health.begin(ok);
        if(ok) {
            m_sensor.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
            m_sensor.setPressureOversampling(BMP3_OVERSAMPLING_8X);
            m_sensor.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
        }
        return ok;
    }

    sensor_status Barometer::getStatus()
    {
        return m_health.get_status();
    }

    bool Barometer::isFresh()
    {
        return (m_health.get_status() != sensor_status::FAILED);
    }

    BaroData Barometer::get()
    {
        if (m_sensor.performReading())
        {
            m_stats.temp_C = m_sensor.temperature;
            m_stats.pressure_Pa = m_sensor.pressure;
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
