
#include <cstdint>

#include "Pito.h"

#include "ms4525do.h"
#include "SensorHealth.h"
#include "SensorStatus.h"

namespace sensors
{
    Pito::Pito() : m_health(pito_small_fail_threshold,
            pito_full_fail_threshold,
            pito_small_recover_threshold,
            pito_full_recover_threshold)
    {
        // TODO
    }

    bool Pito::begin()
    {
        Wire.begin();
        Wire.setClock(400000);
        m_sensor.Config(&Wire, 0x28, 1.0f, -1.0f);
        bool to_return = m_sensor.Begin();
        m_health.begin(to_return);
        return to_return;
    }

    sensor_status Pito::getStatus()
    {
        return m_health.get_status();
    }

    bool Pito::isFresh()
    {
        return (m_health.get_status() != sensor_status::FAILED);
    }

    PitoData Pito::get()
    {
        bool read_succeed = m_sensor.Read();
        if (read_succeed)
        {
            m_stats.pressure_Pa = m_sensor.pres_pa();
            m_stats.die_temp_C = m_sensor.die_temp_c();
        }
        m_health.update_sensor_freshness(read_succeed);
        return m_stats;
    }

    bool Pito::loggable(uint8_t system_code, uint8_t subsystem_code, const void *structPtr) const
    {
        const PitoData *pitot = static_cast<const PitoData *>(structPtr);
        uint8_t b[8];

        put_f32_be(&b[0], pitot->pressure_Pa);
        put_f32_be(&b[4], pitot->die_temp_C);

        return log_message(system_code, subsystem_code, LOG_INFO, b, sizeof(b));
    }
}
