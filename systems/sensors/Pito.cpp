
#include <cstdint>

#include "Pito.h"

#include "ms4525do.h"
#include "SensorHealth.h"
#include "SensorStatus.h"

namespace sensors
{
    Pito::Pito()
    {
        // TODO
    }

    bool Pito::begin()
    {
        Wire.begin();
        Wire.setClock(400000);
        sensor.Config(&Wire, 0x28, 1.0f, -1.0f);
        bool to_return = sensor.Begin();
        m_health.begin(to_return);
        return to_return;
    }

    sensor_status Pito::getStatus()
    {
        return m_health.get_status();
    }

    bool Pito::isFresh()
    {
        return false; // PLACEHOLDER
    }

    PitoData Pito::get()
    {
        if (sensor.Read())
        {
            m_stats.pressure_Pa = sensor.pres_pa();
            m_stats.die_temp_C = sensor.die_temp_c();
        }
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
