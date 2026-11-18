
#include <cstdint>

#include "Pito.h"

#include "ms4525do.h"
#include "SensorHealth.h"
#include "SensorInterface.h"
#include "SensorStatus.h"

namespace sensors {
        Pito::Pito() {
            //TODO
        }

        bool Pito::begin() {
            Wire.begin();
            Wire.setClock(400000);
            sensor.Config(&Wire, 0x28, 1.0f, -1.0f);
            bool to_return = sensor.Begin();
            m_health.begin(to_return);
            return to_return;
        }

        sensor_status Pito::getStatus() {
            return m_health.get_status();
        }

        bool Pito::isFresh() {
            return false; //PLACEHOLDER
        }

        PitoData Pito::get() {
            if (sensor.Read()) {
                m_stats.pressure_Pa = sensor.pres_pa();
                m_stats.die_temp_C = sensor.die_temp_c();
            }
            return m_stats;
        }
}
