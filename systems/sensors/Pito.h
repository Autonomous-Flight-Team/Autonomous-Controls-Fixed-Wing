#pragma once

#include <cstdint>

#include "ms4525do.h"
#include "SensorHealth.h"
#include "SensorInterface.h"
#include "SensorStatus.h"

namespace sensors {

    struct PitoData {
        float pressure_Pa; //Pressure in pascals 🧯
        float die_temp_C; //Temperature in celsius 🌡️
    };

    class Pito {
     public:
        Pito();

        bool begin();

        sensor_status getStatus();

        bool isFresh();

        PitoData get();

     private:
        bfs::Ms4525do sensor;

        PitoData m_stats;

        SensorHealth m_health;
    };
}
