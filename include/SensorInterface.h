#pragma once

#include "SensorStatus.h"

namespace sensors {
    class sensor_interface {
        public: sensor_status get_status();
        public: bool is_fresh();
    };
}