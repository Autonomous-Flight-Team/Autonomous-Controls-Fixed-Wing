#ifndef GOOSE_SENSORS_SENSOR_INTERFACE_H_
#define GOOSE_SENSORS_SENSOR_INTERFACE_H_

#include "SensorStatus.h"

namespace sensors {
    class sensor_interface {
        public: sensor_status get_status();
        public: bool is_fresh();
    };
}

#endif // GOOSE_SENSORS_SENSOR_INTERFACE_H_
