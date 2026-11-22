#ifndef GOOSE_SENSORS_SENSOR_STATUS_H_
#define GOOSE_SENSORS_SENSOR_STATUS_H_

namespace sensors {
    enum class sensor_status {
        UNINITIALIZED,
        NOMINAL,
        DEGRADED,
        FAILED
    };
}

#endif // GOOSE_SENSORS_SENSOR_STATUS_H_
