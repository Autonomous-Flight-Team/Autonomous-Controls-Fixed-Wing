#pragma once

namespace sensors {
    enum class sensor_status {
        UNINITIALIZED,
        NOMINAL,
        DEGRADED,
        FAILED
    };
}