#ifndef GOOSE_SENSORS_SENSOR_MANAGER_H_
#define GOOSE_SENSORS_SENSOR_MANAGER_H_
// Owns the four sensors, polls each at its own rate and logs every fresh
// sample. Templated on the sensor types so the same code runs with the real
// drivers on the Teensy and with DummySensors in the native unit tests.
#include <stdint.h>

#include "SensorStatus.h"
#include "logging_subsystem.h"
#include "rate_gate.h"
#include "sensor_packets.h"

// Poll rates in Hz. CALL_IMU and CALL_BARO live in logging_subsystem.h.
#ifndef CALL_GPS
#define CALL_GPS 10
#endif
#ifndef CALL_PITOT
#define CALL_PITOT 50
#endif

namespace sensors {

struct SensorCounts {
    uint32_t reads{0};  // read attempts
    uint32_t logged{0}; // fresh samples handed to the logger
    uint32_t stale{0};  // reads that produced no new data
};

template <class Imu, class Baro, class Gps, class Pitot> class SensorManager {
public:
    Imu imu;
    Baro baro;
    Gps gps;
    Pitot pitot;
    SensorCounts imu_counts, baro_counts, gps_counts, pitot_counts;

    // Starts every sensor (a failed sensor is marked FAILED, not fatal) and
    // records the trust limits in the log. Returns true if all started.
    bool begin(bool log_enabled) {
        bool ok = imu.begin();
        ok = baro.begin() && ok;
        ok = gps.begin() && ok;
        ok = pitot.begin() && ok;
        if (log_enabled) {
            sensor_trust_limits t = imu.health().limits();
            imu.health().loggable(SENSOR, SUB_SENSOR_TRUST, &t);
        }
        return ok;
    }

    // Call every loop iteration with micros().
    void update(uint32_t now_us, bool log_enabled) {
        if (m_imu_tick.due(now_us)) {
            poll(imu, SUB_IMU, imu_counts, log_enabled);
        }
        if (m_baro_tick.due(now_us)) {
            poll(baro, SUB_BARO, baro_counts, log_enabled);
        }
        if (m_gps_tick.due(now_us)) {
            poll(gps, SUB_GPS, gps_counts, log_enabled);
        }
        if (m_pitot_tick.due(now_us)) {
            poll(pitot, SUB_PITOT, pitot_counts, log_enabled);
        }
    }

    // True when no sensor is FAILED or UNINITIALIZED.
    bool all_usable() {
        return usable(imu.getStatus()) && usable(baro.getStatus()) &&
               usable(gps.getStatus()) && usable(pitot.getStatus());
    }

private:
    template <class S>
    static void poll(S &s, uint8_t sub, SensorCounts &c, bool log_enabled) {
        auto data = s.get();
        c.reads++;
        if (!s.isFresh()) {
            c.stale++;
            return;
        }
        if (log_enabled && s.loggable(SENSOR, sub, &data)) {
            c.logged++;
        }
    }

    static bool usable(sensor_status s) {
        return s == sensor_status::NOMINAL || s == sensor_status::DEGRADED;
    }

    RateGate m_imu_tick{CALL_IMU};
    RateGate m_baro_tick{CALL_BARO};
    RateGate m_gps_tick{CALL_GPS};
    RateGate m_pitot_tick{CALL_PITOT};
};

} // namespace sensors

#endif // GOOSE_SENSORS_SENSOR_MANAGER_H_
