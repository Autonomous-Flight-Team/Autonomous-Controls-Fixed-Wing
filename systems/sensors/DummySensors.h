#ifndef GOOSE_SENSORS_DUMMY_SENSORS_H_
#define GOOSE_SENSORS_DUMMY_SENSORS_H_
// Dummy sensors: same interface as the real drivers (begin, get, getStatus,
// isFresh, loggable) but no hardware. They produce deterministic data that
// changes every sample, and tests can make them fail on demand.
// Used by the native unit tests and by firmware built with
// GOOSE_DUMMY_SENSORS (bench-testing logging with no sensors attached).
#include <stdint.h>

#include "SensorHealth.h"
#include "SensorStatus.h"
#include "logging_interface.h"
#include "sensor_packets.h"

namespace sensors {

// Shared behaviour; Data is the sensor's data struct.
template <typename Data> class DummySensor : public logging {
public:
    bool begin() { return m_health.begin(!m_fail_begin); }
    sensor_status getStatus() const { return m_health.get_status(); }
    bool isFresh() const { return m_health.fresh(); }
    SensorHealth &health() { return m_health; }

    // Test controls
    void set_fail_begin(bool fail) { m_fail_begin = fail; }
    void set_failing(bool failing) { m_failing = failing; }
    uint32_t samples() const { return m_samples; }

protected:
    // Runs one read; make() builds sample n. Failed reads keep the old data.
    template <typename Make> Data read(Make make) {
        bool ok = !m_failing && m_health.get_status() !=
                                    sensor_status::UNINITIALIZED;
        m_health.update_sensor_freshness(ok);
        if (ok) {
            m_last = make(m_samples++);
        }
        return m_last;
    }

    Data m_last{};
    SensorHealth m_health;
    bool m_fail_begin{false};
    bool m_failing{false};
    uint32_t m_samples{0};
};

class DummyImu : public DummySensor<ImuData> {
public:
    ImuData get() {
        return read([](uint32_t n) {
            float t = (float)(n % 100) * 0.01f;
            return ImuData{t, -t, 9.80665f, 0.5f * t, 0.0f, -0.5f * t,
                           0.2f, 0.0f, -0.4f};
        });
    }
    bool loggable(uint8_t sys, uint8_t sub, const void *p) const override {
        return log_imu(sys, sub, *static_cast<const ImuData *>(p));
    }
};

class DummyBaro : public DummySensor<BaroData> {
public:
    BaroData get() {
        return read([](uint32_t n) {
            return BaroData{21.5f, 101325.0f - (float)(n % 50)};
        });
    }
    bool loggable(uint8_t sys, uint8_t sub, const void *p) const override {
        return log_baro(sys, sub, *static_cast<const BaroData *>(p));
    }
};

class DummyGps : public DummySensor<GpsData> {
public:
    GpsData get() {
        return read([](uint32_t n) {
            return GpsData{47.6062 + n * 1e-6, -122.3321, 56.0f, 3.5f,
                           3.5f, 0.0f, 0.0f, 22, 13, 45,
                           (int8_t)(n % 60), 0};
        });
    }
    bool loggable(uint8_t sys, uint8_t sub, const void *p) const override {
        return log_gps(sys, sub, *static_cast<const GpsData *>(p));
    }
};

class DummyPitot : public DummySensor<PitoData> {
public:
    PitoData get() {
        return read([](uint32_t n) {
            return PitoData{(float)(n % 20) * 10.0f, 21.5f};
        });
    }
    bool loggable(uint8_t sys, uint8_t sub, const void *p) const override {
        return log_pitot(sys, sub, *static_cast<const PitoData *>(p));
    }
};

} // namespace sensors

#endif // GOOSE_SENSORS_DUMMY_SENSORS_H_
