// HARDWARE ONLY: compiled to nothing unless GNC_HARDWARE_BUILD is defined
// (env:teensy41), so the native build can include all of systems/.
#ifdef GNC_HARDWARE_BUILD

#include <cstdint>

#include "Gps.h"

#include "ubx.h"
#include <Arduino.h>

#include "SensorHealth.h"
#include "SensorInterface.h"
#include "SensorStatus.h"

// Must match the baud the receiver was configured for (u-center).
#ifndef GOOSE_GPS_BAUD
#define GOOSE_GPS_BAUD 921600
#endif

namespace sensors
{
    Gps::Gps() : sensor(&Serial1)
    {
    }

    bool Gps::begin()
    {
        bool ok = sensor.Begin(GOOSE_GPS_BAUD);
        m_health.begin(ok);
        return ok;
    }

    sensor_status Gps::getStatus()
    {
        return m_health.get_status();
    }

    bool Gps::isFresh()
    {
        return m_health.fresh();
    }

    GpsData Gps::get()
    {
        bool got_new = sensor.Read(); // true when a new fix arrived
        m_health.update_sensor_freshness(got_new);
        if (got_new)
        {
            m_stats.latitude = sensor.lat_deg();
            m_stats.longitude = sensor.lon_deg();
            m_stats.altitude = sensor.alt_wgs84_m();
            m_stats.gr_speed = sensor.gnd_spd_mps();
            m_stats.n_vel = sensor.north_vel_mps();
            m_stats.e_vel = sensor.east_vel_mps();
            m_stats.d_vel = sensor.down_vel_mps();
            m_stats.utc_day = sensor.utc_day();
            m_stats.utc_hour = sensor.utc_hour();
            m_stats.utc_min = sensor.utc_min();
            m_stats.utc_sec = sensor.utc_sec();
            m_stats.utc_nanosec = sensor.utc_nano();
        }
        return m_stats;
    }

    bool Gps::loggable(uint8_t system_code, uint8_t subsystem_code, const void *structPtr) const
    {
        return log_gps(system_code, subsystem_code,
                      *static_cast<const GpsData *>(structPtr));
    }
};

#endif // GNC_HARDWARE_BUILD
