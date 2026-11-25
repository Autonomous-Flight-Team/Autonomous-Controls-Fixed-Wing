
#include <cstdint>

#include "Gps.h"

#include "ubx.h"
#include <Arduino.h>

#include "SensorHealth.h"
#include "SensorInterface.h"
#include "SensorStatus.h"

#define baud 921600

namespace sensors
{
    Gps::Gps()  : m_health(gps_small_fail_threshold,
            gps_full_fail_threshold,
            gps_small_recover_threshold,
            gps_full_recover_threshold)
    {
        m_sensor = bfs::Ubx(&Serial1);
    }

    bool Gps::begin()
    {
        bool ok = m_sensor.Begin(baud);
        m_health.begin(ok);
        return ok;
    }

    sensor_status Gps::getStatus()
    {
        return m_health.get_status();
    }

    bool Gps::isFresh()
    {
        return (m_health.get_status() != sensor_status::FAILED);
    }

    GpsData Gps::get()
    {
        bool success;
        success = m_sensor.Read();
        m_health.update_sensor_freshness(success);
        if (success)
        {
            m_stats.latitude = m_sensor.lat_deg();
            m_stats.longitude = m_sensor.lon_deg();
            m_stats.altitude = m_sensor.alt_wgs84_m();
            m_stats.gr_speed = m_sensor.gnd_spd_mps();
            m_stats.n_vel = m_sensor.north_vel_mps();
            m_stats.e_vel = m_sensor.east_vel_mps();
            m_stats.d_vel = m_sensor.down_vel_mps();
            m_stats.utc_day = m_sensor.utc_day();
            m_stats.utc_hour = m_sensor.utc_hour();
            m_stats.utc_min = m_sensor.utc_min();
            m_stats.utc_sec = m_sensor.utc_sec();
            m_stats.utc_nanosec = m_sensor.utc_nano();
        }
        return m_stats;
    }

    bool Gps::loggable(uint8_t system_code, uint8_t subsystem_code, const void *structPtr) const
    {
        const GpsData *gps = static_cast<const GpsData *>(structPtr);
        uint8_t g[44];

        put_f64_be(&g[0], gps->latitude);
        put_f64_be(&g[8], gps->longitude);
        put_f32_be(&g[16], gps->altitude);
        put_f32_be(&g[20], gps->gr_speed);
        put_f32_be(&g[24], gps->n_vel);
        put_f32_be(&g[28], gps->e_vel);
        put_f32_be(&g[32], gps->d_vel);
        g[36] = (uint8_t)gps->utc_day;
        g[37] = (uint8_t)gps->utc_hour;
        g[38] = (uint8_t)gps->utc_min;
        g[39] = (uint8_t)gps->utc_sec;
        put_be32(&g[40], (uint32_t)gps->utc_nanosec);

        return log_message(system_code, subsystem_code, LOG_INFO, g, sizeof(g));
    }
};
