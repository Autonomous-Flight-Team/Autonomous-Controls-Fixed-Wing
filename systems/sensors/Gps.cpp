
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
    Gps::Gps()
    {
        bfs::Ubx sensor(&Serial1);
    }

    bool Gps::begin()
    {
        bool ok = sensor.Begin(baud);
    }

    sensor_status Gps::getStatus()
    {
        return m_health.get_status();
    }

    bool Gps::isFresh()
    {
        return false; // PLACEHOLDER
    }

    GpsData Gps::get()
    {
        bool fail;
        fail = sensor.Read();
        m_health.update_sensor_freshness(fail);
        if (!fail)
        {
            m_stats.latitude = sensor.lat_deg();
            m_stats.longitude = sensor.lon_deg();
            m_stats.altitude = sensor.alt_wgs84_m();
            m_stats.gr_speed = sensor.north_vel_mps();
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

    bool Gps::loggable(uint8_t system_code, uint8_t subsystem_code, const void *structPtr)
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
