#pragma once

#include <cstdint>

#include "ubx.h"
#include <Arduino.h>

#include "SensorHealth.h"
#include "SensorInterface.h"
#include "SensorStatus.h"
#include "logging_interface.h"

namespace sensors
{

    struct GpsData
    {
        double latitude;     // Latitude, in degrees 🌎
        double longitude;    // Longitude, in degrees 🌎
        float altitude;      // Altitude above sea level, meters ⛰️
        float gr_speed;      // Ground speed, m/s 🚀
        float n_vel;         // North Velocity, m/s 🚀
        float e_vel;         // South Velocity, m/s 🚀
        float d_vel;         // Down Velocity, m/s 🚀
        int8_t utc_day;      // Day at UTC time 📅
        int8_t utc_hour;     // Hour at UTC time ⌛
        int8_t utc_min;      // Minutes at UTC time ⏱️
        int8_t utc_sec;      // Seconds at UTC time ⏱️
        int32_t utc_nanosec; // Nanoseconds at UTC time ⏱️
    };

    class Gps : public logging
    {
    public:
        Gps();

        bool begin();

        sensor_status getStatus();

        bool isFresh();

        GpsData get();

        bool loggable(uint8_t system_code, uint8_t subsystem_code, const void *structPtr) const override;

    private:
        GpsData m_stats;

        bfs::Ubx sensor;

        SensorHealth m_health;
    };
}
