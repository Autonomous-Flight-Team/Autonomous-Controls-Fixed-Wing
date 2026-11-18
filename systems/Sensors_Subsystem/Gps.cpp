
#include <cstdint>

#include "Gps.h"

#include "ubx.h"
#include <Arduino.h>

#include "SensorHealth.h"
#include "SensorInterface.h"
#include "SensorStatus.h"

#define baud 921600

namespace sensors {
        Gps::Gps() {
            bfs::Ubx sensor(&Serial1);
        }
        
        bool Gps::begin() {
            bool ok = sensor.Begin(baud);
        }

        sensor_status Gps::getStatus() {
            return m_health.get_status();
        }
        
        bool Gps::isFresh() {
            return false; //PLACEHOLDER
        }
        
        GpsData Gps::get() {
            bool fail;
            fail = sensor.Read();
            m_health.update_sensor_freshness(fail);
            if(!fail) {
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
    }
        