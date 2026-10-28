#include "SensorHealth.h"

namespace sensors {

    namespace {
        void increment(std::uint8_t &count) {
            if (count < UINT8_MAX) {
                count++;
            }
        }
    }

    bool SensorHealth::begin(bool start_succeeded) {
        if(start_succeeded) {
            m_status = sensor_status::NOMINAL;
        } else {
            m_status = sensor_status::FAILED;
        }
        num_successes = 0;
        num_failures = 0;
        last_sample = 0;
        is_fresh = false;
    }

    void SensorHealth::update_sensor_freshness(bool read_succeded) {
        if(read_succeded) {
            increment(num_successes);
            num_failures = 0;
            switch (m_status) {
                case sensor_status::FAILED:
                    if(num_successes > m_cfg.success_threshold_from_failed) {
                        //A failed sensor is now succeeding
                        num_successes = 0;
                        m_status = sensor_status::DEGRADED;
                    }
                case sensor_status::DEGRADED:
                    if(num_successes > m_cfg.success_threshold_from_degraded) {
                        //A degraded sensor is now failing
                        num_successes= 0;
                        m_status = sensor_status::NOMINAL;
                    }
                default:
                    break;
            }
        } else {
            increment(num_failures);
            num_successes = 0;
            switch(m_status) {
                case sensor_status::NOMINAL:
                    if(num_successes > m_cfg.fail_threshold) {
                        //A succesful sensor is now failing
                        num_failures = 0;
                        m_status = sensor_status::DEGRADED;
                    }
                case sensor_status::DEGRADED:
                    if(num_successes > m_cfg.full_fail_threshold) {
                        //A degraded sensor is still failing
                        num_failures= 0;
                        m_status = sensor_status::NOMINAL;
                    }
                default:
                    break;
            }
        }
    }    
    
    void SensorHealth::degrade() {
        num_failures= 0;
        m_status = sensor_status::NOMINAL;
    }

    sensor_status SensorHealth::get_status() {
        return m_status;
    }
}