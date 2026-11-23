#include "SensorHealth.h"

namespace sensors
{

    namespace
    {
        void increment(std::uint8_t &count)
        {
            if (count < UINT8_MAX)
            {
                count++;
            }
        }
    }
    SensorHealth::SensorHealth(std::uint8_t fail_threshold_,  
            std::uint8_t full_fail_threshold_, 
            std::uint8_t success_threshold_from_degraded_,
            std::uint8_t success_threshold_from_failed_) {
            
        m_cfg.fail_threshold = fail_threshold_;
        m_cfg.full_fail_threshold = full_fail_threshold_;
        m_cfg.success_threshold_from_degraded = success_threshold_from_degraded_;
        m_cfg.success_threshold_from_failed = success_threshold_from_failed_;
    }

    // 😀
    bool SensorHealth::begin(bool start_succeeded)
    {
        if (start_succeeded)
        {
            m_status = sensor_status::NOMINAL;
        }
        else
        {
            m_status = sensor_status::FAILED;
        }
        num_successes = 0;
        num_failures = 0;
        last_sample = 0;
        return start_succeeded;
    }

    void SensorHealth::update_sensor_freshness(bool read_succeded)
    {
        if (read_succeded)
        {
            increment(num_successes);
            num_failures = 0;
            switch (m_status)
            {
                case sensor_status::FAILED:
                    if (num_successes > m_cfg.success_threshold_from_failed)
                    {
                        // A failed sensor is now succeeding
                        num_successes = 0;
                        m_status = sensor_status::DEGRADED;
                    }
                case sensor_status::DEGRADED:
                    if (num_successes > m_cfg.success_threshold_from_degraded)
                    {
                        // A degraded sensor is now failing
                        num_successes = 0;
                        m_status = sensor_status::NOMINAL;
                    }
                default:
                    break;
            }
        }
        else
        {
            increment(num_failures);
            num_successes = 0;
            switch (m_status)
            {
            case sensor_status::NOMINAL:
                if (num_failures > m_cfg.fail_threshold)
                {
                    // A succesful sensor is now failing
                    num_failures = 0;
                    m_status = sensor_status::DEGRADED;
                }
            case sensor_status::DEGRADED:
                if (num_failures > m_cfg.full_fail_threshold)
                {
                    // A degraded sensor is still failing
                    num_failures = 0;
                    m_status = sensor_status::FAILED;
                }
            default:
                break;
            }
        }
    }

    void SensorHealth::degrade()
    {
        num_failures = 0;
        switch (m_status)
        {
        case sensor_status::NOMINAL:
            m_status = sensor_status::DEGRADED;
            break;
        case sensor_status::DEGRADED:
            m_status = sensor_status::FAILED;
            break;
        default:
            break;
        }
    }

    sensor_status SensorHealth::get_status()
    {
        return m_status;
    }

    bool SensorHealth::loggable(uint8_t system_code, uint8_t subsystem_code, const void *structPtr) const
    {
        const sensor_trust_limits *t = static_cast<const sensor_trust_limits *>(structPtr);
        uint8_t b[4];

        b[0] = t->fail_threshold;
        b[1] = t->full_fail_threshold;
        b[2] = t->success_threshold_from_degraded;
        b[3] = t->success_threshold_from_failed;

        return log_message(system_code, subsystem_code, LOG_INFO, b, sizeof(b));
    }
}
