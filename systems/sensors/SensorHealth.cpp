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

    SensorHealth::SensorHealth(const sensor_trust_limits &limits) : m_cfg(limits)
    {
    }

    // 😀
    bool SensorHealth::begin(bool start_succeeded)
    {
        m_status = start_succeeded ? sensor_status::NOMINAL : sensor_status::FAILED;
        num_successes = 0;
        num_failures = 0;
        is_fresh = false;
        return start_succeeded;
    }

    void SensorHealth::update_sensor_freshness(bool read_succeeded)
    {
        is_fresh = read_succeeded;
        if (m_status == sensor_status::UNINITIALIZED)
        {
            return; // begin() not called yet
        }
        if (read_succeeded)
        {
            increment(num_successes);
            num_failures = 0;
            if (m_status == sensor_status::FAILED &&
                num_successes >= m_cfg.success_threshold_from_failed)
            {
                // A failed sensor is succeeding again
                num_successes = 0;
                m_status = sensor_status::DEGRADED;
            }
            else if (m_status == sensor_status::DEGRADED &&
                     num_successes >= m_cfg.success_threshold_from_degraded)
            {
                // A degraded sensor has recovered
                num_successes = 0;
                m_status = sensor_status::NOMINAL;
            }
            return;
        }

        increment(num_failures);
        num_successes = 0;
        if (m_status == sensor_status::NOMINAL &&
            num_failures >= m_cfg.fail_threshold)
        {
            // A good sensor is now failing
            num_failures = 0;
            m_status = sensor_status::DEGRADED;
        }
        else if (m_status == sensor_status::DEGRADED &&
                 num_failures >= m_cfg.full_fail_threshold)
        {
            // A degraded sensor is still failing
            num_failures = 0;
            m_status = sensor_status::FAILED;
        }
    }

    void SensorHealth::on_success()
    {
        update_sensor_freshness(true);
    }

    void SensorHealth::on_failure()
    {
        update_sensor_freshness(false);
    }

    void SensorHealth::degrade()
    {
        num_failures = 0;
        num_successes = 0;
        m_status = sensor_status::DEGRADED;
    }

    sensor_status SensorHealth::get_status() const
    {
        return m_status;
    }

    bool SensorHealth::fresh() const
    {
        return is_fresh;
    }

    void SensorHealth::set_limits(const sensor_trust_limits &limits)
    {
        m_cfg = limits;
    }

    const sensor_trust_limits &SensorHealth::limits() const
    {
        return m_cfg;
    }

    bool SensorHealth::loggable(uint8_t system_code, uint8_t subsystem_code, const void *structPtr) const
    {
        return log_trust(system_code, subsystem_code,
                         *static_cast<const sensor_trust_limits *>(structPtr));
    }
}
