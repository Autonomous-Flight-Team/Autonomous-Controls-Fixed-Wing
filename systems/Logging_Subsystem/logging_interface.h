#pragma once
#include <stdint.h>
#include <string.h>
#include <Arduino.h>
#include "logging_subsystem.h"

class logging
{
public:
    virtual bool loggable(uint8_t system_code, uint8_t subsystem_code, const void *structPtr) = 0;
    virtual ~logging() = default;
};

// Example implementations: serialize each field big-endian
class ImuLogger : public logging
{
public:
    bool loggable(uint8_t system_code, uint8_t subsystem_code, const void *structPtr) override
    {
        const imu_raw_t *imu = static_cast<const imu_raw_t *>(structPtr);
        uint8_t b[12];
        put_be32(&b[0], (uint32_t)imu->x);
        put_be32(&b[4], (uint32_t)imu->y);
        put_be32(&b[8], (uint32_t)imu->z);
        return log_message(system_code, subsystem_code, LOG_INFO, b, sizeof(b));
    }
};

class BaroLogger : public logging
{
public:
    bool loggable(uint8_t system_code, uint8_t subsystem_code, const void *structPtr) override
    {
        const baro_raw_t *baro = static_cast<const baro_raw_t *>(structPtr);
        uint8_t b[4];
        put_be32(&b[0], (uint32_t)baro->z);
        return log_message(system_code, subsystem_code, LOG_INFO, b, sizeof(b));
    }
};