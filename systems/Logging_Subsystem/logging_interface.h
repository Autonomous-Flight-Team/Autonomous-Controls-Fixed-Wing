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

// Example implementation
// class ImuLogger : public logging
// {
// public:
//     bool loggable(uint8_t system_code, uint8_t subsystem_code, const void *structPtr) override
//     {
//         // cast void* to the real type before use
//         const imu_raw_t *imu = static_cast<const imu_raw_t *>(structPtr);
//         return log_message(system_code, subsystem_code, LOG_INFO, imu, sizeof(*imu));
//     }
// };