#pragma once
#include <stdint.h>
#include <string.h>
#include <Arduino.h>
#include "logging_subsystem.h"

class logging
{
public:
    virtual bool loggable(uint8_t system_code, uint8_t subsystem_code, const void *structPtr) const = 0;
    virtual ~logging() = default;
};
