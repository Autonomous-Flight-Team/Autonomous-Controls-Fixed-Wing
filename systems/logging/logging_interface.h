#ifndef GOOSE_LOGGING_LOGGING_INTERFACE_H_
#define GOOSE_LOGGING_LOGGING_INTERFACE_H_
#include <stdint.h>
#include <string.h>
#include <stdint.h>
#include "logging_subsystem.h"

class logging
{
public:
    virtual bool loggable(uint8_t system_code, uint8_t subsystem_code, const void *structPtr) const = 0;
    virtual ~logging() = default;
};

#endif // GOOSE_LOGGING_LOGGING_INTERFACE_H_
