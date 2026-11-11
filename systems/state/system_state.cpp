#include "system_state.h"

const char* to_string(SystemState s)
{
    switch (s)
    {
    case SystemState::kBoot:
        return "BOOT";
    case SystemState::kPreflight:
        return "PREFLIGHT";
    case SystemState::kStandby:
        return "STANDBY";
    case SystemState::kFaultGround:
        return "FAULT_GROUND";
    case SystemState::kBench:
        return "BENCH";
    case SystemState::kActive:
        return "ACTIVE";
    case SystemState::kDegraded:
        return "DEGRADED";
    case SystemState::kCritical:
        return "CRITICAL";
    case SystemState::kEmergency:
        return "EMERGENCY";
    case SystemState::kInAirReset:
        return "IN_AIR_RESET";
    }
    return "UNKNOWN";
}