#include "fault_table.h"

// No default case: -Wswitch flags any enum value added without a string here.
// The return after each switch only covers out-of-range values (e.g. a bad cast from a log byte).

const char* to_string(FaultClass c) {
    switch (c) {
    case FaultClass::kNone:
        return "NONE";
    case FaultClass::kNonCritical:
        return "NON_CRITICAL";
    case FaultClass::kCritical:
        return "CRITICAL";
    case FaultClass::kEmergency:
        return "EMERGENCY";
    }
    return "UNKNOWN_CLASS";
}

const char* to_string(FaultResponse r) {
    switch (r) {
    case FaultResponse::kNone:
        return "NONE";
    case FaultResponse::kFailsafeLoiter:
        return "FAILSAFE_LOITER";
    case FaultResponse::kRtl:
        return "RTL";
    case FaultResponse::kEmergencyLand:
        return "EMERGENCY_LAND";
    }
    return "UNKNOWN_RESPONSE";
}