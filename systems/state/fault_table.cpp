#include "fault_table.h"

// to_string(FaultId) is inline in the header: it reads the name from kFaultTable.

const char *to_string(FaultClass c)
{
    switch (c)
    {
    case FaultClass::kNone:
        return "NONE";
    case FaultClass::kNonCritical:
        return "NON_CRITICAL";
    case FaultClass::kCritical:
        return "CRITICAL";
    case FaultClass::kEmergency:
        return "EMERGENCY";
    }
    return "UNKNOWN";
}

const char *to_string(FaultResponse r)
{
    switch (r)
    {
    case FaultResponse::kNone:
        return "NONE";
    case FaultResponse::kFailsafeLoiter:
        return "FAILSAFE_LOITER";
    case FaultResponse::kRtl:
        return "RTL";
    case FaultResponse::kEmergencyLand:
        return "EMERGENCY_LAND";
    }
    return "UNKNOWN";
}
