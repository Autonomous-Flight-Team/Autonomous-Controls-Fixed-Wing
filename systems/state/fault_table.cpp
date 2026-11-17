#include "fault_table.h"

// to_string(FaultId) is inline in the header: it reads the name from kFaultTable.

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
    return "UNKNOWN";
}
