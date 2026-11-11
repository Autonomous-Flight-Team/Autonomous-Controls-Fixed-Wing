#include "flight_state.h"

const char* to_string(FlightState s)
{
    switch (s)
    {
    case FlightState::kOnGround:
        return "ON_GROUND";
    case FlightState::kTakeoffRoll:
        return "TAKEOFF_ROLL";
    case FlightState::kRotate:
        return "ROTATE";
    case FlightState::kLanded:
        return "LANDED";
    case FlightState::kClimb:
        return "CLIMB";
    case FlightState::kCruise:
        return "CRUISE";
    case FlightState::kLoiter:
        return "LOITER";
    case FlightState::kRtl:
        return "RTL";
    case FlightState::kDescent:
        return "DESCENT";
    case FlightState::kLanding:
        return "LANDING";
    case FlightState::kRejectedTakeoff:
        return "REJECTED_TAKEOFF";
    case FlightState::kFailsafeLoiter:
        return "FAILSAFE_LOITER";
    case FlightState::kEmergencyLand:
        return "EMERGENCY_LAND";
    }
    return "UNKNOWN";
}