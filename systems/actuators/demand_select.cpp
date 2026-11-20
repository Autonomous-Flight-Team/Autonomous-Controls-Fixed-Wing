#include "demand_select.h"

const char* to_string(OutputSource s) {
    switch (s) {
    case OutputSource::kSafe:
        return "SAFE";
    case OutputSource::kMirror:
        return "MIRROR";
    case OutputSource::kBench:
        return "BENCH";
    case OutputSource::kControl:
        return "CONTROL";
    }
    return "UNKNOWN";
}

OutputSource select_source(SystemState sys, bool manual_active) {
    if (manual_active) {
        return OutputSource::kMirror;
    }
    switch (sys) {
    case SystemState::kInAirReset:
        return OutputSource::kMirror;
    case SystemState::kBench:
        return OutputSource::kBench;
    case SystemState::kActive:
    case SystemState::kDegraded:
    case SystemState::kCritical:
    case SystemState::kEmergency:
        return OutputSource::kControl;
    case SystemState::kBoot:
    case SystemState::kPreflight:
    case SystemState::kStandby:
    case SystemState::kFaultGround:
        return OutputSource::kSafe;
    }
    return OutputSource::kSafe;
}
