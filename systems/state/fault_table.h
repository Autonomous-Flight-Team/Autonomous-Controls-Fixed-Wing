#ifndef FAULT_TABLE_H_
#define FAULT_TABLE_H_

#include <cstddef>
#include <cstdint>

// Fault severity. Order matters: the highest active class wins, so compare with >.
enum class FaultClass : uint8_t
{
    kNone = 0,
    kNonCritical = 1, // -> DEGRADED: log only, no flight effect
    kCritical = 2,    // -> CRITICAL: failsafe
    kEmergency = 3,   // -> EMERGENCY: flight can't continue
};

// What the flight FSM does when this fault is the active CRITICAL/EMERGENCY one.
enum class FaultResponse : uint8_t
{
    kNone = 0, // log only
    kFailsafeLoiter = 1,
    kRtl = 2,
    kEmergencyLand = 3,
};

// One bit per fault in FaultMask: the value IS the bit index.
// Values are written to logs and telemetry: never renumber, only add before kCount.
enum class FaultId : uint8_t
{
    // Non-critical
    kTelemetryLost = 0,   // no GCS packets for debounce_ms
    kRedundantSensor = 1, // one of N sensors invalid or out of agreement
    kBaro = 2,            // no samples, stuck, or disagrees with GPS altitude
    kMag = 3,             // no samples, stuck, or disagrees with GPS course
    kLoggingFailure = 4,  // SD card full or write error; blocks arming on ground
    kRcLost = 5,          // no valid RC frames or receiver failsafe flag
    kLoopOverrun = 6,     // control loop missed its deadline

    // Critical
    kGpsLost = 7,     // no 3D fix, sats < N_sat, or HDOP > H_max
    kAirspeed = 8,    // pitot stuck, out of range, or disagrees with GPS + wind by > dV_as
    kGeofence = 9,    // position outside fence
    kBatteryLow = 10, // past return reserve
    kInAirReset = 11, // rebooted airborne; handled in BOOT -> IN_AIR_RESET

    // Emergency
    kBatteryCritical = 12,   // below landing reserve
    kImuLost = 13,           // no samples or values out of range
    kEstimatorDiverged = 14, // innovations or covariance over limit
    kMotorEsc = 15,          // no current or RPM response to throttle
    kServo = 16,             // feedback mismatch (if measured)

    kCount // not a fault: number of faults
};
// Watchdog timeout / pilot switch -> MANUAL is hardware (servo mux), not in this table.

using FaultMask = uint32_t;
static_assert(static_cast<size_t>(FaultId::kCount) <= 32, "FaultMask is too small");

constexpr FaultMask fault_bit(FaultId id)
{
    return FaultMask{1} << static_cast<uint8_t>(id);
}

struct FaultDef
{
    FaultId id;
    const char *name;        // for logs; matches page 3 of the drawio
    FaultClass cls;          // class when first raised
    FaultClass escalates_to; // class after escalation (kNone = never escalates)
    bool recoverable;        // can CRITICAL recover to DEGRADED once it clears
    FaultResponse response;  // flight response at its CRITICAL/EMERGENCY class
};

// Escalation triggers (fault manager):
//   kRcLost:      lost > t_rc, only if RC_REQUIRED
//   kLoopOverrun: N_overrun_max misses in a row
//   kGpsLost:     still lost after t_loiter
inline constexpr FaultDef kFaultTable[] = {
    // id                           name                    class                     escalates to            recov  response
    {FaultId::kTelemetryLost, "TELEMETRY_LOST", FaultClass::kNonCritical, FaultClass::kNone, true, FaultResponse::kNone},
    {FaultId::kRedundantSensor, "REDUNDANT_SENSOR", FaultClass::kNonCritical, FaultClass::kNone, true, FaultResponse::kNone},
    {FaultId::kBaro, "BARO", FaultClass::kNonCritical, FaultClass::kNone, true, FaultResponse::kNone},
    {FaultId::kMag, "MAG", FaultClass::kNonCritical, FaultClass::kNone, true, FaultResponse::kNone},
    {FaultId::kLoggingFailure, "LOGGING_FAILURE", FaultClass::kNonCritical, FaultClass::kNone, true, FaultResponse::kNone},
    {FaultId::kRcLost, "RC_LOST", FaultClass::kNonCritical, FaultClass::kCritical, true, FaultResponse::kRtl},
    {FaultId::kLoopOverrun, "LOOP_OVERRUN", FaultClass::kNonCritical, FaultClass::kEmergency, true, FaultResponse::kEmergencyLand},
    {FaultId::kGpsLost, "GPS_LOST", FaultClass::kCritical, FaultClass::kEmergency, true, FaultResponse::kFailsafeLoiter},
    {FaultId::kAirspeed, "AIRSPEED", FaultClass::kCritical, FaultClass::kNone, false, FaultResponse::kRtl},
    {FaultId::kGeofence, "GEOFENCE", FaultClass::kCritical, FaultClass::kNone, false, FaultResponse::kRtl},
    {FaultId::kBatteryLow, "BATTERY_LOW", FaultClass::kCritical, FaultClass::kNone, false, FaultResponse::kRtl},
    {FaultId::kInAirReset, "IN_AIR_RESET", FaultClass::kCritical, FaultClass::kNone, false, FaultResponse::kNone},
    {FaultId::kBatteryCritical, "BATTERY_CRITICAL", FaultClass::kEmergency, FaultClass::kNone, false, FaultResponse::kEmergencyLand},
    {FaultId::kImuLost, "IMU_LOST", FaultClass::kEmergency, FaultClass::kNone, false, FaultResponse::kEmergencyLand},
    {FaultId::kEstimatorDiverged, "ESTIMATOR_DIVERGED", FaultClass::kEmergency, FaultClass::kNone, false, FaultResponse::kEmergencyLand},
    {FaultId::kMotorEsc, "MOTOR_ESC", FaultClass::kEmergency, FaultClass::kNone, false, FaultResponse::kEmergencyLand},
    {FaultId::kServo, "SERVO", FaultClass::kEmergency, FaultClass::kNone, false, FaultResponse::kEmergencyLand},
};

inline constexpr size_t kFaultCount = static_cast<size_t>(FaultId::kCount);

// Compile-time check: one row per FaultId, in enum order, so kFaultTable[id] works.
constexpr bool fault_table_in_order()
{
    if (sizeof(kFaultTable) / sizeof(kFaultTable[0]) != kFaultCount)
        return false;
    for (size_t i = 0; i < kFaultCount; ++i)
        if (static_cast<size_t>(kFaultTable[i].id) != i)
            return false;
    return true;
}
static_assert(fault_table_in_order(), "kFaultTable must have one row per FaultId, in enum order");

constexpr const FaultDef &fault_def(FaultId id)
{
    return kFaultTable[static_cast<size_t>(id)];
}

const char *to_string(FaultClass c);
const char *to_string(FaultResponse r);
inline const char *to_string(FaultId id) { return fault_def(id).name; }

#endif // FAULT_TABLE_H_