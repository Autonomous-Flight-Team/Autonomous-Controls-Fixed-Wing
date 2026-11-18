#include "state_bridge.h"

#include <string.h>

// ---------------------------------------------------------------- dialect == FSM code
// goose.xml mirrors SystemState, FlightState and FaultId value for value. These checks
// fail the build if either side changes without the other:
//  - each static_assert compares one value;
//  - each switch lists every enumerator with no default, and -Wswitch is an error here,
//    so a new SystemState / FlightState / FaultId that goose.xml lacks stops the build.

#pragma GCC diagnostic push
#pragma GCC diagnostic error "-Wswitch"

#define SAME(goose, fsm) static_assert((int)(goose) == (int)(fsm), #goose " != " #fsm ": update goose.xml")

static constexpr int goose_value(SystemState s)
{
    switch (s)
    {
    case SystemState::kBoot: return GOOSE_SYSTEM_STATE_BOOT;
    case SystemState::kPreflight: return GOOSE_SYSTEM_STATE_PREFLIGHT;
    case SystemState::kStandby: return GOOSE_SYSTEM_STATE_STANDBY;
    case SystemState::kFaultGround: return GOOSE_SYSTEM_STATE_FAULT_GROUND;
    case SystemState::kBench: return GOOSE_SYSTEM_STATE_BENCH;
    case SystemState::kActive: return GOOSE_SYSTEM_STATE_ACTIVE;
    case SystemState::kDegraded: return GOOSE_SYSTEM_STATE_DEGRADED;
    case SystemState::kCritical: return GOOSE_SYSTEM_STATE_CRITICAL;
    case SystemState::kEmergency: return GOOSE_SYSTEM_STATE_EMERGENCY;
    case SystemState::kInAirReset: return GOOSE_SYSTEM_STATE_IN_AIR_RESET;
    }
    return -1;
}

static constexpr int goose_value(FlightState s)
{
    switch (s)
    {
    case FlightState::kOnGround: return GOOSE_FLIGHT_STATE_ON_GROUND;
    case FlightState::kTakeoffRoll: return GOOSE_FLIGHT_STATE_TAKEOFF_ROLL;
    case FlightState::kRotate: return GOOSE_FLIGHT_STATE_ROTATE;
    case FlightState::kLanded: return GOOSE_FLIGHT_STATE_LANDED;
    case FlightState::kClimb: return GOOSE_FLIGHT_STATE_CLIMB;
    case FlightState::kCruise: return GOOSE_FLIGHT_STATE_CRUISE;
    case FlightState::kLoiter: return GOOSE_FLIGHT_STATE_LOITER;
    case FlightState::kRtl: return GOOSE_FLIGHT_STATE_RTL;
    case FlightState::kDescent: return GOOSE_FLIGHT_STATE_DESCENT;
    case FlightState::kLanding: return GOOSE_FLIGHT_STATE_LANDING;
    case FlightState::kRejectedTakeoff: return GOOSE_FLIGHT_STATE_REJECTED_TAKEOFF;
    case FlightState::kFailsafeLoiter: return GOOSE_FLIGHT_STATE_FAILSAFE_LOITER;
    case FlightState::kEmergencyLand: return GOOSE_FLIGHT_STATE_EMERGENCY_LAND;
    }
    return -1;
}

static constexpr long goose_value(FaultId f)
{
    switch (f)
    {
    case FaultId::kTelemetryLost: return GOOSE_FAULT_TELEMETRY_LOST;
    case FaultId::kRedundantSensor: return GOOSE_FAULT_REDUNDANT_SENSOR;
    case FaultId::kBaro: return GOOSE_FAULT_BARO;
    case FaultId::kMag: return GOOSE_FAULT_MAG;
    case FaultId::kLoggingFailure: return GOOSE_FAULT_LOGGING_FAILURE;
    case FaultId::kRcLost: return GOOSE_FAULT_RC_LOST;
    case FaultId::kLoopOverrun: return GOOSE_FAULT_LOOP_OVERRUN;
    case FaultId::kGpsLost: return GOOSE_FAULT_GPS_LOST;
    case FaultId::kAirspeed: return GOOSE_FAULT_AIRSPEED;
    case FaultId::kGeofence: return GOOSE_FAULT_GEOFENCE;
    case FaultId::kBatteryLow: return GOOSE_FAULT_BATTERY_LOW;
    case FaultId::kInAirReset: return GOOSE_FAULT_IN_AIR_RESET;
    case FaultId::kBatteryCritical: return GOOSE_FAULT_BATTERY_CRITICAL;
    case FaultId::kImuLost: return GOOSE_FAULT_IMU_LOST;
    case FaultId::kEstimatorDiverged: return GOOSE_FAULT_ESTIMATOR_DIVERGED;
    case FaultId::kMotorEsc: return GOOSE_FAULT_MOTOR_ESC;
    case FaultId::kServo: return GOOSE_FAULT_SERVO;
    case FaultId::kCount: return -1;
    }
    return -1;
}

#pragma GCC diagnostic pop

// System state: same value, every enumerator.
SAME(goose_value(SystemState::kBoot), SystemState::kBoot);
SAME(goose_value(SystemState::kPreflight), SystemState::kPreflight);
SAME(goose_value(SystemState::kStandby), SystemState::kStandby);
SAME(goose_value(SystemState::kFaultGround), SystemState::kFaultGround);
SAME(goose_value(SystemState::kBench), SystemState::kBench);
SAME(goose_value(SystemState::kActive), SystemState::kActive);
SAME(goose_value(SystemState::kDegraded), SystemState::kDegraded);
SAME(goose_value(SystemState::kCritical), SystemState::kCritical);
SAME(goose_value(SystemState::kEmergency), SystemState::kEmergency);
SAME(goose_value(SystemState::kInAirReset), SystemState::kInAirReset);
SAME(GOOSE_SYSTEM_STATE_ENUM_END, (int)SystemState::kInAirReset + 1); // nothing extra in goose.xml

// Flight state.
SAME(goose_value(FlightState::kOnGround), FlightState::kOnGround);
SAME(goose_value(FlightState::kTakeoffRoll), FlightState::kTakeoffRoll);
SAME(goose_value(FlightState::kRotate), FlightState::kRotate);
SAME(goose_value(FlightState::kLanded), FlightState::kLanded);
SAME(goose_value(FlightState::kClimb), FlightState::kClimb);
SAME(goose_value(FlightState::kCruise), FlightState::kCruise);
SAME(goose_value(FlightState::kLoiter), FlightState::kLoiter);
SAME(goose_value(FlightState::kRtl), FlightState::kRtl);
SAME(goose_value(FlightState::kDescent), FlightState::kDescent);
SAME(goose_value(FlightState::kLanding), FlightState::kLanding);
SAME(goose_value(FlightState::kRejectedTakeoff), FlightState::kRejectedTakeoff);
SAME(goose_value(FlightState::kFailsafeLoiter), FlightState::kFailsafeLoiter);
SAME(goose_value(FlightState::kEmergencyLand), FlightState::kEmergencyLand);
SAME(GOOSE_FLIGHT_STATE_ENUM_END, (int)FlightState::kEmergencyLand + 1);

// Faults: GOOSE_FAULT bit N == fault_bit(FaultId N), and exactly kFaultCount bits.
#define SAME_FAULT(id) static_assert(goose_value(FaultId::id) == (long)fault_bit(FaultId::id), \
                                     "GOOSE_FAULT bit != fault_bit(FaultId::" #id "): update goose.xml")
SAME_FAULT(kTelemetryLost);
SAME_FAULT(kRedundantSensor);
SAME_FAULT(kBaro);
SAME_FAULT(kMag);
SAME_FAULT(kLoggingFailure);
SAME_FAULT(kRcLost);
SAME_FAULT(kLoopOverrun);
SAME_FAULT(kGpsLost);
SAME_FAULT(kAirspeed);
SAME_FAULT(kGeofence);
SAME_FAULT(kBatteryLow);
SAME_FAULT(kInAirReset);
SAME_FAULT(kBatteryCritical);
SAME_FAULT(kImuLost);
SAME_FAULT(kEstimatorDiverged);
SAME_FAULT(kMotorEsc);
SAME_FAULT(kServo);
static_assert(kFaultCount == 17, "fault count changed: add the new FaultId to goose.xml and to the lists above");
static_assert((long)GOOSE_FAULT_ENUM_END == (1L << (kFaultCount - 1)) + 1, "goose.xml GOOSE_FAULT has extra or missing bits");
static_assert(sizeof(GooseStateSnapshot::active_faults) >= sizeof(FaultMask), "active_faults too small for FaultMask");

// goose_system_armed() (comms) vs is_armed(SystemState): checked in test_state_bridge.

// ---------------------------------------------------------------- loop interface

void StateBridge::apply(SystemInputs &si, FlightInputs &fi)
{
    si.arm_cmd = si.arm_cmd || sys_pending_.arm;
    si.disarm_cmd = si.disarm_cmd || sys_pending_.disarm;
    si.bench_cmd = si.bench_cmd || sys_pending_.bench;
    si.exit_bench_cmd = si.exit_bench_cmd || sys_pending_.exit_bench;
    si.fault_clear_cmd = si.fault_clear_cmd || sys_pending_.fault_clear;
    fi.takeoff_cmd = fi.takeoff_cmd || fl_pending_.takeoff;
    fi.loiter_cmd = fi.loiter_cmd || fl_pending_.loiter;
    fi.resume_cmd = fi.resume_cmd || fl_pending_.resume;
    fi.rtl_cmd = fi.rtl_cmd || fl_pending_.rtl;
    sys_pending_ = SystemPulses{};
    fl_pending_ = FlightPulses{};
    seen_si_ = si;
    seen_fi_ = fi;
}

void StateBridge::set_sensor_status(const uint8_t status[GOOSE_NUM_SENSORS])
{
    memcpy(sensor_status_, status, sizeof(sensor_status_));
}

// ---------------------------------------------------------------- guards
// Each guard mirrors a branch of SystemStateFSM::update() / FlightStateFSM::update().

bool StateBridge::system_ready_for_ground_cmd() const
{
    // kStandby: "if (f >= kCritical || !preflight_ok) -> PREFLIGHT" is checked before bench/arm.
    return sys_.state() == SystemState::kStandby && seen_si_.preflight_ok &&
           seen_si_.highest_fault < FaultClass::kCritical;
}

bool StateBridge::failsafe_override_active() const
{
    // Flight FSM step 2: while airborne, system EMERGENCY / CRITICAL-with-a-response move
    // the flight state first and the normal (commanded) transitions are skipped.
    if (!fl_.is_airborne())
        return false;
    SystemState s = sys_.state();
    return s == SystemState::kEmergency ||
           (s == SystemState::kCritical && seen_fi_.critical_response != FaultResponse::kNone);
}

MAV_RESULT StateBridge::arm(bool arm, bool /*force: the FSM has no forced disarm*/)
{
    if (sys_pending_.any())
        return MAV_RESULT_TEMPORARILY_REJECTED;
    if (arm)
    {
        if (sys_.is_armed())
            return MAV_RESULT_ACCEPTED; // already armed: nothing to do
        if (!system_ready_for_ground_cmd())
            return MAV_RESULT_DENIED;
        sys_pending_.arm = true;
        return MAV_RESULT_ACCEPTED;
    }
    if (!sys_.is_armed())
        return MAV_RESULT_ACCEPTED; // already disarmed
    // try_disarm(): only when landed (ON_GROUND, TAKEOFF_ROLL, REJECTED_TAKEOFF, LANDED).
    if (!fl_.is_landed())
        return MAV_RESULT_DENIED;
    sys_pending_.disarm = true;
    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT StateBridge::bench(bool enter)
{
    if (sys_pending_.any())
        return MAV_RESULT_TEMPORARILY_REJECTED;
    if (enter)
    {
        if (sys_.state() == SystemState::kBench)
            return MAV_RESULT_ACCEPTED;
        if (!system_ready_for_ground_cmd())
            return MAV_RESULT_DENIED;
        sys_pending_.bench = true;
        return MAV_RESULT_ACCEPTED;
    }
    if (sys_.state() != SystemState::kBench)
        return MAV_RESULT_DENIED;
    sys_pending_.exit_bench = true;
    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT StateBridge::fault_clear()
{
    if (sys_pending_.any())
        return MAV_RESULT_TEMPORARILY_REJECTED;
    SystemState s = sys_.state();
    // kFaultGround: ignored while a critical fault is still active.
    // kInAirReset: leaves only on fault_clear_cmd.
    if ((s == SystemState::kFaultGround && seen_si_.highest_fault < FaultClass::kCritical) ||
        s == SystemState::kInAirReset)
    {
        sys_pending_.fault_clear = true;
        return MAV_RESULT_ACCEPTED;
    }
    return MAV_RESULT_DENIED;
}

MAV_RESULT StateBridge::takeoff(float, float)
{
    if (fl_pending_.any())
        return MAV_RESULT_TEMPORARILY_REJECTED;
    // kOnGround: takeoff_cmd && system == ACTIVE (not DEGRADED) && mission_valid.
    if (seen_fi_.manual_active || fl_.state() != FlightState::kOnGround || sys_.state() != SystemState::kActive ||
        !seen_fi_.mission_valid)
        return MAV_RESULT_DENIED;
    fl_pending_.takeoff = true;
    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT StateBridge::loiter_unlimited(float, float, float, float)
{
    if (fl_pending_.any())
        return MAV_RESULT_TEMPORARILY_REJECTED;
    if (seen_fi_.manual_active || failsafe_override_active())
        return MAV_RESULT_DENIED;
    if (fl_.state() == FlightState::kLoiter)
        return MAV_RESULT_ACCEPTED;
    // loiter_cmd is only read in kCruise.
    if (fl_.state() != FlightState::kCruise)
        return MAV_RESULT_DENIED;
    fl_pending_.loiter = true;
    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT StateBridge::return_to_launch()
{
    if (fl_pending_.any())
        return MAV_RESULT_TEMPORARILY_REJECTED;
    if (seen_fi_.manual_active)
        return MAV_RESULT_DENIED;
    if (fl_.state() == FlightState::kRtl)
        return MAV_RESULT_ACCEPTED;
    // rtl_cmd is read in kClimb, kCruise and kLoiter.
    FlightState f = fl_.state();
    if (f != FlightState::kClimb && f != FlightState::kCruise && f != FlightState::kLoiter)
        return MAV_RESULT_DENIED;
    if (failsafe_override_active())
    {
        // A CRITICAL fault with an RTL response sends these states to RTL anyway.
        bool rtl_anyway = sys_.state() == SystemState::kCritical && seen_fi_.critical_response == FaultResponse::kRtl;
        return rtl_anyway ? MAV_RESULT_ACCEPTED : MAV_RESULT_DENIED;
    }
    fl_pending_.rtl = true;
    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT StateBridge::resume_mission()
{
    if (fl_pending_.any())
        return MAV_RESULT_TEMPORARILY_REJECTED;
    if (seen_fi_.manual_active || failsafe_override_active())
        return MAV_RESULT_DENIED;
    FlightState f = fl_.state();
    // kLoiter: mission_valid && !mission_done && resume_cmd (rtl_cmd checked first).
    bool from_loiter = f == FlightState::kLoiter && seen_fi_.mission_valid && !seen_fi_.mission_done;
    // kRtl: resume_cmd && system != CRITICAL && mission_valid (over_landing_site checked first).
    bool from_rtl = f == FlightState::kRtl && sys_.state() != SystemState::kCritical && seen_fi_.mission_valid &&
                    !seen_fi_.over_landing_site;
    if (!from_loiter && !from_rtl)
        return MAV_RESULT_DENIED;
    fl_pending_.resume = true;
    return MAV_RESULT_ACCEPTED;
}

// ---------------------------------------------------------------- telemetry

bool StateBridge::goose_state(GooseStateSnapshot &out)
{
    out.system_state = (uint8_t)sys_.state();
    out.flight_state = (uint8_t)fl_.state();
    out.active_faults = faults_;
    out.critical_entries = sys_.get_num_crit_entries();
    out.flags = (uint8_t)((sys_.is_armed() ? GOOSE_STATE_FLAGS_ARMED : 0) |
                          (seen_fi_.manual_active ? GOOSE_STATE_FLAGS_MANUAL_ACTIVE : 0));
    memcpy(out.sensor_status, sensor_status_, sizeof(out.sensor_status));
    return true;
}
