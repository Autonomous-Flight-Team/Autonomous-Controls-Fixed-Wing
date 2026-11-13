// State subsystem tests: system FSM and flight FSM run together, wired the way the
// main loop does it (system first, flight second, is_landed() fed back).
// Run: pio test -e native -f test_state

#include <unity.h>
#include "system_state.h"
#include "flight_state.h"

namespace
{

    constexpr uint32_t kLoopMs = 20; // 50 Hz control loop

    struct Harness
    {
        SystemStateFSM sys;
        FlightStateFSM fl;
        SystemInputs si;
        FlightInputs fi;
        uint32_t t = 0;

        // One control loop. Commands are one-loop pulses, so they are cleared after each step.
        void step()
        {
            t += kLoopMs;
            si.landed = fl.is_landed();
            sys.update(si, t);
            fi.system_state = sys.state();
            fl.update(fi, t);

            si.arm_cmd = si.disarm_cmd = si.fault_clear_cmd = false;
            si.bench_cmd = si.exit_bench_cmd = false;
            fi.takeoff_cmd = fi.loiter_cmd = fi.resume_cmd = fi.rtl_cmd = false;
        }

        void run_ms(uint32_t ms)
        {
            for (uint32_t e = 0; e < ms; e += kLoopMs)
                step();
        }

        void bring_up_and_arm()
        {
            si.boot_finished = true;
            si.preflight_ok = true;
            fi.mission_valid = true;
            step(); // BOOT -> PREFLIGHT
            step(); // PREFLIGHT -> STANDBY
            si.arm_cmd = true;
            step(); // STANDBY -> ACTIVE
        }

        // ON_GROUND -> TAKEOFF_ROLL -> ROTATE -> CLIMB -> CRUISE
        void takeoff_to_cruise()
        {
            fi.takeoff_cmd = true;
            step();
            fi.airspeed_mps = 13.0f;
            fi.groundspeed_mps = 13.0f;
            step();
            fi.agl_m = 6.0f;
            fi.climb_rate_mps = 2.0f;
            step();
            fi.target_alt_reached = true;
            step();
            fi.target_alt_reached = false;
        }

        // RTL -> DESCENT -> LANDING -> LANDED, then stop, disarm -> ON_GROUND
        void land_and_disarm()
        {
            fi.over_landing_site = true;
            step();
            fi.over_landing_site = false;
            fi.on_final = true;
            step();
            fi.on_final = false;
            fi.touchdown = true;
            fi.agl_m = 0.0f;
            step();
            fi.touchdown = false;
            fi.airspeed_mps = 0.0f;
            fi.groundspeed_mps = 0.0f;
            si.disarm_cmd = true;
            step();
            step();
        }

        void raise(FaultClass cls, bool recoverable, FaultResponse response)
        {
            si.highest_fault = cls;
            si.fault_recoverable = recoverable;
            fi.critical_response = response;
        }

        void clear_faults() { raise(FaultClass::kNone, false, FaultResponse::kNone); }
    };

#define ASSERT_SYS(h, s) TEST_ASSERT_EQUAL_STRING(to_string(s), to_string((h).sys.state()))
#define ASSERT_FLT(h, s) TEST_ASSERT_EQUAL_STRING(to_string(s), to_string((h).fl.state()))

} // namespace

void setUp() {}
void tearDown() {}

// ---------------- System FSM ----------------

void test_boot_to_armed()
{
    Harness h;
    h.bring_up_and_arm();
    ASSERT_SYS(h, SystemState::kActive);
    ASSERT_FLT(h, FlightState::kOnGround);
}

void test_boot_timeout_latches_fault_ground()
{
    Harness h;
    h.run_ms(31000); // boot_finished never set
    ASSERT_SYS(h, SystemState::kFaultGround);

    h.si.boot_finished = true;
    h.step();
    ASSERT_SYS(h, SystemState::kFaultGround); // latched: needs acknowledgement

    h.si.fault_clear_cmd = true;
    h.step();
    ASSERT_SYS(h, SystemState::kPreflight);
}

void test_arm_with_non_critical_fault_starts_degraded()
{
    Harness h;
    h.si.boot_finished = h.si.preflight_ok = true;
    h.step();
    h.step();
    h.raise(FaultClass::kNonCritical, true, FaultResponse::kNone);
    h.si.arm_cmd = true;
    h.step();
    ASSERT_SYS(h, SystemState::kDegraded);
}

void test_critical_recovers_after_t_hold()
{
    Harness h;
    h.bring_up_and_arm();
    h.takeoff_to_cruise();
    h.raise(FaultClass::kCritical, true, FaultResponse::kFailsafeLoiter);
    h.step();
    ASSERT_SYS(h, SystemState::kCritical);

    h.clear_faults();
    h.run_ms(4900);
    ASSERT_SYS(h, SystemState::kCritical); // t_hold not yet passed
    h.run_ms(300);
    ASSERT_SYS(h, SystemState::kActive); // CRITICAL -> DEGRADED -> ACTIVE
}

void test_non_recoverable_critical_latches_in_air()
{
    Harness h;
    h.bring_up_and_arm();
    h.takeoff_to_cruise();
    h.raise(FaultClass::kCritical, false, FaultResponse::kRtl);
    h.step();
    h.clear_faults();
    h.run_ms(10000);
    ASSERT_SYS(h, SystemState::kCritical);
}

void test_critical_latches_after_n_crit_max_entries()
{
    Harness h;
    h.bring_up_and_arm();
    h.takeoff_to_cruise();
    for (int i = 0; i < 3; ++i) // n_crit_max = 3
    {
        h.raise(FaultClass::kCritical, true, FaultResponse::kFailsafeLoiter);
        h.step();
        h.clear_faults();
        h.run_ms(5200);
    }
    ASSERT_SYS(h, SystemState::kCritical); // third entry can't recover
}

void test_failsafe_failed_escalates_to_emergency()
{
    Harness h;
    h.bring_up_and_arm();
    h.takeoff_to_cruise();
    h.raise(FaultClass::kCritical, true, FaultResponse::kRtl);
    h.step();
    h.si.failsafe_failed = true;
    h.step();
    ASSERT_SYS(h, SystemState::kEmergency);
    ASSERT_FLT(h, FlightState::kEmergencyLand);
}

void test_disarm_ignored_in_flight()
{
    Harness h;
    h.bring_up_and_arm();
    h.takeoff_to_cruise();
    h.si.disarm_cmd = true;
    h.step();
    TEST_ASSERT_TRUE(h.sys.is_armed());
}

void test_in_air_reset_needs_acknowledgement()
{
    Harness h;
    h.si.airborne_at_boot = true;
    h.step();
    ASSERT_SYS(h, SystemState::kInAirReset);
    h.si.airborne_at_boot = false;
    h.run_ms(1000);
    ASSERT_SYS(h, SystemState::kInAirReset);
    h.si.fault_clear_cmd = true;
    h.step();
    ASSERT_SYS(h, SystemState::kPreflight);
}

// ---------------- Flight FSM ----------------

void test_normal_flight_with_go_around()
{
    Harness h;
    h.bring_up_and_arm();
    h.takeoff_to_cruise();
    ASSERT_FLT(h, FlightState::kCruise);

    h.fi.mission_done = true;
    h.step();
    ASSERT_FLT(h, FlightState::kRtl);

    h.fi.over_landing_site = true;
    h.step();
    h.fi.over_landing_site = false;
    h.fi.on_final = true;
    h.step();
    h.fi.on_final = false;
    h.fi.go_around_needed = true;
    h.step();
    h.fi.go_around_needed = false;
    ASSERT_FLT(h, FlightState::kClimb);

    h.fi.target_alt_reached = true;
    h.step();
    h.fi.target_alt_reached = false;
    ASSERT_FLT(h, FlightState::kRtl); // back to the pattern, not the mission

    h.land_and_disarm();
    ASSERT_FLT(h, FlightState::kOnGround);
    ASSERT_SYS(h, SystemState::kStandby);
}

void test_go_around_limit_commits_landing()
{
    Harness h;
    h.bring_up_and_arm();
    h.takeoff_to_cruise();
    h.fi.rtl_cmd = true;
    h.step();
    for (int i = 0; i < 3; ++i) // n_goaround_max = 2
    {
        h.fi.over_landing_site = true;
        h.step();
        h.fi.over_landing_site = false;
        h.fi.on_final = true;
        h.step();
        h.fi.on_final = false;
        h.fi.go_around_needed = true;
        h.step();
        h.fi.go_around_needed = false;
        if (h.fl.state() == FlightState::kClimb)
        {
            h.fi.target_alt_reached = true;
            h.step();
            h.fi.target_alt_reached = false;
        }
    }
    ASSERT_FLT(h, FlightState::kLanding); // third go-around refused
}

void test_takeoff_refused_without_valid_mission()
{
    Harness h;
    h.bring_up_and_arm();
    h.fi.mission_valid = false;
    h.fi.takeoff_cmd = true;
    h.step();
    ASSERT_FLT(h, FlightState::kOnGround);
}

void test_critical_fault_on_takeoff_roll_rejects()
{
    Harness h;
    h.bring_up_and_arm();
    h.fi.takeoff_cmd = true;
    h.step();
    h.fi.airspeed_mps = h.fi.groundspeed_mps = 5.0f;
    h.raise(FaultClass::kCritical, true, FaultResponse::kRtl);
    h.step();
    ASSERT_FLT(h, FlightState::kRejectedTakeoff);
    h.fi.groundspeed_mps = 0.0f;
    h.step();
    ASSERT_FLT(h, FlightState::kOnGround);
}

void test_disarm_aborts_takeoff_roll()
{
    Harness h;
    h.bring_up_and_arm();
    h.fi.takeoff_cmd = true;
    h.step();
    h.fi.airspeed_mps = h.fi.groundspeed_mps = 6.0f;
    h.si.disarm_cmd = true;
    h.step();
    ASSERT_SYS(h, SystemState::kStandby);
    ASSERT_FLT(h, FlightState::kRejectedTakeoff);
}

void test_disarm_ignored_after_v1()
{
    Harness h;
    h.bring_up_and_arm();
    h.fi.takeoff_cmd = true;
    h.step();
    h.fi.airspeed_mps = 13.0f;
    h.step();
    ASSERT_FLT(h, FlightState::kRotate);
    h.si.disarm_cmd = true;
    h.step();
    TEST_ASSERT_TRUE(h.sys.is_armed());
}

void test_no_liftoff_rejects_takeoff()
{
    Harness h;
    h.bring_up_and_arm();
    h.fi.takeoff_cmd = true;
    h.step();
    h.fi.airspeed_mps = h.fi.groundspeed_mps = 13.0f;
    h.step();
    h.run_ms(8100); // t_takeoff_max, still on the ground
    ASSERT_FLT(h, FlightState::kRejectedTakeoff);
    h.fi.airspeed_mps = h.fi.groundspeed_mps = 0.0f;
    h.step();
    ASSERT_FLT(h, FlightState::kOnGround);
}

void test_gps_lost_failsafe_loiter_and_recovery()
{
    Harness h;
    h.bring_up_and_arm();
    h.takeoff_to_cruise();
    h.raise(FaultClass::kCritical, true, FaultResponse::kFailsafeLoiter);
    h.fi.gps_ok = false;
    h.step();
    ASSERT_FLT(h, FlightState::kFailsafeLoiter);

    h.clear_faults();
    h.fi.gps_ok = true;
    h.step();
    ASSERT_FLT(h, FlightState::kCruise);
}

void test_rtl_failsafe_is_sticky()
{
    Harness h;
    h.bring_up_and_arm();
    h.takeoff_to_cruise();
    h.raise(FaultClass::kCritical, false, FaultResponse::kRtl); // battery low
    h.step();
    ASSERT_FLT(h, FlightState::kRtl);

    h.fi.resume_cmd = true;
    h.step();
    ASSERT_FLT(h, FlightState::kRtl); // resume refused while CRITICAL

    h.land_and_disarm();
    ASSERT_SYS(h, SystemState::kFaultGround);
    ASSERT_FLT(h, FlightState::kOnGround);
}

void test_emergency_in_cruise_lands()
{
    Harness h;
    h.bring_up_and_arm();
    h.takeoff_to_cruise();
    h.raise(FaultClass::kEmergency, false, FaultResponse::kEmergencyLand);
    h.step();
    ASSERT_SYS(h, SystemState::kEmergency);
    ASSERT_FLT(h, FlightState::kEmergencyLand);
    h.fi.touchdown = true;
    h.step();
    ASSERT_FLT(h, FlightState::kLanded);
}

void test_manual_freezes_then_reengages_to_loiter()
{
    Harness h;
    h.bring_up_and_arm();
    h.takeoff_to_cruise();
    h.fi.manual_active = true;
    h.fi.rtl_cmd = true;
    h.run_ms(1000);
    ASSERT_FLT(h, FlightState::kCruise); // frozen while the pilot flies
    h.fi.manual_active = false;
    h.step();
    ASSERT_FLT(h, FlightState::kLoiter);
}

int main(int, char **)
{
    UNITY_BEGIN();

    // System FSM
    RUN_TEST(test_boot_to_armed);
    RUN_TEST(test_boot_timeout_latches_fault_ground);
    RUN_TEST(test_arm_with_non_critical_fault_starts_degraded);
    RUN_TEST(test_critical_recovers_after_t_hold);
    RUN_TEST(test_non_recoverable_critical_latches_in_air);
    RUN_TEST(test_critical_latches_after_n_crit_max_entries);
    RUN_TEST(test_failsafe_failed_escalates_to_emergency);
    RUN_TEST(test_disarm_ignored_in_flight);
    RUN_TEST(test_in_air_reset_needs_acknowledgement);

    // Flight FSM
    RUN_TEST(test_normal_flight_with_go_around);
    RUN_TEST(test_go_around_limit_commits_landing);
    RUN_TEST(test_takeoff_refused_without_valid_mission);
    RUN_TEST(test_critical_fault_on_takeoff_roll_rejects);
    RUN_TEST(test_disarm_aborts_takeoff_roll);
    RUN_TEST(test_disarm_ignored_after_v1);
    RUN_TEST(test_no_liftoff_rejects_takeoff);
    RUN_TEST(test_gps_lost_failsafe_loiter_and_recovery);
    RUN_TEST(test_rtl_failsafe_is_sticky);
    RUN_TEST(test_emergency_in_cruise_lands);
    RUN_TEST(test_manual_freezes_then_reengages_to_loiter);

    return UNITY_END();
}