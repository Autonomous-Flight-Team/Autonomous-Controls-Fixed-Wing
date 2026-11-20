// Fault manager tests: unit tests on its own, then the fault manager and both FSMs wired
// the way the main loop does it (fault manager -> system FSM -> flight FSM).
// Run: pio test -e native -f test_fault_manager

#include "fault_manager.h"
#include "flight_state.h"
#include "system_state.h"
#include <unity.h>

namespace {

constexpr uint32_t kLoopMs = 20; // 50 Hz control loop

// ---- Fault manager alone ----
struct FmHarness {
    FaultManager fm;
    FaultManagerInputs in;
    uint32_t t = 0;

    FmHarness() = default;
    explicit FmHarness(const FaultManagerParams& p) : fm(p) {}

    void step() {
        t += kLoopMs;
        fm.update(in, t);
        in.fault_clear_cmd = false;
    }

    void run_ms(uint32_t ms) {
        for (uint32_t e = 0; e < ms; e += kLoopMs)
            step();
    }

    void set(FaultId id) { in.raw |= fault_bit(id); }
    void clear(FaultId id) { in.raw &= ~fault_bit(id); }
};

// ---- Fault manager + both FSMs ----
struct Harness {
    FaultManager fm;
    SystemStateFSM sys;
    FlightStateFSM fl;
    FaultManagerInputs fmi;
    SystemInputs si;
    FlightInputs fi;
    uint32_t t = 0;

    // One control loop, in the README's order.
    void step() {
        t += kLoopMs;
        fm.update(fmi, t);

        si.highest_fault = fm.highest_class();
        si.fault_recoverable = fm.recoverable();
        si.fault_clear_cmd = fmi.fault_clear_cmd;
        si.landed = fl.is_landed();
        sys.update(si, t);

        fi.system_state = sys.state();
        fi.critical_response = fm.critical_response();
        fi.gps_ok = (fmi.raw & fault_bit(FaultId::kGpsLost)) == 0; // straight from the GPS
        fi.motor_ok = !fm.is_active(FaultId::kMotorEsc);
        fl.update(fi, t);

        fmi.fault_clear_cmd = false;
        si.arm_cmd = si.disarm_cmd = si.fault_clear_cmd = false;
        si.bench_cmd = si.exit_bench_cmd = false;
        fi.takeoff_cmd = fi.loiter_cmd = fi.resume_cmd = fi.rtl_cmd = false;
    }

    void run_ms(uint32_t ms) {
        for (uint32_t e = 0; e < ms; e += kLoopMs)
            step();
    }

    void set(FaultId id) { fmi.raw |= fault_bit(id); }
    void clear(FaultId id) { fmi.raw &= ~fault_bit(id); }

    void bring_up_takeoff_to_cruise() {
        si.boot_finished = si.preflight_ok = true;
        fi.mission_valid = true;
        step();
        step();
        si.arm_cmd = true;
        step();
        fi.takeoff_cmd = true;
        step();
        fi.airspeed_mps = fi.groundspeed_mps = 13.0f;
        step();
        fi.agl_m = 30.0f;
        fi.climb_rate_mps = 2.0f;
        step();
        fi.target_alt_reached = true;
        step();
        fi.target_alt_reached = false;
    }
};

#define ASSERT_CLASS(expected, actual)                                                             \
    TEST_ASSERT_EQUAL_STRING(to_string(expected), to_string(actual))
#define ASSERT_RESP(expected, actual)                                                              \
    TEST_ASSERT_EQUAL_STRING(to_string(expected), to_string(actual))
#define ASSERT_SYS(h, s) TEST_ASSERT_EQUAL_STRING(to_string(s), to_string((h).sys.state()))
#define ASSERT_FLT(h, s) TEST_ASSERT_EQUAL_STRING(to_string(s), to_string((h).fl.state()))

} // namespace

void setUp() {}
void tearDown() {}

// ---------------- Debounce ----------------

void test_raise_after_debounce() {
    FmHarness h;
    h.set(FaultId::kGpsLost);
    h.run_ms(180);
    TEST_ASSERT_FALSE(h.fm.is_active(FaultId::kGpsLost));
    h.run_ms(40);
    TEST_ASSERT_TRUE(h.fm.is_active(FaultId::kGpsLost));
    TEST_ASSERT_EQUAL_HEX32(fault_bit(FaultId::kGpsLost), h.fm.raised_mask());
    h.step();
    TEST_ASSERT_EQUAL_HEX32(0, h.fm.raised_mask()); // event only on the raising loop
    TEST_ASSERT_FALSE(h.fm.changed());
}

void test_short_glitches_ignored() {
    FmHarness h;
    for (int i = 0; i < 5; ++i) {
        h.set(FaultId::kBaro);
        h.run_ms(100);
        h.clear(FaultId::kBaro);
        h.run_ms(40);
    }
    TEST_ASSERT_FALSE(h.fm.is_active(FaultId::kBaro));
    ASSERT_CLASS(FaultClass::kNone, h.fm.highest_class());
}

void test_clear_after_debounce() {
    FmHarness h;
    h.set(FaultId::kMag);
    h.run_ms(220);
    h.clear(FaultId::kMag);
    h.run_ms(180);
    TEST_ASSERT_TRUE(h.fm.is_active(FaultId::kMag)); // still holding
    h.run_ms(40);
    TEST_ASSERT_FALSE(h.fm.is_active(FaultId::kMag));
    TEST_ASSERT_EQUAL_HEX32(fault_bit(FaultId::kMag), h.fm.cleared_mask());
}

void test_debounce_across_millis_wrap() {
    FmHarness h;
    h.t = UINT32_MAX - 100;
    h.set(FaultId::kGeofence);
    h.run_ms(180);
    TEST_ASSERT_FALSE(h.fm.is_active(FaultId::kGeofence));
    h.run_ms(40);
    TEST_ASSERT_TRUE(h.fm.is_active(FaultId::kGeofence));
}

// ---------------- Summary outputs ----------------

void test_non_critical_only() {
    FmHarness h;
    h.set(FaultId::kTelemetryLost);
    h.run_ms(220);
    ASSERT_CLASS(FaultClass::kNonCritical, h.fm.highest_class());
    ASSERT_RESP(FaultResponse::kNone, h.fm.critical_response()); // no flight effect
    TEST_ASSERT_TRUE(h.fm.recoverable());
}

void test_response_priority_and_recoverability() {
    FmHarness h;
    h.set(FaultId::kGpsLost);
    h.run_ms(220);
    ASSERT_CLASS(FaultClass::kCritical, h.fm.highest_class());
    ASSERT_RESP(FaultResponse::kFailsafeLoiter, h.fm.critical_response());
    TEST_ASSERT_TRUE(h.fm.recoverable());

    h.set(FaultId::kBatteryLow); // RTL beats FAILSAFE_LOITER; non-recoverable taints the episode
    h.run_ms(220);
    ASSERT_RESP(FaultResponse::kRtl, h.fm.critical_response());
    TEST_ASSERT_FALSE(h.fm.recoverable());

    h.set(FaultId::kImuLost);
    h.run_ms(220);
    ASSERT_CLASS(FaultClass::kEmergency, h.fm.highest_class());
    ASSERT_RESP(FaultResponse::kEmergencyLand, h.fm.critical_response());
}

// ---------------- Escalation ----------------

void test_rc_lost_escalates_after_t_rc() {
    FmHarness h;
    h.set(FaultId::kRcLost);
    h.run_ms(220); // raised at ~200 ms
    ASSERT_CLASS(FaultClass::kNonCritical, h.fm.highest_class());
    h.run_ms(2940);
    ASSERT_CLASS(FaultClass::kNonCritical, h.fm.highest_class()); // t_rc not yet
    h.run_ms(60);
    ASSERT_CLASS(FaultClass::kCritical, h.fm.highest_class());
    ASSERT_RESP(FaultResponse::kRtl, h.fm.critical_response());
    TEST_ASSERT_TRUE(h.fm.recoverable());

    h.clear(FaultId::kRcLost); // RC back: escalation drops with the fault
    h.run_ms(220);
    TEST_ASSERT_FALSE(h.fm.is_active(FaultId::kRcLost));
    TEST_ASSERT_FALSE(h.fm.is_escalated(FaultId::kRcLost));
    ASSERT_CLASS(FaultClass::kNone, h.fm.highest_class());
}

void test_rc_lost_never_escalates_when_not_required() {
    FaultManagerParams p;
    p.rc_required = false;
    FmHarness h(p);
    h.set(FaultId::kRcLost);
    h.run_ms(20000);
    ASSERT_CLASS(FaultClass::kNonCritical, h.fm.highest_class());
}

void test_rc_escalation_restarts_after_clear() {
    FmHarness h;
    h.set(FaultId::kRcLost);
    h.run_ms(2000);
    h.clear(FaultId::kRcLost);
    h.run_ms(300); // cleared
    h.set(FaultId::kRcLost);
    h.run_ms(2000); // 2 s again, not 4 s total
    ASSERT_CLASS(FaultClass::kNonCritical, h.fm.highest_class());
}

void test_gps_lost_escalates_after_t_loiter() {
    FmHarness h;
    h.set(FaultId::kGpsLost);
    h.run_ms(220);
    h.run_ms(59900);
    ASSERT_CLASS(FaultClass::kCritical, h.fm.highest_class());
    h.run_ms(200);
    ASSERT_CLASS(FaultClass::kEmergency, h.fm.highest_class());
    TEST_ASSERT_EQUAL_HEX32(0, h.fm.escalated_mask()); // event was on an earlier loop
    TEST_ASSERT_TRUE(h.fm.is_escalated(FaultId::kGpsLost));
}

void test_loop_overrun_streak() {
    const int n = FaultManagerParams{}.n_overrun_max;
    FmHarness h;
    h.set(FaultId::kLoopOverrun);
    h.step();
    TEST_ASSERT_TRUE(h.fm.is_active(FaultId::kLoopOverrun)); // raised on the first miss
    ASSERT_CLASS(FaultClass::kNonCritical, h.fm.highest_class());

    for (int i = 0; i < n - 2; ++i) // n - 1 in a row, then a clean loop
        h.step();
    h.clear(FaultId::kLoopOverrun);
    h.step();
    h.set(FaultId::kLoopOverrun);
    for (int i = 0; i < n - 1; ++i)
        h.step();
    ASSERT_CLASS(FaultClass::kNonCritical, h.fm.highest_class());

    h.step(); // n-th in a row
    ASSERT_CLASS(FaultClass::kEmergency, h.fm.highest_class());
    ASSERT_RESP(FaultResponse::kEmergencyLand, h.fm.critical_response());
    TEST_ASSERT_EQUAL_HEX32(fault_bit(FaultId::kLoopOverrun), h.fm.escalated_mask());
}

// ---------------- In-air reset ----------------

void test_in_air_reset_latched_until_ack() {
    FmHarness h;
    h.set(FaultId::kInAirReset);
    h.step();
    TEST_ASSERT_TRUE(h.fm.is_active(FaultId::kInAirReset)); // no debounce
    h.clear(FaultId::kInAirReset);
    h.run_ms(5000);
    TEST_ASSERT_TRUE(h.fm.is_active(FaultId::kInAirReset)); // latched
    ASSERT_CLASS(FaultClass::kCritical, h.fm.highest_class());
    h.in.fault_clear_cmd = true;
    h.step();
    TEST_ASSERT_FALSE(h.fm.is_active(FaultId::kInAirReset));
    ASSERT_CLASS(FaultClass::kNone, h.fm.highest_class());
}

// ---------------- Fault manager + FSMs ----------------

void test_gps_loss_loiters_then_emergency_lands() {
    Harness h;
    h.bring_up_takeoff_to_cruise();
    ASSERT_FLT(h, FlightState::kCruise);
    h.set(FaultId::kGpsLost);
    h.step();
    ASSERT_FLT(h, FlightState::kCruise); // still debouncing
    h.run_ms(220);
    ASSERT_SYS(h, SystemState::kCritical);
    ASSERT_FLT(h, FlightState::kFailsafeLoiter);
    h.run_ms(60000);
    ASSERT_SYS(h, SystemState::kEmergency);
    ASSERT_FLT(h, FlightState::kEmergencyLand);
}

void test_gps_back_during_debounce_resumes_without_oscillating() {
    Harness h;
    h.bring_up_takeoff_to_cruise();
    h.set(FaultId::kGpsLost);
    h.run_ms(1000);
    ASSERT_FLT(h, FlightState::kFailsafeLoiter);
    h.clear(FaultId::kGpsLost); // fault stays active for debounce, CRITICAL for t_hold
    h.step();
    ASSERT_FLT(h, FlightState::kCruise);
    h.run_ms(5500);
    ASSERT_FLT(h, FlightState::kCruise);
    ASSERT_SYS(h, SystemState::kActive);
}

void test_rc_loss_degrades_then_rtl_and_recovers() {
    Harness h;
    h.bring_up_takeoff_to_cruise();
    h.set(FaultId::kRcLost);
    h.run_ms(220);
    ASSERT_SYS(h, SystemState::kDegraded);
    ASSERT_FLT(h, FlightState::kCruise);
    h.run_ms(3000);
    ASSERT_SYS(h, SystemState::kCritical);
    ASSERT_FLT(h, FlightState::kRtl);

    h.clear(FaultId::kRcLost);
    h.run_ms(5500); // debounce + t_hold
    ASSERT_SYS(h, SystemState::kActive);
    ASSERT_FLT(h, FlightState::kRtl); // sticky until resume_cmd
}

void test_battery_low_with_gps_loss_goes_home_when_gps_returns() {
    Harness h;
    h.bring_up_takeoff_to_cruise();
    h.set(FaultId::kGpsLost);
    h.set(FaultId::kBatteryLow);
    h.run_ms(220);
    ASSERT_FLT(h, FlightState::kFailsafeLoiter); // no GPS: can't fly home yet
    h.clear(FaultId::kGpsLost);
    h.step();
    ASSERT_FLT(h, FlightState::kRtl);
}

void test_in_air_reset_full_path() {
    Harness h;
    h.si.airborne_at_boot = true;
    h.si.boot_finished = h.si.preflight_ok = true;
    h.set(FaultId::kInAirReset);
    h.step();
    ASSERT_SYS(h, SystemState::kInAirReset);
    h.si.airborne_at_boot = false;
    h.clear(FaultId::kInAirReset);
    h.run_ms(2000);
    ASSERT_SYS(h, SystemState::kInAirReset);

    h.fmi.fault_clear_cmd = true; // fault manager and system FSM see the same pulse
    h.step();
    ASSERT_SYS(h, SystemState::kPreflight);
    h.step();
    ASSERT_SYS(h, SystemState::kStandby); // not bounced to FAULT_GROUND
}

void test_loop_overrun_streak_emergency_lands() {
    Harness h;
    h.bring_up_takeoff_to_cruise();
    h.set(FaultId::kLoopOverrun);
    h.step();
    ASSERT_SYS(h, SystemState::kDegraded);
    h.run_ms((FaultManagerParams{}.n_overrun_max - 2) * kLoopMs);
    ASSERT_SYS(h, SystemState::kDegraded); // one miss short
    h.step();
    ASSERT_SYS(h, SystemState::kEmergency);
    ASSERT_FLT(h, FlightState::kEmergencyLand);
}

int main(int, char**) {
    UNITY_BEGIN();

    RUN_TEST(test_raise_after_debounce);
    RUN_TEST(test_short_glitches_ignored);
    RUN_TEST(test_clear_after_debounce);
    RUN_TEST(test_debounce_across_millis_wrap);
    RUN_TEST(test_non_critical_only);
    RUN_TEST(test_response_priority_and_recoverability);
    RUN_TEST(test_rc_lost_escalates_after_t_rc);
    RUN_TEST(test_rc_lost_never_escalates_when_not_required);
    RUN_TEST(test_rc_escalation_restarts_after_clear);
    RUN_TEST(test_gps_lost_escalates_after_t_loiter);
    RUN_TEST(test_loop_overrun_streak);
    RUN_TEST(test_in_air_reset_latched_until_ack);

    RUN_TEST(test_gps_loss_loiters_then_emergency_lands);
    RUN_TEST(test_gps_back_during_debounce_resumes_without_oscillating);
    RUN_TEST(test_rc_loss_degrades_then_rtl_and_recovers);
    RUN_TEST(test_battery_low_with_gps_loss_goes_home_when_gps_returns);
    RUN_TEST(test_in_air_reset_full_path);
    RUN_TEST(test_loop_overrun_streak_emergency_lands);

    return UNITY_END();
}
