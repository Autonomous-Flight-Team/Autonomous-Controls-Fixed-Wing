// Native tests for StateBridge (comms <-> real state machines): `pio test -e native`.
// The harness runs the loop in the order systems/state/README.MD gives, with
// bridge.apply() just before the FSM updates.

#include <unity.h>

#include <string.h>

#include "comms.h"
#include "comms_platform.h"
#include "state_bridge.h"

namespace
{
    constexpr uint32_t kLoopMs = 20; // 50 Hz

    struct Harness
    {
        SystemStateFSM sys;
        FlightStateFSM fl;
        SystemInputs si;
        FlightInputs fi;
        StateBridge bridge{sys, fl};
        uint32_t t = 0;

        void step()
        {
            t += kLoopMs;
            si.landed = fl.is_landed();
            bridge.apply(si, fi);
            sys.update(si, t);
            fi.system_state = sys.state();
            fl.update(fi, t);
            si.arm_cmd = si.disarm_cmd = si.fault_clear_cmd = false; // README step 6
            si.bench_cmd = si.exit_bench_cmd = false;
            fi.takeoff_cmd = fi.loiter_cmd = fi.resume_cmd = fi.rtl_cmd = false;
        }

        void run_ms(uint32_t ms)
        {
            for (uint32_t e = 0; e < ms; e += kLoopMs)
                step();
        }

        void to_standby()
        {
            si.boot_finished = true;
            si.preflight_ok = true;
            fi.mission_valid = true;
            step(); // BOOT -> PREFLIGHT
            step(); // PREFLIGHT -> STANDBY
        }

        void arm()
        {
            TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, bridge.arm(true, false));
            step();
            TEST_ASSERT_EQUAL(SystemState::kActive, sys.state());
        }

        void takeoff_to_cruise()
        {
            TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, bridge.takeoff(0, 0));
            step();
            TEST_ASSERT_EQUAL(FlightState::kTakeoffRoll, fl.state());
            fi.airspeed_mps = fi.groundspeed_mps = 13.0f;
            step(); // ROTATE
            fi.agl_m = 10.0f;
            fi.climb_rate_mps = 2.0f;
            step(); // CLIMB
            fi.target_alt_reached = true;
            step(); // CRUISE
            fi.target_alt_reached = false;
            TEST_ASSERT_EQUAL(FlightState::kCruise, fl.state());
        }
    };
} // namespace

void setUp() {}
void tearDown() {}

static void test_pulse_lasts_exactly_one_loop()
{
    Harness h;
    h.to_standby();
    TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, h.bridge.arm(true, false));
    TEST_ASSERT_FALSE(h.si.arm_cmd); // latched, not yet applied

    h.bridge.apply(h.si, h.fi);
    TEST_ASSERT_TRUE(h.si.arm_cmd);
    h.si.arm_cmd = false; // main loop clears after the updates
    h.bridge.apply(h.si, h.fi);
    TEST_ASSERT_FALSE(h.si.arm_cmd); // the pending pulse was consumed by the first apply()

    // flight pulses too
    Harness g;
    g.to_standby();
    g.arm();
    TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, g.bridge.takeoff(0, 0));
    g.bridge.apply(g.si, g.fi);
    TEST_ASSERT_TRUE(g.fi.takeoff_cmd);
    g.fi.takeoff_cmd = false;
    g.bridge.apply(g.si, g.fi);
    TEST_ASSERT_FALSE(g.fi.takeoff_cmd);
}

static void test_one_pending_pulse_per_fsm()
{
    Harness h;
    h.to_standby();
    TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, h.bridge.bench(true));
    TEST_ASSERT_EQUAL(MAV_RESULT_TEMPORARILY_REJECTED, h.bridge.arm(true, false)); // bench would win in the FSM
    h.step();
    TEST_ASSERT_EQUAL(SystemState::kBench, h.sys.state());
}

static void test_arm_guards()
{
    Harness h;
    TEST_ASSERT_EQUAL(MAV_RESULT_DENIED, h.bridge.arm(true, false)); // BOOT
    h.to_standby();
    TEST_ASSERT_EQUAL(SystemState::kStandby, h.sys.state());

    h.si.highest_fault = FaultClass::kCritical; // STANDBY re-checks before arming
    h.bridge.apply(h.si, h.fi);
    TEST_ASSERT_EQUAL(MAV_RESULT_DENIED, h.bridge.arm(true, false));
    h.si.highest_fault = FaultClass::kNone;
    h.si.preflight_ok = false;
    h.bridge.apply(h.si, h.fi);
    TEST_ASSERT_EQUAL(MAV_RESULT_DENIED, h.bridge.arm(true, false));
    h.si.preflight_ok = true;
    h.bridge.apply(h.si, h.fi);

    // non-critical fault: arm allowed, FSM starts DEGRADED
    h.si.highest_fault = FaultClass::kNonCritical;
    h.bridge.apply(h.si, h.fi);
    TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, h.bridge.arm(true, false));
    h.step();
    TEST_ASSERT_EQUAL(SystemState::kDegraded, h.sys.state());
    TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, h.bridge.arm(true, false)); // already armed: no-op
    h.bridge.apply(h.si, h.fi);
    TEST_ASSERT_FALSE(h.si.arm_cmd);
}

static void test_disarm_guards()
{
    Harness h;
    h.to_standby();
    TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, h.bridge.arm(false, false)); // already disarmed
    h.arm();
    h.takeoff_to_cruise();
    TEST_ASSERT_EQUAL(MAV_RESULT_DENIED, h.bridge.arm(false, true)); // airborne, even forced
    h.step();
    TEST_ASSERT_TRUE(h.sys.is_armed());

    // on the takeoff roll a disarm is allowed (aborts the roll)
    Harness g;
    g.to_standby();
    g.arm();
    TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, g.bridge.takeoff(0, 0));
    g.step();
    TEST_ASSERT_EQUAL(FlightState::kTakeoffRoll, g.fl.state());
    TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, g.bridge.arm(false, false));
    g.step();
    TEST_ASSERT_EQUAL(SystemState::kStandby, g.sys.state());

    // ROTATE is past V1: committed, no disarm
    Harness r;
    r.to_standby();
    r.arm();
    TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, r.bridge.takeoff(0, 0));
    r.step();
    r.fi.airspeed_mps = 13.0f;
    r.step();
    TEST_ASSERT_EQUAL(FlightState::kRotate, r.fl.state());
    TEST_ASSERT_EQUAL(MAV_RESULT_DENIED, r.bridge.arm(false, false));
}

static void test_takeoff_guards()
{
    Harness h;
    h.to_standby();
    TEST_ASSERT_EQUAL(MAV_RESULT_DENIED, h.bridge.takeoff(0, 0)); // not armed
    h.si.highest_fault = FaultClass::kNonCritical;
    h.bridge.apply(h.si, h.fi);
    TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, h.bridge.arm(true, false));
    h.step();
    TEST_ASSERT_EQUAL(SystemState::kDegraded, h.sys.state());
    TEST_ASSERT_EQUAL(MAV_RESULT_DENIED, h.bridge.takeoff(0, 0)); // code: ACTIVE only

    Harness g;
    g.to_standby();
    g.arm();
    g.fi.mission_valid = false;
    g.bridge.apply(g.si, g.fi);
    TEST_ASSERT_EQUAL(MAV_RESULT_DENIED, g.bridge.takeoff(0, 0)); // no valid mission
    g.fi.mission_valid = true;
    g.fi.manual_active = true;
    g.bridge.apply(g.si, g.fi);
    TEST_ASSERT_EQUAL(MAV_RESULT_DENIED, g.bridge.takeoff(0, 0)); // pilot has the servos
    g.fi.manual_active = false;
    g.bridge.apply(g.si, g.fi);
    TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, g.bridge.takeoff(0, 0));
    g.step();
    TEST_ASSERT_EQUAL(FlightState::kTakeoffRoll, g.fl.state());
}

static void test_loiter_rtl_resume_guards()
{
    Harness h;
    h.to_standby();
    TEST_ASSERT_EQUAL(MAV_RESULT_DENIED, h.bridge.loiter_unlimited(0, 0, 0, 0)); // on the ground
    TEST_ASSERT_EQUAL(MAV_RESULT_DENIED, h.bridge.return_to_launch());
    TEST_ASSERT_EQUAL(MAV_RESULT_DENIED, h.bridge.resume_mission());
    h.arm();
    h.takeoff_to_cruise();

    TEST_ASSERT_EQUAL(MAV_RESULT_DENIED, h.bridge.resume_mission()); // already in CRUISE
    TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, h.bridge.loiter_unlimited(0, 0, 0, 0));
    h.step();
    TEST_ASSERT_EQUAL(FlightState::kLoiter, h.fl.state());
    TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, h.bridge.loiter_unlimited(0, 0, 0, 0)); // no-op
    h.step();
    TEST_ASSERT_EQUAL(FlightState::kLoiter, h.fl.state());

    h.fi.mission_done = true; // LOITER resume needs an unfinished mission
    h.bridge.apply(h.si, h.fi);
    TEST_ASSERT_EQUAL(MAV_RESULT_DENIED, h.bridge.resume_mission());
    h.fi.mission_done = false;
    h.bridge.apply(h.si, h.fi);
    TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, h.bridge.resume_mission());
    h.step();
    TEST_ASSERT_EQUAL(FlightState::kCruise, h.fl.state());

    TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, h.bridge.return_to_launch());
    h.step();
    TEST_ASSERT_EQUAL(FlightState::kRtl, h.fl.state());
    TEST_ASSERT_EQUAL(MAV_RESULT_DENIED, h.bridge.loiter_unlimited(0, 0, 0, 0)); // RTL ignores loiter_cmd

    // RTL resume is refused while the system is CRITICAL
    h.si.highest_fault = FaultClass::kCritical;
    h.si.fault_recoverable = true;
    h.fi.critical_response = FaultResponse::kRtl;
    h.step();
    TEST_ASSERT_EQUAL(SystemState::kCritical, h.sys.state());
    TEST_ASSERT_EQUAL(MAV_RESULT_DENIED, h.bridge.resume_mission());
    h.si.highest_fault = FaultClass::kNone;
    h.run_ms(6000); // cleared for t_hold -> DEGRADED, then no fault left -> ACTIVE
    TEST_ASSERT_EQUAL(SystemState::kActive, h.sys.state());
    TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, h.bridge.resume_mission());
    h.step();
    TEST_ASSERT_EQUAL(FlightState::kCruise, h.fl.state());
}

static void test_commands_refused_during_failsafe_and_manual()
{
    Harness h;
    h.to_standby();
    h.arm();
    h.takeoff_to_cruise();
    h.fi.manual_active = true;
    h.step();
    TEST_ASSERT_EQUAL(MAV_RESULT_DENIED, h.bridge.return_to_launch());
    TEST_ASSERT_EQUAL(MAV_RESULT_DENIED, h.bridge.loiter_unlimited(0, 0, 0, 0));
    h.fi.manual_active = false;
    h.step(); // re-engage -> LOITER
    TEST_ASSERT_EQUAL(FlightState::kLoiter, h.fl.state());

    // GPS lost: FAILSAFE_LOITER takes over; no commanded loiter/RTL/resume there
    h.si.highest_fault = FaultClass::kCritical;
    h.si.fault_recoverable = true;
    h.fi.critical_response = FaultResponse::kFailsafeLoiter;
    h.fi.gps_ok = false;
    h.step();
    TEST_ASSERT_EQUAL(FlightState::kFailsafeLoiter, h.fl.state());
    TEST_ASSERT_EQUAL(MAV_RESULT_DENIED, h.bridge.return_to_launch());
    TEST_ASSERT_EQUAL(MAV_RESULT_DENIED, h.bridge.resume_mission());

    // CRITICAL with an RTL response in CRUISE: RTL happens anyway, so RTL is ACCEPTED, loiter DENIED
    Harness g;
    g.to_standby();
    g.arm();
    g.takeoff_to_cruise();
    g.si.highest_fault = FaultClass::kCritical;
    g.fi.critical_response = FaultResponse::kRtl;
    g.bridge.apply(g.si, g.fi);
    g.si.highest_fault = FaultClass::kCritical; // keep the fault for the update
    g.sys.update(g.si, g.t += kLoopMs);           // system -> CRITICAL, flight not yet updated
    TEST_ASSERT_EQUAL(SystemState::kCritical, g.sys.state());
    TEST_ASSERT_EQUAL(FlightState::kCruise, g.fl.state());
    TEST_ASSERT_EQUAL(MAV_RESULT_DENIED, g.bridge.loiter_unlimited(0, 0, 0, 0));
    TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, g.bridge.return_to_launch());
    g.step();
    TEST_ASSERT_EQUAL(FlightState::kRtl, g.fl.state());
}

static void test_bench_and_fault_clear_guards()
{
    Harness h;
    TEST_ASSERT_EQUAL(MAV_RESULT_DENIED, h.bridge.bench(true)); // BOOT
    TEST_ASSERT_EQUAL(MAV_RESULT_DENIED, h.bridge.bench(false)); // not in BENCH
    h.to_standby();
    TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, h.bridge.bench(true));
    h.step();
    TEST_ASSERT_EQUAL(SystemState::kBench, h.sys.state());
    TEST_ASSERT_EQUAL(MAV_RESULT_DENIED, h.bridge.arm(true, false)); // no arming from BENCH
    TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, h.bridge.bench(false));
    h.step();
    TEST_ASSERT_EQUAL(SystemState::kPreflight, h.sys.state()); // code: BENCH exits via PREFLIGHT
    h.step();
    TEST_ASSERT_EQUAL(SystemState::kStandby, h.sys.state());
    TEST_ASSERT_EQUAL(MAV_RESULT_DENIED, h.bridge.fault_clear()); // nothing to clear

    // boot timeout latches FAULT_GROUND; clear is refused while a critical fault is active
    Harness g;
    g.run_ms(31000);
    TEST_ASSERT_EQUAL(SystemState::kFaultGround, g.sys.state());
    g.si.highest_fault = FaultClass::kCritical;
    g.bridge.apply(g.si, g.fi);
    TEST_ASSERT_EQUAL(MAV_RESULT_DENIED, g.bridge.fault_clear());
    g.si.highest_fault = FaultClass::kNone;
    g.step();
    TEST_ASSERT_EQUAL(SystemState::kFaultGround, g.sys.state()); // still latched
    TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, g.bridge.fault_clear());
    g.step();
    TEST_ASSERT_EQUAL(SystemState::kPreflight, g.sys.state());

    // IN_AIR_RESET leaves only on fault clear
    Harness r;
    r.si.airborne_at_boot = true;
    r.step();
    TEST_ASSERT_EQUAL(SystemState::kInAirReset, r.sys.state());
    TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, r.bridge.arm(false, false)); // disarmed already
    TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, r.bridge.fault_clear());
    r.step();
    TEST_ASSERT_EQUAL(SystemState::kPreflight, r.sys.state());
}

static void test_snapshot_matches_fsm()
{
    Harness h;
    uint8_t sensors[GOOSE_NUM_SENSORS] = {1, 1, 2, 3, 1, 1, 1, 0};
    h.bridge.set_sensor_status(sensors);
    h.to_standby();
    h.arm();
    h.takeoff_to_cruise();
    FaultMask mask = fault_bit(FaultId::kGpsLost) | fault_bit(FaultId::kServo);
    h.bridge.set_fault_mask(mask);
    h.si.highest_fault = FaultClass::kCritical;
    h.si.fault_recoverable = true;
    h.step();
    h.fi.manual_active = true;
    h.step();

    GooseStateSnapshot s;
    TEST_ASSERT_TRUE(h.bridge.goose_state(s));
    TEST_ASSERT_EQUAL((uint8_t)h.sys.state(), s.system_state);
    TEST_ASSERT_EQUAL(GOOSE_SYSTEM_STATE_CRITICAL, s.system_state);
    TEST_ASSERT_EQUAL((uint8_t)h.fl.state(), s.flight_state);
    TEST_ASSERT_EQUAL(h.sys.get_num_crit_entries(), s.critical_entries);
    TEST_ASSERT_EQUAL(1, s.critical_entries);
    TEST_ASSERT_EQUAL_UINT32(mask, s.active_faults);
    TEST_ASSERT_EQUAL(GOOSE_FAULT_GPS_LOST | GOOSE_FAULT_SERVO, s.active_faults);
    TEST_ASSERT_EQUAL(GOOSE_STATE_FLAGS_ARMED | GOOSE_STATE_FLAGS_MANUAL_ACTIVE, s.flags);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(sensors, s.sensor_status, GOOSE_NUM_SENSORS);
}

static void test_armed_helper_matches_fsm()
{
    for (int v = 0; v < 256; v++)
    {
        bool fsm = is_armed(static_cast<SystemState>(v)); // false for values that are not states
        TEST_ASSERT_EQUAL(fsm, goose_system_armed((uint8_t)v));
    }
}

// End to end: a MAVLink COMMAND_LONG through comms reaches the real FSM via the bridge.
struct RecordingSink : BenchServoSink
{
    uint8_t channel = 0;
    uint16_t pwm = 0;
    bool refuse = false;
    bool set_bench_servo(uint8_t ch, uint16_t us) override
    {
        if (refuse)
            return false;
        channel = ch;
        pwm = us;
        return true;
    }
};

struct FakeOutputs : TelemetrySource // no goose_state(): a sensor source need not implement it
{
    bool servo_outputs(ServoOutputSample &o) override
    {
        o = ServoOutputSample();
        o.count = 1;
        o.us[0] = 1234;
        return true;
    }
    bool rc_inputs(RcInputSample &o) override
    {
        o = RcInputSample();
        o.count = 1;
        o.us[0] = 1700;
        o.link_quality_pct = 90;
        return true;
    }
};

static void test_set_servo_only_in_bench()
{
    Harness h;
    TEST_ASSERT_EQUAL(MAV_RESULT_UNSUPPORTED, h.bridge.set_servo(1, 1500)); // no sink
    RecordingSink sink;
    h.bridge.set_bench_servo_sink(&sink);

    h.to_standby();
    TEST_ASSERT_EQUAL(MAV_RESULT_DENIED, h.bridge.set_servo(1, 1600)); // STANDBY
    TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, h.bridge.bench(true));
    TEST_ASSERT_EQUAL(MAV_RESULT_DENIED, h.bridge.set_servo(1, 1600)); // BENCH requested, not entered yet
    h.step();
    TEST_ASSERT_EQUAL(SystemState::kBench, h.sys.state());

    TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, h.bridge.set_servo(3, 1600));
    TEST_ASSERT_EQUAL(3, sink.channel);
    TEST_ASSERT_EQUAL(1600, sink.pwm);
    sink.refuse = true; // outside the sink's limits
    TEST_ASSERT_EQUAL(MAV_RESULT_DENIED, h.bridge.set_servo(3, 2500));
    sink.refuse = false;

    TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, h.bridge.bench(false));
    TEST_ASSERT_EQUAL(MAV_RESULT_DENIED, h.bridge.set_servo(3, 1700)); // exit pending
    h.step();
    TEST_ASSERT_EQUAL(MAV_RESULT_DENIED, h.bridge.set_servo(3, 1700)); // PREFLIGHT
    TEST_ASSERT_EQUAL(1600, sink.pwm);
}

static void test_servo_and_rc_pass_through()
{
    SystemStateFSM sys;
    FlightStateFSM fl;
    ServoOutputSample so;
    RcInputSample rc;
    StateBridge none(sys, fl);
    TEST_ASSERT_FALSE(none.servo_outputs(so));
    TEST_ASSERT_FALSE(none.rc_inputs(rc));

    FakeOutputs src;
    StateBridge bridge(sys, fl, &src);
    TEST_ASSERT_TRUE(bridge.servo_outputs(so));
    TEST_ASSERT_EQUAL(1234, so.us[0]);
    TEST_ASSERT_TRUE(bridge.rc_inputs(rc));
    TEST_ASSERT_EQUAL(1700, rc.us[0]);
    TEST_ASSERT_EQUAL(90, rc.link_quality_pct);
}

static void test_end_to_end_through_comms()
{
    static PipeRing up, down;
    static MemoryByteStream vehicle(up, down);
    up.clear();
    down.clear();
    comms_test_wipe_store();
    comms_test_set_time_us(0);

    Harness h;
    CommsConfig cfg;
    cfg.require_signing = false; // signing is covered in test_comms
    comms_init(vehicle, h.bridge, h.bridge, cfg);
    h.to_standby();

    mavlink_message_t m;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_command_long_pack_chan(255, 190, MAVLINK_COMM_1, &m, COMMS_SYSID, COMMS_COMPID,
                                       MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0);
    uint16_t n = mavlink_msg_to_send_buffer(buf, &m);
    for (uint16_t i = 0; i < n; i++)
        up.push(buf[i]);

    int acks = 0, result = -1;
    uint8_t flags = 0, sys_state = 0;
    for (int loop = 0; loop < 50; loop++)
    {
        h.step();
        comms_test_advance_us(kLoopMs * 1000);
        comms_update();
        mavlink_status_t st;
        int c;
        while ((c = down.pop()) >= 0)
            if (mavlink_parse_char(MAVLINK_COMM_1, (uint8_t)c, &m, &st))
            {
                if (m.msgid == MAVLINK_MSG_ID_COMMAND_ACK)
                {
                    acks++;
                    result = mavlink_msg_command_ack_get_result(&m);
                }
                if (m.msgid == MAVLINK_MSG_ID_GOOSE_STATE)
                {
                    flags = mavlink_msg_goose_state_get_flags(&m);
                    sys_state = mavlink_msg_goose_state_get_system_state(&m);
                }
            }
    }
    TEST_ASSERT_EQUAL(1, acks);
    TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, result);
    TEST_ASSERT_EQUAL(SystemState::kActive, h.sys.state());
    TEST_ASSERT_EQUAL(GOOSE_SYSTEM_STATE_ACTIVE, sys_state);
    TEST_ASSERT_EQUAL(GOOSE_STATE_FLAGS_ARMED, flags);
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_pulse_lasts_exactly_one_loop);
    RUN_TEST(test_one_pending_pulse_per_fsm);
    RUN_TEST(test_arm_guards);
    RUN_TEST(test_disarm_guards);
    RUN_TEST(test_takeoff_guards);
    RUN_TEST(test_loiter_rtl_resume_guards);
    RUN_TEST(test_commands_refused_during_failsafe_and_manual);
    RUN_TEST(test_bench_and_fault_clear_guards);
    RUN_TEST(test_snapshot_matches_fsm);
    RUN_TEST(test_armed_helper_matches_fsm);
    RUN_TEST(test_end_to_end_through_comms);
    RUN_TEST(test_set_servo_only_in_bench);
    RUN_TEST(test_servo_and_rc_pass_through);
    return UNITY_END();
}
