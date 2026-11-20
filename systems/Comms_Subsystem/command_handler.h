#pragma once
// What comms calls when an authenticated, de-duplicated COMMAND_LONG arrives. The real
// implementation is StateBridge (state_bridge.h), which checks the FSM guards and
// latches one-loop command pulses. The returned MAV_RESULT goes back in COMMAND_ACK.
// Calls happen inside comms_update(), so they must return quickly (decide, do not wait
// for the transition).

#include "comms_mavlink.h"
#include "telemetry_source.h"

class CommandHandler
{
public:
    virtual ~CommandHandler() = default;
    // MAV_CMD_COMPONENT_ARM_DISARM. force = param2 was 21196 (the MAVLink "force" magic).
    virtual MAV_RESULT arm(bool arm, bool force) = 0;
    // MAV_CMD_NAV_TAKEOFF. param1 pitch (deg), param7 altitude (m).
    virtual MAV_RESULT takeoff(float pitch_deg, float altitude_m) = 0;
    // MAV_CMD_NAV_RETURN_TO_LAUNCH.
    virtual MAV_RESULT return_to_launch() = 0;
    // MAV_CMD_NAV_LOITER_UNLIM. param3 radius (m), param5/6 lat/lon (deg, 0 = here), param7 alt (m).
    virtual MAV_RESULT loiter_unlimited(float radius_m, float lat_deg, float lon_deg, float alt_m) = 0;
    // MAV_CMD_GOOSE_RESUME_MISSION: LOITER or RTL -> CRUISE.
    virtual MAV_RESULT resume_mission() = 0;
    // MAV_CMD_GOOSE_FAULT_CLEAR: acknowledge FAULT_GROUND / IN_AIR_RESET.
    virtual MAV_RESULT fault_clear() = 0;
    // MAV_CMD_GOOSE_BENCH: enter (true) or exit (false) BENCH.
    virtual MAV_RESULT bench(bool enter) = 0;
    // MAV_CMD_DO_SET_SERVO: param1 output (1-based), param2 pulse width (us). Only for
    // BENCH: implementations must return DENIED in any other system state. comms has
    // already checked the channel is 1..GOOSE_MAX_SERVOS and the width 1..65535 us;
    // servo limits are the implementer's job. Default: not supported.
    virtual MAV_RESULT set_servo(uint8_t /*channel*/, uint16_t /*pwm_us*/) { return MAV_RESULT_UNSUPPORTED; }
};

// Where StateBridge::set_servo sends an accepted bench setpoint. Implemented by manual
// controls (bench setpoints). Return false to refuse (unknown output, outside limits):
// the ground then gets DENIED.
class BenchServoSink
{
public:
    virtual ~BenchServoSink() = default;
    virtual bool set_bench_servo(uint8_t channel, uint16_t pwm_us) = 0;
};

// Stand-in for the FSM, for bench tests of the link alone (no sensors, no FSM code). It
// moves a DummyTelemetrySource state with simplified versions of the FSM guards:
//   ARM:      only from STANDBY -> ACTIVE (ACCEPTED if already armed)
//   DISARM:   only when landed (ON_GROUND, TAKEOFF_ROLL, REJECTED_TAKEOFF, LANDED)
//   TAKEOFF:  only ACTIVE and ON_GROUND -> TAKEOFF_ROLL
//   RTL:      CLIMB, CRUISE, LOITER -> RTL     LOITER: CRUISE -> LOITER
//   RESUME:   LOITER or RTL -> CRUISE
//   BENCH:    STANDBY -> BENCH; exit BENCH -> PREFLIGHT
//   FAULT_CLEAR: FAULT_GROUND or IN_AIR_RESET -> PREFLIGHT
//   SET_SERVO: only in BENCH, outputs 1-6, 800-2200 us; written to `servo_us` if given
// Use StateBridge with the real FSMs for anything beyond link testing.
class StubCommandHandler : public CommandHandler
{
public:
    explicit StubCommandHandler(GooseStateSnapshot &state, uint16_t *servo_us = nullptr)
        : s_(state), servo_us_(servo_us) {}
    MAV_RESULT arm(bool arm, bool force) override;
    MAV_RESULT takeoff(float pitch_deg, float altitude_m) override;
    MAV_RESULT return_to_launch() override;
    MAV_RESULT loiter_unlimited(float radius_m, float lat_deg, float lon_deg, float alt_m) override;
    MAV_RESULT resume_mission() override;
    MAV_RESULT fault_clear() override;
    MAV_RESULT bench(bool enter) override;
    MAV_RESULT set_servo(uint8_t channel, uint16_t pwm_us) override;

    uint32_t calls = 0; // handy in tests: how many commands actually reached the handler

private:
    GooseStateSnapshot &s_;
    uint16_t *servo_us_;
};
