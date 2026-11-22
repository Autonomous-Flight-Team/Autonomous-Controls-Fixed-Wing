#include "command_handler.h"

static bool flight_is(const GooseStateSnapshot &s, uint8_t a, uint8_t b = 0xFF, uint8_t c = 0xFF, uint8_t d = 0xFF)
{
    return s.flight_state == a || s.flight_state == b || s.flight_state == c || s.flight_state == d;
}

MAV_RESULT StubCommandHandler::arm(bool arm, bool /*force*/)
{
    calls++;
    bool armed = goose_system_armed(s_.system_state);
    if (arm)
    {
        if (armed)
            return MAV_RESULT_ACCEPTED;
        if (s_.system_state != GOOSE_SYSTEM_STATE_STANDBY)
            return MAV_RESULT_DENIED;
        s_.system_state = GOOSE_SYSTEM_STATE_ACTIVE;
        return MAV_RESULT_ACCEPTED;
    }
    if (!armed)
        return MAV_RESULT_ACCEPTED;
    if (!flight_is(s_, GOOSE_FLIGHT_STATE_ON_GROUND, GOOSE_FLIGHT_STATE_TAKEOFF_ROLL, GOOSE_FLIGHT_STATE_REJECTED_TAKEOFF,
                   GOOSE_FLIGHT_STATE_LANDED))
        return MAV_RESULT_DENIED; // like the FSM: never disarms in the air, even with force
    s_.system_state = s_.active_faults ? GOOSE_SYSTEM_STATE_FAULT_GROUND : GOOSE_SYSTEM_STATE_STANDBY;
    s_.flight_state = GOOSE_FLIGHT_STATE_ON_GROUND;
    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT StubCommandHandler::takeoff(float, float)
{
    calls++;
    if (s_.system_state != GOOSE_SYSTEM_STATE_ACTIVE || s_.flight_state != GOOSE_FLIGHT_STATE_ON_GROUND)
        return MAV_RESULT_DENIED;
    s_.flight_state = GOOSE_FLIGHT_STATE_TAKEOFF_ROLL;
    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT StubCommandHandler::return_to_launch()
{
    calls++;
    if (s_.flight_state == GOOSE_FLIGHT_STATE_RTL)
        return MAV_RESULT_ACCEPTED;
    if (!flight_is(s_, GOOSE_FLIGHT_STATE_CLIMB, GOOSE_FLIGHT_STATE_CRUISE, GOOSE_FLIGHT_STATE_LOITER))
        return MAV_RESULT_DENIED;
    s_.flight_state = GOOSE_FLIGHT_STATE_RTL;
    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT StubCommandHandler::loiter_unlimited(float, float, float, float)
{
    calls++;
    if (s_.flight_state == GOOSE_FLIGHT_STATE_LOITER)
        return MAV_RESULT_ACCEPTED;
    if (s_.flight_state != GOOSE_FLIGHT_STATE_CRUISE)
        return MAV_RESULT_DENIED;
    s_.flight_state = GOOSE_FLIGHT_STATE_LOITER;
    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT StubCommandHandler::resume_mission()
{
    calls++;
    if (!flight_is(s_, GOOSE_FLIGHT_STATE_LOITER, GOOSE_FLIGHT_STATE_RTL))
        return MAV_RESULT_DENIED;
    s_.flight_state = GOOSE_FLIGHT_STATE_CRUISE;
    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT StubCommandHandler::fault_clear()
{
    calls++;
    if (s_.system_state != GOOSE_SYSTEM_STATE_FAULT_GROUND && s_.system_state != GOOSE_SYSTEM_STATE_IN_AIR_RESET)
        return MAV_RESULT_DENIED;
    s_.system_state = GOOSE_SYSTEM_STATE_PREFLIGHT;
    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT StubCommandHandler::bench(bool enter)
{
    calls++;
    if (enter)
    {
        if (s_.system_state != GOOSE_SYSTEM_STATE_STANDBY)
            return s_.system_state == GOOSE_SYSTEM_STATE_BENCH ? MAV_RESULT_ACCEPTED : MAV_RESULT_DENIED;
        s_.system_state = GOOSE_SYSTEM_STATE_BENCH;
        return MAV_RESULT_ACCEPTED;
    }
    if (s_.system_state != GOOSE_SYSTEM_STATE_BENCH)
        return MAV_RESULT_DENIED;
    s_.system_state = GOOSE_SYSTEM_STATE_PREFLIGHT;
    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT StubCommandHandler::set_servo(uint8_t channel, uint16_t pwm_us)
{
    calls++;
    if (s_.system_state != GOOSE_SYSTEM_STATE_BENCH || channel < 1 || channel > 6 || pwm_us < 800 || pwm_us > 2200)
        return MAV_RESULT_DENIED;
    if (servo_us_)
        servo_us_[channel - 1] = pwm_us;
    return MAV_RESULT_ACCEPTED;
}
