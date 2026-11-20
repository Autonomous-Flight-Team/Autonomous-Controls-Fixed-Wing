#include "telemetry_source.h"

#include <math.h>

#include "comms_mavlink.h"

DummyTelemetrySource::DummyTelemetrySource()
{
    state_.system_state = GOOSE_SYSTEM_STATE_STANDBY;
    state_.flight_state = GOOSE_FLIGHT_STATE_ON_GROUND;
    for (int i = 0; i < GOOSE_NUM_SENSORS; i++)
        state_.sensor_status[i] = GOOSE_SENSOR_STATUS_NOMINAL;
    state_.sensor_status[GOOSE_SENSOR_INDEX_SPARE] = GOOSE_SENSOR_STATUS_UNINITIALIZED;
}

bool goose_system_armed(uint8_t s)
{
    return s == GOOSE_SYSTEM_STATE_ACTIVE || s == GOOSE_SYSTEM_STATE_DEGRADED || s == GOOSE_SYSTEM_STATE_CRITICAL ||
           s == GOOSE_SYSTEM_STATE_EMERGENCY;
}

// Walks the fake state forward so a bench test can exercise every command:
// PREFLIGHT -> STANDBY after 1 s; TAKEOFF_ROLL -> CLIMB -> CRUISE and
// RTL -> DESCENT -> LANDING -> LANDED, 5 s per step.
void DummyTelemetrySource::update(uint32_t now_ms)
{
    now_ms_ = now_ms;
    if (state_.system_state != last_system_state_)
    {
        last_system_state_ = state_.system_state;
        system_since_ms_ = now_ms;
    }
    if (state_.system_state == GOOSE_SYSTEM_STATE_PREFLIGHT && now_ms - system_since_ms_ >= 1000)
        state_.system_state = GOOSE_SYSTEM_STATE_STANDBY;

    if (state_.flight_state != last_flight_state_)
    {
        last_flight_state_ = state_.flight_state;
        flight_since_ms_ = now_ms;
    }
    if (now_ms - flight_since_ms_ < 5000)
        return;
    switch (state_.flight_state)
    {
    case GOOSE_FLIGHT_STATE_TAKEOFF_ROLL: state_.flight_state = GOOSE_FLIGHT_STATE_CLIMB; break;
    case GOOSE_FLIGHT_STATE_CLIMB: state_.flight_state = GOOSE_FLIGHT_STATE_CRUISE; break;
    case GOOSE_FLIGHT_STATE_RTL: state_.flight_state = GOOSE_FLIGHT_STATE_DESCENT; break;
    case GOOSE_FLIGHT_STATE_DESCENT: state_.flight_state = GOOSE_FLIGHT_STATE_LANDING; break;
    case GOOSE_FLIGHT_STATE_LANDING: state_.flight_state = GOOSE_FLIGHT_STATE_LANDED; break;
    default: break;
    }
}

static float phase(uint32_t ms, float period_s) { return 2.0f * (float)M_PI * (float)(ms % (uint32_t)(period_s * 1000)) / (period_s * 1000.0f); }

bool DummyTelemetrySource::goose_state(GooseStateSnapshot &out)
{
    out = state_;
    out.flags = (uint8_t)((out.flags & ~GOOSE_STATE_FLAGS_ARMED) | (goose_system_armed(out.system_state) ? GOOSE_STATE_FLAGS_ARMED : 0));
    return true;
}

bool DummyTelemetrySource::attitude(AttitudeSample &out)
{
    float p = phase(now_ms_, 10.0f);
    out = {0.3f * sinf(p), 0.1f * cosf(p), p - (float)M_PI, 0.3f * cosf(p), -0.1f * sinf(p), 0.6f};
    return true;
}

bool DummyTelemetrySource::position(PositionSample &out)
{
    float p = phase(now_ms_, 60.0f);
    out.lat = 476530000 + (int32_t)(5000 * sinf(p)); // around Seattle
    out.lon = -1223050000 + (int32_t)(5000 * cosf(p));
    out.alt_mm = 120000;
    out.relative_alt_mm = 100000;
    out.vx = (int16_t)(1500 * cosf(p));
    out.vy = (int16_t)(-1500 * sinf(p));
    out.vz = 0;
    out.hdg_cdeg = (uint16_t)(p * 18000.0f / (float)M_PI) % 36000;
    return true;
}

bool DummyTelemetrySource::gps(GpsSample &out)
{
    PositionSample pos;
    position(pos);
    out = {GPS_FIX_TYPE_3D_FIX, pos.lat, pos.lon, pos.alt_mm, 90, 140, 1500, pos.hdg_cdeg, 14};
    return true;
}

bool DummyTelemetrySource::air_data(AirDataSample &out)
{
    out = {16.0f, 15.0f, 120.0f, 0.0f, (int16_t)(now_ms_ / 1000 % 360), 55};
    return true;
}

bool DummyTelemetrySource::imu(ImuSample &out)
{
    out = {0, 0, -1000, 0, 0, 0, 200, 0, 400, 2500};
    return true;
}

bool DummyTelemetrySource::pressure(PressureSample &out)
{
    out = {1013.25f, 1.6f, 2500};
    return true;
}

bool DummyTelemetrySource::battery(BatterySample &out)
{
    out = {12400, 850, (int32_t)(now_ms_ / 4000), 80};
    return true;
}

bool DummyTelemetrySource::servo_outputs(ServoOutputSample &out)
{
    out = ServoOutputSample();
    out.count = 6;
    for (int i = 0; i < out.count; i++)
        out.us[i] = servo_us_[i];
    return true;
}

bool DummyTelemetrySource::rc_inputs(RcInputSample &out)
{
    out = RcInputSample();
    out.count = 8;
    for (int i = 0; i < out.count; i++)
        out.us[i] = 1500;
    out.us[0] = (uint16_t)(1500 + 400 * sinf(phase(now_ms_, 4.0f))); // roll stick waving
    out.us[2] = 1000;                                                  // throttle low
    out.link_quality_pct = 100;
    return true;
}
