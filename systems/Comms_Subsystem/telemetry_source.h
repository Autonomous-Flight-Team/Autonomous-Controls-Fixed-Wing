#pragma once
// What comms needs from the rest of the autopilot to fill the downlink. The state
// machine / estimator owners implement TelemetrySource; comms only reads from it.
// Each getter returns false when that data is not available (the message is skipped).
// Units follow the MAVLink message the data ends up in.

#include <stdint.h>

#define GOOSE_NUM_SENSORS 8
#define GOOSE_MAX_SERVOS 16      // SERVO_OUTPUT_RAW carries 16
#define GOOSE_MAX_RC_CHANNELS 18 // RC_CHANNELS carries 18

struct GooseStateSnapshot // -> GOOSE_STATE and HEARTBEAT
{
    uint8_t system_state = 0;    // GOOSE_SYSTEM_STATE (= SystemState)
    uint8_t flight_state = 0;    // GOOSE_FLIGHT_STATE (= FlightState)
    uint32_t active_faults = 0;  // GOOSE_FAULT bits (= FaultMask, bit N = FaultId N)
    uint8_t critical_entries = 0;
    uint8_t flags = 0;           // GOOSE_STATE_FLAGS (ARMED, MANUAL_ACTIVE)
    uint8_t sensor_status[GOOSE_NUM_SENSORS] = {}; // sensors::sensor_status values, GOOSE_SENSOR_INDEX order
};

// True for the armed system states: ACTIVE, DEGRADED, CRITICAL, EMERGENCY. Same set as
// is_armed(SystemState); IN_AIR_RESET (20) is disarmed, so never compare with >=.
bool goose_system_armed(uint8_t system_state);

struct AttitudeSample // -> ATTITUDE
{
    float roll, pitch, yaw;                // rad
    float rollspeed, pitchspeed, yawspeed; // rad/s
};

struct PositionSample // -> GLOBAL_POSITION_INT
{
    int32_t lat, lon;          // degE7
    int32_t alt_mm;            // MSL
    int32_t relative_alt_mm;   // above home
    int16_t vx, vy, vz;        // cm/s, NED
    uint16_t hdg_cdeg;         // 0..35999, UINT16_MAX if unknown
};

struct GpsSample // -> GPS_RAW_INT
{
    uint8_t fix_type;          // GPS_FIX_TYPE
    int32_t lat, lon, alt_mm;
    uint16_t eph, epv;         // HDOP/VDOP * 100, UINT16_MAX if unknown
    uint16_t vel_cms, cog_cdeg;
    uint8_t satellites_visible;
};

struct AirDataSample // -> VFR_HUD
{
    float airspeed, groundspeed; // m/s
    float alt, climb;            // m MSL, m/s
    int16_t heading_deg;
    uint16_t throttle_pct;
};

struct ImuSample // -> SCALED_IMU
{
    int16_t xacc, yacc, zacc;    // mG
    int16_t xgyro, ygyro, zgyro; // mrad/s
    int16_t xmag, ymag, zmag;    // mgauss
    int16_t temperature_cdeg;
};

struct PressureSample // -> SCALED_PRESSURE
{
    float press_abs_hpa, press_diff_hpa;
    int16_t temperature_cdeg;
};

struct ServoOutputSample // -> SERVO_OUTPUT_RAW
{
    uint16_t us[GOOSE_MAX_SERVOS]; // pulse width actually commanded, us; entries >= count are sent as 0
    uint8_t count;                 // outputs in use
};

struct RcInputSample // -> RC_CHANNELS
{
    uint16_t us[GOOSE_MAX_RC_CHANNELS]; // pilot channels in us (CRSF ticks already converted)
    uint8_t count;                      // channels received; 0 = no RC link
    uint8_t link_quality_pct;           // CRSF LQ 0-100, or 255 if unknown
};

struct BatterySample // -> BATTERY_STATUS and SYS_STATUS
{
    uint16_t voltage_mv;
    int16_t current_ca;     // 10 mA units, -1 if unknown
    int32_t consumed_mah;   // -1 if unknown
    int8_t remaining_pct;   // -1 if unknown
};

class TelemetrySource
{
public:
    virtual ~TelemetrySource() = default;
    virtual bool goose_state(GooseStateSnapshot &out) = 0;
    virtual bool attitude(AttitudeSample &) { return false; }
    virtual bool position(PositionSample &) { return false; }
    virtual bool gps(GpsSample &) { return false; }
    virtual bool air_data(AirDataSample &) { return false; }
    virtual bool imu(ImuSample &) { return false; }
    virtual bool pressure(PressureSample &) { return false; }
    virtual bool battery(BatterySample &) { return false; }
    virtual bool servo_outputs(ServoOutputSample &) { return false; }
    virtual bool rc_inputs(RcInputSample &) { return false; }
    virtual uint16_t cpu_load_permille() { return 0; } // SYS_STATUS.load
};

// Fake data for bench tests before the real subsystems exist. Its state is public so
// StubCommandHandler can move it through the state machine.
class DummyTelemetrySource : public TelemetrySource
{
public:
    DummyTelemetrySource();
    void update(uint32_t now_ms); // advance the fake signals and the fake flight
    GooseStateSnapshot &state() { return state_; }

    bool goose_state(GooseStateSnapshot &out) override;
    bool attitude(AttitudeSample &out) override;
    bool position(PositionSample &out) override;
    bool gps(GpsSample &out) override;
    bool air_data(AirDataSample &out) override;
    bool imu(ImuSample &out) override;
    bool pressure(PressureSample &out) override;
    bool battery(BatterySample &out) override;
    bool servo_outputs(ServoOutputSample &out) override;
    bool rc_inputs(RcInputSample &out) override;

    // Fake servo outputs; StubCommandHandler::set_servo writes here in BENCH.
    uint16_t *servo_us() { return servo_us_; }

private:
    GooseStateSnapshot state_;
    uint16_t servo_us_[GOOSE_MAX_SERVOS] = {1500, 1500, 1500, 1500, 1000, 1500}; // 6 outputs, ch5 throttle
    uint32_t now_ms_ = 0;
    uint8_t last_flight_state_ = 0, last_system_state_ = 0;
    uint32_t flight_since_ms_ = 0, system_since_ms_ = 0;
};
