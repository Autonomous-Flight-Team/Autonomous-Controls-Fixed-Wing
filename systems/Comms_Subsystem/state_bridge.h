#pragma once
// Bridge between comms and the real state machines (systems/state/). No Arduino code,
// so it builds and is tested on native.
//
//  - CommandHandler: checks the same guards the FSM code applies, so COMMAND_ACK is
//    honest, and on ACCEPTED latches a one-loop command pulse.
//  - TelemetrySource: GOOSE_STATE from the FSMs; other telemetry is passed through to an
//    optional sensor source.
//
// Main loop, once per loop (see systems/state/README.MD "Usage"):
//
//   fill si / fi from sensors, fault manager, guidance     (not the command fields)
//   bridge.set_fault_mask(active_faults);
//   bridge.apply(si, fi);          // ORs pending pulses into the command fields
//   sys.update(si, now);  fi.system_state = sys.state();  fl.update(fi, now);
//   clear all command fields in si / fi                     (state README step 6)
//   comms_update();                // may accept commands for the next loop
//
// A command accepted during comms_update() reaches the FSM on the next loop's apply().
// The guards use the inputs from the last apply(), so they are at most one loop old: if
// a fault appears in that loop the FSM can still ignore the command.
// Only one pending pulse per FSM at a time: a second system (or flight) command before
// the next apply() gets MAV_RESULT_TEMPORARILY_REJECTED.

#include "command_handler.h"
#include "flight_state.h"
#include "system_state.h"
#include "telemetry_source.h"

class StateBridge : public CommandHandler, public TelemetrySource
{
public:
    StateBridge(SystemStateFSM &sys, FlightStateFSM &flight, TelemetrySource *sensors = nullptr)
        : sys_(sys), fl_(flight), sensors_(sensors) {}

    void apply(SystemInputs &si, FlightInputs &fi);
    void set_fault_mask(FaultMask mask) { faults_ = mask; }
    void set_sensor_status(const uint8_t status[GOOSE_NUM_SENSORS]);

    // CommandHandler
    MAV_RESULT arm(bool arm, bool force) override;
    MAV_RESULT takeoff(float pitch_deg, float altitude_m) override;
    MAV_RESULT return_to_launch() override;
    MAV_RESULT loiter_unlimited(float radius_m, float lat_deg, float lon_deg, float alt_m) override;
    MAV_RESULT resume_mission() override;
    MAV_RESULT fault_clear() override;
    MAV_RESULT bench(bool enter) override;

    // TelemetrySource
    bool goose_state(GooseStateSnapshot &out) override;
    bool attitude(AttitudeSample &out) override { return sensors_ && sensors_->attitude(out); }
    bool position(PositionSample &out) override { return sensors_ && sensors_->position(out); }
    bool gps(GpsSample &out) override { return sensors_ && sensors_->gps(out); }
    bool air_data(AirDataSample &out) override { return sensors_ && sensors_->air_data(out); }
    bool imu(ImuSample &out) override { return sensors_ && sensors_->imu(out); }
    bool pressure(PressureSample &out) override { return sensors_ && sensors_->pressure(out); }
    bool battery(BatterySample &out) override { return sensors_ && sensors_->battery(out); }
    uint16_t cpu_load_permille() override { return sensors_ ? sensors_->cpu_load_permille() : 0; }

private:
    struct SystemPulses
    {
        bool arm, disarm, bench, exit_bench, fault_clear;
        bool any() const { return arm || disarm || bench || exit_bench || fault_clear; }
    };
    struct FlightPulses
    {
        bool takeoff, loiter, resume, rtl;
        bool any() const { return takeoff || loiter || resume || rtl; }
    };

    bool system_ready_for_ground_cmd() const; // STANDBY checks: preflight ok, no critical fault
    bool failsafe_override_active() const;    // flight FSM's AIRBORNE overrides will act first

    SystemStateFSM &sys_;
    FlightStateFSM &fl_;
    TelemetrySource *sensors_;
    SystemPulses sys_pending_{};
    FlightPulses fl_pending_{};
    SystemInputs seen_si_{}; // inputs as of the last apply()
    FlightInputs seen_fi_{};
    FaultMask faults_ = 0;
    uint8_t sensor_status_[GOOSE_NUM_SENSORS] = {};
};
