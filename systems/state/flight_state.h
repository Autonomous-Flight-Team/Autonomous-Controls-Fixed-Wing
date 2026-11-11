#ifndef FLIGHT_STATE_H_
#define FLIGHT_STATE_H_

#include <cstdint>
#include "fault_table.h"
#include "system_state.h"

// Transition rules: flight_state.cpp and "GOOSE State Machines.drawio" (page 2).
// Values are written to logs and telemetry: never renumber, only add.
enum class FlightState : uint8_t
{
    // Ground
    kOnGround = 0,    // motor locked unless system is ACTIVE; must stay 0
    kTakeoffRoll = 1, // full throttle, track centerline
    kRotate = 2,      // past V1, committed; pitch to takeoff attitude
    kLanded = 3,      // motor cut, rollout; "landed" signal for the system FSM

    // Airborne (AIRBORNE overrides apply)
    kClimb = 10,   // full power, pitch for Vy
    kCruise = 11,  // fly the mission
    kLoiter = 12,  // commanded GPS orbit around a point
    kRtl = 13,     // fly to home (captured at arm)
    kDescent = 14, // descend to pattern altitude
    kLanding = 15, // final, flare, touchdown

    // Aborts and failsafes
    kRejectedTakeoff = 20, // motor cut, wings level, roll out
    kFailsafeLoiter = 21,  // GPS lost: attitude-hold circle
    kEmergencyLand = 22,   // best reachable site, straight ahead if low
};

const char* to_string(FlightState s);

// What happens when the mission ends.
enum class EndAction : uint8_t
{
    kRtl = 0,
    kLoiter = 1,
};

// Tunable values (page 3 of the drawio). Same for every flight.
// PLACEHOLDER airframe values: set from Ranger 1800 ground and flight tests.
struct FlightParams
{
    // _m = meter, _ms = milliseconds, _mps - meters/second
    float v1_mps = 12.0f;             // PLACEHOLDER: committed-to-takeoff airspeed
    float h_min_agl_m = 5.0f;         // PLACEHOLDER: AGL (Above Ground Level) that confirms liftoff
    float climb_rate_min_mps = 0.0f;  // climb rate that confirms liftoff
    float v_stopped_mps = 0.5f;       // groundspeed counted as "stopped"
    uint32_t t_takeoff_max_ms = 8000; // PLACEHOLDER: ROTATE without liftoff -> rejected takeoff
    uint8_t n_goaround_max = 2;       // go-arounds before the landing is committed
    EndAction end_action = EndAction::kRtl;
};

// Everything the flight FSM reads each loop. Filled in by the main loop.
struct FlightInputs
{
    // From the system FSM and fault manager
    SystemState system_state = SystemState::kBoot;
    FaultResponse critical_response = FaultResponse::kNone; // response of the active CRITICAL/EMERGENCY fault
    bool gps_ok = true;   // GPS usable (FAILSAFE_LOITER recovery)
    bool motor_ok = true; // powered go-around possible

    // Measurements (from the estimator)
    float airspeed_mps = 0.0f;
    float groundspeed_mps = 0.0f;
    float agl_m = 0.0f;
    float climb_rate_mps = 0.0f;

    // From guidance / mission
    bool mission_valid = false;
    bool mission_done = false;
    bool target_alt_reached = false;
    bool over_landing_site = false;
    bool on_final = false;           // on final, aligned
    bool touchdown = false;
    bool go_around_needed = false;   // unstable approach or runway blocked
    bool loiter_done = false;        // loiter time or mission item finished

    // Commands (GCS or RC switch)
    bool takeoff_cmd = false;
    bool loiter_cmd = false;
    bool resume_cmd = false;
    bool rtl_cmd = false;

    // Hardware bypass
    bool manual_active = false; // servo mux is on the pilot (MANUAL)
};

// One state change, for logging.
struct FlightTransition
{
    FlightState from = FlightState::kOnGround;
    FlightState to = FlightState::kOnGround;
    const char *reason = ""; // string literal, never freed
    uint32_t time_ms = 0;
};

class FlightStateFSM
{
public:
    explicit FlightStateFSM(const FlightParams &params = FlightParams{})
        : m_params(params) {}

    // One state object per system: no copies.
    FlightStateFSM(const FlightStateFSM &) = delete;
    FlightStateFSM &operator=(const FlightStateFSM &) = delete;

    // Call once per control loop, after the system FSM's update().
    void update(const FlightInputs &in, uint32_t now_ms);

    FlightState state() const { return m_state; }
    bool is_airborne() const; // inside the AIRBORNE group (AIRBORNE overrides apply)
    bool is_landed() const;   // ON_GROUND or LANDED: SystemInputs::landed
    uint32_t state_entry_ms() const { return m_state_entry_ms; }

    // True on the loop where the state changed; last_transition() has the details.
    bool changed() const { return m_changed; }
    const FlightTransition &last_transition() const { return m_last_transition; }

private:
    // The only way to change m_state.
    void transition(FlightState to, const char *reason, uint32_t now_ms);

    FlightParams m_params;

    FlightState m_state = FlightState::kOnGround;
    uint32_t m_state_entry_ms = 0; // when the current state was entered (t_takeoff_max)

    uint8_t m_goaround_count = 0;  // go-arounds this landing (n_goaround_max)
    bool m_rtl_latched = false;    // RTL stays RTL after the fault clears, until resume_cmd
    bool m_prev_manual = false;    // manual_active last loop: detects pilot re-engaging

    bool m_changed = false;
    FlightTransition m_last_transition{};
};

#endif // FLIGHT_STATE_H_