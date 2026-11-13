#include "flight_state.h"

// Transition rules: "GOOSE State Machines.drawio" (page 2).

namespace
{

    // Armed and healthy enough to start or continue a takeoff roll.
    bool takeoff_allowed(SystemState s)
    {
        return s == SystemState::kActive || s == SystemState::kDegraded;
    }

} // namespace

const char *to_string(FlightState s)
{
    switch (s)
    {
    case FlightState::kOnGround:
        return "ON_GROUND";
    case FlightState::kTakeoffRoll:
        return "TAKEOFF_ROLL";
    case FlightState::kRotate:
        return "ROTATE";
    case FlightState::kLanded:
        return "LANDED";
    case FlightState::kClimb:
        return "CLIMB";
    case FlightState::kCruise:
        return "CRUISE";
    case FlightState::kLoiter:
        return "LOITER";
    case FlightState::kRtl:
        return "RTL";
    case FlightState::kDescent:
        return "DESCENT";
    case FlightState::kLanding:
        return "LANDING";
    case FlightState::kRejectedTakeoff:
        return "REJECTED_TAKEOFF";
    case FlightState::kFailsafeLoiter:
        return "FAILSAFE_LOITER";
    case FlightState::kEmergencyLand:
        return "EMERGENCY_LAND";
    }
    return "UNKNOWN";
}

bool FlightStateFSM::is_airborne() const
{
    switch (m_state)
    {
    case FlightState::kClimb:
    case FlightState::kCruise:
    case FlightState::kLoiter:
    case FlightState::kRtl:
    case FlightState::kDescent:
    case FlightState::kLanding:
    case FlightState::kFailsafeLoiter:
    case FlightState::kEmergencyLand:
        return true;
    case FlightState::kOnGround:
    case FlightState::kTakeoffRoll:
    case FlightState::kRotate:
    case FlightState::kLanded:
    case FlightState::kRejectedTakeoff:
        return false;
    }
    return false;
}

bool FlightStateFSM::is_landed() const
{
    // On the ground below V1: the system FSM may disarm. TAKEOFF_ROLL is included so a
    // disarm aborts the roll; ROTATE is not (committed past V1).
    switch (m_state)
    {
    case FlightState::kOnGround:
    case FlightState::kTakeoffRoll:
    case FlightState::kRejectedTakeoff:
    case FlightState::kLanded:
        return true;
    case FlightState::kRotate:
    case FlightState::kClimb:
    case FlightState::kCruise:
    case FlightState::kLoiter:
    case FlightState::kRtl:
    case FlightState::kDescent:
    case FlightState::kLanding:
    case FlightState::kFailsafeLoiter:
    case FlightState::kEmergencyLand:
        return false;
    }
    return false;
}

void FlightStateFSM::transition(FlightState to, const char *reason, uint32_t now_ms)
{
    if (to == m_state)
        return;

    const FlightState from = m_state;

    // Entry actions
    switch (to)
    {
    case FlightState::kTakeoffRoll:
        m_goaround_count = 0; // new flight
        m_rtl_latched = false;
        break;
    case FlightState::kRtl:
        m_rtl_latched = true; // RTL is now the intent, until resume_cmd
        break;
    case FlightState::kCruise:
        m_rtl_latched = false; // only reachable by resume or normal flight
        break;
    case FlightState::kClimb:
        if (from == FlightState::kLanding && m_goaround_count < UINT8_MAX)
            ++m_goaround_count;
        break;
    default:
        break;
    }

    m_last_transition.from = from;
    m_last_transition.to = to;
    m_last_transition.reason = reason;
    m_last_transition.time_ms = now_ms;

    m_state = to;
    m_state_entry_ms = now_ms;
    m_changed = true;
}

void FlightStateFSM::update(const FlightInputs &in, uint32_t now_ms)
{
    m_changed = false;

    const SystemState sys = in.system_state;
    const uint32_t t_in_state = now_ms - m_state_entry_ms; // unsigned: safe across millis() wrap

    // ---- 1. MANUAL (hardware bypass): the pilot is flying, so the FSM holds its state. ----
    if (in.manual_active)
    {
        m_prev_manual = true;
        return;
    }
    if (m_prev_manual)
    {
        // Pilot handed control back. The held state may be stale, so restart from what
        // the plane is actually doing: hold position if airborne, else back to ON_GROUND.
        m_prev_manual = false;
        const bool flying = in.agl_m >= m_params.h_min_agl_m || in.airspeed_mps >= m_params.v1_mps;
        if (!flying)
            transition(FlightState::kOnGround, "autopilot re-engaged on ground", now_ms);
        else if (in.gps_ok)
            transition(FlightState::kLoiter, "autopilot re-engaged airborne", now_ms);
        else
            transition(FlightState::kFailsafeLoiter, "autopilot re-engaged, no GPS", now_ms);
        return;
    }

    // ---- 2. AIRBORNE overrides: react to the system FSM before normal transitions. ----
    if (is_airborne())
    {
        if (sys == SystemState::kEmergency)
        {
            // Already on final: finishing the landing beats picking a new site.
            if (m_state != FlightState::kEmergencyLand && m_state != FlightState::kLanding)
                transition(FlightState::kEmergencyLand, "system EMERGENCY", now_ms);
        }
        else if (sys == SystemState::kCritical)
        {
            switch (in.critical_response)
            {
            case FaultResponse::kFailsafeLoiter:
                // GPS lost: can't navigate. Everywhere except short final.
                if (m_state == FlightState::kClimb || m_state == FlightState::kCruise ||
                    m_state == FlightState::kLoiter || m_state == FlightState::kRtl ||
                    m_state == FlightState::kDescent)
                    transition(FlightState::kFailsafeLoiter, "critical: GPS lost", now_ms);
                break;
            case FaultResponse::kRtl:
                // Already heading home (RTL, DESCENT, LANDING): nothing to change.
                if (m_state == FlightState::kClimb || m_state == FlightState::kCruise ||
                    m_state == FlightState::kLoiter ||
                    (m_state == FlightState::kFailsafeLoiter && in.gps_ok))
                    transition(FlightState::kRtl, "critical: RTL failsafe", now_ms);
                break;
            case FaultResponse::kEmergencyLand:
                // Emergency-class faults put the system in EMERGENCY; handled defensively.
                if (m_state != FlightState::kEmergencyLand && m_state != FlightState::kLanding)
                    transition(FlightState::kEmergencyLand, "critical: emergency land", now_ms);
                break;
            case FaultResponse::kNone:
                break;
            }
        }
        if (m_changed)
            return;
    }

    // ---- 3. Normal transitions. ----
    switch (m_state)
    {
    case FlightState::kOnGround:
        // Takeoff needs a healthy system (ACTIVE only) and a mission to fly.
        if (in.takeoff_cmd && sys == SystemState::kActive && in.mission_valid)
            transition(FlightState::kTakeoffRoll, "takeoff", now_ms);
        break;

    case FlightState::kTakeoffRoll:
        // Below V1 a disarm (pilot abort) or any CRITICAL/EMERGENCY aborts.
        if (!is_armed(sys))
            transition(FlightState::kRejectedTakeoff, "disarmed on takeoff roll", now_ms);
        else if (!takeoff_allowed(sys))
            transition(FlightState::kRejectedTakeoff, "system fault below V1", now_ms);
        else if (in.airspeed_mps >= m_params.v1_mps)
            transition(FlightState::kRotate, "V1 reached", now_ms);
        break;

    case FlightState::kRotate:
        // Committed: faults don't abort here. AIRBORNE overrides apply once in CLIMB.
        if (in.agl_m >= m_params.h_min_agl_m && in.climb_rate_mps > m_params.climb_rate_min_mps)
            transition(FlightState::kClimb, "liftoff confirmed", now_ms);
        else if (t_in_state > m_params.t_takeoff_max_ms)
            transition(FlightState::kRejectedTakeoff, "no liftoff by t_takeoff_max", now_ms);
        break;

    case FlightState::kRejectedTakeoff:
        if (in.groundspeed_mps < m_params.v_stopped_mps)
            transition(FlightState::kOnGround, "stopped", now_ms);
        break;

    case FlightState::kClimb:
        if (in.rtl_cmd)
            transition(FlightState::kRtl, "RTL commanded", now_ms);
        else if (in.target_alt_reached)
        {
            // After a go-around or an interrupted RTL, go back to the landing pattern.
            if (m_goaround_count > 0 || m_rtl_latched)
                transition(FlightState::kRtl, "climb done, back to pattern", now_ms);
            else
                transition(FlightState::kCruise, "climb done", now_ms);
        }
        break;

    case FlightState::kCruise:
        if (in.rtl_cmd)
            transition(FlightState::kRtl, "RTL commanded", now_ms);
        else if (in.loiter_cmd)
            transition(FlightState::kLoiter, "loiter commanded", now_ms);
        else if (!in.mission_valid)
            transition(FlightState::kLoiter, "mission invalid, holding", now_ms);
        else if (in.mission_done)
        {
            if (m_params.end_action == EndAction::kLoiter)
                transition(FlightState::kLoiter, "mission done: loiter", now_ms);
            else
                transition(FlightState::kRtl, "mission done: RTL", now_ms);
        }
        break;

    case FlightState::kLoiter:
        if (in.rtl_cmd)
            transition(FlightState::kRtl, "RTL commanded", now_ms);
        else if (in.mission_valid && !in.mission_done && (in.resume_cmd || in.loiter_done))
            transition(FlightState::kCruise, in.resume_cmd ? "resume" : "loiter done", now_ms);
        else if (in.loiter_done && in.mission_done)
            transition(FlightState::kRtl, "loiter done, mission over", now_ms);
        break;

    case FlightState::kRtl:
        // Sticky: leaves only for the landing pattern or a resume once the fault is gone.
        if (in.over_landing_site)
            transition(FlightState::kDescent, "over landing site", now_ms);
        else if (in.resume_cmd && sys != SystemState::kCritical && in.mission_valid)
            transition(FlightState::kCruise, "resume", now_ms);
        break;

    case FlightState::kDescent:
        if (in.on_final)
            transition(FlightState::kLanding, "on final", now_ms);
        break;

    case FlightState::kLanding:
        if (in.touchdown)
            transition(FlightState::kLanded, "touchdown", now_ms);
        else if (in.go_around_needed && in.motor_ok && sys != SystemState::kEmergency &&
                 m_goaround_count < m_params.n_goaround_max)
            transition(FlightState::kClimb, "go-around", now_ms);
        // Out of go-arounds (or no power): the landing is committed.
        break;

    case FlightState::kFailsafeLoiter:
        // Escalation (still lost after t_loiter) arrives as system EMERGENCY via the override.
        if (in.gps_ok)
        {
            if (m_rtl_latched)
                transition(FlightState::kRtl, "GPS recovered, back to RTL", now_ms);
            else
                transition(FlightState::kCruise, "GPS recovered", now_ms);
        }
        break;

    case FlightState::kEmergencyLand:
        if (in.touchdown)
            transition(FlightState::kLanded, "emergency touchdown", now_ms);
        break;

    case FlightState::kLanded:
        // Wait for the rollout to stop AND the system FSM to disarm.
        if (in.groundspeed_mps < m_params.v_stopped_mps && !is_armed(sys))
            transition(FlightState::kOnGround, "stopped and disarmed", now_ms);
        break;
    }
}