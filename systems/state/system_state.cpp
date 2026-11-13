#include "system_state.h"

// Transition rules: "GOOSE State Machines.drawio" (page 1).

namespace
{

    bool is_armed_state(SystemState s)
    {
        switch (s)
        {
        case SystemState::kActive:
        case SystemState::kDegraded:
        case SystemState::kCritical:
        case SystemState::kEmergency:
            return true;
        case SystemState::kBoot:
        case SystemState::kPreflight:
        case SystemState::kStandby:
        case SystemState::kFaultGround:
        case SystemState::kBench:
        case SystemState::kInAirReset:
            return false;
        }
        return false;
    }

} // namespace

const char *to_string(SystemState s)
{
    switch (s)
    {
    case SystemState::kBoot:
        return "BOOT";
    case SystemState::kPreflight:
        return "PREFLIGHT";
    case SystemState::kStandby:
        return "STANDBY";
    case SystemState::kFaultGround:
        return "FAULT_GROUND";
    case SystemState::kBench:
        return "BENCH";
    case SystemState::kActive:
        return "ACTIVE";
    case SystemState::kDegraded:
        return "DEGRADED";
    case SystemState::kCritical:
        return "CRITICAL";
    case SystemState::kEmergency:
        return "EMERGENCY";
    case SystemState::kInAirReset:
        return "IN_AIR_RESET";
    }
    return "UNKNOWN";
}

bool SystemStateFSM::is_armed() const
{
    return is_armed_state(m_state);
}

void SystemStateFSM::transition(SystemState to, const char *reason, uint32_t now_ms)
{
    if (to == m_state)
        return;

    const SystemState from = m_state;

    // Exit actions
    if (from == SystemState::kFaultGround)
        m_fault_latched = false;
    if (is_armed_state(from) && !is_armed_state(to))
        m_crit_entries = 0; // new flight on next arm

    // Entry actions
    if (to == SystemState::kCritical)
    {
        if (m_crit_entries < UINT8_MAX)
            ++m_crit_entries;
        m_fault_clear_timing = false;
    }

    m_last_transition.from = from;
    m_last_transition.to = to;
    m_last_transition.reason = reason;
    m_last_transition.time_ms = now_ms;

    m_state = to;
    m_state_entry_ms = now_ms;
    m_changed = true;
}

void SystemStateFSM::update(const SystemInputs &in, uint32_t now_ms)
{
    m_changed = false;

    const FaultClass f = in.highest_fault;
    const uint32_t t_in_state = now_ms - m_state_entry_ms; // unsigned: safe across millis() wrap

    // Disarm from any armed state. Only honored on the ground: airborne disarm is ignored.
    // Leaving CRITICAL/EMERGENCY, or disarming with a critical fault, latches FAULT_GROUND.
    auto try_disarm = [&]() -> bool
    {
        if (!in.disarm_cmd || !in.landed)
            return false;

        if (m_state == SystemState::kCritical || m_state == SystemState::kEmergency ||
            f >= FaultClass::kCritical)
        {
            m_fault_latched = true;
            transition(SystemState::kFaultGround, "disarmed after critical fault", now_ms);
        }
        else
        {
            transition(SystemState::kStandby, "disarmed", now_ms);
        }
        return true;
    };

    // ACTIVE/DEGRADED: move to the state matching the highest active fault.
    auto follow_fault = [&]()
    {
        if (f == FaultClass::kEmergency)
        {
            transition(SystemState::kEmergency, "emergency fault", now_ms);
        }
        else if (f == FaultClass::kCritical)
        {
            m_crit_recoverable = in.fault_recoverable;
            transition(SystemState::kCritical, "critical fault", now_ms);
        }
        else if (f == FaultClass::kNonCritical)
        {
            transition(SystemState::kDegraded, "non-critical fault", now_ms);
        }
        else
        {
            transition(SystemState::kActive, "faults cleared", now_ms);
        }
    };

    switch (m_state)
    {
    case SystemState::kBoot:
        if (in.airborne_at_boot)
        {
            transition(SystemState::kInAirReset, "rebooted airborne", now_ms);
        }
        else if (in.boot_finished)
        {
            transition(SystemState::kPreflight, "boot finished", now_ms);
        }
        else if (t_in_state > m_params.t_boot_max_ms)
        {
            m_fault_latched = true;
            transition(SystemState::kFaultGround, "boot timeout", now_ms);
        }
        break;

    case SystemState::kPreflight:
        if (f >= FaultClass::kCritical)
        {
            m_fault_latched = false; // clears on its own, then re-checks
            transition(SystemState::kFaultGround, "preflight fault", now_ms);
        }
        else if (in.preflight_ok)
        {
            transition(SystemState::kStandby, "preflight passed", now_ms);
        }
        break;

    case SystemState::kStandby:
        if (f >= FaultClass::kCritical || !in.preflight_ok)
        {
            transition(SystemState::kPreflight, "health re-check", now_ms);
        }
        else if (in.bench_cmd)
        {
            transition(SystemState::kBench, "bench mode", now_ms);
        }
        else if (in.arm_cmd)
        {
            // A non-critical fault doesn't block arming, but it starts in DEGRADED.
            if (f == FaultClass::kNonCritical)
                transition(SystemState::kDegraded, "armed with non-critical fault", now_ms);
            else
                transition(SystemState::kActive, "armed", now_ms);
        }
        break;

    case SystemState::kFaultGround:
        if (f >= FaultClass::kCritical)
            break; // still faulted
        if (!m_fault_latched)
            transition(SystemState::kPreflight, "fault cleared", now_ms);
        else if (in.fault_clear_cmd)
            transition(SystemState::kPreflight, "fault acknowledged", now_ms);
        break;

    case SystemState::kBench:
        // Back through PREFLIGHT: bench tests may have moved surfaces or spun the motor.
        if (in.exit_bench_cmd)
            transition(SystemState::kPreflight, "bench exit", now_ms);
        break;

    case SystemState::kActive:
    case SystemState::kDegraded:
        if (try_disarm())
            break;
        follow_fault(); // no-op if already in the matching state
        break;

    case SystemState::kCritical:
        if (try_disarm())
            break;
        if (f == FaultClass::kEmergency)
        {
            transition(SystemState::kEmergency, "emergency fault", now_ms);
            break;
        }
        if (in.failsafe_failed)
        {
            transition(SystemState::kEmergency, "failsafe can't hold", now_ms);
            break;
        }
        if (f == FaultClass::kCritical)
        {
            // Still active: any non-recoverable fault this episode latches CRITICAL.
            m_crit_recoverable = m_crit_recoverable && in.fault_recoverable;
            m_fault_clear_timing = false;
            break;
        }

        // Critical fault has cleared. Recover only if recoverable and under N_crit_max;
        // otherwise CRITICAL is latched until landing and disarm.
        if (!m_crit_recoverable || m_crit_entries >= m_params.n_crit_max)
            break;
        if (!m_fault_clear_timing)
        {
            m_fault_clear_timing = true;
            m_fault_clear_ms = now_ms;
        }
        else if (now_ms - m_fault_clear_ms >= m_params.t_hold_ms)
        {
            transition(SystemState::kDegraded, "critical fault cleared for t_hold", now_ms);
        }
        break;

    case SystemState::kEmergency:
        // Only way out is landing and disarming.
        try_disarm();
        break;

    case SystemState::kInAirReset:
        // Disarmed, pilot flying in MANUAL. The flight FSM restarts in ON_GROUND after a
        // reboot, so `landed` can't be trusted here: leave only on operator acknowledgement.
        if (in.fault_clear_cmd)
            transition(SystemState::kPreflight, "in-air reset acknowledged", now_ms);
        break;
    }
}