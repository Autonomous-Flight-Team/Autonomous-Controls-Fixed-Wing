#include "fault_manager.h"

// Rules: fault_table.h and "GOOSE State Machines.drawio" (page 3).

namespace {

// Faults with a time- or count-based escalation trigger in FaultManager::escalation_due().
constexpr bool has_escalation_trigger(FaultId id) {
    return id == FaultId::kRcLost || id == FaultId::kGpsLost || id == FaultId::kLoopOverrun;
}

// The table and the triggers must agree: a fault escalates iff it has a trigger.
constexpr bool escalation_rows_match_triggers() {
    for (size_t i = 0; i < kFaultCount; ++i) {
        const FaultDef& d = kFaultTable[i];
        if ((d.escalates_to != FaultClass::kNone) != has_escalation_trigger(d.id))
            return false;
        if (d.escalates_to != FaultClass::kNone && d.escalates_to <= d.cls)
            return false; // escalation must raise the class
    }
    return true;
}
static_assert(escalation_rows_match_triggers(),
              "kFaultTable escalates_to must match FaultManager escalation triggers");

// Raised on the first loop the raw bit is seen, not after debounce_ms:
//   LOOP_OVERRUN: each miss is a one-loop event; the streak decides escalation.
//   IN_AIR_RESET: set once at boot.
bool raises_immediately(FaultId id) {
    return id == FaultId::kLoopOverrun || id == FaultId::kInAirReset;
}

// Stays active after its raw bit clears, until fault_clear_cmd (operator acknowledgement).
bool is_latched(FaultId id) { return id == FaultId::kInAirReset; }

} // namespace

FaultClass FaultManager::effective_class(FaultId id) const {
    if (!is_active(id))
        return FaultClass::kNone;
    const FaultDef& d = fault_def(id);
    return is_escalated(id) ? d.escalates_to : d.cls;
}

void FaultManager::update(const FaultManagerInputs& in, uint32_t now_ms) {
    m_raised = m_cleared = m_newly_escalated = 0;

    // Acknowledged in-air reset: clear it on the same loop the system FSM leaves IN_AIR_RESET.
    const FaultMask reset_bit = fault_bit(FaultId::kInAirReset);
    if (in.fault_clear_cmd && (m_active & reset_bit) && !(in.raw & reset_bit)) {
        m_active &= ~reset_bit;
        m_cleared |= reset_bit;
    }

    if (in.raw & fault_bit(FaultId::kLoopOverrun)) {
        if (m_overrun_streak < UINT8_MAX)
            ++m_overrun_streak;
    } else {
        m_overrun_streak = 0;
    }

    for (size_t i = 0; i < kFaultCount; ++i) {
        const FaultId id = static_cast<FaultId>(i);
        debounce(id, (in.raw & fault_bit(id)) != 0, now_ms);
    }

    // Escalation is sticky until the fault clears (debounce() drops it with the fault).
    for (size_t i = 0; i < kFaultCount; ++i) {
        const FaultId id = static_cast<FaultId>(i);
        if (is_active(id) && !is_escalated(id) && escalation_due(id, now_ms)) {
            m_escalated |= fault_bit(id);
            m_newly_escalated |= fault_bit(id);
        }
    }

    summarize();
}

void FaultManager::debounce(FaultId id, bool raw, uint32_t now_ms) {
    Slot& s = m_slots[static_cast<size_t>(id)];
    const FaultMask bit = fault_bit(id);

    if (raw != s.raw) {
        s.raw = raw;
        s.raw_edge_ms = now_ms;
    }
    const uint32_t held = now_ms - s.raw_edge_ms; // unsigned: safe across millis() wrap
    const uint32_t raise_ms = raises_immediately(id) ? 0 : m_params.debounce_ms;

    if (!(m_active & bit)) {
        if (raw && held >= raise_ms) {
            m_active |= bit;
            m_raised |= bit;
            s.active_ms = now_ms;
        }
    } else if (!raw && held >= m_params.debounce_ms && !is_latched(id)) {
        m_active &= ~bit;
        m_escalated &= ~bit;
        m_cleared |= bit;
    }
}

bool FaultManager::escalation_due(FaultId id, uint32_t now_ms) const {
    // Timers count from when the fault was raised (after debounce).
    const uint32_t t_active = now_ms - m_slots[static_cast<size_t>(id)].active_ms;
    switch (id) {
    case FaultId::kRcLost:
        return m_params.rc_required && t_active >= m_params.t_rc_ms;
    case FaultId::kGpsLost:
        return t_active >= m_params.t_loiter_ms;
    case FaultId::kLoopOverrun:
        return m_overrun_streak >= m_params.n_overrun_max;
    default:
        return false; // has_escalation_trigger() and the static_assert keep this in sync
    }
}

void FaultManager::summarize() {
    m_highest = FaultClass::kNone;
    m_recoverable = true;
    m_response = FaultResponse::kNone;

    for (size_t i = 0; i < kFaultCount; ++i) {
        const FaultId id = static_cast<FaultId>(i);
        const FaultClass cls = effective_class(id);
        if (cls == FaultClass::kNone)
            continue;
        if (cls > m_highest)
            m_highest = cls;
        if (cls < FaultClass::kCritical)
            continue; // non-critical faults have no flight effect

        // One non-recoverable critical fault makes the whole episode non-recoverable.
        const FaultDef& d = fault_def(id);
        m_recoverable = m_recoverable && d.recoverable;
        // Most drastic response wins (FaultResponse is ordered): RTL beats FAILSAFE_LOITER,
        // so a battery-low plane goes home once GPS returns instead of resuming the mission.
        if (d.response > m_response)
            m_response = d.response;
    }
}
