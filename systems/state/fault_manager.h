#ifndef GOOSE_STATE_FAULT_MANAGER_H_
#define GOOSE_STATE_FAULT_MANAGER_H_

#include "fault_table.h"
#include <cstdint>

// Turns raw fault bits from the health monitors into what the FSMs read:
// debounced active faults, the highest class, recoverability and the flight response.
// Pure, like the FSMs: reads only its inputs and now_ms, no heap, builds on native.
// Rules: fault_table.h and "GOOSE State Machines.drawio" (page 3).

// Tunable values (page 3 of the drawio).
struct FaultManagerParams {
    uint32_t debounce_ms = 200;   // raw bit must hold this long to raise, or stay clear to clear
    uint32_t t_rc_ms = 3000;      // RC_LOST active this long -> CRITICAL (if rc_required)
    uint32_t t_loiter_ms = 60000; // GPS_LOST active this long -> EMERGENCY
    uint8_t n_overrun_max = 5;    // LOOP_OVERRUN misses in a row -> EMERGENCY
    bool rc_required = true;      // false for autonomous missions: RC loss never escalates
};

// Everything the fault manager reads each loop. Filled in by the main loop.
struct FaultManagerInputs {
    FaultMask raw = 0;            // health monitors, this loop (set = fault condition present)
    bool fault_clear_cmd = false; // same one-loop pulse as SystemInputs::fault_clear_cmd
};

class FaultManager {
  public:
    explicit FaultManager(const FaultManagerParams& params = FaultManagerParams{})
        : m_params(params) {}

    // One fault manager per system: no copies.
    FaultManager(const FaultManager&) = delete;
    FaultManager& operator=(const FaultManager&) = delete;

    // Call once per control loop, before the system FSM's update().
    void update(const FaultManagerInputs& in, uint32_t now_ms);

    // ---- For the FSMs ----
    FaultClass highest_class() const { return m_highest; } // SystemInputs::highest_fault
    bool recoverable() const { return m_recoverable; }     // SystemInputs::fault_recoverable
    FaultResponse critical_response() const { return m_response; } // FlightInputs

    // ---- Per fault ----
    FaultMask active_mask() const { return m_active; }
    bool is_active(FaultId id) const { return (m_active & fault_bit(id)) != 0; }
    bool is_escalated(FaultId id) const { return (m_escalated & fault_bit(id)) != 0; }
    FaultClass effective_class(FaultId id) const; // kNone if not active

    // ---- For logging: what changed on this update() ----
    bool changed() const { return (m_raised | m_cleared | m_newly_escalated) != 0; }
    FaultMask raised_mask() const { return m_raised; }
    FaultMask cleared_mask() const { return m_cleared; }
    FaultMask escalated_mask() const { return m_newly_escalated; }

  private:
    struct Slot {
        bool raw = false;         // raw bit last loop
        uint32_t raw_edge_ms = 0; // when the raw bit last changed
        uint32_t active_ms = 0;   // when the fault was raised (escalation timers)
    };

    void debounce(FaultId id, bool raw, uint32_t now_ms);
    bool escalation_due(FaultId id, uint32_t now_ms) const;
    void summarize();

    FaultManagerParams m_params;
    Slot m_slots[kFaultCount]{};

    FaultMask m_active = 0;
    FaultMask m_escalated = 0;
    uint8_t m_overrun_streak = 0; // LOOP_OVERRUN raw bit set this many loops in a row

    FaultMask m_raised = 0;
    FaultMask m_cleared = 0;
    FaultMask m_newly_escalated = 0;

    FaultClass m_highest = FaultClass::kNone;
    bool m_recoverable = true;
    FaultResponse m_response = FaultResponse::kNone;
};

#endif // GOOSE_STATE_FAULT_MANAGER_H_
