#ifndef SYSTEM_STATE_H_
#define SYSTEM_STATE_H_

#include <cstdint>
#include "fault_table.h"

// Values are written to logs and telemetry: never renumber, only add.
enum class SystemState : uint8_t
{
    // On ground, disarmed
    kBoot = 0, // must stay 0: default-initialized state is safe
    kPreflight = 1,
    kStandby = 2,
    kFaultGround = 3,
    kBench = 4,

    // Armed
    kActive = 10,
    kDegraded = 11,
    kCritical = 12,
    kEmergency = 13,

    // Rebooted in flight (disarmed, pilot in MANUAL)
    kInAirReset = 20,
};

const char *to_string(SystemState s); // for logs and serial debug

// Tunable values (page 3 of the drawio). Same for every flight.
struct SystemParams
{
    uint32_t t_hold_ms = 5000;      // fault cleared this long before CRITICAL can recover
    uint32_t t_boot_max_ms = 30000; // BOOT taking longer than this is a fault
    uint8_t n_crit_max = 3;         // CRITICAL entries per flight before it latches
};

// Everything the system FSM reads each loop. Filled in by the main loop.
// Commands are one-loop pulses: the main loop edge-detects RC switches and GCS messages.
struct SystemInputs
{
    // Bring-up and checks
    bool boot_finished = false;    // all bring-up sub-FSMs report online
    bool airborne_at_boot = false; // in-air reset evidence (only read in BOOT)
    bool preflight_ok = false;     // every PREFLIGHT checklist item passes

    // Faults (time-based escalation, e.g. RC after t_rc, is done by the fault manager)
    FaultClass highest_fault = FaultClass::kNone;
    bool fault_recoverable = false; // the active CRITICAL fault can recover
    bool failsafe_failed = false;   // the failsafe can't hold

    // From the flight FSM
    bool landed = false; // flight FSM is ON_GROUND or LANDED

    // Commands (GCS or RC switch)
    bool arm_cmd = false;
    bool disarm_cmd = false;
    bool bench_cmd = false;
    bool exit_bench_cmd = false;
    bool fault_clear_cmd = false; // acknowledge a latched FAULT_GROUND
};

// One state change, for logging.
struct SystemTransition
{
    SystemState from = SystemState::kBoot;
    SystemState to = SystemState::kBoot;
    const char *reason = ""; // string literal, never freed
    uint32_t time_ms = 0;
};

class SystemStateFSM
{
public:
    explicit SystemStateFSM(const SystemParams &params = SystemParams{})
        : m_params(params) {}

    // One state object per system: no copies.
    SystemStateFSM(const SystemStateFSM &) = delete;
    SystemStateFSM &operator=(const SystemStateFSM &) = delete;

    // Call once per control loop.
    void update(const SystemInputs &in, uint32_t now_ms);

    SystemState state() const { return m_state; }
    bool is_armed() const;
    uint32_t state_entry_ms() const { return m_state_entry_ms; }

    // True on the loop where the state changed; last_transition() has the details.
    bool changed() const { return m_changed; }
    const SystemTransition &last_transition() const { return m_last_transition; }

private:
    // The only way to change m_state.
    void transition(SystemState to, const char *reason, uint32_t now_ms);

    SystemParams m_params;

    SystemState m_state = SystemState::kBoot;
    uint32_t m_state_entry_ms = 0; // when the current state was entered

    uint32_t m_fault_clear_ms = 0;     // when the CRITICAL fault last cleared (for t_hold)
    bool m_fault_clear_timing = false; // m_fault_clear_ms is valid
    bool m_crit_recoverable = false;   // every critical fault this CRITICAL episode was recoverable
    uint8_t m_crit_entries = 0;        // CRITICAL entries this flight; reset on disarm
    bool m_fault_latched = false;      // FAULT_GROUND needs fault_clear_cmd to leave

    bool m_changed = false;
    SystemTransition m_last_transition{};
};

#endif // SYSTEM_STATE_H_