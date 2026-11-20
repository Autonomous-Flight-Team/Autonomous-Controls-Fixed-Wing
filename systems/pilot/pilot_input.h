#ifndef PILOT_INPUT_H_
#define PILOT_INPUT_H_

#include <cstdint>

// The boundary between comms and manual controls: plain data, owned here,
// filled by comms (rc_link). Manual controls never parses a link itself.

// Values are logged: never renumber, only add.
enum class InputSource : uint8_t {
    kRc = 0,  // ER8 over CRSF
    kGcs = 1, // MAVLink MANUAL_CONTROL (reserved, unused)
};

// Bit positions in PilotInput::switches. Fixed by the TX16S setup.
enum class PilotSwitch : uint8_t {
    kBypass = 0,   // CH7: servo mux on the pilot (SA)
    kAutonomy = 1, // SB: autonomy level (reserved)
};

constexpr uint8_t kMaxRcChannels = 16;

// One frame of pilot input, already calibrated by the producer.
struct PilotInput {
    InputSource source = InputSource::kRc;
    float roll = 0.0f;     // [-1, 1], +right wing down
    float pitch = 0.0f;    // [-1, 1], +nose up
    float yaw = 0.0f;      // [-1, 1], +nose right
    float throttle = 0.0f; // [0, 1]
    float flaps = 0.0f;    // [0, 1]
    uint16_t switches = 0; // bit = PilotSwitch value
    // Receiver channels in µs, for mirroring. rc_link converts CRSF ticks
    // with crsf_ticks_to_us() (pilot.h), identical to ExpressLRS CRSF_to_US.
    uint16_t raw_us[kMaxRcChannels] = {};
    uint8_t num_raw = 0;
    uint32_t stamp_ms = 0; // when the producer received the frame
};

// Link facts from rc_link. No "failsafe" flag: an ELRS receiver just stops
// sending frames when it loses the transmitter. pilot/ decides kRcLost.
struct RcLinkStatus {
    bool frames_seen = false;
    uint8_t link_quality = 0; // CRSF LQ, %
    int8_t rssi_dbm = 0;
    uint32_t last_frame_ms = 0;
    uint32_t bad_frames = 0;
};

struct PilotParams {
    uint32_t rc_timeout_ms = 100; // no frame this long -> raw kRcLost
    uint8_t min_link_quality = 0; // LQ at or below this -> raw kRcLost
};

#endif // PILOT_INPUT_H_
