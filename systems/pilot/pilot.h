#ifndef PILOT_H_
#define PILOT_H_

#include "pilot_input.h"
#include <cstdint>

// Pure helpers between the comms boundary and the rest of the system.
// Faults: only the raw kRcLost bit comes from here; FaultManager debounces
// (debounce_ms), escalates (t_rc) and the FSMs respond.

// CRSF 11-bit ticks -> µs (172 -> 988, 992 -> 1500, 1811 -> 2012).
// For rc_link and its tests; kept here so both sides use one formula.
uint16_t crsf_ticks_to_us(uint16_t ticks);

// Raw FaultId::kRcLost condition for this loop.
bool rc_lost_raw(const RcLinkStatus& link, const PilotParams& p,
                 uint32_t now_ms);

// The frame is recent enough to use (same timeout as rc_lost_raw).
bool pilot_fresh(const PilotInput& in, const PilotParams& p, uint32_t now_ms);

bool switch_on(const PilotInput& in, PilotSwitch sw);

// FlightInputs::manual_active, set by the main loop every loop.
// From the bypass switch while the frame is fresh. A stale frame reads as
// not manual: the ER8 failsafe puts CH7 on the Teensy side. Replace with a
// SEL sense input if one is added.
bool manual_active(const PilotInput& in, bool fresh);

// Receiver µs for one RC channel, for mirroring. False if the frame is
// stale or the channel isn't in it.
bool mirror_us(const PilotInput& in, bool fresh, int8_t rc_channel,
               uint16_t* out_us);

// ---- For comms telemetry (RcInputSample -> RC_CHANNELS) ----

constexpr uint8_t kLinkQualityUnknown = 255;

// 0-100 once frames have been seen, kLinkQualityUnknown before.
uint8_t rc_link_quality_pct(const RcLinkStatus& link);

// Copies the receiver µs for telemetry. Returns the count: 0 if the frame
// is stale, so the GCS shows no RC rather than the last good sticks.
uint8_t rc_telemetry_us(const PilotInput& in, bool fresh, uint16_t* us,
                        uint8_t cap);

#endif // PILOT_H_
