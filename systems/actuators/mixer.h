#ifndef MIXER_H_
#define MIXER_H_

#include "pwm_out.h"
#include "surface_demand.h"
#include <cstdint>

// Airframe mixing as a weight table: each output is a weighted sum of the
// demand axes. Conventional tail, V-tail and flying wing are just different
// tables, so a new airframe needs new data, not new code.
struct MixerRow {
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float throttle = 0.0f;
    float flaps = 0.0f;
};

struct MixerConfig {
    MixerRow rows[pwm_out::kMaxChannels]{};
    uint8_t num_rows = 0;
};

// out[i] = rows[i] . demand, unclamped (OutputChannel clamps per kind).
// Writes num_rows values; out must hold that many.
void mix(const MixerConfig& cfg, const SurfaceDemand& d, float* out);

// Example table for a conventional tail with split ailerons and flaps.
// Channel order: ail_l, ail_r, elev, rud, thr, flap_l, flap_r.
MixerConfig conventional_mixer();

#endif // MIXER_H_
