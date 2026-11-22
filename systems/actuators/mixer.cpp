#include "mixer.h"

void mix(const MixerConfig& cfg, const SurfaceDemand& d, float* out) {
    for (uint8_t i = 0; i < cfg.num_rows; ++i) {
        const MixerRow& r = cfg.rows[i];
        out[i] = r.roll * d.roll + r.pitch * d.pitch + r.yaw * d.yaw +
                 r.throttle * d.throttle + r.flaps * d.flaps;
    }
}

MixerConfig conventional_mixer() {
    MixerConfig m;
    m.num_rows = 7;
    m.rows[0].roll = 1.0f;     // ail_l: +roll = right wing down = left up
    m.rows[1].roll = -1.0f;    // ail_r
    m.rows[2].pitch = 1.0f;    // elev
    m.rows[3].yaw = 1.0f;      // rud
    m.rows[4].throttle = 1.0f; // thr
    m.rows[5].flaps = 1.0f;    // flap_l
    m.rows[6].flaps = 1.0f;    // flap_r (mirror with ChannelConfig::reversed)
    return m;
}
