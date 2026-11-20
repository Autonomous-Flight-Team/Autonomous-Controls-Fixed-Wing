#include "pilot.h"

uint16_t crsf_ticks_to_us(uint16_t ticks) {
    // Same integer math as ExpressLRS CRSF_to_US() (fmap() in
    // src/include/crsf_protocol.h), which the ER8 uses for its PWM outputs:
    // 172..1811 ticks -> 988..2012 µs, rounded. A 0.625 slope matches at
    // 172/992/1811 but is 1 µs off at about a quarter of the ticks between.
    constexpr int32_t kInMin = 172, kInMax = 1811;
    constexpr int32_t kOutMin = 988, kOutMax = 2012;
    const int32_t scaled = (static_cast<int32_t>(ticks) - kInMin) *
                           (kOutMax - kOutMin) * 2 / (kInMax - kInMin);
    const int32_t us = (scaled + kOutMin * 2 + 1) / 2;
    return us < 0 ? 0 : static_cast<uint16_t>(us > 65535 ? 65535 : us);
}

bool rc_lost_raw(const RcLinkStatus& link, const PilotParams& p,
                 uint32_t now_ms) {
    if (!link.frames_seen) {
        return true;
    }
    if ((now_ms - link.last_frame_ms) > p.rc_timeout_ms) {
        return true;
    }
    return link.link_quality <= p.min_link_quality;
}

bool pilot_fresh(const PilotInput& in, const PilotParams& p,
                 uint32_t now_ms) {
    return in.num_raw > 0 && (now_ms - in.stamp_ms) <= p.rc_timeout_ms;
}

bool switch_on(const PilotInput& in, PilotSwitch sw) {
    const uint16_t bit = static_cast<uint16_t>(1u << static_cast<uint8_t>(sw));
    return (in.switches & bit) != 0;
}

bool manual_active(const PilotInput& in, bool fresh) {
    return fresh && switch_on(in, PilotSwitch::kBypass);
}

uint8_t rc_link_quality_pct(const RcLinkStatus& link) {
    if (!link.frames_seen) {
        return kLinkQualityUnknown;
    }
    return link.link_quality > 100 ? 100 : link.link_quality;
}

uint8_t rc_telemetry_us(const PilotInput& in, bool fresh, uint16_t* us,
                        uint8_t cap) {
    if (!fresh || us == nullptr) {
        return 0;
    }
    uint8_t n = in.num_raw < cap ? in.num_raw : cap;
    n = n < kMaxRcChannels ? n : kMaxRcChannels;
    for (uint8_t i = 0; i < n; ++i) {
        us[i] = in.raw_us[i];
    }
    return n;
}

bool mirror_us(const PilotInput& in, bool fresh, int8_t rc_channel,
               uint16_t* out_us) {
    if (!fresh || rc_channel < 0 || out_us == nullptr) {
        return false;
    }
    const uint8_t idx = static_cast<uint8_t>(rc_channel);
    if (idx >= in.num_raw || idx >= kMaxRcChannels) {
        return false;
    }
    *out_us = in.raw_us[idx];
    return true;
}
