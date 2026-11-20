#include "rc_link.h"

#include "pilot.h"
#include <cstring>

namespace {

struct LinkState {
    ByteStream* uart = nullptr;
    RcLinkConfig cfg;
    PilotInput latest;
    bool have_frame = false;
    RcLinkStatus status;
    uint8_t buf[kCrsfMaxFrame] = {}; // frame being assembled, buf[0] = sync
    uint8_t len = 0;                 // bytes in buf
};

LinkState g;

float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// Centered stick: 988..2012 µs -> -1..1 (1500 = 0).
float stick(const PilotInput& in, int8_t ch) {
    if (ch < 0 || ch >= kMaxRcChannels) {
        return 0.0f;
    }
    return clampf((static_cast<float>(in.raw_us[ch]) - 1500.0f) / 512.0f,
                  -1.0f, 1.0f);
}

// One-sided lever (throttle, flaps): 988..2012 µs -> 0..1.
float lever(const PilotInput& in, int8_t ch) {
    if (ch < 0 || ch >= kMaxRcChannels) {
        return 0.0f;
    }
    return clampf((static_cast<float>(in.raw_us[ch]) - 988.0f) / 1024.0f,
                  0.0f, 1.0f);
}

uint16_t switch_bit(const PilotInput& in, int8_t ch, PilotSwitch sw) {
    if (ch < 0 || ch >= kMaxRcChannels ||
        in.raw_us[ch] < g.cfg.switch_on_us) {
        return 0;
    }
    return static_cast<uint16_t>(1u << static_cast<uint8_t>(sw));
}

void on_rc_channels(const uint8_t* payload, uint32_t now_ms) {
    uint16_t ticks[kMaxRcChannels];
    crsf_unpack_channels(payload, ticks);
    PilotInput in;
    in.source = InputSource::kRc;
    for (uint8_t i = 0; i < kMaxRcChannels; ++i) {
        in.raw_us[i] = crsf_ticks_to_us(ticks[i]);
    }
    in.num_raw = kMaxRcChannels;
    in.roll = stick(in, g.cfg.roll_ch);
    in.pitch = stick(in, g.cfg.pitch_ch);
    in.yaw = stick(in, g.cfg.yaw_ch);
    in.throttle = lever(in, g.cfg.throttle_ch);
    in.flaps = lever(in, g.cfg.flaps_ch);
    in.switches = static_cast<uint16_t>(
        switch_bit(in, g.cfg.bypass_ch, PilotSwitch::kBypass) |
        switch_bit(in, g.cfg.autonomy_ch, PilotSwitch::kAutonomy));
    in.stamp_ms = now_ms;
    g.latest = in;
    g.have_frame = true;
    g.status.frames_seen = true;
    g.status.last_frame_ms = now_ms;
}

void on_link_stats(const uint8_t* p) {
    // uplink RSSI ant1, ant2 (dBm, sent as a positive number), uplink LQ %,
    // uplink SNR, active antenna, rf mode, tx power, downlink rssi/lq/snr.
    const uint8_t rssi = p[4] ? p[1] : p[0];
    g.status.rssi_dbm = static_cast<int8_t>(rssi > 128 ? -128 : -rssi);
    g.status.link_quality = p[2];
}

// buf holds a whole frame: check it and dispatch it.
void dispatch(uint32_t now_ms) {
    const uint8_t frame_len = g.buf[1]; // type + payload + crc
    const uint8_t* body = &g.buf[2];    // type, payload...
    if (crsf_crc8(body, static_cast<uint8_t>(frame_len - 1)) !=
        body[frame_len - 1]) {
        g.status.bad_frames++;
        return;
    }
    const uint8_t type = body[0];
    const uint8_t payload_len = static_cast<uint8_t>(frame_len - 2);
    if (type == kCrsfTypeRcChannels && payload_len == kCrsfRcPayloadLen) {
        on_rc_channels(&body[1], now_ms);
    } else if (type == kCrsfTypeLinkStats &&
               payload_len >= kCrsfLinkStatsPayloadLen) {
        on_link_stats(&body[1]);
    }
    // Other frame types (ELRS device info, etc.) are valid and ignored.
}

// Drop buf[0] and look for the next sync byte in what was already read.
void resync() {
    uint8_t i = 1;
    while (i < g.len && g.buf[i] != kCrsfSync) {
        ++i;
    }
    g.len = static_cast<uint8_t>(g.len - i);
    std::memmove(g.buf, &g.buf[i], g.len);
}

// Parse what buf holds. Returns when more bytes are needed.
void parse(uint32_t now_ms) {
    while (g.len > 0) {
        if (g.buf[0] != kCrsfSync) {
            resync();
            continue;
        }
        if (g.len < 2) {
            return;
        }
        const uint8_t frame_len = g.buf[1];
        if (frame_len < 2 || frame_len > kCrsfMaxFrame - 2) {
            g.status.bad_frames++; // impossible length: false sync byte
            resync();
            continue;
        }
        const uint8_t total = static_cast<uint8_t>(frame_len + 2);
        if (g.len < total) {
            return;
        }
        dispatch(now_ms);
        g.len = static_cast<uint8_t>(g.len - total);
        std::memmove(g.buf, &g.buf[total], g.len);
    }
}

} // namespace

uint8_t crsf_crc8(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; ++b) {
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0xD5)
                               : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

void crsf_unpack_channels(const uint8_t payload[kCrsfRcPayloadLen],
                          uint16_t ticks[kMaxRcChannels]) {
    uint32_t bits = 0;
    uint8_t nbits = 0;
    uint8_t in = 0;
    for (uint8_t ch = 0; ch < kMaxRcChannels; ++ch) {
        while (nbits < 11) {
            bits |= static_cast<uint32_t>(payload[in++]) << nbits;
            nbits = static_cast<uint8_t>(nbits + 8);
        }
        ticks[ch] = static_cast<uint16_t>(bits & 0x7FF);
        bits >>= 11;
        nbits = static_cast<uint8_t>(nbits - 11);
    }
}

void rc_link_init(ByteStream& uart, const RcLinkConfig& cfg) {
    g = LinkState();
    g.uart = &uart;
    g.cfg = cfg;
}

void rc_link_update(uint32_t now_ms) {
    if (g.uart == nullptr) {
        return;
    }
    // Bounded: a stuck-high line can't hold the loop for long.
    for (int n = 0; n < 2048 && g.uart->available() > 0; ++n) {
        const int c = g.uart->read();
        if (c < 0) {
            break;
        }
        g.buf[g.len++] = static_cast<uint8_t>(c);
        if (g.len == kCrsfMaxFrame) {
            parse(now_ms); // always frees room: a full buf holds a frame
        }
    }
    parse(now_ms);
}

bool rc_link_latest(PilotInput* out) {
    if (!g.have_frame || out == nullptr) {
        return false;
    }
    *out = g.latest;
    return true;
}

const RcLinkStatus& rc_link_status() {
    return g.status;
}
