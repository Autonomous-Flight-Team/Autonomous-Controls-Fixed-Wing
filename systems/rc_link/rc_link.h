#ifndef RC_LINK_H_
#define RC_LINK_H_

#include "byte_stream.h"
#include "pilot_input.h"
#include <cstdint>

// CRSF receiver link (ExpressLRS ER8 on Serial2 at 420000 baud): parses the
// receiver's frames into PilotInput and reports link facts. No decisions:
// pilot/ turns RcLinkStatus into kRcLost (see pilot.h).
//
// Main loop, at the START of every loop (before the fault manager):
//
//   rc_link_update(now_ms);
//   PilotInput rc;
//   rc_link_latest(&rc);
//
// Not ISR-safe: call everything from the main loop. Never blocks.
//
// Teensy wiring: the default Serial2 RX buffer is 64 bytes, a few ms at
// 420000 baud. Give it room in setup(), before rc_link_init():
//   static uint8_t rc_rx_buf[RC_LINK_RX_EXTRA_BUFFER];
//   Serial2.begin(420000);
//   Serial2.addMemoryForRead(rc_rx_buf, sizeof(rc_rx_buf));

#define RC_LINK_BAUD 420000
#define RC_LINK_RX_EXTRA_BUFFER 1024

// Which receiver channel (0-based raw_us index) feeds each PilotInput field;
// -1 = not mapped (the field stays 0 / the switch bit stays clear). Defaults
// are the ELRS/EdgeTX AETR order plus the bypass switch on CH7.
struct RcLinkConfig {
    int8_t roll_ch = 0;      // CH1 aileron
    int8_t pitch_ch = 1;     // CH2 elevator
    int8_t throttle_ch = 2;  // CH3
    int8_t yaw_ch = 3;       // CH4 rudder
    int8_t flaps_ch = -1;    // not assigned yet
    int8_t bypass_ch = 6;    // CH7 (SA): PilotSwitch::kBypass
    int8_t autonomy_ch = -1; // SB: PilotSwitch::kAutonomy (reserved)
    // A switch reads on at or above this. 2-position: 988 / 2012 µs;
    // 3-position adds 1500 (reads off).
    uint16_t switch_on_us = 1750;
};

void rc_link_init(ByteStream& uart, const RcLinkConfig& cfg = RcLinkConfig());

// Reads every byte waiting and parses complete frames. Keeps the newest
// RC channels frame; a frame with a bad CRC or length is dropped and counted.
void rc_link_update(uint32_t now_ms);

// Newest RC channels frame. False until the first valid one.
// stamp_ms is the rc_link_update() time it was parsed in.
bool rc_link_latest(PilotInput* out);

// Link facts. last_frame_ms and frames_seen follow RC channels frames only
// (the receiver stops sending them on failsafe). link_quality / rssi_dbm
// come from LINK_STATISTICS frames; stats_seen is false (LQ unknown) until
// the first one arrives.
const RcLinkStatus& rc_link_status();

// ---- CRSF details, exposed for tests ----

constexpr uint8_t kCrsfSync = 0xC8;       // receiver -> flight controller
constexpr uint8_t kCrsfMaxFrame = 64;     // sync + len + type..crc
constexpr uint8_t kCrsfTypeLinkStats = 0x14;
constexpr uint8_t kCrsfTypeRcChannels = 0x16;
constexpr uint8_t kCrsfRcPayloadLen = 22; // 16 channels x 11 bits
constexpr uint8_t kCrsfLinkStatsPayloadLen = 10;

// CRC-8/DVB-S2 (poly 0xD5) over type + payload, as CRSF uses.
uint8_t crsf_crc8(const uint8_t* data, uint8_t len);

// Unpack the 16 little-endian 11-bit channels of an RC channels payload.
void crsf_unpack_channels(const uint8_t payload[kCrsfRcPayloadLen],
                          uint16_t ticks[kMaxRcChannels]);

#endif // RC_LINK_H_
