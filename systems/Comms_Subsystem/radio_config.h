#pragma once
// One-shot RFD900x (SiK) configuration check at boot, on the ground, before
// comms_init(). Blocking for about 3-4 s (AT mode needs 1 s of silence around "+++").
// Never call it in flight: both radios must match, and the link drops while in AT mode.
//
// It reads ATI5 and only writes registers that differ, then AT&W + ATZ. Managed:
//   S2 AIR_SPEED, S3 NETID, S6 MAVLINK (= 1), S15 encryption level, AT&E key (if AES on)

#include <stdint.h>

#include "byte_stream.h"
#include "comms_config.h"
#include "signing.h"

struct RadioSettings
{
    uint16_t net_id = COMMS_RADIO_NETID;
    uint16_t air_speed = COMMS_RADIO_AIR_SPEED;
    uint8_t mavlink = 1;
    bool aes = COMMS_RADIO_AES;
    uint8_t aes_key[RADIO_AES_KEY_LEN] = {}; // filled from EEPROM by radio_config_default_settings()
};

enum RadioConfigResult
{
    RADIO_CONFIG_UNCHANGED,   // radio already matched
    RADIO_CONFIG_APPLIED,     // settings written, saved and radio rebooted
    RADIO_CONFIG_NO_RESPONSE, // no "OK" to +++ (wrong baud, not powered, wiring)
    RADIO_CONFIG_PARSE_ERROR, // ATI5 did not list the managed registers
    RADIO_CONFIG_WRITE_FAILED,
    RADIO_CONFIG_NO_AES_KEY   // AES requested but no key provisioned (radio left unchanged)
};

// Settings from comms_config.h plus the AES key from EEPROM. Returns false if AES is
// on but no key is stored.
bool radio_config_default_settings(RadioSettings &out);

RadioConfigResult radio_config_apply(ByteStream &radio, const RadioSettings &settings);
const char *radio_config_result_name(RadioConfigResult r);
