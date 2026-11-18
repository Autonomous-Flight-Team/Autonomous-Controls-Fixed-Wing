#pragma once
// MAVLink signing support: key and timestamp storage, the unsigned-frame allowlist,
// and the USB provisioning commands. The key lives only in EEPROM: it is never in
// git and never sent over the radio.
//
// EEPROM block at COMMS_EEPROM_BASE (68 bytes):
//   +0  "GSK1" magic, +4 signing key[32], +36 timestamp (u64, 10 us since 2015-01-01),
//   +44 "GAE1" magic, +48 radio AES key[16], +64 reserved[4]

#include <stddef.h>
#include <stdint.h>

#define SIGNING_KEY_LEN 32
#define RADIO_AES_KEY_LEN 16
#define SIGNING_TICKS_PER_SEC 100000ULL      // MAVLink timestamp unit is 10 us
#define SIGNING_BOOT_BUMP_TICKS (60ULL * SIGNING_TICKS_PER_SEC)

bool signing_load_key(uint8_t key[SIGNING_KEY_LEN]);   // false if never provisioned
void signing_save_key(const uint8_t key[SIGNING_KEY_LEN]);
void signing_clear_key();

uint64_t signing_load_timestamp();                      // 0 if never saved
void signing_save_timestamp(uint64_t ts);
uint64_t signing_boot_timestamp();                      // saved value + 60 s (ArduPilot does the same)

bool signing_load_aes_key(uint8_t key[RADIO_AES_KEY_LEN]);
void signing_save_aes_key(const uint8_t key[RADIO_AES_KEY_LEN]);
void signing_clear_aes_key();

// First 6 bytes of SHA-256(key), as hex. Safe to print; compare with the ground side
// (python: hashlib.sha256(key).hexdigest()[:12]).
void signing_fingerprint(const uint8_t key[SIGNING_KEY_LEN], char out[13]);

// Policy for frames that arrive without a valid signature. RADIO_STATUS injected by
// the SiK radio (sysid '3' = 51, compid 'D' = 68) is always unsigned, so it is let in.
bool signing_allow_unsigned(uint32_t msgid, uint8_t sysid, uint8_t compid, bool require_signing);

// USB console provisioning. Handles one text line; returns true if it was a signing
// command, with a human-readable answer in `reply`. Commands:
//   KEY <64 hex>    store signing key          KEY?     show fingerprint
//   KEY CLEAR       erase signing key
//   AESKEY <32 hex> store radio AES key        AESKEY?  show whether set
//   AESKEY CLEAR    erase radio AES key
// `key_changed` is set when the signing key was stored or cleared.
bool signing_console_command(const char *line, char *reply, size_t reply_len, bool *key_changed);

// Parse `n` bytes of hex (exactly 2n hex chars). Returns false on bad input.
bool signing_parse_hex(const char *hex, uint8_t *out, size_t n);
