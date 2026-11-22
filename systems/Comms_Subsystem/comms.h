#pragma once
// GOOSE comms: MAVLink v2 (signed) over the RFD900x. See README.md for the ground-side
// interface. Usage from main.cpp:
//
//   comms_init(radio_stream, telemetry, command_handler);   // once, in setup()
//   comms_update();                                           // every loop(), never blocks
//
// Not ISR-safe: call everything from the main loop.

#include <stddef.h>
#include <stdint.h>

#include "byte_stream.h"
#include "command_handler.h"
#include "comms_config.h"
#include "telemetry_source.h"

struct CommsConfig
{
    uint8_t sysid = COMMS_SYSID;
    uint8_t compid = COMMS_COMPID;
    bool require_signing = COMMS_REQUIRE_SIGNING;
    uint32_t gcs_timeout_ms = COMMS_GCS_TIMEOUT_MS;
    uint32_t tx_bytes_per_sec = COMMS_TX_BYTES_PER_SEC;
};

struct CommsStats
{
    uint32_t rx_frames = 0;           // accepted and dispatched
    uint32_t rx_bad_crc = 0;
    uint32_t rx_bad_signature = 0;    // wrong key, replayed or stale timestamp
    uint32_t rx_unsigned_dropped = 0; // unsigned and not allowlisted
    uint32_t tx_frames = 0;
    uint32_t tx_bytes = 0;
    uint32_t cmd_received = 0;
    uint32_t cmd_duplicates = 0;      // retries answered from the cache, not re-executed
    uint32_t events_dropped = 0;      // STATUSTEXT/GOOSE_LOG events lost to a full queue
    uint32_t timestamp_saves = 0;
    uint8_t radio_rssi = 0, radio_remrssi = 0, radio_txbuf = 0; // last RADIO_STATUS
};

void comms_init(ByteStream &link, TelemetrySource &telemetry, CommandHandler &handler,
                const CommsConfig &config = CommsConfig());
void comms_update();

// True while a GCS heartbeat (signed, when signing is required) arrived within gcs_timeout_ms.
bool comms_gcs_link_ok();

// Logger hook: call from log_message() once the logging branch merges. INFO records
// are decimated per (system, subsystem), latest sample wins; WARN/ERROR are queued
// immediately and also raise a STATUSTEXT. Returns false if rejected (len > 64).
bool comms_forward_log(uint8_t system, uint8_t subsystem, uint8_t level, uint32_t ts_ms,
                       const void *payload, uint8_t len);

// Queue a STATUSTEXT (MAV_SEVERITY_*). Text is cut to 50 characters.
bool comms_send_statustext(uint8_t severity, const char *text);

// USB console line (key provisioning, see signing.h). Returns true if handled.
bool comms_console_line(const char *line, char *reply, size_t reply_len);

bool comms_signing_key_loaded();
void comms_save_signing_timestamp(); // also done automatically (periodically and on disarm)
uint64_t comms_signing_timestamp();

const CommsStats &comms_stats();
// Current interval (us, 0 = off) and dropped-frame count for a standard stream;
// UINT32_MAX if the message is not a stream.
uint32_t comms_stream_interval_us(uint32_t msg_id);
uint32_t comms_stream_dropped(uint32_t msg_id);
uint32_t comms_log_interval_us(uint8_t system, uint8_t subsystem);
