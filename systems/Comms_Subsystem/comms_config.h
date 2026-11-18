#pragma once
// Compile-time defaults for the comms subsystem. Change values here, not in comms.cpp.
// Everything marked "must match the ground" has to agree with the ground radio / GCS.

#include <stdint.h>

// ---- Serial link to the RFD900x ----
#define COMMS_RADIO_SERIAL Serial1     // Teensy port wired to the radio (used in main.cpp)
#define COMMS_RADIO_BAUD 57600         // radio S1 SERIAL_SPEED=57 (factory default)
#define COMMS_TX_EXTRA_BUFFER 1024     // extra Serial1 TX buffer (Teensy default is only 64 B)

// ---- MAVLink identity ----
#define COMMS_SYSID 1                  // GOOSE system id
#define COMMS_COMPID 1                 // MAV_COMP_ID_AUTOPILOT1

// ---- Link health ----
#define COMMS_GCS_TIMEOUT_MS 3000      // ground heartbeats at 2 Hz; link lost after 3 s
#define COMMS_HEARTBEAT_INTERVAL_US 1000000

// ---- Signing ----
#define COMMS_REQUIRE_SIGNING 1        // 1: drop unsigned/badly signed frames (RADIO_STATUS allowlisted)
#define COMMS_TIMESTAMP_SAVE_S 50      // persist the signing timestamp each time it advances this much
                                       // (about once a minute, and also on disarm); must stay < 60
#define COMMS_EEPROM_BASE 4000         // EEPROM address of the key/timestamp block (68 bytes)

// ---- Scheduler / back-pressure ----
#define COMMS_TX_BYTES_PER_SEC 3000    // downlink byte budget (57600 baud is ~5760 B/s raw)
#define COMMS_MAX_RX_BYTES_PER_UPDATE 512
#define COMMS_MAX_TX_FRAMES_PER_UPDATE 8
#define COMMS_MIN_INTERVAL_US 20000    // fastest allowed stream rate: 50 Hz
#define COMMS_RADIO_STATUS_STALE_MS 5000
#define COMMS_CMD_DUP_WINDOW_MS 3000   // a retry (confirmation > 0) inside this window is a duplicate

// ---- Default stream intervals in microseconds (0 = off) ----
#define COMMS_RATE_GOOSE_STATE_US 500000
#define COMMS_RATE_SYS_STATUS_US 1000000
#define COMMS_RATE_ATTITUDE_US 100000
#define COMMS_RATE_GLOBAL_POSITION_INT_US 250000
#define COMMS_RATE_GPS_RAW_INT_US 500000
#define COMMS_RATE_VFR_HUD_US 250000
#define COMMS_RATE_SCALED_IMU_US 0
#define COMMS_RATE_SCALED_PRESSURE_US 1000000
#define COMMS_RATE_BATTERY_STATUS_US 1000000
#define COMMS_RATE_LOG_DEFAULT_US 1000000  // each logger (system, subsystem) stream

// ---- Logger ID scheme (mirrors Logging_Subsystem/logging_subsystem.h) ----
#define COMMS_LOG_SYS_SENSOR 0x00
#define COMMS_LOG_SUB_IMU 0x01
#define COMMS_LOG_SUB_BARO 0x02
#define COMMS_LOG_INFO 0x00
#define COMMS_LOG_WARN 0x0F
#define COMMS_LOG_ERROR 0xFF
#define COMMS_LOG_MAX_PAYLOAD 64

// ---- Radio configuration at boot (radio_config.cpp) ----
#define COMMS_RADIO_AUTOCONFIG 1       // check/apply radio registers once in setup()
#define COMMS_RADIO_AES 0              // 1: radio AES-128 on (key provisioned with AESKEY over USB)
#define COMMS_RADIO_NETID 117          // S3, must match the ground radio; pick a non-default value
#define COMMS_RADIO_AIR_SPEED 64       // S2, kbps, must match the ground radio
