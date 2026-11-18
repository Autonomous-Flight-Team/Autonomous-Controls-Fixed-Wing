#pragma once
#include <stdint.h>
#include <stddef.h>

// Msg encodings
#define MESSAGE_START 0xFEED // 16-bit marker, sent big-endian: FE ED
#define END_MESSAGE 0xDEAD   // 16-bit marker, sent big-endian: DE AD

// System / subsystem IDs
#define SENSOR 0x00
#define SUB_IMU 0x01
#define SUB_BARO 0x02

// Log levels
#define LOG_INFO 0x00
#define LOG_WARN 0x0F
#define LOG_ERROR 0xFF

// time to call (in hz)
#define CALL_IMU 100
#define CALL_BARO 190

typedef struct {
    int subsystem_id;      // which subsystem data comes from
    int sub_sub_system_id; // subsystem-unique code
    int payload_size;      // size of the data coming in
    int period_ms;         // how often to run probe
    int next_call;         // ticking; when it == period_ms run probe
} probe_entry_t;

// dummy sensors
typedef struct {
    int x, y, z;
} imu_raw_t;
typedef struct {
    int z;
} baro_raw_t;

// Writes to big endian
static inline void put_be16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)v;
}
static inline void put_be32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}

bool initialize_sd();
bool write_to_sd(bool force = false);
void logging_close();
uint32_t logging_dropped();
bool log_message(uint8_t system_code, uint8_t subsystem_code, uint8_t level,
                 const void *payload, uint8_t payload_len);
