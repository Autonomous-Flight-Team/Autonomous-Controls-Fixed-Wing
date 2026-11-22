#ifndef GOOSE_LOGGING_LOGGING_SUBSYSTEM_H_
#define GOOSE_LOGGING_LOGGING_SUBSYSTEM_H_
#include <stdint.h>
#include <stddef.h>
#include <string.h>

// Msg encodings (big endian)
#define MESSAGE_START 0xFEED
#define END_MESSAGE 0xDEAD

// System / subsystem IDs
#define SENSOR 0x00
#define SUB_IMU 0x01
#define SUB_BARO 0x02
#define SUB_GPS 0x03
#define SUB_PITOT 0x04
#define SUB_SENSOR_TRUST 0x05

// Log levels
#define LOG_INFO 0x00
#define LOG_WARN 0x0F
#define LOG_ERROR 0xFF

// time to call (in hz)
#define CALL_IMU 100
#define CALL_BARO 190

typedef struct
{
    int subsystem_id;
    int sub_sub_system_id;
    int payload_size;
    int period_ms;
    int next_call;
} probe_entry_t;

// dummy sensor (for testing)
typedef struct
{
    int z;
} baro_raw_t;

// convert to Big-endian
static inline void put_be16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)v;
}
static inline void put_be32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}

static inline void put_be64(uint8_t *p, uint64_t v)
{
    put_be32(p, (uint32_t)(v >> 32));
    put_be32(p + 4, (uint32_t)v);
}

// Floating point to big endian
static_assert(sizeof(float) == 4 && sizeof(double) == 8, "expects IEEE-754 float/double");
static inline void put_f32_be(uint8_t *p, float f)
{
    uint32_t u;
    memcpy(&u, &f, sizeof(u));
    put_be32(p, u);
}
static inline void put_f64_be(uint8_t *p, double d)
{
    uint64_t u;
    memcpy(&u, &d, sizeof(u));
    put_be64(p, u);
}

bool initialize_sd();
bool write_to_sd(bool force = false);
void logging_close();
uint32_t logging_dropped();    // packets rejected because the ring buffer was full
uint32_t logging_lost_bytes(); // bytes discarded because an SD write failed

bool log_message(uint8_t system_code, uint8_t subsystem_code, uint8_t level,
                 const void *payload, uint8_t payload_len);

#endif // GOOSE_LOGGING_LOGGING_SUBSYSTEM_H_
