

// Msg Encodings

#define MESSAGE_START 0xFF
#define END_MESSAGE 0x00

// Subsystem IDs
#define SENSOR 0x00
#define SUB_IMU 0x01
#define SUB_BARO 0x02

// LOG LEVELS
#define INFO 0x00
#define WARN 0x0F
#define ERROR 0xFF

// time to call (in hz)
#define CALL_IMU 100
#define CALL_BARO 190

// Structs
typedef struct
{
    int subsystem_id;      // which subsystem data comes from
    int sub_sub_system_id; // subsystem unique code (in case a subsystem wants to log multiple things, link input/output state)
    int payload_size;      // size of the data coming in
    int period_ms;         // how often to run probe
    int next_call;         // ticking, when it == period_ms run probe}
} probe_entry_t;

// dummy sensor
typedef struct
{
    int x, y, z;
} imu_raw_t;

typedef struct
{
    int z;
} baro_raw_t;
