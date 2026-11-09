#include <iostream>
#include <string>

class logging
{
public:
    virtual std::string loggable(uint8_t system_code, uint8_t subsystem_code, void *structPtr) = 0;
    virtual ~logging() = default;
};

/*
used for sensor writing
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
#include <chrono>


typedef struct
{
    int x, y, z;
} imu_raw_t;

bool loggable(uint8_t system_code, uint8_t subsystem_code, void *structPtr){
    int64_t millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

    std::string struct_info = std::to_string(structPtr->x) + std::to_string(structPtr->y) + std::to_string(structPtr->z)
    std::string message = MESSAGE_START + system_code + subsystem_code + INFO + millis + <struct> + END_MESSAGE;

    if (write_to_buffer(&message)){
        return true;
    }

    return false;
}



*/