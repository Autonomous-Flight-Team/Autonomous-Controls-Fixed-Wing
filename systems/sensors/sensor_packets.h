#ifndef GOOSE_SENSORS_SENSOR_PACKETS_H_
#define GOOSE_SENSORS_SENSOR_PACKETS_H_
// Sensor data structs and their log payload encodings.
// Hardware-free on purpose: the native unit tests build against this file,
// and systems/logging/decoder.py must match every layout below.
#include <stdint.h>
#include "logging_subsystem.h"

namespace sensors {

struct ImuData {
    float accelX; // m/s^2
    float accelY;
    float accelZ;
    float gyroX; // deg/s
    float gyroY;
    float gyroZ;
    float magX; // gauss
    float magY;
    float magZ;
};

struct BaroData {
    float temp_C;
    float pressure_Pa;
};

struct PitoData {
    float pressure_Pa;
    float die_temp_C;
};

struct GpsData {
    double latitude;  // deg
    double longitude; // deg
    float altitude;   // m above sea level
    float gr_speed;   // m/s
    float n_vel;      // m/s
    float e_vel;      // m/s
    float d_vel;      // m/s
    int8_t utc_day;
    int8_t utc_hour;
    int8_t utc_min;
    int8_t utc_sec;
    int32_t utc_nanosec;
};

// Name kept from SensorHealth for compatibility.
// goose-lint: disable-next-line=type-naming
struct sensor_trust_limits {
    uint8_t fail_threshold;                  // failures before degraded
    uint8_t full_fail_threshold;             // failures before failed
    uint8_t success_threshold_from_degraded; // successes back to nominal
    uint8_t success_threshold_from_failed;   // successes back to nominal
};

// Payload sizes; decoder.py's struct formats must have the same sizes.
const uint8_t IMU_PAYLOAD_LEN = 36;   // >9f
const uint8_t BARO_PAYLOAD_LEN = 8;   // >2f
const uint8_t PITOT_PAYLOAD_LEN = 8;  // >2f
const uint8_t GPS_PAYLOAD_LEN = 44;   // >2d5f4bi
const uint8_t TRUST_PAYLOAD_LEN = 4;  // >4B

inline void encode_imu(const ImuData &d, uint8_t *b) {
    const float v[9] = {d.accelX, d.accelY, d.accelZ, d.gyroX, d.gyroY,
                        d.gyroZ, d.magX, d.magY, d.magZ};
    for (int i = 0; i < 9; i++) {
        put_f32_be(&b[i * 4], v[i]);
    }
}

inline void encode_baro(const BaroData &d, uint8_t *b) {
    put_f32_be(&b[0], d.temp_C);
    put_f32_be(&b[4], d.pressure_Pa);
}

inline void encode_pitot(const PitoData &d, uint8_t *b) {
    put_f32_be(&b[0], d.pressure_Pa);
    put_f32_be(&b[4], d.die_temp_C);
}

inline void encode_gps(const GpsData &d, uint8_t *b) {
    put_f64_be(&b[0], d.latitude);
    put_f64_be(&b[8], d.longitude);
    put_f32_be(&b[16], d.altitude);
    put_f32_be(&b[20], d.gr_speed);
    put_f32_be(&b[24], d.n_vel);
    put_f32_be(&b[28], d.e_vel);
    put_f32_be(&b[32], d.d_vel);
    b[36] = (uint8_t)d.utc_day;
    b[37] = (uint8_t)d.utc_hour;
    b[38] = (uint8_t)d.utc_min;
    b[39] = (uint8_t)d.utc_sec;
    put_be32(&b[40], (uint32_t)d.utc_nanosec);
}

inline void encode_trust(const sensor_trust_limits &d, uint8_t *b) {
    b[0] = d.fail_threshold;
    b[1] = d.full_fail_threshold;
    b[2] = d.success_threshold_from_degraded;
    b[3] = d.success_threshold_from_failed;
}

// Encode + log in one call. These are what the sensor classes' loggable() use.
inline bool log_imu(uint8_t sys, uint8_t sub, const ImuData &d) {
    uint8_t b[IMU_PAYLOAD_LEN];
    encode_imu(d, b);
    return log_message(sys, sub, LOG_INFO, b, sizeof(b));
}
inline bool log_baro(uint8_t sys, uint8_t sub, const BaroData &d) {
    uint8_t b[BARO_PAYLOAD_LEN];
    encode_baro(d, b);
    return log_message(sys, sub, LOG_INFO, b, sizeof(b));
}
inline bool log_pitot(uint8_t sys, uint8_t sub, const PitoData &d) {
    uint8_t b[PITOT_PAYLOAD_LEN];
    encode_pitot(d, b);
    return log_message(sys, sub, LOG_INFO, b, sizeof(b));
}
inline bool log_gps(uint8_t sys, uint8_t sub, const GpsData &d) {
    uint8_t b[GPS_PAYLOAD_LEN];
    encode_gps(d, b);
    return log_message(sys, sub, LOG_INFO, b, sizeof(b));
}
inline bool log_trust(uint8_t sys, uint8_t sub,
                      const sensor_trust_limits &d) {
    uint8_t b[TRUST_PAYLOAD_LEN];
    encode_trust(d, b);
    return log_message(sys, sub, LOG_INFO, b, sizeof(b));
}

} // namespace sensors

#endif // GOOSE_SENSORS_SENSOR_PACKETS_H_
