#include <Arduino.h>
#include "logging_subsystem.h"
#include "logging_interface.h"
#include "SensorHealth.h"
#include "Barometer.h"
#include "Gps.h"
#include "ImuK.h"
#include "Pito.h"

#ifndef CALL_GPS
#define CALL_GPS 10
#endif
#ifndef CALL_PITOT
#define CALL_PITOT 50
#endif

class Periodic
{
public:
    explicit Periodic(uint32_t hz) : period_us_(1000000UL / hz) {}
    bool due()
    {
        if (t_ < period_us_)
            return false;
        t_ -= period_us_;
        if (t_ >= period_us_)
            t_ = 0;
        return true;
    }

private:
    uint32_t period_us_;
    elapsedMicros t_;
};

static Periodic imu_tick(CALL_IMU);
static Periodic baro_tick(CALL_BARO);
static Periodic gps_tick(CALL_GPS);
static Periodic pitot_tick(CALL_PITOT);

static sensors::Barometer baro_logger;
static sensors::ImuK imu_logger;
static sensors::Gps gps_logger;
static sensors::Pito pitot_logger;
static sensors::SensorHealth health_logger;

static bool logging_ok = false;
static uint32_t write_errors = 0;
static elapsedMillis status_timer, blink_timer;

static bool read_imu(sensors::ImuData &d)
{
    d = {0.0f, 0.0f, 9.81f, 0.0f, 0.0f, 0.0f, 0.2f, 0.0f, -0.4f}; // TODO real IMU read
    return true;
}
static bool read_baro(sensors::BaroData &d)
{
    d = {21.5f, 101325.0f}; // TODO real barometer read
    return true;
}
static bool read_gps(sensors::GpsData &d)
{
    d = {47.6062, -122.3321, 56.0f, 0.0f, 0.0f, 0.0f, 0.0f, 22, 13, 45, 59, 0}; // TODO real GPS read
    return true;
}
static bool read_pitot(sensors::PitoData &d)
{
    d = {101325.0f, 21.5f}; // TODO real pitot read
    return true;
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000)
    {
    }
    pinMode(LED_BUILTIN, OUTPUT);

    logging_ok = initialize_sd();
    if (!logging_ok)
        Serial.println("[ERROR] Logging disabled: SD init failed. Continuing without logging.");
    else
    {
        // Record the trust thresholds once at the start of every run.
        sensors::sensor_trust_limits limits = {3, 10, 5, 20}; // TODO real values
        health_logger.loggable(SENSOR, SUB_SENSOR_TRUST, &limits);
    }
}

void loop()
{
    if (imu_tick.due())
    {
        sensors::ImuData imu;
        if (read_imu(imu) && logging_ok)
            imu_logger.loggable(SENSOR, SUB_IMU, &imu);
    }

    if (baro_tick.due())
    {
        sensors::BaroData baro;
        if (read_baro(baro) && logging_ok)
            baro_logger.loggable(SENSOR, SUB_BARO, &baro);
    }

    if (gps_tick.due())
    {
        sensors::GpsData gps;
        if (read_gps(gps) && logging_ok)
            gps_logger.loggable(SENSOR, SUB_GPS, &gps);
    }

    if (pitot_tick.due())
    {
        sensors::PitoData pitot;
        if (read_pitot(pitot) && logging_ok)
            pitot_logger.loggable(SENSOR, SUB_PITOT, &pitot);
    }

    if (logging_ok && !write_to_sd())
        write_errors++;

    if (Serial.available() && Serial.read() == 'q')
    {
        logging_close();
        logging_ok = false;
        Serial.println("Log closed. Safe to remove the SD card.");
    }

    if (status_timer >= 5000)
    {
        status_timer = 0;
        Serial.printf("log: %s dropped=%lu write_errors=%lu\n", logging_ok ? "ON" : "OFF",
                      (unsigned long)logging_dropped(), (unsigned long)write_errors);
    }

    if (blink_timer >= (logging_ok ? 500u : 100u))
    {
        blink_timer = 0;
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
}