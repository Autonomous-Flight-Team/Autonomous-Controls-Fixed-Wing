// GOOSE firmware entry point: sensors -> logging -> SD card.
// Teensy only: env:native excludes src/ entirely.
// Real or dummy sensors are chosen at compile time (systems/sensors/
// sensor_suite.h). Build with -D GOOSE_DUMMY_SENSORS (env teensy41_dummy)
// to bench-test logging on a Teensy with no sensors attached.
#include <Arduino.h>

#include "logging_subsystem.h"
#include "sensor_suite.h"

static sensors::SensorSuite suite;

static bool logging_ok = false;
static uint32_t write_errors = 0;
static elapsedMillis status_timer, blink_timer;

static const char *status_name(sensors::sensor_status s) {
    switch (s) {
    case sensors::sensor_status::NOMINAL:
        return "OK";
    case sensors::sensor_status::DEGRADED:
        return "DEGRADED";
    case sensors::sensor_status::FAILED:
        return "FAILED";
    default:
        return "UNINIT";
    }
}

static void print_status() {
    // goose-lint: disable-next-line=fixed-width-int  (printf %lu needs it)
    typedef unsigned long ul;
    Serial.printf("log: %s dropped=%lu lost_bytes=%lu write_errors=%lu\n",
                  logging_ok ? "ON" : "OFF", (ul)logging_dropped(),
                  (ul)logging_lost_bytes(), (ul)write_errors);
    Serial.printf("sensors (%s): imu=%s baro=%s gps=%s pitot=%s\n",
                  GOOSE_SENSORS_ARE_REAL ? "real" : "dummy",
                  status_name(suite.imu.getStatus()),
                  status_name(suite.baro.getStatus()),
                  status_name(suite.gps.getStatus()),
                  status_name(suite.pitot.getStatus()));
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
    }
    pinMode(LED_BUILTIN, OUTPUT);

    logging_ok = initialize_sd();
    if (!logging_ok) {
        Serial.println("[ERROR] SD init failed. Continuing without logging.");
    }

    if (!suite.begin(logging_ok)) {
        Serial.println("[WARN] Not every sensor started; see status below.");
    }
    print_status();
}

void loop() {
    suite.update(micros(), logging_ok);

    if (logging_ok && !write_to_sd()) {
        write_errors++;
    }

    if (Serial.available() && Serial.read() == 'q') {
        logging_close();
        logging_ok = false;
        Serial.println("Log closed. Safe to remove the SD card.");
    }

    if (status_timer >= 5000) {
        status_timer = 0;
        print_status();
    }

    // Slow blink: logging. Fast blink: not logging.
    if (blink_timer >= (logging_ok ? 500u : 100u)) {
        blink_timer = 0;
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
}
