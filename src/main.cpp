// Comms demo wiring: RFD900x on Serial1, dummy telemetry, stub command handler.
// To fly the real state machines instead, pass a StateBridge (state_bridge.h) as both the
// telemetry source and the command handler; see systems/Comms_Subsystem/README.md.
//
// USB serial console (115200): type KEY? / KEY <64 hex> / AESKEY <32 hex> etc. to
// provision keys (see systems/Comms_Subsystem/README.md), STATS for link counters.

#ifdef ARDUINO
#include <Arduino.h>

#include "comms.h"
#include "radio_config.h"

static DummyTelemetrySource telemetry;
static StubCommandHandler command_handler(telemetry.state());
static ArduinoByteStream radio_stream(COMMS_RADIO_SERIAL);
static uint8_t radio_tx_buffer[COMMS_TX_EXTRA_BUFFER];

static void poll_console()
{
    static char line[96];
    static size_t len = 0;
    while (Serial.available() > 0)
    {
        char c = (char)Serial.read();
        if (c != '\n' && c != '\r')
        {
            if (len < sizeof(line) - 1)
                line[len++] = c;
            continue;
        }
        if (len == 0)
            continue;
        line[len] = '\0';
        len = 0;

        char reply[96];
        if (comms_console_line(line, reply, sizeof(reply)))
            Serial.println(reply);
        else if (strcmp(line, "STATS") == 0)
        {
            const CommsStats &s = comms_stats();
            Serial.printf("link %s rx %lu crc %lu badsig %lu unsigned %lu tx %lu bytes %lu dup %lu txbuf %u rssi %u/%u\n",
                          comms_gcs_link_ok() ? "OK" : "LOST", s.rx_frames, s.rx_bad_crc, s.rx_bad_signature,
                          s.rx_unsigned_dropped, s.tx_frames, s.tx_bytes, s.cmd_duplicates, s.radio_txbuf,
                          s.radio_rssi, s.radio_remrssi);
        }
        else
            Serial.println("commands: KEY? | KEY <64 hex> | KEY CLEAR | AESKEY? | AESKEY <32 hex> | AESKEY CLEAR | STATS");
    }
}

void setup()
{
    Serial.begin(115200);
    COMMS_RADIO_SERIAL.begin(COMMS_RADIO_BAUD);
    COMMS_RADIO_SERIAL.addMemoryForWrite(radio_tx_buffer, sizeof(radio_tx_buffer));

#if COMMS_RADIO_AUTOCONFIG
    RadioSettings rs;
    RadioConfigResult r = radio_config_default_settings(rs) ? radio_config_apply(radio_stream, rs) : RADIO_CONFIG_NO_AES_KEY;
    Serial.printf("radio_config: %s\n", radio_config_result_name(r));
#endif

    comms_init(radio_stream, telemetry, command_handler);
    Serial.println(comms_signing_key_loaded() ? "comms: signing key loaded" : "comms: NO signing key, use KEY <64 hex>");
}

void loop()
{
    uint32_t now = millis();
    telemetry.update(now);

    // fake 10 Hz IMU log records, same layout as Logging_Subsystem's imu_raw_t
    static uint32_t next_log = 0;
    if (now >= next_log)
    {
        next_log = now + 100;
        int32_t imu_raw[3] = {(int32_t)(now % 1000), 0, -1000};
        comms_forward_log(COMMS_LOG_SYS_SENSOR, COMMS_LOG_SUB_IMU, COMMS_LOG_INFO, now, imu_raw, sizeof(imu_raw));
    }

    comms_update();
    poll_console();
}

#else
// Native builds only run the unit tests (pio test -e native); nothing to do here.
int main() { return 0; }
#endif
