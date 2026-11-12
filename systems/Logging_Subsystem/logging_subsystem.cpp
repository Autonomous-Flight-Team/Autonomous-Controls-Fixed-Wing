#include <Arduino.h>
#include <string.h>
#include "SdFat.h"
#include "ring_buffer.h"
#include "logging_subsystem.h"

#define BUFFER_SIZE 4096 // power of two; 128 is too small for ~100 Hz sensors
#define MAX_PAYLOAD 64
#define HEADER_SIZE 8 // START, sys, sub, level, ts(4)
// packet = HEADER + len byte + payload + END

static_assert((BUFFER_SIZE & (BUFFER_SIZE - 1)) == 0, "BUFFER_SIZE must be power of two");

// move to main at some point
static SdFat sd;
static FsFile myFile;
static ringbuf_t buffer;
static uint8_t backing_buffer[BUFFER_SIZE];
static RingBuffer logging_buffer;

static const char *filename = "datalog.bin";
static volatile uint32_t packet_drop = 0;

bool initialize_sd()
{
    if (!sd.begin(SdioConfig(FIFO_SDIO)))
    {
        Serial.println("[ERROR] SD initialization failed!");
        return false;
    }
    if (!logging_buffer.ringbuf_init(&buffer, backing_buffer, BUFFER_SIZE))
    {
        Serial.println("[ERROR] Buffer initialization failed!");
        return false;
    }
    if (!myFile.open(filename, O_WRONLY | O_CREAT | O_AT_END))
    {
        Serial.println("[ERROR] Could not open log file.");
        return false;
    }
    Serial.println("[SUCCESS] Logging ready.");
    return true;
}

bool write_to_sd(bool force)
{
    if (!myFile.isOpen())
        return false;

    static uint32_t last_sync = 0;
    uint8_t chunk[512];
    bool ok = true;

    while (logging_buffer.ringbuf_count(&buffer) >= sizeof(chunk) ||
           (force && !logging_buffer.ringbuf_is_empty(&buffer)))
    {
        size_t n = 0;
        while (n < sizeof(chunk) && logging_buffer.ringbuf_pop(&buffer, &chunk[n]))
            n++;
        if (myFile.write(chunk, n) != n)
        {
            Serial.println("[ERROR] SD write failed.");
            ok = false;
            break;
        }
    }

    if (force || millis() - last_sync >= 1000)
    {
        myFile.sync();
        last_sync = millis();
    }
    return ok;
}

void logging_close()
{
    write_to_sd(true);
    myFile.close();
}

uint32_t logging_dropped() { return packet_drop; }

static bool write_to_buffer(const uint8_t *data, size_t len)
{
    if (!data || len == 0)
        return false;
    if (len > logging_buffer.ring_buf_remaining(&buffer))
    {
        packet_drop++;
        return false;
    }
    for (size_t i = 0; i < len; ++i)
        logging_buffer.ringbuf_push(&buffer, data[i]);
    return true;
}

bool log_message(uint8_t system_code, uint8_t subsystem_code, uint8_t level,
                 const void *payload, uint8_t payload_len)
{
    if (payload_len > MAX_PAYLOAD || (payload_len && !payload))
        return false;

    uint8_t msg[HEADER_SIZE + 1 + MAX_PAYLOAD + 1];
    size_t i = 0;
    uint32_t ms = millis();

    msg[i++] = MESSAGE_START;
    msg[i++] = system_code;
    msg[i++] = subsystem_code;
    msg[i++] = level;
    memcpy(&msg[i], &ms, sizeof(ms));
    i += sizeof(ms);
    msg[i++] = payload_len;
    if (payload_len)
    {
        memcpy(&msg[i], payload, payload_len);
        i += payload_len;
    }
    msg[i++] = END_MESSAGE;

    return write_to_buffer(msg, i);
}