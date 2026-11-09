
#include <string>
#include "SdFat.h"
#include "ring_buffer.cpp"

#define BUFFER_SIZE 128

// move to main at some point
SdFat sd;
FsFile myFile;
static ringbuf_t buffer;
static uint8_t backing_buffer[BUFFER_SIZE];
RingBuffer logging_buffer;

const char *filename = "datalog.bin";
int packet_drop = 0;

bool initialize_sd()
{ // Move to initializing code eventually

    Serial.println("Initializing SD Card");

    if (!sd.begin(SdioConfig(FIFO_SDIO)))
    {
        Serial.println("[ERROR] SD initialization failed!");
        return false;
    }

    Serial.println("[SUCCESS] SD card ready.");
    Serial.println("Initializing buffer");

    if (!logging_buffer.ringbuf_init(&buffer, backing_buffer, BUFFER_SIZE))
    {
        Serial.println("[ERROR] Buffer initialization failed!");
        return false;
    }

    return true;
}

bool write_to_sd()
{
    if (myFile.open(filename, O_WRONLY | O_CREAT | O_AT_END))
    {

        while (!logging_buffer.ringbuf_is_empty(&buffer))
        {
            myFile.print(logging_buffer.ringbuf_pop(&buffer));
        }
        myFile.close();
        Serial.println("Data batch written successfully.");
    }
    else
    {
        Serial.println("[ERROR] Could not open file for writing.");
    }
}

bool write_to_buffer(std::string *message)
{
    if (!message)
        return false;
    int size_left = logging_buffer.ring_buf_remaining(&buffer);

    if (message->size() > size_left)
    {
        packet_drop += 1;
        return false;
    }

    for (size_t i = 0; i < message->size(); ++i)
    {
        char byte = (*message)[i];
        logging_buffer.ringbuf_push(&buffer, byte);
    }

    return true;
}
