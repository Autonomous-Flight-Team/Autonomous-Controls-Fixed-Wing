#pragma once
// Minimal byte transport used by comms and radio_config. On the Teensy it wraps an
// Arduino Stream (Serial1); on native it is an in-memory pipe for tests.

#include <stddef.h>
#include <stdint.h>

class ByteStream
{
public:
    virtual ~ByteStream() = default;
    virtual int available() = 0;                               // bytes ready to read
    virtual int read() = 0;                                    // next byte, or -1
    virtual size_t write(const uint8_t *data, size_t len) = 0; // never blocks if len <= availableForWrite()
    virtual int availableForWrite() = 0;                       // bytes that can be written without blocking
};

#ifdef ARDUINO
#include <Arduino.h>

class ArduinoByteStream : public ByteStream
{
public:
    explicit ArduinoByteStream(Stream &s) : s_(s) {}
    int available() override { return s_.available(); }
    int read() override { return s_.read(); }
    size_t write(const uint8_t *data, size_t len) override { return s_.write(data, len); }
    int availableForWrite() override { return s_.availableForWrite(); }

private:
    Stream &s_;
};
#endif

// Fixed-size byte FIFO, no allocation.
template <size_t N>
class ByteRing
{
public:
    size_t count() const { return count_; }
    size_t space() const { return N - count_; }
    bool push(uint8_t b)
    {
        if (count_ == N)
            return false;
        buf_[(head_ + count_) % N] = b;
        count_++;
        return true;
    }
    int pop()
    {
        if (count_ == 0)
            return -1;
        uint8_t b = buf_[head_];
        head_ = (head_ + 1) % N;
        count_--;
        return b;
    }
    void clear() { head_ = count_ = 0; }

private:
    uint8_t buf_[N] = {};
    size_t head_ = 0;
    size_t count_ = 0;
};

using PipeRing = ByteRing<4096>;

// One end of an in-memory pipe: reads from `rx`, writes to `tx`. Two MemoryByteStreams
// sharing two rings in opposite directions form a loopback link. `write_limit` caps
// availableForWrite() to simulate a full radio buffer.
class MemoryByteStream : public ByteStream
{
public:
    MemoryByteStream(PipeRing &rx, PipeRing &tx) : rx_(rx), tx_(tx) {}
    int available() override { return (int)rx_.count(); }
    int read() override { return rx_.pop(); }
    size_t write(const uint8_t *data, size_t len) override
    {
        size_t room = (size_t)availableForWrite_internal();
        size_t n = 0;
        while (n < len && n < room && tx_.push(data[n]))
            n++;
        written_total += n;
        return n;
    }
    int availableForWrite() override { return availableForWrite_internal(); }

    size_t write_limit = SIZE_MAX; // bytes the "UART" will accept right now
    size_t written_total = 0;

private:
    int availableForWrite_internal() const
    {
        size_t room = tx_.space();
        return (int)(room < write_limit ? room : write_limit);
    }
    PipeRing &rx_;
    PipeRing &tx_;
};
