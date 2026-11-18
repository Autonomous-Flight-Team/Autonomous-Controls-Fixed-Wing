#include "comms_platform.h"

#include <string.h>

#ifdef ARDUINO
#include <Arduino.h>
#include <EEPROM.h>

uint64_t comms_time_us()
{
    // Extend the 32-bit micros() (wraps every 71 min) to 64 bits.
    static uint32_t last = 0;
    static uint64_t total = 0;
    uint32_t now = micros();
    total += (uint32_t)(now - last);
    last = now;
    return total;
}

void comms_delay_ms(uint32_t ms) { delay(ms); }

void comms_store_read(uint16_t addr, void *dst, size_t len)
{
    uint8_t *d = (uint8_t *)dst;
    for (size_t i = 0; i < len; i++)
        d[i] = EEPROM.read(addr + i);
}

void comms_store_write(uint16_t addr, const void *src, size_t len)
{
    const uint8_t *s = (const uint8_t *)src;
    for (size_t i = 0; i < len; i++)
        EEPROM.update(addr + i, s[i]);
}

#else // native

static uint64_t g_time_us = 0;
static uint8_t g_store[4284]; // same size as Teensy 4.1 emulated EEPROM

uint64_t comms_time_us() { return g_time_us; }
void comms_delay_ms(uint32_t ms) { g_time_us += (uint64_t)ms * 1000; }
void comms_test_set_time_us(uint64_t t) { g_time_us = t; }
void comms_test_advance_us(uint64_t dt) { g_time_us += dt; }
void comms_test_wipe_store() { memset(g_store, 0xFF, sizeof(g_store)); } // erased EEPROM reads 0xFF

void comms_store_read(uint16_t addr, void *dst, size_t len)
{
    if (addr + len > sizeof(g_store))
        return;
    memcpy(dst, &g_store[addr], len);
}

void comms_store_write(uint16_t addr, const void *src, size_t len)
{
    if (addr + len > sizeof(g_store))
        return;
    memcpy(&g_store[addr], src, len);
}

#endif
