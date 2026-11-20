#pragma once
// Time, delay and non-volatile storage for comms. Teensy: micros()/delay()/EEPROM.
// Native: a fake clock the tests move by hand, and a RAM array standing in for EEPROM
// (it survives comms_init(), so tests can simulate a reboot).

#include <stddef.h>
#include <stdint.h>

uint64_t comms_time_us();            // monotonic microseconds since boot (64-bit, no wrap)
void comms_delay_ms(uint32_t ms);    // blocking; only used by radio_config at boot

void comms_store_read(uint16_t addr, void *dst, size_t len);
void comms_store_write(uint16_t addr, const void *src, size_t len); // writes only changed bytes

#ifndef ARDUINO
void comms_test_set_time_us(uint64_t t);
void comms_test_advance_us(uint64_t dt);
void comms_test_wipe_store();
#endif
