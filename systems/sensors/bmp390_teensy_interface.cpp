// HARDWARE ONLY: compiled to nothing unless GNC_HARDWARE_BUILD is defined
// (env:teensy41), so the native build can include all of systems/.
#ifdef GNC_HARDWARE_BUILD

// Teensy 4.1 implementation of the libdriver BMP390 interface
// (interface/driver_bmp390_interface.h). The barometer is on I2C (Wire,
// pins 18/19). SPI is not wired, so the SPI hooks report failure.
#include <Arduino.h>
#include <Wire.h>
#include <stdarg.h>
#include <stdio.h>

#include "driver_bmp390_interface.h"

// libdriver passes 8-bit (shifted) addresses; Wire wants 7-bit.
static uint8_t addr7(uint8_t addr) {
    return addr >> 1;
}

uint8_t bmp390_interface_iic_init(void) {
    Wire.begin();
    Wire.setClock(400000);
    return 0;
}

uint8_t bmp390_interface_iic_deinit(void) {
    return 0; // Wire is shared with the pitot tube; leave it running
}

uint8_t bmp390_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf,
                                  uint16_t len) {
    Wire.beginTransmission(addr7(addr));
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return 1;
    }
    if (Wire.requestFrom(addr7(addr), (uint8_t)len) != len) {
        return 1;
    }
    for (uint16_t i = 0; i < len; i++) {
        buf[i] = Wire.read();
    }
    return 0;
}

uint8_t bmp390_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf,
                                   uint16_t len) {
    Wire.beginTransmission(addr7(addr));
    Wire.write(reg);
    Wire.write(buf, len);
    return Wire.endTransmission() == 0 ? 0 : 1;
}

uint8_t bmp390_interface_spi_init(void) {
    return 1;
}

uint8_t bmp390_interface_spi_deinit(void) {
    return 1;
}

uint8_t bmp390_interface_spi_read(uint8_t reg, uint8_t *buf, uint16_t len) {
    (void)reg;
    (void)buf;
    (void)len;
    return 1;
}

uint8_t bmp390_interface_spi_write(uint8_t reg, uint8_t *buf, uint16_t len) {
    (void)reg;
    (void)buf;
    (void)len;
    return 1;
}

void bmp390_interface_delay_ms(uint32_t ms) {
    // Only used during bring-up, never in the control loop.
    delay(ms); // goose-lint: disable=blocking-delay
}

void bmp390_interface_debug_print(const char *const fmt, ...) {
    char line[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    Serial.print(line);
}

void bmp390_interface_receive_callback(uint8_t type) {
    (void)type; // FIFO/interrupt mode is not used
}

#endif // GNC_HARDWARE_BUILD
