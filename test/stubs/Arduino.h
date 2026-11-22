// Minimal Arduino stand-in for the native (PC) test build.
// Only what the logging subsystem uses. Tests set g_millis directly.
#ifndef GOOSE_STUBS_ARDUINO_H_
#define GOOSE_STUBS_ARDUINO_H_

#include <stdint.h>
#include <stdio.h>
#include <string.h>

extern uint32_t g_millis;

inline uint32_t millis() { return g_millis; }

struct SerialStub {
    void println(const char *) {}
    template <typename... Args> void printf(const char *, Args...) {}
};

extern SerialStub Serial;

#endif // GOOSE_STUBS_ARDUINO_H_
