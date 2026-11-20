// Minimal Arduino stand-in for the native (PC) test build.
// Only what the logging subsystem uses. Tests set g_millis directly.
#pragma once

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
