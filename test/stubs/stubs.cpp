// Storage for the native test stubs (Arduino.h, SdFat.h).
#include "Arduino.h"
#include "SdFat.h"

uint32_t g_millis = 0;
SerialStub Serial;
SdStub g_sd;
