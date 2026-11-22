#pragma once
// Single entry point to the generated MAVLink headers. Include this, never the
// generated files directly, so every file sees the same options.
//
// Note: the generated helpers are `static inline`, so each .cpp file gets its own
// channel-status table. All packing and parsing for the radio link therefore lives
// in comms.cpp; other files only use the types and enums.

#define MAVLINK_COMM_NUM_BUFFERS 2      // channel 0 = radio; channel 1 is free for tests
#define MAVLINK_MAX_SIGNING_STREAMS 8

// The generated headers take addresses of packed members in messages we do not use;
// silence that warning for them only.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include "mavlink/generated/goose/mavlink.h"
#pragma GCC diagnostic pop
