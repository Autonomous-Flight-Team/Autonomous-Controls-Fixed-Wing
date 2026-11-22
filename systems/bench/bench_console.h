#ifndef BENCH_CONSOLE_H_
#define BENCH_CONSOLE_H_

#include "bench_setpoints.h"
#include <cstddef>
#include <cstdint>
#include <cstdio>

// USB console commands for bench work. Chained after comms_console_line():
// main tries comms first, then bench_console_line(), then prints help.
//
//   bench | bench exit | exit    enter / leave BENCH       (via StateBridge)
//   arm | disarm | clear         arm, disarm, fault clear  (via StateBridge)
//   us <ch> <µs>                 one surface setpoint      (BenchSetpoints)
//   center                       every surface to trim     (BenchSetpoints)
//   sweep <ch> | sweep all       one slow cycle            (BenchSetpoints)
//
// <ch> is a channel name ("elev") or index. Mode commands never touch FSM
// inputs directly: they go through the same StateBridge guards and one-loop
// pulses as MAVLink. Setpoints only matter in BENCH (Actuators ignores them
// otherwise). Console mode commands are allowed without MAVLink signing
// (needs a physical cable); the reply is tagged "USB" for the log.

// Values are logged: never renumber, only add.
enum class BenchOp : uint8_t {
    kNone = 0,
    kEnter = 1,
    kExit = 2,
    kArm = 3,
    kDisarm = 4,
    kClear = 5,
    kSetUs = 6,
    kCenter = 7,
    kSweep = 8,
};

struct BenchCommand {
    BenchOp op = BenchOp::kNone;
    int8_t channel = -1; // -1 = all (sweep)
    uint16_t us = 0;
};

// Pure parser. False if the line isn't a bench command or is malformed
// (unknown channel, bad number); out->op is kNone then.
bool parse_bench_line(const char* line, const BenchSetpoints& sp,
                      BenchCommand* out);

const char* to_string(BenchOp op);

// Bridge is comms' StateBridge (or a test fake) with
//   arm(bool arm, bool force), bench(bool enter), fault_clear()
// each returning MAV_RESULT (MAV_RESULT_ACCEPTED == 0).
// Returns true if the line was a bench command; reply holds the answer.
template <typename Bridge>
bool bench_console_line(const char* line, uint32_t now_ms, Bridge& bridge,
                        BenchSetpoints& sp, char* reply, size_t len) {
    BenchCommand cmd;
    if (!parse_bench_line(line, sp, &cmd)) {
        return false;
    }
    int result = 0; // MAV_RESULT for mode commands
    bool ok = true;
    switch (cmd.op) {
    case BenchOp::kEnter:
        result = static_cast<int>(bridge.bench(true));
        break;
    case BenchOp::kExit:
        result = static_cast<int>(bridge.bench(false));
        break;
    case BenchOp::kArm:
        result = static_cast<int>(bridge.arm(true, false));
        break;
    case BenchOp::kDisarm:
        result = static_cast<int>(bridge.arm(false, false));
        break;
    case BenchOp::kClear:
        result = static_cast<int>(bridge.fault_clear());
        break;
    case BenchOp::kSetUs:
        ok = sp.set_us(static_cast<uint8_t>(cmd.channel), cmd.us, now_ms);
        break;
    case BenchOp::kCenter:
        sp.center_all();
        break;
    case BenchOp::kSweep:
        ok = sp.sweep(cmd.channel, now_ms);
        break;
    case BenchOp::kNone:
        return false;
    }
    if (result != 0) {
        snprintf(reply, len, "USB %s: rejected (MAV_RESULT %d)",
                 to_string(cmd.op), result);
    } else {
        snprintf(reply, len, "USB %s: %s", to_string(cmd.op),
                 ok ? "ok" : "rejected (range or channel)");
    }
    return true;
}

#endif // BENCH_CONSOLE_H_
