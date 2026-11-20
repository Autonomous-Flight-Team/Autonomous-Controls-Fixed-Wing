// Bench tests: setpoints, console parser, and the console driving the real
// system FSM through a fake StateBridge (comms isn't on main yet).
// Run: pio test -e native -f test_bench

#include "actuators.h"
#include "bench_console.h"
#include "bench_setpoints.h"
#include "system_state.h"
#include <cstring>
#include <unity.h>

namespace {

enum Ch : uint8_t { kAil, kElev, kThr, kN };

ChannelConfig g_ch[kN];

void make_channels() {
    g_ch[kAil] = ChannelConfig{};
    g_ch[kAil].name = "ail";
    g_ch[kElev] = ChannelConfig{};
    g_ch[kElev].name = "elev";
    g_ch[kElev].min_us = 1100;
    g_ch[kElev].trim_us = 1450;
    g_ch[kElev].max_us = 1900;
    g_ch[kElev].safe_us = 1450;
    g_ch[kThr] = ChannelConfig{};
    g_ch[kThr].name = "thr";
    g_ch[kThr].kind = ChannelKind::kThrottle;
    g_ch[kThr].safe_us = 1000;
}

// Stand-in for comms' StateBridge: same method shapes, latches one-loop
// pulses that apply() ORs into SystemInputs.
struct FakeBridge {
    int result = 0; // MAV_RESULT to return; 0 = ACCEPTED
    bool bench_pulse = false, exit_pulse = false;
    bool arm_pulse = false, disarm_pulse = false, clear_pulse = false;
    int calls = 0;

    int bench(bool enter) {
        ++calls;
        if (result == 0) {
            (enter ? bench_pulse : exit_pulse) = true;
        }
        return result;
    }
    int arm(bool a, bool force) {
        (void)force;
        ++calls;
        if (result == 0) {
            (a ? arm_pulse : disarm_pulse) = true;
        }
        return result;
    }
    int fault_clear() {
        ++calls;
        clear_pulse = result == 0;
        return result;
    }
    void apply(SystemInputs& si) {
        si.bench_cmd = bench_pulse;
        si.exit_bench_cmd = exit_pulse;
        si.arm_cmd = arm_pulse;
        si.disarm_cmd = disarm_pulse;
        si.fault_clear_cmd = clear_pulse;
        bench_pulse = exit_pulse = arm_pulse = disarm_pulse = false;
        clear_pulse = false;
    }
};

} // namespace

void setUp() { make_channels(); }
void tearDown() {}

// ---------------- Setpoints ----------------

void test_unset_reads_trim() {
    BenchSetpoints sp(g_ch, kN);
    uint16_t us = 0;
    TEST_ASSERT_FALSE(sp.get_us(kElev, &us));
    TEST_ASSERT_EQUAL_UINT16(1450, us);
}

void test_set_us_range_checked() {
    BenchSetpoints sp(g_ch, kN);
    TEST_ASSERT_TRUE(sp.set_us(kElev, 1800, 0));
    uint16_t us = 0;
    TEST_ASSERT_TRUE(sp.get_us(kElev, &us));
    TEST_ASSERT_EQUAL_UINT16(1800, us);
    TEST_ASSERT_FALSE(sp.set_us(kElev, 1950, 0)); // above max
    TEST_ASSERT_FALSE(sp.set_us(kElev, 1000, 0)); // below min
    TEST_ASSERT_FALSE(sp.set_us(7, 1500, 0));     // no such channel
}

void test_throttle_never_accepted() {
    BenchSetpoints sp(g_ch, kN);
    TEST_ASSERT_FALSE(sp.set_us(kThr, 1100, 0));
    TEST_ASSERT_FALSE(sp.sweep(kThr, 0));
    TEST_ASSERT_TRUE(sp.sweep(-1, 0)); // all: surfaces only
    for (uint32_t t = 0; t < BenchSetpoints::kSweepMs; t += 20) {
        sp.update(t);
        TEST_ASSERT_EQUAL_UINT8(0, sp.frame().set_mask & (1u << kThr));
    }
}

void test_center_all() {
    BenchSetpoints sp(g_ch, kN);
    sp.set_us(kAil, 1700, 0);
    sp.center_all();
    TEST_ASSERT_EQUAL_UINT8(0, sp.frame().set_mask);
}

void test_sweep_cycle() {
    BenchSetpoints sp(g_ch, kN);
    sp.sweep(kElev, 1000);
    uint16_t us = 0;
    sp.update(1000 + 1000); // quarter: max
    sp.get_us(kElev, &us);
    TEST_ASSERT_EQUAL_UINT16(1900, us);
    sp.update(1000 + 3000); // three quarters: min
    sp.get_us(kElev, &us);
    TEST_ASSERT_EQUAL_UINT16(1100, us);
    sp.update(1000 + 4000); // done: back to trim, unset
    TEST_ASSERT_FALSE(sp.get_us(kElev, &us));
    TEST_ASSERT_EQUAL_UINT16(1450, us);
}

void test_set_us_cancels_sweep() {
    BenchSetpoints sp(g_ch, kN);
    sp.sweep(kElev, 0);
    sp.set_us(kElev, 1600, 100);
    sp.update(1000);
    uint16_t us = 0;
    sp.get_us(kElev, &us);
    TEST_ASSERT_EQUAL_UINT16(1600, us);
}

// ---------------- Parser ----------------

void test_parse_words() {
    BenchSetpoints sp(g_ch, kN);
    BenchCommand c;
    TEST_ASSERT_TRUE(parse_bench_line("bench", sp, &c));
    TEST_ASSERT_EQUAL(BenchOp::kEnter, c.op);
    TEST_ASSERT_TRUE(parse_bench_line("  bench   exit ", sp, &c));
    TEST_ASSERT_EQUAL(BenchOp::kExit, c.op);
    TEST_ASSERT_TRUE(parse_bench_line("exit", sp, &c));
    TEST_ASSERT_EQUAL(BenchOp::kExit, c.op);
    TEST_ASSERT_TRUE(parse_bench_line("arm", sp, &c));
    TEST_ASSERT_EQUAL(BenchOp::kArm, c.op);
    TEST_ASSERT_TRUE(parse_bench_line("disarm", sp, &c));
    TEST_ASSERT_EQUAL(BenchOp::kDisarm, c.op);
    TEST_ASSERT_TRUE(parse_bench_line("clear", sp, &c));
    TEST_ASSERT_EQUAL(BenchOp::kClear, c.op);
    TEST_ASSERT_TRUE(parse_bench_line("center", sp, &c));
    TEST_ASSERT_EQUAL(BenchOp::kCenter, c.op);
}

void test_parse_us_by_name_and_index() {
    BenchSetpoints sp(g_ch, kN);
    BenchCommand c;
    TEST_ASSERT_TRUE(parse_bench_line("us elev 1650", sp, &c));
    TEST_ASSERT_EQUAL(BenchOp::kSetUs, c.op);
    TEST_ASSERT_EQUAL_INT8(kElev, c.channel);
    TEST_ASSERT_EQUAL_UINT16(1650, c.us);
    TEST_ASSERT_TRUE(parse_bench_line("us 0 1200", sp, &c));
    TEST_ASSERT_EQUAL_INT8(kAil, c.channel);
}

void test_parse_sweep() {
    BenchSetpoints sp(g_ch, kN);
    BenchCommand c;
    TEST_ASSERT_TRUE(parse_bench_line("sweep all", sp, &c));
    TEST_ASSERT_EQUAL_INT8(-1, c.channel);
    TEST_ASSERT_TRUE(parse_bench_line("sweep ail", sp, &c));
    TEST_ASSERT_EQUAL_INT8(kAil, c.channel);
}

void test_parse_rejects_garbage() {
    BenchSetpoints sp(g_ch, kN);
    BenchCommand c;
    const char* bad[] = {"",          "   ",        "KEY?",
                         "us",        "us elev",    "us nope 1500",
                         "us 3 1500", "us elev -5", "us elev 15x0",
                         "us elev 99999",           "sweep",
                         "bench now", "arm please", "a b c d e",
                         "averyveryverylongtoken"};
    for (const char* line : bad) {
        TEST_ASSERT_FALSE_MESSAGE(parse_bench_line(line, sp, &c), line);
        TEST_ASSERT_EQUAL(BenchOp::kNone, c.op);
    }
    TEST_ASSERT_FALSE(parse_bench_line(nullptr, sp, &c));
}

// ---------------- Console ----------------

void test_console_mode_commands_go_through_bridge() {
    BenchSetpoints sp(g_ch, kN);
    FakeBridge b;
    char reply[64];
    TEST_ASSERT_TRUE(bench_console_line("arm", 0, b, sp, reply, 64));
    TEST_ASSERT_TRUE(b.arm_pulse);
    TEST_ASSERT_EQUAL_STRING("USB arm: ok", reply);
    b.result = 4; // MAV_RESULT_FAILED
    TEST_ASSERT_TRUE(bench_console_line("bench", 0, b, sp, reply, 64));
    TEST_ASSERT_FALSE(b.bench_pulse);
    TEST_ASSERT_EQUAL_STRING("USB bench: rejected (MAV_RESULT 4)", reply);
}

void test_console_setpoints_dont_touch_bridge() {
    BenchSetpoints sp(g_ch, kN);
    FakeBridge b;
    char reply[64];
    TEST_ASSERT_TRUE(bench_console_line("us elev 1700", 0, b, sp, reply, 64));
    TEST_ASSERT_EQUAL_INT(0, b.calls);
    TEST_ASSERT_EQUAL_STRING("USB us: ok", reply);
    TEST_ASSERT_TRUE(bench_console_line("us thr 1500", 0, b, sp, reply, 64));
    TEST_ASSERT_EQUAL_STRING("USB us: rejected (range or channel)", reply);
}

void test_console_passes_unknown_lines_on() {
    BenchSetpoints sp(g_ch, kN);
    FakeBridge b;
    char reply[64] = "untouched";
    TEST_ASSERT_FALSE(bench_console_line("STATS", 0, b, sp, reply, 64));
    TEST_ASSERT_EQUAL_STRING("untouched", reply);
}

// Console -> bridge pulse -> system FSM -> BENCH -> setpoints on the outputs.
void test_console_end_to_end() {
    ActuatorConfig cfg;
    cfg.num_channels = kN;
    for (uint8_t i = 0; i < kN; ++i) {
        cfg.channels[i] = g_ch[i];
    }
    cfg.mixer.num_rows = kN;
    cfg.esc.arm_hold_ms = 0;
    Actuators act(cfg);
    TEST_ASSERT_TRUE(act.config_ok());
    BenchSetpoints sp(g_ch, kN);
    FakeBridge b;
    SystemStateFSM sys;
    SystemInputs si;
    si.boot_finished = si.preflight_ok = true;
    OutputFrame out;
    char reply[64];
    uint32_t t = 0;
    auto loop = [&]() {
        t += 20;
        b.apply(si);
        sys.update(si, t);
        sp.update(t);
        ActuatorInputs ai;
        ai.system = sys.state();
        ai.bench = sp.frame();
        act.update(ai, t, &out);
    };
    loop();
    loop(); // STANDBY
    bench_console_line("us elev 1800", t, b, sp, reply, 64);
    loop();
    TEST_ASSERT_EQUAL_UINT16(1450, out.us[kElev]); // not in BENCH yet
    bench_console_line("bench", t, b, sp, reply, 64);
    loop();
    TEST_ASSERT_EQUAL(SystemState::kBench, sys.state());
    TEST_ASSERT_EQUAL_UINT16(1800, out.us[kElev]);
    TEST_ASSERT_EQUAL_UINT16(1000, out.us[kThr]);
    bench_console_line("exit", t, b, sp, reply, 64);
    loop();
    TEST_ASSERT_EQUAL(SystemState::kPreflight, sys.state());
    TEST_ASSERT_EQUAL_UINT16(1450, out.us[kElev]);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_unset_reads_trim);
    RUN_TEST(test_set_us_range_checked);
    RUN_TEST(test_throttle_never_accepted);
    RUN_TEST(test_center_all);
    RUN_TEST(test_sweep_cycle);
    RUN_TEST(test_set_us_cancels_sweep);
    RUN_TEST(test_parse_words);
    RUN_TEST(test_parse_us_by_name_and_index);
    RUN_TEST(test_parse_sweep);
    RUN_TEST(test_parse_rejects_garbage);
    RUN_TEST(test_console_mode_commands_go_through_bridge);
    RUN_TEST(test_console_setpoints_dont_touch_bridge);
    RUN_TEST(test_console_passes_unknown_lines_on);
    RUN_TEST(test_console_end_to_end);
    return UNITY_END();
}
