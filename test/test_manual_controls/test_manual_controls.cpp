// Manual controls integration tests: pilot input -> FaultManager -> system
// FSM -> flight FSM -> Actuators, wired in the order of the state README's
// "Usage" section, at 50 Hz simulated time. rc_link is faked by hand.
// Run: pio test -e native -f test_manual_controls

#include "actuators.h"
#include "demand_select.h"
#include "fault_manager.h"
#include "flight_state.h"
#include "pilot.h"
#include "system_state.h"
#include <unity.h>

namespace {

constexpr uint32_t kLoopMs = 20;
enum Ch : uint8_t { kAilL, kAilR, kElev, kRud, kThr, kFlapL, kFlapR, kN };

ChannelConfig surface(const char* name, int8_t rc) {
    ChannelConfig c;
    c.name = name;
    c.max_rate_per_s = 2.0f; // full travel in 1 s
    c.rc_channel = rc;
    return c;
}

ActuatorConfig plane() {
    ActuatorConfig a;
    a.num_channels = kN;
    a.channels[kAilL] = surface("ail_l", 0);
    a.channels[kAilR] = surface("ail_r", 1);
    a.channels[kElev] = surface("elev", 2);
    a.channels[kRud] = surface("rud", 4);
    a.channels[kThr].name = "thr";
    a.channels[kThr].kind = ChannelKind::kThrottle;
    a.channels[kThr].safe_us = 1000;
    a.channels[kThr].rc_channel = 3;
    a.channels[kFlapL] = surface("flap_l", 5);
    a.channels[kFlapR] = surface("flap_r", 5);
    a.mixer = conventional_mixer();
    a.esc.arm_hold_ms = 1000;
    return a;
}

struct Harness {
    FaultManager fm;
    SystemStateFSM sys;
    FlightStateFSM fl;
    Actuators act{plane()};
    PilotParams pp;
    FaultManagerInputs fmi;
    SystemInputs si;
    FlightInputs fi;
    PilotInput rc;
    RcLinkStatus link;
    bool rc_alive = true;
    const SurfaceDemand* control = nullptr;
    BenchFrame bench;
    OutputFrame out;
    uint32_t t = 0;

    Harness() {
        rc.num_raw = 8;
        for (uint8_t i = 0; i < 8; ++i) {
            rc.raw_us[i] = 1500;
        }
        rc.raw_us[3] = 1000; // throttle stick low
        link.link_quality = 100;
    }

    void set_bypass(bool on) {
        const uint16_t bit =
            1u << static_cast<uint8_t>(PilotSwitch::kBypass);
        rc.switches = on ? static_cast<uint16_t>(rc.switches | bit)
                         : static_cast<uint16_t>(rc.switches & ~bit);
    }

    // One control loop, in the Usage order.
    void step() {
        t += kLoopMs;
        if (rc_alive) { // fake rc_link: a frame every loop
            rc.stamp_ms = t;
            link.frames_seen = true;
            link.last_frame_ms = t;
        }
        const bool fresh = pilot_fresh(rc, pp, t);
        fmi.raw = 0;
        if (rc_lost_raw(link, pp, t)) {
            fmi.raw |= fault_bit(FaultId::kRcLost);
        }
        fm.update(fmi, t);
        si.highest_fault = fm.highest_class();
        si.fault_recoverable = fm.recoverable();
        si.landed = fl.is_landed();
        si.preflight_ok = act.preflight_ok(t);
        sys.update(si, t);
        fi.system_state = sys.state();
        fi.critical_response = fm.critical_response();
        fi.manual_active = manual_active(rc, fresh);
        fl.update(fi, t);

        ActuatorInputs ai;
        ai.system = sys.state();
        ai.flight = fl.state();
        ai.manual_active = fi.manual_active;
        ai.rc = &rc;
        ai.rc_fresh = fresh;
        ai.control = control;
        ai.bench = bench;
        act.update(ai, t, &out);

        fmi.fault_clear_cmd = false;
        si.arm_cmd = si.disarm_cmd = si.fault_clear_cmd = false;
        si.bench_cmd = si.exit_bench_cmd = false;
        fi.takeoff_cmd = fi.loiter_cmd = fi.resume_cmd = fi.rtl_cmd = false;
    }

    void run_ms(uint32_t ms) {
        for (uint32_t e = 0; e < ms; e += kLoopMs) {
            step();
        }
    }

    // BOOT -> PREFLIGHT -> (ESC sees stop for arm_hold_ms) -> STANDBY
    void to_standby() {
        si.boot_finished = true;
        run_ms(1200);
    }

    void arm() {
        si.arm_cmd = true;
        step();
    }
};

#define ASSERT_SYS(h, s) \
    TEST_ASSERT_EQUAL_STRING(to_string(s), to_string((h).sys.state()))
#define ASSERT_SRC(h, s) \
    TEST_ASSERT_EQUAL_STRING(to_string(s), to_string((h).out.source))

void assert_all_safe(const Harness& h) {
    uint16_t safe[pwm_out::kMaxChannels] = {};
    h.act.safe_us(safe);
    TEST_ASSERT_EQUAL_UINT16_ARRAY(safe, h.out.us, kN);
    TEST_ASSERT_FALSE(h.out.throttle.armed);
}

} // namespace

void setUp() {}
void tearDown() {}

void test_select_source_table() {
    const SystemState disarmed_ground[] = {
        SystemState::kBoot, SystemState::kPreflight, SystemState::kStandby,
        SystemState::kFaultGround};
    for (SystemState s : disarmed_ground) {
        TEST_ASSERT_EQUAL(OutputSource::kSafe, select_source(s, false));
        TEST_ASSERT_EQUAL(OutputSource::kMirror, select_source(s, true));
    }
    const SystemState armed[] = {SystemState::kActive, SystemState::kDegraded,
                                 SystemState::kCritical,
                                 SystemState::kEmergency};
    for (SystemState s : armed) {
        TEST_ASSERT_EQUAL(OutputSource::kControl, select_source(s, false));
        TEST_ASSERT_EQUAL(OutputSource::kMirror, select_source(s, true));
    }
    TEST_ASSERT_EQUAL(OutputSource::kBench,
                      select_source(SystemState::kBench, false));
    TEST_ASSERT_EQUAL(OutputSource::kMirror,
                      select_source(SystemState::kBench, true));
    TEST_ASSERT_EQUAL(OutputSource::kMirror,
                      select_source(SystemState::kInAirReset, false));
}

void test_bad_config_drives_nothing() {
    ActuatorConfig bad = plane();
    bad.channels[kAilL].kind = ChannelKind::kThrottle; // two throttles
    TEST_ASSERT_FALSE(actuator_config_valid(bad));
    TEST_ASSERT_TRUE(actuator_config_valid(plane()));
    Actuators a(bad);
    TEST_ASSERT_FALSE(a.config_ok());
    OutputFrame f;
    a.update(ActuatorInputs{}, 0, &f);
    TEST_ASSERT_EQUAL_UINT8(0, f.num_channels);
}

void test_boot_outputs_safe_and_preflight_waits_for_esc() {
    Harness h;
    h.si.boot_finished = true;
    h.run_ms(500);
    ASSERT_SYS(h, SystemState::kPreflight); // ESC hasn't seen stop long enough
    ASSERT_SRC(h, OutputSource::kSafe);
    assert_all_safe(h);
    h.run_ms(700);
    ASSERT_SYS(h, SystemState::kStandby);
    assert_all_safe(h);
}

void test_bench_moves_surfaces_but_never_the_motor() {
    Harness h;
    h.to_standby();
    h.si.bench_cmd = true;
    h.step();
    ASSERT_SYS(h, SystemState::kBench);
    ASSERT_SRC(h, OutputSource::kBench);
    TEST_ASSERT_EQUAL_UINT16(1500, h.out.us[kElev]); // unset: trim
    h.bench.us[kElev] = 1800;
    h.bench.us[kThr] = 1900; // must be ignored
    h.bench.set_mask = (1u << kElev) | (1u << kThr);
    h.step();
    TEST_ASSERT_EQUAL_UINT16(1800, h.out.us[kElev]);
    TEST_ASSERT_EQUAL_UINT16(1000, h.out.us[kThr]);
    TEST_ASSERT_FALSE(h.out.throttle.armed);
    h.si.exit_bench_cmd = true;
    h.step();
    ASSERT_SYS(h, SystemState::kPreflight);
    assert_all_safe(h);
}

void test_bench_setpoints_ignored_outside_bench() {
    Harness h;
    h.bench.us[kElev] = 1800;
    h.bench.set_mask = 1u << kElev;
    h.to_standby();
    TEST_ASSERT_EQUAL_UINT16(1500, h.out.us[kElev]);
    h.arm();
    TEST_ASSERT_EQUAL_UINT16(1500, h.out.us[kElev]);
}

void test_armed_without_control_laws_is_safe() {
    Harness h;
    h.to_standby();
    h.arm();
    ASSERT_SYS(h, SystemState::kActive);
    ASSERT_SRC(h, OutputSource::kControl);
    assert_all_safe(h);
}

void test_armed_control_mixes_and_gates_throttle() {
    Harness h;
    SurfaceDemand d;
    d.roll = 0.5f;
    d.throttle = 0.5f;
    h.control = &d;
    h.to_standby();
    h.arm();
    h.run_ms(1000); // let the rate limiters settle
    TEST_ASSERT_EQUAL_UINT16(1750, h.out.us[kAilL]);
    TEST_ASSERT_EQUAL_UINT16(1250, h.out.us[kAilR]);
    // ON_GROUND + ACTIVE: motor allowed (takeoff roll needs it next)
    TEST_ASSERT_TRUE(h.out.throttle.armed);
    TEST_ASSERT_EQUAL_UINT16(1500, h.out.us[kThr]);
}

void test_degraded_on_ground_locks_motor() {
    Harness h;
    SurfaceDemand d;
    d.throttle = 0.5f;
    h.control = &d;
    h.to_standby();
    h.arm();
    h.rc_alive = false; // RC lost: non-critical -> DEGRADED after debounce
    h.run_ms(400);
    ASSERT_SYS(h, SystemState::kDegraded);
    TEST_ASSERT_FALSE(h.out.throttle.armed);
    TEST_ASSERT_EQUAL_UINT16(1000, h.out.us[kThr]);
}

void test_rc_lost_goes_through_fault_manager() {
    Harness h;
    h.to_standby();
    h.arm();
    h.rc_alive = false;
    h.run_ms(100); // stale, but not yet past timeout + debounce
    TEST_ASSERT_FALSE(h.fm.is_active(FaultId::kRcLost));
    h.run_ms(300);
    TEST_ASSERT_TRUE(h.fm.is_active(FaultId::kRcLost));
    h.rc_alive = true;
    h.run_ms(400);
    TEST_ASSERT_FALSE(h.fm.is_active(FaultId::kRcLost));
}

void test_manual_mirrors_receiver_exactly() {
    Harness h;
    h.to_standby();
    h.arm();
    h.rc.raw_us[0] = 1712;
    h.rc.raw_us[2] = 1234;
    h.rc.raw_us[3] = 1600; // pilot throttle
    h.set_bypass(true);
    h.step();
    ASSERT_SRC(h, OutputSource::kMirror);
    TEST_ASSERT_TRUE(h.fi.manual_active);
    TEST_ASSERT_EQUAL_UINT16(1712, h.out.us[kAilL]);
    TEST_ASSERT_EQUAL_UINT16(1234, h.out.us[kElev]);
    TEST_ASSERT_EQUAL_UINT16(1600, h.out.us[kThr]); // armed: mirrored
}

void test_manual_while_disarmed_keeps_motor_stopped() {
    Harness h;
    h.to_standby();
    h.rc.raw_us[3] = 1600;
    h.rc.raw_us[0] = 1700;
    h.set_bypass(true);
    h.step();
    ASSERT_SRC(h, OutputSource::kMirror);
    TEST_ASSERT_EQUAL_UINT16(1700, h.out.us[kAilL]);
    TEST_ASSERT_EQUAL_UINT16(1000, h.out.us[kThr]);
}

void test_handback_from_manual_is_bumpless() {
    Harness h;
    SurfaceDemand d; // control wants neutral
    h.control = &d;
    h.to_standby();
    h.arm();
    h.rc.raw_us[2] = 1900; // pilot holding full-ish up elevator
    h.set_bypass(true);
    h.run_ms(200);
    TEST_ASSERT_EQUAL_UINT16(1900, h.out.us[kElev]);
    h.set_bypass(false);
    h.step();
    ASSERT_SRC(h, OutputSource::kControl);
    // Rate limited from 1900, not snapped to 1500: 2/s * 20 ms = 0.04
    TEST_ASSERT_EQUAL_UINT16(1880, h.out.us[kElev]);
    h.run_ms(1000);
    TEST_ASSERT_EQUAL_UINT16(1500, h.out.us[kElev]);
}

void test_stale_rc_is_not_manual() {
    Harness h;
    h.to_standby();
    h.set_bypass(true);
    h.step();
    TEST_ASSERT_TRUE(h.fi.manual_active);
    h.rc_alive = false;
    h.run_ms(200);
    TEST_ASSERT_FALSE(h.fi.manual_active);
    ASSERT_SRC(h, OutputSource::kSafe);
}

void test_in_air_reset_mirrors_pilot_with_motor_stopped() {
    Harness h;
    h.si.airborne_at_boot = true;
    h.rc.raw_us[0] = 1650;
    h.rc.raw_us[3] = 1700;
    h.step();
    ASSERT_SYS(h, SystemState::kInAirReset);
    ASSERT_SRC(h, OutputSource::kMirror);
    TEST_ASSERT_EQUAL_UINT16(1650, h.out.us[kAilL]);
    TEST_ASSERT_EQUAL_UINT16(1000, h.out.us[kThr]); // disarmed per the FSM
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_select_source_table);
    RUN_TEST(test_bad_config_drives_nothing);
    RUN_TEST(test_boot_outputs_safe_and_preflight_waits_for_esc);
    RUN_TEST(test_bench_moves_surfaces_but_never_the_motor);
    RUN_TEST(test_bench_setpoints_ignored_outside_bench);
    RUN_TEST(test_armed_without_control_laws_is_safe);
    RUN_TEST(test_armed_control_mixes_and_gates_throttle);
    RUN_TEST(test_degraded_on_ground_locks_motor);
    RUN_TEST(test_rc_lost_goes_through_fault_manager);
    RUN_TEST(test_manual_mirrors_receiver_exactly);
    RUN_TEST(test_manual_while_disarmed_keeps_motor_stopped);
    RUN_TEST(test_handback_from_manual_is_bumpless);
    RUN_TEST(test_stale_rc_is_not_manual);
    RUN_TEST(test_in_air_reset_mirrors_pilot_with_motor_stopped);
    return UNITY_END();
}
