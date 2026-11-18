// Native loopback tests for the comms subsystem: `pio test -e native`.
// A fake ground station lives in this file on MAVLink channel 1 and talks to comms
// through an in-memory pipe.

#include <unity.h>

#include <stdlib.h>
#include <string.h>
#include <vector>

#include "comms.h"
#include "comms_platform.h"
#include "signing.h"

// Obviously fake key, used only by these tests.
static const uint8_t TEST_KEY[32] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a,
                                     0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
                                     0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f};
static const char *TEST_KEY_HEX = "000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f";
static const uint8_t WRONG_KEY[32] = {0xde, 0xad, 0xbe, 0xef};
static const float FORCE_MAGIC = 21196.0f;

static const uint8_t GCS_CHAN = MAVLINK_COMM_1;
static const uint8_t GCS_SYS = 255, GCS_COMP = MAV_COMP_ID_MISSIONPLANNER;
static const uint64_t GCS_WALL_TICKS = 3000000000000ULL; // ~ late 2024 in 10 us ticks since 2015

static PipeRing g_up, g_down; // up: ground -> aircraft, down: aircraft -> ground
static MemoryByteStream g_vehicle_end(g_up, g_down);
static MemoryByteStream g_gcs_end(g_down, g_up);
static DummyTelemetrySource g_tel;
static StubCommandHandler g_handler(g_tel.state());

static mavlink_signing_t g_gcs_signing;
static mavlink_signing_streams_t g_gcs_streams;
static std::vector<mavlink_message_t> g_rx; // everything the ground received
static uint32_t g_gcs_bad = 0;              // frames the ground rejected (CRC or signature)

// ------------------------------------------------------------------ fake ground station

static void gcs_setup(const uint8_t *key)
{
    mavlink_status_t *st = mavlink_get_channel_status(GCS_CHAN);
    memset(st, 0, sizeof(*st));
    memset(&g_gcs_signing, 0, sizeof(g_gcs_signing));
    memset(&g_gcs_streams, 0, sizeof(g_gcs_streams));
    if (key)
    {
        memcpy(g_gcs_signing.secret_key, key, 32);
        g_gcs_signing.flags = MAVLINK_SIGNING_FLAG_SIGN_OUTGOING;
        g_gcs_signing.link_id = 0;
        g_gcs_signing.timestamp = GCS_WALL_TICKS;
        st->signing = &g_gcs_signing;
        st->signing_streams = &g_gcs_streams;
    }
}

static void gcs_set_key(const uint8_t *key) // change key, keep the ground's clock
{
    memcpy(g_gcs_signing.secret_key, key, 32);
}

static void gcs_set_signing_enabled(bool on)
{
    mavlink_status_t *st = mavlink_get_channel_status(GCS_CHAN);
    st->signing = on ? &g_gcs_signing : nullptr;
    st->signing_streams = on ? &g_gcs_streams : nullptr;
}

static std::vector<uint8_t> to_bytes(const mavlink_message_t &m)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t n = mavlink_msg_to_send_buffer(buf, &m);
    return std::vector<uint8_t>(buf, buf + n);
}

static void gcs_write(const std::vector<uint8_t> &b) { g_gcs_end.write(b.data(), b.size()); }

static std::vector<uint8_t> gcs_heartbeat_bytes()
{
    mavlink_message_t m;
    mavlink_msg_heartbeat_pack_chan(GCS_SYS, GCS_COMP, GCS_CHAN, &m, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
    return to_bytes(m);
}

static std::vector<uint8_t> gcs_command_bytes(uint16_t cmd, float p1 = 0, float p2 = 0, float p3 = 0, float p7 = 0,
                                              uint8_t confirmation = 0, uint8_t target = COMMS_SYSID)
{
    mavlink_message_t m;
    mavlink_msg_command_long_pack_chan(GCS_SYS, GCS_COMP, GCS_CHAN, &m, target, COMMS_COMPID, cmd, confirmation,
                                       p1, p2, p3, 0, 0, 0, p7);
    return to_bytes(m);
}

static void gcs_heartbeat() { gcs_write(gcs_heartbeat_bytes()); }
static void gcs_command(uint16_t cmd, float p1 = 0, float p2 = 0, float p3 = 0, float p7 = 0, uint8_t confirmation = 0)
{
    gcs_write(gcs_command_bytes(cmd, p1, p2, p3, p7, confirmation));
}

static void gcs_collect()
{
    mavlink_message_t m;
    mavlink_status_t s;
    int c;
    while ((c = g_down.pop()) >= 0)
    {
        uint8_t r = mavlink_frame_char(GCS_CHAN, (uint8_t)c, &m, &s);
        if (r == MAVLINK_FRAMING_OK)
            g_rx.push_back(m);
        else if (r == MAVLINK_FRAMING_BAD_CRC || r == MAVLINK_FRAMING_BAD_SIGNATURE)
            g_gcs_bad++;
    }
}

// Run comms for `ms` milliseconds in 5 ms loop steps, the ground draining the link.
static void run_ms(uint32_t ms, bool ground_heartbeats = false)
{
    for (uint32_t t = 0; t < ms; t += 5)
    {
        if (ground_heartbeats && (comms_time_us() / 1000) % 500 == 0)
            gcs_heartbeat();
        comms_test_advance_us(5000);
        g_tel.update((uint32_t)(comms_time_us() / 1000));
        comms_update();
        gcs_collect();
    }
}

static int count_rx(uint32_t msgid)
{
    int n = 0;
    for (auto &m : g_rx)
        n += (m.msgid == msgid);
    return n;
}

static const mavlink_message_t *last_rx(uint32_t msgid)
{
    for (auto it = g_rx.rbegin(); it != g_rx.rend(); ++it)
        if (it->msgid == msgid)
            return &*it;
    return nullptr;
}

static int last_ack_result(uint16_t command)
{
    for (auto it = g_rx.rbegin(); it != g_rx.rend(); ++it)
        if (it->msgid == MAVLINK_MSG_ID_COMMAND_ACK && mavlink_msg_command_ack_get_command(&*it) == command)
            return mavlink_msg_command_ack_get_result(&*it);
    return -1;
}

static int count_acks(uint16_t command)
{
    int n = 0;
    for (auto &m : g_rx)
        n += (m.msgid == MAVLINK_MSG_ID_COMMAND_ACK && mavlink_msg_command_ack_get_command(&m) == command);
    return n;
}

static uint64_t signature_timestamp(const mavlink_message_t &m)
{
    uint64_t t = 0;
    memcpy(&t, &m.signature[1], 6);
    return t;
}

// Boot the aircraft side. `key` is provisioned into the fake EEPROM first (nullptr = none).
static void boot(const uint8_t *key, bool require_signing)
{
    if (key)
        signing_save_key(key);
    CommsConfig cfg;
    cfg.require_signing = require_signing;
    comms_init(g_vehicle_end, g_tel, g_handler, cfg);
}

void setUp()
{
    comms_test_wipe_store();
    comms_test_set_time_us(1000000);
    g_up.clear();
    g_down.clear();
    g_vehicle_end.write_limit = SIZE_MAX;
    g_tel = DummyTelemetrySource();
    g_handler.calls = 0;
    g_rx.clear();
    g_gcs_bad = 0;
    gcs_setup(nullptr);
}

void tearDown() {}

// ------------------------------------------------------------------ tests

static void test_heartbeat_both_ways_and_link_timeout()
{
    boot(nullptr, false);
    run_ms(100);
    const mavlink_message_t *hb = last_rx(MAVLINK_MSG_ID_HEARTBEAT);
    TEST_ASSERT_NOT_NULL(hb);
    TEST_ASSERT_EQUAL(MAV_TYPE_FIXED_WING, mavlink_msg_heartbeat_get_type(hb));
    TEST_ASSERT_EQUAL(MAV_STATE_STANDBY, mavlink_msg_heartbeat_get_system_status(hb));
    TEST_ASSERT_EQUAL_UINT32(GOOSE_SYSTEM_STATE_STANDBY << 8, mavlink_msg_heartbeat_get_custom_mode(hb));

    TEST_ASSERT_FALSE(comms_gcs_link_ok());
    gcs_heartbeat();
    run_ms(10);
    TEST_ASSERT_TRUE(comms_gcs_link_ok());
    run_ms(2900);
    TEST_ASSERT_TRUE(comms_gcs_link_ok());
    run_ms(200);
    TEST_ASSERT_FALSE(comms_gcs_link_ok());

    // a non-GCS heartbeat (another vehicle) does not count
    mavlink_message_t m;
    mavlink_msg_heartbeat_pack_chan(7, 1, GCS_CHAN, &m, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4, 0, 0, MAV_STATE_ACTIVE);
    gcs_write(to_bytes(m));
    run_ms(10);
    TEST_ASSERT_FALSE(comms_gcs_link_ok());
}

static void test_round_trip_every_message()
{
    boot(TEST_KEY, true);
    gcs_setup(TEST_KEY);
    gcs_heartbeat();
    gcs_command(MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_SCALED_IMU, 200000);
    gcs_command(MAV_CMD_GET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_ATTITUDE);
    uint8_t imu[12] = {1, 0, 0, 0, 2, 0, 0, 0, 3, 0, 0, 0};
    comms_forward_log(COMMS_LOG_SYS_SENSOR, COMMS_LOG_SUB_IMU, COMMS_LOG_INFO, 1234, imu, sizeof(imu));
    uint8_t baro[4] = {9, 9, 9, 9};
    comms_forward_log(COMMS_LOG_SYS_SENSOR, COMMS_LOG_SUB_BARO, COMMS_LOG_WARN, 1300, baro, sizeof(baro));
    run_ms(2000);

    TEST_ASSERT_EQUAL_UINT32(0, g_gcs_bad);
    for (uint32_t id : {MAVLINK_MSG_ID_HEARTBEAT, MAVLINK_MSG_ID_SYS_STATUS, MAVLINK_MSG_ID_ATTITUDE,
                        MAVLINK_MSG_ID_GLOBAL_POSITION_INT, MAVLINK_MSG_ID_GPS_RAW_INT, MAVLINK_MSG_ID_VFR_HUD,
                        MAVLINK_MSG_ID_SCALED_IMU, MAVLINK_MSG_ID_SCALED_PRESSURE, MAVLINK_MSG_ID_BATTERY_STATUS,
                        MAVLINK_MSG_ID_STATUSTEXT, MAVLINK_MSG_ID_COMMAND_ACK, MAVLINK_MSG_ID_MESSAGE_INTERVAL,
                        MAVLINK_MSG_ID_GOOSE_STATE, MAVLINK_MSG_ID_GOOSE_LOG})
    {
        char what[48];
        snprintf(what, sizeof(what), "message id %u missing", (unsigned)id);
        TEST_ASSERT_TRUE_MESSAGE(count_rx(id) > 0, what);
    }
    for (auto &m : g_rx)
        TEST_ASSERT_TRUE_MESSAGE(m.incompat_flags & MAVLINK_IFLAG_SIGNED, "every downlink frame is signed");

    mavlink_goose_state_t gs;
    mavlink_msg_goose_state_decode(last_rx(MAVLINK_MSG_ID_GOOSE_STATE), &gs);
    TEST_ASSERT_EQUAL(GOOSE_SYSTEM_STATE_STANDBY, gs.system_state);
    TEST_ASSERT_EQUAL(GOOSE_FLIGHT_STATE_ON_GROUND, gs.flight_state);
    TEST_ASSERT_EQUAL(GOOSE_SENSOR_STATUS_NOMINAL, gs.sensor_status[GOOSE_SENSOR_INDEX_IMU]);
    TEST_ASSERT_EQUAL(GOOSE_SENSOR_STATUS_UNINITIALIZED, gs.sensor_status[GOOSE_SENSOR_INDEX_SPARE]);

    mavlink_attitude_t att;
    AttitudeSample want;
    mavlink_msg_attitude_decode(last_rx(MAVLINK_MSG_ID_ATTITUDE), &att);
    g_tel.attitude(want);
    TEST_ASSERT_FLOAT_WITHIN(0.2f, want.roll, att.roll);

    mavlink_global_position_int_t pos;
    mavlink_msg_global_position_int_decode(last_rx(MAVLINK_MSG_ID_GLOBAL_POSITION_INT), &pos);
    TEST_ASSERT_INT32_WITHIN(10000, 476530000, pos.lat);

    bool saw_info = false, saw_warn = false;
    for (auto &m : g_rx)
    {
        if (m.msgid != MAVLINK_MSG_ID_GOOSE_LOG)
            continue;
        mavlink_goose_log_t log;
        mavlink_msg_goose_log_decode(&m, &log);
        if (log.subsystem == COMMS_LOG_SUB_IMU)
        {
            saw_info = true;
            TEST_ASSERT_EQUAL(sizeof(imu), log.len);
            TEST_ASSERT_EQUAL_UINT32(1234, log.time_boot_ms);
            TEST_ASSERT_EQUAL_UINT8_ARRAY(imu, log.payload, sizeof(imu));
        }
        if (log.subsystem == COMMS_LOG_SUB_BARO)
        {
            saw_warn = true;
            TEST_ASSERT_EQUAL(COMMS_LOG_WARN, log.level);
            TEST_ASSERT_EQUAL_UINT8_ARRAY(baro, log.payload, sizeof(baro));
        }
    }
    TEST_ASSERT_TRUE(saw_info);
    TEST_ASSERT_TRUE(saw_warn);

    mavlink_message_interval_t mi;
    mavlink_msg_message_interval_decode(last_rx(MAVLINK_MSG_ID_MESSAGE_INTERVAL), &mi);
    TEST_ASSERT_EQUAL(MAVLINK_MSG_ID_ATTITUDE, mi.message_id);
    TEST_ASSERT_EQUAL_INT32(COMMS_RATE_ATTITUDE_US, mi.interval_us);

    char text[51] = {};
    mavlink_msg_statustext_get_text(last_rx(MAVLINK_MSG_ID_STATUSTEXT), text);
    TEST_ASSERT_NOT_NULL(strstr(text, "WARN"));
}

static void test_commands_guards_and_acks()
{
    boot(nullptr, false);
    gcs_command(MAV_CMD_NAV_TAKEOFF); // not armed yet
    run_ms(20);
    TEST_ASSERT_EQUAL(MAV_RESULT_DENIED, last_ack_result(MAV_CMD_NAV_TAKEOFF));

    gcs_command(MAV_CMD_COMPONENT_ARM_DISARM, 1);
    run_ms(20);
    TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, last_ack_result(MAV_CMD_COMPONENT_ARM_DISARM));
    TEST_ASSERT_EQUAL(GOOSE_SYSTEM_STATE_ACTIVE, g_tel.state().system_state);

    gcs_command(MAV_CMD_NAV_RETURN_TO_LAUNCH); // on the ground
    run_ms(20);
    TEST_ASSERT_EQUAL(MAV_RESULT_DENIED, last_ack_result(MAV_CMD_NAV_RETURN_TO_LAUNCH));

    gcs_command(MAV_CMD_NAV_TAKEOFF, 10, 0, 0, 50);
    run_ms(20);
    TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, last_ack_result(MAV_CMD_NAV_TAKEOFF));
    TEST_ASSERT_EQUAL(GOOSE_FLIGHT_STATE_TAKEOFF_ROLL, g_tel.state().flight_state);

    run_ms(11000); // dummy flight: TAKEOFF_ROLL -> CLIMB -> CRUISE
    TEST_ASSERT_EQUAL(GOOSE_FLIGHT_STATE_CRUISE, g_tel.state().flight_state);
    gcs_command(MAV_CMD_COMPONENT_ARM_DISARM, 0, FORCE_MAGIC); // disarm in the air, even forced
    gcs_command(MAV_CMD_NAV_LOITER_UNLIM);
    run_ms(20);
    TEST_ASSERT_EQUAL(MAV_RESULT_DENIED, last_ack_result(MAV_CMD_COMPONENT_ARM_DISARM));
    TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, last_ack_result(MAV_CMD_NAV_LOITER_UNLIM));
    TEST_ASSERT_EQUAL(GOOSE_FLIGHT_STATE_LOITER, g_tel.state().flight_state);
    gcs_command(MAV_CMD_GOOSE_RESUME_MISSION);
    run_ms(20);
    TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, last_ack_result(MAV_CMD_GOOSE_RESUME_MISSION));
    TEST_ASSERT_EQUAL(GOOSE_FLIGHT_STATE_CRUISE, g_tel.state().flight_state);
    gcs_command(MAV_CMD_NAV_RETURN_TO_LAUNCH);
    run_ms(20);
    TEST_ASSERT_EQUAL(GOOSE_FLIGHT_STATE_RTL, g_tel.state().flight_state);

    // the state change shows up in GOOSE_STATE right away, with the ARMED flag
    mavlink_goose_state_t gs;
    mavlink_msg_goose_state_decode(last_rx(MAVLINK_MSG_ID_GOOSE_STATE), &gs);
    TEST_ASSERT_EQUAL(GOOSE_FLIGHT_STATE_RTL, gs.flight_state);
    TEST_ASSERT_EQUAL(GOOSE_STATE_FLAGS_ARMED, gs.flags);

    // no land input in the flight FSM: LAND is not supported, landing comes from RTL
    gcs_command(MAV_CMD_NAV_LAND);
    run_ms(20);
    TEST_ASSERT_EQUAL(MAV_RESULT_UNSUPPORTED, last_ack_result(MAV_CMD_NAV_LAND));
    TEST_ASSERT_EQUAL(GOOSE_FLIGHT_STATE_RTL, g_tel.state().flight_state);

    run_ms(15100); // dummy: RTL -> DESCENT -> LANDING -> LANDED
    TEST_ASSERT_EQUAL(GOOSE_FLIGHT_STATE_LANDED, g_tel.state().flight_state);
    gcs_command(MAV_CMD_COMPONENT_ARM_DISARM, 0);
    run_ms(20);
    TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, last_ack_result(MAV_CMD_COMPONENT_ARM_DISARM));
    TEST_ASSERT_EQUAL(GOOSE_SYSTEM_STATE_STANDBY, g_tel.state().system_state);

    // bench enter / exit, and the param check
    gcs_command(MAV_CMD_GOOSE_BENCH, 2);
    run_ms(20);
    TEST_ASSERT_EQUAL(MAV_RESULT_DENIED, last_ack_result(MAV_CMD_GOOSE_BENCH));
    gcs_command(MAV_CMD_GOOSE_BENCH, 1);
    run_ms(20);
    TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, last_ack_result(MAV_CMD_GOOSE_BENCH));
    TEST_ASSERT_EQUAL(GOOSE_SYSTEM_STATE_BENCH, g_tel.state().system_state);
    gcs_command(MAV_CMD_COMPONENT_ARM_DISARM, 1); // no arming from BENCH
    run_ms(20);
    TEST_ASSERT_EQUAL(MAV_RESULT_DENIED, last_ack_result(MAV_CMD_COMPONENT_ARM_DISARM));
    gcs_command(MAV_CMD_GOOSE_BENCH, 0);
    run_ms(20);
    TEST_ASSERT_EQUAL(GOOSE_SYSTEM_STATE_PREFLIGHT, g_tel.state().system_state);

    // fault clear only from FAULT_GROUND / IN_AIR_RESET
    gcs_command(MAV_CMD_GOOSE_FAULT_CLEAR);
    run_ms(20);
    TEST_ASSERT_EQUAL(MAV_RESULT_DENIED, last_ack_result(MAV_CMD_GOOSE_FAULT_CLEAR));
    g_tel.state().system_state = GOOSE_SYSTEM_STATE_FAULT_GROUND;
    gcs_command(MAV_CMD_GOOSE_FAULT_CLEAR);
    run_ms(20);
    TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, last_ack_result(MAV_CMD_GOOSE_FAULT_CLEAR));
    TEST_ASSERT_EQUAL(GOOSE_SYSTEM_STATE_PREFLIGHT, g_tel.state().system_state);

    gcs_command(MAV_CMD_DO_SET_MODE);
    run_ms(20);
    TEST_ASSERT_EQUAL(MAV_RESULT_UNSUPPORTED, last_ack_result(MAV_CMD_DO_SET_MODE));

    mavlink_message_t m;
    mavlink_msg_command_int_pack_chan(GCS_SYS, GCS_COMP, GCS_CHAN, &m, COMMS_SYSID, COMMS_COMPID, 0,
                                      MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    gcs_write(to_bytes(m));
    run_ms(20);
    TEST_ASSERT_EQUAL(MAV_RESULT_COMMAND_LONG_ONLY, last_ack_result(MAV_CMD_NAV_RETURN_TO_LAUNCH));

    int acks = count_acks(MAV_CMD_GOOSE_RESUME_MISSION);
    gcs_write(gcs_command_bytes(MAV_CMD_GOOSE_RESUME_MISSION, 0, 0, 0, 0, 0, 42)); // for another aircraft
    run_ms(20);
    TEST_ASSERT_EQUAL(acks, count_acks(MAV_CMD_GOOSE_RESUME_MISSION));
}

static void test_heartbeat_armed_set_and_manual_flag()
{
    boot(nullptr, false);
    // IN_AIR_RESET (20) is numerically above the armed states but disarmed
    g_tel.state().system_state = GOOSE_SYSTEM_STATE_IN_AIR_RESET;
    g_tel.state().flags = GOOSE_STATE_FLAGS_MANUAL_ACTIVE | GOOSE_STATE_FLAGS_ARMED; // a wrong ARMED is overridden
    run_ms(1100);
    const mavlink_message_t *hb = last_rx(MAVLINK_MSG_ID_HEARTBEAT);
    TEST_ASSERT_NOT_NULL(hb);
    uint8_t base = mavlink_msg_heartbeat_get_base_mode(hb);
    TEST_ASSERT_EQUAL(0, base & MAV_MODE_FLAG_SAFETY_ARMED);
    TEST_ASSERT_TRUE(base & MAV_MODE_FLAG_MANUAL_INPUT_ENABLED);
    TEST_ASSERT_EQUAL(MAV_STATE_CRITICAL, mavlink_msg_heartbeat_get_system_status(hb));
    mavlink_goose_state_t gs;
    mavlink_msg_goose_state_decode(last_rx(MAVLINK_MSG_ID_GOOSE_STATE), &gs);
    TEST_ASSERT_EQUAL(GOOSE_SYSTEM_STATE_IN_AIR_RESET, gs.system_state);
    TEST_ASSERT_EQUAL(GOOSE_STATE_FLAGS_MANUAL_ACTIVE, gs.flags);

    for (uint8_t st : {GOOSE_SYSTEM_STATE_ACTIVE, GOOSE_SYSTEM_STATE_DEGRADED, GOOSE_SYSTEM_STATE_CRITICAL,
                       GOOSE_SYSTEM_STATE_EMERGENCY})
    {
        g_tel.state().system_state = st;
        g_tel.state().flags = 0;
        g_rx.clear();
        run_ms(1100);
        TEST_ASSERT_TRUE(mavlink_msg_heartbeat_get_base_mode(last_rx(MAVLINK_MSG_ID_HEARTBEAT)) & MAV_MODE_FLAG_SAFETY_ARMED);
        mavlink_msg_goose_state_decode(last_rx(MAVLINK_MSG_ID_GOOSE_STATE), &gs);
        TEST_ASSERT_EQUAL(GOOSE_STATE_FLAGS_ARMED, gs.flags);
    }
}

static void test_duplicate_suppression()
{
    boot(nullptr, false);
    gcs_command(MAV_CMD_COMPONENT_ARM_DISARM, 1, 0, 0, 0, 0);
    run_ms(20);
    gcs_command(MAV_CMD_COMPONENT_ARM_DISARM, 1, 0, 0, 0, 1); // ground retry (ack "lost")
    gcs_command(MAV_CMD_COMPONENT_ARM_DISARM, 1, 0, 0, 0, 2);
    run_ms(20);
    TEST_ASSERT_EQUAL_UINT32(1, g_handler.calls);
    TEST_ASSERT_EQUAL_UINT32(2, comms_stats().cmd_duplicates);
    TEST_ASSERT_EQUAL(3, count_acks(MAV_CMD_COMPONENT_ARM_DISARM));
    TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, last_ack_result(MAV_CMD_COMPONENT_ARM_DISARM));

    // a new attempt (confirmation 0) is a new command
    gcs_command(MAV_CMD_COMPONENT_ARM_DISARM, 1, 0, 0, 0, 0);
    run_ms(20);
    TEST_ASSERT_EQUAL_UINT32(2, g_handler.calls);

    // a retry outside the window is executed
    run_ms(COMMS_CMD_DUP_WINDOW_MS + 100);
    gcs_command(MAV_CMD_COMPONENT_ARM_DISARM, 1, 0, 0, 0, 1);
    run_ms(20);
    TEST_ASSERT_EQUAL_UINT32(3, g_handler.calls);
}

static void test_rate_changes()
{
    boot(nullptr, false);
    gcs_command(MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_ATTITUDE, 200000); // 5 Hz
    run_ms(20);
    TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, last_ack_result(MAV_CMD_SET_MESSAGE_INTERVAL));
    TEST_ASSERT_EQUAL_UINT32(200000, comms_stream_interval_us(MAVLINK_MSG_ID_ATTITUDE));
    g_rx.clear();
    run_ms(2000);
    int n = count_rx(MAVLINK_MSG_ID_ATTITUDE);
    TEST_ASSERT_INT_WITHIN(1, 10, n);

    gcs_command(MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_ATTITUDE, -1); // off
    run_ms(20);
    g_rx.clear();
    run_ms(1000);
    TEST_ASSERT_EQUAL(0, count_rx(MAVLINK_MSG_ID_ATTITUDE));

    gcs_command(MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_ATTITUDE, 0); // default
    run_ms(20);
    TEST_ASSERT_EQUAL_UINT32(COMMS_RATE_ATTITUDE_US, comms_stream_interval_us(MAVLINK_MSG_ID_ATTITUDE));

    gcs_command(MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_ATTITUDE, 10); // clamped to 50 Hz
    run_ms(20);
    TEST_ASSERT_EQUAL_UINT32(COMMS_MIN_INTERVAL_US, comms_stream_interval_us(MAVLINK_MSG_ID_ATTITUDE));
    gcs_command(MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_ATTITUDE, 0);

    gcs_command(MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_HEARTBEAT, 100000);
    run_ms(20);
    TEST_ASSERT_EQUAL(MAV_RESULT_DENIED, last_ack_result(MAV_CMD_SET_MESSAGE_INTERVAL));

    // per-logger-stream rate: IMU log at 2 Hz while the logger produces 100 Hz
    gcs_command(MAV_CMD_GOOSE_SET_LOG_RATE, COMMS_LOG_SYS_SENSOR, COMMS_LOG_SUB_IMU, 500000);
    run_ms(20);
    TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, last_ack_result(MAV_CMD_GOOSE_SET_LOG_RATE));
    TEST_ASSERT_EQUAL_UINT32(500000, comms_log_interval_us(COMMS_LOG_SYS_SENSOR, COMMS_LOG_SUB_IMU));
    g_rx.clear();
    uint32_t last_value = 0;
    for (uint32_t i = 0; i < 200; i++) // 2 s at 100 Hz
    {
        last_value = i;
        comms_forward_log(COMMS_LOG_SYS_SENSOR, COMMS_LOG_SUB_IMU, COMMS_LOG_INFO, i * 10, &i, sizeof(i));
        run_ms(10);
    }
    TEST_ASSERT_INT_WITHIN(1, 4, count_rx(MAVLINK_MSG_ID_GOOSE_LOG));
    mavlink_goose_log_t log;
    mavlink_msg_goose_log_decode(last_rx(MAVLINK_MSG_ID_GOOSE_LOG), &log);
    uint32_t got;
    memcpy(&got, log.payload, 4);
    TEST_ASSERT_UINT32_WITHIN(60, last_value, got); // a recent sample, not the first one

    gcs_command(MAV_CMD_GOOSE_SET_LOG_RATE, COMMS_LOG_SYS_SENSOR, COMMS_LOG_SUB_IMU, -1);
    run_ms(20);
    g_rx.clear();
    for (uint32_t i = 0; i < 100; i++)
    {
        comms_forward_log(COMMS_LOG_SYS_SENSOR, COMMS_LOG_SUB_IMU, COMMS_LOG_INFO, i, &i, sizeof(i));
        run_ms(10);
    }
    TEST_ASSERT_EQUAL(0, count_rx(MAVLINK_MSG_ID_GOOSE_LOG));
    // ...but WARN/ERROR always go out
    comms_forward_log(COMMS_LOG_SYS_SENSOR, COMMS_LOG_SUB_IMU, COMMS_LOG_ERROR, 5, nullptr, 0);
    run_ms(20);
    TEST_ASSERT_EQUAL(1, count_rx(MAVLINK_MSG_ID_GOOSE_LOG));
    TEST_ASSERT_EQUAL(1, count_rx(MAVLINK_MSG_ID_STATUSTEXT));
    TEST_ASSERT_FALSE(comms_forward_log(0, 1, COMMS_LOG_INFO, 0, "x", 65)); // over 64 bytes
}

static void test_signing_accepts_signed_and_survives_reboot()
{
    boot(TEST_KEY, true);
    gcs_setup(TEST_KEY);
    TEST_ASSERT_TRUE(comms_signing_key_loaded());
    gcs_heartbeat();
    run_ms(50);
    TEST_ASSERT_TRUE(comms_gcs_link_ok());
    gcs_command(MAV_CMD_COMPONENT_ARM_DISARM, 1);
    run_ms(2000);
    TEST_ASSERT_EQUAL(MAV_RESULT_ACCEPTED, last_ack_result(MAV_CMD_COMPONENT_ARM_DISARM));
    TEST_ASSERT_EQUAL_UINT32(0, g_gcs_bad);
    // the aircraft adopted the ground's (newer) clock and saved it
    TEST_ASSERT_TRUE(comms_signing_timestamp() >= GCS_WALL_TICKS);
    TEST_ASSERT_TRUE(comms_stats().timestamp_saves >= 1);

    uint64_t last_ts = signature_timestamp(g_rx.back());

    // Power cycle: clock restarts at 0, EEPROM keeps key and timestamp. The ground
    // keeps its stream table, so frames older than last_ts would be rejected.
    comms_test_set_time_us(0);
    g_rx.clear();
    boot(nullptr, true);
    run_ms(1500); // no ground traffic yet, so no help from the ground's clock
    TEST_ASSERT_TRUE(count_rx(MAVLINK_MSG_ID_HEARTBEAT) > 0);
    TEST_ASSERT_TRUE(signature_timestamp(g_rx.front()) > last_ts);
    TEST_ASSERT_EQUAL_UINT32(0, g_gcs_bad);
}

static void test_signing_rejects_unsigned_wrong_key_and_replay()
{
    boot(TEST_KEY, true);
    gcs_setup(TEST_KEY);

    // unsigned
    gcs_set_signing_enabled(false);
    gcs_heartbeat();
    gcs_command(MAV_CMD_COMPONENT_ARM_DISARM, 1);
    run_ms(50);
    TEST_ASSERT_FALSE(comms_gcs_link_ok());
    TEST_ASSERT_EQUAL_UINT32(0, g_handler.calls);
    TEST_ASSERT_EQUAL(-1, last_ack_result(MAV_CMD_COMPONENT_ARM_DISARM));
    TEST_ASSERT_EQUAL_UINT32(2, comms_stats().rx_unsigned_dropped);

    // wrong key
    gcs_set_signing_enabled(true);
    gcs_set_key(WRONG_KEY);
    gcs_heartbeat();
    gcs_command(MAV_CMD_COMPONENT_ARM_DISARM, 1);
    run_ms(50);
    TEST_ASSERT_FALSE(comms_gcs_link_ok());
    TEST_ASSERT_EQUAL_UINT32(0, g_handler.calls);
    TEST_ASSERT_EQUAL_UINT32(2, comms_stats().rx_bad_signature);

    // replay: a correctly signed ARM, captured and sent again
    gcs_set_key(TEST_KEY);
    std::vector<uint8_t> arm = gcs_command_bytes(MAV_CMD_COMPONENT_ARM_DISARM, 1);
    gcs_write(arm);
    run_ms(50);
    TEST_ASSERT_EQUAL_UINT32(1, g_handler.calls);
    gcs_command(MAV_CMD_COMPONENT_ARM_DISARM, 0); // disarm, so a replayed ARM would change state
    run_ms(50);
    TEST_ASSERT_EQUAL(GOOSE_SYSTEM_STATE_STANDBY, g_tel.state().system_state);
    gcs_write(arm);
    run_ms(50);
    TEST_ASSERT_EQUAL_UINT32(2, g_handler.calls);
    TEST_ASSERT_EQUAL(GOOSE_SYSTEM_STATE_STANDBY, g_tel.state().system_state);
    TEST_ASSERT_EQUAL_UINT32(3, comms_stats().rx_bad_signature);
}

static void test_radio_status_allowlist()
{
    boot(TEST_KEY, true);
    mavlink_message_t m;
    // unsigned RADIO_STATUS as the SiK radio injects it
    mavlink_msg_radio_status_pack_chan('3', 'D', GCS_CHAN, &m, 150, 140, 37, 20, 25, 0, 0);
    gcs_write(to_bytes(m));
    run_ms(20);
    TEST_ASSERT_EQUAL_UINT8(37, comms_stats().radio_txbuf);
    TEST_ASSERT_EQUAL_UINT8(150, comms_stats().radio_rssi);

    // the same message from anyone else is not allowlisted
    mavlink_msg_radio_status_pack_chan(GCS_SYS, GCS_COMP, GCS_CHAN, &m, 1, 1, 99, 1, 1, 0, 0);
    gcs_write(to_bytes(m));
    run_ms(20);
    TEST_ASSERT_EQUAL_UINT8(37, comms_stats().radio_txbuf);
    TEST_ASSERT_EQUAL_UINT32(1, comms_stats().rx_unsigned_dropped);
}

static void test_no_key_with_required_signing_never_links()
{
    boot(nullptr, true);
    TEST_ASSERT_FALSE(comms_signing_key_loaded());
    gcs_heartbeat();
    run_ms(100);
    TEST_ASSERT_FALSE(comms_gcs_link_ok());
    char text[51] = {};
    const mavlink_message_t *st = last_rx(MAVLINK_MSG_ID_STATUSTEXT);
    TEST_ASSERT_NOT_NULL(st);
    mavlink_msg_statustext_get_text(st, text);
    TEST_ASSERT_NOT_NULL(strstr(text, "no signing key"));
}

static void test_console_provisioning()
{
    boot(nullptr, true);
    char reply[96];
    TEST_ASSERT_TRUE(comms_console_line("KEY?", reply, sizeof(reply)));
    TEST_ASSERT_EQUAL_STRING("KEY not set", reply);
    TEST_ASSERT_TRUE(comms_console_line("KEY 1234", reply, sizeof(reply)));
    TEST_ASSERT_NOT_NULL(strstr(reply, "error"));
    TEST_ASSERT_FALSE(comms_signing_key_loaded());

    char line[80];
    snprintf(line, sizeof(line), "KEY %s", TEST_KEY_HEX);
    TEST_ASSERT_TRUE(comms_console_line(line, reply, sizeof(reply)));
    TEST_ASSERT_TRUE(comms_signing_key_loaded());
    // python: hashlib.sha256(bytes(range(32))).hexdigest()[:12]
    TEST_ASSERT_NOT_NULL(strstr(reply, "630dcd2966c4"));
    TEST_ASSERT_NULL(strstr(reply, TEST_KEY_HEX)); // never echo the key

    // the new key works without a reboot
    gcs_setup(TEST_KEY);
    gcs_heartbeat();
    run_ms(50);
    TEST_ASSERT_TRUE(comms_gcs_link_ok());

    TEST_ASSERT_TRUE(comms_console_line("AESKEY 00112233445566778899aabbccddeeff", reply, sizeof(reply)));
    uint8_t aes[16];
    TEST_ASSERT_TRUE(signing_load_aes_key(aes));
    TEST_ASSERT_EQUAL_HEX8(0xff, aes[15]);
    TEST_ASSERT_TRUE(comms_console_line("KEY CLEAR", reply, sizeof(reply)));
    TEST_ASSERT_FALSE(comms_signing_key_loaded());
    TEST_ASSERT_FALSE(comms_console_line("hello", reply, sizeof(reply)));
}

static void test_back_pressure_never_blocks()
{
    boot(nullptr, false);
    run_ms(200);
    g_vehicle_end.write_limit = 0; // radio buffer full
    size_t before = g_vehicle_end.written_total;
    gcs_command(MAV_CMD_COMPONENT_ARM_DISARM, 1);
    run_ms(1000); // returns: comms_update never waits for room
    TEST_ASSERT_EQUAL(before, g_vehicle_end.written_total);
    TEST_ASSERT_TRUE(comms_stream_dropped(MAVLINK_MSG_ID_ATTITUDE) >= 9);

    // room comes back: the queued ACK goes first, then heartbeat/state, then streams
    g_rx.clear();
    g_vehicle_end.write_limit = SIZE_MAX;
    comms_test_advance_us(5000);
    comms_update();
    gcs_collect();
    TEST_ASSERT_TRUE(g_rx.size() >= 2);
    TEST_ASSERT_EQUAL(MAVLINK_MSG_ID_COMMAND_ACK, g_rx[0].msgid);

    // a small UART buffer: frames are never cut in half
    g_vehicle_end.write_limit = 100;
    g_rx.clear();
    run_ms(3000);
    TEST_ASSERT_EQUAL_UINT32(0, g_gcs_bad);
    TEST_ASSERT_TRUE(count_rx(MAVLINK_MSG_ID_HEARTBEAT) >= 2);
}

static void test_radio_txbuf_slows_streams()
{
    boot(nullptr, false);
    mavlink_message_t m;
    g_rx.clear();
    for (int i = 0; i < 4; i++) // the radio reports every ~second while its buffer is 20 % free
    {
        mavlink_msg_radio_status_pack_chan('3', 'D', GCS_CHAN, &m, 150, 140, 20, 20, 25, 0, 0);
        gcs_write(to_bytes(m));
        run_ms(1000);
    }
    int slow = count_rx(MAVLINK_MSG_ID_ATTITUDE); // 10 Hz / 4 over ~4 s
    TEST_ASSERT_INT_WITHIN(3, 10, slow);

    run_ms(COMMS_RADIO_STATUS_STALE_MS + 100); // reports stop: back to full rate
    g_rx.clear();
    run_ms(2000);
    TEST_ASSERT_INT_WITHIN(2, 20, count_rx(MAVLINK_MSG_ID_ATTITUDE));
}

static void test_parser_survives_noise_and_loss()
{
    boot(TEST_KEY, true);
    gcs_setup(TEST_KEY);
    srand(1234);
    for (int round = 0; round < 20; round++)
    {
        uint8_t junk[500];
        for (auto &b : junk)
            b = (uint8_t)rand();
        if (round % 4 == 0)
            junk[rand() % 400] = MAVLINK_STX; // fake frame starts
        g_gcs_end.write(junk, sizeof(junk));
        run_ms(10);
    }
    TEST_ASSERT_EQUAL_UINT32(0, g_handler.calls);
    TEST_ASSERT_FALSE(comms_gcs_link_ok());

    // lossy link: every frame is sent, 1 byte in 200 is dropped and 1 in 200 flipped
    int sent = 0;
    for (int i = 0; i < 200; i++)
    {
        std::vector<uint8_t> f = (i % 2) ? gcs_heartbeat_bytes() : gcs_command_bytes(MAV_CMD_GET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_ATTITUDE);
        std::vector<uint8_t> out;
        for (uint8_t b : f)
        {
            int r = rand() % 200;
            if (r == 0)
                continue;                 // lost byte
            out.push_back(r == 1 ? (uint8_t)(b ^ 0x5a) : b); // corrupted byte
        }
        gcs_write(out);
        sent++;
        run_ms(10);
    }
    const CommsStats &s = comms_stats();
    TEST_ASSERT_TRUE(s.rx_bad_crc + s.rx_bad_signature > 0);
    TEST_ASSERT_TRUE(s.rx_frames > (uint32_t)sent / 3); // most frames still get through
    gcs_heartbeat();
    run_ms(20);
    TEST_ASSERT_TRUE(comms_gcs_link_ok());
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_heartbeat_both_ways_and_link_timeout);
    RUN_TEST(test_round_trip_every_message);
    RUN_TEST(test_commands_guards_and_acks);
    RUN_TEST(test_heartbeat_armed_set_and_manual_flag);
    RUN_TEST(test_duplicate_suppression);
    RUN_TEST(test_rate_changes);
    RUN_TEST(test_signing_accepts_signed_and_survives_reboot);
    RUN_TEST(test_signing_rejects_unsigned_wrong_key_and_replay);
    RUN_TEST(test_radio_status_allowlist);
    RUN_TEST(test_no_key_with_required_signing_never_links);
    RUN_TEST(test_console_provisioning);
    RUN_TEST(test_back_pressure_never_blocks);
    RUN_TEST(test_radio_txbuf_slows_streams);
    RUN_TEST(test_parser_survives_noise_and_loss);
    return UNITY_END();
}
