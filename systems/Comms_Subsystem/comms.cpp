#include "comms.h"

#include <stdio.h>
#include <string.h>

#include "comms_mavlink.h"
#include "comms_platform.h"
#include "signing.h"
#include "stream_scheduler.h"

static_assert(COMMS_TIMESTAMP_SAVE_S < 60, "must be below the 60 s boot bump or the ground may reject frames after a reboot");

static const uint8_t CHAN = MAVLINK_COMM_0;
static const int ACK_QUEUE_LEN = 4;
static const int EVENT_QUEUE_LEN = 8;
static const int CMD_CACHE_LEN = 4;
static const float FORCE_ARM_MAGIC = 21196.0f;

enum EventKind : uint8_t
{
    EV_STATUSTEXT,
    EV_LOG,
    EV_MESSAGE_INTERVAL
};

struct Event
{
    EventKind kind;
    uint8_t severity;                  // EV_STATUSTEXT
    char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN];
    uint8_t system, subsystem, level;  // EV_LOG
    uint8_t len;
    uint32_t ts_ms;
    uint8_t payload[COMMS_LOG_MAX_PAYLOAD];
    uint16_t msg_id;                   // EV_MESSAGE_INTERVAL
    int32_t interval_us;
};

struct Ack
{
    uint16_t command;
    uint8_t result;
    uint8_t target_system, target_component;
};

struct CachedCommand // for duplicate suppression
{
    bool valid;
    uint8_t sysid, compid;
    uint16_t command;
    float params[7];
    uint8_t result;
    uint64_t time_us;
};

enum SendResult
{
    SENT,
    BLOCKED, // no room in the UART or no byte budget
    NO_DATA
};

static struct
{
    ByteStream *link = nullptr;
    TelemetrySource *tel = nullptr;
    CommandHandler *handler = nullptr;
    CommsConfig cfg;
    CommsStats stats;

    mavlink_signing_t signing;
    mavlink_signing_streams_t signing_streams;
    bool have_key = false;
    bool accepted_by_callback = false; // set by the library callback for the frame being parsed
    uint64_t boot_ticks = 0;           // signing timestamp at init
    uint64_t init_us = 0;
    uint64_t last_saved_ticks = 0;

    StreamScheduler sched;
    uint64_t next_heartbeat_us = 0;
    int64_t log_interval_request = 0; // last SET_MESSAGE_INTERVAL(GOOSE_LOG), applied to new log rows

    GooseStateSnapshot state;          // latest from telemetry
    GooseStateSnapshot last_sent_state;
    bool state_valid = false;
    bool state_changed = false;
    bool was_armed = false;

    bool have_gcs = false;
    uint64_t last_gcs_us = 0;

    Ack acks[ACK_QUEUE_LEN];
    int ack_head = 0, ack_count = 0;
    Event events[EVENT_QUEUE_LEN];
    int ev_head = 0, ev_count = 0;
    CachedCommand cmd_cache[CMD_CACHE_LEN];
    int cmd_next = 0;
} g;

// ---------------------------------------------------------------- helpers

static uint32_t boot_ms(uint64_t now_us) { return (uint32_t)((now_us - g.init_us) / 1000); }

static bool is_armed(const GooseStateSnapshot &s) { return goose_system_armed(s.system_state); } // not >=: IN_AIR_RESET is 20

static bool accept_unsigned_cb(const mavlink_status_t *, uint32_t)
{
    // Accept at the library level and decide in dispatch(), where the whole frame
    // (signed flag, sysid, compid) is known.
    g.accepted_by_callback = true;
    return true;
}

static void setup_signing(bool keep_timestamp)
{
    uint64_t ts = keep_timestamp ? g.signing.timestamp : g.boot_ticks;
    memset(&g.signing, 0, sizeof(g.signing));
    g.have_key = signing_load_key(g.signing.secret_key);
    g.signing.link_id = CHAN;
    g.signing.flags = g.have_key ? MAVLINK_SIGNING_FLAG_SIGN_OUTGOING : 0;
    g.signing.timestamp = ts;
    g.signing.accept_unsigned_callback = accept_unsigned_cb;
    mavlink_status_t *st = mavlink_get_channel_status(CHAN);
    st->signing = &g.signing;
    st->signing_streams = &g.signing_streams;
}

static void save_timestamp()
{
    signing_save_timestamp(g.signing.timestamp);
    g.last_saved_ticks = g.signing.timestamp;
    g.stats.timestamp_saves++;
}

static size_t frame_upper_bound(uint32_t msg_id)
{
    const mavlink_msg_entry_t *e = mavlink_get_msg_entry(msg_id);
    return MAVLINK_NUM_NON_PAYLOAD_BYTES + (e ? e->max_msg_len : MAVLINK_MAX_PAYLOAD_LEN) + MAVLINK_SIGNATURE_BLOCK_LEN;
}

static bool can_send(uint32_t msg_id, bool use_budget)
{
    size_t need = frame_upper_bound(msg_id);
    if (g.link->availableForWrite() < (int)need)
        return false;
    return !use_budget || g.sched.budget_allows(need);
}

static void write_msg(const mavlink_message_t &msg)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    g.link->write(buf, len); // room was checked by can_send(), so this does not block
    g.stats.tx_frames++;
    g.stats.tx_bytes += len;
    g.sched.spend(len);
}

static void push_ack(uint16_t command, uint8_t result, uint8_t sys, uint8_t comp)
{
    if (g.ack_count == ACK_QUEUE_LEN) // drop oldest; the ground will retry
    {
        g.ack_head = (g.ack_head + 1) % ACK_QUEUE_LEN;
        g.ack_count--;
    }
    g.acks[(g.ack_head + g.ack_count) % ACK_QUEUE_LEN] = {command, result, sys, comp};
    g.ack_count++;
}

static Event *push_event(EventKind kind)
{
    if (g.ev_count == EVENT_QUEUE_LEN)
    {
        g.ev_head = (g.ev_head + 1) % EVENT_QUEUE_LEN;
        g.ev_count--;
        g.stats.events_dropped++;
    }
    Event *e = &g.events[(g.ev_head + g.ev_count) % EVENT_QUEUE_LEN];
    memset(e, 0, sizeof(*e));
    e->kind = kind;
    g.ev_count++;
    return e;
}

// ---------------------------------------------------------------- receive

static bool targets_us(uint8_t sys, uint8_t comp)
{
    return (sys == g.cfg.sysid || sys == 0) && (comp == g.cfg.compid || comp == 0);
}

static CachedCommand *find_cached(const mavlink_message_t &m, uint16_t command, const float p[7], uint64_t now)
{
    for (auto &c : g.cmd_cache)
        if (c.valid && c.sysid == m.sysid && c.compid == m.compid && c.command == command &&
            memcmp(c.params, p, sizeof(c.params)) == 0 && now - c.time_us <= (uint64_t)COMMS_CMD_DUP_WINDOW_MS * 1000)
            return &c;
    return nullptr;
}

static int32_t interval_for_report(uint16_t msg_id)
{
    // MESSAGE_INTERVAL convention: -1 disabled, 0 not available, >0 interval in us
    if (msg_id == MAVLINK_MSG_ID_HEARTBEAT)
        return COMMS_HEARTBEAT_INTERVAL_US;
    if (msg_id == MAVLINK_MSG_ID_GOOSE_LOG)
        return g.log_interval_request < 0 ? -1 : g.log_interval_request == 0 ? COMMS_RATE_LOG_DEFAULT_US : (int32_t)g.log_interval_request;
    StreamRow *r = g.sched.find_message(msg_id);
    if (!r)
        return 0;
    return r->interval_us == 0 ? -1 : (int32_t)r->interval_us;
}

static MAV_RESULT execute_command(const mavlink_command_long_t &c, uint64_t now)
{
    switch (c.command)
    {
    case MAV_CMD_COMPONENT_ARM_DISARM:
        if (c.param1 != 0.0f && c.param1 != 1.0f)
            return MAV_RESULT_DENIED;
        return g.handler->arm(c.param1 == 1.0f, c.param2 == FORCE_ARM_MAGIC);
    case MAV_CMD_NAV_TAKEOFF:
        return g.handler->takeoff(c.param1, c.param7);
    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        return g.handler->return_to_launch();
    case MAV_CMD_NAV_LOITER_UNLIM:
        return g.handler->loiter_unlimited(c.param3, c.param5, c.param6, c.param7);
    case MAV_CMD_GOOSE_RESUME_MISSION:
        return g.handler->resume_mission();
    case MAV_CMD_GOOSE_FAULT_CLEAR:
        return g.handler->fault_clear();
    case MAV_CMD_GOOSE_BENCH:
        if (c.param1 != 0.0f && c.param1 != 1.0f)
            return MAV_RESULT_DENIED;
        return g.handler->bench(c.param1 == 1.0f);
    // MAV_CMD_NAV_LAND is not supported: the flight FSM has no land input; landing
    // is reached through RTL (RTL -> DESCENT -> LANDING).

    case MAV_CMD_SET_MESSAGE_INTERVAL:
    {
        uint32_t id = (uint32_t)c.param1;
        int64_t interval = (int64_t)c.param2;
        if (id == MAVLINK_MSG_ID_GOOSE_LOG)
        {
            g.log_interval_request = interval;
            g.sched.set_all_log_intervals(interval, now);
            return MAV_RESULT_ACCEPTED;
        }
        StreamRow *r = g.sched.find_message(id);
        if (!r)
            return MAV_RESULT_DENIED; // not a configurable stream (includes HEARTBEAT)
        g.sched.set_interval(*r, interval, now);
        return MAV_RESULT_ACCEPTED;
    }
    case MAV_CMD_GET_MESSAGE_INTERVAL:
    {
        Event *e = push_event(EV_MESSAGE_INTERVAL);
        e->msg_id = (uint16_t)c.param1;
        e->interval_us = interval_for_report(e->msg_id);
        return MAV_RESULT_ACCEPTED;
    }
    case MAV_CMD_GOOSE_SET_LOG_RATE:
    {
        if (c.param1 < 0 || c.param1 > 255 || c.param2 < 0 || c.param2 > 255)
            return MAV_RESULT_DENIED;
        StreamRow *r = g.sched.find_or_add_log((uint8_t)c.param1, (uint8_t)c.param2);
        if (!r)
            return MAV_RESULT_DENIED; // log stream table full
        g.sched.set_interval(*r, (int64_t)c.param3, now);
        return MAV_RESULT_ACCEPTED;
    }
    default:
        return MAV_RESULT_UNSUPPORTED;
    }
}

static void handle_command_long(const mavlink_message_t &m, uint64_t now)
{
    mavlink_command_long_t c;
    mavlink_msg_command_long_decode(&m, &c);
    if (!targets_us(c.target_system, c.target_component))
        return;
    g.stats.cmd_received++;

    float p[7] = {c.param1, c.param2, c.param3, c.param4, c.param5, c.param6, c.param7};
    CachedCommand *prev = find_cached(m, c.command, p, now);
    if (c.confirmation > 0 && prev)
    {
        // A retry of a command we already handled (our ACK was probably lost): answer
        // again with the same result, do not run it twice.
        g.stats.cmd_duplicates++;
        push_ack(c.command, prev->result, m.sysid, m.compid);
        return;
    }

    MAV_RESULT result = execute_command(c, now);
    CachedCommand &slot = g.cmd_cache[g.cmd_next];
    g.cmd_next = (g.cmd_next + 1) % CMD_CACHE_LEN;
    slot = {true, m.sysid, m.compid, c.command, {}, (uint8_t)result, now};
    memcpy(slot.params, p, sizeof(p));
    push_ack(c.command, (uint8_t)result, m.sysid, m.compid);
}

static void dispatch(const mavlink_message_t &m, bool accepted_by_callback, uint64_t now)
{
    bool is_signed = (m.incompat_flags & MAVLINK_IFLAG_SIGNED) != 0;
    if (accepted_by_callback)
    {
        if (is_signed) // signature present but wrong key, replayed or too old
        {
            g.stats.rx_bad_signature++;
            return;
        }
        if (!signing_allow_unsigned(m.msgid, m.sysid, m.compid, g.cfg.require_signing))
        {
            g.stats.rx_unsigned_dropped++;
            return;
        }
    }
    g.stats.rx_frames++;

    switch (m.msgid)
    {
    case MAVLINK_MSG_ID_HEARTBEAT:
        if (mavlink_msg_heartbeat_get_type(&m) == MAV_TYPE_GCS)
        {
            g.have_gcs = true;
            g.last_gcs_us = now;
        }
        break;
    case MAVLINK_MSG_ID_RADIO_STATUS:
        if (m.sysid == '3' && m.compid == 'D') // injected by our SiK radio
        {
            mavlink_radio_status_t r;
            mavlink_msg_radio_status_decode(&m, &r);
            g.stats.radio_rssi = r.rssi;
            g.stats.radio_remrssi = r.remrssi;
            g.stats.radio_txbuf = r.txbuf;
            g.sched.on_radio_txbuf(r.txbuf, now);
        }
        break;
    case MAVLINK_MSG_ID_COMMAND_LONG:
        handle_command_long(m, now);
        break;
    case MAVLINK_MSG_ID_COMMAND_INT:
    {
        mavlink_command_int_t c;
        mavlink_msg_command_int_decode(&m, &c);
        if (targets_us(c.target_system, c.target_component))
            push_ack(c.command, MAV_RESULT_COMMAND_LONG_ONLY, m.sysid, m.compid);
        break;
    }
    default:
        break; // everything else is ignored (no param/mission protocol)
    }
}

static void receive(uint64_t now)
{
    mavlink_message_t msg;
    mavlink_status_t status;
    for (int n = 0; n < COMMS_MAX_RX_BYTES_PER_UPDATE && g.link->available() > 0; n++)
    {
        int c = g.link->read();
        if (c < 0)
            break;
        g.accepted_by_callback = false;
        uint8_t r = mavlink_frame_char(CHAN, (uint8_t)c, &msg, &status);
        if (r == MAVLINK_FRAMING_OK)
            dispatch(msg, g.accepted_by_callback, now);
        else if (r == MAVLINK_FRAMING_BAD_CRC)
            g.stats.rx_bad_crc++;
        else if (r == MAVLINK_FRAMING_BAD_SIGNATURE)
            g.stats.rx_bad_signature++;
    }
}

// ---------------------------------------------------------------- transmit

static uint8_t mav_state(uint8_t sys)
{
    switch (sys)
    {
    case GOOSE_SYSTEM_STATE_BOOT: return MAV_STATE_BOOT;
    case GOOSE_SYSTEM_STATE_PREFLIGHT: return MAV_STATE_CALIBRATING;
    case GOOSE_SYSTEM_STATE_FAULT_GROUND:
    case GOOSE_SYSTEM_STATE_STANDBY:
    case GOOSE_SYSTEM_STATE_BENCH: return MAV_STATE_STANDBY;
    case GOOSE_SYSTEM_STATE_ACTIVE:
    case GOOSE_SYSTEM_STATE_DEGRADED: return MAV_STATE_ACTIVE;
    case GOOSE_SYSTEM_STATE_CRITICAL: return MAV_STATE_CRITICAL;
    case GOOSE_SYSTEM_STATE_EMERGENCY: return MAV_STATE_EMERGENCY;
    case GOOSE_SYSTEM_STATE_IN_AIR_RESET: return MAV_STATE_CRITICAL; // rebooted airborne, pilot flying
    default: return MAV_STATE_UNINIT;
    }
}

static SendResult send_heartbeat()
{
    if (!can_send(MAVLINK_MSG_ID_HEARTBEAT, false))
        return BLOCKED;
    const GooseStateSnapshot &s = g.state;
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (is_armed(s) ? MAV_MODE_FLAG_SAFETY_ARMED : 0) |
                        ((s.flags & GOOSE_STATE_FLAGS_MANUAL_ACTIVE) ? MAV_MODE_FLAG_MANUAL_INPUT_ENABLED : 0);
    uint32_t custom_mode = (uint32_t)s.flight_state | ((uint32_t)s.system_state << 8);
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack_chan(g.cfg.sysid, g.cfg.compid, CHAN, &msg, MAV_TYPE_FIXED_WING,
                                    MAV_AUTOPILOT_GENERIC, base_mode, custom_mode, mav_state(s.system_state));
    write_msg(msg);
    return SENT;
}

static SendResult send_goose_state(uint64_t now)
{
    if (!g.state_valid)
        return NO_DATA;
    if (!can_send(MAVLINK_MSG_ID_GOOSE_STATE, false))
        return BLOCKED;
    const GooseStateSnapshot &s = g.state;
    mavlink_message_t msg;
    mavlink_msg_goose_state_pack_chan(g.cfg.sysid, g.cfg.compid, CHAN, &msg, boot_ms(now), s.active_faults,
                                      s.system_state, s.flight_state, s.critical_entries, s.flags, s.sensor_status);
    write_msg(msg);
    g.last_sent_state = s;
    g.state_changed = false;
    return SENT;
}

static SendResult send_ack(const Ack &a)
{
    if (!can_send(MAVLINK_MSG_ID_COMMAND_ACK, false))
        return BLOCKED;
    mavlink_message_t msg;
    mavlink_msg_command_ack_pack_chan(g.cfg.sysid, g.cfg.compid, CHAN, &msg, a.command, a.result, 0, 0,
                                      a.target_system, a.target_component);
    write_msg(msg);
    return SENT;
}

static SendResult send_event(const Event &e)
{
    mavlink_message_t msg;
    switch (e.kind)
    {
    case EV_STATUSTEXT:
        if (!can_send(MAVLINK_MSG_ID_STATUSTEXT, false))
            return BLOCKED;
        mavlink_msg_statustext_pack_chan(g.cfg.sysid, g.cfg.compid, CHAN, &msg, e.severity, e.text, 0, 0);
        break;
    case EV_LOG:
        if (!can_send(MAVLINK_MSG_ID_GOOSE_LOG, false))
            return BLOCKED;
        mavlink_msg_goose_log_pack_chan(g.cfg.sysid, g.cfg.compid, CHAN, &msg, e.ts_ms, e.system, e.subsystem,
                                        e.level, e.len, e.payload);
        break;
    case EV_MESSAGE_INTERVAL:
        if (!can_send(MAVLINK_MSG_ID_MESSAGE_INTERVAL, false))
            return BLOCKED;
        mavlink_msg_message_interval_pack_chan(g.cfg.sysid, g.cfg.compid, CHAN, &msg, e.msg_id, e.interval_us);
        break;
    }
    write_msg(msg);
    return SENT;
}

static uint32_t sensor_bits(int index)
{
    switch (index)
    {
    case GOOSE_SENSOR_INDEX_IMU: return MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL;
    case GOOSE_SENSOR_INDEX_BARO: return MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    case GOOSE_SENSOR_INDEX_GPS: return MAV_SYS_STATUS_SENSOR_GPS;
    case GOOSE_SENSOR_INDEX_MAG: return MAV_SYS_STATUS_SENSOR_3D_MAG;
    case GOOSE_SENSOR_INDEX_AIRSPEED: return MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
    case GOOSE_SENSOR_INDEX_BATTERY: return MAV_SYS_STATUS_SENSOR_BATTERY;
    case GOOSE_SENSOR_INDEX_RC: return MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    default: return 0;
    }
}

// Pack one rate stream. Data is fetched first so a missing source is NO_DATA, then
// room and byte budget are checked before anything is packed (packing uses up a
// sequence number and a signing timestamp).
static SendResult send_stream(StreamRow &r, uint64_t now)
{
    mavlink_message_t msg;
    const uint8_t sys = g.cfg.sysid, comp = g.cfg.compid;
    const uint32_t t_ms = boot_ms(now);

    if (r.is_log)
    {
        if (!can_send(MAVLINK_MSG_ID_GOOSE_LOG, true))
            return BLOCKED;
        mavlink_msg_goose_log_pack_chan(sys, comp, CHAN, &msg, r.sample_ts_ms, r.log_system, r.log_subsystem,
                                        r.sample_level, r.sample_len, r.sample);
        write_msg(msg);
        return SENT;
    }

    switch (r.msg_id)
    {
    case MAVLINK_MSG_ID_GOOSE_STATE:
        return send_goose_state(now);
    case MAVLINK_MSG_ID_SYS_STATUS:
    {
        if (!g.state_valid)
            return NO_DATA;
        BatterySample b;
        bool have_b = g.tel->battery(b);
        uint32_t present = 0, health = 0;
        for (int i = 0; i < GOOSE_NUM_SENSORS; i++)
        {
            uint8_t st = g.state.sensor_status[i];
            if (st != GOOSE_SENSOR_STATUS_UNINITIALIZED)
                present |= sensor_bits(i);
            if (st == GOOSE_SENSOR_STATUS_NOMINAL || st == GOOSE_SENSOR_STATUS_DEGRADED)
                health |= sensor_bits(i);
        }
        if (!can_send(MAVLINK_MSG_ID_SYS_STATUS, true))
            return BLOCKED;
        uint32_t errs = g.stats.rx_bad_crc + g.stats.rx_bad_signature;
        mavlink_msg_sys_status_pack_chan(sys, comp, CHAN, &msg, present, present, health, g.tel->cpu_load_permille(),
                                         have_b ? b.voltage_mv : UINT16_MAX, have_b ? b.current_ca : -1,
                                         have_b ? b.remaining_pct : -1, 0, (uint16_t)(errs > UINT16_MAX ? UINT16_MAX : errs),
                                         0, 0, 0, 0, 0, 0, 0);
        break;
    }
    case MAVLINK_MSG_ID_ATTITUDE:
    {
        AttitudeSample a;
        if (!g.tel->attitude(a))
            return NO_DATA;
        if (!can_send(r.msg_id, true))
            return BLOCKED;
        mavlink_msg_attitude_pack_chan(sys, comp, CHAN, &msg, t_ms, a.roll, a.pitch, a.yaw, a.rollspeed, a.pitchspeed, a.yawspeed);
        break;
    }
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
    {
        PositionSample p;
        if (!g.tel->position(p))
            return NO_DATA;
        if (!can_send(r.msg_id, true))
            return BLOCKED;
        mavlink_msg_global_position_int_pack_chan(sys, comp, CHAN, &msg, t_ms, p.lat, p.lon, p.alt_mm, p.relative_alt_mm,
                                                  p.vx, p.vy, p.vz, p.hdg_cdeg);
        break;
    }
    case MAVLINK_MSG_ID_GPS_RAW_INT:
    {
        GpsSample p;
        if (!g.tel->gps(p))
            return NO_DATA;
        if (!can_send(r.msg_id, true))
            return BLOCKED;
        mavlink_msg_gps_raw_int_pack_chan(sys, comp, CHAN, &msg, now - g.init_us, p.fix_type, p.lat, p.lon, p.alt_mm, p.eph,
                                          p.epv, p.vel_cms, p.cog_cdeg, p.satellites_visible, 0, 0, 0, 0, 0, 0);
        break;
    }
    case MAVLINK_MSG_ID_VFR_HUD:
    {
        AirDataSample a;
        if (!g.tel->air_data(a))
            return NO_DATA;
        if (!can_send(r.msg_id, true))
            return BLOCKED;
        mavlink_msg_vfr_hud_pack_chan(sys, comp, CHAN, &msg, a.airspeed, a.groundspeed, a.heading_deg, a.throttle_pct, a.alt, a.climb);
        break;
    }
    case MAVLINK_MSG_ID_SCALED_IMU:
    {
        ImuSample i;
        if (!g.tel->imu(i))
            return NO_DATA;
        if (!can_send(r.msg_id, true))
            return BLOCKED;
        mavlink_msg_scaled_imu_pack_chan(sys, comp, CHAN, &msg, t_ms, i.xacc, i.yacc, i.zacc, i.xgyro, i.ygyro, i.zgyro,
                                         i.xmag, i.ymag, i.zmag, i.temperature_cdeg);
        break;
    }
    case MAVLINK_MSG_ID_SCALED_PRESSURE:
    {
        PressureSample p;
        if (!g.tel->pressure(p))
            return NO_DATA;
        if (!can_send(r.msg_id, true))
            return BLOCKED;
        mavlink_msg_scaled_pressure_pack_chan(sys, comp, CHAN, &msg, t_ms, p.press_abs_hpa, p.press_diff_hpa, p.temperature_cdeg, 0);
        break;
    }
    case MAVLINK_MSG_ID_BATTERY_STATUS:
    {
        BatterySample b;
        if (!g.tel->battery(b))
            return NO_DATA;
        if (!can_send(r.msg_id, true))
            return BLOCKED;
        uint16_t cells[10];
        uint16_t cells_ext[4] = {0, 0, 0, 0};
        for (auto &v : cells)
            v = UINT16_MAX; // unknown
        cells[0] = b.voltage_mv; // total pack voltage in cell 0, as allowed by the spec
        mavlink_msg_battery_status_pack_chan(sys, comp, CHAN, &msg, 0, MAV_BATTERY_FUNCTION_ALL, MAV_BATTERY_TYPE_LIPO, INT16_MAX,
                                             cells, b.current_ca, b.consumed_mah, -1, b.remaining_pct, 0,
                                             MAV_BATTERY_CHARGE_STATE_UNDEFINED, cells_ext, 0, 0);
        break;
    }
    default:
        return NO_DATA;
    }
    write_msg(msg);
    return SENT;
}

static void transmit(uint64_t now)
{
    int frames = 0;
    bool blocked = false;

    // 1. command acks
    while (g.ack_count > 0 && frames < COMMS_MAX_TX_FRAMES_PER_UPDATE)
    {
        if (send_ack(g.acks[g.ack_head]) != SENT)
        {
            blocked = true;
            break;
        }
        g.ack_head = (g.ack_head + 1) % ACK_QUEUE_LEN;
        g.ack_count--;
        frames++;
    }

    // 2. heartbeat
    if (!blocked && frames < COMMS_MAX_TX_FRAMES_PER_UPDATE && now >= g.next_heartbeat_us)
    {
        if (send_heartbeat() == SENT)
        {
            frames++;
            g.next_heartbeat_us += COMMS_HEARTBEAT_INTERVAL_US;
            if (g.next_heartbeat_us <= now)
                g.next_heartbeat_us = now + COMMS_HEARTBEAT_INTERVAL_US;
        }
        else
            blocked = true;
    }

    // 3. GOOSE_STATE: on change, or when its stream row is due
    StreamRow *gs = g.sched.find_message(MAVLINK_MSG_ID_GOOSE_STATE);
    bool gs_due = gs && gs->interval_us > 0 && now >= gs->next_due_us;
    if (!blocked && frames < COMMS_MAX_TX_FRAMES_PER_UPDATE && (g.state_changed || gs_due))
    {
        SendResult r = send_goose_state(now);
        if (r == SENT)
        {
            frames++;
            if (gs_due)
                g.sched.mark_sent(g.sched.index_of(gs), now);
        }
        else if (r == BLOCKED)
            blocked = true;
    }

    // 4. events (STATUSTEXT, WARN/ERROR logs, MESSAGE_INTERVAL replies)
    while (!blocked && g.ev_count > 0 && frames < COMMS_MAX_TX_FRAMES_PER_UPDATE)
    {
        if (send_event(g.events[g.ev_head]) != SENT)
        {
            blocked = true;
            break;
        }
        g.ev_head = (g.ev_head + 1) % EVENT_QUEUE_LEN;
        g.ev_count--;
        frames++;
    }

    // 5. rate streams, most overdue first
    while (!blocked && frames < COMMS_MAX_TX_FRAMES_PER_UPDATE)
    {
        int i = g.sched.next_due(now);
        if (i < 0)
            break;
        SendResult r = send_stream(*g.sched.row(i), now);
        if (r == SENT)
        {
            g.sched.mark_sent(i, now);
            frames++;
        }
        else if (r == NO_DATA)
            g.sched.mark_skipped(i, now);
        else
            blocked = true;
    }

    // Back-pressure: everything still due this period is skipped and counted, so the
    // next period sends fresh data instead of a backlog.
    if (blocked)
    {
        int i;
        while ((i = g.sched.next_due(now)) >= 0)
            g.sched.mark_dropped(i, now);
    }
}

// ---------------------------------------------------------------- public API

void comms_init(ByteStream &link, TelemetrySource &telemetry, CommandHandler &handler, const CommsConfig &config)
{
    uint64_t now = comms_time_us();
    g.link = &link;
    g.tel = &telemetry;
    g.handler = &handler;
    g.cfg = config;
    g.stats = CommsStats();
    g.init_us = now;
    g.have_gcs = false;
    g.ack_head = g.ack_count = g.ev_head = g.ev_count = g.cmd_next = 0;
    memset(g.cmd_cache, 0, sizeof(g.cmd_cache));
    g.next_heartbeat_us = now;
    g.log_interval_request = 0;
    g.state_valid = g.state_changed = g.was_armed = false;

    mavlink_status_t *st = mavlink_get_channel_status(CHAN);
    memset(st, 0, sizeof(*st));
    memset(&g.signing_streams, 0, sizeof(g.signing_streams));
    g.boot_ticks = signing_boot_timestamp();
    setup_signing(false);
    save_timestamp(); // so a second reboot before the next save still moves forward

    StreamScheduler &s = g.sched;
    s.reset(now);
    s.add_message(MAVLINK_MSG_ID_GOOSE_STATE, COMMS_RATE_GOOSE_STATE_US);
    s.add_message(MAVLINK_MSG_ID_SYS_STATUS, COMMS_RATE_SYS_STATUS_US);
    s.add_message(MAVLINK_MSG_ID_ATTITUDE, COMMS_RATE_ATTITUDE_US);
    s.add_message(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, COMMS_RATE_GLOBAL_POSITION_INT_US);
    s.add_message(MAVLINK_MSG_ID_GPS_RAW_INT, COMMS_RATE_GPS_RAW_INT_US);
    s.add_message(MAVLINK_MSG_ID_VFR_HUD, COMMS_RATE_VFR_HUD_US);
    s.add_message(MAVLINK_MSG_ID_SCALED_IMU, COMMS_RATE_SCALED_IMU_US);
    s.add_message(MAVLINK_MSG_ID_SCALED_PRESSURE, COMMS_RATE_SCALED_PRESSURE_US);
    s.add_message(MAVLINK_MSG_ID_BATTERY_STATUS, COMMS_RATE_BATTERY_STATUS_US);

    if (!g.have_key)
        comms_send_statustext(MAV_SEVERITY_WARNING, "COMMS: no signing key provisioned");
}

void comms_update()
{
    if (!g.link)
        return;
    uint64_t now = comms_time_us();

    // Signing clock: never behind real time since boot (the library also bumps it by
    // one per signed frame and adopts newer timestamps from the ground).
    uint64_t floor_ticks = g.boot_ticks + (now - g.init_us) / 10;
    if (g.signing.timestamp < floor_ticks)
        g.signing.timestamp = floor_ticks;

    receive(now);

    GooseStateSnapshot s;
    if (g.tel->goose_state(s))
    {
        // ARMED always follows the armed set, whatever the source filled in.
        s.flags = (uint8_t)((s.flags & ~GOOSE_STATE_FLAGS_ARMED) | (is_armed(s) ? GOOSE_STATE_FLAGS_ARMED : 0));
        if (!g.state_valid || memcmp(&s, &g.last_sent_state, sizeof(s)) != 0)
            g.state_changed = true;
        bool armed = is_armed(s);
        if (g.was_armed && !armed)
            save_timestamp(); // on disarm
        g.was_armed = armed;
        g.state = s;
        g.state_valid = true;
    }
    if (g.signing.timestamp - g.last_saved_ticks >= (uint64_t)COMMS_TIMESTAMP_SAVE_S * SIGNING_TICKS_PER_SEC)
        save_timestamp();

    g.sched.refill(now, g.cfg.tx_bytes_per_sec);
    transmit(now);
}

bool comms_gcs_link_ok()
{
    return g.have_gcs && comms_time_us() - g.last_gcs_us <= (uint64_t)g.cfg.gcs_timeout_ms * 1000;
}

bool comms_forward_log(uint8_t system, uint8_t subsystem, uint8_t level, uint32_t ts_ms, const void *payload, uint8_t len)
{
    if (!g.link || len > COMMS_LOG_MAX_PAYLOAD || (len && !payload))
        return false;

    if (level == COMMS_LOG_WARN || level == COMMS_LOG_ERROR)
    {
        Event *e = push_event(EV_LOG);
        e->system = system;
        e->subsystem = subsystem;
        e->level = level;
        e->ts_ms = ts_ms;
        e->len = len;
        if (len)
            memcpy(e->payload, payload, len);
        char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1];
        snprintf(text, sizeof(text), "LOG %s sys 0x%02X sub 0x%02X", level == COMMS_LOG_ERROR ? "ERROR" : "WARN", system, subsystem);
        comms_send_statustext(level == COMMS_LOG_ERROR ? MAV_SEVERITY_ERROR : MAV_SEVERITY_WARNING, text);
        return true;
    }

    bool is_new = g.sched.find_log(system, subsystem) == nullptr;
    StreamRow *r = g.sched.find_or_add_log(system, subsystem);
    if (!r)
        return false;
    if (is_new && g.log_interval_request != 0)
        g.sched.set_interval(*r, g.log_interval_request, comms_time_us());
    r->has_sample = true; // latest sample wins
    r->sample_ts_ms = ts_ms;
    r->sample_level = level;
    r->sample_len = len;
    memset(r->sample, 0, sizeof(r->sample));
    if (len)
        memcpy(r->sample, payload, len);
    return true;
}

bool comms_send_statustext(uint8_t severity, const char *text)
{
    if (!text)
        return false;
    Event *e = push_event(EV_STATUSTEXT);
    e->severity = severity;
    strncpy(e->text, text, sizeof(e->text)); // not NUL-terminated at 50 chars, as MAVLink expects
    return true;
}

bool comms_console_line(const char *line, char *reply, size_t reply_len)
{
    bool key_changed = false;
    if (!signing_console_command(line, reply, reply_len, &key_changed))
        return false;
    if (key_changed)
        setup_signing(true);
    return true;
}

bool comms_signing_key_loaded() { return g.have_key; }
void comms_save_signing_timestamp() { save_timestamp(); }
uint64_t comms_signing_timestamp() { return g.signing.timestamp; }
const CommsStats &comms_stats() { return g.stats; }

uint32_t comms_stream_interval_us(uint32_t msg_id)
{
    StreamRow *r = g.sched.find_message(msg_id);
    return r ? r->interval_us : UINT32_MAX;
}

uint32_t comms_stream_dropped(uint32_t msg_id)
{
    StreamRow *r = g.sched.find_message(msg_id);
    return r ? r->dropped : UINT32_MAX;
}

uint32_t comms_log_interval_us(uint8_t system, uint8_t subsystem)
{
    StreamRow *r = g.sched.find_log(system, subsystem);
    return r ? r->interval_us : UINT32_MAX;
}
