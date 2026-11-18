#pragma once
// Per-stream downlink timing. One row per stream: a standard MAVLink message id, or a
// logger (system, subsystem) pair carried in GOOSE_LOG. Each row has an interval in
// microseconds (0 = off). Also holds the downlink byte budget and the slow-down
// factor derived from the radio's RADIO_STATUS.txbuf. No MAVLink packing in here.

#include <stddef.h>
#include <stdint.h>

#include "comms_config.h"

#define STREAM_MAX_ROWS 20
#define STREAM_MAX_LOG_ROWS 8

struct StreamRow
{
    bool in_use = false;
    bool is_log = false;
    uint32_t msg_id = 0;        // standard rows
    uint8_t log_system = 0;     // log rows
    uint8_t log_subsystem = 0;
    uint32_t interval_us = 0;   // 0 = off
    uint32_t default_us = 0;
    uint64_t next_due_us = 0;
    uint32_t sent = 0;
    uint32_t dropped = 0;       // due but not sent (no room / no budget)

    // Log rows only: latest record waiting to go out ("latest sample wins").
    bool has_sample = false;
    uint32_t sample_ts_ms = 0;
    uint8_t sample_level = 0;
    uint8_t sample_len = 0;
    uint8_t sample[COMMS_LOG_MAX_PAYLOAD] = {};
};

class StreamScheduler
{
public:
    void reset(uint64_t now_us);

    StreamRow *add_message(uint32_t msg_id, uint32_t default_interval_us);
    StreamRow *find_message(uint32_t msg_id);
    StreamRow *find_log(uint8_t system, uint8_t subsystem);
    StreamRow *find_or_add_log(uint8_t system, uint8_t subsystem); // nullptr if the log table is full
    int index_of(const StreamRow *r) const { return (int)(r - rows_); }
    StreamRow *row(int i) { return (i >= 0 && i < STREAM_MAX_ROWS && rows_[i].in_use) ? &rows_[i] : nullptr; }

    // MAVLink interval convention: -1 off, 0 default, >0 microseconds (clamped to
    // COMMS_MIN_INTERVAL_US). Returns the interval now in effect (0 = off).
    uint32_t set_interval(StreamRow &r, int64_t requested_us, uint64_t now_us);
    void set_all_log_intervals(int64_t requested_us, uint64_t now_us);

    // Most overdue row that is due now (log rows only when they hold a sample), or -1.
    int next_due(uint64_t now_us) const;
    void mark_sent(int i, uint64_t now_us);
    void mark_dropped(int i, uint64_t now_us);  // skip this period, count it
    void mark_skipped(int i, uint64_t now_us);  // no data this period, not counted

    // Back-pressure from the radio: txbuf is RADIO_STATUS.txbuf (% free in the radio).
    void on_radio_txbuf(uint8_t txbuf_pct, uint64_t now_us);
    uint8_t rate_scale(uint64_t now_us) const; // 1 = full rate, 2 = half, 4 = quarter

    // Token bucket on downlink bytes, refilled at COMMS_TX_BYTES_PER_SEC.
    void refill(uint64_t now_us, uint32_t bytes_per_sec);
    bool budget_allows(size_t bytes) const { return tokens_ >= (int32_t)bytes; }
    void spend(size_t bytes) { tokens_ -= (int32_t)bytes; }

private:
    void reschedule(StreamRow &r, uint64_t now_us);
    StreamRow rows_[STREAM_MAX_ROWS];
    uint8_t txbuf_pct_ = 100;
    uint64_t txbuf_time_us_ = 0;
    bool have_txbuf_ = false;
    int32_t tokens_ = 0;
    uint64_t last_refill_us_ = 0;
};
