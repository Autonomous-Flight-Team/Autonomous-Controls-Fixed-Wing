#include "stream_scheduler.h"

void StreamScheduler::reset(uint64_t now_us)
{
    for (auto &r : rows_)
        r = StreamRow();
    have_txbuf_ = false;
    txbuf_pct_ = 100;
    tokens_ = 0;
    last_refill_us_ = now_us;
}

static StreamRow *free_row(StreamRow *rows)
{
    for (int i = 0; i < STREAM_MAX_ROWS; i++)
        if (!rows[i].in_use)
            return &rows[i];
    return nullptr;
}

StreamRow *StreamScheduler::add_message(uint32_t msg_id, uint32_t default_interval_us)
{
    StreamRow *r = find_message(msg_id);
    if (!r)
        r = free_row(rows_);
    if (!r)
        return nullptr;
    *r = StreamRow();
    r->in_use = true;
    r->msg_id = msg_id;
    r->interval_us = r->default_us = default_interval_us;
    return r;
}

StreamRow *StreamScheduler::find_message(uint32_t msg_id)
{
    for (auto &r : rows_)
        if (r.in_use && !r.is_log && r.msg_id == msg_id)
            return &r;
    return nullptr;
}

StreamRow *StreamScheduler::find_log(uint8_t system, uint8_t subsystem)
{
    for (auto &r : rows_)
        if (r.in_use && r.is_log && r.log_system == system && r.log_subsystem == subsystem)
            return &r;
    return nullptr;
}

StreamRow *StreamScheduler::find_or_add_log(uint8_t system, uint8_t subsystem)
{
    StreamRow *r = find_log(system, subsystem);
    if (r)
        return r;
    int logs = 0;
    for (auto &x : rows_)
        logs += (x.in_use && x.is_log);
    if (logs >= STREAM_MAX_LOG_ROWS || !(r = free_row(rows_)))
        return nullptr;
    *r = StreamRow();
    r->in_use = r->is_log = true;
    r->log_system = system;
    r->log_subsystem = subsystem;
    r->interval_us = r->default_us = COMMS_RATE_LOG_DEFAULT_US;
    return r;
}

uint32_t StreamScheduler::set_interval(StreamRow &r, int64_t requested_us, uint64_t now_us)
{
    if (requested_us < 0)
        r.interval_us = 0;
    else if (requested_us == 0)
        r.interval_us = r.default_us;
    else
        r.interval_us = requested_us < COMMS_MIN_INTERVAL_US ? COMMS_MIN_INTERVAL_US
                        : requested_us > UINT32_MAX          ? UINT32_MAX
                                                             : (uint32_t)requested_us;
    r.next_due_us = now_us; // apply right away
    return r.interval_us;
}

void StreamScheduler::set_all_log_intervals(int64_t requested_us, uint64_t now_us)
{
    for (auto &r : rows_)
        if (r.in_use && r.is_log)
            set_interval(r, requested_us, now_us);
}

int StreamScheduler::next_due(uint64_t now_us) const
{
    int best = -1;
    for (int i = 0; i < STREAM_MAX_ROWS; i++)
    {
        const StreamRow &r = rows_[i];
        if (!r.in_use || r.interval_us == 0 || r.next_due_us > now_us)
            continue;
        if (r.is_log && !r.has_sample)
            continue;
        if (best < 0 || r.next_due_us < rows_[best].next_due_us)
            best = i;
    }
    return best;
}

void StreamScheduler::reschedule(StreamRow &r, uint64_t now_us)
{
    uint64_t step = (uint64_t)r.interval_us * rate_scale(now_us);
    r.next_due_us += step;
    if (r.next_due_us <= now_us) // fell behind: do not burst to catch up
        r.next_due_us = now_us + step;
}

void StreamScheduler::mark_sent(int i, uint64_t now_us)
{
    StreamRow &r = rows_[i];
    r.sent++;
    r.has_sample = false;
    reschedule(r, now_us);
}

void StreamScheduler::mark_dropped(int i, uint64_t now_us)
{
    StreamRow &r = rows_[i];
    r.dropped++;
    reschedule(r, now_us); // a log row keeps its sample; a newer one may replace it
}

void StreamScheduler::mark_skipped(int i, uint64_t now_us) { reschedule(rows_[i], now_us); }

void StreamScheduler::on_radio_txbuf(uint8_t txbuf_pct, uint64_t now_us)
{
    txbuf_pct_ = txbuf_pct;
    txbuf_time_us_ = now_us;
    have_txbuf_ = true;
}

uint8_t StreamScheduler::rate_scale(uint64_t now_us) const
{
    if (!have_txbuf_ || now_us - txbuf_time_us_ > (uint64_t)COMMS_RADIO_STATUS_STALE_MS * 1000)
        return 1;
    if (txbuf_pct_ < 25)
        return 4;
    if (txbuf_pct_ < 50)
        return 2;
    return 1;
}

void StreamScheduler::refill(uint64_t now_us, uint32_t bytes_per_sec)
{
    uint64_t dt = now_us - last_refill_us_;
    uint64_t add = dt * bytes_per_sec / 1000000;
    if (add == 0)
        return; // keep the remainder for next time
    last_refill_us_ += add * 1000000 / bytes_per_sec;
    int32_t cap = (int32_t)(bytes_per_sec / 4); // allow a 250 ms burst
    int64_t t = (int64_t)tokens_ + (int64_t)add;
    tokens_ = (int32_t)(t > cap ? cap : t);
}
