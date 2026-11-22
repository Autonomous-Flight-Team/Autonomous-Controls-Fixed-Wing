#ifndef GOOSE_SENSORS_RATE_GATE_H_
#define GOOSE_SENSORS_RATE_GATE_H_
#include <stdint.h>

// Fixed-rate scheduler driven by an explicit microsecond clock, so it can be
// unit tested. Wraparound-safe. If the caller falls more than one period
// behind, it skips the missed ticks instead of bursting to catch up.
class RateGate {
public:
    explicit RateGate(uint32_t hz) : m_period_us(1000000UL / hz) {}

    bool due(uint32_t now_us) {
        if (!m_started) {
            m_started = true;
            m_next = now_us;
        }
        if ((int32_t)(now_us - m_next) < 0) {
            return false;
        }
        m_next += m_period_us;
        if ((int32_t)(now_us - m_next) >= 0) {
            m_next = now_us + m_period_us; // fell behind: resync
        }
        return true;
    }

    uint32_t period_us() const { return m_period_us; }

private:
    uint32_t m_period_us;
    uint32_t m_next{0};
    bool m_started{false};
};

#endif // GOOSE_SENSORS_RATE_GATE_H_
