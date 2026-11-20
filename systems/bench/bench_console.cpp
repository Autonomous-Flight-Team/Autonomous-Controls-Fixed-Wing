#include "bench_console.h"
#include <cstdlib>
#include <cstring>

namespace {

constexpr size_t kTokLen = 16;
constexpr uint8_t kMaxToks = 4;

struct Tokens {
    char t[kMaxToks][kTokLen] = {};
    uint8_t n = 0;
};

// Splits on spaces/tabs. False if a token is too long or there are too many.
bool tokenize(const char* line, Tokens* out) {
    out->n = 0;
    const char* p = line;
    while (*p != '\0') {
        while (*p == ' ' || *p == '\t') {
            ++p;
        }
        if (*p == '\0') {
            break;
        }
        if (out->n == kMaxToks) {
            return false;
        }
        size_t len = 0;
        while (p[len] != '\0' && p[len] != ' ' && p[len] != '\t') {
            ++len;
        }
        if (len >= kTokLen) {
            return false;
        }
        memcpy(out->t[out->n], p, len);
        out->t[out->n][len] = '\0';
        ++out->n;
        p += len;
    }
    return true;
}

bool parse_uint(const char* s, int32_t max, int32_t* out) {
    char* end = nullptr;
    // strtol's own type; range-checked before narrowing.
    const long v = strtol(s, &end, 10); // goose-lint: disable=fixed-width-int
    if (end == s || *end != '\0' || v < 0 || v > max) {
        return false;
    }
    *out = static_cast<int32_t>(v);
    return true;
}

// Channel by name or index.
bool parse_channel(const char* s, const BenchSetpoints& sp, int8_t* out) {
    for (uint8_t i = 0; i < sp.num_channels(); ++i) {
        const char* name = sp.channel(i).name;
        if (name != nullptr && strcmp(s, name) == 0) {
            *out = static_cast<int8_t>(i);
            return true;
        }
    }
    int32_t idx = 0;
    if (!parse_uint(s, static_cast<int32_t>(sp.num_channels()) - 1, &idx)) {
        return false;
    }
    *out = static_cast<int8_t>(idx);
    return true;
}

bool is(const char* tok, const char* word) { return strcmp(tok, word) == 0; }

// Commands with no arguments.
bool parse_word(const Tokens& tk, BenchCommand* out) {
    const char* w = tk.t[0];
    if (tk.n == 2 && is(w, "bench") && is(tk.t[1], "exit")) {
        out->op = BenchOp::kExit;
        return true;
    }
    if (tk.n != 1) {
        return false;
    }
    if (is(w, "bench")) {
        out->op = BenchOp::kEnter;
    } else if (is(w, "exit")) {
        out->op = BenchOp::kExit;
    } else if (is(w, "arm")) {
        out->op = BenchOp::kArm;
    } else if (is(w, "disarm")) {
        out->op = BenchOp::kDisarm;
    } else if (is(w, "clear")) {
        out->op = BenchOp::kClear;
    } else if (is(w, "center")) {
        out->op = BenchOp::kCenter;
    } else {
        return false;
    }
    return true;
}

} // namespace

const char* to_string(BenchOp op) {
    switch (op) {
    case BenchOp::kNone:
        return "none";
    case BenchOp::kEnter:
        return "bench";
    case BenchOp::kExit:
        return "bench exit";
    case BenchOp::kArm:
        return "arm";
    case BenchOp::kDisarm:
        return "disarm";
    case BenchOp::kClear:
        return "clear";
    case BenchOp::kSetUs:
        return "us";
    case BenchOp::kCenter:
        return "center";
    case BenchOp::kSweep:
        return "sweep";
    }
    return "unknown";
}

bool parse_bench_line(const char* line, const BenchSetpoints& sp,
                      BenchCommand* out) {
    *out = BenchCommand{};
    Tokens tk;
    if (line == nullptr || !tokenize(line, &tk) || tk.n == 0) {
        return false;
    }
    if (parse_word(tk, out)) {
        return true;
    }
    const char* w = tk.t[0];
    if (is(w, "us") && tk.n == 3) {
        int32_t us = 0;
        if (!parse_channel(tk.t[1], sp, &out->channel) ||
            !parse_uint(tk.t[2], 65535, &us)) {
            *out = BenchCommand{};
            return false;
        }
        out->op = BenchOp::kSetUs;
        out->us = static_cast<uint16_t>(us);
        return true;
    }
    if (is(w, "sweep") && tk.n == 2) {
        if (!is(tk.t[1], "all") && !parse_channel(tk.t[1], sp, &out->channel)) {
            *out = BenchCommand{};
            return false;
        }
        out->op = BenchOp::kSweep;
        return true;
    }
    *out = BenchCommand{};
    return false;
}
