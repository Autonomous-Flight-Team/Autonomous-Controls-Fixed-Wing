#include "radio_config.h"

#include <ctype.h>
#include <strings.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "comms_platform.h"

static const uint32_t GUARD_MS = 1100;   // SiK needs 1 s of silence before and after "+++"
static const uint32_t REPLY_MS = 500;
static const int MAX_REG = 32;

static void drain(ByteStream &r)
{
    while (r.available() > 0)
        r.read();
}

static void send(ByteStream &r, const char *s) { r.write((const uint8_t *)s, strlen(s)); }

// Read one non-empty line (CR/LF stripped). False on timeout.
static bool read_line(ByteStream &r, char *buf, size_t n, uint32_t timeout_ms)
{
    uint64_t end = comms_time_us() + (uint64_t)timeout_ms * 1000;
    size_t len = 0;
    while (comms_time_us() < end)
    {
        if (r.available() <= 0)
        {
            comms_delay_ms(1);
            continue;
        }
        int c = r.read();
        if (c == '\r' || c == '\n')
        {
            if (len == 0)
                continue;
            buf[len] = '\0';
            return true;
        }
        if (len < n - 1)
            buf[len++] = (char)c;
    }
    buf[len] = '\0';
    return false;
}

static bool expect_ok(ByteStream &r, uint32_t timeout_ms)
{
    char line[80];
    while (read_line(r, line, sizeof(line), timeout_ms))
    {
        if (strcmp(line, "OK") == 0)
            return true;
        if (strcmp(line, "ERROR") == 0)
            return false;
        // anything else is the radio echoing our command
    }
    return false;
}

static bool command_ok(ByteStream &r, const char *cmd)
{
    send(r, cmd);
    send(r, "\r");
    return expect_ok(r, REPLY_MS);
}

static bool enter_command_mode(ByteStream &r)
{
    comms_delay_ms(GUARD_MS);
    drain(r);
    send(r, "+++");
    if (expect_ok(r, GUARD_MS + 1000))
        return true;
    return command_ok(r, "AT"); // maybe it was already in command mode
}

// Parse ATI5 lines such as "S3:NETID=25".
static bool read_registers(ByteStream &r, int32_t regs[MAX_REG], bool have[MAX_REG])
{
    for (int i = 0; i < MAX_REG; i++)
        have[i] = false;
    send(r, "ATI5\r");
    char line[80];
    int count = 0;
    while (read_line(r, line, sizeof(line), REPLY_MS))
    {
        if (line[0] != 'S' || !isdigit((unsigned char)line[1]))
            continue;
        char *p;
        long reg = strtol(line + 1, &p, 10);
        const char *eq = strchr(line, '=');
        if (*p != ':' || !eq || reg < 0 || reg >= MAX_REG)
            continue;
        regs[reg] = (int32_t)strtol(eq + 1, nullptr, 10);
        have[reg] = true;
        count++;
    }
    return count > 0;
}

static void key_to_hex(const uint8_t key[RADIO_AES_KEY_LEN], char out[RADIO_AES_KEY_LEN * 2 + 1])
{
    for (int i = 0; i < RADIO_AES_KEY_LEN; i++)
        snprintf(&out[i * 2], 3, "%02X", key[i]);
}

static bool aes_key_matches(ByteStream &r, const char *want_hex)
{
    send(r, "AT&E?\r");
    char line[80];
    bool match = false;
    while (read_line(r, line, sizeof(line), REPLY_MS))
        if (strlen(line) == RADIO_AES_KEY_LEN * 2 && strncasecmp(line, want_hex, RADIO_AES_KEY_LEN * 2) == 0)
            match = true;
    return match;
}

bool radio_config_default_settings(RadioSettings &out)
{
    out = RadioSettings();
    if (!out.aes)
        return true;
    return signing_load_aes_key(out.aes_key);
}

RadioConfigResult radio_config_apply(ByteStream &radio, const RadioSettings &s)
{
    if (!enter_command_mode(radio))
        return RADIO_CONFIG_NO_RESPONSE;

    int32_t regs[MAX_REG];
    bool have[MAX_REG];
    if (!read_registers(radio, regs, have) || !have[2] || !have[3] || !have[6] || (s.aes && !have[15]))
    {
        command_ok(radio, "ATO");
        return RADIO_CONFIG_PARSE_ERROR;
    }

    struct
    {
        int reg;
        int32_t want;
    } wanted[] = {{2, s.air_speed}, {3, s.net_id}, {6, s.mavlink}, {15, s.aes ? 1 : 0}};

    bool changed = false;
    char cmd[64];
    for (auto &w : wanted)
    {
        if (!have[w.reg] || regs[w.reg] == w.want) // S15 may be absent on firmware without AES; fine if AES is off
            continue;
        snprintf(cmd, sizeof(cmd), "ATS%d=%ld", w.reg, (long)w.want);
        if (!command_ok(radio, cmd))
        {
            send(radio, "ATZ\r"); // reboot without AT&W: discard partial changes
            return RADIO_CONFIG_WRITE_FAILED;
        }
        changed = true;
    }

    if (s.aes)
    {
        char hex[RADIO_AES_KEY_LEN * 2 + 1];
        key_to_hex(s.aes_key, hex);
        if (!aes_key_matches(radio, hex))
        {
            snprintf(cmd, sizeof(cmd), "AT&E=%s", hex);
            if (!command_ok(radio, cmd))
            {
                send(radio, "ATZ\r");
                return RADIO_CONFIG_WRITE_FAILED;
            }
            changed = true;
        }
        memset(hex, 0, sizeof(hex));
    }
    memset(cmd, 0, sizeof(cmd));

    if (!changed)
    {
        command_ok(radio, "ATO");
        return RADIO_CONFIG_UNCHANGED;
    }
    if (!command_ok(radio, "AT&W"))
    {
        send(radio, "ATZ\r");
        return RADIO_CONFIG_WRITE_FAILED;
    }
    send(radio, "ATZ\r"); // reboot to apply; there is no OK
    comms_delay_ms(1500);
    drain(radio);
    return RADIO_CONFIG_APPLIED;
}

const char *radio_config_result_name(RadioConfigResult r)
{
    switch (r)
    {
    case RADIO_CONFIG_UNCHANGED: return "radio settings already correct";
    case RADIO_CONFIG_APPLIED: return "radio settings updated, radio rebooted";
    case RADIO_CONFIG_NO_RESPONSE: return "radio did not answer +++";
    case RADIO_CONFIG_PARSE_ERROR: return "could not read radio registers (ATI5)";
    case RADIO_CONFIG_WRITE_FAILED: return "radio rejected a setting";
    case RADIO_CONFIG_NO_AES_KEY: return "AES on but no AESKEY provisioned";
    }
    return "?";
}
