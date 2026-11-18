#include "signing.h"

#include <stdio.h>
#include <string.h>

#include "comms_config.h"
#include "comms_mavlink.h"
#include "comms_platform.h"

static const uint16_t ADDR_KEY_MAGIC = COMMS_EEPROM_BASE + 0;
static const uint16_t ADDR_KEY = COMMS_EEPROM_BASE + 4;
static const uint16_t ADDR_TIMESTAMP = COMMS_EEPROM_BASE + 36;
static const uint16_t ADDR_AES_MAGIC = COMMS_EEPROM_BASE + 44;
static const uint16_t ADDR_AES_KEY = COMMS_EEPROM_BASE + 48;

static const uint8_t KEY_MAGIC[4] = {'G', 'S', 'K', '1'};
static const uint8_t AES_MAGIC[4] = {'G', 'A', 'E', '1'};
static const uint8_t ERASED[4] = {0xFF, 0xFF, 0xFF, 0xFF};

static bool magic_ok(uint16_t addr, const uint8_t magic[4])
{
    uint8_t m[4];
    comms_store_read(addr, m, 4);
    return memcmp(m, magic, 4) == 0;
}

bool signing_load_key(uint8_t key[SIGNING_KEY_LEN])
{
    if (!magic_ok(ADDR_KEY_MAGIC, KEY_MAGIC))
        return false;
    comms_store_read(ADDR_KEY, key, SIGNING_KEY_LEN);
    return true;
}

void signing_save_key(const uint8_t key[SIGNING_KEY_LEN])
{
    comms_store_write(ADDR_KEY, key, SIGNING_KEY_LEN);
    comms_store_write(ADDR_KEY_MAGIC, KEY_MAGIC, 4);
}

void signing_clear_key()
{
    uint8_t blank[SIGNING_KEY_LEN];
    memset(blank, 0xFF, sizeof(blank));
    comms_store_write(ADDR_KEY_MAGIC, ERASED, 4);
    comms_store_write(ADDR_KEY, blank, sizeof(blank));
}

uint64_t signing_load_timestamp()
{
    uint64_t ts;
    comms_store_read(ADDR_TIMESTAMP, &ts, sizeof(ts));
    return ts == UINT64_MAX ? 0 : ts; // erased EEPROM
}

void signing_save_timestamp(uint64_t ts) { comms_store_write(ADDR_TIMESTAMP, &ts, sizeof(ts)); }

uint64_t signing_boot_timestamp() { return signing_load_timestamp() + SIGNING_BOOT_BUMP_TICKS; }

bool signing_load_aes_key(uint8_t key[RADIO_AES_KEY_LEN])
{
    if (!magic_ok(ADDR_AES_MAGIC, AES_MAGIC))
        return false;
    comms_store_read(ADDR_AES_KEY, key, RADIO_AES_KEY_LEN);
    return true;
}

void signing_save_aes_key(const uint8_t key[RADIO_AES_KEY_LEN])
{
    comms_store_write(ADDR_AES_KEY, key, RADIO_AES_KEY_LEN);
    comms_store_write(ADDR_AES_MAGIC, AES_MAGIC, 4);
}

void signing_clear_aes_key()
{
    uint8_t blank[RADIO_AES_KEY_LEN];
    memset(blank, 0xFF, sizeof(blank));
    comms_store_write(ADDR_AES_MAGIC, ERASED, 4);
    comms_store_write(ADDR_AES_KEY, blank, sizeof(blank));
}

void signing_fingerprint(const uint8_t key[SIGNING_KEY_LEN], char out[13])
{
    mavlink_sha256_ctx ctx;
    uint8_t digest[6];
    mavlink_sha256_init(&ctx);
    mavlink_sha256_update(&ctx, key, SIGNING_KEY_LEN);
    mavlink_sha256_final_48(&ctx, digest);
    for (int i = 0; i < 6; i++)
        snprintf(&out[i * 2], 3, "%02x", digest[i]);
}

bool signing_allow_unsigned(uint32_t msgid, uint8_t sysid, uint8_t compid, bool require_signing)
{
    if (!require_signing)
        return true;
    return msgid == MAVLINK_MSG_ID_RADIO_STATUS && sysid == '3' && compid == 'D';
}

static int hex_val(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    return -1;
}

bool signing_parse_hex(const char *hex, uint8_t *out, size_t n)
{
    if (strlen(hex) != n * 2)
        return false;
    for (size_t i = 0; i < n; i++)
    {
        int hi = hex_val(hex[2 * i]), lo = hex_val(hex[2 * i + 1]);
        if (hi < 0 || lo < 0)
            return false;
        out[i] = (uint8_t)(hi << 4 | lo);
    }
    return true;
}

bool signing_console_command(const char *line, char *reply, size_t reply_len, bool *key_changed)
{
    if (key_changed)
        *key_changed = false;

    uint8_t key[SIGNING_KEY_LEN];
    char fp[13];

    if (strcmp(line, "KEY?") == 0)
    {
        if (signing_load_key(key))
        {
            signing_fingerprint(key, fp);
            snprintf(reply, reply_len, "KEY set, fingerprint %s", fp);
        }
        else
            snprintf(reply, reply_len, "KEY not set");
        memset(key, 0, sizeof(key));
        return true;
    }
    if (strcmp(line, "KEY CLEAR") == 0)
    {
        signing_clear_key();
        if (key_changed)
            *key_changed = true;
        snprintf(reply, reply_len, "KEY cleared");
        return true;
    }
    if (strncmp(line, "KEY ", 4) == 0)
    {
        if (!signing_parse_hex(line + 4, key, SIGNING_KEY_LEN))
        {
            snprintf(reply, reply_len, "KEY error: need 64 hex characters");
            return true;
        }
        signing_save_key(key);
        signing_fingerprint(key, fp);
        memset(key, 0, sizeof(key));
        if (key_changed)
            *key_changed = true;
        snprintf(reply, reply_len, "KEY stored, fingerprint %s", fp);
        return true;
    }

    uint8_t aes[RADIO_AES_KEY_LEN];
    if (strcmp(line, "AESKEY?") == 0)
    {
        snprintf(reply, reply_len, signing_load_aes_key(aes) ? "AESKEY set" : "AESKEY not set");
        memset(aes, 0, sizeof(aes));
        return true;
    }
    if (strcmp(line, "AESKEY CLEAR") == 0)
    {
        signing_clear_aes_key();
        snprintf(reply, reply_len, "AESKEY cleared");
        return true;
    }
    if (strncmp(line, "AESKEY ", 7) == 0)
    {
        if (!signing_parse_hex(line + 7, aes, RADIO_AES_KEY_LEN))
        {
            snprintf(reply, reply_len, "AESKEY error: need 32 hex characters");
            return true;
        }
        signing_save_aes_key(aes);
        memset(aes, 0, sizeof(aes));
        snprintf(reply, reply_len, "AESKEY stored (applied to the radio at next boot)");
        return true;
    }
    return false;
}
