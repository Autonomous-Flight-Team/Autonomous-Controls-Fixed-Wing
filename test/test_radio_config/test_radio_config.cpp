// Native tests for radio_config against a fake SiK radio: `pio test -e native`.

#include <unity.h>

#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <string>

#include "comms_platform.h"
#include "radio_config.h"

// Behaves like SiK firmware in AT mode: needs 1 s of silence before "+++", echoes
// commands, answers ATI5 / ATSn= / AT&E / AT&W / ATZ / ATO.
class FakeSikRadio : public ByteStream
{
public:
    int regs[32] = {};
    std::string aes_hex = "00000000000000000000000000000000";
    bool powered = true;
    bool command_mode = false;
    int saves = 0, reboots = 0, writes = 0;
    std::string log; // every command seen

    FakeSikRadio()
    {
        regs[1] = 57;
        regs[2] = 64;
        regs[3] = 25;
        regs[4] = 30;
        regs[5] = 0;
        regs[6] = 1;
        regs[15] = 0;
        saved_regs_ = std::string((const char *)regs, sizeof(regs));
    }

    int available() override { return (int)(out_.size() - pos_); }
    int read() override { return pos_ < out_.size() ? (uint8_t)out_[pos_++] : -1; }
    int availableForWrite() override { return 1024; }
    size_t write(const uint8_t *d, size_t n) override
    {
        if (!powered)
            return n;
        for (size_t i = 0; i < n; i++)
            byte_in((char)d[i]);
        last_write_us_ = comms_time_us();
        return n;
    }

private:
    void reply(const std::string &s) { out_ += s + "\r\n"; }

    void byte_in(char c)
    {
        if (!command_mode)
        {
            if (c == '+' && plus_ == 0 && comms_time_us() - last_write_us_ < 1000000)
                return; // no guard time: just data
            plus_ = (c == '+') ? plus_ + 1 : 0;
            if (plus_ == 3)
            {
                command_mode = true;
                plus_ = 0;
                reply("OK");
            }
            return;
        }
        if (c != '\r')
        {
            line_ += c;
            return;
        }
        std::string cmd = line_;
        line_.clear();
        log += cmd + ";";
        reply(cmd); // echo
        handle(cmd);
    }

    void handle(const std::string &cmd)
    {
        int reg, val;
        if (cmd == "AT")
            reply("OK");
        else if (cmd == "ATI5")
        {
            const char *names[32] = {"FORMAT", "SERIAL_SPEED", "AIR_SPEED", "NETID", "TXPOWER", "ECC", "MAVLINK", "OPPRESEND"};
            char line[64];
            for (int r : {0, 1, 2, 3, 4, 5, 6, 7, 13, 15})
            {
                snprintf(line, sizeof(line), "S%d:%s=%d", r, r < 8 ? names[r] : (r == 13 ? "RTSCTS" : "ENCRYPTION_LEVEL"), regs[r]);
                reply(line);
            }
        }
        else if (sscanf(cmd.c_str(), "ATS%d=%d", &reg, &val) == 2 && reg < 32)
        {
            regs[reg] = val;
            writes++;
            reply("OK");
        }
        else if (cmd == "AT&E?")
            reply(aes_hex);
        else if (cmd.rfind("AT&E=", 0) == 0)
        {
            aes_hex = cmd.substr(5);
            writes++;
            reply("OK");
        }
        else if (cmd == "AT&W")
        {
            saves++;
            saved_regs_ = std::string((const char *)regs, sizeof(regs));
            reply("OK");
        }
        else if (cmd == "ATZ")
        {
            reboots++;
            command_mode = false;
            memcpy(regs, saved_regs_.data(), sizeof(regs)); // unsaved changes are lost
        }
        else if (cmd == "ATO")
        {
            command_mode = false;
            reply("OK");
        }
        else
            reply("ERROR");
    }

    std::string out_, line_, saved_regs_;
    size_t pos_ = 0;
    int plus_ = 0;
    uint64_t last_write_us_ = 0;
};

void setUp()
{
    comms_test_wipe_store();
    comms_test_set_time_us(5000000);
}
void tearDown() {}

static void test_factory_radio_gets_configured_then_left_alone()
{
    FakeSikRadio radio;
    RadioSettings s; // AES off
    TEST_ASSERT_EQUAL(RADIO_CONFIG_APPLIED, radio_config_apply(radio, s));
    TEST_ASSERT_EQUAL(COMMS_RADIO_NETID, radio.regs[3]);
    TEST_ASSERT_EQUAL(1, radio.saves);
    TEST_ASSERT_EQUAL(1, radio.reboots);
    TEST_ASSERT_EQUAL(1, radio.writes); // only NETID differed

    TEST_ASSERT_EQUAL(RADIO_CONFIG_UNCHANGED, radio_config_apply(radio, s));
    TEST_ASSERT_EQUAL(1, radio.saves);
    TEST_ASSERT_FALSE(radio.command_mode); // left with ATO
}

static void test_aes_on_sets_level_and_key()
{
    FakeSikRadio radio;
    RadioSettings s;
    s.aes = true;
    for (int i = 0; i < RADIO_AES_KEY_LEN; i++)
        s.aes_key[i] = (uint8_t)(0xA0 + i); // fake test key
    TEST_ASSERT_EQUAL(RADIO_CONFIG_APPLIED, radio_config_apply(radio, s));
    TEST_ASSERT_EQUAL(1, radio.regs[15]);
    TEST_ASSERT_EQUAL_STRING("A0A1A2A3A4A5A6A7A8A9AAABACADAEAF", radio.aes_hex.c_str());
    // same key in lower case from the radio still counts as a match
    for (auto &ch : radio.aes_hex)
        ch = (char)tolower(ch);
    TEST_ASSERT_EQUAL(RADIO_CONFIG_UNCHANGED, radio_config_apply(radio, s));

    // turning AES off again only touches S15
    s.aes = false;
    TEST_ASSERT_EQUAL(RADIO_CONFIG_APPLIED, radio_config_apply(radio, s));
    TEST_ASSERT_EQUAL(0, radio.regs[15]);
}

static void test_default_settings_need_aes_key_when_aes_on()
{
    RadioSettings s;
    TEST_ASSERT_EQUAL(COMMS_RADIO_AES == 0, radio_config_default_settings(s));
    uint8_t key[RADIO_AES_KEY_LEN] = {1};
    signing_save_aes_key(key);
    TEST_ASSERT_TRUE(radio_config_default_settings(s));
}

static void test_silent_radio_reports_no_response()
{
    FakeSikRadio radio;
    radio.powered = false;
    RadioSettings s;
    uint64_t t0 = comms_time_us();
    TEST_ASSERT_EQUAL(RADIO_CONFIG_NO_RESPONSE, radio_config_apply(radio, s));
    TEST_ASSERT_TRUE(comms_time_us() - t0 < 5000000); // gives up within a few seconds
}

static void test_guard_time_is_respected()
{
    FakeSikRadio radio;
    // the fake only enters AT mode if the line was quiet for 1 s before "+++"
    radio.write((const uint8_t *)"x", 1);
    RadioSettings s;
    TEST_ASSERT_NOT_EQUAL(RADIO_CONFIG_NO_RESPONSE, radio_config_apply(radio, s));
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_factory_radio_gets_configured_then_left_alone);
    RUN_TEST(test_aes_on_sets_level_and_key);
    RUN_TEST(test_default_settings_need_aes_key_when_aes_on);
    RUN_TEST(test_silent_radio_reports_no_response);
    RUN_TEST(test_guard_time_is_respected);
    return UNITY_END();
}
