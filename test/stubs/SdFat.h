// Minimal SdFat stand-in for the native (PC) test build.
// Everything the logger writes lands in g_sd so tests can inspect it.
#ifndef GOOSE_STUBS_SD_FAT_H_
#define GOOSE_STUBS_SD_FAT_H_

#include <fcntl.h>
#include <stddef.h>
#include <stdint.h>
#include <string>
#include <vector>

#ifndef O_AT_END
#define O_AT_END 0x4000
#endif

#define FIFO_SDIO 0

struct SdStub {
    std::vector<uint8_t> data;
    std::vector<size_t> write_sizes;
    std::string filename;
    int open_flags = 0;
    int open_calls = 0;
    int sync_calls = 0;
    int close_calls = 0;
    bool fail_begin = false;
    bool fail_open = false;
    bool fail_write = false;

    void reset() { *this = SdStub(); }
};

extern SdStub g_sd;

struct SdioConfig {
    explicit SdioConfig(int) {}
};

class SdFat {
public:
    bool begin(SdioConfig) { return !g_sd.fail_begin; }
};

class FsFile {
public:
    bool open(const char *path, int flags) {
        g_sd.open_calls++;
        g_sd.filename = path;
        g_sd.open_flags = flags;
        is_open_ = !g_sd.fail_open;
        return is_open_;
    }
    size_t write(const void *buf, size_t n) {
        if (g_sd.fail_write) {
            return 0;
        }
        const uint8_t *p = static_cast<const uint8_t *>(buf);
        g_sd.data.insert(g_sd.data.end(), p, p + n);
        g_sd.write_sizes.push_back(n);
        return n;
    }
    bool sync() {
        g_sd.sync_calls++;
        return true;
    }
    bool close() {
        g_sd.close_calls++;
        is_open_ = false;
        return true;
    }
    bool isOpen() const { return is_open_; }

private:
    bool is_open_ = false;
};

#endif // GOOSE_STUBS_SD_FAT_H_
