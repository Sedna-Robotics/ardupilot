#pragma once

#if AP_BATTERY_VEDIRECT_ENABLED

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

// ===== Driver tuning =====
#ifndef VEDIRECT_TIMEOUT_MS
#define VEDIRECT_TIMEOUT_MS   5000U   // mark stale if no good block in 5 s
#endif

#ifndef VEDIRECT_MAX_FIELDS
#define VEDIRECT_MAX_FIELDS   24      // per spec ~22; give a little headroom
#endif

#ifndef VEDIRECT_MAX_LABEL
#define VEDIRECT_MAX_LABEL    16      // spec ~9; allow future growth + NUL
#endif

#ifndef VEDIRECT_MAX_VALUE
#define VEDIRECT_MAX_VALUE    64      // spec ~33; allow longer values + NUL
#endif

// Enable to get verbose logs into DF logs / console (keep off in production)
#ifndef VEDIRECT_DEBUG
#define VEDIRECT_DEBUG        1
#endif

#if VEDIRECT_DEBUG
  #define VEDBG(fmt, ...) do { ::hal.console->printf("[VED] " fmt "\n", ##__VA_ARGS__); } while (0)
#else
  #define VEDBG(...) do {} while (0)
#endif

// Sub class for Victron VE.Direct battery monitors
class AP_BattMonitor_VEDirect : public AP_BattMonitor_Backend 
{
public:
    // Inherit constructor
    using AP_BattMonitor_Backend::AP_BattMonitor_Backend;

    void init(void) override;
    void read() override;

    bool has_current() const override { return true; }
    bool has_consumed_energy() const override { return true; }
    bool has_temperature() const override { return true; }


private:
    AP_BattMonitor_VEDirect(AP_BattMonitor &mon, uint8_t instance);

    void thread_main();
    bool init_uart();
    void poll_uart();
    void process_byte(uint8_t b);

    void start_new_block();
    void start_new_line();
    void commit_line();
    void finish_block_if_checksum_ok();
    void apply_fields();
    void decode_field(const char* label, const char* value);

    // --------- parsing storage ----------
    struct FieldKV {
        char label[VEDIRECT_MAX_LABEL];
        char value[VEDIRECT_MAX_VALUE];
    };
    FieldKV _kv[VEDIRECT_MAX_FIELDS];
    uint8_t _kv_count = 0;

    // state machine
    enum class ParseState : uint8_t {
        IDLE,
        RECORD_BEGIN,
        RECORD_NAME,
        RECORD_VALUE,
        CHECKSUM,
        SKIP_HEX
    };
    ParseState   _parse_state = ParseState::IDLE;

    // running checksum: modulo-256 sum of ALL bytes in the block (including \r,\n,\t, label, value, Checksum line, etc.)
    uint8_t _cksum_running = 0;
    bool    _saw_checksum_line = false;

    // current line buffers + overflow guards
    char    _label_buf[VEDIRECT_MAX_LABEL] = {};
    char    _value_buf[VEDIRECT_MAX_VALUE] = {};
    uint8_t _label_len = 0;
    uint8_t _value_len = 0;
    bool    _label_overflow = false;
    bool    _value_overflow = false;

    // UART
    AP_HAL::UARTDriver* _uart = nullptr;
    bool _uart_ok = false;

    // health timer
    uint32_t _last_good_block_ms = 0;
};
#endif // AP_BATTERY_VEDIRECT_ENABLED