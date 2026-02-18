/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_BattMonitor_config.h"

#if AP_BATTERY_VEDIRECT_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_HAL/AP_HAL.h>
#include <algorithm>
#include <cctype>
#include <AP_SerialManager/AP_SerialManager.h>

#include "AP_BattMonitor_VEDirect.h"

extern const AP_HAL::HAL& hal;

// parse a signed int32 from a string; returns false if not a valid int32
static inline bool parse_int32(const char* s, int32_t& out) {
    char* endp = nullptr;
    long v = strtol(s, &endp, 10);
    if (endp == s || *endp != '\0') { return false; }
    out = (int32_t)v; return true;
}

// thread function for background serial reading
void AP_BattMonitor_VEDirect::thread_main()
{
    if (!init_uart()) {
        return;
    }

    while (true) {
        hal.scheduler->delay_microseconds(1000);
        poll_uart();
    }
}

bool AP_BattMonitor_VEDirect::init_uart()
{
    _uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_VEDirect, 0);
    if (_uart == nullptr) {
        VEDBG("no UART available");
        _uart_ok = false;
        return false;
    }

    // 19200 8N1, no flow control
    _uart->begin(19200, 256, 256);
    _uart->set_flow_control(AP_HAL::UARTDriver::flow_control::FLOW_CONTROL_DISABLE);
    _uart_ok = true;

    start_new_block();
    start_new_line();
    VEDBG("init OK");

    return true;
}

void AP_BattMonitor_VEDirect::init()
{
    // create background thread for serial reading
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_BattMonitor_VEDirect::thread_main, void), "ve_direct", 2048, AP_HAL::Scheduler::PRIORITY_UART, 1)) {
        return;
    }
}

void AP_BattMonitor_VEDirect::read()
{
    _state.healthy = ((AP_HAL::millis() - _last_good_block_ms) < VEDIRECT_TIMEOUT_MS);
}

void AP_BattMonitor_VEDirect::poll_uart()
{
    uint8_t b;
    while (_uart->available() > 0) {
        if (_uart->read(&b, 1) != 1) { break; }
        process_byte(b);
    }
}

void AP_BattMonitor_VEDirect::start_new_block()
{
    _cksum_running = 0;
    _saw_checksum_line = false;
    _kv_count = 0;
}

void AP_BattMonitor_VEDirect::start_new_line()
{
    _label_len = 0;
    _value_len = 0;
    _label_buf[0] = 0;
    _value_buf[0] = 0;
    _label_overflow = false;
    _value_overflow = false;
}

void AP_BattMonitor_VEDirect::commit_line()
{
    // Safely NUL-terminate what we have (even if overflow flagged)
    _label_buf[std::min<uint8_t>(_label_len, VEDIRECT_MAX_LABEL-1)] = 0;
    _value_buf[std::min<uint8_t>(_value_len, VEDIRECT_MAX_VALUE-1)] = 0;

    if (_label_overflow || _value_overflow) {
        VEDBG("line truncated: '%s'='%s'", _label_buf, _value_buf);
        // drop overly long lines; don't add to _kv
        return;
    }

    if (_kv_count < VEDIRECT_MAX_FIELDS) {
        if (strcmp(_label_buf, "CHECKSUM") == 0) {
            _saw_checksum_line = true;
        }
        strncpy(_kv[_kv_count].label, _label_buf, sizeof(_kv[_kv_count].label));
        _kv[_kv_count].label[sizeof(_kv[_kv_count].label) - 1] = '\0';
        strncpy(_kv[_kv_count].value, _value_buf, sizeof(_kv[_kv_count].value));
        _kv[_kv_count].value[sizeof(_kv[_kv_count].value) - 1] = '\0';
        _kv_count++;
    } else {
        VEDBG("too many fields in block; dropping extra");
    }
}

void AP_BattMonitor_VEDirect::process_byte(uint8_t b)
{
    // HEX line detection: per FAQ, HEX lines begin with ':'; ignore entire line
    if (_parse_state != ParseState::SKIP_HEX && b == ':') {
        VEDBG("HEX line detected; skipping until NL");
        _parse_state = ParseState::SKIP_HEX;
    }

    if (_parse_state != ParseState::SKIP_HEX) {
        _cksum_running += b;
    }

    b = toupper(b);

    switch (_parse_state) {
    case ParseState::IDLE:
        /* wait for \n of the start of an record */
        switch(b) {
        case '\n':
            _parse_state = ParseState::RECORD_BEGIN;
            break;
        case '\r': /* skip */
        default:
            break;
        }
        break;

    case ParseState::RECORD_BEGIN:
        start_new_line();
        _parse_state = ParseState::RECORD_NAME;
        FALLTHROUGH;

    case ParseState::RECORD_NAME:
        switch(b) {
        case '\t':
            // if this is a CHECKSUM line, change state to CHECKSUM
            if (strcmp(_label_buf, "CHECKSUM") == 0) {
                _parse_state = ParseState::CHECKSUM;
                break;
            } 
            _parse_state = ParseState::RECORD_VALUE;
            break;
        default:
            if (_label_len < VEDIRECT_MAX_LABEL-1) {
                _label_buf[_label_len++] = char(b);
            } else {
                _label_overflow = true;
            }
            break;
        }
        break;

    case ParseState::RECORD_VALUE:
        // the value is being received. newline indicates a new record
        switch(b) {
        case '\n':
            commit_line();
            _parse_state = ParseState::RECORD_BEGIN;
            break;
        case '\r': /* skip */
            break;
        default:
            // accumulate value characters
            if (_value_len < VEDIRECT_MAX_VALUE-1) {
                _value_buf[_value_len++] = char(b);
            } else {
                _value_overflow = true;
            }
            break;
        }
        break;

    case ParseState::CHECKSUM:
        finish_block_if_checksum_ok();
        start_new_block();
        _parse_state = ParseState::IDLE;
        break;

    case ParseState::SKIP_HEX:
        _cksum_running = 0;
        _parse_state = ParseState::IDLE;
        break;

    default:
        break;
    }
}

void AP_BattMonitor_VEDirect::finish_block_if_checksum_ok()
{
    if (_cksum_running == 0) {
        VEDBG("block OK (%u fields)", (unsigned)_kv_count);
        apply_fields();
        _last_good_block_ms = AP_HAL::millis();
    } else {
        VEDBG("bad checksum; dropping block");
    }
}

void AP_BattMonitor_VEDirect::apply_fields()
{
    char decoded_fields[128] = "";
    size_t offset = 0;
    for (uint8_t i = 0; i < _kv_count; i++) {
        const char* L = _kv[i].label;
        const char* V = _kv[i].value;

        if (i > 0 && offset < sizeof(decoded_fields) - 1) {
            decoded_fields[offset++] = ',';
            decoded_fields[offset] = '\0';
        }
        if (offset < sizeof(decoded_fields) - 1) {
            size_t label_len = strlen(L);
            size_t remaining = sizeof(decoded_fields) - offset - 1;
            size_t copy_len = (label_len < remaining) ? label_len : remaining;
            memcpy(decoded_fields + offset, L, copy_len);
            offset += copy_len;
            decoded_fields[offset] = '\0';
        }

        decode_field(L, V);
    }
    VEDBG("Decoded fields: %s", decoded_fields);
}

void AP_BattMonitor_VEDirect::decode_field(const char* label, const char* value)
{
    int32_t iv = 0;

    // Battery voltage (V)
    if (!strcmp(label, "V")) {
        if (parse_int32(value, iv)) {
            _state.voltage = 0.001f * (float)iv;       // mV -> V
        }
        return;
    }

    // Panel voltage (VPV)
    if (!strcmp(label, "VPV")) {
        // TODO: putting voltage (V) into temperature for now (uses existing telemetry messages)
        if (parse_int32(value, iv)) {
            _state.temperature = 0.001f * (float)iv; // mV -> V
        }
        return;
    }

    // Panel power (PPV)
    if (!strcmp(label, "PPV")) {
        // TODO: putting power (W) into mah consumed for now (uses existing telemetry messages)
        if (parse_int32(value, iv)) {
            _state.consumed_mah = (float)iv;   // W
        }
        return;
    }

    // Battery current (I)
    if (!strcmp(label, "I")) {
        if (parse_int32(value, iv)) {
            // VE.Direct I: + means CHARGING (into battery)
            // ArduPilot: + means DISCHARGING (from battery)
            _state.current_amps = -0.001f * (float)iv;       // mA -> A, invert sign
        }
        return;
    }

    // Load current (IL)
    if (!strcmp(label, "IL")) {
        // save as local variable for now; could be used for load current calc later
        if (parse_int32(value, iv)) {
            // float load_current = 0.001f * (float)iv; // mA
            // (currently unused)
        }
        return;
    }

    // Load ParseState (on/off) (LOAD)
    if (!strcmp(label, "LOAD")) {
        // save as local variable for now; could be used for load state calc later
        if (parse_int32(value, iv)) {
            // bool load_on = (iv != 0);
            // (currently unused)
        }
        return;
    }

    // Relay state (on/off) (Relay)
    if (!strcmp(label, "RELAY")) {
        // save as local variable for now; could be used for relay state calc later
        if (parse_int32(value, iv)) {
            // bool relay_on = (iv != 0);
            // (currently unused)
        }
        return;
    }

    // Off reason (OR)
    if (!strcmp(label, "OR")) {
        // save as local variable for now; could be used for off reason calc later
        if (parse_int32(value, iv)) {
            // int off_reason = iv;
            // (currently unused)
        }
        return;
    }

    // Unknown labels fall through; stored in _extra for debugging/telemetry.
}
#endif // AP_BATTERY_VEDIRECT_ENABLED
