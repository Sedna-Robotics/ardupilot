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

#include "AP_TetherDrop_Serial.h"

#if AP_TETHERDROP_SERIAL_ENABLED

#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

// true if tether drop is healthy
bool AP_TetherDrop_Serial::healthy() const
{
    // healthy if we have received a status update in the last second
    return (AP_HAL::millis() - status.last_update_ms < STATUS_TIMEOUT_MS) && (uart != nullptr);
}

// initialise the tether drop
void AP_TetherDrop_Serial::init()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();
    
    // find the first serial port configured with Winch protocol (protocol 31)
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Winch, 0);
    
    if (uart == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "TetherDrop: No serial port found with protocol 31 (Winch)");
        return;
    }

    // configure serial port for 115200 8N1
    // large rx buffer so bursts of controller output are not lost between updates
    uart->begin(115200, 1024, 512);
    uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);

    // initialize state
    serial_buffer_len = 0;
    discard_line = false;
    last_checksum_report_ms = 0;
    last_overflow_report_ms = 0;
    status.last_update_ms = 0;
    memset(status.state, 0, sizeof(status.state));
    memset(user_update.state, 0, sizeof(user_update.state));
    user_update.last_ms = 0;

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TetherDrop: Initialized on Winch protocol");
    
    // Send a status request to kick off communication
    send_status_request();
}

// update - read from serial and send commands
void AP_TetherDrop_Serial::update()
{
    if (uart == nullptr) {
        return;
    }

    // read incoming data
    read_serial();

    // update user with state changes
    update_user();
}

// send command to winch controller
void AP_TetherDrop_Serial::send_command(const char* cmd, const char* params)
{
    if (uart == nullptr) {
        return;
    }

    char msg[MSG_BUFFER_SIZE];
    if (params != nullptr) {
        hal.util->snprintf(msg, sizeof(msg), "WINCH,%s,%s", cmd, params);
    } else {
        hal.util->snprintf(msg, sizeof(msg), "WINCH,%s", cmd);
    }

    uint8_t checksum = calculate_checksum(msg);
    
    char full_msg[MSG_BUFFER_SIZE];
    hal.util->snprintf(full_msg, sizeof(full_msg), "$%s*%02X\r\n", msg, checksum);
    
    uart->write((const uint8_t*)full_msg, strlen(full_msg));
}

// calculate XOR checksum
uint8_t AP_TetherDrop_Serial::calculate_checksum(const char* msg) const
{
    uint8_t checksum = 0;
    for (uint32_t i = 0; msg[i] != '\0'; i++) {
        checksum ^= msg[i];
    }
    return checksum;
}

// verify checksum in received message
bool AP_TetherDrop_Serial::verify_checksum(const char* msg) const
{
    // find the checksum delimiter
    const char* checksum_pos = strchr(msg, '*');
    if (checksum_pos == nullptr) {
        return false;
    }

    // calculate checksum on portion between $ and * (excluding both)
    // This matches the Arduino's sendMessage() which checksums after the $
    uint8_t calculated = 0;
    for (const char* p = msg + 1; p < checksum_pos; p++) {  // start after $, stop before *
        calculated ^= *p;
    }

    // parse received checksum (2 hex digits after *)
    if (checksum_pos[1] == '\0' || checksum_pos[2] == '\0') {
        return false;
    }
    char hex_str[3] = {checksum_pos[1], checksum_pos[2], '\0'};
    uint32_t received = strtoul(hex_str, nullptr, 16);

    return calculated == (uint8_t)received;
}

// returns true if an error of this kind should be reported to the user now
bool AP_TetherDrop_Serial::report_error(uint32_t &last_report_ms) const
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_report_ms < ERROR_REPORT_INTERVAL_MS) {
        return false;
    }
    last_report_ms = now_ms;
    return true;
}

// read data from serial port
void AP_TetherDrop_Serial::read_serial()
{
    if (uart == nullptr) {
        return;
    }

    uint32_t nbytes = uart->available();
    
    while (nbytes-- > 0) {
        char c = uart->read();

        // '$' always starts a new frame, so use it to resynchronise after
        // dropped bytes or after unterminated human readable output
        if (c == '$') {
            serial_buffer_len = 0;
            discard_line = false;
            serial_buffer[serial_buffer_len++] = c;
            continue;
        }

        if (c == '\r') {
            // ignore carriage return
            continue;
        }

        if (c == '\n') {
            if (!discard_line && serial_buffer_len > 0) {
                serial_buffer[serial_buffer_len] = '\0';
                parse_message(serial_buffer);
            }
            serial_buffer_len = 0;
            discard_line = false;
            continue;
        }

        if (discard_line) {
            continue;
        }

        if (serial_buffer_len < SERIAL_BUFFER_SIZE - 1) {
            serial_buffer[serial_buffer_len++] = c;
        } else {
            // over long line, drop the remainder rather than parsing its tail
            const bool was_message = (serial_buffer[0] == '$');
            serial_buffer_len = 0;
            discard_line = true;
            if (was_message && report_error(last_overflow_report_ms)) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "TetherDrop: Buffer overflow");
            }
        }
    }
}

// parse a complete message
void AP_TetherDrop_Serial::parse_message(const char* msg)
{
    // all messages should start with $
    if (msg[0] != '$') {
        return;
    }

    // determine message type first so human readable output which happens to
    // start with '$' is ignored rather than reported as a checksum error
    const char *fields = nullptr;
    void (AP_TetherDrop_Serial::*handler)(const char*) = nullptr;

    if (strncmp(msg, "$WSTAT,", 7) == 0) {
        handler = &AP_TetherDrop_Serial::parse_status_message;
        fields = msg + 7;
    } else if (strncmp(msg, "$WACK,", 6) == 0) {
        handler = &AP_TetherDrop_Serial::parse_ack_message;
        fields = msg + 6;
    } else if (strncmp(msg, "$WNAK,", 6) == 0) {
        handler = &AP_TetherDrop_Serial::parse_nak_message;
        fields = msg + 6;
    } else if (strncmp(msg, "$WEVT,", 6) == 0) {
        handler = &AP_TetherDrop_Serial::parse_event_message;
        fields = msg + 6;
    } else if (strncmp(msg, "$WERR,", 6) == 0) {
        handler = &AP_TetherDrop_Serial::parse_error_message;
        fields = msg + 6;
    } else {
        return;
    }

    // verify checksum
    if (!verify_checksum(msg)) {
        if (report_error(last_checksum_report_ms)) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "TetherDrop: Checksum error");
        }
        return;
    }

    (this->*handler)(fields);
}

// parse status update message
// Format: $WSTAT,STATE,DEPTH,SPEED,POSITION,RPM*CHECKSUM
void AP_TetherDrop_Serial::parse_status_message(const char* fields)
{
    // make a copy to tokenize
    char buffer[MSG_BUFFER_SIZE];
    strncpy(buffer, fields, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    // remove checksum portion
    char* checksum_pos = strchr(buffer, '*');
    if (checksum_pos != nullptr) {
        *checksum_pos = '\0';
    }

    // parse fields
    char* token = strtok(buffer, ",");
    if (token != nullptr) {
        strncpy(status.state, token, sizeof(status.state) - 1);
        status.state[sizeof(status.state) - 1] = '\0';
    }

    token = strtok(nullptr, ",");
    if (token != nullptr) {
        int32_t depth_mm = atoi(token);
        status.depth_m = depth_mm / 1000.0f;
    }

    token = strtok(nullptr, ",");
    if (token != nullptr) {
        int32_t speed_mm_s = atoi(token);
        status.speed_mps = speed_mm_s / 1000.0f;
    }

    token = strtok(nullptr, ",");
    if (token != nullptr) {
        status.position = atol(token);
    }

    token = strtok(nullptr, ",");
    if (token != nullptr) {
        int32_t rpm_x1000 = atoi(token);
        status.rpm = rpm_x1000 / 1000.0f;
    }

    status.last_update_ms = AP_HAL::millis();
    
    if ((config.options & uint16_t(AP_TetherDrop::Options::VerboseOutput)) != 0) {
        static uint32_t last_status_log_ms = 0;
        uint32_t now = AP_HAL::millis();
        if (now - last_status_log_ms > 2000) {  // Log every 2 seconds
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TetherDrop: Status: %s %.1fm %.3fmps pos=%ld rpm=%.1f", 
                          status.state, (double)status.depth_m, (double)status.speed_mps,
                          (long)status.position, (double)status.rpm);
            last_status_log_ms = now;
        }
    }
}

// parse acknowledgment message
void AP_TetherDrop_Serial::parse_ack_message(const char* fields)
{
    // ACK received, nothing to do since we don't retry commands
}

// parse negative acknowledgment message
void AP_TetherDrop_Serial::parse_nak_message(const char* fields)
{
    char buffer[MSG_BUFFER_SIZE];
    strncpy(buffer, fields, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';
    
    char* checksum_pos = strchr(buffer, '*');
    if (checksum_pos != nullptr) {
        *checksum_pos = '\0';
    }

    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "TetherDrop: NAK - %s", buffer);
}

// parse event message
void AP_TetherDrop_Serial::parse_event_message(const char* fields)
{
    char buffer[MSG_BUFFER_SIZE];
    strncpy(buffer, fields, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';
    
    char* checksum_pos = strchr(buffer, '*');
    if (checksum_pos != nullptr) {
        *checksum_pos = '\0';
    }

    // parse event type and states
    char* token = strtok(buffer, ",");
    if (token == nullptr || strcmp(token, "STATE") != 0) {
        return;
    }

    char from_state[16] = "";
    char to_state[16] = "";

    token = strtok(nullptr, ",");
    if (token != nullptr) {
        strncpy(from_state, token, sizeof(from_state) - 1);
    }

    token = strtok(nullptr, ",");
    if (token != nullptr) {
        strncpy(to_state, token, sizeof(to_state) - 1);
    }

    handle_state_change(from_state, to_state);
}

// parse error message
void AP_TetherDrop_Serial::parse_error_message(const char* fields)
{
    char buffer[MSG_BUFFER_SIZE];
    strncpy(buffer, fields, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';
    
    char* checksum_pos = strchr(buffer, '*');
    if (checksum_pos != nullptr) {
        *checksum_pos = '\0';
    }

    // parse error code and description
    char error_code[32] = "";
    char error_desc[64] = "";

    char* token = strtok(buffer, ",");
    if (token != nullptr) {
        strncpy(error_code, token, sizeof(error_code) - 1);
    }

    token = strtok(nullptr, ",");
    if (token != nullptr) {
        strncpy(error_desc, token, sizeof(error_desc) - 1);
    }

    // send error to GCS
    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "TetherDrop: %s - %s", error_code, error_desc);
}

// handle state change events
void AP_TetherDrop_Serial::handle_state_change(const char* from_state, const char* to_state)
{
    // Just log the state change - don't update control mode
    // The winch maintains its own state and we just observe it
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TetherDrop: %s -> %s", from_state, to_state);
}



// send deploy command (starts deployment sequence)
void AP_TetherDrop_Serial::send_deploy_command()
{
    send_command("DEPLOY");
}

// send winch up command
void AP_TetherDrop_Serial::send_winchup_command()
{
    send_command("WINCHUP");
}

// send lock command
void AP_TetherDrop_Serial::send_lock_command()
{
    send_command("LOCK");
}

// deploy to specified depth
void AP_TetherDrop_Serial::deploy(float depth_meters, int32_t bottom_time_ms)
{
    // Set the target depth first
    send_setdepth_command(depth_meters);
    
    // Set the bottom time
    send_setbottomtime_command(bottom_time_ms);
    
    // Then send the deploy command
    send_deploy_command();
}

// winch up from current position
void AP_TetherDrop_Serial::winch_up()
{
    send_winchup_command();
}

// lock the winch
void AP_TetherDrop_Serial::lock()
{
    send_lock_command();
}

// send set depth command
void AP_TetherDrop_Serial::send_setdepth_command(float depth_meters)
{
    int32_t depth_mm = (int32_t)(depth_meters * 1000.0f);
    char params[16];
    hal.util->snprintf(params, sizeof(params), "%ld", (long)depth_mm);
    send_command("SETDEPTH", params);
}

// send set bottom time command
void AP_TetherDrop_Serial::send_setbottomtime_command(int32_t time_ms)
{
    char params[16];
    hal.util->snprintf(params, sizeof(params), "%ld", (long)time_ms);
    send_command("SETBOTTOMTIME", params);
}

// Home - Home the winch (uses motor stall detection)
void AP_TetherDrop_Serial::home()
{
    send_command("HOME");
}

// set bottom time limit (ms, -1 = indefinite)
void AP_TetherDrop_Serial::set_bottom_time(int32_t time_ms)
{
    send_setbottomtime_command(time_ms);
}

// send status request command
void AP_TetherDrop_Serial::send_status_request()
{
    send_command("STATUS");
}

// update user with state changes
void AP_TetherDrop_Serial::update_user()
{
    const uint32_t now_ms = AP_HAL::millis();
    
    // only update if state has changed and at least 1 second has passed
    if (strcmp(status.state, user_update.state) != 0 && 
        (now_ms - user_update.last_ms > 1000)) {
        
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TetherDrop: State=%s Depth=%.1fm", 
                      status.state, (double)status.depth_m);
        
        strncpy(user_update.state, status.state, sizeof(user_update.state) - 1);
        user_update.state[sizeof(user_update.state) - 1] = 0;  // ensure null termination
        user_update.last_ms = now_ms;
    }
}

// send status to ground station
void AP_TetherDrop_Serial::send_status(const GCS_MAVLINK &channel)
{
    // Send WINCH_STATUS message (ID: 9005)
    // Note: We're reusing the WINCH_STATUS message for tether drop
    
    uint32_t time_usec = AP_HAL::micros64();
    
    // Map our states to winch status values
    // 0: HEALTHY, 1: FULLY_RETRACTED, 2: MOVING, 3: CLUTCH_ENGAGED
    uint16_t winch_status = 0;  // default HEALTHY
    
    if (strcmp(status.state, "PAYOUT") == 0) {
        winch_status = 32;  // DROPPING
    } else if (strcmp(status.state, "WINCH_UP") == 0) {
        winch_status = 256;  // RETRACTING
    } else if (strcmp(status.state, "HOME") == 0) {
        winch_status = 4;  // MOVING (homing)
    } else if (strcmp(status.state, "LOCK") == 0) {
        winch_status = 16;  // LOCKED
    } else if (strcmp(status.state, "ON_BOTTOM") == 0) {
        winch_status = 128;  // GROUND_SENSE (at bottom)
    }

    mavlink_msg_winch_status_send(
        channel.get_chan(),
        time_usec,
        status.depth_m,          // line_length
        status.speed_mps,        // speed
        0.0f,                    // tension (not available)
        0.0f,                    // voltage (not available)
        0.0f,                    // current (not available)
        0,                       // temperature (not available)
        winch_status             // status
    );
}

#if HAL_LOGGING_ENABLED
// write log
void AP_TetherDrop_Serial::write_log()
{
    // @LoggerMessage: TTDR
    // @Description: Tether drop status information
    // @Field: TimeUS: Time since system startup
    // @Field: State: Current state of the tether drop system
    // @Field: Depth: Deployed depth
    // @Field: Speed: Deployment/retrieval speed
    // @Field: Pos: Encoder position
    // @Field: RPM: Motor RPM
    // @Field: Healthy: Health status
    AP::logger().Write(
        "TTDR",
        "TimeUS,State,Depth,Speed,Pos,RPM,Healthy",
        "s-mmn--",
        "F------",
        "QNffifB",
        AP_HAL::micros64(),
        status.state,
        (double)status.depth_m,
        (double)status.speed_mps,
        status.position,
        status.rpm,
        (uint8_t)healthy()
    );
}
#endif

#endif  // AP_TETHERDROP_SERIAL_ENABLED
