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

/*
   Tether Drop Serial Backend
   
   Implements the machine-to-machine serial protocol for communicating
   with an Arduino-based winch controller over 115200 8N1 serial.
   
   Protocol: $PREFIX,field1,field2,...*CHECKSUM\r\n
*/

#pragma once

#include <AP_TetherDrop/AP_TetherDrop_Backend.h>

#if AP_TETHERDROP_SERIAL_ENABLED

class AP_TetherDrop_Serial : public AP_TetherDrop_Backend {
public:

    using AP_TetherDrop_Backend::AP_TetherDrop_Backend;

    // true if tether drop is healthy
    bool healthy() const override;

    // initialise the tether drop
    void init() override;

    // update - read from serial and send commands
    void update() override;

    // calibrate - set encoder zero at current position
    void calibrate() override;

    // send status to ground station
    void send_status(const GCS_MAVLINK &channel) override;

#if HAL_LOGGING_ENABLED
    // write log
    void write_log() override;
#endif

private:

    // Serial protocol management
    void send_command(const char* cmd, const char* params = nullptr);
    void read_serial();
    void parse_message(const char* msg);
    uint8_t calculate_checksum(const char* msg) const;
    bool verify_checksum(const char* msg) const;

    // Command sending
    void send_payout_command();
    void send_bottom_command();
    void send_winchup_command();
    void send_setdepth_command(float depth_meters);
    void send_status_request();
    void send_winch_status_mavlink();

    // State management
    void update_state_machine();
    void handle_state_change(const char* from_state, const char* to_state);

    // Parsing helpers
    void parse_status_message(const char* fields);
    void parse_ack_message(const char* fields);
    void parse_nak_message(const char* fields);
    void parse_event_message(const char* fields);
    void parse_error_message(const char* fields);

    static const uint8_t SERIAL_BUFFER_SIZE = 128;
    static const uint8_t MSG_BUFFER_SIZE = 128;
    static const uint32_t STATUS_TIMEOUT_MS = 1000;  // consider unhealthy if no status for 1s
    static const uint32_t COMMAND_RETRY_MS = 500;    // retry commands every 500ms

    AP_HAL::UARTDriver *uart;
    char serial_buffer[SERIAL_BUFFER_SIZE];
    uint8_t serial_buffer_len;

    // Winch status from controller
    struct WinchStatus {
        uint32_t last_update_ms;    // last time status was received
        char state[16];             // current state string
        float depth_m;              // depth in meters
        float speed_mps;            // speed in m/s
        int32_t position;           // encoder position
        float rpm;                  // motor RPM
    } status;

    // Last commanded state for tracking
    uint32_t last_command_ms;       // last time we sent a command
    char last_command[16];          // last command sent
    bool waiting_for_ack;           // waiting for acknowledgment

    // update user with state changes via send text messages
    void update_user();
    struct {
        uint32_t last_ms;           // system time of last update to user
        char state[16];             // last reported state
    } user_update;
};

#endif  // AP_TETHERDROP_SERIAL_ENABLED
