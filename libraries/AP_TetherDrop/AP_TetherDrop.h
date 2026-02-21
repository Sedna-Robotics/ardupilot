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

#pragma once

#include "AP_TetherDrop_config.h"

#if AP_TETHERDROP_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AP_Logger/AP_Logger_config.h>

class AP_TetherDrop_Backend;

class AP_TetherDrop {
    friend class AP_TetherDrop_Backend;
    friend class AP_TetherDrop_Serial;

public:
    AP_TetherDrop();

    // Do not allow copies
    CLASS_NO_COPY(AP_TetherDrop);

    // indicate whether this module is enabled
    bool enabled() const;

    // true if tether drop is healthy
    bool healthy() const;

    // initialise the tether drop
    void init();

    // update the tether drop
    void update();

    // lock the tether drop (engage servo, stop motor)
    void lock();

    // start payout to specified depth (in meters)
    void deploy_to_depth(float depth);

    // winch up from bottom
    void winch_up();

    // home the winch (uses motor stall detection)
    void home();

    // get maximum configured depth
    float get_max_depth() const { return config.max_depth; }

    // set bottom time limit (in milliseconds, -1 = indefinite)
    void set_bottom_time(int32_t time_ms);

    // send status to ground station
    void send_status(const class GCS_MAVLINK &channel);

#if HAL_LOGGING_ENABLED
    // write log
    void write_log();
#endif

    // returns true if pre arm checks have passed
    bool pre_arm_check(char *failmsg, uint8_t failmsg_len) const;

    static AP_TetherDrop *get_singleton();

    static const struct AP_Param::GroupInfo        var_info[];

private:

    enum class TetherDropType {
        NONE = 0,
        SERIAL = 1
    };

    // enum for OPTIONS parameter
    enum class Options : int16_t {
        VerboseOutput = (1U << 0),  // verbose output of tether drop state sent to GCS
    };

    struct Backend_Config {
        AP_Int8     type;               // tether drop type
        AP_Float    max_depth;          // maximum payout depth (in meters)
        AP_Int16    options;            // options bitmask
        AP_Int32    bottom_time;        // time to wait on bottom (ms, -1 = indefinite)
        float       target_depth;       // target payout depth (in meters)
    } config;

    AP_TetherDrop_Backend *backend;

    static AP_TetherDrop *_singleton;
};

namespace AP {
    AP_TetherDrop *tetherdrop();
};

#endif  // AP_TETHERDROP_ENABLED
