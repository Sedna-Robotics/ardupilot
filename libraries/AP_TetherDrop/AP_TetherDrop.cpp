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

#include "AP_TetherDrop.h"

#if AP_TETHERDROP_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_TetherDrop_Serial.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_TetherDrop::var_info[] = {
    // @Param: _TYPE
    // @DisplayName: Tether Drop Type
    // @Description: Tether Drop Type
    // @User: Standard
    // @Values: 0:None, 1:Serial
    AP_GROUPINFO_FLAGS("_TYPE", 1, AP_TetherDrop, config.type, (int8_t)TetherDropType::NONE, AP_PARAM_FLAG_ENABLE),

    // @Param: _MAX_DEPTH
    // @DisplayName: Tether Drop maximum payout depth
    // @Description: Maximum payout depth in meters. This is the default depth used when no depth is specified.
    // @User: Standard
    // @Range: 1 200
    // @Units: m
    AP_GROUPINFO("_MAX_DEPTH", 3, AP_TetherDrop, config.max_depth, 50.0f),

    // @Param: _OPTIONS
    // @DisplayName: Tether Drop options
    // @Description: Tether Drop options
    // @Bitmask:  0:Verbose output
    // @User: Standard
    AP_GROUPINFO("_OPTIONS", 4, AP_TetherDrop, config.options, 1),

    // @Param: _BOTTOM_TIME
    // @DisplayName: Tether Drop bottom time limit
    // @Description: Time to wait on bottom before automatically winching up. Set to -1 for indefinite (wait for manual WINCHUP command).
    // @User: Standard
    // @Range: -1 300000
    // @Units: ms
    AP_GROUPINFO("_BOTTOM_TIME", 5, AP_TetherDrop, config.bottom_time, 3000),

    AP_GROUPEND
};

AP_TetherDrop::AP_TetherDrop()
{
    if (_singleton) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Too many tether drops");
#endif
        return;
    }
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);
}

// indicate whether this module is enabled
bool AP_TetherDrop::enabled() const
{
   return ((config.type > 0) && (backend != nullptr));
}

// true if tether drop is healthy
bool AP_TetherDrop::healthy() const
{
    if (backend != nullptr) {
        return backend->healthy();
    }
    return false;
}

void AP_TetherDrop::init()
{
    switch ((TetherDropType)config.type.get()) {
    case TetherDropType::NONE:
        break;
#if AP_TETHERDROP_SERIAL_ENABLED
    case TetherDropType::SERIAL:
        backend = NEW_NOTHROW AP_TetherDrop_Serial(config);
        break;
#endif
    default:
        break;
    }
    if (backend != nullptr) {
        backend->init();
        // initialize in locked state
        lock();
    }
}

// start payout to specified depth (in meters)
void AP_TetherDrop::deploy_to_depth(float depth)
{
    if (backend == nullptr) {
        return;
    }
    // constrain depth to valid range
    if (depth < 0.0f) {
        depth = 0.0f;
    }
    if (depth > config.max_depth) {
        depth = config.max_depth;
    }
    config.target_depth = depth;

    // directly call backend to send command
    backend->deploy(depth, config.bottom_time);

    // display verbose output to user
    if ((config.options & uint16_t(Options::VerboseOutput)) != 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TetherDrop: payout to %.1fm for %ld ms", (double)config.target_depth, (long)config.bottom_time);
    }
}

// Home - Home the winch (uses motor stall detection)
void AP_TetherDrop::home()
{
    if (backend == nullptr) {
        return;
    }
    // The backend will send the HOME command to the Arduino controller
    backend->home();

    // display verbose output to user
    if ((config.options & uint16_t(Options::VerboseOutput)) != 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TetherDrop: Homing");
    }
}

// lock the tether drop (engage servo, stop motor)
void AP_TetherDrop::lock()
{
    if (backend == nullptr) {
        return;
    }
    backend->lock();

    // display verbose output to user
    if ((config.options & uint16_t(Options::VerboseOutput)) != 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TetherDrop: Lock");
    }
}

// winch up from bottom
void AP_TetherDrop::winch_up()
{
    if (backend == nullptr) {
        return;
    }
    backend->winch_up();

    // display verbose output to user
    if ((config.options & uint16_t(Options::VerboseOutput)) != 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TetherDrop: Winch up");
    }
}

// set bottom time limit (ms, -1 = indefinite)
void AP_TetherDrop::set_bottom_time(int32_t time_ms)
{
    if (backend == nullptr) {
        return;
    }
    
    // Update parameter
    config.bottom_time.set_and_save(time_ms);
    
    // Send to backend
    backend->set_bottom_time(time_ms);

    // display verbose output to user
    if ((config.options & uint16_t(Options::VerboseOutput)) != 0) {
        if (time_ms == -1) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TetherDrop: Bottom time set to indefinite");
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TetherDrop: Bottom time set to %ld ms", (long)time_ms);
        }
    }
}

// send status to ground station
void AP_TetherDrop::send_status(const GCS_MAVLINK &channel)
{
    if (backend != nullptr) {
        backend->send_status(channel);
    }
}

// returns true if pre arm checks have passed
bool AP_TetherDrop::pre_arm_check(char *failmsg, uint8_t failmsg_len) const
{
    // succeed if tether drop is disabled
    if ((TetherDropType)config.type.get() == TetherDropType::NONE) {
        return true;
    }

    // fail if unhealthy
    if (!healthy()) {
        hal.util->snprintf(failmsg, failmsg_len, "TetherDrop unhealthy");
        return false;
    }

    return true;
}

// update - should be called at at least 10hz
#define PASS_TO_BACKEND(function_name) \
    void AP_TetherDrop::function_name()   \
    {                                  \
        if (!enabled()) {              \
            return;                    \
        }                              \
        if (backend != nullptr) {      \
            backend->function_name();  \
        }                              \
    }

PASS_TO_BACKEND(update)
#if HAL_LOGGING_ENABLED
PASS_TO_BACKEND(write_log)
#endif

#undef PASS_TO_BACKEND

/*
 * Get the AP_TetherDrop singleton
 */
AP_TetherDrop *AP_TetherDrop::_singleton;
AP_TetherDrop *AP_TetherDrop::get_singleton()
{
    return _singleton;
}

namespace AP {

AP_TetherDrop *tetherdrop()
{
    return AP_TetherDrop::get_singleton();
}

};

#endif  // AP_TETHERDROP_ENABLED
