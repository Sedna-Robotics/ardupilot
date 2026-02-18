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

#include <AP_TetherDrop/AP_TetherDrop.h>

#if AP_TETHERDROP_ENABLED

#include <AP_Logger/AP_Logger_config.h>

class AP_TetherDrop_Backend {
public:
    AP_TetherDrop_Backend(struct AP_TetherDrop::Backend_Config &_config) :
        config(_config) { }

    // true if tether drop is healthy
    virtual bool healthy() const = 0;

    // initialise the backend
    virtual void init() = 0;

    // update - should be called at at least 10hz
    virtual void update() = 0;

    // calibrate - set encoder zero at current position
    virtual void calibrate() = 0;

    // send status to ground station
    virtual void send_status(const GCS_MAVLINK &channel) = 0;

#if HAL_LOGGING_ENABLED
    // write log
    virtual void write_log() = 0;
#endif

protected:

    struct AP_TetherDrop::Backend_Config &config;
};

#endif  // AP_TETHERDROP_ENABLED
