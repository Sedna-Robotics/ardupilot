/// @file   AP_BeaconLight.h
/// @brief  Amber warning beacon control: arm-triggered flash plus sunset-to-sunrise auto on/off
#pragma once

#include "AP_BeaconLight_config.h"

#if AP_BEACONLIGHT_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>

// single source of truth for the param group prefix: used both here (for the
// compile-time name-length check below) and in the vehicle's AP_SUBGROUPINFO call
#define AP_BEACONLIGHT_PARAM_PREFIX "BCNL_"

/// @class  AP_BeaconLight
/// @brief  Drives a relay-connected, self-flashing amber warning beacon
class AP_BeaconLight {
public:
    AP_BeaconLight()
    {
        _singleton = this;
        AP_Param::setup_object_defaults(this, var_info);
    }

    CLASS_NO_COPY(AP_BeaconLight);

    // seed the RTC from BCN_BOOT_UTC if no better time source is available yet
    void init();

    // check arming state and sun elevation, drive the relay accordingly; call at ~10Hz
    void update();

    static const struct AP_Param::GroupInfo var_info[];

    static AP_BeaconLight *get_singleton() { return _singleton; }

private:
    static AP_BeaconLight *_singleton;

    // Parameters
    AP_Int8   _enable;      // master enable
    AP_Int8   _relay;       // relay instance the beacon is wired to
    AP_Int8   _arm_enable;  // enable arm-triggered flash
    AP_Int16  _arm_ms;      // duration to hold relay on after arming
    AP_Int8   _sun_enable;  // enable sunset-to-sunrise auto on/off
    AP_Float  _sun_deg;     // sun elevation threshold (degrees) for auto on/off
    AP_Int32  _boot_utc;    // unix time (s) to seed RTC at boot if no better source available

    // internal state
    bool     _armed_prev;
    uint32_t _flash_until_ms;
    uint32_t _last_sun_check_ms;

    // returns true if the sun is below threshold_deg at the given location/time
    bool sun_below_threshold(const Location &loc, uint64_t unix_time_us, float threshold_deg) const;
};

namespace AP {
    AP_BeaconLight *beaconlight();
};

#endif  // AP_BEACONLIGHT_ENABLED
