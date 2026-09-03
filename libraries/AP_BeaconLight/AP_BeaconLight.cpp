#include "AP_BeaconLight.h"

#if AP_BEACONLIGHT_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_Relay/AP_Relay.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_Math/AP_Math.h>
#include <math.h>

// how often (ms) the sun-elevation check is recomputed; no need to do this every update()
#define AP_BEACONLIGHT_SUN_CHECK_PERIOD_MS 5000

// compile-time guard: full param name (group prefix + field name) must fit within
// AP_MAX_NAME_SIZE, or the param will silently fail to load/save and can corrupt
// storage/boot behaviour. Checked here against the longest field name used below.
namespace {
constexpr size_t ap_beaconlight_constexpr_strlen(const char *s)
{
    return *s ? 1 + ap_beaconlight_constexpr_strlen(s + 1) : 0;
}
constexpr size_t ap_beaconlight_longest_field_name = ap_beaconlight_constexpr_strlen("BOOT_UTC");
static_assert(ap_beaconlight_constexpr_strlen(AP_BEACONLIGHT_PARAM_PREFIX) + ap_beaconlight_longest_field_name <= AP_MAX_NAME_SIZE,
              "AP_BeaconLight: group prefix + field name exceeds AP_MAX_NAME_SIZE");
}

const AP_Param::GroupInfo AP_BeaconLight::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Beacon light enable
    // @Description: Enable the amber warning beacon control module
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 0, AP_BeaconLight, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: RELAY
    // @DisplayName: Beacon light relay instance
    // @Description: Relay instance (0-based) that the beacon light is wired to
    // @Range: 0 5
    // @User: Standard
    AP_GROUPINFO("RELAY", 1, AP_BeaconLight, _relay, 2),

    // @Param: ARM_EN
    // @DisplayName: Beacon light arm-flash enable
    // @Description: Enable flashing the beacon light for BCNL_ARM_MS after the vehicle arms
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ARM_EN", 2, AP_BeaconLight, _arm_enable, 1),

    // @Param: ARM_MS
    // @DisplayName: Beacon light arm-flash duration
    // @Description: Duration the relay is held on after arming, allowing a self-flashing beacon to strobe as an arm indication
    // @Range: 0 30000
    // @Units: ms
    // @User: Standard
    AP_GROUPINFO("ARM_MS", 3, AP_BeaconLight, _arm_ms, 5000),

    // @Param: SUN_EN
    // @DisplayName: Beacon light sunset-to-sunrise enable
    // @Description: Automatically turn the beacon light on at night and off when day returns. Between transitions the relay is left unchanged so manual relay commands are not overridden.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("SUN_EN", 4, AP_BeaconLight, _sun_enable, 1),

    // @Param: ARM_ONLY
    // @DisplayName: Beacon light automatic activation while armed only
    // @Description: Restrict sunset-to-sunrise automatic beacon light activation to when the vehicle is armed
    // @Values: 0:Always,1:Armed only
    // @User: Standard
    AP_GROUPINFO("ARM_ONLY", 7, AP_BeaconLight, _arm_only, 1),

    // @Param: SUN_DEG
    // @DisplayName: Beacon light sun elevation threshold
    // @Description: Beacon light is turned on when the sun's elevation drops below this angle, and off when it rises above it. Default of -6 (civil twilight) gives a small margin around sunset/sunrise per USCG guidance that lights be shown from sunset to sunrise. Use -0.83 for strict sunset/sunrise, or -12 for nautical twilight.
    // @Range: -18 0
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("SUN_DEG", 5, AP_BeaconLight, _sun_deg, -6.0),

    // @Param: BOOT_UTC
    // @DisplayName: Beacon light boot UTC time
    // @Description: Unix time (seconds, UTC) used to seed the RTC at boot if no better time source (e.g. GPS) is available yet. A GPS fix will automatically supersede this once available. 0 to disable.
    // @User: Advanced
    AP_GROUPINFO("BOOT_UTC", 6, AP_BeaconLight, _boot_utc, 0),

    AP_GROUPEND
};

AP_BeaconLight *AP_BeaconLight::_singleton;

void AP_BeaconLight::init()
{
    if (_boot_utc > 0) {
        AP::rtc().set_utc_usec((uint64_t)_boot_utc.get() * 1000000ULL, AP_RTC::SOURCE_HW);
    }
}

void AP_BeaconLight::update()
{
    if (!_enable) {
        return;
    }

    AP_Relay *relay = AP::relay();
    if (relay == nullptr || !relay->enabled(_relay)) {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();

    // detect the rising edge of arming and (re)start the arm-flash window
    const bool armed = AP::arming().is_armed();
    if (armed && !_armed_prev && _arm_enable) {
        _flash_until_ms = now_ms + (uint32_t)_arm_ms;
    }
    _armed_prev = armed;

    // arm-flash takes priority: hold the relay on for the flash window and skip sun logic
    if (_flash_until_ms != 0) {
        if (now_ms < _flash_until_ms) {
            relay->on(_relay);
            return;
        }
        _flash_until_ms = 0;
    }

    if (!_sun_enable) {
        // automatic day/night control disabled: do not command the relay outside the arm-flash
        return;
    }

    if (_arm_only && !armed) {
        return;
    }

    // throttle the sun-elevation calculation; not needed every tick
    if (now_ms - _last_sun_check_ms < AP_BEACONLIGHT_SUN_CHECK_PERIOD_MS && _last_sun_check_ms != 0) {
        return;
    }
    _last_sun_check_ms = now_ms;

    Location loc;
    uint64_t utc_usec;
    if (!AP::ahrs().get_location(loc) || !AP::rtc().get_utc_usec(utc_usec)) {
        // can't determine day/night: leave the relay unchanged rather than overriding a manual command
        return;
    }

    const bool night_active = sun_below_threshold(loc, utc_usec, _sun_deg);
    if (!_night_valid) {
        // first evaluation since boot: apply the automatic state once
        _night_valid = true;
        _night_active = night_active;
        if (night_active) {
            relay->on(_relay);
        } else {
            relay->off(_relay);
        }
        return;
    }

    if (night_active != _night_active) {
        // only command the relay on day/night transitions so manual relay control is not overridden
        _night_active = night_active;
        if (night_active) {
            relay->on(_relay);
        } else {
            relay->off(_relay);
        }
    }
}

// Low-precision solar position (NOAA/Meeus approximation), accurate to well under
// a degree, sufficient for a sunset/twilight on-off threshold.
bool AP_BeaconLight::sun_below_threshold(const Location &loc, uint64_t unix_time_us, float threshold_deg) const
{
    const double unix_s = (double)unix_time_us / 1.0e6;
    const double lat_deg = loc.lat / 1.0e7;
    const double lon_deg = loc.lng / 1.0e7;

    // days since J2000 epoch (2000-01-01 12:00 UTC)
    const double days = (unix_s / 86400.0) - 10957.5;

    const double mean_long = fmod(280.460 + 0.9856474 * days, 360.0);
    const double mean_anom = radians(fmod(357.528 + 0.9856003 * days, 360.0));
    const double eclip_long = radians(mean_long + 1.915 * sin(mean_anom) + 0.020 * sin(2 * mean_anom));
    const double obliq = radians(23.439 - 0.0000004 * days);

    // clamp asin() inputs to [-1,1]; floating-point rounding can nudge them just outside
    // that domain, which would otherwise yield NaN
    const double decl = asin(constrain_double(sin(obliq) * sin(eclip_long), -1.0, 1.0));

    const double y = pow(tan(obliq / 2.0), 2);
    const double mean_long_r = radians(mean_long);
    const double eot = degrees(y * sin(2 * mean_long_r) - 2 * 0.0167 * sin(mean_anom)
                                + 4 * 0.0167 * y * sin(mean_anom) * cos(2 * mean_long_r)
                                - 0.5 * y * y * sin(4 * mean_long_r) - 1.25 * 0.0167 * 0.0167 * sin(2 * mean_anom)) * 4.0;

    const double utc_hours = fmod(unix_s / 3600.0, 24.0);
    const double solar_time = utc_hours + eot / 60.0 + lon_deg / 15.0;
    const double hour_angle = radians((solar_time - 12.0) * 15.0);

    const double lat_r = radians(lat_deg);
    const double elevation = asin(constrain_double(sin(lat_r) * sin(decl) + cos(lat_r) * cos(decl) * cos(hour_angle), -1.0, 1.0));

    return degrees(elevation) < threshold_deg;
}

namespace AP {

AP_BeaconLight *beaconlight()
{
    return AP_BeaconLight::get_singleton();
}

}

#endif  // AP_BEACONLIGHT_ENABLED
