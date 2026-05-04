#include "Rover.h"

bool ModeLoiter::_enter()
{
    // set _destination to reasonable stopping point
    if (!g2.wp_nav.get_stopping_location(_destination)) {
        return false;
    }

    // initialise desired speed to current speed
    if (!attitude_control.get_forward_speed(_desired_speed)) {
        _desired_speed = 0.0f;
    }

    // initialise heading: use bearing to destination if outside loiter radius,
    // so the first update() call does not produce a sudden yaw discontinuity
    const float dist_to_dest = rover.current_loc.get_distance(_destination);
    const float loiter_radius = g2.sailboat.tack_enabled() ? g2.sailboat.get_loiter_radius() : g2.loit_radius;
    if (dist_to_dest > loiter_radius) {
        _desired_yaw_cd = rover.current_loc.get_bearing_to(_destination);
    } else {
        _desired_yaw_cd = ahrs.yaw_sensor;
    }

    // initialise drift loiter state
    _active_loit_type = g2.loit_type;
    if (g2.loit_type == 3) {
        g2.wp_nav.init();
        _loiter_center = _destination;
        _drift_state = LoiterDriftState::DRIFTING;
    }

    return true;
}

void ModeLoiter::update()
{
    // detect runtime LOIT_TYPE change and reinitialise accordingly
    if (_active_loit_type != g2.loit_type) {
        if (g2.loit_type == 3) {
            // switching into drift loiter: initialise wp_nav and set anchor to current destination
            g2.wp_nav.init();
            _loiter_center = _destination;
            _drift_state = LoiterDriftState::DRIFTING;
        } else if (_active_loit_type == 3) {
            // switching away from drift loiter: restore _destination to the loiter center
            _destination = _loiter_center;
        }
        _active_loit_type = g2.loit_type;
    }

    // drift loiter (LOIT_TYPE==3) uses its own state-machine update
    if (g2.loit_type == 3) {
        update_drift_loiter();
        return;
    }

    // get distance (in meters) to destination
    _distance_to_destination = rover.current_loc.get_distance(_destination);

    const float loiter_radius = g2.sailboat.tack_enabled() ? g2.sailboat.get_loiter_radius() : g2.loit_radius;

    // if within loiter radius slew desired speed towards zero and use existing desired heading
    if (_distance_to_destination <= loiter_radius) {
        // sailboats should not stop unless motoring
        const float desired_speed_within_radius = g2.sailboat.tack_enabled() ? 0.1f : 0.0f;
        _desired_speed = attitude_control.get_desired_speed_accel_limited(desired_speed_within_radius, rover.G_Dt);

        // if we have a sail but not trying to use it then point into the wind
        if (!g2.sailboat.tack_enabled() && g2.sailboat.sail_enabled()) {
            _desired_yaw_cd = degrees(g2.windvane.get_true_wind_direction_rad()) * 100.0f;
        }
    } else {
        // P controller with hard-coded gain to convert distance to desired speed
        _desired_speed = MIN((_distance_to_destination - loiter_radius) * g2.loiter_speed_gain, g2.wp_nav.get_default_speed());

        // calculate bearing to destination
        _desired_yaw_cd = rover.current_loc.get_bearing_to(_destination);
        float yaw_error_cd = wrap_180_cd(_desired_yaw_cd - ahrs.yaw_sensor);
        // if destination is behind vehicle, reverse towards it
        if ((fabsf(yaw_error_cd) > 9000 && g2.loit_type == 0) || g2.loit_type == 2) {
            _desired_yaw_cd = wrap_180_cd(_desired_yaw_cd + 18000);
            yaw_error_cd = wrap_180_cd(_desired_yaw_cd - ahrs.yaw_sensor);
            _desired_speed = -_desired_speed;
        }

        // reduce desired speed if yaw_error is large
        // 45deg of error reduces speed to 75%, 90deg of error reduces speed to 50%
        float yaw_error_ratio = 1.0f - constrain_float(fabsf(yaw_error_cd / 9000.0f), 0.0f, 1.0f) * 0.5f;
        _desired_speed *= yaw_error_ratio;
    }

    // 0 turn rate is no limit
    float turn_rate = 0.0;

    // make sure sailboats don't try and sail directly into the wind
    if (g2.sailboat.use_indirect_route(_desired_yaw_cd)) {
        _desired_yaw_cd = g2.sailboat.calc_heading(_desired_yaw_cd);
        if (g2.sailboat.tacking()) {
            // use pivot turn rate for tacks
            turn_rate = g2.wp_nav.get_pivot_rate();
        }
    }

    // run steering and throttle controllers
    calc_steering_to_heading(_desired_yaw_cd, turn_rate);
    calc_throttle(_desired_speed, true);
}

// get desired location
bool ModeLoiter::get_desired_location(Location& destination) const
{
    destination = _destination;
    return true;
}

// Drift Loiter state machine (LOIT_TYPE==3)
//   DRIFTING: motors zeroed, boat drifts freely until it exits the loiter radius
//   DRIVING:  use AR_WPNav (same as Auto/Guided) to drive back to the computed
//             target point (loiter center offset toward the opposing direction by
//             LOIT_OVERSHOOT * loit_radius), then switch back to DRIFTING
void ModeLoiter::update_drift_loiter()
{
    const float loiter_radius = g2.sailboat.tack_enabled() ? g2.sailboat.get_loiter_radius() : (float)g2.loit_radius;

    if (_drift_state == LoiterDriftState::DRIFTING) {
        // report distance to the loiter center
        _distance_to_destination = rover.current_loc.get_distance(_loiter_center);

        if (g2.sailboat.tack_enabled()) {
            // sailboats that are motoring/tacking: keep gentle forward progress
            _desired_speed = attitude_control.get_desired_speed_accel_limited(0.1f, rover.G_Dt);
            calc_steering_to_heading(_desired_yaw_cd, g2.wp_nav.get_pivot_rate());
            calc_throttle(_desired_speed, true);
        } else {
            // motorized boats: fully release steering and throttle to drift
            if (!g2.sailboat.tack_enabled() && g2.sailboat.sail_enabled()) {
                // sail-only: point into the wind while drifting
                _desired_yaw_cd = degrees(g2.windvane.get_true_wind_direction_rad()) * 100.0f;
            }
            g2.motors.set_throttle(0.0f);
            g2.motors.set_steering(0.0f, false);
        }

        // check whether the boat has drifted outside the loiter radius
        if (_distance_to_destination > loiter_radius) {
            // compute the bearing from loiter center to current position (the exit direction)
            const float exit_bearing_cd = _loiter_center.get_bearing_to(rover.current_loc);
            // opposite direction: toward the upwind/upcurrent side
            const float return_bearing_cd = wrap_180_cd(exit_bearing_cd + 18000);
            // compute target point: loiter center offset by (overshoot fraction * radius) in the opposing direction
            _destination = _loiter_center;
            _destination.offset_bearing(return_bearing_cd * 0.01f, loiter_radius * constrain_float(g2.loit_overshoot, 0.0f, 1.0f));
            // hand the target to AR_WPNav and cap the drive speed
            if (set_desired_location(_destination)) {
                g2.wp_nav.set_speed_max(MAX(g2.loit_drive_speed, 0.1f));
                _drift_state = LoiterDriftState::DRIVING;
            }
        }

    } else {
        // DRIVING state: use full AR_WPNav navigation (identical to Auto/Guided)
        _distance_to_destination = g2.wp_nav.get_distance_to_destination();

        if (g2.wp_nav.reached_destination()) {
            // arrived – restore destination to loiter center and switch to drifting
            _destination = _loiter_center;
            _drift_state = LoiterDriftState::DRIFTING;
            // release motors immediately so we start drifting on the very next tick
            g2.motors.set_throttle(0.0f);
            g2.motors.set_steering(0.0f, false);
        } else {
            // apply drive speed cap every tick so runtime changes to LOIT_DRIVE_SPD take effect immediately
            g2.wp_nav.set_speed_max(MAX(g2.loit_drive_speed, 0.1f));
            navigate_to_waypoint();
        }
    }
}
