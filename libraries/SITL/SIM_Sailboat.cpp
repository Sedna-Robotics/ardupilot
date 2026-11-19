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
    Sailboat simulator class

    see explanation of lift and drag explained here: https://en.wikipedia.org/wiki/Forces_on_sails

    To-Do: add heel handling by calculating lateral force from wind vs gravity force from heel to arrive at roll rate or acceleration
*/

#include "SIM_Sailboat.h"
#include <AP_Math/AP_Math.h>
#include <string.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

namespace SITL {

#define STEERING_SERVO_CH   0   // steering controlled by servo output 1
#define MAINSAIL_SERVO_CH   3   // main sail controlled by servo output 4
#define THROTTLE_SERVO_CH   2   // throttle controlled by servo output 3
#define MOTORLEFT_SERVO_CH   0   // skid-steering left motor controlled by servo output 1
#define MOTORRIGHT_SERVO_CH   2   // skid-steering right motor controlled by servo output 3
#define DIRECT_WING_SERVO_CH 4

    // very roughly sort of a stability factors for waves
#define WAVE_ANGLE_GAIN 1
#define WAVE_HEAVE_GAIN 1

Sailboat::Sailboat(const char *frame_str) :
    Aircraft(frame_str),
    sail_area(1.0),
    steering_angle_max(35),
    turning_circle(1.8)
{
    motor_connected = (strcmp(frame_str, "sailboat-motor") == 0);
    skid_steering = strstr(frame_str, "skid") != nullptr;
    // "vector": the steering servo turns the motor (e.g. an outboard) rather than a
    // rudder, so yaw comes from vectored thrust and requires forward thrust.  These
    // frames are also forward-thrust only (reverse throttle produces no thrust)
    vectored_thrust = strstr(frame_str, "vector") != nullptr;
    lock_step_scheduled = true;
}

// calculate the lift and drag as values from 0 to 1
// given an apparent wind speed in m/s and angle-of-attack in degrees
void Sailboat::calc_lift_and_drag(float wind_speed, float angle_of_attack_deg, float& lift, float& drag) const
{
    const uint16_t index_width_deg = 10;
    const uint8_t index_max = ARRAY_SIZE(lift_curve) - 1;

    // Convert to expected range
    angle_of_attack_deg = wrap_180(angle_of_attack_deg);

    // assume a symmetrical airfoil
    const float aoa = fabs(angle_of_attack_deg);

    // check extremes
    if (aoa <= 0.0f) {
        lift = lift_curve[0];
        drag = drag_curve[0];
    } else if (aoa >= index_max * index_width_deg) {
        lift = lift_curve[index_max];
        drag = drag_curve[index_max];
    } else {
        uint8_t index = constrain_int16(aoa / index_width_deg, 0, index_max);
        float remainder = aoa - (index * index_width_deg);
        lift = linear_interpolate(lift_curve[index], lift_curve[index+1], remainder, 0.0f, index_width_deg);
        drag = linear_interpolate(drag_curve[index], drag_curve[index+1], remainder, 0.0f, index_width_deg);
    }

    // apply scaling by wind speed
    lift *= wind_speed * sail_area;
    drag *= wind_speed * sail_area;

    if (is_negative(angle_of_attack_deg)) {
        // invert lift for negative aoa
        lift *= -1;
    }
}

// return turning circle (diameter) in meters for steering angle proportion in the range -1 to +1
float Sailboat::get_turn_circle(float steering) const
{
    if (is_zero(steering)) {
        return 0;
    }
    return turning_circle * sinf(radians(steering_angle_max)) / sinf(radians(steering * steering_angle_max));
}

// return yaw rate in deg/sec given a steering input (in the range -1 to +1) and speed in m/s
float Sailboat::get_yaw_rate(float steering, float speed) const
{
    float rate = 0.0f;
    if (is_zero(steering) || (!skid_steering && is_zero(speed))) {
        return rate;
    } 
    
    if (is_zero(speed) && skid_steering) {
        rate = steering * M_PI * 5;
    } else {
        float d = get_turn_circle(steering);
        float c = M_PI * d;
        float t = c / speed;
        rate = 360.0f / t;
    }
    return rate;
}

// return lateral acceleration in m/s/s given a steering input (in the range -1 to +1) and speed in m/s
float Sailboat::get_lat_accel(float steering, float speed) const
{
    float yaw_rate = get_yaw_rate(steering, speed);
    float accel = radians(yaw_rate) * speed;
    return accel;
}

// simulate basic waves / swell
void Sailboat::update_wave(float delta_time)
{
    const float wave_heading = sitl->wave.direction;
    const float wave_speed = sitl->wave.speed;
    const float wave_lenght = sitl->wave.length;
    const float wave_amp = sitl->wave.amp;

    // apply rate propositional to error between boat angle and water angle
    // this gives a 'stability' effect
    float r, p, y;
    dcm.to_euler(&r, &p, &y); 

    // if not armed don't do waves, to allow gyro init
    if (sitl->wave.enable == 0 || !hal.util->get_soft_armed() || is_zero(wave_amp) ) { 
        wave_gyro = Vector3f(-r,-p,0.0f) * WAVE_ANGLE_GAIN;
        wave_heave = -velocity_ef.z * WAVE_HEAVE_GAIN;
        wave_phase = 0.0f;
        return;
    }

    // calculate the sailboat speed in the direction of the wave
    const float boat_speed = velocity_ef.x * sinf(radians(wave_heading)) + velocity_ef.y * cosf(radians(wave_heading));

    // update the wave phase
    const float aprarent_wave_distance = (wave_speed - boat_speed) * delta_time;
    const float apparent_wave_phase_change = (aprarent_wave_distance / wave_lenght) * M_2PI;

    wave_phase += apparent_wave_phase_change;
    wave_phase = wrap_2PI(wave_phase);

    // calculate the angles at this phase on the wave
    // use basic sine wave, dy/dx of sine = cosine
    // atan( cosine ) = wave angle
    const float wave_slope = (wave_amp * 0.5f) * (M_2PI / wave_lenght) * cosf(wave_phase);
    const float wave_angle = atanf(wave_slope);

    // convert wave angle to vehicle frame
    const float heading_dif = wave_heading - y;
    float angle_error_x = (sinf(heading_dif) * wave_angle) - r;
    float angle_error_y = (cosf(heading_dif) * wave_angle) - p;

    // apply gain
    wave_gyro.x = angle_error_x * WAVE_ANGLE_GAIN;
    wave_gyro.y = angle_error_y * WAVE_ANGLE_GAIN;
    wave_gyro.z = 0.0f;

    // calculate wave height (NED)
    if (sitl->wave.enable == 2) {
        wave_heave = (wave_slope - velocity_ef.z) * WAVE_HEAVE_GAIN;
    } else {
        wave_heave = 0.0f;
    }
}

/*
  physical heel, windage and lateral resistance model for a small monohull with a
  bulb keel and a horizontal solar panel (SIM_BOAT_PANEL 1 = Sedna Warden)

  Warden: 2.1m long, 0.3m x 0.3m hull with a 5.9kg (13lb) lead bulb on a keel amidships,
  an 0.3m deep motor pod aft (keel and pod both 11cm chord) and a 1.5m (fore-aft) x 0.8m
  (athwartships) solar panel flush with the deck over the aft hull.  Mass, freeboard
  (0.133m), draft (0.662m to the keel tip, ~0.167m hull) and the righting arm curve are
  from the Orca3D hydrostatics summary v0.8 (no payload): 33.46kg, GMt 0.048m.

  The panel is a low aspect ratio flat plate.  Level, a beam wind passes over it, but
  once the boat heels the wind meets its underside and it lifts: its normal force acts
  near the windward (leading) edge, heeling the boat further, and the normal force
  tilts to leeward, adding to the side force.  The keel's lateral resistance falls with
  cos^2(heel), so past about 30 degrees of heel the boat slips sideways much faster.

  Forces are computed in Newtons and returned in this model's normalised units
  (mass 2, hull drag 0.5*v^2), so Warden's forward resistance is 8.4*v^2 N
 */
// Orca3D righting arm (m) at 0, 5 ... 90 degrees of heel: v0.8 to 45 degrees, v0.7 (which
// agrees to within 2mm there) beyond.  Both use an input VCG of -0.078m, giving GMt 0.048m
static const float warden_gz[] = {0.000f, 0.004f, 0.008f, 0.013f, 0.017f, 0.022f, 0.027f, 0.032f, 0.038f, 0.044f,
                                  0.055f, 0.063f, 0.070f, 0.075f, 0.079f, 0.081f, 0.083f, 0.085f, 0.098f};
static const float warden_gm = 0.048f;

// righting arm in metres for a heel in radians, +ve for +ve heel
static float warden_righting_arm(float heel_rad)
{
    const float heel_deg = MIN(fabsf(degrees(heel_rad)), 89.99f);
    const uint8_t i = heel_deg / 5.0f;
    const float frac = (heel_deg - i * 5.0f) / 5.0f;
    const float gz = warden_gz[i] + (warden_gz[i+1] - warden_gz[i]) * frac;
    return copysignf(gz, heel_rad);
}

Vector2f Sailboat::update_heel(const Vector2f &air_rel_hf, const Vector2f &vel_water_hf, float delta_time)
{
    const float rho_air = 1.225f;
    const float rho_water = 1025.0f;
    const float boat_mass = 33.46f;         // kg
    // SIM_BOAT_PANEL 2: the same boat without its solar panel (mass and VCG unchanged)
    const float panel_area = (sitl->boat_wind.panel.get() == 2) ? 0.0f : 1.5f * 0.8f;   // m^2
    const float panel_width = 0.8f;         // m athwartships
    const float panel_lift_slope = 2.5f;    // per radian, aspect ratio ~1.9
    const float panel_cn_max = 1.25f;
    const float hull_side_area_air = 0.31f; // m^2, 2.1m x 0.133m freeboard plus the panel edge
    const float hull_side_height = 0.066f;  // m, centre of hull side windage above the roll axis
    const float hull_front_area_air = 0.08f;// m^2
    const float hull_side_area_water = 0.275f;  // m^2, 2.06m waterline x 0.167m hull draft
    const float hull_side_depth = 0.08f;        // m
    const float foil_area = 0.495f * 0.11f + 0.3f * 0.11f;  // m^2, keel below the hull and motor pod
    const float foil_depth = 0.38f;         // m, centre of pressure of keel and pod below the roll axis
    const float foil_lift_slope = 4.9f;     // per radian, aspect ratio ~7
    const float roll_inertia = 3.0f;        // kg m^2, mostly the bulb
    const float roll_damping_ratio = 0.3f;

    const float cos_heel = cosf(heel);
    const float sin_heel = sinf(heel);

    // panel: air velocity in the heeled panel's frame, +ve normal component flows up into the underside
    const float air_speed = air_rel_hf.length();
    const float air_y_panel = air_rel_hf.y * cos_heel;
    const float air_normal = air_rel_hf.y * sin_heel;
    float panel_normal_force = 0.0f;
    float panel_roll_moment = 0.0f;
    if (is_positive(air_speed)) {
        const float sin_aoa = constrain_float(air_normal / air_speed, -1.0f, 1.0f);
        const float abs_sin_aoa = fabsf(sin_aoa);
        const float cn = MIN(panel_lift_slope * abs_sin_aoa * safe_sqrt(1.0f - sq(sin_aoa)) + 1.2f * sq(sin_aoa), panel_cn_max);
        panel_normal_force = copysignf(0.5f * rho_air * sq(air_speed) * panel_area * cn, sin_aoa);
        // centre of pressure moves from the quarter chord of the windward edge at small
        // angles of attack to mid chord at 90 degrees.  Only the athwartships flow component moves it sideways
        const float in_plane_speed = norm(air_rel_hf.x, air_y_panel);
        if (is_positive(in_plane_speed)) {
            const float cp_offset = (fabsf(air_y_panel) / in_plane_speed) * panel_width * 0.25f * (1.0f - abs_sin_aoa);
            panel_roll_moment = copysignf(cp_offset, air_y_panel) * panel_normal_force;
        }
    }

    // hull windage
    const float hull_air_x = 0.5f * rho_air * hull_front_area_air * air_rel_hf.x * fabsf(air_rel_hf.x);
    const float hull_air_y = 0.5f * rho_air * hull_side_area_air * cos_heel * air_rel_hf.y * fabsf(air_rel_hf.y);

    // lateral resistance: bluff hull, plus keel and pod as foils at the leeway angle.  Heeling
    // reduces the keel's projected area and turns its lift away from horizontal
    const float u = fabsf(vel_water_hf.x);
    const float v = vel_water_hf.y;
    const float hull_water_y = -0.5f * rho_water * 0.7f * hull_side_area_water * v * fabsf(v);
    const float foil_water_y = -0.5f * rho_water * sq(cos_heel) * foil_area * (foil_lift_slope * u * v + 1.2f * v * fabsf(v));

    // roll: +ve heels to starboard.  Side forces above the roll axis heel the boat away
    // from them, lateral resistance below the roll axis heels it the same way
    const float heel_moment = panel_roll_moment
                            + hull_air_y * hull_side_height * cos_heel
                            - hull_water_y * hull_side_depth
                            - foil_water_y * foil_depth * cos_heel;
    // righting moment from the Orca3D curve.  SIM_BOAT_GM moves the centre of gravity: the
    // Orca3D VCG is an input estimate, and lowering G by dz adds dz*sin(heel) to the righting arm
    const float gm = is_positive(sitl->boat_wind.gm.get()) ? sitl->boat_wind.gm.get() : warden_gm;
    const float righting_arm = warden_righting_arm(heel) + (gm - warden_gm) * sin_heel;
    const float righting_stiffness = boat_mass * GRAVITY_MSS * gm;
    const float roll_damping = 2.0f * roll_damping_ratio * safe_sqrt(righting_stiffness * roll_inertia);
    const float heel_accel = (heel_moment - boat_mass * GRAVITY_MSS * righting_arm - roll_damping * heel_rate) / roll_inertia;
    heel_rate += heel_accel * delta_time;
    heel += heel_rate * delta_time;
    const float heel_max = radians(85.0f);
    if (fabsf(heel) > heel_max) {
        heel = constrain_float(heel, -heel_max, heel_max);
        heel_rate = 0.0f;
    }

    // horizontal force on the boat in Newtons, converted to normalised units
    const Vector2f force_N {hull_air_x, hull_air_y + panel_normal_force * sin_heel + hull_water_y + foil_water_y};
    return force_N * (mass / boat_mass);
}

/*
  update the sailboat simulation by one time step
 */
void Sailboat::update(const struct sitl_input &input)
{
    // update wind
    update_wind(input);

    // in sailboats the steering controls the rudder, the throttle controls the main sail position
    float steering = 0.0f;
    if (skid_steering) {
        const float steering_left = input.servos[MOTORLEFT_SERVO_CH] ? normalise_servo_input(input.servos[MOTORLEFT_SERVO_CH]) : 0;
        const float steering_right = input.servos[MOTORRIGHT_SERVO_CH] ? normalise_servo_input(input.servos[MOTORRIGHT_SERVO_CH]) : 0;
        steering = steering_left - steering_right;
    } else {
        // invalid input (0us) centres the rudder, which is not great
        steering = input.servos[STEERING_SERVO_CH] ? normalise_servo_input(input.servos[STEERING_SERVO_CH]) : 0;
    }

    // calculate apparent wind in earth-frame (this is the direction the wind is coming from)
    // Note than the SITL wind direction is defined as the direction the wind is travelling to
    // This is accounted for in these calculations
    Vector3f wind_apparent_ef = velocity_ef - wind_ef;
    const float wind_apparent_dir_ef = degrees(atan2f(wind_apparent_ef.y, wind_apparent_ef.x));
    const float wind_apparent_speed = safe_sqrt(sq(wind_apparent_ef.x)+sq(wind_apparent_ef.y));

    float roll, pitch, yaw;
    dcm.to_euler(&roll, &pitch, &yaw);

    const float wind_apparent_dir_bf = wrap_180(wind_apparent_dir_ef - degrees(yaw));

    // set RPM and airspeed from wind speed, allows to test RPM and Airspeed wind vane back end in SITL
    rpm[0] = wind_apparent_speed;
    airspeed_pitot = wind_apparent_speed;

    float aoa_deg = 0.0f;
    if (sitl->sail_type.get() == 1) {
        // directly actuated wing
        float wing_angle_bf = constrain_float((input.servos[DIRECT_WING_SERVO_CH]-1500)/500.0f * 90.0f, -90.0f, 90.0f);

        aoa_deg = wind_apparent_dir_bf - wing_angle_bf;

    } else {
        // mainsail with sheet

        // calculate mainsail angle from servo output 4, 0 to 90 degrees
        float mainsail_angle_bf = constrain_float((input.servos[MAINSAIL_SERVO_CH]-1000)/1000.0f * 90.0f, 0.0f, 90.0f);

        // calculate angle-of-attack from wind to mainsail, cannot have negative angle of attack, sheet would go slack
        aoa_deg = MAX(fabsf(wind_apparent_dir_bf) - mainsail_angle_bf, 0);

        if (is_negative(wind_apparent_dir_bf)) {
            // take into account the current tack
            aoa_deg *= -1;
        }

    }

    // calculate Lift force (perpendicular to wind direction) and Drag force (parallel to wind direction)
    float lift_wf, drag_wf;
    calc_lift_and_drag(wind_apparent_speed, aoa_deg, lift_wf, drag_wf);

    // rotate lift and drag from wind frame into body frame
    const float sin_rot_rad = sinf(radians(wind_apparent_dir_bf));
    const float cos_rot_rad = cosf(radians(wind_apparent_dir_bf));
    const float force_fwd = (lift_wf * sin_rot_rad) - (drag_wf * cos_rot_rad);

    // how much time has passed?
    float delta_time = frame_time_us * 1.0e-6f;

    const bool heel_model = vectored_thrust && (sitl->boat_wind.panel.get() > 0);

    // speed in m/s in body frame (heading frame for the heel model, the body may be heeled)
    Vector3f velocity_body;
    if (heel_model) {
        Matrix3f heading_dcm;
        heading_dcm.from_euler(0.0f, 0.0f, yaw);
        velocity_body = heading_dcm.transposed() * velocity_ef_water;
    } else {
        velocity_body = dcm.transposed() * velocity_ef_water;
    }

    // speed along x axis, +ve is forward
    float speed = velocity_body.x;

    // forward-only vectored thrust: thrust direction set by the steering servo
    float vectored_throttle = 0.0f;
    float vectored_angle_rad = 0.0f;
    if (vectored_thrust && motor_connected) {
        vectored_throttle = MAX(input.servos[THROTTLE_SERVO_CH] ? normalise_servo_input(input.servos[THROTTLE_SERVO_CH]) : 0, 0.0f) * MAX(sitl->boat_wind.thrust_scale.get(), 0.0f);
        vectored_angle_rad = radians(steering * vectored_angle_max);
    }

    // yaw rate in degrees/s
    float yaw_rate;
    if (vectored_thrust) {
        // no rudder: the turning moment comes from the side component of the
        // thrust, so turn as a rudder boat would at the water speed this thrust
        // sustains (thrust == hull drag at 10*sqrt(throttle) m/s).  No thrust
        // means no turning, but the boat can turn from rest under thrust
        // with a first order lag for hull yaw inertia
        const float yaw_rate_target = constrain_float(get_yaw_rate(steering, 10.0f * safe_sqrt(vectored_throttle)), -60.0f, 60.0f);
        vectored_yaw_rate += (yaw_rate_target - vectored_yaw_rate) * MIN(delta_time / 0.5f, 1.0f);
        yaw_rate = vectored_yaw_rate;
    } else {
        yaw_rate = get_yaw_rate(steering, speed);
    }

    // windage: force from the air moving relative to the boat, resisted by the hull
    // moving sideways through the water, plus a weathervane yaw moment
    Vector2f wind_force_bf;
    float lateral_drag = 0.0f;
    const float drift_ratio = sitl->boat_wind.drift_ratio;
    if (heel_model) {
        const float cy = cosf(yaw);
        const float sy = sinf(yaw);
        const Vector2f air_rel_ef = (wind_ef - velocity_ef).xy();
        const Vector2f air_rel_hf {air_rel_ef.x * cy + air_rel_ef.y * sy, -air_rel_ef.x * sy + air_rel_ef.y * cy};
        wind_force_bf = update_heel(air_rel_hf, velocity_body.xy(), delta_time);
        const float air_rel_speed = air_rel_hf.length();
        yaw_rate += sitl->boat_wind.weathervane * air_rel_speed * air_rel_hf.y / 100.0f;
        yaw_rate -= sitl->boat_wind.beam * 2.0f * air_rel_hf.x * air_rel_hf.y / 100.0f;
    } else if (is_positive(drift_ratio)) {
        const Vector3f air_rel_bf = dcm.transposed() * (wind_ef - velocity_ef);
        // hull drag is 0.5*v^2 so a stern-to-wind drift of drift_ratio*wind needs 0.5*drift_ratio^2
        const float k_x = 0.5f * sq(drift_ratio);
        const float k_y = 1.5f * k_x;
        wind_force_bf.x = k_x * air_rel_bf.x * fabsf(air_rel_bf.x);
        wind_force_bf.y = k_y * air_rel_bf.y * fabsf(air_rel_bf.y);
        // lateral hull resistance is twice the longitudinal
        const float speed_lateral = velocity_body.y;
        lateral_drag = 2.0f * 0.5f * speed_lateral * fabsf(speed_lateral);
        const float air_rel_speed = safe_sqrt(sq(air_rel_bf.x) + sq(air_rel_bf.y));
        yaw_rate += sitl->boat_wind.weathervane * air_rel_speed * air_rel_bf.y / 100.0f;
        // beam-on tendency: zero yaw moment beam-on and bow/stern to wind, stable beam-on
        yaw_rate -= sitl->boat_wind.beam * 2.0f * air_rel_bf.x * air_rel_bf.y / 100.0f;
    }

    if (heel_model) {
        // heel about the heading frame x axis, waves are not modelled
        const float yaw_new = yaw + radians(yaw_rate) * delta_time;
        gyro = Vector3f(heel_rate, radians(yaw_rate) * sinf(heel), radians(yaw_rate) * cosf(heel));
        dcm.from_euler(heel, 0.0f, yaw_new);
    } else {
        gyro = Vector3f(0,0,radians(yaw_rate)) + wave_gyro;

        // update attitude
        dcm.rotate(gyro * delta_time);
        dcm.normalize();
    }

    // hull drag
    float hull_drag = sq(speed) * 0.5f;
    if (!is_positive(speed)) {
        hull_drag *= -1.0f;
    }

    // throttle force (for motor sailing)
    // gives throttle force == hull drag at 10m/s
    float throttle_force = 0.0f;
    if (motor_connected) {
        if (skid_steering) {
            const float throttle_left = input.servos[MOTORLEFT_SERVO_CH] ? normalise_servo_input(input.servos[MOTORLEFT_SERVO_CH]) : 0;
            const float throttle_right = input.servos[MOTORRIGHT_SERVO_CH] ? normalise_servo_input(input.servos[MOTORRIGHT_SERVO_CH]) : 0;
            throttle_force = (0.5f*(throttle_left + throttle_right)) * 50.0f;
        } else {
            if (vectored_thrust) {
                throttle_force = vectored_throttle * cosf(vectored_angle_rad) * 50.0f;
            } else {
                const float throttle_out = input.servos[THROTTLE_SERVO_CH] ? normalise_servo_input(input.servos[THROTTLE_SERVO_CH]) : 0;
                throttle_force = throttle_out * 50.0f;
            }
        }
    }

    // accel in body frame due acceleration from sail and deceleration from hull friction
    accel_body = Vector3f((throttle_force + force_fwd + wind_force_bf.x) - hull_drag, wind_force_bf.y - lateral_drag, 0);
    accel_body /= mass;

    // add in accel due to direction change
    accel_body.y += radians(yaw_rate) * speed;

    // now in earth frame
    // remove roll and pitch effects from waves
    float r, p, y;
    dcm.to_euler(&r, &p, &y);
    Matrix3f temp_dcm;
    temp_dcm.from_euler(0.0f, 0.0f, y);
    Vector3f accel_earth = temp_dcm * accel_body;

    // we are on the ground, so our vertical accel is zero
    accel_earth.z = 0 + wave_heave;

    // work out acceleration as seen by the accelerometers. It sees the kinematic
    // acceleration (ie. real movement), plus gravity
    accel_body = dcm.transposed() * (accel_earth + Vector3f(0, 0, -GRAVITY_MSS));

    // tide calcs
    Vector3f tide_velocity_ef;
     if (hal.util->get_soft_armed() && !is_zero(sitl->tide.speed) ) {
        tide_velocity_ef.x = -cosf(radians(sitl->tide.direction)) * sitl->tide.speed;
        tide_velocity_ef.y = -sinf(radians(sitl->tide.direction)) * sitl->tide.speed;
        tide_velocity_ef.z = 0.0f;
     }

    // new velocity vector
    velocity_ef_water += accel_earth * delta_time;
    velocity_ef = velocity_ef_water + tide_velocity_ef;

    // new position vector
    position += (velocity_ef * delta_time).todouble();

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();

    // update wave calculations
    update_wave(delta_time);

}

} // namespace SITL
