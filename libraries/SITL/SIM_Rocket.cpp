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
  model rocket simulator class
*/

#include "SIM_Rocket.h"

#include <stdio.h>
#include <string.h>

namespace SITL {

Rocket::Rocket(const char *frame_str) :
    Aircraft(frame_str)
{
    mass = mass_dry + mass_prop;

    // A rocket on a rail is held by the rail, not resting on gear, so the
    // generic ground handling is not wanted. The rail is modelled here.
    ground_behavior = GROUND_BEHAVIOR_NONE;

    // Keep the airframe near the ground reference. The rocket is vertical so
    // its footprint as far as the ground check is concerned is small.
    frame_height = 0.1f;

    if (strstr(frame_str, "vertical") != nullptr) {
        launch_elevation_deg = 90.0f;
    } else if (strstr(frame_str, "tilted") != nullptr) {
        launch_elevation_deg = 60.0f;
    }
    if (strstr(frame_str, "nofins") != nullptr) {
        fins_disabled = true;
    }
    if (strstr(frame_str, "x45") != nullptr) {
        fin_mount_angle_deg = 45.0f;
    }

    // Point the nose up the rail. ArduPilot pitch is positive nose-up, and a
    // 90 degree pitch is a singularity for Euler angles, so the nominal frame
    // sits a few degrees off vertical like a real rail would.
    point_up_the_rail();

    launch_position = position;

    ::printf("Rocket: elevation %.1f deg, fins %s at %.0f deg, mass %.3f kg\n",
             (double)launch_elevation_deg,
             fins_disabled ? "DISABLED" : "active",
             (double)fin_mount_angle_deg,
             (double)mass);
    if (fin_mount_angle_deg > 0) {
        ::printf("Rocket: X layout, set RKT_FIN_ANGLE 45 to match\n");
    }
    ::printf("Rocket: arm, set mode ROCKET, then 'servo 9 2000' to ignite\n");
}

/*
  Aim the airframe up the launch rail.
*/
void Rocket::point_up_the_rail()
{
    // 90 degrees exactly is a singularity for Euler angles, so the vertical
    // case is held just short of it.
    const float pitch_rad = radians(MIN(launch_elevation_deg, 89.5f));
    dcm.from_euler(0.0f, pitch_rad, radians(home_yaw));
}

/*
  The base class points the airframe level here, which is wrong for a rocket
  on a rail, so the launch attitude is restored afterwards.
*/
void Rocket::set_start_location(const Location &start_loc, const float start_yaw)
{
    Aircraft::set_start_location(start_loc, start_yaw);
    point_up_the_rail();
    launch_position = position;
}

/*
  RD1-300 class thrust curve, N against seconds from ignition.
  Integrates to 292 Ns, peaking at 330 N over a 2.0 s burn, which matches the
  280-300 Ns / 330 N / ~2 s the airframe is being built around. A solid motor
  of this class is regressive, so mean thrust (146 N) sits well below the peak.
*/
float Rocket::thrust_at(float t) const
{
    if (t < 0 || t > burn_time) {
        return 0.0f;
    }
    static const struct {
        float t;
        float thrust;
    } curve[] = {
        { 0.00f,   0.0f },
        { 0.04f, 300.0f },   // fast pressure rise
        { 0.10f, 330.0f },   // peak
        { 0.30f, 260.0f },
        { 0.60f, 190.0f },
        { 0.90f, 145.0f },
        { 1.25f, 105.0f },
        { 1.60f,  70.0f },
        { 1.85f,  35.0f },
        { 2.00f,   0.0f },   // tail-off
    };
    const uint8_t n = ARRAY_SIZE(curve);
    if (t <= curve[0].t) {
        return curve[0].thrust;
    }
    for (uint8_t i = 1; i < n; i++) {
        if (t <= curve[i].t) {
            const float span = curve[i].t - curve[i-1].t;
            const float frac = (span > 0) ? (t - curve[i-1].t) / span : 0.0f;
            return curve[i-1].thrust + frac * (curve[i].thrust - curve[i-1].thrust);
        }
    }
    return 0.0f;
}

void Rocket::calculate_forces(const struct sitl_input &input, Vector3f &rot_accel)
{
    const float dt = frame_time_us * 1.0e-6f;

    // ---- ignition ----
    if (!ignited && input.servos[igniter_servo] > 1700) {
        ignited = true;
        ignition_us = AP_HAL::micros64();
        ::printf("Rocket: IGNITION\n");
    }

    float thrust = 0.0f;
    if (ignited && !burnt_out) {
        const float t = (AP_HAL::micros64() - ignition_us) * 1.0e-6f;
        thrust = thrust_at(t);
        if (t > burn_time) {
            burnt_out = true;
            ::printf("Rocket: BURNOUT at %.2f s, impulse %.1f Ns\n",
                     (double)t, (double)impulse_delivered);
        }
        // Deplete propellant in proportion to delivered impulse.
        impulse_delivered += thrust * dt;
        const float total_impulse = 292.0f;
        const float burned = constrain_float(impulse_delivered / total_impulse, 0.0f, 1.0f);
        mass = mass_dry + mass_prop * (1.0f - burned);
    }

    // ---- fin deflections ----
    // SERVO1..4_FUNCTION are 190..193 (k_rocket_fin1..4), so the fins land on
    // outputs 1-4, which are servos[0..3] here. Each fin n sits at
    // phi_n = RKT_FIN_ANGLE + 90*(n-1) from "up", and deflecting it produces a
    // force tangential to the body. Applying each fin's contribution from its
    // own geometry (rather than inverting the mixer) means this independently
    // checks the mixer in ArduPlane/servos.cpp.
    float fin[4];
    for (uint8_t i = 0; i < 4; i++) {
        fin[i] = fins_disabled ? 0.0f : filtered_servo_angle(input, i);
    }

    // Fin n sits at phi_n = fin_mount_angle + 90*(n-1) about the body axis.
    // This has to be told to the simulator via the frame string, because a
    // SITL model cannot read RKT_FIN_ANGLE off the vehicle.
    float phi[4];
    for (uint8_t i = 0; i < 4; i++) {
        phi[i] = radians(fin_mount_angle_deg + 90.0f * i);
    }

    const float qbar = 0.5f * air_density * velocity_air_bf.length_squared();
    const float fin_force_per_rad = qbar * fin_area * fin_cl_alpha;

    float roll_moment = 0.0f;
    float pitch_moment = 0.0f;
    float yaw_moment = 0.0f;
    for (uint8_t i = 0; i < 4; i++) {
        const float deflect_rad = fin[i] * fin_max_deflect_rad;
        const float force = fin_force_per_rad * deflect_rad;
        // All four fins contribute equally in roll.
        roll_moment  += force * fin_arm_roll;
        // Pitch and yaw authority depend on where the fin sits around the body.
        pitch_moment += force * fin_arm_pitch * sinf(phi[i]);
        yaw_moment   -= force * fin_arm_pitch * cosf(phi[i]);
    }

    // ---- body aerodynamics ----
    const float airspeed_bf = velocity_air_bf.length();
    float alpha = 0.0f;
    float beta = 0.0f;
    if (airspeed_bf > 1.0f) {
        alpha = atan2f(velocity_air_bf.z, velocity_air_bf.x);
        beta  = atan2f(velocity_air_bf.y, velocity_air_bf.x);
    }

    // Drag rises through the transonic region. This airframe tops out near
    // Mach 0.7 so it stays mostly on the subsonic plateau.
    const float mach = airspeed_bf / 340.0f;
    float cd = cd0;
    if (mach > 0.8f) {
        cd += 0.6f * (mach - 0.8f);
    }
    const float drag = qbar * ref_area * cd;

    // Normal force from angle of attack, opposing the crossflow.
    const float normal_force = qbar * ref_area * cn_alpha;
    const float force_z = -normal_force * alpha;
    const float force_y = -normal_force * beta;

    // CP aft of CG turns that normal force into a restoring moment.
    const float static_arm = cp_from_nose - cg_from_nose;
    pitch_moment += force_z * static_arm;
    yaw_moment   -= force_y * static_arm;

    // Aerodynamic damping, non-dimensionalised on body diameter.
    if (airspeed_bf > 1.0f) {
        const float damp = qbar * ref_area * sq(body_diameter) / (2.0f * airspeed_bf);
        pitch_moment += damp * cm_q * gyro.y;
        yaw_moment   += damp * cm_q * gyro.z;
        roll_moment  += damp * cl_p * gyro.x;
    }

    // ---- assemble ----
    accel_body = Vector3f(thrust - drag, force_y, force_z) / mass;

    rot_accel = Vector3f(roll_moment / inertia_roll,
                         pitch_moment / inertia_pitch,
                         yaw_moment / inertia_pitch);

    // ---- launch rail ----
    // While the rail is still holding the airframe it constrains attitude and
    // keeps motion along the rail axis. Without this the rocket would tip over
    // from the first asymmetry, before it has the airspeed to be stable.
    if (!left_rail) {
        const Vector3d travel = position - launch_position;
        const float distance = travel.length();
        if (distance >= rail_length) {
            left_rail = true;
            ::printf("Rocket: off rail at %.1f m/s\n", (double)velocity_ef.length());
        } else {
            // No rotation while guided.
            rot_accel.zero();
            gyro.zero();
            // Only the along-rail component of thrust is available; the rail
            // reacts everything else.
            const Vector3f rail_dir_ef = dcm * Vector3f(1, 0, 0);
            const float along = accel_body.x;
            if (along <= 0) {
                // Not enough thrust to move yet: the rail holds it still.
                accel_body.zero();
                velocity_ef.zero();
            } else {
                accel_body = Vector3f(along, 0, 0);
                // Project any accumulated velocity back onto the rail.
                const float v_along = velocity_ef * rail_dir_ef;
                velocity_ef = rail_dir_ef * MAX(v_along, 0.0f);
            }
        }
    }

    // Report the peak the IMU would have to survive. The ICM42688/ICM42605 on
    // a MatekH743 saturate at +-16g, so a peak above that is clipped on the
    // real airframe even though the simulator reports it faithfully.
    if (ignited && !reported_clip) {
        const float now_g = accel_body.length() / GRAVITY_MSS;
        if (now_g > 16.0f) {
            reported_clip = true;
            ::printf("Rocket: accel %.1f g exceeds the +-16g IMU range\n", (double)now_g);
        }
    }

    // Trajectory summary from the simulator's own state, so it can be compared
    // against what the EKF reported over MAVLink.
    if (ignited) {
        const float alt = -(float)(position.z - launch_position.z);
        if (alt > peak_altitude) {
            peak_altitude = alt;
        } else if (!reported_apogee && peak_altitude > 5.0f && alt < peak_altitude - 1.0f) {
            reported_apogee = true;
            ::printf("Rocket: apogee %.1f m, peak speed %.1f m/s\n",
                     (double)peak_altitude, (double)peak_speed);
        }
        peak_speed = MAX(peak_speed, velocity_ef.length());

        // Periodic trajectory trace, so a flight that goes wrong can be read
        // without a log. Rate-limited to 10 Hz of simulated time.
        const uint64_t now_ms = AP_HAL::millis64();
        if (now_ms - last_trace_ms >= 100) {
            last_trace_ms = now_ms;
            float r, p, y;
            dcm.to_euler(&r, &p, &y);
            ::printf("TRACE t=%.2f alt=%.1f v=%.1f pitch=%.1f roll=%.1f "
                     "accel_bx=%.1f thrust=%.0f rail=%d\n",
                     (double)((AP_HAL::micros64() - ignition_us) * 1.0e-6f),
                     (double)alt, (double)velocity_ef.length(),
                     (double)degrees(p), (double)degrees(r),
                     (double)accel_body.x, (double)thrust,
                     left_rail ? 0 : 1);
        }
    }
}

/*
  update the rocket simulation by one time step
 */
void Rocket::update(const struct sitl_input &input)
{
    Vector3f rot_accel;

    update_wind(input);

    calculate_forces(input, rot_accel);

    update_dynamics(rot_accel);
    update_external_payload(input);

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();

    update_battery();
}

} // namespace SITL
