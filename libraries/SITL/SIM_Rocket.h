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

  Simulates a fin-stabilised model rocket on a launch rail, intended for
  exercising the ArduPlane ROCKET flight mode (Mode::Number::ROCKET) and the
  4-fin mixer in SITL.

  Defaults model a 60mm x 730mm airframe at 1.4kg wet with an RD1-300 class
  motor (~290 Ns, 330 N peak, ~2 s burn).

  Frame string options, appended with '-':
    rocket           nominal, 5 degrees off vertical, + fin layout
    rocket-vertical  exactly vertical (90 deg elevation)
    rocket-tilted    30 degrees off vertical; flies an arc to apogee and
                     transitions, which is correct: an angled trajectory is
                     not a failure. The ABORT path is driven by RKT_ABORT_RATE
                     instead, see Tools/autotest/rocket_cases.sh case 6.
    rocket-nofins    zero fin control authority, exercises loss of control
    rocket-x45       fins rotated 45 deg (X layout), needs RKT_FIN_ANGLE 45

  The fin mounting angle has to be told to the simulator separately from
  RKT_FIN_ANGLE, because SITL models cannot read vehicle parameters. Use
  rocket-x45 together with RKT_FIN_ANGLE 45 (models/rocket-x45.parm does
  both); a mismatch between the two is itself a useful test, since it is
  what a miswired airframe looks like.

  Ignition is commanded from servo output 9 (1-based, so input.servos[8])
  rising above 1700us, because ROCKET mode holds the throttle channel at
  zero throughout. In SITL drive it with "rc 9 2000" from MAVProxy.
*/

#pragma once

#include "SIM_Aircraft.h"

namespace SITL {

/*
  a model rocket simulator
 */
class Rocket : public Aircraft {
public:
    Rocket(const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /*
      The base class resets the attitude to level in set_start_location, which
      runs after the constructor whenever --home is given. A rocket has to be
      re-pointed up the rail afterwards.
    */
    void set_start_location(const Location &start_loc, const float start_yaw) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return NEW_NOTHROW Rocket(frame_str);
    }

private:
    // ---- airframe geometry and mass ----
    const float body_diameter = 0.060f;        // m
    const float body_length   = 0.730f;        // m
    const float mass_dry      = 1.235f;        // kg, without propellant
    const float mass_prop     = 0.165f;        // kg of propellant
    const float ref_area      = M_PI * 0.030f * 0.030f;  // m^2, body cross section

    // CG and CP measured from the nose, along the body axis. CP aft of CG is
    // what makes the airframe statically stable; the difference divided by
    // the body diameter is the static margin in calibers (here ~2.0).
    const float cg_from_nose  = 0.400f;        // m
    const float cp_from_nose  = 0.520f;        // m

    // Moments of inertia. Pitch/yaw treat the airframe as a slender rod about
    // its CG, roll as a thin-walled tube.
    const float inertia_roll  = 8.0e-4f;       // kg m^2
    const float inertia_pitch = 0.0622f;       // kg m^2

    // ---- aerodynamics ----
    const float cd0           = 0.50f;         // subsonic zero-lift drag
    const float cn_alpha      = 12.0f;         // 1/rad, body + fins normal force
    const float cm_q          = -9.0f;         // pitch/yaw damping derivative
    const float cl_p          = -1.2f;         // roll damping derivative

    // Fin control effectiveness. One fin at full deflection, per radian.
    const float fin_area      = 0.0020f;       // m^2 (50mm chord x 40mm span)
    const float fin_cl_alpha  = 2.5f;          // 1/rad, low aspect ratio fin
    const float fin_arm_pitch = 0.300f;        // m, CG to fin centre of pressure
    const float fin_arm_roll  = 0.050f;        // m, body radius + half fin span
    const float fin_max_deflect_rad = radians(45.0f);  // matches SRV_Channel 4500

    // ---- launch rail ----
    const float rail_length   = 1.000f;        // m of guided travel

    // ---- motor ----
    const uint8_t igniter_servo = 8;           // 0-based index of output 9
    float thrust_at(float t) const;            // N, from ignition time
    const float burn_time     = 2.0f;          // s

    // ---- state ----
    bool     ignited = false;
    bool     burnt_out = false;
    bool     left_rail = false;
    uint64_t ignition_us = 0;
    float    impulse_delivered = 0.0f;         // N s, for mass depletion
    Vector3d launch_position;
    float    launch_elevation_deg = 85.0f;     // from horizontal
    bool     fins_disabled = false;
    // Angle of fin 1 from "up", measured about the body axis. 0 is the +
    // layout, 45 the X layout. Must match RKT_FIN_ANGLE on the vehicle.
    float    fin_mount_angle_deg = 0.0f;
    bool     reported_clip = false;
    bool     reported_apogee = false;
    float    peak_altitude = 0.0f;
    float    peak_speed = 0.0f;
    uint64_t last_trace_ms = 0;

    void calculate_forces(const struct sitl_input &input, Vector3f &rot_accel);
    void point_up_the_rail();
};

} // namespace SITL
