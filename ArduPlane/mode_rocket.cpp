/*
   Rocket flight mode for ArduPlane
   Provides stabilization during rocket boost phase and transition to normal flight modes

   Author: VladimirAndreev1985
   License: GPLv3
*/

#include "mode_rocket.h"
#include "Plane.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>

#if MODE_ROCKET_ENABLED

const AP_Param::GroupInfo ModeRocket::var_info[] = {
    // @Param: ACCEL_THR
    // @DisplayName: Launch acceleration threshold
    // @Description: Minimum forward acceleration to detect launch. Set based on expected motor thrust.
    // @Range: 5 50
    // @Units: m/s/s
    // @User: Standard
    AP_GROUPINFO("ACCEL_THR", 1, ModeRocket, launch_accel_threshold, 15),

    // @Param: ACCEL_TIME
    // @DisplayName: Launch acceleration time
    // @Description: Time acceleration must be sustained to confirm launch
    // @Range: 20 500
    // @Units: ms
    // @User: Standard
    AP_GROUPINFO("ACCEL_TIME", 2, ModeRocket, launch_accel_time_ms, 50),

    // @Param: RAIL_TIME
    // @DisplayName: Rail time
    // @Description: Time to keep control surfaces neutral after launch detection (while on rail)
    // @Range: 0 500
    // @Units: ms
    // @User: Standard
    AP_GROUPINFO("RAIL_TIME", 3, ModeRocket, rail_time_ms, 100),

    // @Param: BST_PTCH_LIM
    // @DisplayName: Boost pitch limit
    // @Description: Maximum pitch deviation from launch angle during boost phase
    // @Range: 1 45
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("BST_PTCH_LIM", 4, ModeRocket, boost_pitch_limit, 15),

    // @Param: BST_ROLL_LIM
    // @DisplayName: Boost roll limit
    // @Description: Maximum roll deviation during boost phase
    // @Range: 1 45
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("BST_ROLL_LIM", 5, ModeRocket, boost_roll_limit, 10),

    // @Param: BST_YAW_LIM
    // @DisplayName: Boost yaw limit
    // @Description: Maximum yaw deviation from launch heading during boost
    // @Range: 1 45
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("BST_YAW_LIM", 6, ModeRocket, boost_yaw_limit, 15),

    // @Param: BST_TIME_MAX
    // @DisplayName: Maximum boost time
    // @Description: Maximum expected motor burn time
    // @Range: 0.5 10
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("BST_TIME_MAX", 7, ModeRocket, boost_time_max, 3.0),

    // @Param: CST_TIME_MAX
    // @DisplayName: Maximum coast time
    // @Description: Maximum coast time before transition
    // @Range: 1 60
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("CST_TIME_MAX", 8, ModeRocket, coast_time_max, 15.0),

    // @Param: TRANS_MODE
    // @DisplayName: Transition flight mode
    // @Description: Mode after coast. 0=MANUAL,5=FBWA,7=CRUISE,10=AUTO,11=RTL,12=LOITER
    // @Range: 0 26
    // @User: Standard
    AP_GROUPINFO("TRANS_MODE", 9, ModeRocket, transition_mode, 5),

    // @Param: ABORT_LIM
    // @DisplayName: Abort attitude limit
    // @Description: Attitude deviation triggering abort
    // @Range: 20 90
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("ABORT_LIM", 10, ModeRocket, abort_attitude_limit, 60),

    // @Param: FIN_ANGLE
    // @DisplayName: Fin mounting angle
    // @Description: 0 for cross (+), 45 for X configuration
    // @Range: 0 45
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("FIN_ANGLE", 11, ModeRocket, fin_angle_offset, 0),

    // @Param: SPD_SCL_MIN
    // @DisplayName: Speed scaler minimum
    // @Description: Min speed scaler value
    // @Range: 0.1 1.0
    // @User: Advanced
    AP_GROUPINFO("SPD_SCL_MIN", 12, ModeRocket, speed_scaler_min, 0.2),

    // @Param: SPD_SCL_MAX
    // @DisplayName: Speed scaler maximum
    // @Description: Max speed scaler value
    // @Range: 1.0 5.0
    // @User: Advanced
    AP_GROUPINFO("SPD_SCL_MAX", 13, ModeRocket, speed_scaler_max, 2.0),

    // @Param: TRANS_SPD
    // @DisplayName: Transition speed
    // @Description: Speed below which to transition, as a secondary test after RKT_TRANS_VZ. Defaults to 0 (disabled), because speed magnitude is the wrong apogee signal: on a tilted trajectory horizontal speed survives apogee, and dead reckoning underestimates speed badly when the boost exceeds the IMU range (a 24 g launch on a +-16g IMU clips, so integrated speed comes out low and this test can fire while the rocket is still climbing). Use RKT_TRANS_VZ instead. 0 disables.
    // @Range: 0 100
    // @Units: m/s
    // @User: Advanced
    AP_GROUPINFO("TRANS_SPD", 14, ModeRocket, transition_speed, 0),

    // @Param: P_KP
    // @DisplayName: Pitch P gain
    // @Description: Proportional gain for pitch stabilization
    // @Range: 0.1 3.0
    // @User: Advanced
    AP_GROUPINFO("P_KP", 15, ModeRocket, kp_pitch, 0.8),

    // @Param: P_KD
    // @DisplayName: Pitch D gain
    // @Description: Derivative gain for pitch damping
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("P_KD", 16, ModeRocket, kd_pitch, 0.3),

    // @Param: R_KP
    // @DisplayName: Roll P gain
    // @Description: Proportional gain for roll stabilization. A slender rocket has very little roll inertia (around 8e-4 kg m^2 for a 60mm/730mm airframe) against considerable fin roll authority, so this needs to be far smaller than the pitch and yaw gains. SITL diverged to a full 180 degree roll at 0.05 and held inside 3 degrees at 0.01.
    // @Range: 0.0 3.0
    // @User: Advanced
    AP_GROUPINFO("R_KP", 17, ModeRocket, kp_roll, 0.01),

    // @Param: R_KD
    // @DisplayName: Roll D gain
    // @Description: Derivative gain for roll damping
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("R_KD", 18, ModeRocket, kd_roll, 0.05),

    // @Param: Y_KP
    // @DisplayName: Yaw P gain
    // @Description: Proportional gain for yaw stabilization
    // @Range: 0.1 3.0
    // @User: Advanced
    AP_GROUPINFO("Y_KP", 19, ModeRocket, kp_yaw, 0.5),

    // @Param: Y_KD
    // @DisplayName: Yaw D gain
    // @Description: Derivative gain for yaw damping
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("Y_KD", 20, ModeRocket, kd_yaw, 0.2),

    // @Param: LAUNCH_ANG
    // @DisplayName: Launch angle
    // @Description: Initial launch angle from vertical. 0=vertical, 90=horizontal. -1 uses current pitch at arm.
    // @Range: -1 90
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("LAUNCH_ANG", 21, ModeRocket, launch_angle, -1),

    // @Param: SPD_SCL_REF
    // @DisplayName: Speed scaler reference speed
    // @Description: Airspeed at which the fin gains are nominal (scaler == 1). The scaler is SPD_SCL_REF/speed, clamped to SPD_SCL_MIN..SPD_SCL_MAX, so fin deflection is reduced as speed rises above this. Set this to the speed the RKT P/D gains were tuned at.
    // @Range: 5 100
    // @Units: m/s
    // @User: Advanced
    AP_GROUPINFO("SPD_SCL_REF", 22, ModeRocket, speed_scaler_ref, 30),

    // @Param: DRAG_K
    // @DisplayName: Velocity integrator drag coefficient
    // @Description: Drag term used by the inertial velocity integrator, as acceleration = DRAG_K * v^2. This only matters when there is no GPS or airspeed, because the integrator is then the only speed source the mode has. Compute it as 0.5 * rho * Cd * A / mass: for a 60mm airframe (A = 0.00283 m^2) at 1.3 kg with Cd 0.5 that is about 0.00067. Too large a value makes the mode underestimate speed, which inflates the fin speed scaler at peak dynamic pressure.
    // @Range: 0.0 0.01
    // @User: Advanced
    // @Param: DRAG_K
    // @DisplayName: Velocity integrator drag coefficient (legacy, unused)
    // @Description: No longer used. The velocity estimate now integrates earth-frame acceleration from the IMU, which already contains drag, so modelling drag separately would count it twice. Kept so existing parameter files still load.
    // @Range: 0.0 0.01
    // @User: Advanced
    AP_GROUPINFO("DRAG_K", 23, ModeRocket, drag_k, 0.00067),

    // @Param: TRANS_VZ
    // @DisplayName: Transition climb rate
    // @Description: Climb rate below which COAST ends and the mode hands over. This is the primary apogee test and it needs no GPS: the climb rate comes from the EKF when available and from the barometer otherwise. The old RKT_TRANS_SPD test used speed magnitude, which never falls on a tilted trajectory because horizontal speed survives apogee, so the rocket used to sit in COAST until the timeout. Set 0 to disable.
    // @Range: 0 50
    // @Units: m/s
    // @User: Standard
    AP_GROUPINFO("TRANS_VZ", 24, ModeRocket, transition_vz, 5),

    // @Param: ABORT_RATE
    // @DisplayName: Abort body rate limit
    // @Description: Body rotation rate above which the mode aborts during COAST. Attitude deviation cannot be used for the abort test in COAST, because an unguided rocket arcs over on a ballistic path and pitch legitimately passes RKT_ABORT_LIM near apogee. Tumbling shows up as rotation rate instead. Set 0 to disable the COAST abort test.
    // @Range: 0 1000
    // @Units: deg/s
    // @User: Standard
    AP_GROUPINFO("ABORT_RATE", 25, ModeRocket, abort_rate_limit, 360),

    // @Param: GPS_PROT
    // @DisplayName: GPS handover protection
    // @Description: When enabled, the mode checks GPS quality and refuses to hand over to a mode that needs position (AUTO, RTL, LOITER, GUIDED, CIRCLE) if the fix is lost or looks spoofed, using RKT_GPS_FBK instead. Flight in BOOST and COAST never uses GPS, so this does not affect stabilisation.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("GPS_PROT", 26, ModeRocket, gps_protect, 1),

    // @Param: GPS_FBK
    // @DisplayName: GPS-denied fallback mode
    // @Description: Mode to hand over to when RKT_GPS_PROT is enabled and GPS is lost or spoofed. Must be a mode that flies without position: 0=MANUAL, 2=STABILIZE, 5=FBWA. Do not set a position-dependent mode here.
    // @Range: 0 26
    // @User: Standard
    AP_GROUPINFO("GPS_FBK", 27, ModeRocket, gps_fallback_mode, 5),

    // @Param: GPS_LOST_MS
    // @DisplayName: GPS loss timeout
    // @Description: Time without a 3D fix after which GPS is treated as lost. Before the first fix the timer runs from mode entry.
    // @Range: 500 10000
    // @Units: ms
    // @User: Advanced
    AP_GROUPINFO("GPS_LOST_MS", 28, ModeRocket, gps_lost_ms, 2000),

    // @Param: GPS_JUMP_M
    // @DisplayName: GPS position jump limit
    // @Description: Position jump between consecutive GPS solutions that marks the fix as spoofed, when the inertial estimate does not support a move that large. Set 0 to disable the check.
    // @Range: 0 1000
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("GPS_JUMP_M", 29, ModeRocket, gps_jump_m, 150),

    // @Param: GPS_MIN_SAT
    // @DisplayName: Minimum satellite count
    // @Description: Satellite count below which the fix is treated as degraded. Degraded still counts as usable position; only LOST and SPOOFED trigger the fallback mode.
    // @Range: 4 20
    // @User: Advanced
    AP_GROUPINFO("GPS_MIN_SAT", 30, ModeRocket, gps_min_sat, 6),

    // @Param: GPS_HDOP
    // @DisplayName: Maximum HDOP for a healthy fix
    // @Description: HDOP above which the fix is treated as degraded. HDOP is the geometric uncertainty of the horizontal solution, so a receiver under partial jamming can keep reporting enough satellites while the solution itself decays. Satellite count alone does not catch that. Set 0 to check satellite count only.
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("GPS_HDOP", 31, ModeRocket, gps_max_hdop, 2.5),

    // @Param: BURN_TIME
    // @DisplayName: Burnout confirmation time
    // @Description: Time the forward acceleration must stay below the burnout threshold before BOOST ends. A single accelerometer dropout, or the vibration notch of a motor running rough, would otherwise end BOOST early and fly the rest of the burn on COAST settings. Mirrors RKT_ACCEL_TIME on the launch side. Set 0 to react on the first sample.
    // @Range: 0 500
    // @Units: ms
    // @User: Advanced
    AP_GROUPINFO("BURN_TIME", 32, ModeRocket, burnout_confirm_ms, 100),

    AP_GROUPEND
};

ModeRocket::ModeRocket() :
    Mode(),
    current_phase(Phase::PAD),
    phase_start_ms(0),
    launch_detect_start_ms(0),
    burnout_candidate_ms(0),
    launch_accel_count(0),
    dr_velocity_ned(0, 0, 0),
    dr_velocity_valid(false),
    inertial_velocity_mps(0),
    last_velocity_update_ms(0),
    last_gps_health(GPSHealth::LOST),
    gps_last_fix_ms(0),
    gps_monitor_start_ms(0),
    gps_prev_loc_valid(false),
    launch_pitch_cd(0),
    launch_yaw_cd(0),
    handover_attempt_ms(0)
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool ModeRocket::_enter()
{
    current_phase = Phase::PAD;
    phase_start_ms = AP_HAL::millis();
    launch_detect_start_ms = 0;
    burnout_candidate_ms = 0;
    launch_accel_count = 0;
    dr_velocity_ned.zero();
    // The rocket is stationary on the pad, so zero velocity is a known-good
    // initial condition for dead reckoning. No GPS is needed to establish it.
    dr_velocity_valid = true;
    inertial_velocity_mps = 0;
    last_velocity_update_ms = AP_HAL::millis();
    launch_pitch_cd = ahrs.pitch_sensor;
    launch_yaw_cd = ahrs.yaw_sensor;
    handover_attempt_ms = 0;
    reset_gps_monitor();

    gcs().send_text(MAV_SEVERITY_INFO, "Rocket: PAD - waiting for launch");
    return true;
}

void ModeRocket::_exit()
{
    gcs().send_text(MAV_SEVERITY_INFO, "Rocket: exit, phase was %s", get_phase_name());
}

bool ModeRocket::_pre_arm_checks(size_t buflen, char *buffer) const
{
    if (fin_angle_offset < 0 || fin_angle_offset > 45) {
        hal.util->snprintf(buffer, buflen, "RKT_FIN_ANGLE must be 0-45");
        return false;
    }
    if (transition_mode < 0 || transition_mode > 26) {
        hal.util->snprintf(buffer, buflen, "RKT_TRANS_MODE invalid");
        return false;
    }
    // A fallback that itself needs position would defeat the purpose, so it
    // is rejected here rather than at the moment of handover in flight.
    switch ((Mode::Number)gps_fallback_mode.get()) {
        case Mode::Number::MANUAL:
        case Mode::Number::STABILIZE:
        case Mode::Number::FLY_BY_WIRE_A:
        case Mode::Number::ACRO:
        case Mode::Number::TRAINING:
            break;
        default:
            hal.util->snprintf(buffer, buflen, "RKT_GPS_FBK needs a no-GPS mode");
            return false;
    }
    return true;
}

const char* ModeRocket::get_phase_name() const
{
    switch (current_phase) {
        case Phase::PAD: return "PAD";
        case Phase::RAIL: return "RAIL";
        case Phase::BOOST: return "BOOST";
        case Phase::COAST: return "COAST";
        case Phase::TRANSITION: return "TRANS";
        case Phase::ABORT: return "ABORT";
        default: return "UNK";
    }
}

bool ModeRocket::is_taking_off() const
{
    return current_phase == Phase::RAIL || current_phase == Phase::BOOST;
}

void ModeRocket::update()
{
    update_inertial_velocity();
    update_gps_health();
    check_phase_transition();
    if (should_abort() && current_phase != Phase::ABORT) {
        do_abort();
    }
}

void ModeRocket::run()
{
    // ROCKET drives the fins itself and never calls the fixed-wing rate
    // controllers, so their state has to be held reset every loop. Without
    // this they see a gap of thousands of loops and the handover to any
    // stabilised mode trips "FW rate control must be run or reset every
    // loop": AP_HAL::panic in SITL, and a stale integrator plus a bad TECS
    // state on real hardware. ModeManual::run does the same thing for the
    // same reason. Handover to MANUAL worked before this and handover to
    // FBWA or STABILIZE wedged the vehicle at apogee.
    reset_controllers();

    switch (current_phase) {
        case Phase::PAD:
        case Phase::RAIL:
        case Phase::ABORT:
            SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, 0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, 0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, 0);
            break;
        case Phase::BOOST:
        case Phase::COAST:
        case Phase::TRANSITION:
            stabilize_boost();
            break;
    }
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);
}

/*
  Forward (body x) acceleration with the gravity component removed.

  ahrs.get_accel() is the specific force measured by the primary IMU, so a
  rocket sitting nose-up on the pad reads about +g on its body x axis.
  Subtracting the projection of gravity on that axis leaves the true forward
  acceleration, which is what both the launch threshold and the velocity
  integration need, and makes the threshold independent of launch angle.
*/
float ModeRocket::get_forward_accel() const
{
    const Vector3f accel_body = ahrs.get_accel() - ahrs.get_accel_bias();
    const float gravity_comp = GRAVITY_MSS * sinf(ahrs.get_pitch_rad());
    return accel_body.x - gravity_comp;
}

bool ModeRocket::detect_launch()
{
    const uint32_t now = AP_HAL::millis();
    const float forward_accel = get_forward_accel();
    
    if (forward_accel >= launch_accel_threshold) {
        if (launch_detect_start_ms == 0) {
            launch_detect_start_ms = now;
        }
        if ((now - launch_detect_start_ms) >= (uint32_t)launch_accel_time_ms) {
            gcs().send_text(MAV_SEVERITY_INFO, "Rocket: Launch! accel=%.1f", (double)forward_accel);
            return true;
        }
    } else {
        launch_detect_start_ms = 0;
    }
    return false;
}

/*
  Dead-reckoned velocity in the NED frame.

  This is what makes the mode usable with GPS jammed, spoofed or simply not
  fitted. ahrs.get_accel_ef() is the specific force rotated into the earth
  frame, so adding GRAVITY_MSS to its Down component cancels the gravity
  reaction and leaves true kinematic acceleration: zero for a vehicle at
  rest, +g down in free fall. Integrating that gives velocity without any
  position source.

  Drag is not modelled here. The accelerometer already measures it, so the
  old RKT_DRAG_K term counted drag twice and biased the estimate low.

  When the EKF does have a velocity solution the dead-reckoned state is
  pulled towards it, so bias does not accumulate across a long wait on the
  pad or after GPS comes back. With no solution the integration simply runs
  open-loop and drifts, which is accepted.
*/
void ModeRocket::update_inertial_velocity()
{
    const uint32_t now = AP_HAL::millis();
    if (last_velocity_update_ms == 0) {
        last_velocity_update_ms = now;
        return;
    }
    const float dt = (now - last_velocity_update_ms) * 0.001f;
    last_velocity_update_ms = now;
    if (!is_positive(dt) || dt > 0.1f) {
        return;
    }

    // Still on the pad: the rocket cannot be moving, so hold the integrator
    // at zero rather than letting accelerometer bias walk it away during an
    // open-ended wait for launch.
    if (current_phase == Phase::PAD) {
        dr_velocity_ned.zero();
        dr_velocity_valid = true;
        inertial_velocity_mps = 0;
        return;
    }

    Vector3f accel_ef = ahrs.get_accel_ef();
    accel_ef.z += GRAVITY_MSS;
    dr_velocity_ned += accel_ef * dt;

    // Correct against the EKF solution when there is one. A first-time fix
    // snaps the state; after that a light complementary blend keeps the
    // estimate honest without letting a GPS glitch step the velocity.
    Vector3f vel_ned;
    if (ahrs.get_velocity_NED(vel_ned)) {
        if (!dr_velocity_valid) {
            dr_velocity_ned = vel_ned;
        } else {
            const float tau = 2.0f;
            const float alpha = constrain_float(dt / tau, 0.0f, 1.0f);
            dr_velocity_ned += (vel_ned - dr_velocity_ned) * alpha;
        }
        dr_velocity_valid = true;
    }

    dr_velocity_ned.x = constrain_float(dr_velocity_ned.x, -1000, 1000);
    dr_velocity_ned.y = constrain_float(dr_velocity_ned.y, -1000, 1000);
    dr_velocity_ned.z = constrain_float(dr_velocity_ned.z, -1000, 1000);
    inertial_velocity_mps = constrain_float(dr_velocity_ned.length(), 0, 1000);
}

/*
  Climb rate, m/s, positive up, from the best source available.

  Order matters. The EKF velocity is preferred, then the kinematically
  consistent vertical position rate, then the barometer, and dead reckoning
  last. The barometer entry is the important one: it needs no GPS, so apogee
  detection keeps working when the only position source is gone.
*/
float ModeRocket::get_climb_rate()
{
    Vector3f vel_ned;
    if (ahrs.get_velocity_NED(vel_ned)) {
        return -vel_ned.z;
    }
    float vel_d = 0.0f;
    if (ahrs.get_vert_pos_rate_D(vel_d)) {
        return -vel_d;
    }
    if (AP::baro().healthy()) {
        return AP::baro().get_climb_rate();
    }
    if (dr_velocity_valid) {
        return -dr_velocity_ned.z;
    }
    return 0.0f;
}

float ModeRocket::get_speed_scaler() const
{
    // inertial_velocity_mps already carries the EKF solution when there is
    // one and the dead-reckoned estimate when there is not, so the scaler
    // behaves the same with and without GPS.
    const float velocity = inertial_velocity_mps;
    if (velocity < 1.0f) return speed_scaler_max;
    // RKT_SPD_SCL_REF is the speed at which the gains are nominal. Guard
    // against a zero/negative setting, which would null out the fins.
    const float ref = MAX(speed_scaler_ref, 1.0f);
    float scaler = ref / velocity;
    return constrain_float(scaler, speed_scaler_min, speed_scaler_max);
}

void ModeRocket::stabilize_boost()
{
    float pitch_err = constrain_float(launch_pitch_cd - ahrs.pitch_sensor, -boost_pitch_limit*100, boost_pitch_limit*100);
    float roll_err = constrain_float(-ahrs.roll_sensor, -boost_roll_limit*100, boost_roll_limit*100);
    float yaw_err = constrain_float(wrap_180_cd(launch_yaw_cd - ahrs.yaw_sensor), -boost_yaw_limit*100, boost_yaw_limit*100);
    
    Vector3f gyro = ahrs.get_gyro();
    
    float p_out = kp_pitch * pitch_err * 0.01f - kd_pitch * gyro.y * RAD_TO_DEG;
    float r_out = kp_roll * roll_err * 0.01f - kd_roll * gyro.x * RAD_TO_DEG;
    float y_out = kp_yaw * yaw_err * 0.01f - kd_yaw * gyro.z * RAD_TO_DEG;
    
    float scaler = get_speed_scaler();
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, constrain_float(p_out*scaler*45, -4500, 4500));
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, constrain_float(r_out*scaler*45, -4500, 4500));
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, constrain_float(y_out*scaler*45, -4500, 4500));
}

void ModeRocket::stabilize_coast() { stabilize_boost(); }

void ModeRocket::check_phase_transition()
{
    const uint32_t now = AP_HAL::millis();
    const uint32_t pt = now - phase_start_ms;
    
    switch (current_phase) {
        case Phase::PAD:
            if (detect_launch()) {
                launch_pitch_cd = ahrs.pitch_sensor;
                launch_yaw_cd = ahrs.yaw_sensor;
                enter_phase(Phase::RAIL);
            }
            break;
        case Phase::RAIL:
            if (pt >= (uint32_t)rail_time_ms) enter_phase(Phase::BOOST);
            break;
        case Phase::BOOST: {
            if (pt >= (uint32_t)(boost_time_max * 1000)) {
                enter_phase(Phase::COAST);
                break;
            }
            // Burnout by acceleration, confirmed over RKT_BURN_TIME. The
            // 500 ms guard keeps the ramp-up at ignition from reading as a
            // burnout before the motor reaches full thrust.
            if (pt > 500 && get_forward_accel() < launch_accel_threshold * 0.3f) {
                if (burnout_candidate_ms == 0) {
                    burnout_candidate_ms = now;
                }
                if (now - burnout_candidate_ms >= (uint32_t)burnout_confirm_ms) {
                    enter_phase(Phase::COAST);
                }
            } else {
                burnout_candidate_ms = 0;
            }
            break;
        }
        case Phase::COAST: {
            if (pt >= (uint32_t)(coast_time_max * 1000)) {
                enter_phase(Phase::TRANSITION);
                break;
            }
            if (pt <= 1000) {
                break;
            }
            // Apogee test. Climb rate is the right signal here and it needs
            // no GPS: near apogee the vertical component goes through zero
            // whatever the trajectory. Speed magnitude does not work on a
            // tilted flight, because horizontal speed survives apogee and
            // keeps the total above RKT_TRANS_SPD until the timeout.
            if (is_positive(transition_vz) && get_climb_rate() < transition_vz) {
                enter_phase(Phase::TRANSITION);
                break;
            }
            // Secondary test on total speed. Retained for compatibility and
            // for airframes that want to hand over well after apogee; the
            // climb rate test above normally fires first. Set 0 to disable.
            if (is_positive(transition_speed) && inertial_velocity_mps < transition_speed) {
                enter_phase(Phase::TRANSITION);
            }
            break;
        }
        case Phase::TRANSITION: transition_to_next_mode(); break;
        case Phase::ABORT: break;
    }
}

void ModeRocket::enter_phase(Phase p)
{
    current_phase = p;
    phase_start_ms = AP_HAL::millis();
    // Each phase starts its own burnout confirmation window.
    burnout_candidate_ms = 0;
    const float climb = get_climb_rate();
    gcs().send_text(MAV_SEVERITY_INFO, "Rocket: %s v=%.1f vz=%.1f",
                    get_phase_name(), (double)inertial_velocity_mps, (double)climb);
    // VZ and GPSH are logged because they are what the phase transitions and
    // the handover decision now key off, so a flight log has to show them.
    AP::logger().Write("RCKT", "TimeUS,Phase,Vel,VZ,Pitch,Roll,Yaw,GPSH", "s#nnddd-", "F-------", "QBfffffB",
        AP_HAL::micros64(), (uint8_t)p, inertial_velocity_mps, climb,
        ahrs.pitch_sensor*0.01f, ahrs.roll_sensor*0.01f, ahrs.yaw_sensor*0.01f,
        (uint8_t)last_gps_health);
}

/*
  Abort test.

  The attitude test only applies while the motor is burning. In RAIL and
  BOOST the rocket is meant to hold its launch attitude, so a large
  deviation really does mean loss of control.

  In COAST it does not. An unguided rocket follows a ballistic arc: it
  pitches over through 90 degrees and beyond on the way down, which passes
  RKT_ABORT_LIM on every normal flight. Treating that as a failure fired a
  false ABORT, and with no GPS it fired on every flight, because the old
  speed test could not end COAST before the arc-over. Tumbling is what
  actually needs catching after burnout, and that shows up as body rate.
*/
bool ModeRocket::should_abort() const
{
    if (current_phase == Phase::PAD ||
        current_phase == Phase::TRANSITION ||
        current_phase == Phase::ABORT) {
        return false;
    }

    if (current_phase == Phase::RAIL || current_phase == Phase::BOOST) {
        const float pe = fabsf(wrap_180_cd(launch_pitch_cd - ahrs.pitch_sensor)) * 0.01f;
        const float re = fabsf(ahrs.roll_sensor) * 0.01f;
        const float ye = fabsf(wrap_180_cd(launch_yaw_cd - ahrs.yaw_sensor)) * 0.01f;
        return MAX(pe, MAX(re, ye)) > abort_attitude_limit;
    }

    // COAST: rate-based tumble detection.
    if (abort_rate_limit <= 0) {
        return false;
    }
    const Vector3f gyro = ahrs.get_gyro();
    const float max_rate_dps = MAX(fabsf(gyro.x), MAX(fabsf(gyro.y), fabsf(gyro.z))) * RAD_TO_DEG;
    return max_rate_dps > (float)abort_rate_limit;
}

void ModeRocket::do_abort()
{
    gcs().send_text(MAV_SEVERITY_CRITICAL, "Rocket: ABORT!");
    enter_phase(Phase::ABORT);
#if HAL_PARACHUTE_ENABLED
    if (plane.parachute.enabled()) plane.parachute_release();
#endif
}

/*
  GPS quality monitoring.

  Nothing in BOOST or COAST uses GPS: stabilisation runs off the IMU and
  apogee detection off the barometer. This exists purely so the handover at
  the end of the flight does not drop the rocket into a mode that needs a
  position it does not have. Jamming and spoofing are the expected case,
  not an exception.
*/
void ModeRocket::reset_gps_monitor()
{
    last_gps_health = GPSHealth::LOST;
    gps_last_fix_ms = 0;
    gps_monitor_start_ms = AP_HAL::millis();
    gps_prev_loc_valid = false;
}

void ModeRocket::update_gps_health()
{
    const AP_GPS &gps = AP::gps();
    const uint32_t now = AP_HAL::millis();
    const bool have_fix = gps.status() >= AP_GPS_FixType::FIX_3D;

    if (have_fix) {
        gps_last_fix_ms = now;
    }

    // Before the first fix the timeout runs from mode entry, so a receiver
    // that never acquires is still declared LOST rather than staying OK.
    const uint32_t since_fix = now - (gps_last_fix_ms != 0 ? gps_last_fix_ms : gps_monitor_start_ms);
    if (!have_fix && since_fix > (uint32_t)gps_lost_ms.get()) {
        last_gps_health = GPSHealth::LOST;
        gps_prev_loc_valid = false;
        return;
    }
    if (!have_fix) {
        return;
    }

    GPSHealth health = GPSHealth::OK;

    // Spoof check: compare the step between consecutive fixes against what
    // the inertial estimate says is possible. A jammer or simulator that
    // walks the position will not agree with the accelerometers.
    const Location &loc = gps.location();
    if (gps_jump_m.get() > 0 && gps_prev_loc_valid) {
        const float jump_m = gps_prev_loc.get_distance_NED(loc).length();
        // One GPS update at 5 Hz plus margin for the estimate itself.
        const float plausible_m = inertial_velocity_mps * 0.25f + 30.0f;
        if (jump_m > (float)gps_jump_m.get() && jump_m > plausible_m) {
            health = GPSHealth::SPOOFED;
        }
    }
    gps_prev_loc = loc;
    gps_prev_loc_valid = true;

    if (health == GPSHealth::OK) {
        // Satellite count and HDOP are independent symptoms: a receiver under
        // partial jamming can hold the satellite count while the geometry of
        // the solution decays, so both are checked.
        const float hdop = gps.get_hdop() * 0.01f;   // AP_GPS reports HDOP*100
        if (gps.num_sats() < (uint8_t)gps_min_sat.get() ||
            (is_positive(gps_max_hdop) && hdop > gps_max_hdop)) {
            health = GPSHealth::DEGRADED;
        }
    }

    if (health == GPSHealth::SPOOFED && last_gps_health != GPSHealth::SPOOFED) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Rocket: GPS spoof suspected");
    }
    last_gps_health = health;
}

/*
  Pick the mode to hand over to, given GPS quality.

  A mode that needs position will not hold a course without one, so with the
  fix lost or spoofed the mode falls back to RKT_GPS_FBK (FBWA by default).
  FBWA keeps the airframe stable on IMU alone and leaves the pilot in
  control, which is the best available outcome under jamming.
*/
Mode::Number ModeRocket::resolve_post_mode(Mode::Number desired) const
{
    if (!gps_protect) {
        return desired;
    }
    if (last_gps_health == GPSHealth::OK || last_gps_health == GPSHealth::DEGRADED) {
        return desired;
    }

    bool needs_position;
    switch (desired) {
        case Mode::Number::AUTO:
        case Mode::Number::RTL:
        case Mode::Number::LOITER:
        case Mode::Number::GUIDED:
        case Mode::Number::CIRCLE:
        case Mode::Number::CRUISE:
        case Mode::Number::AVOID_ADSB:
        case Mode::Number::QRTL:
        case Mode::Number::QLOITER:
        case Mode::Number::QLAND:
        case Mode::Number::LOITER_ALT_QLAND:
        case Mode::Number::TAKEOFF:
        case Mode::Number::THERMAL:
            needs_position = true;
            break;
        default:
            needs_position = false;
            break;
    }
    if (!needs_position) {
        return desired;
    }

    gcs().send_text(MAV_SEVERITY_WARNING, "Rocket: GPS %s, mode %d -> %d",
                    last_gps_health == GPSHealth::LOST ? "lost" : "spoofed",
                    (int)desired, (int)gps_fallback_mode.get());
    return (Mode::Number)gps_fallback_mode.get();
}

/*
  Hand over to the post-flight mode.

  The attempt is made once and the outcome reported, then retried at 1 Hz if
  it failed. The previous code called this straight out of the 400 Hz state
  machine with no record of success, which both flooded the GCS text queue
  (so the failure was invisible) and gave no way to tell a failed handover
  from one that never ran.
*/
bool ModeRocket::transition_to_next_mode()
{
    const uint32_t now = AP_HAL::millis();
    if (handover_attempt_ms != 0 && now - handover_attempt_ms < 1000) {
        return false;
    }
    handover_attempt_ms = now;

    const Mode::Number t = resolve_post_mode((Mode::Number)transition_mode.get());
    if (plane.set_mode_by_number(t, ModeReason::MISSION_END)) {
        gcs().send_text(MAV_SEVERITY_INFO, "Rocket: handed over to mode %d", (int)t);
        return true;
    }
    gcs().send_text(MAV_SEVERITY_WARNING, "Rocket: mode %d refused, trying FBWA", (int)t);
    if (plane.set_mode_by_number(Mode::Number::FLY_BY_WIRE_A, ModeReason::MISSION_END)) {
        gcs().send_text(MAV_SEVERITY_INFO, "Rocket: handed over to FBWA");
        return true;
    }
    gcs().send_text(MAV_SEVERITY_CRITICAL, "Rocket: handover failed, staying in ROCKET");
    return false;
}

#endif // MODE_ROCKET_ENABLED
