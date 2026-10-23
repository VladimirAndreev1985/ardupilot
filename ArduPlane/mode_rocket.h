/*
   Rocket flight mode for ArduPlane
   Provides stabilization during rocket boost phase and transition to normal flight modes

   Author: VladimirAndreev1985
   License: GPLv3
*/
#pragma once

#include "mode.h"

// MODE_ROCKET_ENABLED is defined in mode.h, alongside the other mode
// feature defines, so that it is visible to the Mode::Number enum itself.

#if MODE_ROCKET_ENABLED

class ModeRocket : public Mode
{
public:
    ModeRocket();

    Number mode_number() const override { return Number::ROCKET; }
    const char *name() const override { return "Rocket"; }
    const char *name4() const override { return "RCKT"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;
    void run() override;

    bool does_auto_throttle() const override { return true; }
    bool is_taking_off() const override;
    
    // Rocket mode parameters
    static const struct AP_Param::GroupInfo var_info[];

    // Flight phases
    enum class Phase : uint8_t {
        PAD = 0,           // On launch pad, waiting for launch
        RAIL = 1,          // On launch rail, controls locked
        BOOST = 2,         // Rocket motor burning, active stabilization
        COAST = 3,         // Motor burned out, coasting
        TRANSITION = 4,    // Transitioning to next flight mode
        ABORT = 5,         // Emergency - deploy parachute
    };
    
    Phase get_phase() const { return current_phase; }
    const char* get_phase_name() const;

    // Get estimated speed, m/s. Dead-reckoned when there is no GPS.
    float get_inertial_velocity() const { return inertial_velocity_mps; }

    // Quality of the GPS solution. The rocket does not need GPS to fly:
    // BOOST and COAST are stabilised against the launch attitude using the
    // IMU alone. GPS quality only decides which mode is safe to hand over
    // to once the rocket is subsonic and falling.
    enum class GPSHealth : uint8_t {
        OK       = 0,   // 3D fix, satellite count and jump check all pass
        DEGRADED = 1,   // fix present but weak; still usable for position
        SPOOFED  = 2,   // position jumped further than inertial data allows
        LOST     = 3,   // no 3D fix for longer than RKT_GPS_LOST_MS
    };

    GPSHealth get_gps_health() const { return last_gps_health; }

protected:
    bool _enter() override;
    void _exit() override;
    bool _pre_arm_checks(size_t buflen, char *buffer) const override;

private:
    // State machine
    Phase current_phase;
    uint32_t phase_start_ms;
    uint32_t launch_detect_start_ms;
    
    // Launch detection
    bool detect_launch();
    uint8_t launch_accel_count;
    
    // --- Velocity estimation without GPS ---
    //
    // The mode dead-reckons velocity in the NED frame by integrating
    // earth-frame acceleration. get_forward_accel() deliberately removes
    // gravity, which is right for launch detection but wrong for velocity:
    // with gravity removed the vertical component never decelerates, so
    // apogee is never reached and the old speed-magnitude test could not
    // detect it. Integrating ahrs.get_accel_ef() with gravity cancelled
    // only in the Down axis gives a true kinematic acceleration and works
    // with no GPS at all. The estimate drifts, which is accepted: over a
    // 30 s flight it is still far better than having no velocity at all.
    Vector3f dr_velocity_ned;           // dead-reckoned velocity, NED, m/s
    bool     dr_velocity_valid;
    float    inertial_velocity_mps;     // speed magnitude, m/s
    uint32_t last_velocity_update_ms;
    void  update_inertial_velocity();

    // Climb rate from the best source available, m/s, positive up. Falls
    // back through EKF velocity, vertical position rate, baro and finally
    // dead reckoning, so it stays usable with GPS jammed or absent.
    // Not const: AP_Baro::get_climb_rate() is not const.
    float get_climb_rate();

    // --- GPS quality monitoring (only used to pick the handover mode) ---
    void      update_gps_health();
    Mode::Number resolve_post_mode(Mode::Number desired) const;
    void      reset_gps_monitor();
    GPSHealth last_gps_health;
    uint32_t  gps_last_fix_ms;
    uint32_t  gps_monitor_start_ms;
    Location  gps_prev_loc;
    bool      gps_prev_loc_valid;

    // Forward (body x) acceleration with gravity removed, m/s/s
    float get_forward_accel() const;
    
    // Initial orientation at launch
    float launch_pitch_cd;
    float launch_yaw_cd;
    
    // Stabilization
    void stabilize_boost();
    void stabilize_coast();
    float get_speed_scaler() const;
    
    // Phase transitions
    void check_phase_transition();
    void enter_phase(Phase new_phase);
    bool should_abort() const;
    void do_abort();
    
    // Transition to next mode
    bool transition_to_next_mode();
    uint32_t handover_attempt_ms;
    
    // Parameters
    AP_Int8 launch_accel_threshold;     // m/s^2, minimum accel to detect launch
    AP_Int16 launch_accel_time_ms;      // ms, time accel must be sustained
    AP_Int16 rail_time_ms;              // ms, time to keep controls locked after launch
    AP_Int8 boost_pitch_limit;          // deg, max pitch deviation during boost
    AP_Int8 boost_roll_limit;           // deg, max roll deviation during boost  
    AP_Int8 boost_yaw_limit;            // deg, max yaw deviation during boost
    AP_Float boost_time_max;            // s, maximum expected boost duration
    AP_Float coast_time_max;            // s, maximum coast time before transition
    AP_Int8 transition_mode;            // Mode to transition to after coast
    AP_Int8 abort_attitude_limit;       // deg, attitude deviation to trigger abort
    AP_Int8 fin_angle_offset;           // deg, fin mounting angle (0=cross, 45=X)
    AP_Float speed_scaler_min;          // minimum speed scaler value
    AP_Float speed_scaler_max;          // maximum speed scaler value
    AP_Float speed_scaler_ref;          // m/s, speed at which the scaler is 1.0
    AP_Float drag_k;                    // 1/m, quadratic drag (legacy, unused)
    AP_Float transition_speed;          // m/s, speed below which to transition
    AP_Float transition_vz;             // m/s, climb rate below which to transition
    AP_Int16 abort_rate_limit;          // deg/s, body rate triggering abort in COAST
    AP_Int8  gps_protect;               // 0 = off, 1 = pick handover mode by GPS health
    AP_Int8  gps_fallback_mode;         // mode used when GPS is lost or spoofed
    AP_Int32 gps_lost_ms;               // ms without 3D fix before GPS is LOST
    AP_Int16 gps_jump_m;                // m, position jump that implies spoofing
    AP_Int8  gps_min_sat;               // satellites below which GPS is DEGRADED
    
    // PID gains (configurable)
    AP_Float kp_pitch;
    AP_Float kd_pitch;
    AP_Float kp_roll;
    AP_Float kd_roll;
    AP_Float kp_yaw;
    AP_Float kd_yaw;
    
    // Launch angle setting
    AP_Int8 launch_angle;               // deg from vertical, -1 = use current
public:
    // For mixer access to fin_angle_offset
    int8_t get_fin_angle() const { return fin_angle_offset; }
};

#endif // MODE_ROCKET_ENABLED
