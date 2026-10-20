#include "mode.h"
#include "Rocket.h"
#include "Plane.h"

/*
   ModeRocket implementation. See mode.h for the class declaration.
*/

const AP_Param::GroupInfo ModeRocket::var_info[] = {
    AP_SUBGROUPINFO(_rocket, "", 0, ModeRocket, Rocket),
    AP_GROUPEND
};

ModeRocket::ModeRocket()
    : Mode()
    , post_target(Mode::Number::FLY_BY_WIRE_A)
    , handover_started(false)
{
}

bool ModeRocket::_enter()

{
    if (!rocket().enabled()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "ROCKET mode disabled (RKTN_ENABLE=0)");
        return false;
    }

    handover_started = false;
    gcs().send_text(MAV_SEVERITY_INFO, "ROCKET: waiting for launch");
    rocket().set_stage(Rocket::Stage::WAIT_LAUNCH);
    rocket().reset_gps_monitor();
    switch (rocket().post_mode()) {
    case Rocket::PostMode::MANUAL: post_target = Mode::Number::MANUAL; break;
    case Rocket::PostMode::FBWA:   post_target = Mode::Number::FLY_BY_WIRE_A; break;
    case Rocket::PostMode::AUTO:   post_target = Mode::Number::AUTO; break;
    case Rocket::PostMode::CIRCLE: post_target = Mode::Number::CIRCLE; break;
    default:                       post_target = Mode::Number::FLY_BY_WIRE_A; break;
    }
    return true;
}

void ModeRocket::_exit()
{
}

bool ModeRocket::sequence_active() const
{
    const Rocket::Stage s = (Rocket::Stage)_rocket.get_stage();
    return s == Rocket::Stage::BOOST || s == Rocket::Stage::COAST;
}

void ModeRocket::update()
{
    Rocket &rocket_inst = rocket();
    if (!rocket_inst.enabled()) {
        IGNORE_RETURN(plane.set_mode_by_number(post_target, ModeReason::UNKNOWN));
        return;
    }

    rocket_inst.update();
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    rocket_inst.apply_sitl_stage_hold();
#endif
    const Vector3f &accel = ahrs.get_accel();

    float climb_rate = 0.0f;
    Vector3f vel_ned;
    bool have_climb = ahrs.get_velocity_NED(vel_ned);
    if (have_climb) {
        climb_rate = -vel_ned.z;
    } else {
        float vel_d = 0.0f;
        if (ahrs.get_velocity_D(vel_d)) {
            climb_rate = -vel_d;
            have_climb = true;
        }
    }
    if (!have_climb) {
        climb_rate = NAN;
    }
    rocket_inst.log_sensors(accel.x, climb_rate);

    if (rocket_inst.sitl_stage_held()) {
        return;
    }

    switch (rocket_inst.get_stage()) {
    case Rocket::Stage::WAIT_LAUNCH:
        if (rocket_inst.detect_launch(accel, climb_rate)) {
            rocket_inst.set_stage(Rocket::Stage::BOOST);
        }
        break;
    case Rocket::Stage::BOOST:
        if (rocket_inst.detect_burnout(accel, climb_rate)) {
            rocket_inst.set_stage(Rocket::Stage::COAST);
        }
        break;
    case Rocket::Stage::COAST:
        if (rocket_inst.detect_coast_complete(climb_rate)) {
            rocket_inst.set_stage(Rocket::Stage::DONE);
        }
        break;
    case Rocket::Stage::DONE: {
        if (handover_started) {
            break;
        }
        handover_started = true;

        // A long failsafe raised during BOOST/COAST was deferred by
        // Plane::failsafe_long_on_event(). Act on it now instead of the
        // configured post mode: the operator has lost the link, so FS_LONG_ACTN
        // decides. Must run before any set_mode, because Mode::enter() clears
        // long_failsafe_pending.
        if (plane.long_failsafe_pending) {
            plane.long_failsafe_pending = false;
            const int16_t fs_state = plane.failsafe.state;
            if (fs_state == FAILSAFE_LONG || fs_state == FAILSAFE_GCS) {
                gcs().send_text(MAV_SEVERITY_WARNING,
                                "ROCKET: complete, running deferred failsafe");
                plane.failsafe_long_on_event((enum failsafe_state)fs_state,
                                             fs_state == FAILSAFE_GCS ?
                                             ModeReason::GCS_FAILSAFE :
                                             ModeReason::RADIO_FAILSAFE);
                break;
            }
        }

        const Mode::Number resolved_mode = (Mode::Number)rocket_inst.resolve_post_mode((uint8_t)post_target);
        gcs().send_text(MAV_SEVERITY_INFO, "ROCKET: complete, switching mode %u", (unsigned)resolved_mode);
        IGNORE_RETURN(plane.set_mode_by_number(resolved_mode, ModeReason::MISSION_CMD));
        break;
    }
    }
}
