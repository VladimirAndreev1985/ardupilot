#include "Plane.h"

/*
 *  failsafe support
 *  Andrew Tridgell, December 2011
 */

/*
 *  our failsafe strategy is to detect main loop lockup and switch to
 *  passing inputs straight from the RC inputs to RC outputs.
 */

/*
 *  this failsafe_check function is called from the core timer interrupt
 *  at 1kHz.
 */


static uint32_t failsafe_timer = 0;

void Plane::check_attitude_failsafe() {
//    if (failsafe.state != FAILSAFE_NONE) { 
        float roll = ahrs.roll_sensor * 0.01f;   // В градусах
        float pitch = ahrs.pitch_sensor * 0.01f; // В градусах
        
        // Берём значения из параметров вместо фиксированных констант
        float max_roll = g.parachute_roll_limit.get();
        float max_pitch = g.parachute_pitch_limit.get();

        if (fabsf(roll) > max_roll || fabsf(pitch) > max_pitch) {
            if (failsafe_timer == 0) {
                failsafe_timer = AP_HAL::millis();
            }
            if ((AP_HAL::millis() - failsafe_timer) > static_cast<uint32_t>(g.parachute_timeout_ms.get())) {
            activate_parachute();
                }
        } else {
            failsafe_timer = 0;
        }
//    }
}

void Plane::activate_parachute() {
    // Проверяем, не выпущен ли уже парашют
    if (failsafe.state == Failsafe_Action_Parachute) {
        // значит уже выпустили парашют и выслали сообщение
        return;
    }
    failsafe.state = Failsafe_Action_Parachute; // Устанавливаем состояние
    parachute_release();
    gcs().send_text(MAV_SEVERITY_CRITICAL, "Failsafe: Excessive Roll/Pitch, deploying parachute!");
}

void Plane::failsafe_check(void)
{
    static uint16_t last_ticks;
    static uint32_t last_timestamp;
    static bool in_failsafe;
    uint32_t tnow = micros();

    const uint16_t ticks = scheduler.ticks();
    if (ticks != last_ticks) {
        last_ticks = ticks;
        last_timestamp = tnow;
        in_failsafe = false;
        return;
    }

    if (tnow - last_timestamp > 200000) {
        in_failsafe = true;
    }

    if (in_failsafe && tnow - last_timestamp > 20000) {
        rc().read_input();
        last_timestamp = tnow;
        rc().read_input();

#if AP_ADVANCEDFAILSAFE_ENABLED
        if (in_calibration) {
            afs.heartbeat();
        }
#endif

        if (RC_Channels::get_valid_channel_count() < 5) {
            return;
        }

        RC_Channels::clear_overrides();

        float roll = roll_in_expo(false);
        float pitch = pitch_in_expo(false);
        float throttle = get_throttle_input(true);
        float rudder = rudder_in_expo(false);

        if (!arming.is_armed_and_safety_off()) {
            throttle = 0;
        }
        
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, roll);
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, pitch);
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, rudder);
        SRV_Channels::set_output_scaled(SRV_Channel::k_steering, rudder);
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle);

#if AP_ADVANCEDFAILSAFE_ENABLED
        if (afs.should_crash_vehicle()) {
            afs.terminate_vehicle();
            if (!afs.terminating_vehicle_via_landing()) {
                return;
            }
        }
#endif

        SRV_Channels::copy_radio_in_out(SRV_Channel::k_manual, true);
        SRV_Channels::set_output_scaled(SRV_Channel::k_flap, 0.0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_flap_auto, 0.0);

        flaperon_update();
        servos_output();

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        GCS_MAVLINK *chan = gcs().chan(0);
        if (HAVE_PAYLOAD_SPACE(chan->get_chan(), SERVO_OUTPUT_RAW)) {
            chan->send_servo_output_raw();
        }
#endif
    }
    
    check_attitude_failsafe();
}
