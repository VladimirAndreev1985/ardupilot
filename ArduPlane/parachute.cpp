#include "Plane.h"

/* 
   call parachute library update
*/
void Plane::parachute_check()
{
#if HAL_PARACHUTE_ENABLED
    static bool init_pillow = false;  // Флаг, чтобы выставить значения один раз
    
    // Только один раз, при старте
    if (!init_pillow) {
        // Задаём серво2 = 1100, серво3 = 988 (закрытая крышка и выкл. надув)
        SRV_Channels::set_output_pwm_chan_timeout(2, 1100, 1000);
        SRV_Channels::set_output_pwm_chan_timeout(3, 988, 1000);
        init_pillow = true;
    }

    // Дальше стандартная логика парашюта
    parachute.update();
    parachute.check_sink_rate();
#endif
}

#if HAL_PARACHUTE_ENABLED

/*
  parachute_release - trigger the release of the parachute
*/
void Plane::parachute_release()
{
    if (parachute.release_in_progress()) {
        return;
    }
    if (parachute.released()) {
        gcs().send_text(MAV_SEVERITY_CRITICAL,"Parachute: Released again");
    } else {
        gcs().send_text(MAV_SEVERITY_CRITICAL,"Parachute: Released");
    }

    // Срабатывание парашюта
    parachute.release();

    // --- ДВЕ СТРОКИ ДЛЯ ОДНОВРЕМЕННОГО ЗАПУСКА ---
    SRV_Channels::set_output_pwm_chan_timeout(2, 2000, 5000);
    SRV_Channels::set_output_pwm_chan_timeout(3, 1600, 0);

#if AP_LANDINGGEAR_ENABLED
    // deploy landing gear
    g2.landing_gear.set_position(AP_LandingGear::LandingGear_Deploy);
#endif
} // <-- закрываем функцию ровно один раз

/*
  parachute_manual_release - trigger the release of the parachute,
  after performing some checks for pilot error checks if the vehicle
  is landed
*/
bool Plane::parachute_manual_release()
{
    // exit immediately if parachute is not enabled
    if (!parachute.enabled() || parachute.released()) {
        return false;
    }

    if (parachute.alt_min() > 0 && relative_ground_altitude(false) < parachute.alt_min() &&
            auto_state.last_flying_ms > 0) {
        // Allow manual ground tests by only checking if flying too low if we've taken off
        gcs().send_text(MAV_SEVERITY_WARNING, "Parachute: Too low");
        return false;
    }

    // if we get this far release parachute
    parachute_release();

#if AP_LANDINGGEAR_ENABLED
    // deploy landing gear
    g2.landing_gear.set_position(AP_LandingGear::LandingGear_Deploy);
#endif
    return true;    
}

/*
  activate_parachute - triggers parachute deployment due to failsafe conditions
*/

#endif
