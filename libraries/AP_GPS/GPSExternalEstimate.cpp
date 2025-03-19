#include "GPSExternalEstimate.h"
#include <AP_HAL/AP_HAL.h>

GPSExternalEstimate::GPSExternalEstimate(AP_GPS &_gps, AP_GPS::GPS_State &_state) :
    AP_GPS_Backend(_gps, _state),
    _last_update_ms(0)
{
    _loc.lat = 0;
    _loc.lng = 0;
    _loc.alt = 0;
}

bool GPSExternalEstimate::read()
{
    uint32_t now = AP_HAL::millis();

    if ((now - _last_update_ms) < 2000) {
        state.location = _loc;
        state.status = AP_GPS::GPS_OK_FIX_3D;
        state.hdop = 100;
        return true;
    } else {
        state.status = AP_GPS::NO_GPS;
        return false;
    }
}

void GPSExternalEstimate::set_position(int32_t lat, int32_t lon, float alt, uint32_t time_ms)
{
    _loc.lat = lat;
    _loc.lng = lon;
    _loc.alt = (int32_t)(alt * 100.0f);
    _last_update_ms = AP_HAL::millis();
}
