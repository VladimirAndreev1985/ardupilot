#pragma once

#include "AP_GPS.h"
#include "GPS_Backend.h"

class GPSExternalEstimate : public AP_GPS_Backend {
public:
    GPSExternalEstimate(AP_GPS &_gps, AP_GPS::GPS_State &_state);

    bool read() override;

    void set_position(int32_t lat, int32_t lon, float alt, uint32_t time_ms);

private:
    uint32_t _last_update_ms;
    Location _loc;
};
