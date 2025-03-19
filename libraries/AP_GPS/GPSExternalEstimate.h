#pragma once

#include "AP_GPS.h" // <-- оставьте только эту строку и удалите строку с AP_GPS_Backend.h

class GPSExternalEstimate : public AP_GPS_Backend {
public:
    GPSExternalEstimate(AP_GPS &gps, AP_GPS::GPS_State &_state);

    int8_t read() override;
    bool configure() override;

    void set_position(int32_t lat, int32_t lon, float alt, uint32_t time_ms);

private:
    uint32_t _last_update_ms;
    Location _loc;
};
