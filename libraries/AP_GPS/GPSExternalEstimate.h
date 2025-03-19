#pragma once

#include "AP_GPS.h"
#include "GPS_Backend.h"

class GPSExternalEstimate : public AP_GPS_Backend {
public:
    GPSExternalEstimate(AP_GPS &_gps, AP_GPS::Params &_params, AP_GPS::GPS_State &_state);
    void set_position(int32_t lat, int32_t lon, float timestamp);
    bool read() override;

    const char *name() const override { return "ExternalEstimate"; }

private:
    uint32_t _last_update_ms;
    Location _loc;
};
