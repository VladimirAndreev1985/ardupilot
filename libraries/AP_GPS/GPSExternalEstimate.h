#pragma once

#include "AP_GPS.h"  // <-- базовый класс GPSDriver

class GPSExternalEstimate : public AP_GPS_Backend {
public:
    GPSExternalEstimate();

    // вызывается периодически внутри AP_GPS, чтобы прочитать новые данные
    int8_t read() override;

    // вызывается при инициализации драйвера GPS
    bool configure() override;

    // Наш метод, чтобы установить данные извне (lat, lon, alt, time_ms)
    void set_position(int32_t lat, int32_t lon, float alt, uint32_t time_ms);

private:
    uint32_t _last_update_ms;
    Location _loc;  // В ArduPilot Location хранит lat/lng в int32 (1e7 формат)
};
