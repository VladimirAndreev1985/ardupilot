#include "GPSExternalEstimate.h"
#include <AP_HAL/AP_HAL.h>

GPSExternalEstimate::GPSExternalEstimate()
{
    // инициализация полей
    _last_update_ms = 0;
    _loc.lat = 0;
    _loc.lng = 0;
    _loc.alt = 0;
    _loc.flags = 0; // обычно хранит bitmask
}

bool GPSExternalEstimate::configure()
{
    // У нас нет реального протокола, так что просто вернём true
    return true;
}

int8_t GPSExternalEstimate::read()
{
    // вызывается регулярно. Проверим, нет ли "просрочки" данных
    uint32_t now = AP_HAL::millis();

    // Если данные приходили недавно, считаем, что fix есть
    if ((now - _last_update_ms) < 2000) {
        // Заполняем поля структуры state (она унаследована от GPSDriver):
        state.location = _loc;
        // фиктивный уровень сигнала
        state.status = GPS::GPSStatus::GPS_OK_FIX_3D; // можно GPS_OK_FIX_2D
        state.hdop = 100; // пример (hdop=1.0 => 100, hdop=0.5 => 50, в некоторых ветках)
        // можно ground_speed и ground_course задать, если нужно
        return 1; // сообщаем AP_GPS, что "новые данные" есть
    } else {
        // Давно не получали координаты => no fix
        state.status = GPS::GPSStatus::NO_GPS;
        return 0; // нет новых данных
    }
}

void GPSExternalEstimate::set_position(int32_t lat, int32_t lon, float alt, uint32_t time_ms)
{
    // lat/lon уже в int32, т.е. 1e7 формат
    _loc.lat = lat;  // в ArduPilot Location lat/lng тоже хранятся в 1e7
    _loc.lng = lon; 
    // alt обычно в сантиметрах (Location.alt), если ваша прошивка так ждёт
    // можно alt в метрах -> alt*100
    _loc.alt = (int32_t)(alt * 100.0f);

    // отметим флаги, что высота есть
    // _loc.flags |= LOCATION_ALT_VALID;
    // _loc.flags |= LOCATION_COORDINATES_VALID; // если нужно

    _last_update_ms = AP_HAL::millis(); 
}
