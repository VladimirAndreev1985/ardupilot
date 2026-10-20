/*
   Rocket implementation. See Rocket.h for design notes.
*/

#include "Rocket.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>

const AP_Param::GroupInfo Rocket::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Включение режима ROCKET
    // @Description: Установите 1 для включения режима ROCKET. При отключении режим скрывается из списка режимов полёта.
    // @Values: 0:Отключено,1:Включено
    // @User: Advanced
    AP_GROUPINFO("ENABLE", 1, Rocket, enable, 0),

    // @Param: TAIL_MODE
    // @DisplayName: Геометрия хвостового оперения
    // @Description: Схема рулей во всех режимах Plane: 0 = CROSS (штатные выходы); 1 = X45 (рули 190–193 и микшер независимо от RKTN_ENABLE).
    // @Values: 0:Крестовая схема,1:X-образная схема 45 градусов
    // @User: Advanced
    AP_GROUPINFO("TAIL_MODE", 2, Rocket, tail_mode_p, 0),

    // @Param: BOOST_TIME
    // @DisplayName: Длительность этапа BOOST
    // @Description: Штатный тайм-аут этапа BOOST: по его истечении выполняется переход в COAST, даже если датчики не обнаружили выгорание.
    // @Units: s
    // @Range: 0.5 10.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("BOOST_TIME", 3, Rocket, boost_time, 2.5f),

    // @Param: LCH_ACC
    // @DisplayName: Порог ускорения для обнаружения старта ROCKET
    // @Description: Порог вертикального ускорения, выше которого система считает, что аппарат покинул стартовую позицию.
    // @Units: m/s/s
    // @Range: 1.0 50.0
    // @Increment: 0.5
    // @User: Advanced
    AP_GROUPINFO("LCH_ACC", 4, Rocket, launch_accel, 15.0f),

    // @Param: LCH_CLIMB
    // @DisplayName: Порог вертикальной скорости для обнаружения старта ROCKET
    // @Description: Порог вертикальной скорости, выше которого система считает, что аппарат покинул стартовую позицию.
    // @Units: m/s
    // @Range: 1.0 30.0
    // @Increment: 0.5
    // @User: Advanced
    AP_GROUPINFO("LCH_CLIMB", 5, Rocket, launch_climb, 8.0f),

    // @Param: BURN_ACC
    // @DisplayName: Порог ускорения выгорания двигателя
    // @Description: Переход из BOOST в COAST при вертикальном ускорении ниже этого порога. Переход также возможен по скорости набора высоты или тайм-ауту BOOST.
    // @Units: m/s/s
    // @Range: 0.0 30.0
    // @Increment: 0.5
    // @User: Advanced
    AP_GROUPINFO("BURN_ACC", 30, Rocket, burn_accel, 11.0f),

    // @Param: BOOST_DAMP
    // @DisplayName: Демпфирование по гироскопу на этапе BOOST
    // @Description: Отклонение рулевой поверхности на единицу угловой скорости корпуса (град/с) на этапе BOOST.
    // @Range: 0.0 2.0
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("BOOST_DAMP", 6, Rocket, boost_damp, 0.20f),

    // @Param: COAST_DAMP
    // @DisplayName: Демпфирование по гироскопу на этапе COAST
    // @Description: Отклонение рулевой поверхности на единицу угловой скорости корпуса (град/с) на этапе COAST.
    // @Range: 0.0 2.0
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("COAST_DAMP", 7, Rocket, coast_damp, 0.40f),

    // @Param: BOOST_GMAX
    // @DisplayName: Предел угловой скорости для демпфирования BOOST
    // @Description: Угловая скорость (град/с), при которой выход демпфирования BOOST достигает ограничения.
    // @Units: deg/s
    // @Range: 10 300
    // @Increment: 5
    // @User: Advanced
    AP_GROUPINFO("BOOST_GMAX", 8, Rocket, boost_gmax, 60.0f),

    // @Param: COAST_GMAX
    // @DisplayName: Предел угловой скорости для демпфирования COAST
    // @Description: Угловая скорость, при которой выход демпфирования COAST достигает ограничения.
    // @Units: deg/s
    // @Range: 10 300
    // @Increment: 5
    // @User: Advanced
    AP_GROUPINFO("COAST_GMAX", 9, Rocket, coast_gmax, 90.0f),

    // @Param: BOOST_THR
    // @DisplayName: Смещение сервоприводов на этапе BOOST
    // @Description: Постоянное смещение команд сервоприводам на этапе BOOST.
    // @Range: -1.0 1.0
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("BOOST_THR", 10, Rocket, boost_thr, 0.0f),

    // @Param: COAST_THR
    // @DisplayName: Смещение сервоприводов на этапе COAST
    // @Description: Постоянное смещение команд сервоприводам на этапе COAST.
    // @Range: -1.0 1.0
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("COAST_THR", 11, Rocket, coast_thr, 0.0f),

    // @Param: BOOST_MIN
    // @DisplayName: Нижний предел рулей на этапе BOOST
    // @Description: Нижний предел нормированной команды руля на BOOST (-1..0), применяемый к каждому рулю после смешения осей. Вместе с BOOST_MAX ограничивает ход относительно SERVOx_TRIM. Для нейтральных рулей задайте обе границы равными нулю.
    // @Range: -1.0 0.0
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("BOOST_MIN", 12, Rocket, boost_min, -0.05f),

    // @Param: BOOST_MAX
    // @DisplayName: Верхний предел рулей на этапе BOOST
    // @Description: Верхний предел нормированной команды руля на BOOST (0..1), применяемый к каждому рулю после смешения осей. Вместе с BOOST_MIN ограничивает ход относительно SERVOx_TRIM; ноль в обеих границах удерживает рули в нейтрали.
    // @Range: 0.0 1.0
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("BOOST_MAX", 13, Rocket, boost_max, 0.05f),

    // @Param: XWING_GAIN
    // @DisplayName: Коэффициент микшера X-wing
    // @Description: Масштабирует смешанные команды крена, тангажа и рыскания. Значение по умолчанию 1/√2 уменьшает насыщение рулей при одновременной команде по двум осям.
    // @Range: 0.3 1.0
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("XWING_GAIN", 14, Rocket, xwing_gain, 0.707f),

    // @Param: POST_MODE
    // @DisplayName: Режим после завершения ROCKET
    // @Description: Режим ArduPlane после этапа DONE, если защита GPS не выбрала резервный режим: 0=MANUAL (ручное управление), 1=FBWA (стабилизация по крену/тангажу), 2=AUTO (автополёт по миссии), 3=CIRCLE (полёт по кругу).
    // @Values: 0:Ручной,1:FBWA,2:Автополёт,3:Круг
    // @User: Advanced
    AP_GROUPINFO("POST_MODE", 15, Rocket, post_mode_p, 1),

    // @Param: DBG
    // @DisplayName: Маска отладочных сообщений ROCKET
    // @Description: Битовая маска диагностики: бит 1 (значение 1) включает подробные текстовые сообщения в GCS, бит 2 (значение 2) записывает переходы между этапами в лог. Значение 3 включает оба вида диагностики.
    // @Bitmask: 1:Подробные сообщения GCS,2:Переходы этапов в лог
    // @User: Advanced
    AP_GROUPINFO("DBG", 16, Rocket, debug_mask, 0),

    // ---------------- GPS protection (RKTN_GPS_*) ----------------
    // The rocket does not need GPS for BOOST or COAST: those stages run
    // pure gyro-rate damping. GPS matters only for (a) the mode we hand
    // control over to at DONE, and (b) locating the airframe afterwards.
    // These parameters let the mode detect GNSS jamming / spoofing and
    // pick a safe handover mode when the position source is hostile.

    // @Param: GPS_PROTECT
    // @DisplayName: Включение защиты GPS/GNSS для ROCKET
    // @Description: При значении 1 режим ROCKET контролирует качество GPS и при обнаружении помех или спуфинга выбирает RKTN_GPS_FBK вместо RKTN_POST_MODE при переходе из DONE. Стабилизация в BOOST и COAST не зависит от GPS.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("GPS_PROTECT", 17, Rocket, gps_protect, 1),

    // @Param: GPS_LOST_MS
    // @DisplayName: Время до признания GPS потерянным
    // @Description: GPS считается потерянным после указанного времени без 3D-fix. До первого fix отсчёт начинается при входе в ROCKET.
    // @Units: ms
    // @Range: 500 60000
    // @Increment: 500
    // @User: Advanced
    AP_GROUPINFO("GPS_LOST_MS", 18, Rocket, gps_lost_ms, 5000),

    // @Param: GPS_JUMP_M
    // @DisplayName: Порог скачка координат GPS/GNSS
    // @Description: Если за одно обновление координаты сместились дальше этого порога, а скорость EKF указывает на существенно меньшее перемещение, GPS-решение считается подозрительным и не используется при выборе режима после DONE.
    // @Units: m
    // @Range: 50 2000
    // @Increment: 10
    // @User: Advanced
    AP_GROUPINFO("GPS_JUMP_M", 19, Rocket, gps_jump_m, 200),

    // @Param: GPS_MIN_SAT
    // @DisplayName: Минимальное число спутников GPS/GNSS
    // @Description: При меньшем числе спутников качество GPS-решения считается сниженным.
    // @Range: 4 32
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("GPS_MIN_SAT", 20, Rocket, gps_min_sat, 8),

    // @Param: GPS_HDOP
    // @DisplayName: Максимальный HDOP GPS/GNSS для нормального качества
    // @Description: HDOP показывает геометрическую неопределённость GPS-позиции: чем значение ниже, тем лучше. Если HDOP выше этого порога, качество GPS считается сниженным; при включённой защите это может повлиять на выбор режима после DONE.
    // @Range: 0.8 10.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("GPS_HDOP", 21, Rocket, gps_max_hdop, 2.0f),

    // @Param: GPS_FBK
    // @DisplayName: Резервный режим при потере или спуфинге GPS
    // @Description: Номер режима ArduPlane для перехода из DONE при RKTN_GPS_PROTECT=1 и потере GPS или обнаружении спуфинга. 0=MANUAL, 1=CIRCLE, 2=STABILIZE, 5=FBWA, 10=AUTO, 11=RTL. Дефолт 5=FBWA.
    // @Range: 0 30
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("GPS_FBK", 22, Rocket, gps_fallback, 5),

    // @Param: BST_HARD
    // @DisplayName: Предельная длительность этапа BOOST
    // @Description: Независимый предел длительности BOOST в миллисекундах. Таймер не зависит от акселерометра и барометра и срабатывает, даже если выгорание нельзя обнаружить по вертикальной скорости (нет барометра, GNSS или доступна только счисленная навигация). По истечении выполняется принудительный переход в COAST. Установите значение выше ожидаемого времени работы двигателя с запасом; обычно датчики обнаруживают выгорание раньше.
    // @Range: 1000 60000
    // @Increment: 100
    // @Units: ms
    // @User: Advanced
    AP_GROUPINFO("BST_HARD", 23, Rocket, boost_hardtime, 8000),

    // @Param: CST_HARD
    // @DisplayName: Предельная длительность этапа COAST
    // @Description: Независимый предел длительности COAST в миллисекундах. При отсутствии достоверной вертикальной скорости (нет барометра, EKF3 работает в режиме счисления без GNSS) обычное условие завершения COAST не сработает; таймер гарантирует переход в DONE и передачу управления пилоту через RKTN_POST_MODE или RKTN_GPS_FBK. Задайте максимальную ожидаемую длительность полёта на этапе COAST с запасом (по умолчанию 30 с).
    // @Range: 5000 180000
    // @Increment: 100
    // @Units: ms
    // @User: Advanced
    AP_GROUPINFO("CST_HARD", 24, Rocket, coast_hardtime, 30000),

    // @Param: CST_MIN
    // @DisplayName: Нижний предел рулей на этапе COAST
    // @Description: Минимальная нормированная команда каждого руля после смешения осей на этапе COAST. Диапазон -1..0 ограничивает ход относительно SERVOx_TRIM; вместе с CST_MAX задаёт допустимый ход. Если обе границы равны 0, рули остаются в нейтрали.
    // @Range: -1.0 0.0
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("CST_MIN", 25, Rocket, coast_min, -1.0f),

    // @Param: CST_MAX
    // @DisplayName: Верхний предел рулей на этапе COAST
    // @Description: Максимальная нормированная команда каждого руля после смешения осей на этапе COAST. Диапазон 0..1 ограничивает ход относительно SERVOx_TRIM; значение 1 разрешает полный ход, заданный SERVOx_MIN/MAX/TRIM. Вместе с CST_MIN ограничивает отклонение рулей.
    // @Range: 0.0 1.0
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("CST_MAX", 26, Rocket, coast_max, 1.0f),

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // @Param: TST_STAGE
    // @DisplayName: Тестовая фиксация этапа ROCKET (только SITL)
    // @Description: Только для симулятора SITL: удерживает WAIT_LAUNCH (0), BOOST (1) или COAST (2) в режиме ROCKET; -1 включает обычные переходы. В полётную прошивку не входит.
    // @Values: -1:Обычные переходы,0:Ожидание старта,1:Разгон,2:Полёт по инерции
    // @User: Advanced
    AP_GROUPINFO("TST_STAGE", 27, Rocket, sitl_stage, -1),
    // @Param: TST_GX
    // @DisplayName: Тестовая угловая скорость крена (только SITL)
    // @Description: Только SITL: задаваемая скорость вращения по крену в рад/с при фиксации этапа.
    // @User: Advanced
    AP_GROUPINFO("TST_GX", 28, Rocket, sitl_gyro_x, 0),
    // @Param: TST_GY
    // @DisplayName: Тестовая угловая скорость тангажа (только SITL)
    // @Description: Только SITL: задаваемая скорость вращения по тангажу в рад/с при фиксации этапа.
    // @User: Advanced
    AP_GROUPINFO("TST_GY", 29, Rocket, sitl_gyro_y, 0),
    // @Param: TST_GZ
    // @DisplayName: Тестовая угловая скорость рыскания (только SITL)
    // @Description: Только SITL: задаваемая скорость вращения по рысканию в рад/с при фиксации этапа.
    // @User: Advanced
    AP_GROUPINFO("TST_GZ", 31, Rocket, sitl_gyro_z, 0),
#endif

    AP_GROUPEND
};

Rocket::Rocket()
{
    AP_Param::setup_object_defaults(this, var_info);
    stage.set((uint8_t)Stage::WAIT_LAUNCH);
    stage_enter_ms = 0;
    armed_ms = 0;
    launch_ms = 0;
    burnout_ms = 0;
    launch_detected = false;

    // Текущее состояние контроля GPS
    last_gps_health = GPSHealth::OK;
    gps_last_fix_ms = 0;
    gps_monitor_start_ms = 0;
    gps_monitor_started = false;
    gps_prev_loc_valid = false;
    gps_last_spoof_ms = 0;
}



void Rocket::enter_stage(Stage s)
{
    stage.set((uint8_t)s);
    stage_enter_ms = AP_HAL::millis();
    if (debug_mask.get() & 1) {
        gcs().send_text(MAV_SEVERITY_INFO, "RKTN: stage=%s t=%lu",
                        stage_name(), (unsigned long)stage_enter_ms);
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "RKTN: stage=%s", stage_name());
    }
}

const char* Rocket::stage_name() const
{
    switch (get_stage()) {
    case Stage::WAIT_LAUNCH: return "WAIT_LAUNCH";
    case Stage::BOOST:       return "BOOST";
    case Stage::COAST:       return "COAST";
    case Stage::DONE:        return "DONE";
    }
    return "?";
}

void Rocket::reset_gps_monitor()
{
    gps_last_fix_ms = 0;
    gps_monitor_start_ms = AP_HAL::millis();
    gps_monitor_started = true;
    gps_prev_loc_valid = false;
    last_gps_health = GPSHealth::OK;
}

void Rocket::set_stage(Stage s)
{
    enter_stage(s);
}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
void Rocket::apply_sitl_stage_hold()
{
    if (sitl_stage_held() && get_stage() != (Stage)sitl_stage.get()) {
        enter_stage((Stage)sitl_stage.get());
    }
}
#endif

bool Rocket::detect_launch(const Vector3f &accel, float climb_rate_mps)
{
    if (launch_detected) {
        return true;
    }
    // Достаточно одного ускорения (GPS и данные о вертикальной скорости не нужны).
    // climb_rate может быть NAN: сравнение >= для NAN ложно,
    // поэтому старт в таком случае может обнаружить только акселерометр.
    if (accel.z >= launch_accel.get() || climb_rate_mps >= launch_climb.get()) {
        launch_detected = true;
        launch_ms = AP_HAL::millis();
        return true;
    }
    return false;
}

bool Rocket::detect_burnout(const Vector3f &accel, float climb_rate_mps) const
{
    if (get_stage() != Stage::BOOST) {
        return false;
    }
    // Таймер RKTN_BOOST_TIME не зависит от GPS и остаётся действующим резервом.
    if (AP_HAL::millis() - stage_enter_ms >= (uint32_t)(boost_time.get() * 1000.0f)) {
        return true;
    }
    if (accel.z <= burn_accel.get()) {
        return true;
    }
    // Определение выгорания по вертикальной скорости пропускается при NAN
    // (нет барометра и скорости EKF): иначе отсутствие данных датчика
    // ошибочно вызвало бы выгорание уже на первом цикле BOOST.
    if (!isnan(climb_rate_mps) && climb_rate_mps <= 0.0f) {
        return true;
    }
    return false;
}

bool Rocket::detect_coast_complete(float climb_rate_mps) const
{
    if (get_stage() != Stage::COAST) {
        return false;
    }
    // Без вертикальной скорости (например, при отказе барометра и GPS)
    // невозможно определить, продолжается ли набор высоты. Не переводим
    // аппарат в DONE по этому признаку. Предельный таймер COAST
    // независимо проверяется в Rocket::update().
    if (isnan(climb_rate_mps)) {
        return false;
    }
    return climb_rate_mps <= 5.0f;
}

/*
  Контроль качества GPS.

  Выполняется на каждом цикле в режиме ROCKET. Для управления в BOOST и
  COAST GPS не используется: применяется демпфирование по гироскопам.
  Оценка нужна для выбора режима после DONE и для поиска аппарата после
  посадки.

  Критерии:
    LOST     : нет 3D-fix дольше RKTN_GPS_LOST_MS; до первого fix отсчёт
               начинается при входе в ROCKET.
    SPOOFED  : скачок координат за один цикл превышает RKTN_GPS_JUMP_M,
               тогда как EKF оценивает существенно меньшее перемещение.
    DEGRADED : спутников меньше RKTN_GPS_MIN_SAT или HDOP выше
               RKTN_GPS_HDOP (характерно для помех).
    OK       : ни один из перечисленных критериев не сработал.
*/
Rocket::GPSHealth Rocket::gps_health()
{
    const AP_GPS &gps = AP::gps();
    const uint32_t now = AP_HAL::millis();

    // Запоминаем время последнего 3D-fix. До первого fix льготный
    // интервал отсчитывается от входа в ROCKET функцией
    // gps_lost_long_enough().
    if (gps.status() >= AP_GPS_FixType::FIX_3D) {
        gps_last_fix_ms = now;
    }

    GPSHealth h = GPSHealth::OK;

    if (gps_lost_long_enough()) {
        h = GPSHealth::LOST;
    } else if (gps.status() >= AP_GPS_FixType::FIX_3D) {
        // Fix получен: проверяем качество решения и признаки спуфинга.
        const uint8_t sats = gps.num_sats();
        // AP_GPS::get_hdop() возвращает HDOP, умноженный на 100 (155 = 1,55).
        const float hdop = gps.get_hdop() / 100.0f;

        // Проверка спуфинга/выброса: сопоставляем перемещение между
        // последовательными GPS-решениями с перемещением, рассчитанным
        // по скорости EKF. Подавленный или неисправный приёмник может
        // выдать скачок координат, не подтверждаемый инерциальной оценкой.
        const Location &loc = gps.location();
        if (gps_prev_loc_valid) {
            const float jump_m = gps_prev_loc.get_distance_NED(loc).length();
            float max_move_m = 0.0f;
            Vector3f vel_ned;
            if (AP::ahrs().get_velocity_NED(vel_ned)) {
                // Запас на один период обновления GPS (5 Гц -> 200 мс).
                max_move_m = vel_ned.length() * 0.25f + 30.0f;
            }
            if (jump_m > (float)gps_jump_m.get() && jump_m > max_move_m) {
                h = GPSHealth::SPOOFED;
            }
        }
        gps_prev_loc = loc;
        gps_prev_loc_valid = true;

        if (h == GPSHealth::OK) {
            if (sats < (uint8_t)gps_min_sat.get() || hdop > gps_max_hdop.get()) {
                h = GPSHealth::DEGRADED;
            }
        }
    }

    // Не допускаем повторной отправки одного уведомления GCS при переходе в SPOOFED.
    if (h == GPSHealth::SPOOFED && last_gps_health != GPSHealth::SPOOFED &&
        now - gps_last_spoof_ms > 5000) {
        gps_last_spoof_ms = now;
        if (debug_mask.get() & 1) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "RKTN: GNSS spoof suspected");
        }
    }
    last_gps_health = h;
    return h;
}

uint8_t Rocket::resolve_post_mode(uint8_t desired_mode_number) const
{
    if (!gps_protection_enabled()) {
        return desired_mode_number;
    }
    switch (last_gps_health) {
    case GPSHealth::OK:
    case GPSHealth::DEGRADED:
        // GPS пригоден: сохраняем запрошенный режим.
        return desired_mode_number;
    case GPSHealth::SPOOFED:
    case GPSHealth::LOST:
        // Источник координат отсутствует или недостоверен: выбираем резервный режим.
        gcs().send_text(MAV_SEVERITY_WARNING, "RKTN: GPS %s, fallback mode=%u",
                        last_gps_health == GPSHealth::LOST ? "LOST" : "SPOOFED",
                        (unsigned)gps_fallback.get());
        return (uint8_t)gps_fallback.get();
    }
    return desired_mode_number;
}

float Rocket::damp(float gyro_dps, float gain, float gmax)
{
    if (gmax <= 0.0f) {
        return 0.0f;
    }
    const float x = constrain_float(gyro_dps / gmax, -1.0f, 1.0f);
    return constrain_float(x * gain, -1.0f, 1.0f);
}

void Rocket::compute_outputs(float &fin_lu, float &fin_ru, float &fin_ld, float &fin_rd)
{
    fin_lu = fin_ru = fin_ld = fin_rd = 0.0f;
    if (!enabled()) {
        return;
    }

    const AP_AHRS &ahrs = AP::ahrs();
    Vector3f gyro = ahrs.get_gyro();
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (sitl_stage_held()) {
        gyro = Vector3f(sitl_gyro_x.get(), sitl_gyro_y.get(), sitl_gyro_z.get());
    }
#endif

    switch (get_stage()) {
    case Stage::WAIT_LAUNCH:
        fin_lu = fin_ru = fin_ld = fin_rd = 0.0f;
        break;

    case Stage::BOOST: {
        const float g  = boost_damp.get();
        const float gm = boost_gmax.get();
        const float damp_roll  = damp( gyro.x, g, gm) + boost_thr.get();
        const float damp_pitch = damp( gyro.y, g, gm) + boost_thr.get();
        const float damp_yaw   = damp(-gyro.z, g, gm) + boost_thr.get();
        if (tail_mode() == TailMode::X45) {
            // Mix independent rate corrections before limiting each fin.
            // Averaging the axes into one command loses attitude control.
            mix_x45(damp_roll, damp_pitch, damp_yaw,
                    fin_lu, fin_ru, fin_ld, fin_rd);
        } else {
            // Keep the legacy CROSS BOOST command unchanged.
            const float average = (damp_roll + damp_pitch + damp_yaw) / 3.0f;
            fin_lu = fin_ru = fin_ld = fin_rd = average;
        }
        const float lower = constrain_float(boost_min.get(), -1.0f, 0.0f);
        const float upper = constrain_float(boost_max.get(), 0.0f, 1.0f);
        fin_lu = constrain_float(fin_lu, lower, upper);
        fin_ru = constrain_float(fin_ru, lower, upper);
        fin_ld = constrain_float(fin_ld, lower, upper);
        fin_rd = constrain_float(fin_rd, lower, upper);
        break;
    }

    case Stage::COAST: {
        const float g  = coast_damp.get();
        const float gm = coast_gmax.get();
        const float damp_roll  = damp( gyro.x, g, gm) + coast_thr.get();
        const float damp_pitch = damp( gyro.y, g, gm) + coast_thr.get();
        const float damp_yaw   = damp(-gyro.z, g, gm) + coast_thr.get();
        if (tail_mode() == TailMode::X45) {
            mix_x45(damp_roll, damp_pitch, damp_yaw,
                    fin_lu, fin_ru, fin_ld, fin_rd);
        } else {
            const float pitch = constrain_float(damp_pitch, -1.0f, 1.0f);
            const float yaw   = constrain_float(damp_yaw,   -1.0f, 1.0f);
            fin_lu = fin_ru = pitch;
            fin_ld = fin_rd = yaw;
        }
        const float lower = constrain_float(coast_min.get(), -1.0f, 0.0f);
        const float upper = constrain_float(coast_max.get(), 0.0f, 1.0f);
        fin_lu = constrain_float(fin_lu, lower, upper);
        fin_ru = constrain_float(fin_ru, lower, upper);
        fin_ld = constrain_float(fin_ld, lower, upper);
        fin_rd = constrain_float(fin_rd, lower, upper);
        break;
    }

    case Stage::DONE:
        break;
    }
}

void Rocket::mix_x45(float aileron_in, float elevator_in, float rudder_in,
                     float &fin_lu, float &fin_ru, float &fin_ld, float &fin_rd) const
{
    // Матрица смешения для схемы X45:
    //   LU = +крен - тангаж - рыскание
    //   RU = -крен - тангаж + рыскание
    //   LD = -крен + тангаж + рыскание
    //   RD = +крен + тангаж - рыскание
    const float g = xwing_gain.get();
    fin_lu = ( aileron_in - elevator_in - rudder_in) * g;
    fin_ru = (-aileron_in - elevator_in + rudder_in) * g;
    fin_ld = (-aileron_in + elevator_in + rudder_in) * g;
    fin_rd = ( aileron_in + elevator_in - rudder_in) * g;
    fin_lu = constrain_float(fin_lu, -1.0f, 1.0f);
    fin_ru = constrain_float(fin_ru, -1.0f, 1.0f);
    fin_ld = constrain_float(fin_ld, -1.0f, 1.0f);
    fin_rd = constrain_float(fin_rd, -1.0f, 1.0f);
}

void Rocket::update()
{
    const uint32_t now = AP_HAL::millis();
    if (armed_ms == 0) {
        armed_ms = now;
    }

    // Качество GPS оценивается на каждом цикле, пока активен ROCKET.
    // Оно не влияет на демпфирование в BOOST/COAST — только на выбор
    // режима после завершения последовательности (DONE).
    if (gps_protection_enabled()) {
        gps_health();
    }

    // Предельные длительности стадий, не зависящие от GPS. При отсутствии
    // барометрической вертикальной скорости переходы по набору/снижению
    // высоты не сработают; таймеры гарантируют завершение последовательности
    // и возврат управления пилоту. Значения RKTN_BST_HARD / RKTN_CST_HARD
    // настраиваются для конкретного летательного аппарата.
    // SITL test hold must not be advanced by the independent stage timers.
    if (sitl_stage_held()) {
        return;
    }
    switch (get_stage()) {
    case Stage::BOOST:
        if (now - stage_enter_ms >= (uint32_t)boost_hardtime.get()) {
            set_stage(Stage::COAST);
        }
        break;
    case Stage::COAST:
        if (now - stage_enter_ms >= (uint32_t)coast_hardtime.get()) {
            set_stage(Stage::DONE);
        }
        break;
    default:
        break;
    }
}
