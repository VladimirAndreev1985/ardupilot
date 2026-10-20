/*
   Rocket flight support for ArduPlane.
   Self-contained: declares its own parameter group and runtime state,
   without touching ArduPlane core structs. This keeps merge conflicts
   with upstream ArduPilot to a minimum.

   All rocket-specific parameters are prefixed RKTN_.
*/

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/Location.h>

class Rocket {
public:
    // Текущий этап последовательности запуска ракеты.
    enum class Stage : uint8_t {
        WAIT_LAUNCH = 0,   // ожидание взведения и обнаружения старта
        BOOST       = 1,   // работа маршевого двигателя
        COAST       = 2,   // двигатель выключен, аппарат продолжает набор высоты
        DONE        = 3,   // передача управления в следующий режим
    };

    // Геометрия хвостового оперения
    enum class TailMode : uint8_t {
        CROSS = 0,   // два элевона и два руля направления (штатное смешение ArduPilot)
        X45   = 1,   // четыре руля под углом 45° (специализированное смешение)
    };

    // Целевые режимы после этапа DONE
    enum class PostMode : uint8_t {
        MANUAL = 0,
        FBWA   = 1,
        AUTO   = 2,
        CIRCLE = 3,
    };

    // Результат оценки качества GPS. GPS требуется только после перехода
    // COAST–DONE — при передаче управления в послеполётный режим самолёта.
    // Для управления в BOOST/COAST GPS не используется, поэтому снижение
    // качества GPS не влияет на стабилизацию ориентации.
    enum class GPSHealth : uint8_t {
        OK          = 0,   // HDOP, число спутников и скорость согласуются с порогами
        DEGRADED    = 1,   // отдельные критерии нарушены, критической потери нет
        SPOOFED     = 2,   // скачок координат или нарушение контроля достоверности
        LOST        = 3,   // нет 3D-fix в течение RKTN_GPS_LOST_MS
    };

    // Конструктор подключает группу параметров AP_Param.
    Rocket();

    // Периодическое обновление; вызывается из ModeRocket::update().
    void update();

    // Методы установки и чтения текущего этапа.
    void set_stage(Stage s);
    Stage get_stage() const { return (Stage)stage.get(); }
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // Test-only stage hold. Never present on flight hardware.
    bool sitl_stage_held() const { return sitl_stage.get() >= 0 && sitl_stage.get() <= 2; }
    void apply_sitl_stage_hold();
#else
    bool sitl_stage_held() const { return false; }
#endif
    const char* stage_name() const;

    // Методы доступа к параметрам.
    bool         enabled() const     { return enable.get() != 0; }
    TailMode     tail_mode() const   { return (TailMode)tail_mode_p.get(); }
    PostMode     post_mode() const   { return (PostMode)post_mode_p.get(); }

    // Проверка датчиков и условий перехода между этапами.
    bool detect_launch(const Vector3f &accel, float climb_rate_mps);
    bool detect_burnout(const Vector3f &accel, float climb_rate_mps) const;
    bool detect_coast_complete(float climb_rate_mps) const;

    // Оценивает качество GPS и возвращает одно из значений GPSHealth.
    // Использует RKTN_GPS_PROTECT, RKTN_GPS_MIN_SAT, RKTN_GPS_HDOP,
    // RKTN_GPS_LOST_MS и RKTN_GPS_JUMP_M. update() также обновляет
    // внутреннее состояние и наихудшую зафиксированную оценку GPS.
    GPSHealth gps_health();
    bool      gps_protection_enabled() const { return gps_protect.get() != 0; }
    // Запускает льготный период отсутствия fix при входе в ROCKET.
    void      reset_gps_monitor();

    bool      gps_lost_long_enough() const {
        if (gps_last_fix_ms != 0) {
            return ms_since_last_fix() >= (uint32_t)gps_lost_ms.get();
        }
        return gps_monitor_started &&
               AP_HAL::millis() - gps_monitor_start_ms >= (uint32_t)gps_lost_ms.get();
    }
    bool      gps_spoof_detected() const  { return last_gps_health == GPSHealth::SPOOFED; }
    uint32_t  ms_since_last_fix() const   { return gps_last_fix_ms == 0 ? 0u : (AP_HAL::millis() - gps_last_fix_ms); }

    // Выбирает безопасный режим после DONE с учётом текущего качества GPS.
    // Принимает номер целевого режима ArduPlane (RKTN_POST_MODE) и возвращает
    // его либо номер резервного режима RKTN_GPS_FBK при плохом GPS.
    // Используется uint8_t, а не Mode::Number, чтобы Rocket.h не зависел
    // от mode.h и оставался проще для сопровождения при слиянии с upstream.
    uint8_t resolve_post_mode(uint8_t desired_mode_number) const;

    // Сырые значения параметров выбора режима, для проверок перед вооружением.
    // post_mode() приводит значение к PostMode и скрывает выход за диапазон,
    // поэтому проверка читает исходное значение.
    int8_t  post_mode_raw() const     { return post_mode_p.get(); }
    uint8_t gps_fallback_mode() const { return (uint8_t)gps_fallback.get(); }

    // Формирует команды четырём рулям в нормированном диапазоне от −1 до +1.
    // На вход подаются нормированные команды элеронов, руля высоты и руля направления (от −1 до +1).
    // Используется только при tail_mode() == X45.
    void mix_x45(float aileron_in, float elevator_in, float rudder_in,
                 float &fin_lu, float &fin_ru, float &fin_ld, float &fin_rd) const;

    // Вычисляет команду демпфирования по одной оси на основе
    // текущей угловой скорости гироскопа (град/с), коэффициента
    // RKTN_*_DAMP и предельной скорости RKTN_*_GMAX, после которой команда ограничивается.
    static float damp(float gyro_dps, float gain, float gmax);

    // Допустимый верхний предел жёстких таймеров стадий (ограничение
    // для проверки вводимого параметра). Рабочие значения задаются
    // параметрами RKTN_BST_HARD и RKTN_CST_HARD для конкретного аппарата.
    static constexpr uint32_t BOOST_HARD_TIMEOUT_MS_MIN = 1000;
    static constexpr uint32_t BOOST_HARD_TIMEOUT_MS_MAX = 60000;
    static constexpr uint32_t COAST_HARD_TIMEOUT_MS_MIN = 5000;
    static constexpr uint32_t COAST_HARD_TIMEOUT_MS_MAX = 180000;

    // Открытая таблица параметров и метод выдачи команд сервоприводам Plane.
    static const struct AP_Param::GroupInfo var_info[];
    void compute_outputs(float &fin_lu, float &fin_ru, float &fin_ld, float &fin_rd);

    // Снимок входов и выходов последнего расчёта стабилизации, для записи в
    // лог. Заполняется в compute_outputs() на каждом шаге управления, а
    // читается при записи лога на 25 Гц, поэтому расчёт не дублируется.
    struct LogState {
        float accel_x;      // осевое ускорение (ось X корпуса), м/с^2
        float climb_rate;   // вертикальная скорость, м/с
        float gyro_x;       // угловые скорости, град/с
        float gyro_y;
        float gyro_z;
        float fin_lu;       // команды рулям, от -1 до +1
        float fin_ru;
        float fin_ld;
        float fin_rd;
    };
    const LogState &log_state() const { return logged; }

    // Сохраняет входные данные датчиков для сообщения RKTN. Вызывается из
    // ModeRocket::update(), где они уже вычислены; compute_outputs() к ним
    // доступа не имеет. climb_rate может быть NAN при отсутствии оценки.
    void log_sensors(float accel_x, float climb_rate) {
        logged.accel_x = accel_x;
        logged.climb_rate = climb_rate;
    }
    GPSHealth last_health() const { return last_gps_health; }

private:
    // --- параметры (RKTN_*) ---
    AP_Int8   enable;          // RKTN_ENABLE
    AP_Int8   tail_mode_p;     // RKTN_TAIL_MODE
    AP_Float  boost_time;      // RKTN_BOOST_TIME, s
    AP_Float  launch_accel;    // RKTN_LCH_ACC, m/s^2
    AP_Float  launch_climb;    // RKTN_LCH_CLIMB, m/s
    AP_Float  burn_accel;       // RKTN_BURN_ACC, m/s^2
    AP_Float  boost_damp;      // RKTN_BOOST_DAMP
    AP_Float  coast_damp;      // RKTN_COAST_DAMP
    AP_Float  boost_gmax;      // RKTN_BOOST_GMAX, deg/s
    AP_Float  coast_gmax;      // RKTN_COAST_GMAX, deg/s
    AP_Float  boost_thr;       // RKTN_BOOST_THR, normalized -1..+1
    AP_Float  coast_thr;       // RKTN_COAST_THR
    AP_Float  boost_min;       // RKTN_BOOST_MIN, lower per-fin BOOST limit
    AP_Float  boost_max;       // RKTN_BOOST_MAX, upper per-fin BOOST limit
    AP_Float  coast_min;       // RKTN_CST_MIN, lower per-fin COAST limit
    AP_Float  coast_max;       // RKTN_CST_MAX, upper per-fin COAST limit
    AP_Float  xwing_gain;      // RKTN_XWING_GAIN
    AP_Int8   post_mode_p;     // RKTN_POST_MODE
    AP_Int8   debug_mask;      // RKTN_DBG

    // --- GPS protection parameters (RKTN_GPS_*) ---
    AP_Int8   gps_protect;     // RKTN_GPS_PROTECT     (0=off, 1=on)
    AP_Int32  gps_lost_ms;     // RKTN_GPS_LOST_MS     (ms)
    AP_Int16  gps_jump_m;      // RKTN_GPS_JUMP_M      (metres)
    AP_Int8   gps_min_sat;     // RKTN_GPS_MIN_SAT
    AP_Float  gps_max_hdop;    // RKTN_GPS_HDOP
    AP_Int8   gps_fallback;    // RKTN_GPS_FBK    (4=CIRCLE+NoGPS, 5=CIRCLE, 6=RTL..)

    // --- Предельные таймеры этапов (не зависят от GPS) ---
    AP_Int32  boost_hardtime;  // RKTN_BST_HARD (ms)
    AP_Int32  coast_hardtime;  // RKTN_CST_HARD (ms)

    // --- Текущее состояние (не параметры) ---
    AP_Int8   stage;           // текущее значение перечисления Stage
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    AP_Int8   sitl_stage;      // RKTN_TST_STAGE, SITL-only (-1=off)
    AP_Float  sitl_gyro_x;
    AP_Float  sitl_gyro_y;
    AP_Float  sitl_gyro_z;
#endif
    uint32_t  stage_enter_ms;
    uint32_t  armed_ms;
    uint32_t  launch_ms;
    uint32_t  burnout_ms;
    bool      launch_detected;

    // --- Текущее состояние контроля GPS ---
    GPSHealth last_gps_health;
    uint32_t  gps_last_fix_ms;    // время последнего 3D-fix по millis() (0 — fix ещё не было)
    uint32_t  gps_monitor_start_ms; // время начала контроля GPS в режиме ROCKET по millis()
    bool      gps_monitor_started;
    Location  gps_prev_loc;       // последнее принятое GPS-решение (для обнаружения скачка координат)
    bool      gps_prev_loc_valid;
    uint32_t  gps_last_spoof_ms;  // время последнего уведомления о спуфинге (защита от дублирования GCS)

    // Переводит аппарат на новый этап и фиксирует время перехода
    void enter_stage(Stage s);

    LogState  logged;

};
