#ifndef __HAL_H__
#define __HAL_H__
#include "button_event.h"
#include "HAL_Def.h"
#include <Arduino.h>
#include "config.h"
#include "CommonMacro.h"
#include "wireless_tuning.h"

typedef enum {
    SUPER_DIAL_NULL = 0,
    SUPER_DIAL_LEFT = 1,
    SUPER_DIAL_RIGHT= 2,
} SuperDialMotion;

typedef enum {
    HASS_LEFT = 1,
    HASS_RIGHT= 2,
    HASS_PUSH = 3,
    HASS_MAX,
} HassMotion;

#define SYSTEM_ERR     0
#define SYSTEM_WARN    1
#define SYSTEM_INFO    2
#define SYSTEM_DEBUG   3

#define KNOB_MOTOR_NUM    0
#define ENCODER_MOTOR_NUM 1

enum BOT_RUNNING_MODE {
    BOT_RUNNING_MODE,
    BOT_RUNNING_XKNOB,
    BOT_RUNNING_BALANCE,
};

typedef enum
{
    MOTOR_UNBOUND_FINE_DETENTS,        // Fine values\nWith detents
    MOTOR_UNBOUND_NO_DETENTS,
    MOTOR_SUPER_DIAL, 
    MOTOR_UNBOUND_COARSE_DETENTS, // Coarse values\nStrong detents\n unbound
    MOTOR_BOUND_0_12_NO_DETENTS,
    MOTOR_BOUND_LCD_BK_BRIGHTNESS,
    MOTOR_BOUND_LCD_BK_TIMEOUT,
    MOTOR_COARSE_DETENTS,       // Coarse values\nStrong detents
    MOTOR_FINE_NO_DETENTS,     // Fine values\nNo detents
    MOTOR_ON_OFF_STRONG_DETENTS,             // "On/off\nStrong detent"
    MOTOR_RETURN_TO_CENTER,
    MOTOR_MAX_MODES, //

} MOTOR_RUNNING_MODE_E;

#ifdef D_BOT_HW_V1

#define MOTOR_MAX_SPEED  25
/** using gyro_z */
// #define BOT_MAX_STEERING 500 
/** no feedback */
#define BOT_MAX_STEERING 60

#else
#define MOTOR_MAX_SPEED  15
/** using gyro_z */
// #define BOT_MAX_STEERING 500 
/** no feedback */
#define BOT_MAX_STEERING 60
#endif

#ifdef XK_WIRELESS_PARAMETER
extern WirelessTuning wireless;
#endif

typedef int (*wl_parm_cb)(char *);

extern bool g_system_calibration;

extern TaskHandle_t handleTaskMotor;
extern TaskHandle_t handleTaskIMU;


/*
 * Light mode enumeration
 *
 * @MODE_OFF: All lights turned off
 * @MODE_STANDBY: Vehicle in standby/balanced state
 * @MODE_DRIVING: Vehicle in normal operation
 * @MODE_BRAKING: Vehicle braking
 * @MODE_TURNING_LEFT: Vehicle turning left
 * @MODE_TURNING_RIGHT: Vehicle turning right
 */
enum light_mode {
	MODE_OFF,
	MODE_STANDBY,
	MODE_DRIVING,
	MODE_BRAKING,
	MODE_TURNING_LEFT,
	MODE_TURNING_RIGHT,
};

typedef void (* cmd_cb)(char*);
namespace HAL
{

    void Init();
    void Update();

    void knob_init();
    void knob_update(void);
    bool encoder_is_pushed(void);

    void motor_init(void);
    // void motor_task(void *pvParameters);
    int get_motor_position(int id);
    void update_motor_mode(int id, int mode , int init_position);
    void motor_shake(int id, int strength, int delay_time);
    void motor_set_speed(float speed, float steering);
    double motor_get_cur_angle(void);
    double get_motor_angle_offset(int id);

    void surface_dial_init(void);
    void surface_dial_update(SuperDialMotion direction);
    void surface_dial_release(void);
    void surface_dial_press(void);
    bool surface_dial_is_connected(void);

    void power_init(void);
    void power_off(void);

    bool is_encoder_enabled(void);
    void encoder_disable(void);
    void encoder_enable(void);

    void mqtt_init(void);
    int mqtt_publish(const char *topic, const char *playload);
    int mqtt_subscribe(const char *topic);
    int setup_wifi(void);
    const char *mqtt_get_topic_frefix(void);

    void lcd_brightness_init();
    void set_lcd_brightness(int value);

    void buzz_set_enable(bool en);
    void buzz_init(void);
    void buzz_tone(uint32_t _freq, int32_t _duration = 0);
    void audio_init(void);
    void audio_update(void);
    bool audio_play_music(const char* name);

    void system_led_init(void);
    void system_led_run(unsigned long currentMillis);
    int system_init(void);
    bool system_is_network_config(void);

    void  imu_init(void);
    void  imu_update(void *pvParameters);
    float imu_get_abs_yaw(void);
    float imu_get_pitch(void);
    float imu_get_yaw(void);
    float imu_get_gyro_z(void);
    float imu_get_gyro_y(void);

    int wireless_param_init(wl_parm_cb cb);

    int system_ui_init(void);
    void log_system(int level, const char *format, ...);
    int system_ui_uninit();

    int network_init(void);
    std::string get_wifi_ssid(void);
    std::string get_wifi_passwd(void);
    void network_update(void *pvParameters);
    bool is_network_ready(void);

    int wireless_tuning_init(void);
    WirelessTuning &get_wl_tuning(void);
    int add_tuning_cmd(char id, cmd_cb cb, char* label);

    void rgb_update(void);
    void rgb_init(void);
    void rgb_set_mode(enum light_mode new_mode);
    void rgb_set_mode_by_status(float speed, float steering);
}


#endif