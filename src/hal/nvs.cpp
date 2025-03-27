#include "nvs.h"
#include "config.h"

#define MQTT_SERVER "10.0.0.2"
#define MQTT_PORT 1883
#define MQTT_USER "mqttuser"
#define MQTT_PASSWORD "mqttpassword"

#define HOST               "dingmos"         // owner 
#define MQTT_COMMAND_TOPIC "home"

#define INIT_VALUE 0xdb

NvsConfig ncv_config;

void nvs_init(void)
{
    Preferences prefs;     
    prefs.begin(CONFIG_NAMESPACE);
    uint8_t value = prefs.getUChar(INIT_KEY, 0);
    
    prefs.putUChar(FFAT_KEY, 0);
    set_lcd_bk_brightness(LCD_BK_DEFAULT_BRIGHTNESS); //default lcd bk
    set_lcd_bk_timeout(LCD_BK_DEFAULT_TIMEOUT);       //default lcd bk time out 5mins
    set_mqtt_config(MQTT_SERVER, MQTT_PORT, MQTT_USER, MQTT_PASSWORD, MQTT_HOST );
    set_init_ffat(0);
    prefs.putUChar(INIT_KEY, INIT_VALUE);

    ncv_config.init_ffat_flag = prefs.getUChar(FFAT_KEY, 0);
    //lcd
    ncv_config.lcd_bk_timeout =  prefs.getUShort(LCD_BK_TIME_OUT_KEY, 0);
    ncv_config.lcd_bk_brightness =  prefs.getUShort(LCD_BK_BRIGHTNESS_KEY, 0);

    //mqtt
    ncv_config.mqtt_host = prefs.getString(MQTT_HOST_KEY, "");
    ncv_config.mqtt_port = prefs.getUInt(MQTT_PORT_KEY, 0);
    ncv_config.mqtt_username = prefs.getString(MQTT_USERNAME_KEY, "");
    ncv_config.mqtt_password = prefs.getString(MQTT_PASSWORD_KEY, "");
    ncv_config.mqtt_topic = prefs.getString(MQTT_TPOIC_KEY, "");
    
    log_d("get FFAT_KEY Config: %d\n", ncv_config.init_ffat_flag );
    log_d("get LCD_BK_TIME_OUT_KEY Config: %d\n", ncv_config.lcd_bk_timeout );
    log_d("get LCD_BK_BRIGHTNESS_KEY Config: %d\n", ncv_config.lcd_bk_brightness );
    log_d("get WIFI_SSID_KEY Config: %s\n", ncv_config.wifi_ssid );
    log_d("get WIFI_PASSWORD_KEY Config: %s\n", ncv_config.wifi_password );
    log_d("get MQTT_HOST_KEY Config: %s\n", ncv_config.mqtt_host );
    log_d("get MQTT_PORT_KEY Config: %d\n", ncv_config.mqtt_port );
    log_d("get MQTT_USERNAME_KEY Config: %s\n", ncv_config.mqtt_username );
    log_d("get MQTT_PASSWORD_KEY Config: %s\n", ncv_config.mqtt_password );
    log_d("get MQTT_TPOIC_KEY Config: %s\n", ncv_config.mqtt_topic );

    prefs.end();
}

uint8_t get_init_ffat(void)
{
    return ncv_config.init_ffat_flag;
}

uint16_t get_lcd_bk_brightness(void)
{
    return ncv_config.lcd_bk_brightness;
}

uint16_t get_lcd_bk_timeout(void)
{
    return ncv_config.lcd_bk_timeout;
}

void get_mqtt_config(String &host,uint16_t &port,String &username,
                            String &password,String &topic)
{
    host =  ncv_config.mqtt_host;
    port = ncv_config.mqtt_port;
    username = ncv_config.mqtt_username;
    password = ncv_config.mqtt_password;
    topic = ncv_config.mqtt_topic;

    log_d("get MQTT_HOST_KEY Config: %s\n", ncv_config.mqtt_host.c_str() );
    log_d("get MQTT_PORT_KEY Config: %d\n", ncv_config.mqtt_port );
    log_d("get MQTT_USERNAME_KEY Config: %s\n", ncv_config.mqtt_username.c_str() );
    log_d("get MQTT_PASSWORD_KEY Config: %s\n", ncv_config.mqtt_password.c_str() );
    log_d("get MQTT_TPOIC_KEY Config: %s\n", ncv_config.mqtt_topic.c_str() );
}

void set_init_ffat(uint8_t value)
{
    Preferences prefs; 
    prefs.begin(CONFIG_NAMESPACE);  
    prefs.putUChar(FFAT_KEY, value);
    prefs.end();
    ncv_config.init_ffat_flag = value;
    log_d("set FFAT_KEY Config: %d\n", value );
}

void set_lcd_bk_brightness(uint16_t value)
{
    Preferences prefs;     
    prefs.begin(CONFIG_NAMESPACE); 
    prefs.putUShort(LCD_BK_BRIGHTNESS_KEY, value);
    prefs.end();
    ncv_config.lcd_bk_brightness = value;

    log_d("set LCD_BK_BRIGHTNESS_KEY Config: %d\n", value );
}

void set_lcd_bk_timeout(uint16_t value)
{
    Preferences prefs;     
    prefs.begin(CONFIG_NAMESPACE); 
    prefs.putUShort(LCD_BK_TIME_OUT_KEY, value);
    prefs.end();
    ncv_config.lcd_bk_timeout = value;
    log_d("set LCD_BK_TIME_OUT_KEY Config: %d\n", value );
}


void set_mqtt_config(String host, uint16_t port, String username,
                        String password,String topic)
{
    Preferences prefs;     
    prefs.begin(CONFIG_NAMESPACE); 
    prefs.putString(MQTT_HOST_KEY, host);
    prefs.putUInt(MQTT_PORT_KEY, port);
    prefs.putString(MQTT_USERNAME_KEY, username);
    prefs.putString(MQTT_PASSWORD_KEY, password);
    prefs.putString(MQTT_TPOIC_KEY, topic);
    prefs.end();
    ncv_config.mqtt_host = host;
    ncv_config.mqtt_port = port;
    ncv_config.mqtt_username = username;
    ncv_config.mqtt_password = password;
    ncv_config.mqtt_topic = topic;

    log_d("set MQTT_HOST_KEY Config: %s\n", host.c_str() );
    log_d("set MQTT_PORT_KEY Config: %d\n", port );
    log_d("set MQTT_USERNAME_KEY Config: %s\n", username.c_str() );
    log_d("set MQTT_PASSWORD_KEY Config: %s\n", password.c_str() );
    log_d("set MQTT_TPOIC_KEY Config: %s\n", topic.c_str() );
}

void set_imu_config(float gyroXoffset, float gyroYoffset, float gyroZoffset)
{
    Preferences prefs;     
    prefs.begin(IMU_CONFIG); 
    prefs.putFloat(IMU_GYRO_X_OFFSET, gyroXoffset);
    prefs.putFloat(IMU_GYRO_Y_OFFSET, gyroYoffset);
    prefs.putFloat(IMU_GYRO_Z_OFFSET, gyroZoffset);
    prefs.end();
}

int get_imu_offset(struct imu_offset* offset) 
{
    Preferences preferences;
    int rc = 0;

    if (preferences.begin(IMU_CONFIG) == false) {
        return -1;
    }

    offset->xoffset = preferences.getFloat(IMU_GYRO_X_OFFSET, 0.0);
    offset->yoffset = preferences.getFloat(IMU_GYRO_Y_OFFSET, 0.0);
    offset->zoffset = preferences.getFloat(IMU_GYRO_Z_OFFSET, 0.0);
    preferences.end();

    return 0;
}

void nvs_set_motor_config(float motor_l_offset, float motor_r_offset)
{
    Preferences prefs;
    prefs.begin(MOTOR_CONFIG); 
    prefs.putFloat(MOTOR_LEFT_OFFSET, motor_l_offset);
    prefs.putFloat(MOTOR_RIGHT_OFFSET, motor_r_offset);
    prefs.end();
}

int nvs_get_motor_offset(struct motor_offset *offset) 
{
    Preferences preferences;
    if (preferences.begin(MOTOR_CONFIG) == false) {
        return -1;
    }
    offset->l_offset = preferences.getFloat(MOTOR_LEFT_OFFSET, 0.0);
    offset->r_offset = preferences.getFloat(MOTOR_RIGHT_OFFSET, 0.0);
    preferences.end();

    return 0;
}

int nvs_set_pid_config(const char *name, PIDController pid)
{
    Preferences prefs;
    prefs.begin(name); 
    prefs.putFloat(PID_P_KEY, pid.P);
    prefs.putFloat(PID_I_KEY, pid.I);
    prefs.putFloat(PID_D_KEY, pid.D);
    prefs.putFloat(PID_RAMP_KEY, pid.output_ramp);
    prefs.putFloat(PID_LIMIT_KEY, pid.limit);
    prefs.end();
    return 0;
}

int nvs_get_pid_config(const char *name, PIDController &pid) 
{
    Preferences prefs;
    if (prefs.begin(name, true) == false) {
        return -1;
    }

    pid.P = prefs.getFloat(PID_P_KEY, pid.P);
    pid.I = prefs.getFloat(PID_I_KEY, pid.I);
    pid.D = prefs.getFloat(PID_D_KEY, pid.D);
    pid.output_ramp = prefs.getFloat(PID_RAMP_KEY, pid.output_ramp);
    pid.limit = prefs.getFloat(PID_LIMIT_KEY, pid.limit);
    log_i("%s P: %.3f I: %.3f D: %.3f, ramp: %.3f, limit: %.3f", 
            name, pid.P, pid.I, pid.D, pid.output_ramp, pid.limit);
    prefs.end();

    return 0;
}

int nvs_set_float(const char *name, const char *key, float vaule)
{
    Preferences prefs;
    prefs.begin(name); 
    prefs.putFloat(key, vaule);
    prefs.end();
    return 0;
}

int nvs_get_float(const char *name, const char *key, float &vaule)
{
    Preferences prefs;
    prefs.begin(name); 
    vaule = prefs.getFloat(key, vaule);
    prefs.end();
    return 0;
}