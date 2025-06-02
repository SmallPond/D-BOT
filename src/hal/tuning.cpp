#include "hal.h"
#include "wireless_tuning.h"
#include "tuning.h"
#include "nvs.h"

WirelessTuning wl_tuning = WirelessTuning(WIRELESS_TUNING_PORT);

//  ------------monitor--------------------
#ifdef XK_WIRELESS_PARAMETER
Commander commander = Commander(wl_tuning, '\n', false);
#else 
Commander commander = Commander(Serial, '\n', false);
#endif

void on_stb_pid(char* cmd)
{
    commander.pid(&pid_stb, cmd);
}
void on_vel_pid(char* cmd)
{
    commander.pid(&pid_vel, cmd);
    commander.pid(&pid_vel_tmp, cmd);
}
void on_str_pid(char* cmd) 
{
    commander.pid(&pid_steering, cmd);
}
void on_bot_s_pid(char* cmd)
{
    commander.pid(&pid_bot_s, cmd);
}
void on_bot_m_pid(char* cmd)
{
    commander.pid(&pid_bot_m, cmd);
}

void on_imu_offset(char *cmd)
{
    commander.scalar(&g_zero_angle, cmd);
    log_i("imu offset change to %.2f\n", g_zero_angle);
}

void on_motor(char* cmd)
{
    commander.motor(&motor_0, cmd);
    commander.motor(&motor_1, cmd);
}

static void on_store_parameter(char* cmd) 
{
    nvs_set_pid_config(PID_S_CONFIG, pid_stb);
    nvs_set_pid_config(PID_V_CONFIG, pid_vel);
    nvs_set_pid_config(PID_T_CONFIG, pid_steering);
    nvs_set_pid_config(PID_R_CONFIG, pid_bot_m);
    nvs_set_pid_config(PID_B_CONFIG, pid_bot_s);
    nvs_set_float(NVS_D_BOT, MACHINE_MID_VALUE_KEY, g_zero_angle);
    HAL::audio_play_music("BattChargeStart");
}


int ctrl_restore_parameter(void)
{
    int rc = 0;
    if (nvs_get_pid_config(PID_S_CONFIG, pid_stb) 
        || nvs_get_pid_config(PID_V_CONFIG, pid_vel) 
        || nvs_get_pid_config(PID_T_CONFIG, pid_steering)
        || nvs_get_pid_config(PID_R_CONFIG, pid_bot_m)
        || nvs_get_pid_config(PID_B_CONFIG, pid_bot_s)
        || nvs_get_float(NVS_D_BOT, MACHINE_MID_VALUE_KEY, g_zero_angle)) {
        return -1;
    }
    pid_vel_tmp=pid_vel;
    return 0;
}
void wl_tuning_task(void *pvParameters)
{
    while (1) {
        commander.run();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

TaskHandle_t handle_tuning_task;
int HAL::wireless_tuning_init(void)
{
    int ret = 0;
#ifdef XK_WIRELESS_PARAMETER
    if(!wl_tuning.begin(HAL::get_wifi_ssid().c_str(), HAL::get_wifi_passwd().c_str())) {
        log_system(SYSTEM_ERR, "setup WiFi failed!");
        return -1;
    } else {
        log_system(SYSTEM_INFO, "ip %s", WiFi.localIP().toString().c_str());
    }
#endif
    
    add_tuning_cmd('M', on_motor, const_cast<char*>("my motor"));
    add_tuning_cmd('S', on_stb_pid, const_cast<char*>("PID stable"));
    add_tuning_cmd('V', on_vel_pid, const_cast<char*>("PID vel"));
    add_tuning_cmd('T', on_str_pid, const_cast<char*>("PID str"));
    add_tuning_cmd('B', on_bot_s_pid, const_cast<char*>("PID bot spin"));
    add_tuning_cmd('R', on_bot_m_pid, const_cast<char*>("PID bot move"));
    add_tuning_cmd('X', on_imu_offset, const_cast<char*>("imu offset"));
    add_tuning_cmd('C', on_store_parameter, 
                const_cast<char*>("store all wirelss tuning parameter"));

    // log_system(SYSTEM_INFO, "getting parameter from nvs...");
    if (ctrl_restore_parameter()) {
        log_system(SYSTEM_ERR, "using default pid parameter.");
    }
    ret = xTaskCreatePinnedToCore(
                wl_tuning_task,
                "tuning_stask",
                4096,
                nullptr,
                2,
                &handle_tuning_task,
                ESP32_RUNNING_CORE);

    return ret;
}

WirelessTuning &HAL::get_wl_tuning(void)
{
    return wl_tuning;
}

int HAL::add_tuning_cmd(char id, cmd_cb cb, char* label)
{
    commander.add(id, cb, label);
    return 0;
}