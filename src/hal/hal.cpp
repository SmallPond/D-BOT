
#include "hal.h"
#include "lvgl.h"
#include "sd_card.h"
#include "app/ota/ota.h"

#define DISP_BUF_SIZE        CONFIG_SCREEN_BUFFER_SIZE

extern lv_color_t* disp_draw_buf;
SdCard tf;
Ota ota;

void display_init();

void HAL::Init()
{
    Serial.begin(115200);

    // Move the malloc process to Init() to make sure that the largest heap can be used for this buffer.
    disp_draw_buf = static_cast<lv_color_t*>(malloc(DISP_BUF_SIZE * sizeof(lv_color_t)));
    if (disp_draw_buf == nullptr)
        Serial.printf("lv_port_disp_init malloc failed!\n");
    // power_init();
    display_init();
    system_ui_init();
    buzz_init();
    buzz_tone(700, 30);
    
    log_system(SYSTEM_INFO, "init audio...");
    audio_init();
    delay(500);
    log_system(SYSTEM_INFO, "init system...");
    system_init();
    log_system(SYSTEM_INFO, "init network...");
    network_init();
    log_system(SYSTEM_INFO, "ota check...");
    ota.CheckVersion();
#ifdef XK_WIRELESS_PARAMETER
    log_system(SYSTEM_INFO, "init wireless tuning...");
    wireless_tuning_init();
#endif
    log_system(SYSTEM_INFO, "init mpu...");
    imu_init();
    log_system(SYSTEM_INFO, "init motor...");
    motor_init();
    
    // log_system(SYSTEM_INFO, "D-BOT IP: %s", WiFi.localIP().toString().c_str());
    // tf.init();
    // String test_content = tf.readFileLine("/dingmos_test.txt", 1);        // line-1 for WiFi ssid
    // log_i("read test file: %s", test_content.c_str());

    knob_init();
    // super_dial_init();
    system_ui_uninit();
    // xTaskNotifyGive(handleTaskIMU);
    // xTaskNotifyGive(handleTaskMotor);
    rgb_init();
}


void HAL::Update()
{
    unsigned long currentMillis = millis();
    // __IntervalExecute(HAL::knob_update(), 10);

    system_led_run(currentMillis);
    audio_update();
    rgb_update();
    // imu_update();
    // HAL::motor_task(NULL);
}
