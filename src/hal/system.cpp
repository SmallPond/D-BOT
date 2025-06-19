#include "button_event.h"
#include <Arduino.h>
#include "hal.h"
#include <lvgl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

bool g_system_calibration = false;
static bool g_network_config = false;
static ButtonEvent btn_boot(5000);
static unsigned long pixelPrevious = 0;        // Previous Pixel Millis
#define LED_INTERVAL  500;       // Pixel Interval (ms)

void HAL::system_led_init(void)
{
    pinMode(CONFIG_SYSTEM_LED_PIN, OUTPUT);
}

bool HAL::system_is_network_config(void)
{
    return g_network_config;
}


void HAL::system_led_run(unsigned long currentMillis)
{
    static int val = 1;
    if (currentMillis - pixelPrevious >= 500) { 
        pixelPrevious = currentMillis;                       //  Run current frame
        digitalWrite(CONFIG_SYSTEM_LED_PIN, val);
        val = !val;
    }
}

int HAL::system_init(void)
{
    system_led_init();

    pinMode(SYSTEM_CALIBRATION_BUTTON_PIN, INPUT_PULLUP);
    if (digitalRead(SYSTEM_CALIBRATION_BUTTON_PIN) == LOW) {
        g_system_calibration = true;
    }

    pinMode(CONFIG_LEFT_BUTTON_PIN, INPUT_PULLUP);
    if (digitalRead(CONFIG_LEFT_BUTTON_PIN) == LOW) {
        g_network_config = true;
    }
    log_i("start with calibration %s", g_system_calibration? "true": "false");
    return 0;
}

// 日志缓冲区大小
#define LOG_BUFFER_SIZE 1024

static const lv_color_t log_colors[] = {
    LV_COLOR_MAKE(255, 0, 0),   // ERROR - 红色
    LV_COLOR_MAKE(255, 200, 0), // WARNING - 橙色
    LV_COLOR_MAKE(0, 255, 0),   // INFO - 绿色
    LV_COLOR_MAKE(0, 180, 255)  // DEBUG - 青色
};

static lv_obj_t *g_log_area;

void HAL::log_system(int level, const char *fmt, ...)
{
    char buffer[256];
    va_list args;

    va_start(args, fmt);
    lv_vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    /** log to serial */
    log_i("%s", buffer);

    if (!g_log_area) {
        return;
    }
    
    if(level < 0 || level >= sizeof(log_colors)/sizeof(log_colors[0])) {
        level = SYSTEM_ERR;
    }

    lv_textarea_add_text(g_log_area, buffer);
    lv_textarea_add_char(g_log_area, '\n');
    
    // lv_obj_set_style_text_color(lv_textarea_get_label(g_log_area), 
                            //   log_colors[level], 
                            //   LV_PART_SELECTED);
    
    // lv_obj_scroll_to_view_recursive(lv_textarea_get_label(g_log_area), LV_ANIM_OFF);

    // lv_refr_now(NULL);
}

int HAL::system_ui_init(void) 
{
    lv_obj_t *screen = lv_obj_create(lv_scr_act());
    // lv_obj_remove_style_all(screen);
    // lv_obj_set_size(screen, LV_HOR_RES, LV_VER_RES);
    //lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
    //  lv_obj_set_style_bg_color(screen, lv_color_black(), 0);

    lv_obj_remove_style_all(screen);
	lv_obj_set_size(screen, LV_HOR_RES, LV_VER_RES);
	lv_obj_set_style_bg_color(screen, lv_color_black(), 0);
	lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);

    g_log_area = lv_textarea_create(screen);
    lv_obj_set_size(g_log_area, CONFIG_SCREEN_HOR_RES - 40 , CONFIG_SCREEN_VER_RES - 90);
    lv_obj_align(g_log_area, LV_ALIGN_CENTER, 0, 0);
    lv_obj_clear_flag(g_log_area, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_opa(g_log_area, LV_OPA_0, LV_PART_MAIN);
    lv_obj_set_style_text_color(g_log_area, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_border_width(g_log_area, 0, LV_PART_MAIN);

    lv_textarea_set_text(g_log_area, "booting system...\n");
    return 0;
}

int HAL::system_ui_uninit()
{

    // lv_obj_clean(lv_scr_act());
    lv_textarea_set_text(g_log_area, "");
    // lv_obj_del(g_log_area);
    // g_log_area = NULL;
    return 0;
}