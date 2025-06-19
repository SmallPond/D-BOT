#ifndef __CONFIG_H__
#define __CONFIG_H__

#define TFT_SCLK 12
#define TFT_MOSI 11
#define TFT_CS 10
#define TFT_BLK 21
#define TFT_DC 14
#define TFT_RST 9
#define CONFIG_TFT_BLK_CHANNEL     0

#define MO0_1 GPIO_NUM_4
#define MO0_2 GPIO_NUM_5
#define MO0_3 GPIO_NUM_6
#define MT6701_SS_0 GPIO_NUM_42

<<<<<<< HEAD
<<<<<<< HEAD
=======
#define MO_EN GPIO_NUM_35
// #define MO_EN GPIO_NUM_40
=======
// #define MO_EN GPIO_NUM_35
#define MO_EN GPIO_NUM_40
>>>>>>> 19f88fa ([hw] hardware v1 is ready)

>>>>>>> 761b5a4 ([ota] Add basic OTA functionality to prepare for subsequent OTA upgrades and integration with Xiaozhi MCP.)
#define MO1_1 GPIO_NUM_7
#define MO1_2 GPIO_NUM_15
#define MO1_3 GPIO_NUM_16

#define MT6701_SS_1 GPIO_NUM_41

#define MT6701_SDA 1
#define MT6701_SCL 2

#define CONFIG_SCREEN_HOR_RES       240
#define CONFIG_SCREEN_VER_RES       240
#define CONFIG_SCREEN_BUFFER_SIZE   (CONFIG_SCREEN_HOR_RES * CONFIG_SCREEN_VER_RES /2)

#define PUSH_BUTTON_PIN 5
#define BATTERY_OFF_PIN 7
#define ON_OFF_PIN      18
#define LVGL_RUNNING_CORE   1            
#define ESP32_RUNNING_CORE  1
<<<<<<< HEAD
#define MOTOR_RUNNING_CORE  0          
=======
#define MOTOR_RUNNING_CORE  0            
>>>>>>> b6f9f82 ([hal] add rgb)

 // The owner of the X-Knob      
#define MQTT_HOST               "dingmos"  


#define LCD_BK_DEFAULT_BRIGHTNESS  50
#define LCD_BK_DEFAULT_TIMEOUT     5

/* system */
#define SYSTEM_CALIBRATION_BUTTON_PIN  0

/* Buzzer */
#define CONFIG_BUZZ_PIN             18
#define CONFIG_BUZZ_CHANNEL         2
#define CONFIG_SOUND_ENABLE_DEFAULT false  


/* LED */
#define CONFIG_SYSTEM_LED_PIN             17

/* SD Card */
#define CONFIG_SDCARD_SS_PIN             47

/* MPU */
#define CONFIG_MPU_SDA                39
#define CONFIG_MPU_SCL                38

#define WIRELESS_TUNING_PORT      (4242)

<<<<<<< HEAD
#ifdef D_BOT_HW_V1
#define MO_EN                 GPIO_NUM_40
#define CONFIG_RGB_PIN        GPIO_NUM_46
#else
#define MO_EN GPIO_NUM_35
#endif

=======
// #define CONFIG_RGB_PIN        3
#define CONFIG_RGB_PIN        GPIO_NUM_46
>>>>>>> 19f88fa ([hw] hardware v1 is ready)
#endif 