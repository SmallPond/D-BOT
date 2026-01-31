#include <Arduino.h>
#include "app/app.h"
#include "hal/hal.h"
#include "hal/nvs.h"
#include "app/bot/controller.h"
#include "app/bot/bot.h"
#include "app/bot/comm/espnow_comm.h"
#include "app/bot/comm/udp_comm.h"
#include "app/bot/comm/simple_comm.h"

static int dbot_init(void)
{
    int rc = 0;
    if (HAL::is_network_ready()) {
        log_i("init comm...");
        auto comm = (iot::SimpleComm *)new iot::UDPComm(HAL::get_wifi_ssid(), 
                                                HAL::get_wifi_passwd(), 6090);
        ESP_ERROR_CHECK(comm->Init());
        log_i("init dbot...");
        DBot &dbot = DBot::getInstance();
        dbot.addComm(comm);
        dbot.init();
    } else {
        log_i("skip dbot init.");
    }
    
    log_i("init controller...");
    char ble_addr[24];
    rc = nvs_get_string(GAME_CTRLR, GAME_CTRLR_ADDR_KEY, ble_addr, 
                                    sizeof(ble_addr));
    if (!rc) {
        log_i("get ble addr %s", ble_addr);
        controller_init(ble_addr);
    } else {
        controller_init(DEFAULTU_BLE_ADDR);
    }
    return 0;
}

void push_handler(ButtonEvent* btn, int event)
{
    if (event == ButtonEvent::EVENT_PRESSED)
    {
        Serial.println("button pressed");
    } else if (event == ButtonEvent::EVENT_LONG_PRESSED) {
        Serial.println("button long pressed");
    } 
    if (event == ButtonEvent::EVENT_DOUBLE_CLICKED) {
        Serial.println("button double clicked");
    } else if (event == ButtonEvent::EVENT_SHORT_CLICKED) {
        Serial.println("button short clicked");
    } else if (event == ButtonEvent::EVENT_CLICKED) {
        Serial.println("button clicked"); 
    }
}

void setup() {
    nvs_init();
    HAL::Init();
    
    dbot_init();
    // HWSerial.begin(115200);
    log_i("init App...");
    App_Init();
    // INIT_DONE();
}

void loop() {
    unsigned long currentMillis = millis();

    // strip_start(currentMillis);

    HAL::Update();
    // delay(10);
}
