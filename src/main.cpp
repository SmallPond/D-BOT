#include <Arduino.h>
#include "app/app.h"
#include "hal/hal.h"
#include "hal/nvs.h"
#include "app/bot/controller.h"
#include "app/bot/bot.h"
#include "app/bot/comm/espnow_comm.h"
#include "app/bot/comm/udp_comm.h"
#include "app/bot/comm/simple_comm.h"

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
    // ffat_init();
    log_i("init comm...");
    auto comm = (iot::SimpleComm *)new iot::UDPComm(HAL::get_wifi_ssid(), 
                                            HAL::get_wifi_passwd(), 6090);
    ESP_ERROR_CHECK(comm->Init());
    // uint8_t peerAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    // esp_now->SetPeerAddress(peerAddress);
    // ESP_ERROR_CHECK(esp_now->Init());

    DBot &dbot = DBot::getInstance();
    dbot.addComm(comm);
    dbot.init();

    controller_init();
    // HWSerial.begin(115200);
    App_Init();
    // INIT_DONE();
}

void loop() {
    unsigned long currentMillis = millis();

    // strip_start(currentMillis);

    HAL::Update();
    // delay(10);
}
