#include <Arduino.h>
#include "RGB.h"
#include "app/app.h"
#include "hal/hal.h"
#include "hal/nvs.h"
#include "web/WiFiAsyncWebServer.h"
#include "hal/controller.h"

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
    ffat_init();
    controller_init();
    // strip_init();
    // HWSerial.begin(115200);
    display_init();
    App_Init();
    // INIT_DONE();
}

void loop() {
    unsigned long currentMillis = millis();

    // strip_start(currentMillis);

    HAL::Update();
    // delay(10);
}
