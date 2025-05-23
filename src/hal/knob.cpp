
#include "hal.h"
#include <Arduino.h>
#include "config.h"

static volatile int16_t EncoderDiff = 0;
static bool knob_inited = false;

static ButtonEvent EncoderPush(2000);

/* 
 * lvgl task call it in 5ms
*/ 
bool HAL::encoder_is_pushed(void)
{
    if (!knob_inited) {
        return false;
    }

    if (abs(get_motor_angle_offset(ENCODER_MOTOR_NUM)) > 30) {
        return true;
    }
    // if (digitalRead(PUSH_BUTTON_PIN) == LOW) {
    //     // Serial.printf("Push button Pressed\n");
    //     return true;
    // } 
    return false;
}

void HAL::knob_update(void)
{
    // EncoderPush.EventMonitor(encoder_is_pushed());
}


static void Encoder_PushHandler(ButtonEvent* btn, int event)
{
    
    if (event == ButtonEvent::EVENT_PRESSED)
    {
        Serial.printf("push is pused\n");
        // HAL::Buzz_Tone(500, 20);
        // EncoderDiffDisable = true;
        ;
    } else if (event == ButtonEvent::EVENT_RELEASED)
    {
        // HAL::Buzz_Tone(700, 20);
        // EncoderDiffDisable = false;
        ;
    } else if (event == ButtonEvent::EVENT_LONG_PRESSED)
    {
        // HAL::Audio_PlayMusic("Shutdown");
        // HAL::Power_Shutdown();
        ;
    }
}

void HAL::knob_init(void)
{
    // pinMode(PUSH_BUTTON_PIN, INPUT_PULLUP);
    // EncoderPush.EventAttach(Encoder_PushHandler);
    update_motor_mode(ENCODER_MOTOR_NUM, MOTOR_RETURN_TO_CENTER, 0);
    update_motor_mode(KNOB_MOTOR_NUM, MOTOR_UNBOUND_COARSE_DETENTS, 0);
    knob_inited = true;

    // attachInterrupt(CONFIG_ENCODER_A_PIN, Encoder_A_IrqHandler, CHANGE);
    // push_button.EventAttach(button_handler);

}