#include "Setting.h"
#include "hal/hal.h"
#include "config.h"

using namespace Page;
TaskHandle_t handleTaskWiFi;

const int motor_mode = MOTOR_SUPER_DIAL;
const int app_mode = APP_MODE_SETTING;

Setting::Setting()
{	
}

Setting::~Setting()
{
}

void Setting::onCustomAttrConfig()
{
    SetCustomCacheEnable(false);
    // SetCustomLoadAnimType(PageManager::LOAD_ANIM_OVER_BOTTOM, 500, lv_anim_path_bounce);
}

void Setting::onViewLoad()
{
    Model = new SettingModel();
    View = new SettingView();

    Model->Init();
    View->Create(root);

    AttachEvent(root);
    AttachEvent(View->ui.meter);

    for(int i = 0; i < SETTING_NUM ; i++) {
        AttachEvent((View->m_ui->setting[i]).cont);
    }
}

void Setting::AttachEvent(lv_obj_t* obj)
{
    lv_obj_set_user_data(obj, this);
    lv_obj_add_event_cb(obj, onEvent, LV_EVENT_ALL, this);
}

void Setting::onViewDidLoad()
{

}

void Setting::onViewWillAppear()
{
    Model->ChangeMotorMode(motor_mode);
    Model->SetPlaygroundMode(app_mode);
    View->SetPlaygroundMode(app_mode);

    timer = lv_timer_create(onTimerUpdate, 10, this);
}

void Setting::onViewDidAppear()
{

}

void Setting::onViewWillDisappear()
{

}

void Setting::onViewDidDisappear()
{
    lv_timer_del(timer);
}

void Setting::onViewDidUnload()
{
    View->Delete();
    Model->Deinit();
    delete View;
    delete Model;
}

void Setting::Update()
{
    SettingInfo info;
    Model->GetKnobStatus(&info);
    if (info.konb_direction != SUPER_DIAL_NULL){
        char* name = ((SettingView*)View)->GetEditedSettingName();
    }
    View->UpdateView(&info);
    int pos;
    switch(View->GetViewMode()){
        case LCD_BK_BRIGHTNESS_SETTING_VIEW:
            Model->SetLcdBkBrightness(info.xkonb_value);
            break;
    }
}

void Setting::onTimerUpdate(lv_timer_t* timer)
{
    Setting* instance = (Setting*)timer->user_data;

    instance->Update();
}


void Setting::SettingEventHandler(lv_event_t* event, lv_event_code_t code)
{
    lv_obj_t* obj = lv_event_get_target(event);
    lv_obj_t* label = lv_obj_get_child(obj, 1);
    SettingInfo info;
    Model->GetKnobStatus(&info);
    if (code == LV_EVENT_FOCUSED) {
        if (label != NULL) {
            printf("fouces, name:%s\n", lv_label_get_text(label));
            ((SettingView*)View)->UpdateFocusedSetting(lv_label_get_text(label));
        }
    }
    if (code == LV_EVENT_SHORT_CLICKED){
        printf("LV_EVENT_SHORT_CLICKED\n");
        if (!lv_obj_has_state(obj, LV_STATE_EDITED)) {
            if (label != NULL) {
                printf("Control setting: %s\n", lv_label_get_text(label));
            }
            if ( lv_obj_has_state(obj, LV_STATE_USER_1) ){
                printf("LV_STATE_USER_1 \n");
                lv_obj_clear_state(obj, LV_STATE_USER_1);
                return;
            } else {
                lv_obj_add_state(obj, LV_STATE_EDITED);
            }
            ((SettingView*)View)->SetCtrView(obj);
            switch(((SettingView*)View)->GetViewMode()){
                case DEFAULT_VIEW:
                    log_d("select DEFAULT_VIEW \n");
                case WIFI_SETTING_VIEW:
                    log_e("wifi setting is disabled.\n");
                    break;
                case LCD_BK_TIMEOUT_SETTING_VIEW:
                    log_d("select LCD_BK_TIMEOUT_SETTING_VIEW \n");
                    HAL::encoder_disable();
                    View->SetPlaygroundMode(LCD_BK_TIMEOUT_SETTING_VIEW);
                    Model->SetPlaygroundMode(SETTING_MODE_LCD_BK_TIMEOUT);
                    Model->ChangeMotorModeWithInitPosition(MOTOR_BOUND_LCD_BK_TIMEOUT, Model->knob_value );
                    break;
                case LCD_BK_BRIGHTNESS_SETTING_VIEW:
                    log_d("select LCD_BK_BRIGHTNESS_SETTING_VIEW \n");
                    HAL::encoder_disable();
                    View->SetPlaygroundMode(LCD_BK_BRIGHTNESS_SETTING_VIEW);
                    Model->SetPlaygroundMode(SETTING_MODE_LCD_BK_BRIGHTNESS);
                    Model->ChangeMotorModeWithInitPosition(MOTOR_BOUND_LCD_BK_BRIGHTNESS, Model->knob_value );
                    break;
            }
        } else {
            switch(((SettingView*)View)->GetViewMode()){
                case DEFAULT_VIEW:
                    log_d("Enter DEFAULT_VIEW \n");
                case WIFI_SETTING_VIEW:
                    log_d("Enter WIFI_SETTING_VIEW Press\n");
                    break;
                case LCD_BK_TIMEOUT_SETTING_VIEW:
                    log_d("Enter LCD_BK_TIMEOUT_SETTING_VIEW Press\n");
                    Model->ChangeMotorMode(MOTOR_UNBOUND_COARSE_DETENTS);
                    Model->SaveLcdBkTimeout(info.xkonb_value);
                    ((SettingView*)View)->ClearCtrView(obj);
                    View->SetPlaygroundMode(app_mode);
                    lv_obj_clear_state(obj, LV_STATE_EDITED);
                    HAL::encoder_enable();
                    break;
                case LCD_BK_BRIGHTNESS_SETTING_VIEW:
                    log_d("Enter LCD_BK_BRIGHTNESS_SETTING_VIEW Press\n");
                    Model->ChangeMotorMode(MOTOR_UNBOUND_COARSE_DETENTS);
                    Model->SaveLcdBkBrightness(info.xkonb_value);
                    View->SetPlaygroundMode(app_mode);
                    View->ClearCtrView(obj);
                    lv_obj_clear_state(obj, LV_STATE_EDITED);
                    HAL::encoder_enable();
                    break;
            }
        }
    } else if (code == LV_EVENT_LONG_PRESSED) {
        printf("Setting: LV_EVENT_LONG_PRESSED\n");
        if (lv_obj_has_state(obj, LV_STATE_EDITED)) {
            View->ClearCtrView(obj);
            lv_obj_clear_state(obj, LV_STATE_EDITED);
            HAL::encoder_enable();
            View->SetPlaygroundMode(app_mode);
            Model->ChangeMotorMode(MOTOR_UNBOUND_COARSE_DETENTS);
        }
    } else if (code == LV_EVENT_LONG_PRESSED_REPEAT) {
        // return to memu
        if (!lv_obj_has_state(obj, LV_STATE_EDITED)){
            printf("Setting: LV_EVENT_LONG_PRESSED_REPEAT\n");
            Model->ChangeMotorMode(MOTOR_UNBOUND_COARSE_DETENTS);
            Manager->Pop();
        }
    }
}

void Setting::onEvent(lv_event_t* event)
{
    lv_obj_t* obj = lv_event_get_target(event);
    lv_event_code_t code = lv_event_get_code(event);
    auto* instance = (Setting*)lv_obj_get_user_data(obj);

    instance->SettingEventHandler(event, code);
}