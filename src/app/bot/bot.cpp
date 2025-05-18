/**
  *   Copyright (C) 2025 dingmos. All rights reserved.
  * @file    bot.c
  * @author  dingmos
  * @version V0.1.0
  * @date    2025-02-23
  * @brief   Bot 状态控制
*/
#include "bot.h"
#include "hal/hal.h"

int g_bot_ctrl_type = BOT_CONTROL_TYPE_AI;

#define WHEEL_DIAMETER 6
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * M_PI)
#define BOT_MOVE_END_OFFSET (30)
#define BOT_SPIN_END_OFFSET (10)
#define BOT_ACTION_END_TIME (200)
PIDController pid_bot_s {
    .P = 6, .I = 0, .D = 2, .ramp = 100000, 
    .limit = BOT_MAX_STEERING
};

#ifdef D_BOT_HW_V1
PIDController pid_bot_m {
    .P = 0.07, .I = 0, .D = 0.027, .ramp = 100000, 
    .limit = MOTOR_MAX_SPEED
}; 
#else

PIDController pid_bot_m {
    .P = 0.03, .I = 0, .D = 0.027, .ramp = 100000, 
    .limit = MOTOR_MAX_SPEED
}; 

#endif
static int execute_cmd(Command& cmd) 
{
    int rc = 0;
    float abs_yaw = 0;
    float cur = 0;
    DBot &dbot = DBot::getInstance();
    switch (cmd.type) {
        case CommandType::SPIN:
            abs_yaw = HAL::imu_get_abs_yaw();
#ifdef D_BOT_HW_V1
            dbot.setTargetValue(cmd, abs_yaw - cmd.value);
#else
             // actual yaw need to be x2
            dbot.setTargetValue(cmd, abs_yaw - cmd.value *2);
#endif
           
            cmd.status = CommandStatus::EXECUTING;
            cur = abs_yaw;
            break;
        case CommandType::MOVE:
            cur = HAL::motor_get_cur_angle();
            // log_e("move set target: %lf, cur: %lf.", cmd.value + cur, cur);
            dbot.setTargetValue(cmd, cur - cmd.value);
            cmd.status = CommandStatus::EXECUTING;
            break;
    }

    rc = dbot.cmdExe(cmd, cur);
    if (rc == 0) {
        cmd.status = CommandStatus::COMPLETED;
    }
    return rc;
}

void dbot_loop_thread(void* argument)
{
    DBot &dbot = DBot::getInstance();
    while(1) {
        dbot.loop();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void dbot_thread(void* argument)
{
    int rc = 0;
    bool ready = true;
    Command cmd;
    DBot &dbot = DBot::getInstance();
    while(1) {
        if (dbot.hasCmd()) {
            cmd = dbot.popCommand();
            HAL::audio_play_music("DeviceInsert");
            pid_bot_s.reset();
            pid_bot_m.reset();
            while (cmd.status != CommandStatus::COMPLETED) {
                rc = execute_cmd(cmd);
                vTaskDelay(pdMS_TO_TICKS(5));
            }
            HAL::audio_play_music("DevicePullout");
        } else {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

DBot& DBot::getInstance() {
    // std::unique_ptr<DBot> DBot::_instance
    static auto instance = std::unique_ptr<DBot>(new DBot());
    return *instance;
}

void DBot::addComm(iot::SimpleComm *comm) {
    _comms.push_back(comm);
}

void DBot::init(void)
{
    for (auto& comm : _comms) {
        // ESP_ERROR_CHECK(comm->Init());
        comm->SetRecvCallback([this](const JsonDocument& json) {
            handleMessage(json);
        });
    }
    TaskHandle_t handleXBotThread;
    xTaskCreate(
        dbot_thread,
        "DBotThread",
        4096,
        nullptr,
        ESP32_RUNNING_CORE,
        &handleXBotThread);
    TaskHandle_t handleDBotLoop;
    xTaskCreate(
        dbot_loop_thread,
        "DBotLoop",
        4096,
        nullptr,
        ESP32_RUNNING_CORE,
        &handleDBotLoop);
}

void DBot::handleMessage(const JsonDocument& json) {
    const char* action = json["action"];
    double target = json["target"];
    log_i("atction %s, target %.2lf\n", action, target);
    if (strcmp(action, "MOVE") == 0) {
        move(target);
    } else if (strcmp(action, "SPIN") == 0) {
        spin(target);
    }
}

void DBot::loop() {
    for (auto& comm : _comms) {
        comm->Loop();
    }
}

int DBot::setTargetValue(Command& cmd, double target)
{
    if (cmd.status != CommandStatus::PENDING) {
        return -1;
    }
    log_i("set target: %lf, CUR MOTOR angle %lf, bot angle %lf.\n", target, 
                HAL::motor_get_cur_angle(), HAL::imu_get_abs_yaw());
    cmd.target_value = target;
    return 0;
}

int DBot::cmdExe(const Command &cmd, double cur)
{
    float speed = 0, steering = 0;
    float end_offset = 0;
    bool done = false;
    static uint64_t end_time = 0;

    switch (cmd.type) {
        case CommandType::SPIN:
            steering = pid_bot_s(cur - cmd.target_value);
            end_offset = BOT_SPIN_END_OFFSET;
            break;
        case CommandType::MOVE:
            speed = pid_bot_m(cmd.target_value - cur);;
            end_offset = BOT_MOVE_END_OFFSET;
            break;
    }
    
    // Tmp cmd end condition
    if (abs(cmd.target_value - cur) > end_offset) {
        end_time = millis();
    }
    // wireless.printf("target: %.2f current %.2f, output: %.2f, %.2f.\n", 
    //             cmd.target_value, cur, steering, speed);
    if (millis() > end_time + BOT_ACTION_END_TIME) {
        done = true;
        speed = 0;
        steering = 0;
    }

    HAL::motor_set_speed(speed, steering);
    if (done) {
        return 0;
    }
    return -1;
}

void DBot::spin(double angel)
{
    commandQueue.push_back({CommandType::SPIN, angel, CommandStatus::PENDING});
}

void DBot::move(double distance)
{
    double motor_angle;
    motor_angle = distanceToAngel(distance);
    commandQueue.push_back({CommandType::MOVE, motor_angle, CommandStatus::PENDING});
}

double DBot::distanceToAngel(double distance)
{
    return distance/WHEEL_CIRCUMFERENCE*360.0;
}

bool DBot::hasCmd(void)
{
    return !commandQueue.empty();
}

Command DBot::popCommand(void) 
{
    Command cmd = commandQueue.front(); 
    commandQueue.erase(commandQueue.begin());
    return cmd;
}