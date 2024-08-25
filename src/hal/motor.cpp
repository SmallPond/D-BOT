#include "hal/hal.h"
#include "motor.h"
#include <SimpleFOC.h>
#include "app/Accounts/Account_Master.h"
#include "wireless_tune.h"
#include "nvs.h"

SPIClass* hspi = NULL;
SPIClass* hspi_1 = NULL;
static const int spiClk = 1000000; // 1MHz

BLDCMotor motor_0 = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(MO0_1, MO0_2, MO0_3);
BLDCMotor motor_1 = BLDCMotor(7);
BLDCDriver3PWM driver_1 = BLDCDriver3PWM(MO1_1, MO1_2, MO1_3);

static float g_motor_0_offset = 0;
static float g_motor_1_offset = 0;

// velocity control filtering
LowPassFilter lpf_pitch_cmd = {
    .Tf = 0.07
};
// low pass filters for user commands - throttle(油门) and steering
LowPassFilter lpf_throttle = {
    .Tf = 0.5
};
LowPassFilter lpf_steering = {
    .Tf = 0.1
};

// control algorithm parametersw
// stabilisation pid
// 初始值 P0.3 D: 0.02  -- 0.18 0.024
PIDController pid_stb{.P = 0.6, .I = 0, .D = 0.012, .ramp = 100000, .limit = 7}; 
// velocity pid 速度PID P初始值1.5
PIDController pid_vel{.P = 0, .I = 0, .D = 0.00, .ramp = 100000, .limit = 30};
float g_offset_parameters = 2.2; // 偏置参数
//目标变量
float target_velocity = 0;
#define MACHINE_MID_VALUE 1
Account* actMotorStatus;

#define MAX_MOTOR_NUM      2

static XKnobConfig x_knob_configs[] = {
    // int32_t num_positions;        
    // int32_t position;             
    // float position_width_radians; 
    // float detent_strength_unit;  
    // float endstop_strength_unit;  
    // float snap_point;           
    // char descriptor[50]; 
    [MOTOR_UNBOUND_FINE_DETENTS] = {
        0,
        0,
        1 * PI / 180,
        2,
        1,
        1.1,
        "Fine values\nWith detents", //任意运动的控制  有阻尼 类似于机械旋钮
    },
    [MOTOR_UNBOUND_NO_DETENTS] = {
        0,
        0,
        1 * PI / 180,
        0,
        0.1,
        1.1,
        "Unbounded\nNo detents", //无限制  不制动
    },
    [MOTOR_SUPER_DIAL] = {
        0,
        0,
        5 * PI / 180,
        2,
        1,
        1.1,
        "Super Dial", //无限制  不制动
    },
    [MOTOR_UNBOUND_COARSE_DETENTS] = {
        .num_positions = 0,
        .position = 0,
        .position_width_radians = 8.225806452 * _PI / 180,
        .detent_strength_unit = 2.3,
        .endstop_strength_unit = 1,
        .snap_point = 1.1,
        "Fine values\nWith detents\nUnbound"
    },
    [MOTOR_BOUND_0_12_NO_DETENTS]= {
        13,
        0,
        10 * PI / 180,
        0,
        1,
        1.1,
        "Bounded 0-13\nNo detents",
    },
    [MOTOR_BOUND_LCD_BK_BRIGHTNESS]= {
        101,
        10,
        2 * PI / 180,
        2,
        1,
        1.1,
        "Bounded 0-101\nNo detents",
    },
    [MOTOR_BOUND_LCD_BK_TIMEOUT]= {
        31,
        0,
        5 * PI / 180,
        2,
        1,
        1.1,
        "Bounded 0-3601\nNo detents",
    },
    [MOTOR_COARSE_DETENTS] = {
        32,
        0,
        8.225806452 * PI / 180,
        2,
        1,
        1.1,
        "Coarse values\nStrong detents", //粗糙的棘轮 强阻尼
    },

    [MOTOR_FINE_NO_DETENTS] = {
        256,
        127,
        1 * PI / 180,
        0,
        1,
        1.1,
        "Fine values\nNo detents", //任意运动的控制  无阻尼
    },
    [MOTOR_ON_OFF_STRONG_DETENTS] = {
        2, 
        0,
        60 * PI / 180, 
        1,             
        1,
        0.55,                    // Note the snap point is slightly past the midpoint (0.5); compare to normal detents which use a snap point *past* the next value (i.e. > 1)
        "On/off\nStrong detent", //模拟开关  强制动
    },

};

XKnobConfig motor_config[MAX_MOTOR_NUM] = {
    {
    .num_positions = 0,
    .position = 0,
    .position_width_radians = 8.225806452 * _PI / 180,
    .detent_strength_unit = 2.3,
    .endstop_strength_unit = 1,
    .snap_point = 1.1,
    }, 

    {
    .num_positions = 0,
    .position = 0,
    .position_width_radians = 8.225806452 * _PI / 180,
    .detent_strength_unit = 2.3,
    .endstop_strength_unit = 1,
    .snap_point = 1.1,
    }, 
};

// 死区制动百分率
static const float DEAD_ZONE_DETENT_PERCENT = 0.2;
// 死区RAD?
static const float DEAD_ZONE_RAD = 1 * _PI / 180;

// 怠速速度ewma alpha
static const float IDLE_VELOCITY_EWMA_ALPHA = 0.001;
// 怠速速度每秒钟弧度
static const float IDLE_VELOCITY_RAD_PER_SEC = 0.05;
// 怠速修正延迟millis
static const uint32_t IDLE_CORRECTION_DELAY_MILLIS = 500;
// 怠速校正最大角度rad
static const float IDLE_CORRECTION_MAX_ANGLE_RAD = 5 * PI / 180;
// 怠速修正率
static const float IDLE_CORRECTION_RATE_ALPHA = 0.0005;

struct motor_stat {
    
    float current_detent_center;    // 当前相对位置
    uint32_t last_idle_start;       // 上次空闲开始状态
    float idle_check_velocity_ewma; // 怠速检查速度
    float angle_to_detent_center;   // 电机角度到当前位置的偏差
};

struct motor_stat motor_s[MAX_MOTOR_NUM];

//  ------------monitor--------------------
#ifdef XK_WIRELESS_PARAMETER

CommanderWirelessGlue wireless = CommanderWirelessGlue(4242);
Commander commander = Commander(wireless, '\n', true);

#else 

Commander commander = Commander(Serial, '\n', false);
#endif

void on_stb_pid(char* cmd){commander.pid(&pid_stb, cmd);}
void on_vel_pid(char* cmd){commander.pid(&pid_vel, cmd);}
void on_imu_offset(char *cmd)
{
    commander.scalar(&g_offset_parameters, cmd); 
    log_i("imu offset change to %.2f\n", g_offset_parameters);
}

void on_motor(char* cmd){commander.motor(&motor_0, cmd);}
// -------------monitor--------------------
//目标变量
static float readMySensorCallback(void) {
    hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
    digitalWrite(hspi->pinSS(), LOW);  // pull SS slow to prep other end for transfer
    uint16_t ag = hspi->transfer16(0);
    digitalWrite(hspi->pinSS(), HIGH); // pull ss high to signify end of data transfer
    hspi->endTransaction();
    ag = ag >> 2;
    float rad = (float)ag * 2 * PI / 16384;
    if (rad < 0) {
        rad += 2 * PI;
    }
    return rad;
}
static void initMySensorCallback(void) {
    hspi = new SPIClass(HSPI);
    hspi->begin(MT6701_SCL, MT6701_SDA, -1, MT6701_SS_0); //SCLK, MISO, MOSI, SS
    pinMode(hspi->pinSS(), OUTPUT); //HSPI SS
}

GenericSensor sensor_0 = GenericSensor(readMySensorCallback, initMySensorCallback);

//目标变量
float readMySensorCallback_1(void) 
{
    hspi_1->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
    digitalWrite(hspi_1->pinSS(), LOW); //pull SS slow to prep other end for transfer
    uint16_t ag = hspi_1->transfer16(0);
    digitalWrite(hspi_1->pinSS(), HIGH); //pull ss high to signify end of data transfer
    hspi_1->endTransaction();
    ag = ag >> 2;
    float rad = (float)ag * 2 * PI / 16384;
    if (rad < 0) {
        rad += 2 * PI;
    }
    return rad;
}
void initMySensorCallback_1(void) 
{
    hspi_1 = new SPIClass(HSPI);
    hspi_1->begin(MT6701_SCL, MT6701_SDA, -1, MT6701_SS_1); //SCLK, MISO, MOSI, SS
    pinMode(hspi_1->pinSS(), OUTPUT); //HSPI SS
}

GenericSensor sensor_1 = GenericSensor(readMySensorCallback_1, initMySensorCallback_1);


static BLDCMotor *get_motor_by_id(int i)
{
    BLDCMotor *motor = NULL;

    if (i > MAX_MOTOR_NUM) {
        return NULL;
    }

    switch (i)
    {
    case 0:
        motor = &motor_0;
        break;
    case 1:
        motor = &motor_1;
        break;
    default:
        break;
    }

    return motor;
}

/* 
 * 将随机变化的值限制在一个给定的区间[min,max]内
*/ 
static float CLAMP(const float value, const float low, const float high)
{
    return value < low ? low : (value > high ? high : value);
}

void HAL::motor_shake(int id, int strength, int delay_time)
{
    BLDCMotor *motor = get_motor_by_id(id);
    if (!motor) {
        log_e("get motor by id %d failed.", id);
        return;
    }

    motor->move(strength);
    for (int i = 0; i < delay_time; i++) {
        motor->loopFOC();
        vTaskDelay(1);
    }
    motor->move(-strength);
    for (int i = 0; i < delay_time; i++) {
         motor->loopFOC();
        vTaskDelay(1);
    }
}

int HAL::get_motor_position(int id)
{
    return motor_config[id].position;
}

void HAL::update_motor_mode(int id, int mode , int init_position)
{
    BLDCMotor *motor = get_motor_by_id(id);
    if (!motor) {
        log_e("get motor by id %d failed.", id);
        return;
    }
    motor_config[id] = x_knob_configs[mode];
    motor_config[id].position = init_position;
#if XK_INVERT_ROTATION
    motor_s[id].current_detent_center = -motor->shaft_angle;
#else 
    motor_s[id].current_detent_center = motor.shaft_angle;
#endif
}

static void motor_status_publish(struct motor_stat *ms, int id, bool is_outbound)
{
    // position
    static int32_t last_position = 0;

    if (is_outbound || motor_config[id].position != last_position) {
        MotorStatusInfo info = {
            .is_outbound = is_outbound,
            .position = motor_config[id].position,
            .angle_offset = ms->angle_to_detent_center * 180 / PI,  // 转换为角度
        };
        actMotorStatus->Commit((const void*)&info, sizeof(MotorStatusInfo));
        actMotorStatus->Publish();
        last_position = motor_config[id].position;
    }
    
}

static int run_balance_task(BLDCMotor *motor_l, BLDCMotor *motor_r, float throttle)
{
    float steering = 0;
    float voltage_control;
    HAL::imu_update();
    double mpu_yaw = HAL::imu_get_yaw();

    voltage_control = lpf_pitch_cmd(pid_vel((motor_l->shaft_velocity + motor_r->shaft_velocity) / 2 - lpf_throttle(throttle)));
    // float target_yaw = 0;
    voltage_control = pid_stb(g_offset_parameters + mpu_yaw - voltage_control);
    // float voltage_control = pid_stb(Offset_parameters - mpu_pitch + target_pitch);
    // float steering_adj = lpf_steering(steering);

    float steering_adj = lpf_steering(steering);

#if MACHINE_MID_VALUE
    motor_l->target = -(voltage_control - steering_adj);
    motor_r->target = (voltage_control + steering_adj);

    if (abs(mpu_yaw - g_offset_parameters) > 40) {
        motor_l->target = 0;
        motor_r->target = 0;

    }

    motor_l->move();
    motor_r->move();
#else
    Serial.printf("yaw: %0.2f, %0.2f\n", mpu_yaw, voltage_control);
#endif
    return 0;
}

static int run_knob_task(BLDCMotor *motor, int id)
{
    struct motor_stat *ms = &motor_s[id];
    if (ms == NULL) {
        log_e("motor stats is null !");
        return -1;
    }
    ms->idle_check_velocity_ewma = motor->shaft_velocity * IDLE_VELOCITY_EWMA_ALPHA + 
                        ms->idle_check_velocity_ewma * (1 - IDLE_VELOCITY_EWMA_ALPHA);
    if (fabsf(ms->idle_check_velocity_ewma) > IDLE_VELOCITY_RAD_PER_SEC) {
        ms->last_idle_start = 0;
    } else {
        if (ms->last_idle_start == 0) {
            ms->last_idle_start = millis();
        }
    }

    // 如果我们没有移动，并且我们接近中心(但不是完全在那里)，慢慢调整中心点以匹配当前位置
    // If we are not moving and we're close to the center (but not exactly there), slowly adjust the centerpoint to match the current position
    if (ms->last_idle_start > 0 && millis() - ms->last_idle_start > IDLE_CORRECTION_DELAY_MILLIS 
            && fabsf(motor->shaft_angle - ms->current_detent_center) < IDLE_CORRECTION_MAX_ANGLE_RAD) {
        ms->current_detent_center = motor->shaft_angle * IDLE_CORRECTION_RATE_ALPHA + ms->current_detent_center * (1 - IDLE_CORRECTION_RATE_ALPHA);
    }

    //到控制中心的角度 差值
#if XK_INVERT_ROTATION
    ms->angle_to_detent_center = -motor->shaft_angle - ms->current_detent_center;
#else 
    angle_to_detent_center = motor.shaft_angle - current_detent_center;
#endif 
    // 每一步都乘以了 snap_point 的值

    if (ms->angle_to_detent_center > motor_config[id].position_width_radians * motor_config[id].snap_point 
            && (motor_config[id].num_positions <= 0 || motor_config[id].position > 0)) {
        ms->current_detent_center += motor_config[id].position_width_radians;
        ms->angle_to_detent_center -= motor_config[id].position_width_radians;
        /*
            * 这里判断为正转， position 应该 ++，这里反了之后，
            * encoder 的逻辑也需要反一下
        */
        motor_config[id].position--;   
    }
    else if (ms->angle_to_detent_center < -motor_config[id].position_width_radians * motor_config[id].snap_point 
                && (motor_config[id].num_positions <= 0 || motor_config[id].position < motor_config[id].num_positions - 1))
    {
        ms->current_detent_center -= motor_config[id].position_width_radians;
        ms->angle_to_detent_center += motor_config[id].position_width_radians;
        motor_config[id].position++;
    }

    // 死区调整
    float dead_zone_adjustment = CLAMP(
        ms->angle_to_detent_center,
        fmaxf(-motor_config[id].position_width_radians * DEAD_ZONE_DETENT_PERCENT, -DEAD_ZONE_RAD),
        fminf(motor_config[id].position_width_radians * DEAD_ZONE_DETENT_PERCENT, DEAD_ZONE_RAD));

    // 出界
    bool out_of_bounds = motor_config[id].num_positions > 0 && 
                ((ms->angle_to_detent_center > 0 && motor_config[id].position == 0) 
                || (ms->angle_to_detent_center < 0 && motor_config[id].position == motor_config[id].num_positions - 1));
    motor->PID_velocity.limit = out_of_bounds ? 10 : 3;
    motor->PID_velocity.P = out_of_bounds ? motor_config[id].endstop_strength_unit * 4 : motor_config[id].detent_strength_unit * 4;

    // 处理float类型的取绝对值
    if (fabsf(motor->shaft_velocity) > 60) {
        // 如果速度太高 则不增加扭矩
        // Don't apply torque if velocity is too high (helps avoid positive feedback loop/runaway)
        // Serial.println("(motor.shaft_velocity) > 60 !!!");
        motor->move(0);
    } else {
        // 运算符重载，输入偏差计算 PID 输出值
        float torque = motor->PID_velocity(-ms->angle_to_detent_center + dead_zone_adjustment);
        #if XK_INVERT_ROTATION
            torque = -torque;
        #endif
        motor->move(torque);
    }
    motor_status_publish(ms, id, out_of_bounds);
    return 0;
}

TaskHandle_t handleTaskMotor;
void TaskMotorUpdate(void *pvParameters)
{
    // ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
#if XK_INVERT_ROTATION
    motor_s[0].current_detent_center = -motor_0.shaft_angle;
    motor_s[1].current_detent_center = -motor_1.shaft_angle;
#else 
    motor_s[0].current_detent_center = motor_0.shaft_angle;
    motor_s[0].current_detent_center = motor_1.shaft_angle;
#endif
    while(1) {
        sensor_0.update();
        motor_0.loopFOC();
        // run_knob_task(&motor_0, 0);
        // Serial.printf("angle: %f\n", motor_1.shaft_angle);
        sensor_1.update();
        motor_1.loopFOC();
        // run_knob_task(&motor_1, 1);

        run_balance_task(&motor_0, &motor_1, 0);
        // motor.move(1);
        // motor_1.move(1);

        // motor_0.monitor();
        // motor_0.monitor();
        commander.run();
        // Serial.println(motor_config[id].position);
        vTaskDelay(5);
    }
    
}


static void init_motor(BLDCMotor *motor,BLDCDriver3PWM *driver,GenericSensor *sensor)
{
    sensor->init();
    //连接motor对象与传感器对象
    motor->linkSensor(sensor);
    // PWM 频率 [Hz]
    driver->pwm_frequency = 50000;
    //供电电压设置 [V]
    driver->voltage_power_supply = 8;
    driver->init();
    motor->linkDriver(driver);
    //FOC模型选择
    motor->foc_modulation = FOCModulationType::SpaceVectorPWM;
    //运动控制模式设置
    motor->torque_controller = TorqueControlType::voltage;
    motor->controller = MotionControlType::torque;

    // 速度PI环设置
    motor->PID_velocity.P = 1;
    motor->PID_velocity.I = 0;
    motor->PID_velocity.D = 0.01;

    motor->PID_velocity.output_ramp = 10000;
    motor->PID_velocity.limit = 10;
    //最大电机限制电机
    motor->voltage_limit = 8;
    //速度低通滤波时间常数
    motor->LPF_velocity.Tf = 0.01;
    //设置最大速度限制
    motor->velocity_limit = 10;
#ifdef XK_WIRELESS_PARAMETER
    motor->useMonitoring(wireless);
#elif
    motor->useMonitoring(Serial);
#endif
    //初始化电机
    motor->init();
    // motor->initFOC();
    motor->monitor_downsample = 1000;  // disable monitor at first - optional
}

void motor_initFOC(BLDCMotor *motor, float offset)
{
    if(offset > 0)  {
        log_i("has a offset value %.2f\n", offset);
        Direction foc_direction = Direction::CW;
        motor->initFOC(offset, foc_direction);
    } else {
        if(motor->initFOC()) {
            log_i("motor zero electric angle: %.2f", motor->zero_electric_angle);
        }
    }
}

void HAL::motor_init(void)
{

    int ret = 0;
    log_i("Motor starting...");
    pinMode(MT6701_SS_0, OUTPUT);
    digitalWrite(MT6701_SS_0, HIGH); 
    for (int i = 0; i < MAX_MOTOR_NUM; i++) {
        memset(&motor_s[i], 0, sizeof(struct motor_stat));
    }
    init_motor(&motor_0, &driver, &sensor_0);
    init_motor(&motor_1, &driver_1, &sensor_1);
    vTaskDelay(100);
    pinMode(MO_EN, OUTPUT);
    digitalWrite(MO_EN, HIGH);  

#ifdef XK_WIRELESS_PARAMETER
    String ssid, password;
    get_wifi_config(ssid, password);
    const char *wifi_name = ssid.c_str();  
    const char *wifi_pass = password.c_str(); 
    wireless.begin(wifi_name, wifi_pass);
#endif

    motor_initFOC(&motor_0, g_motor_0_offset);
    motor_initFOC(&motor_1, g_motor_1_offset);

    log_i("Motor ready.");
    log_i("Set the target velocity using serial terminal:");

    commander.add('M', on_motor, "my motor");


    commander.add('S', on_stb_pid, "PID stable");
    commander.add('V', on_vel_pid, "PID vel");
    commander.add('O', on_imu_offset, "imu offset");

    // commander.add('M', onMotor, "my motor");
    actMotorStatus = new Account("MotorStatus", AccountSystem::Broker(), sizeof(MotorStatusInfo), nullptr);
    ret = xTaskCreatePinnedToCore(
        TaskMotorUpdate,
        "MotorThread",
        4096,
        nullptr,
        2,
        &handleTaskMotor,
        ESP32_RUNNING_CORE);
    if (ret != pdPASS) {
        log_e("start motor_run task failed.");
        // return -1;
    }
}

