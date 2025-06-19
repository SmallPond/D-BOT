#include "hal/hal.h"
#include "motor.h"
#include <SimpleFOC.h>
#include "app/Accounts/Account_Master.h"
#include "nvs.h"

SPIClass* hspi = NULL;
SPIClass* hspi_1 = NULL;
static const int spiClk = 1000000; // 1MHz

BLDCMotor motor_0 = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(MO0_1, MO0_2, MO0_3);
BLDCMotor motor_1 = BLDCMotor(7);
BLDCDriver3PWM driver_1 = BLDCDriver3PWM(MO1_1, MO1_2, MO1_3);

enum BALANCE_STATUS {
    BALANCE_OFF,
    BALANCE_WATTING,
    BALANCE_RUNNING,
};

static BALANCE_STATUS g_balance_status = BALANCE_OFF;
// low pass filters for user commands - throttle(油门) and steering
LowPassFilter lpf_throttle = {
    .Tf = 0.5
};
LowPassFilter lpf_steering = {
    .Tf = 0.5
};

#define BALANCE_WAITTING_TIME  1000
#define BALANCE_ENABLE_STEERING_I_TIME  3000
#define BALANCE_STOP_PITCH_OFFSET 40

// P = 0.1 I= 0.08
#ifdef D_BOT_HW_V1

#define MOTOR_MAX_TORQUE 8

PIDController pid_stb {
    .P = 0.3, .I = 0, .D = 0.015, .ramp = 100000, 
    .limit = MOTOR_MAX_TORQUE 
}; 

#define PID_VEL_P (0.7)
#define PID_VEL_I (0.20)
#define PID_VEL_D (0.00)


PIDController pid_steering {
    .P = 0.005, .I = 0, .D = 0.00, .ramp = 100000, 
    .limit = MOTOR_MAX_TORQUE / 2
};


#else

#define MOTOR_MAX_TORQUE 7

// control algorithm parametersw
// stabilisation pid
// 初始值 P0.3 D: 0.02  -- 0.18 0.024
PIDController pid_stb {
    .P = 0.3, .I = 0, .D = 0.008, .ramp = 100000, 
    .limit = MOTOR_MAX_TORQUE 
}; 

#define PID_VEL_P (0.3)
#define PID_VEL_I (0.02)
#define PID_VEL_D (0.00)

PIDController pid_steering {
    .P = 0.01, .I = 0, .D = 0.00, .ramp = 100000, 
    .limit = MOTOR_MAX_TORQUE / 2
};

#endif

PIDController pid_vel {
    .P = PID_VEL_P, .I = PID_VEL_I, .D = PID_VEL_D, .ramp = 100000, 
    .limit = MOTOR_MAX_TORQUE
};

PIDController  pid_vel_tmp{
    .P = PID_VEL_P, .I = PID_VEL_I, .D = PID_VEL_D, .ramp = 100000, 
    .limit = MOTOR_MAX_TORQUE
};


float g_zero_angle = -1.5; // 偏置参数
float g_throttle = 0;
float g_steering = 0;
//目标变量
float target_velocity = 0;
Account* actMotorStatus;
Account* actBotStatus;

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
        2, /* detent_strength_unit */
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
    [MOTOR_RETURN_TO_CENTER] = {
        .num_positions = 1,
        .position = 0,
        .position_width_radians = 60 * PI / 180,
        .detent_strength_unit = 1,
        .endstop_strength_unit = 1,
        .snap_point = 1.1,
        "Return to center"
    }

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

struct motor_stat* motor_status_get_by_id(int id)
{
    return &motor_s[id];
    
}

double HAL::get_motor_angle_offset(int id)
{
    struct motor_stat *ms = motor_status_get_by_id(id);
    if (!ms) {
        log_e("get ms by id %d failed.", id);
        return 0;
    }
    // log_e("angel_offset %lf", ms->angle_to_detent_center * 180 / PI);
    return ms->angle_to_detent_center * 180 / PI;
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
    if (id != KNOB_MOTOR_NUM) {
        return;
    }

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

static void motor_reset_all_pid()
{
    pid_stb.reset();
    pid_vel.reset();
    pid_steering.reset();
}

static int check_balance_status(float mpu_pitch)
{
    static unsigned long start_wait = 0;
    
    if (abs(mpu_pitch - g_zero_angle) > BALANCE_STOP_PITCH_OFFSET) {
        g_balance_status = BALANCE_OFF;
        return -1;
    }

    if (g_balance_status == BALANCE_RUNNING) {
        return 0;
    }

    if (g_balance_status == BALANCE_OFF) {
        g_balance_status = BALANCE_WATTING;
        start_wait = millis();
        return -1;
    }

    if (g_balance_status == BALANCE_WATTING) {
        if (millis() < start_wait + BALANCE_WAITTING_TIME) {
            return -1;
        }
    }
    motor_reset_all_pid();
    g_balance_status = BALANCE_RUNNING;
    return 0;
}

// static void dynamic_angle(void)
// {
//     stb
// }

static int run_balance_task(BLDCMotor *motor_l, BLDCMotor *motor_r,
                                float throttle, float steering)
{
    // float voltage_control;
    static unsigned long ctlr_start_ms = 0; 
    static double target_angle = 0;
    int rc = 0;
    float speed = 0;
    float speed_adj  = 0;
    float stb_adj = 0;
    float steering_adj = 0;
    float all_adj = 0;
    float gyro_y = HAL::imu_get_gyro_y();
    float gyro_z = HAL::imu_get_gyro_z();
    float mpu_pitch = HAL::imu_get_pitch();
    // float mpu_yaw = HAL::imu_get_yaw();
    // float gyro_z = HAL::imu_get_gyro_z();
    rc = check_balance_status(mpu_pitch);
    if (rc) {
        motor_l->target = 0;
        motor_r->target = 0;
        goto out;
    }

    speed = (motor_l->shaft_velocity - motor_r->shaft_velocity) / 2;
    
    /* Cascade PID */
    // float voltage_control = pid_stb(Offset_parameters - mpu_pitch + target_pitch);
    // float steering_adj = lpf_steering(steering);

    /* Parallel PID */

#ifdef D_BOT_HW_V1

    // speed_adj = -pid_vel(speed - lpf_throttle(throttle));
    // stb_adj = pid_stb.P *(mpu_pitch - g_zero_angle) + pid_stb.D * gyro_y;
    stb_adj = pid_stb.P *(mpu_pitch - target_angle) + pid_stb.D * gyro_y;


    if (throttle != 0 || steering != 0) {
        pid_vel.I = 0;
        ctlr_start_ms = millis();
    } else {
        if (millis() > ctlr_start_ms + BALANCE_ENABLE_STEERING_I_TIME) {
            pid_vel.I = pid_vel_tmp.I;
        }
    }

    speed_adj = pid_vel(speed - lpf_throttle(throttle));
    target_angle = g_zero_angle + speed_adj;
    target_angle = constrain(target_angle, -BALANCE_STOP_PITCH_OFFSET + g_zero_angle,
                         BALANCE_STOP_PITCH_OFFSET + g_zero_angle);


    steering_adj = pid_steering.P * (lpf_steering(steering) - 0) + pid_steering.D * gyro_z;
    // all_adj = stb_adj + speed_adj;
    all_adj = stb_adj;
    
    motor_l->target = -(all_adj - steering_adj);
    motor_r->target = (all_adj + steering_adj);
#else

    stb_adj = pid_stb(g_zero_angle - mpu_pitch);
    if (throttle != 0) {
        pid_vel.I = 0;
        ctlr_start_ms = millis();
    } else {
        if (millis() > ctlr_start_ms + BALANCE_ENABLE_STEERING_I_TIME) {
            pid_vel.I = pid_vel_tmp.I;
        }
    }

    speed_adj = pid_vel(speed - lpf_throttle(throttle));
    steering_adj = pid_steering(lpf_steering(steering) - 0);
    all_adj = stb_adj + speed_adj;

    motor_l->target = (all_adj + steering_adj);
    motor_r->target = -(all_adj - steering_adj);
#endif
out:
    motor_l->move();
    motor_r->move();
    return rc;
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
    motor->PID_velocity.limit = out_of_bounds ? MOTOR_MAX_TORQUE : MOTOR_MAX_TORQUE/2;
    motor->PID_velocity.P = out_of_bounds ? motor_config[id].endstop_strength_unit * 3 : motor_config[id].detent_strength_unit * 3;

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

static void act_bot_status_publish(int running_mode)
{
    AccountSystem::BotStatusInfo info = {
        .running_mode = running_mode,
    };
    actBotStatus->Commit((const void*)&info, sizeof(AccountSystem::BotStatusInfo));
    actBotStatus->Publish();
}

static int motor_task_mode_update(int &mode, bool &is_changed)
{
    int mpu_pitch = (int)(HAL::imu_get_pitch());
    static int last_mode = BOT_RUNNING_MODE;
    static unsigned long last_change_time = 0;
    static bool is_timing = false;
    int mode_tmp = BOT_RUNNING_MODE;
   
    if (abs(mpu_pitch) > 120) {
        mode_tmp = BOT_RUNNING_XKNOB;
    }

    if (abs(mpu_pitch) < 60) {
        mode_tmp = BOT_RUNNING_BALANCE;
    }

    if (mode_tmp != mode && !is_timing) {
        last_change_time = millis(); // 记录状态变化的时间
        is_timing = true;
    }
    if (mode_tmp == mode) {
        is_timing = false;
    }

    if (is_timing && millis() - last_change_time >= 1000) {
        mode = mode_tmp; // 更新模式
        is_changed = true; // 标记状态变化
        log_i("pitch: %d mode from %d change to %d", mpu_pitch, last_mode, mode);
        last_mode = mode; // 更新上一次模式
    }
    
    return 0;
}

static void motor_reset_center()
{
    #if XK_INVERT_ROTATION
    motor_s[ENCODER_MOTOR_NUM].current_detent_center = -motor_0.shaft_angle;
    motor_s[KNOB_MOTOR_NUM].current_detent_center = -motor_1.shaft_angle;
#else 
    motor_s[ENCODER_MOTOR_NUM].current_detent_center = motor_0.shaft_angle;
    motor_s[KNOB_MOTOR_NUM].current_detent_center = motor_1.shaft_angle;
#endif
    motor_s[0].last_idle_start = 0;
    motor_s[1].last_idle_start = 0;
}
TaskHandle_t handleTaskMotor;
void motor_task(void *pvParameters)
{
    // ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    int motor_task = BOT_RUNNING_MODE;
    bool is_task_changed = false;
    motor_reset_center();
    while(1) {
        is_task_changed = false;
        sensor_0.update();
        motor_0.loopFOC();
        // Serial.printf("angle: %f\n", motor_1.shaft_angle);
        sensor_1.update();
        motor_1.loopFOC();

        motor_task_mode_update(motor_task, is_task_changed);
        
        switch(motor_task) {
        case BOT_RUNNING_MODE:
            motor_0.move(0);
            motor_1.move(0);
            break;
        case BOT_RUNNING_BALANCE:
            if (is_task_changed) {
                act_bot_status_publish(motor_task);
            }
            run_balance_task(&motor_0, &motor_1, g_throttle, g_steering);
            break;
        case BOT_RUNNING_XKNOB:
            if (is_task_changed) {
                act_bot_status_publish(motor_task);
                motor_reset_center();
            }
            run_knob_task(&motor_0, ENCODER_MOTOR_NUM);
            run_knob_task(&motor_1, KNOB_MOTOR_NUM);
            break;
        default:
            break;
        }

        motor_0.monitor();
        // motor_0.monitor();
        // Serial.println(motor_config[id].position);
        vTaskDelay(pdMS_TO_TICKS(5));
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
    motor->PID_velocity.D = 0.1;

    motor->PID_velocity.output_ramp = 10000;
    motor->PID_velocity.limit = MOTOR_MAX_SPEED;
    //最大电机限制电机
    motor->voltage_limit = 8;
    //速度低通滤波时间常数
    motor->LPF_velocity.Tf = 0.01;
    //设置最大速度限制
    motor->velocity_limit = MOTOR_MAX_SPEED;

    motor->monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE;

    motor->useMonitoring(Serial);
    //初始化电机
    motor->init();
    // motor->initFOC();
    motor->monitor_downsample = 100;  // disable monitor at first - optional
}

void motor_initFOC(BLDCMotor *motor, float offset)
{
    if(offset > 0)  {
        log_i("has a offset value %.2f.", offset);
        Direction foc_direction = Direction::CW;
        motor->initFOC(offset, foc_direction);
    } else {
        if(motor->initFOC()) {
            log_i("motor zero electric angle: %.2f", motor->zero_electric_angle);
        }
    }

#ifdef XK_WIRELESS_PARAMETER
    motor->useMonitoring(HAL::get_wl_tuning());
#else
    motor->useMonitoring(Serial);
#endif
}

void HAL::motor_init(void)
{

    int ret = 0;
    bool has_set_offset = false;
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

    if (g_system_calibration == false) {
        struct motor_offset offset;
        if(!nvs_get_motor_offset(&offset)) {
            log_i("[motor]: set offset %f, %f", offset.l_offset, offset.r_offset);
            motor_initFOC(&motor_0, offset.l_offset);
            motor_initFOC(&motor_1, offset.r_offset);
            has_set_offset = true;
        } else {
            log_i("motor: try auto calibration.");
        }
       
    } 
    if (!has_set_offset) {
        motor_initFOC(&motor_0, 0);
        motor_initFOC(&motor_1, 0);
        nvs_set_motor_config(motor_0.zero_electric_angle, 
                            motor_1.zero_electric_angle);
    }
    
    log_i("Motor ready.");

    actMotorStatus = new Account("MotorStatus", AccountSystem::Broker(), sizeof(MotorStatusInfo), nullptr);
    actBotStatus = new Account("BotStatus", AccountSystem::Broker(), sizeof(AccountSystem::BotStatusInfo), nullptr);
    ret = xTaskCreatePinnedToCore(
        motor_task,
        "MotorThread",
        4096,
        nullptr,
        2,
        &handleTaskMotor,
        MOTOR_RUNNING_CORE);
    if (ret != pdPASS) {
        log_e("start motor_run task failed.");
        // return -1;
    }
}

double HAL::motor_get_cur_angle(void)
{
    /* 弧度转角度 */
    return motor_0.shaft_angle * 180 / PI;
}

void HAL::motor_set_speed(float speed, float steering)
{
    if (g_throttle != speed || g_steering != steering) {
        // log_e("speed: %d steering %d.", speed, steering);

        if (speed < -MOTOR_MAX_SPEED) {
            speed = -MOTOR_MAX_SPEED;
        }
        if (speed > MOTOR_MAX_SPEED) {
            speed = MOTOR_MAX_SPEED;
        }

        if (steering < -BOT_MAX_STEERING) {
            steering = -BOT_MAX_STEERING;
        }
        if (steering > BOT_MAX_STEERING) {
            steering = BOT_MAX_STEERING;
        }

        g_throttle = (float)speed;
        g_steering = (float)-steering;
        // log_e("throttle: %.2f steering %.2f.", g_throttle, g_steering);

        rgb_set_mode_by_status(g_throttle, g_steering);
    }
    
}