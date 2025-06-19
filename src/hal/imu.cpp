#include "hal/hal.h"

#include "hal/HAL_Def.h"
#include "Wire.h"
#include "app/Accounts/Account_Master.h"
#include "nvs.h"


struct imu_data {
    float angle_x;
    float angle_y;
    float angle_z;
    float gyro_z;
    float gyro_x;
    float gyro_y;
};

static struct imu_data g_imu_data;

#ifdef D_BOT_HW_V1
#include "SparkFun_BMI270_Arduino_Library.h"
#include "kalman.h"
uint8_t bmi_addr = BMI2_I2C_PRIM_ADDR;  // 0x68

static BMI270 imu;
static Kalman kalman_y;

float gyroXoffset, gyroYoffset, gyroZoffset;

static float preInterval;
static double kalman_pre_interval = 0;

/*
  * 加速度转换为角度
  * acc2rotation(ax, ay) 
  * acc2rotation(az, ay) 
*/
double acc2_rotation(double x, double y)
{
    double tmp_kal_angle = (atan(x / y) / 1.570796 * 90);
    if (y < 0) {
        return tmp_kal_angle + 180;
    } else if (x < 0) {
        return tmp_kal_angle + 360;
    } else {
        return tmp_kal_angle;
    }
}

void kalman_init()
{
    float pitch = 0;
    for (int t = 0; t < 10; t++) {
        imu.getSensorData();
        // pitch = acc2_rotation(imu.data.accelX, imu.data.accelZ);
        pitch = atan2(imu.data.accelX, imu.data.accelZ) * 360 / -2.0 / PI;;
        log_i("BMI270 pitch: %.2f", pitch);
    }

    kalman_y.setAngle(pitch);
    kalman_pre_interval = micros();
}

void calc_gyro_offsets(void)
{
	float x = 0, y = 0, z = 0;
	int16_t rx, ry, rz;

    log_i("========================================");
    log_i("Calculating gyro offsets");
    log_i("DO NOT MOVE MPU6050");

    for (int i = 0; i < 3000; i++) {
        imu.getSensorData();

        x += imu.data.gyroX;
        y += imu.data.gyroY;
        z += imu.data.gyroZ;
    }
    gyroXoffset = x / 3000;
    gyroYoffset = y / 3000;
    gyroZoffset = z / 3000;

    imu.getSensorData();


    log_i("Done!");
    log_i("X : %.2f, Y: %.2f, Z: %.2f", gyroXoffset, gyroYoffset, gyroZoffset);

}

static float angleGyroX = 0, angleGyroY = 0, angleGyroZ = 0;

void bmi270_update(void)
{
    float accX, accY, accZ;
    float angleAccX, angleAccY;
    float gyroX, gyroY, gyroZ;
    float interval;
    float accCoef = 0.2f;
    float gyroCoef = 0.8f;


    imu.getSensorData();
    accX = imu.data.accelX;
    accY = imu.data.accelY;
    accZ = imu.data.accelZ;
    
    angleAccX = atan2(accY, accZ ) * 360 / 2.0 / PI;
    angleAccY = atan2(accX, accZ ) * 360 / -2.0 / PI;

    gyroX = imu.data.gyroX;
    gyroY = imu.data.gyroY;
    gyroZ = imu.data.gyroZ;

    g_imu_data.gyro_x = gyroX -= gyroXoffset;
    g_imu_data.gyro_y = gyroY -= gyroYoffset;
    g_imu_data.gyro_z = gyroZ -= gyroZoffset;

    interval = (millis() - preInterval) * 0.001;

    angleGyroX += gyroX * interval;
    angleGyroY += gyroY * interval;
    angleGyroZ += gyroZ * interval;

    g_imu_data.angle_x = (gyroCoef * (g_imu_data.angle_x + gyroX * interval)) + (accCoef * angleAccX);
    g_imu_data.angle_y = (gyroCoef * (g_imu_data.angle_y + gyroY * interval)) + (accCoef * angleAccY);
    g_imu_data.angle_z = angleGyroZ;

    preInterval = millis();
}

void
bmi270_update_kalman(void)
{
    imu.getSensorData();
    double interval = (micros() - kalman_pre_interval) * 0.000001;
    kalman_pre_interval = micros();
    // float angle_y = acc_2_rotation(imu.data.accelX, imu.data.accelZ);
    float angle_y = atan2(imu.data.accelX, imu.data.accelZ ) * 360 / -2.0 / PI;
    g_imu_data.angle_y = kalman_y.getAngle(angle_y, imu.data.gyroY - gyroYoffset,
                                                 interval);

    g_imu_data.gyro_z = imu.data.gyroZ - gyroZoffset;
    g_imu_data.gyro_y = imu.data.gyroY - gyroYoffset;
    g_imu_data.angle_z += g_imu_data.gyro_z * interval;
}


#else

#include <MPU6050_tockn.h>
static MPU6050 mpu(Wire);

#endif 


float imu_get_gyro_y(void)
{
    return g_imu_data.gyro_y;
}

// static float g_abs_yaw = 0;
/*
 * roll --- x
 * yaw  --- y
 * pitch -- z
*/

TaskHandle_t handleTaskIMU;
void HAL::imu_update(void *pvParameters)
{
    // ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    float gyro_z;
    float yaw = 0;
    static float last_yaw = 0;
    memset(&g_imu_data, 0, sizeof(g_imu_data));
    preInterval = millis();
    static long log_pre = 0; 
    while(1) {
#ifdef D_BOT_HW_V1


        // bmi270_update();
        bmi270_update_kalman();
#else
        mpu.update();

        g_imu_data.angle_y = mpu.getAngleY();
        g_imu_data.angle_z = mpu.getAngleZ();
        g_imu_data.gyro_z = mpu.getGyroZ();
#endif
        
        vTaskDelay(pdMS_TO_TICKS(5));
        // log_e("yaw : %f, %f, %f\n", mpu.getGyroX(),  mpu.getGyroY(),  mpu.getGyroZ());
        // unsigned long currentMillis = millis();
        // if (currentMillis - log_pre >= 100) {
        //     log_e("%.2f,%.2f,%.2f,%.2f", 
        //             imu_get_gyro_y(), imu_get_gyro_z(), 
        //             imu_get_yaw(), imu_get_pitch());
        //     log_pre = currentMillis;  
        // }
        
    }
    // imuInfo.ax = mpu.getAccX();
    // imuInfo.ay = mpu.getAccY();
    // imuInfo.az = mpu.getAccZ();
    // imuInfo.gx = mpu.getGyroX();
    // imuInfo.gy = mpu.getGyroY();
    // imuInfo.gz = mpu.getGyroZ();
    // imuInfo.mx = mpu.getMagX();
    // imuInfo.my = mpu.getMagY();
    // imuInfo.mz = mpu.getMagZ();
    // imuInfo.roll = mpu.getAngleX();
    // imuInfo.yaw = mpu.getAngleY();
    // imuInfo.pitch = mpu.getAngleZ();
    
    // AccountSystem::imu_commit(&imuInfo);
}

void HAL::imu_init(void)
{
    int ret = 0;
    bool is_set_offset = false;
    // Set I2C frequency to 400kHz
    Wire.begin(CONFIG_MPU_SDA, CONFIG_MPU_SCL, uint32_t(4000000));
#ifdef D_BOT_HW_V1

    while (imu.beginI2C(bmi_addr) != BMI2_OK) {
        // Not connected, inform user
        log_e("Error: BMI270 not connected, check wiring and I2C address!");
        // Wait a bit to see if connection is established
        delay(1000);
    }
    preInterval = millis();
    log_i("BMI270 connected!");
    calc_gyro_offsets();

    kalman_init();
#else
    mpu.begin();
    // if (!g_system_calibration) {
    //     struct imu_offset offset;
    //     int ret = get_imu_offset(&offset);
    //     if (ret == 0) {
    //         log_i("mpu get offset: %f, %f, %f", offset.xoffset, offset.yoffset,
    //                                  offset.zoffset);
    //         mpu.setGyroOffsets(offset.xoffset, offset.yoffset, offset.zoffset);
    //         is_set_offset = true;
    //     }
    // }

    if (!is_set_offset) {
        mpu.calcGyroOffsets(true);
        set_imu_config(mpu.getGyroXoffset(), mpu.getGyroYoffset(), 
                                mpu.getGyroYoffset());
    }

#endif
    log_d("imu init pitch offset : %0.2f\n", imu_get_pitch());

    ret = xTaskCreatePinnedToCore(
        imu_update,
        "IMUThread",
        4096,
        nullptr,
        2,
        &handleTaskIMU,
        MOTOR_RUNNING_CORE);
    if (ret != pdPASS) {
        log_e("start imu_run task failed.");
        // return -1;
    }
}



float HAL::imu_get_pitch(void)
{
    return g_imu_data.angle_y; /* 0-180  -180 - 0 */
}

float HAL::imu_get_yaw(void)
{
    return g_imu_data.angle_z;
}

/*
 * getGyroY = pitch
 * getGyroZ = yaw
 * 
 */
float HAL::imu_get_gyro_z(void)
{
    return g_imu_data.gyro_z;
}

float HAL::imu_get_gyro_y(void)
{
    return g_imu_data.gyro_y;
}

float HAL::imu_get_abs_yaw(void)
{
    return imu_get_yaw();
}
