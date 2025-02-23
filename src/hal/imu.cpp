#include "hal/hal.h"
#include <MPU6050_tockn.h>
#include "hal/HAL_Def.h"
#include "Wire.h"
#include "app/Accounts/Account_Master.h"

static MPU6050 mpu(Wire);
// static float g_abs_yaw = 0;
/*
 * roll --- x
 * yaw  --- y
 * pitch -- z
*/

static TaskHandle_t handleTaskIMU;
void HAL::imu_update(void *pvParameters)
{
    float yaw = 0;
    static float last_yaw = 0;
    while(1) {
        mpu.update();

        vTaskDelay(pdMS_TO_TICKS(5));
        // log_e("yaw: %f, %f\n", yaw, g_abs_yaw);
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
    // Set I2C frequency to 400kHz
    Wire.begin(CONFIG_MPU_SDA, CONFIG_MPU_SCL, uint32_t(4000000));
    mpu.begin();
    mpu.calcGyroOffsets(true);

    log_d("imu init pitch offset : %0.2f\n", imu_get_pitch());

    ret = xTaskCreatePinnedToCore(
        imu_update,
        "IMUThread",
        4096,
        nullptr,
        2,
        &handleTaskIMU,
        ESP32_RUNNING_CORE);
    if (ret != pdPASS) {
        log_e("start imu_run task failed.");
        // return -1;
    }
}



float HAL::imu_get_pitch(void)
{
    return mpu.getAngleY();
}

float HAL::imu_get_yaw(void)
{
    return mpu.getAngleZ();
}

float HAL::imu_get_abs_yaw(void)
{
    return imu_get_yaw();
}
