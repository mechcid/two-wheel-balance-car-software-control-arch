#ifndef SENSOR_APP_H
#define SENSOR_APP_H
#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

//一个MPU6050传感器的原始数据样本，包含加速度、陀螺仪和温度数据。
typedef struct {
    float ax_g, ay_g, az_g;
    float gx_dps, gy_dps, gz_dps;
    float temperature_c;
    uint32_t timestamp_ms; /* 最近一次成功采集的开始时间。 */
    bool valid;            /* 是否已发布过至少一份完整样本。 */
} SensorSample;

//定义修正参数结构体，用于对传感器数据进行校准，包括加速度和陀螺仪的零偏和比例，以及温度偏移。
typedef struct {
    float accel_bias_g[3], accel_scale[3];
    float gyro_bias_dps[3], gyro_scale[3];
    float temperature_offset_c;
} SensorCalibration;

typedef struct {
    bool ready;
    uint8_t address, who_am_i;
    uint32_t reads, failures;
    HAL_StatusTypeDef last_result;
    uint32_t bus_error;
} SensorStatus;

/* 单实例，仅在主循环中访问；不负责调度、日志或显示。
 * 初始化清空样本和统计，并复制传入的校准参数；传 NULL 使用零偏 0、比例 1。
 * 总线为空或校准参数非法时返回 HAL_ERROR，不改变已有状态、不访问硬件。
 * 重连保留已有样本、校准参数和统计。两种初始化路径目前都会在 MPU
 * 驱动中阻塞等待；由调用者决定重试时机。 */
HAL_StatusTypeDef SensorApp_Init(I2C_HandleTypeDef *bus, const SensorCalibration *initial_calibration);
HAL_StatusTypeDef SensorApp_Reconnect(void);
HAL_StatusTypeDef SensorApp_Update(uint32_t now_ms);
const SensorSample *SensorApp_GetSample(void);
const SensorStatus *SensorApp_GetStatus(void);
/* 修正结果 =（输入物理量 - 零偏）× 比例；温度采用加法偏移修正。
 * 传入 NULL 恢复默认参数；新参数在下一次成功采样时生效。
 * 拒绝非有限数值或超出范围的系数；本接口不执行自动校准。 */
bool SensorApp_SetCalibration(const SensorCalibration *calibration);
#endif
