#ifndef SENSOR_APP_H
#define SENSOR_APP_H
#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define IMU_OUTPUT_HZ 200U
#define IMU_PERIOD_US (1000000U / IMU_OUTPUT_HZ)
#define IMU_TIMEOUT_US 20000U

typedef struct {
    float accel[3]; /* 校准后 g，X/Y/Z。 */
    float gyro[3];  /* 校准后 degree/s，X/Y/Z。 */
} ImuData;
typedef struct {
    ImuData data;
    uint32_t timestamp_us; /* 成功事务的读取开始时间，非芯片内部采样时刻。 */
    uint32_t sequence;     /* 软件发布序号；成功才递增，重连不重置。 */
    bool valid;            /* 完整有效；失败、重连或检测到超时后清除。 */
} ImuSample;

//定义修正参数结构体，用于对传感器数据进行校准，包括加速度和陀螺仪的零偏和比例。
typedef struct {
    float accel_bias_g[3], accel_scale[3];
    float gyro_bias_dps[3], gyro_scale[3];
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
 * 重连保留已有数值、序号、校准参数和统计，但清除 valid。两种初始化路径目前都会在 MPU
 * 驱动中阻塞等待；由调用者决定重试时机。 */
HAL_StatusTypeDef SensorApp_Init(I2C_HandleTypeDef *bus, const SensorCalibration *initial_calibration);
HAL_StatusTypeDef SensorApp_Reconnect(void);
/* 检查数据就绪；无新样本返回 HAL_BUSY，不增加序号及失败计数。
 * 旧数据超时则清除 valid；Timebase 必须先初始化。 */
HAL_StatusTypeDef SensorApp_Update(void);
float SensorApp_GetOutputRate(void);
const ImuSample *SensorApp_GetSample(void);
const SensorStatus *SensorApp_GetStatus(void);
/* 修正结果 =（输入物理量 - 零偏）× 比例。
 * 传入 NULL 恢复默认参数；新参数在下一次成功采样时生效。
 * 拒绝非有限数值或超出范围的系数；本接口不执行自动校准。 */
bool SensorApp_SetCalibration(const SensorCalibration *calibration);
#endif
