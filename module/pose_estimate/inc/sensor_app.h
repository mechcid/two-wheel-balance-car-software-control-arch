#ifndef SENSOR_APP_H
#define SENSOR_APP_H
#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

//定义频率，周期和超时的宏，分别为200Hz，5000us和20000us，供后续引入的文件使用
#define IMU_OUTPUT_HZ 200U
#define IMU_PERIOD_US (1000000U / IMU_OUTPUT_HZ)
#define IMU_TIMEOUT_US 20000U

/* 微秒时钟：持续递增，允许 uint32_t 自然回绕；使用前由调用者初始化。
 * 消费者判断样本年龄时必须使用同一时间基准。 */
typedef uint32_t (*SensorClockFn)(void);

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
    bool ready;                 /* 模块是否就绪，可读取数据；不表示样本新鲜。 */
    uint32_t succ_read_count;    /* 成功读取并发布样本的次数。 */
    uint32_t fail_read_count;    /* 数据就绪检查或读取发生错误的次数。 */
    /* 最近一次记录的初始化、重连或采集结果，供后续诊断。
     * 使用 HAL_OK/HAL_ERROR/HAL_BUSY/HAL_TIMEOUT；无新数据及未就绪的
     * 提前返回不更新快照。成功操作会覆盖此前记录，不是故障历史。 */
    HAL_StatusTypeDef reason;
    uint32_t bus_error;          /* 记录 reason 时同步保存的底层错误码，防止后续变化影响诊断。 */
} SensorStatus;

/* 单实例，仅在主循环中访问；管理自身采集周期，不负责其他模块的调度、日志或显示。
 * 初始化清空样本和统计，并复制传入的校准参数；传 NULL 使用零偏 0、比例 1。
 * 初始化时绑定时钟函数，重连保留绑定。
 * 总线或时钟函数为空、校准参数非法时返回 HAL_ERROR，不改变已有状态、不访问硬件。
 * 重连保留已有数值、序号、校准参数和统计，但清除 valid。两种初始化路径目前都会在 MPU
 * 驱动中阻塞等待；由调用者决定重试时机。初始化或重连结束后等待一个周期再采集。 */
HAL_StatusTypeDef SensorApp_Init(I2C_HandleTypeDef *bus, const SensorCalibration *initial_calibration,
                                  SensorClockFn clock_now_us);
HAL_StatusTypeDef SensorApp_Reconnect(void);
/* 唯一公开的采集推进入口，由主循环持续调用。
 * 每次检查样本超时，到期后执行一次内部更新；积压不补读。
 * 周期未到或无新数据不增加序号及失败计数，不覆盖诊断快照；未初始化时不执行。 */
void SensorApp_Poll(void);
/* 只读查询距下一次计划采集的微秒余量；未初始化或已到期返回 0。 */
uint32_t SensorApp_TimeUntilSampleUs(void);
float SensorApp_GetOutputRate(void);

const ImuSample *SensorApp_GetSample(void);
const SensorStatus *SensorApp_GetStatus(void);
/* 修正结果 =（输入物理量 - 零偏）× 比例。
 * 传入 NULL 恢复默认参数；新参数在下一次成功采样时生效。
 * 拒绝非有限数值或超出范围的系数；本接口不执行自动校准。 */
bool SensorApp_SetCalibration(const SensorCalibration *calibration);
#endif
