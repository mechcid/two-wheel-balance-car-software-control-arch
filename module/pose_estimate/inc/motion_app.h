#ifndef MOTION_APP_H
#define MOTION_APP_H
#include "motion_estimator.h"
#include <stdint.h>

/* 返回 uint32_t 微秒时间，允许自然回绕；必须与传感器样本使用同一时间基准。 */
typedef uint32_t (*MotionClockFn)(void);

typedef struct {
    EulerAnglesDeg data;
    uint32_t timestamp_us; /* 对应输入测量时刻，不是计算完成时间。 */
    uint32_t sequence;     /* 成功发布才递增，与输入序号独立。 */
    bool valid;
    bool discontinuity;   /* 缺样导致的连续性丢失，恢复有效后仍保留。 */
} MotionSample;

typedef enum {
    MOTION_NOT_INITIALIZED,
    MOTION_WAITING_INPUT,
    MOTION_RUNNING,
    MOTION_INPUT_UNAVAILABLE,
    MOTION_INPUT_TIMEOUT,
    MOTION_ESTIMATION_FAILED
} MotionRunState;

typedef struct {
    MotionRunState state;
    uint32_t succ_update_count;
    uint32_t fail_update_count; /* 仅统计算法更新失败，等待和超时不递增。 */
    float last_dt_s;
    bool accel_trusted;
} MotionAppStatus;
/* 主循环单实例；时钟和 SensorApp 先初始化，采集由外部独立调度。
 * 初始化时绑定时钟；空时钟或重复初始化返回 false，不改变已有状态。 */
bool MotionApp_Init(const MotionConfig *config, MotionClockFn clock_now_us);
/* 消费最新且未处理的有效样本；每轮检查失效/超时，不触发采集。
 * SensorApp_Init 清空序号，因此运行中恢复设备应使用 SensorApp_Reconnect。 */
void MotionApp_Poll(void);
/* 只读借用；失败保留上次值、时间戳和序号，但清除 valid。 */
const MotionSample *MotionApp_GetSample(void);
const MotionAppStatus *MotionApp_GetStatus(void);
#endif
