#ifndef MOTION_APP_H
#define MOTION_APP_H
#include "motion_estimator.h"
#include <stdint.h>

/* 姿态处理频率上限，与 IMU 输出频率独立；实际更新还需要新输入。
 * 默认 400 Hz，为 200 Hz 输入的到达抖动留余量，不会重复处理输入。
 * 降低频率会跳过中间测量，并需重新评估控制周期和超时约束。 */
#ifndef MOTION_UPDATE_HZ
#define MOTION_UPDATE_HZ 400U
#endif
#if MOTION_UPDATE_HZ == 0 || MOTION_UPDATE_HZ > 1000000U
#error "MOTION_UPDATE_HZ must be between 1 and 1000000"
#endif
#define MOTION_MIN_UPDATE_INTERVAL_US ((1000000U + MOTION_UPDATE_HZ - 1U) / MOTION_UPDATE_HZ)

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
 * 首份输入立即处理；之后两次算法尝试的开始时间至少间隔一个处理周期。
 * 等待新输入或等待周期不会推进处理时间/输入序号；dt 仍使用输入时间戳差。
 * SensorApp_Init 清空序号，因此运行中恢复设备应使用 SensorApp_Reconnect。 */
void MotionApp_Poll(void);
/* 只读借用；失败保留上次值、时间戳和序号，但清除 valid。 */
const MotionSample *MotionApp_GetSample(void);
const MotionAppStatus *MotionApp_GetStatus(void);
#endif
