#ifndef MOTION_APP_H
#define MOTION_APP_H
#include "motion_estimator.h"
#include <stdint.h>

typedef struct {
    float sensor_hz, last_dt_s;
    uint32_t updates, missed_periods, no_data, failures, timestamp_us;
} MotionAppStatus;
/* 主循环单实例；Timebase 和 SensorApp 先初始化，固定 200 Hz 截止时间调度。 */
bool MotionApp_Init(const MotionConfig *config);
void MotionApp_Poll(void);
/* 距下一次读取计划的余量；超期为 0，供 OLED 分块发送让步。 */
uint32_t MotionApp_TimeUntilSampleUs(void);
const MotionState *MotionApp_GetState(void);
const MotionAppStatus *MotionApp_GetStatus(void);
void MotionApp_GetEuler(float degrees[3]);
#endif
