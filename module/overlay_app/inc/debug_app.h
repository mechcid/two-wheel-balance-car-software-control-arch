#ifndef DEBUG_APP_H
#define DEBUG_APP_H
#include "trace_sources.h"
/* 普通及 0.70 系数对照试验共用的自动停机时长。 */
#define DEBUG_LQR_TRIAL_DURATION_MS 7000U
/* 调试协调层和快照适配器；业务模块不依赖此文件或 trace_app。 */
typedef enum { DEBUG_TRACE_LQR, DEBUG_TRACE_IMU } DebugTraceSource;
Trace *DebugApp_GetTrace(DebugTraceSource source);
/* main 先初始化两个有类型记录器，再注入调试协调层。 */
bool DebugApp_Init(LqrTrace *lqr, ImuTrace *imu);
bool DebugApp_StartTrial(uint32_t now_us, bool use_070_scale);
void DebugApp_Stop(uint32_t now_us);
/* 主循环业务更新之后调用，负责限时停机、Trace 轮询和调试参数恢复。 */
void DebugApp_Poll(uint32_t now_us);
#endif
