#ifndef TRAJECTORY_APP_H
#define TRAJECTORY_APP_H
#include "status_state_app.h"

/* 当前只实现固定位置/航向、零速度的保持参考。
 * 后续运动参考需扩展速度/角速度及跟踪模型，不能只让位置随时间变化。 */
typedef struct {
    float x_m, y_m, psi_rad;
} TrajectoryPoint;
typedef struct {
    TrajectoryPoint data;
    uint32_t timestamp_us, sequence;
    bool valid;
    uint32_t valid_for_us;
    SampleSource origin_source; /* 创建保持目标时使用的状态样本。 */
} TrajectorySample;
typedef enum {
    TRAJECTORY_NOT_INITIALIZED, TRAJECTORY_IDLE, TRAJECTORY_HOLD
} TrajectoryMode;
typedef struct { TrajectoryMode mode; } TrajectoryStatus;
#define TRAJECTORY_UPDATE_PERIOD_US 5000U
#define TRAJECTORY_VALID_FOR_US 7500U
bool TrajectoryApp_Init(void);
/* 上层在控制器停用时创建保持任务；本模块不启动或修改控制器。
 * 活动任务不可替换，须先 Stop；暂不支持运行中轨迹切换。 */
bool TrajectoryApp_Hold(const StatusState *origin, uint32_t now_us);
void TrajectoryApp_Stop(void);
void TrajectoryApp_Poll(uint32_t now_us);
const TrajectorySample *TrajectoryApp_GetSample(void);
const TrajectoryStatus *TrajectoryApp_GetStatus(void);
#endif
