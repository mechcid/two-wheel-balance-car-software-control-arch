#ifndef LQR_APP_H
#define LQR_APP_H
#include "status_state_app.h"
#include "lqr_gain.h"
#include "trajectory_app.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum { LQR_DISABLED, LQR_RUNNING, LQR_FAULT } LqrMode;
enum { LQR_BAD_STATE=1u<<0, LQR_TIMING=1u<<1, LQR_RANGE=1u<<2, LQR_OUTPUT=1u<<3, LQR_BAD_REFERENCE=1u<<4 };

//lqr配置的两个结构体，电池电压和各个参数的上下限
typedef struct { 
    float battery_v; 
} LqrActuatorConfig;

typedef struct {
    float duty_limit, start_angle_rad, stop_angle_rad, max_position_m, max_yaw_rad;
} LqrLimits;

//用一个统一的lqr进行封装
typedef struct { 
    LqrActuatorConfig actuator; 
    LqrLimits limits; 
} LqrConfig;
/* 调试比例单独保存，不修改原始误差；当前保留用户选择的默认 0.5。 */
typedef struct {
    float pitch_rate_scale;
    float yaw_rate_scale; /* 只缩放 psi_dot 项；1.0 为原始反馈，0.5 为本次对照。 */
} LqrTuning;
/* 纯计算结果：不含模块工作状态，也不表示已执行。数组顺序沿用 K 的定义。 */
typedef struct {
    float error[6], requested_nm[2], duty[2], estimated_nm[2];
    bool saturated;
} LqrResult;

typedef struct {
    LqrResult data;
    uint32_t timestamp_us, sequence; /* 成功输出的逻辑控制时刻、累计成功发布序号。 */
    bool valid;
    SampleSource state_source, motor_source, reference_source;
    uint32_t dt_us, data_age_us;
    TrajectoryPoint reference;
    LqrTuning tuning;
    float relative_rpm[2];
} LqrSample;
typedef struct {
    LqrMode mode;
    uint32_t faults, event_timestamp_us;
    bool has_output; /* 本次 Start 后是否成功执行过输出；用于启动保护与记录会话。 */
} LqrStatus;
LqrConfig LqrApp_DefaultConfig(void);
LqrTuning LqrApp_DefaultTuning(void);
bool LqrApp_SetTuning(const LqrTuning *tuning); /* 仅不在 RUNNING 时允许，不清除故障。 */
LqrTuning LqrApp_GetTuning(void);
/* Init never enables motors. Start is explicit, only from DISABLED and stopped motors. */
bool LqrApp_Init(const LqrConfig *config, const LqrTuning *tuning);
bool LqrApp_Start(uint32_t now_us);
void LqrApp_Stop(uint32_t now_us);
/* 每轮在电机采样与状态估计之后调用；先检查保护，新状态才执行内部更新。 */
void LqrApp_Poll(uint32_t now_us);
bool LqrApp_IsActive(void);

const LqrStatus *LqrApp_GetStatus(void);
const LqrSample *LqrApp_GetSample(void);

/* Pure calculation, useful for host tests; relative_rpm is raw encoder wheel RPM. */
bool LqrApp_Calculate(const StatusState *state, const TrajectoryPoint *reference,
                      const float relative_rpm[2], const LqrConfig *config, const LqrTuning *tuning,
                      LqrResult *result);
#endif
