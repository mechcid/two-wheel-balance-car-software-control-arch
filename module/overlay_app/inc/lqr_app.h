#ifndef LQR_APP_H
#define LQR_APP_H
#include "status_state_app.h"
#include "lqr_gain.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum { LQR_DISABLED, LQR_HOLD, LQR_FAULT } LqrMode;
enum { LQR_BAD_STATE=1u, LQR_TIMING=2u, LQR_RANGE=4u, LQR_OUTPUT=8u };

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
typedef struct {
    LqrMode mode;
    uint32_t faults, updates, timestamp_us, state_sequence, dt_us, data_age_us;
    uint32_t sequence, input_timestamp_us, event_timestamp_us;
    float relative_rpm[2], pitch_rate_scale, yaw_rate_scale;
    float hold[3]; /* Fixed x/y/psi, captured only on Start. */
    float error[6], requested_nm[2], duty[2], estimated_nm[2];
    bool saturated;
    bool data_valid; /* 当前控制输出对应新鲜有效状态；启用前、停止和故障时为 false。 */
} LqrStatus;
LqrConfig LqrApp_DefaultConfig(void);
LqrTuning LqrApp_DefaultTuning(void);
bool LqrApp_SetTuning(const LqrTuning *tuning); /* 仅不在 HOLD 时允许，不清除故障。 */
LqrTuning LqrApp_GetTuning(void);
/* Init never enables motors. Start is explicit, only from DISABLED and stopped motors. */
bool LqrApp_Init(const LqrConfig *config, const LqrTuning *tuning);
bool LqrApp_Start(uint32_t now_us);
void LqrApp_Stop(uint32_t now_us);
/* Update after fresh StatusStateApp_Update; Poll every main-loop iteration. */
void LqrApp_Update(uint32_t now_us);
void LqrApp_Poll(uint32_t now_us);
bool LqrApp_IsActive(void);
const LqrStatus *LqrApp_GetStatus(void);
/* Pure calculation, useful for host tests; relative_rpm is raw encoder wheel RPM. */
bool LqrApp_Calculate(const StatusState *state, const float hold[3],
                      const float relative_rpm[2], const LqrConfig *config, const LqrTuning *tuning,
                      LqrStatus *result);
#endif
