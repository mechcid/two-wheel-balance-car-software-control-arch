#ifndef STATUS_STATE_APP_H
#define STATUS_STATE_APP_H
#include <stdbool.h>
#include <stdint.h>
#include "sample_source.h"
/* theta=pitch（绕+Y），psi=yaw（绕+Z）；角度零点沿用姿态模块。
 * v 是沿车体+X的有符号速度估计，x/y 为轮轴中点估计位置。
 * 编码器速度/行程均加回车身俯仰分量；底层轮反馈保持原值。
 * 必须与 K 的单位、符号、参考点一致；本模块不执行控制。 */
typedef struct {
    float x_m, y_m, theta_rad, psi_rad, v_mps;
    float theta_dot_radps, psi_dot_radps;
    uint32_t timestamp_us, sequence;
    bool valid;
    /* 本次成功发布实际使用的三个来源，不随上游独立更新。 */
    SampleSource motion, imu, motor;
} StatusState;

enum {
    STATUS_STATE_MOTOR_INVALID=1u<<0,
    STATUS_STATE_ATTITUDE_INVALID=1u<<1,
    STATUS_STATE_TIME_MISMATCH=1u<<2,
    STATUS_STATE_POSITION_DISCONTINUOUS=1u<<3,
    STATUS_STATE_STALE=1u<<4
};
typedef enum {
    STATUS_STATE_RUNNING,
    STATUS_STATE_NOT_INITIALIZED,
    STATUS_STATE_WAITING_INPUT,
    STATUS_STATE_UNAVAILABLE,
} StatusStateRunState;

typedef struct {
    //记录运行状态与错误原因
    StatusStateRunState state;
    uint32_t faults;
} StatusStateAppStatus;

/* Init 在 MotionApp/MotorApp 初始化后调用，独占同步电机反馈回调。
 * 电机采样由调度层负责；由 MotorApp_Poll 接收采样请求。
 * 每帧反馈（包括停机补采样）同步进入 Poll，避免漏掉位移增量。
 * 主循环也应持续调用 Poll，检查失效；无新输入不重复计算。
 * 所有入口在主循环上下文使用，不支持中断调用或并发访问。
 * 轮半径取现有 MotorConfig；仅校验并复制，不修改电机配置。 */
bool StatusStateApp_Init(float left_radius_m, float right_radius_m);
void StatusStateApp_Poll(uint32_t now_us);
const StatusState *StatusStateApp_GetState(void);
const StatusStateAppStatus *StatusStateApp_GetStatus(void);
/* 按 [x,y,theta,psi,v,theta_dot,psi_dot] 输出；无效时不修改 out。 */
bool StatusStateApp_GetVector(float out[7]);
#endif
