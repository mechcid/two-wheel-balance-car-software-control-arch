#ifndef STATUS_STATE_APP_H
#define STATUS_STATE_APP_H
#include <stdbool.h>
#include <stdint.h>
/* theta=pitch（绕+Y），psi=yaw（绕+Z）；角度零点沿用姿态模块。
 * v 是沿车体+X的有符号速度估计，x/y 为轮轴中点估计位置。
 * 编码器速度/行程均加回车身俯仰分量；底层轮反馈保持原值。
 * 必须与 K 的单位、符号、参考点一致；本模块不执行控制。 */
typedef struct {
    float x_m, y_m, theta_rad, psi_rad, v_mps;
    float theta_dot_radps, psi_dot_radps;
    uint32_t timestamp_us, sequence;
    uint32_t attitude_timestamp_us, motor_sequence;
    uint32_t faults;
    bool valid;
} StatusState;
enum {
    STATUS_STATE_MOTOR_INVALID=1u<<0,
    STATUS_STATE_ATTITUDE_INVALID=1u<<1,
    STATUS_STATE_TIME_MISMATCH=1u<<2,
    STATUS_STATE_POSITION_DISCONTINUOUS=1u<<3,
    STATUS_STATE_STALE=1u<<4
};
/* Init 在 MotionApp/MotorApp 初始化后调用，独占电机反馈回调。
 * Update 是200 Hz 任务，负责触发双轮采样；不可再单独调 MotorApp_Update。
 * Poll 每轮主循环执行，失效旧状态，不为旧数据制造新序号。
 * 停机补采样也会发布，故 sequence 不等同于严格周期计数。 */
/* 轮半径直接取现有 MotorConfig；仅校验并复制，不修改电机配置。 */
bool StatusStateApp_Init(float left_radius_m, float right_radius_m);
void StatusStateApp_Update(uint32_t now_us);
void StatusStateApp_Poll(uint32_t now_us);
const StatusState *StatusStateApp_GetState(void);
/* 按 [x,y,theta,psi,v,theta_dot,psi_dot] 输出；无效时不修改 out。 */
bool StatusStateApp_GetVector(float out[7]);
#endif
