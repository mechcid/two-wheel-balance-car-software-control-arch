#ifndef ODOMETRY_APP_H
#define ODOMETRY_APP_H
#include <stdbool.h>
#include <stdint.h>
/* 平面无滑移模型，位置为轮轴中点；X/Y 与姿态参考系一致。 */
typedef struct {
    float x_m, y_m, v_mps;
    float delta_x_m, delta_y_m;
    bool valid;       /* 本次增量与速度有效。 */
    bool continuous;  /* 建立原点后是否一直完整；缺帧后不会自动恢复。 */
    bool initialized;
    uint32_t sequence, timestamp_us;
    float last_psi_rad;
} OdometryState;
void OdometryApp_Init(OdometryState *state);
/* 每份双轮反馈调用一次；第一次可信采样建立位置原点，不积分之前的行程。
 * 速度直接取左右轮线速度均值；valid 必须同时涵盖两轮增量和速度。
 * sequence 使用电机采样序号，interval_us 对应两轮增量的共同时间区间。 */
void OdometryApp_Update(OdometryState *state, float left_m, float right_m,
                        float left_velocity_mps, float right_velocity_mps,
                        float psi_rad, uint32_t sequence, uint32_t timestamp_us,
                        uint32_t interval_us, bool valid);
#endif
