#include "status_state_app.h"
#include "odometry_app.h"
#include "motor_app.h"
#include "motion_app.h"
#include "sensor_app.h"
#include <math.h>
#include <stddef.h>

//这个命名可能有点难以理解，StatusState是状态空间，StatusStateAppStatus则是状态空间模块的工作状态
static StatusState state;
static StatusStateAppStatus status={.state=STATUS_STATE_NOT_INITIALIZED};
/* 输入消费记录：失败帧也要记住，避免重复处理。 */
static bool has_consumed_input;
static uint32_t last_motor_sequence;
/* 仅在同步回调期间借用；Poll 返回前消费，不跨回调保存。 */
static const MotorAppStatus *pending_input;

/* 输出历史：首次成功发布也建立连续航向的起点。
 * 不以 sequence != 0 判断，避免序号回绕时误认为从未发布。 */
static bool has_published_sample;
static float last_wrapped_yaw;

/* 计算历史：输入失效后，旧俯仰不能用于下一次增量补偿。 */
static OdometryState odometry;
static bool pitch_history_valid;
static float last_pitch_rad;
/* 配置参数。 */
static float left_radius_m, right_radius_m;
/* 200 Hz 姿态相对电机采样允许最多 7.5 ms 延迟；状态超过 20 ms 禁用。
 * 这是有界时间偏差，不是硬件同步；不外推旧姿态来冒充新测量。 */
static const uint32_t max_attitude_age_us=7500, state_timeout_us=20000;

/* 单次更新：失败也消费输入并维护里程计连续性，但不发布新样本。 */
static void StatusStateApp_Update(const MotorAppStatus *m){
    //拿到两个模块维护变量的指针，用于提取三轴角度，或者说姿态，和角速度
    const MotionSample *pose=MotionApp_GetSample();
    const ImuSample *imu=SensorApp_GetSample();
    //用于记录错误状态，方便排查
    uint32_t faults=0;
    //依旧是合法性检查
    if (!m->valid || !m->left.feedback_valid || !m->right.feedback_valid) faults|=STATUS_STATE_MOTOR_INVALID;
    if (!pose->valid || pose->discontinuity || !imu->valid)
        faults|=STATUS_STATE_ATTITUDE_INVALID;
    if (pose->imu_source.sequence!=imu->sequence ||
        pose->imu_source.timestamp_us!=imu->timestamp_us ||
        pose->timestamp_us!=pose->imu_source.timestamp_us ||
        (uint32_t)(m->timestamp_us-pose->timestamp_us)>max_attitude_age_us)
        faults|=STATUS_STATE_TIME_MISMATCH;

    //开始提取数据进行处理
    const EulerAnglesDeg angles=pose->data;
    const float radians=0.017453292519943295f;//弧度与角度的转换关系
    float roll=angles.roll*radians, theta=angles.pitch*radians, psi=angles.yaw*radians;
    float q=imu->data.gyro[1]*radians, r=imu->data.gyro[2]*radians;
    // ZYX 欧拉角导数；不要把机体系 gyro Y/Z 直接当作欧拉角导数。
    float theta_dot=q*cosf(roll)-r*sinf(roll);
    float cp=cosf(theta);
    float psi_dot=(fabsf(cp)>0.1f) ? (q*sinf(roll)+r*cosf(roll))/cp : NAN;
    if (!isfinite(roll) || !isfinite(theta) || !isfinite(psi) ||
        !isfinite(theta_dot) || !isfinite(psi_dot)) faults|=STATUS_STATE_ATTITUDE_INVALID;
    // 编码器是相对车身的轮转角：绝对轮角 = 编码器轮角 + theta。
    // 位置用角度增量修正，不把末端角速度乘 dt 冒充整个区间的转角。
    bool pitch_ready=pitch_history_valid && has_consumed_input &&
        (uint32_t)(m->sequence-last_motor_sequence)==1;
    float delta_pitch=pitch_ready ? theta-last_pitch_rad : 0;
    float left_step=m->left.delta_displacement_m+left_radius_m*delta_pitch;
    float right_step=m->right.delta_displacement_m+right_radius_m*delta_pitch;
    float left_v=m->left.velocity_mps+left_radius_m*theta_dot;
    float right_v=m->right.velocity_mps+right_radius_m*theta_dot;
    OdometryApp_Update(&odometry,left_step,right_step,left_v,right_v,
        psi,m->sequence,m->timestamp_us,m->interval_us,faults==0);
    pitch_history_valid=faults==0;
    if (pitch_history_valid) last_pitch_rad=theta;
    if (!odometry.initialized || !odometry.continuous || !odometry.valid)
        faults|=STATUS_STATE_POSITION_DISCONTINUOUS;
    last_motor_sequence=m->sequence;
    has_consumed_input=true;
    status.faults=faults;
    if (faults) {
        state.valid=false;
        status.state=STATUS_STATE_UNAVAILABLE;
        return;
    }
    StatusState next={
        .x_m=odometry.x_m, .y_m=odometry.y_m, .v_mps=odometry.v_mps,
        .theta_rad=theta,
        .psi_rad=has_published_sample ? state.psi_rad+atan2f(sinf(psi-last_wrapped_yaw),
                                               cosf(psi-last_wrapped_yaw)) : psi,
        .theta_dot_radps=theta_dot, .psi_dot_radps=psi_dot,
        .timestamp_us=m->timestamp_us, .sequence=state.sequence+1,
        .motion={.timestamp_us=pose->timestamp_us, .sequence=pose->sequence},
        .imu={.timestamp_us=imu->timestamp_us, .sequence=imu->sequence},
        .motor={.timestamp_us=m->timestamp_us, .sequence=m->sequence},
        .valid=true
    };
    last_wrapped_yaw=psi;
    state=next;
    has_published_sample=true;
    status.state=STATUS_STATE_RUNNING;
}
static void on_motor_feedback(const MotorAppStatus *m)
{
    /* 不等待下一轮主循环：停机补采样也必须在覆盖前消费。 */
    pending_input=m;
    StatusStateApp_Poll(m->timestamp_us);
    pending_input=NULL;
}

bool StatusStateApp_Init(float left_radius, float right_radius)
{
    if (!isfinite(left_radius) || left_radius<=0 ||
        !isfinite(right_radius) || right_radius<=0) return false;
    left_radius_m=left_radius; right_radius_m=right_radius;
    pitch_history_valid=false; last_pitch_rad=0;
    state=(StatusState){0}; has_published_sample=false; last_wrapped_yaw=0;
    odometry=OdometryApp_Init();
    status=(StatusStateAppStatus){.state=STATUS_STATE_WAITING_INPUT};
    has_consumed_input=false;
    last_motor_sequence=0;
    pending_input=NULL;
    MotorApp_SetFeedbackCallback(on_motor_feedback);
    return true;
}
void StatusStateApp_Poll(uint32_t now)
{
    if (status.state==STATUS_STATE_NOT_INITIALIZED) return;
    if (pending_input && (!has_consumed_input || pending_input->sequence!=last_motor_sequence)) {
        StatusStateApp_Update(pending_input);
    }
    /* 等待首帧不是过期；失败原因由 Update 记录。 */
    if (!has_consumed_input) return;
    if (has_published_sample && (uint32_t)(now-state.timestamp_us)>state_timeout_us) {
        state.valid=false;
        status.faults|=STATUS_STATE_STALE;
    }
    if (!MotionApp_GetSample()->valid || MotionApp_GetSample()->discontinuity ||
        !SensorApp_GetSample()->valid) {
        state.valid=false;
        status.faults|=STATUS_STATE_ATTITUDE_INVALID;
        pitch_history_valid=false;
        if (odometry.initialized) { odometry.valid=false; odometry.continuous=false; }
    }
    if (!state.valid) status.state=STATUS_STATE_UNAVAILABLE;
}
const StatusState *StatusStateApp_GetState(void) { return &state; }
const StatusStateAppStatus *StatusStateApp_GetStatus(void) { return &status; }
bool StatusStateApp_GetVector(float out[7])
{
    if (!out || !state.valid) return false;
    out[0]=state.x_m; out[1]=state.y_m; out[2]=state.theta_rad; out[3]=state.psi_rad;
    out[4]=state.v_mps; out[5]=state.theta_dot_radps; out[6]=state.psi_dot_radps;
    return true;
}
