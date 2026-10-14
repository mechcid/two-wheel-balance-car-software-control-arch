#include "status_state_app.h"
#include "odometry_app.h"
#include "motor_app.h"
#include "motion_app.h"
#include "sensor_app.h"
#include <math.h>

static StatusState state;
static OdometryState odometry;
static bool have_sample, have_yaw;
static float last_wrapped_yaw;
static float left_radius_m, right_radius_m, last_pitch_rad;
static bool have_pitch;
/* 200 Hz 姿态相对电机采样允许最多 7.5 ms 延迟；状态超过 20 ms 禁用。
 * 这是有界时间偏差，不是硬件同步；不外推旧姿态来冒充新测量。 */
static const uint32_t max_attitude_age_us=7500, state_timeout_us=20000;
static void on_motor_feedback(const MotorAppStatus *m){
    if (have_sample && m->sequence==state.motor_sequence) return;
    const MotionState *pose=MotionApp_GetState();
    const MotionAppStatus *timing=MotionApp_GetStatus();
    const ImuSample *imu=SensorApp_GetSample();
    uint32_t faults=0;
    if (!m->valid || !m->left.feedback_valid || !m->right.feedback_valid) faults|=STATUS_STATE_MOTOR_INVALID;
    if (!pose->valid || pose->discontinuity || !imu->valid)
        faults|=STATUS_STATE_ATTITUDE_INVALID;
    if (timing->timestamp_us!=imu->timestamp_us ||
        (uint32_t)(m->timestamp_us-timing->timestamp_us)>max_attitude_age_us)
        faults|=STATUS_STATE_TIME_MISMATCH;
    float angles[3]; MotionApp_GetEuler(angles);
    const float radians=0.017453292519943295f;
    float roll=angles[0]*radians, theta=angles[1]*radians, psi=angles[2]*radians;
    float q=imu->data.gyro[1]*radians, r=imu->data.gyro[2]*radians;
    // ZYX 欧拉角导数；不要把机体系 gyro Y/Z 直接当作欧拉角导数。
    float theta_dot=q*cosf(roll)-r*sinf(roll);
    float cp=cosf(theta);
    float psi_dot=(fabsf(cp)>0.1f) ? (q*sinf(roll)+r*cosf(roll))/cp : NAN;
    if (!isfinite(roll) || !isfinite(theta) || !isfinite(psi) ||
        !isfinite(theta_dot) || !isfinite(psi_dot)) faults|=STATUS_STATE_ATTITUDE_INVALID;
    // 编码器是相对车身的轮转角：绝对轮角 = 编码器轮角 + theta。
    // 位置用角度增量修正，不把末端角速度乘 dt 冒充整个区间的转角。
    bool pitch_ready=have_pitch && have_sample &&
        (uint32_t)(m->sequence-state.motor_sequence)==1;
    float delta_pitch=pitch_ready ? theta-last_pitch_rad : 0;
    float left_step=m->left.delta_displacement_m+left_radius_m*delta_pitch;
    float right_step=m->right.delta_displacement_m+right_radius_m*delta_pitch;
    float left_v=m->left.velocity_mps+left_radius_m*theta_dot;
    float right_v=m->right.velocity_mps+right_radius_m*theta_dot;
    OdometryApp_Update(&odometry,left_step,right_step,left_v,right_v,
        psi,m->sequence,m->timestamp_us,m->interval_us,faults==0);
    have_pitch=faults==0;
    if (have_pitch) last_pitch_rad=theta;
    if (!odometry.initialized || !odometry.continuous || !odometry.valid)
        faults|=STATUS_STATE_POSITION_DISCONTINUOUS;
    state.x_m=odometry.x_m; state.y_m=odometry.y_m; state.v_mps=odometry.v_mps;
    // 无效帧保留数值便于诊断，但控制器必须检查整帧 valid。
    if (!(faults&STATUS_STATE_ATTITUDE_INVALID)) {
        state.theta_rad=theta;
        state.psi_rad=have_yaw ? state.psi_rad+atan2f(sinf(psi-last_wrapped_yaw),
                                                     cosf(psi-last_wrapped_yaw)) : psi;
        last_wrapped_yaw=psi; have_yaw=true;
        state.theta_dot_radps=theta_dot; state.psi_dot_radps=psi_dot;
    }
    state.timestamp_us=m->timestamp_us;
    state.attitude_timestamp_us=timing->timestamp_us;
    state.motor_sequence=m->sequence;
    ++state.sequence;
    state.faults=faults;
    state.valid=faults==0;
    have_sample=true;
}
bool StatusStateApp_Init(float left_radius, float right_radius)
{
    if (!isfinite(left_radius) || left_radius<=0 ||
        !isfinite(right_radius) || right_radius<=0) return false;
    left_radius_m=left_radius; right_radius_m=right_radius;
    have_pitch=false; last_pitch_rad=0;
    state=(StatusState){0}; have_sample=false; have_yaw=false; last_wrapped_yaw=0;
    OdometryApp_Init(&odometry);
    MotorApp_SetFeedbackCallback(on_motor_feedback);
    return true;
}
void StatusStateApp_Update(uint32_t now) { MotorApp_Update(now); StatusStateApp_Poll(now); }
void StatusStateApp_Poll(uint32_t now)
{
    if (!have_sample || (uint32_t)(now-state.timestamp_us)>state_timeout_us) {
        state.valid=false;
        state.faults|=STATUS_STATE_STALE;
    }
    if (!MotionApp_GetState()->valid || MotionApp_GetState()->discontinuity ||
        !SensorApp_GetSample()->valid) {
        state.valid=false;
        state.faults|=STATUS_STATE_ATTITUDE_INVALID;
        have_pitch=false;
        if (odometry.initialized) { odometry.valid=false; odometry.continuous=false; }
    }
}
const StatusState *StatusStateApp_GetState(void) { return &state; }
bool StatusStateApp_GetVector(float out[7])
{
    if (!out || !state.valid) return false;
    out[0]=state.x_m; out[1]=state.y_m; out[2]=state.theta_rad; out[3]=state.psi_rad;
    out[4]=state.v_mps; out[5]=state.theta_dot_radps; out[6]=state.psi_dot_radps;
    return true;
}
