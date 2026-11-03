#include "lqr_app.h"
#include "motor_app.h"
#include <math.h>
#include <string.h>

static LqrConfig config;
static LqrStatus status;
static LqrSample sample;

static uint32_t last_state_sequence;
static bool initialized;
static LqrTuning tuning;
static uint32_t last_control_us, last_sample_us;
static const uint32_t max_age_us=7500U, min_period_us=2500U, max_period_us=7500U;
static float wrap(float a) { return atan2f(sinf(a),cosf(a)); }

//检查输入的lqr配置的合法性
static bool config_valid(const LqrConfig *c){
    return c && isfinite(c->actuator.battery_v) && c->actuator.battery_v>0 &&
        isfinite(c->limits.duty_limit) && c->limits.duty_limit>0 && c->limits.duty_limit<=1 &&
        isfinite(c->limits.start_angle_rad) && c->limits.start_angle_rad>0 &&
        isfinite(c->limits.stop_angle_rad) && c->limits.stop_angle_rad>c->limits.start_angle_rad &&
        c->limits.stop_angle_rad<1.0f && isfinite(c->limits.max_position_m) && c->limits.max_position_m>0 &&
        isfinite(c->limits.max_yaw_rad) && c->limits.max_yaw_rad>0 && c->limits.max_yaw_rad<1.0f;
}
//使用的默认配置
LqrConfig LqrApp_DefaultConfig(void){
    return (LqrConfig){
        .actuator={
            .battery_v=12},
        .limits={
            .duty_limit=.40f,
            .start_angle_rad=.10f,
            .stop_angle_rad=.35f,
            .max_position_m=.25f,
            .max_yaw_rad=.50f}
        };
}
static bool tuning_valid(const LqrTuning *t)
{ return t && isfinite(t->pitch_rate_scale) && t->pitch_rate_scale>=.5f && t->pitch_rate_scale<=1 &&
    isfinite(t->yaw_rate_scale) && t->yaw_rate_scale>=.5f && t->yaw_rate_scale<=1; }
LqrTuning LqrApp_DefaultTuning(void) { return (LqrTuning){.pitch_rate_scale=.5f,.yaw_rate_scale=.5f}; }
LqrTuning LqrApp_GetTuning(void) { return tuning; }
bool LqrApp_SetTuning(const LqrTuning *t)
{
    if (!initialized || status.mode==LQR_RUNNING || !tuning_valid(t)) return false;
    tuning=*t; return true;
}
//进行lqr计算的函数，输出控制力矩
bool LqrApp_Calculate(const StatusState *s, const TrajectoryPoint *hold,
                     const float rpm[2], const LqrConfig *c, const LqrTuning *t, LqrResult *out){
    //合法性检测
    if (!s || !hold || !rpm || !out || !s->valid || !config_valid(c) || !tuning_valid(t)) return false;
    if (!isfinite(hold->x_m) || !isfinite(hold->y_m) || !isfinite(hold->psi_rad)) return false;
    float e[6]={cosf(hold->psi_rad)*(s->x_m-hold->x_m)+sinf(hold->psi_rad)*(s->y_m-hold->y_m),
        wrap(s->psi_rad-hold->psi_rad),s->theta_rad,s->v_mps*cosf(s->psi_rad-hold->psi_rad),
        s->psi_dot_radps,s->theta_dot_radps};
    for (unsigned j=0;j<6;++j) if (!isfinite(e[j])) return false;
    float torque[2], duty[2], estimated[2];
    bool saturated=false;
    for (unsigned i=0;i<2;++i) {
        if (!isfinite(rpm[i])) return false;
        torque[i]=0;
        for (unsigned j=0;j<6;++j) torque[i]-=lqr_k[i][j]*e[j]*(j==5 ? t->pitch_rate_scale : j==4 ? t->yaw_rate_scale : 1.0f);
        /* Back EMF uses speed relative to the motor housing; no pitch correction. */
        float w=rpm[i]*0.10471975512f;
        float raw=(torque[i]+0.01139f*w)/(0.03677f*c->actuator.battery_v);
        if (!isfinite(raw) || !isfinite(torque[i])) return false;
        duty[i]=fmaxf(-c->limits.duty_limit,fminf(c->limits.duty_limit,raw));
        saturated |= raw!=duty[i];
        estimated[i]=0.03677f*c->actuator.battery_v*duty[i]-0.01139f*w;
        if (!isfinite(estimated[i])) return false;
    }
    memcpy(out->error,e,sizeof e); memcpy(out->requested_nm,torque,sizeof torque);
    memcpy(out->duty,duty,sizeof duty); memcpy(out->estimated_nm,estimated,sizeof estimated);
    out->saturated=saturated;
    return true;
}
bool LqrApp_Init(const LqrConfig *c, const LqrTuning *t)
{
    if (initialized || !config_valid(c) || !tuning_valid(t)) return false;
    config=*c; tuning=*t; status=(LqrStatus){0}; initialized=true;
    return true;
}
static bool fresh(const StatusState *s, uint32_t now){
    return s->valid && (uint32_t)(now-s->timestamp_us)<=max_age_us &&
        (uint32_t)(now-s->motion.timestamp_us)<=max_age_us;
}

/* 事件状态独立于历史成功样本；先切换模式再停机，防止同步反馈重入控制。 */
static void fault(uint32_t reason, uint32_t now){
    status.mode=LQR_FAULT;
    status.faults|=reason;
    status.event_timestamp_us=now;
    sample.valid=false;
    MotorApp_Stop(now);
}
static bool matching_motor(const StatusState *s,const MotorAppSample *m) {
    return m->valid && m->left.feedback_valid && m->right.feedback_valid &&
        s->motor.sequence==m->sequence && s->motor.timestamp_us==m->timestamp_us;
}
static bool reference_valid(const TrajectorySample *r,uint32_t now) {
    return r->valid && r->valid_for_us>0 && r->valid_for_us<=TRAJECTORY_VALID_FOR_US &&
        (uint32_t)(now-r->timestamp_us)<=r->valid_for_us &&
        isfinite(r->data.x_m) && isfinite(r->data.y_m) && isfinite(r->data.psi_rad);
}
bool LqrApp_Start(uint32_t now){
    if (!initialized || status.mode!=LQR_DISABLED || MotorApp_GetStatus()->mode!=MOTOR_STOPPED)
        return false;
    const StatusState *s=StatusStateApp_GetState();
    const MotorAppSample *m=MotorApp_GetSample();
    if (!fresh(s,now) || !matching_motor(s,m) || fabsf(s->theta_rad)>config.limits.start_angle_rad)
        return false;
    const TrajectorySample *reference=TrajectoryApp_GetSample();
    if (!reference_valid(reference,now)) return false;
    LqrResult candidate;
    float rpm[2]={m->left.rpm,m->right.rpm};
    if (!LqrApp_Calculate(s,&reference->data,rpm,&config,&tuning,&candidate)) return false;
    if (fabsf(candidate.error[0])>config.limits.max_position_m ||
        fabsf(candidate.error[1])>config.limits.max_yaw_rad) return false;
    status=(LqrStatus){.mode=LQR_RUNNING,.event_timestamp_us=now};
    sample.valid=false;
    last_state_sequence=s->sequence;
    last_control_us=now; last_sample_us=s->timestamp_us;
    // 首次输出等待下一份完整状态，不发布启动校验的候选结果。
    return true;
}
void LqrApp_Stop(uint32_t now){
    status.mode=LQR_DISABLED; status.faults=0; status.event_timestamp_us=now;
    sample.valid=false;
    MotorApp_Stop(now);
}
static void LqrApp_Update(uint32_t now,const StatusState *s){
    uint32_t dt=s->timestamp_us-last_sample_us;
    if (dt<min_period_us || dt>max_period_us) { fault(LQR_TIMING,now); return; }


    const MotorAppSample *m=MotorApp_GetSample();
    float rpm[2]={m->left.rpm,m->right.rpm};
    const TrajectorySample *reference=TrajectoryApp_GetSample();
    LqrSample next={0};
    if (!matching_motor(s,m) || !LqrApp_Calculate(s,&reference->data,rpm,&config,&tuning,&next.data)) {
        fault(LQR_BAD_STATE,now); return;
    }
    if (fabsf(next.data.error[0])>config.limits.max_position_m ||
        fabsf(next.data.error[1])>config.limits.max_yaw_rad || fabsf(next.data.error[2])>config.limits.stop_angle_rad) {
        fault(LQR_RANGE,now); return;
    }
    // 在执行前保存输入身份：输出失败可能同步触发停机补采样，改变上游数据。
    next.timestamp_us=now; next.sequence=sample.sequence+1;
    next.state_source=(SampleSource){s->timestamp_us,s->sequence};
    next.motor_source=(SampleSource){m->timestamp_us,m->sequence};
    next.dt_us=dt; next.data_age_us=now-s->motion.timestamp_us;
    next.reference=reference->data; next.tuning=tuning;
    next.reference_source=(SampleSource){reference->timestamp_us,reference->sequence};
    next.relative_rpm[0]=rpm[0]; next.relative_rpm[1]=rpm[1];
    
    if (!MotorApp_ApplyDuty(next.data.duty[0],next.data.duty[1],now)) { fault(LQR_OUTPUT,now); return; }
    next.valid=true;
    sample=next;
    last_sample_us=next.state_source.timestamp_us;
    last_state_sequence=next.state_source.sequence;
    last_control_us=now;
    status.has_output=true;
}
void LqrApp_Poll(uint32_t now){
    if (!initialized || status.mode!=LQR_RUNNING) return;
    const StatusState *s=StatusStateApp_GetState();
    if (!fresh(s,now)) { fault(LQR_BAD_STATE,now); return; }
    if ((uint32_t)(now-last_control_us)>max_period_us) { fault(LQR_TIMING,now); return; }
    if (MotorApp_GetStatus()->mode==MOTOR_FAULT ||
        (status.has_output && MotorApp_GetStatus()->mode!=MOTOR_DUTY)) { fault(LQR_OUTPUT,now); return; }
    if (!reference_valid(TrajectoryApp_GetSample(),now)) { fault(LQR_BAD_REFERENCE,now); return; }
    if (s->sequence!=last_state_sequence) LqrApp_Update(now,s);
}
bool LqrApp_IsActive(void) { return status.mode==LQR_RUNNING; }
const LqrStatus *LqrApp_GetStatus(void) { return &status; }
const LqrSample *LqrApp_GetSample(void) { return &sample; }
