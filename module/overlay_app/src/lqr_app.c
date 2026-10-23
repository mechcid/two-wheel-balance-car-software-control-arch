#include "lqr_app.h"
#include "motor_app.h"
#include <math.h>
#include <string.h>

static LqrConfig config;
static LqrStatus status;
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
    if (!initialized || status.mode==LQR_HOLD || !tuning_valid(t)) return false;
    tuning=*t; return true;
}
//进行lqr计算的函数，输出控制力矩
bool LqrApp_Calculate(const StatusState *s, const float hold[3],
                     const float rpm[2], const LqrConfig *c, const LqrTuning *t, LqrStatus *out){
    //合法性检测
    if (!s || !hold || !rpm || !out || !s->valid || !config_valid(c) || !tuning_valid(t)) return false;
    for (unsigned i=0;i<3;++i) if (!isfinite(hold[i])) return false;
    float e[6]={cosf(hold[2])*(s->x_m-hold[0])+sinf(hold[2])*(s->y_m-hold[1]),
        wrap(s->psi_rad-hold[2]),s->theta_rad,s->v_mps*cosf(s->psi_rad-hold[2]),
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
        (uint32_t)(now-s->attitude_timestamp_us)<=max_age_us;
}

//进行错误检测的函数
static void fault(uint32_t reason, uint32_t now){
    status.mode=LQR_FAULT; status.faults|=reason; status.data_valid=false;
    /* Set mode before Stop, which can synchronously publish another state frame. */
    MotorApp_Stop(now);
    status.duty[0]=status.duty[1]=0;
    status.event_timestamp_us=now;
}
bool LqrApp_Start(uint32_t now){
    if (!initialized || status.mode!=LQR_DISABLED || MotorApp_GetStatus()->mode!=MOTOR_STOPPED)
        return false;
    const StatusState *s=StatusStateApp_GetState();
    const MotorAppStatus *m=MotorApp_GetStatus();
    if (!fresh(s,now) || !m->valid || !m->left.feedback_valid || !m->right.feedback_valid ||
        s->motor_sequence!=m->sequence || fabsf(s->theta_rad)>config.limits.start_angle_rad) return false;
    LqrStatus next={0};
    next.hold[0]=s->x_m; next.hold[1]=s->y_m; next.hold[2]=s->psi_rad;
    float rpm[2]={m->left.rpm,m->right.rpm};
    if (!LqrApp_Calculate(s,next.hold,rpm,&config,&tuning,&next)) return false;
    next.mode=LQR_HOLD; next.state_sequence=s->sequence;
    next.duty[0]=next.duty[1]=0; /* No output has been applied yet. */
    next.sequence=status.sequence; next.event_timestamp_us=now;
    status=next; last_control_us=now; last_sample_us=s->timestamp_us;
    /* First output waits for the next complete scheduled state, never a stale frame. */
    return true;
}
void LqrApp_Stop(uint32_t now)
{
    status.mode=LQR_DISABLED; status.faults=0; status.data_valid=false;
    MotorApp_Stop(now); status.duty[0]=status.duty[1]=0;
    status.event_timestamp_us=now;
}
void LqrApp_Poll(uint32_t now)
{
    if (status.mode!=LQR_HOLD) return;
    if (!fresh(StatusStateApp_GetState(),now)) fault(LQR_BAD_STATE,now);
    else if ((uint32_t)(now-last_control_us)>max_period_us) fault(LQR_TIMING,now);
    else if (MotorApp_GetStatus()->mode==MOTOR_FAULT ||
             (status.updates && MotorApp_GetStatus()->mode!=MOTOR_DUTY)) fault(LQR_OUTPUT,now);
}
void LqrApp_Update(uint32_t now)
{
    LqrApp_Poll(now);
    if (status.mode!=LQR_HOLD) return;
    const StatusState *s=StatusStateApp_GetState();
    if (s->sequence==status.state_sequence) return;
    uint32_t dt=s->timestamp_us-last_sample_us;
    if (dt<min_period_us || dt>max_period_us) { fault(LQR_TIMING,now); return; }
    const MotorAppStatus *m=MotorApp_GetStatus();
    float rpm[2]={m->left.rpm,m->right.rpm};
    if (!m->valid || !m->left.feedback_valid || !m->right.feedback_valid ||
        s->motor_sequence!=m->sequence || !LqrApp_Calculate(s,status.hold,rpm,&config,&tuning,&status)) {
        fault(LQR_BAD_STATE,now); return;
    }
    if (fabsf(status.error[0])>config.limits.max_position_m ||
        fabsf(status.error[1])>config.limits.max_yaw_rad || fabsf(status.error[2])>config.limits.stop_angle_rad) {
        fault(LQR_RANGE,now); return;
    }
    status.dt_us=dt; status.timestamp_us=now; status.data_age_us=now-s->attitude_timestamp_us;
    status.state_sequence=s->sequence;
    last_sample_us=s->timestamp_us;
    if (!MotorApp_ApplyDuty(status.duty[0],status.duty[1],now)) { fault(LQR_OUTPUT,now); return; }
    last_control_us=now; ++status.updates; status.data_valid=true;
    ++status.sequence;
    status.input_timestamp_us=s->timestamp_us;
    status.relative_rpm[0]=rpm[0]; status.relative_rpm[1]=rpm[1];
    status.pitch_rate_scale=tuning.pitch_rate_scale;
    status.yaw_rate_scale=tuning.yaw_rate_scale;
}
bool LqrApp_IsActive(void) { return status.mode==LQR_HOLD; }
const LqrStatus *LqrApp_GetStatus(void) { return &status; }
