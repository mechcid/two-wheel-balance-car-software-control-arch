#include "motion_estimator.h"
#include <math.h>
#include <string.h>

static float norm3(const float v[3])
{
    return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}
static float clamp(float x, float lo, float hi)
{
    return fmaxf(lo, fminf(hi, x));
}
/* 四元数对应旋转矩阵的第三行，即参考系竖直方向在机体系中的表示。 */
static void gravity_body(const float q[4], float g[3])
{
    g[0] = 2*(q[1]*q[3]-q[0]*q[2]);
    g[1] = 2*(q[0]*q[1]+q[2]*q[3]);
    g[2] = 1-2*(q[1]*q[1]+q[2]*q[2]);
}
MotionConfig MotionEstimator_DefaultConfig(void)
{
    return (MotionConfig){.accel_sign=-1,
        .correction_gain=2, .accel_cutoff_hz=10,
        .accel_tolerance_g=0.12f, .max_dt_s=0.05f};
}
bool MotionEstimator_Init(MotionEstimator *e, const MotionConfig *config)
{
    if (!e) return false;
    MotionConfig c=config ? *config : MotionEstimator_DefaultConfig();
    if (!isfinite(c.accel_sign) || fabsf(c.accel_sign)!=1 ||
        !isfinite(c.correction_gain) || c.correction_gain<0 || c.correction_gain>1000 ||
        !isfinite(c.accel_cutoff_hz) || c.accel_cutoff_hz<=0 || c.accel_cutoff_hz>10000 ||
        !isfinite(c.accel_tolerance_g) || c.accel_tolerance_g<=0 || c.accel_tolerance_g>=1 ||
        !isfinite(c.max_dt_s) || c.max_dt_s<=0 || c.max_dt_s>1) return false;
    *e=(MotionEstimator){.config=c, .state={.q={1,0,0,0}}};
    return true;
}
void MotionEstimator_Invalidate(MotionEstimator *e)
{
    if (!e) return;
    e->state.valid=false;
    e->state.accel_trusted=false;
    e->state.discontinuity=true;
}
bool MotionEstimator_Update(MotionEstimator *e, const float accel[3], const float gyro[3], float dt)
{
    if (!e) return false;
    if (!accel || !gyro) { MotionEstimator_Invalidate(e); return false; }
    MotionState *s=&e->state;
    for (unsigned i=0;i<3;++i) {
        if (!isfinite(accel[i]) || !isfinite(gyro[i]) || fabsf(accel[i])>16 || fabsf(gyro[i])>2000) {
            MotionEstimator_Invalidate(e); return false;
        }
    }
    if (!isfinite(dt) || dt<0 || dt>e->config.max_dt_s) {
        MotionEstimator_Invalidate(e);
        return false;
    }
    float a[3], omega[3];
    for (unsigned i=0;i<3;++i) {
        a[i]=e->config.accel_sign*accel[i];
        omega[i]=gyro[i]*0.01745329252f;
    }
    /* 首次及缺样后重建倾角，恢复时保留上次航向；不补算缺失的转动。 */
    bool first=!s->valid;
    float magnitude=norm3(a);
    s->accel_trusted=fabsf(magnitude-1)<e->config.accel_tolerance_g;
    if (first) {
        if (!s->accel_trusted) return false;
        float roll=atan2f(a[1],a[2]);
        float pitch=atan2f(-a[0],sqrtf(a[1]*a[1]+a[2]*a[2]));
        /* 首次启动航向为零；恢复后沿用上一帧航向。 */
        float old_angles[3];
        MotionEstimator_GetEuler(e,old_angles);
        float yaw=old_angles[2]*0.01745329252f;
        float cr=cosf(roll/2),sr=sinf(roll/2),cp=cosf(pitch/2),sp=sinf(pitch/2);
        float cy=cosf(yaw/2),sy=sinf(yaw/2);
        s->q[0]=cr*cp*cy+sr*sp*sy; s->q[1]=sr*cp*cy-cr*sp*sy;
        s->q[2]=cr*sp*cy+sr*cp*sy; s->q[3]=cr*cp*sy-sr*sp*cy;
        memcpy(e->gravity_lpf,a,sizeof(a));
    } else {
        if (dt==0) {
            MotionEstimator_Invalidate(e);
            return false;
        }
        float k=6.28318530718f*e->config.accel_cutoff_hz*dt;
        float gain=k/(1+k);
        for (unsigned i=0;i<3;++i) e->gravity_lpf[i]+=gain*(a[i]-e->gravity_lpf[i]);
        float measured[3], predicted[3];
        float n=norm3(e->gravity_lpf);
        gravity_body(s->q,predicted);
        float dot=0;
        for (unsigned i=0;i<3;++i) {
            measured[i]=n>0.001f ? e->gravity_lpf[i]/n : 0;
            dot+=measured[i]*predicted[i];
        }
        /* 模长和方向残差双重限制，明显运动时减弱加速度对倾角的误导。
         * 匀加速仍可能与倾斜无法区分，这不是门限能彻底解决的问题。 */
        s->accel_trusted=s->accel_trusted && n>0.001f && dot>0.8660254f;
        if (s->accel_trusted) {
            float error[3]={measured[1]*predicted[2]-measured[2]*predicted[1],
                measured[2]*predicted[0]-measured[0]*predicted[2],
                measured[0]*predicted[1]-measured[1]*predicted[0]};
            for (unsigned i=0;i<3;++i) omega[i]+=e->config.correction_gain*error[i];
        }
        float w=s->q[0],x=s->q[1],y=s->q[2],z=s->q[3],h=dt/2;
        s->q[0]+=(-x*omega[0]-y*omega[1]-z*omega[2])*h;
        s->q[1]+=(w*omega[0]+y*omega[2]-z*omega[1])*h;
        s->q[2]+=(w*omega[1]-x*omega[2]+z*omega[0])*h;
        s->q[3]+=(w*omega[2]+x*omega[1]-y*omega[0])*h;
        float inv=1/sqrtf(s->q[0]*s->q[0]+s->q[1]*s->q[1]+s->q[2]*s->q[2]+s->q[3]*s->q[3]);
        for (unsigned i=0;i<4;++i) s->q[i]*=inv;
    }
    s->valid=true;
    return true;
}
void MotionEstimator_GetEuler(const MotionEstimator *e, float out[3])
{
    if (!e || !out) return;
    const float *q=e->state.q;
    out[0]=atan2f(2*(q[0]*q[1]+q[2]*q[3]),1-2*(q[1]*q[1]+q[2]*q[2]))*57.29577951f;
    out[1]=asinf(clamp(2*(q[0]*q[2]-q[3]*q[1]),-1,1))*57.29577951f;
    out[2]=atan2f(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]))*57.29577951f;
}
