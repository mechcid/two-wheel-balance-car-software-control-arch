#include "debug_app.h"
#include "lqr_app.h"

static Trace *lqr_trace, *imu_trace;
static bool initialized, trial_active;
static uint32_t trial_started_us;
static LqrTuning saved_tuning;
bool DebugApp_Init(LqrTrace *lqr, ImuTrace *imu)
{
    if (initialized || !lqr || !imu || !lqr->recorder.initialized || !imu->recorder.initialized) return false;
    lqr_trace=&lqr->recorder; imu_trace=&imu->recorder;
    initialized=true; return true;
}
Trace *DebugApp_GetTrace(DebugTraceSource source)
{
    return source==DEBUG_TRACE_LQR ? lqr_trace : source==DEBUG_TRACE_IMU ? imu_trace : NULL;
}
bool DebugApp_StartTrial(uint32_t now, bool soft)
{
    const volatile TraceStatus *s=TraceApp_GetStatus(lqr_trace);
    if (!initialized || trial_active || s->source_id!=TRACE_SOURCE_LQR ||
        s->mode==TRACE_ARMED || s->mode==TRACE_RECORDING || LqrApp_GetStatus()->mode!=LQR_DISABLED)
        return false;
    saved_tuning=LqrApp_GetTuning();
    LqrTuning trial=saved_tuning;
    if (soft) trial.pitch_rate_scale=.70f;
    if (!LqrApp_SetTuning(&trial)) return false;
    if (!LqrApp_Start(now)) { LqrApp_SetTuning(&saved_tuning); return false; }
    if (!TraceApp_Arm(lqr_trace)) { LqrApp_Stop(now); LqrApp_SetTuning(&saved_tuning); return false; }
    trial_started_us=now; trial_active=true;
    return true;
}
static void restore_tuning(void)
{
    if (!trial_active || LqrApp_IsActive()) return;
    /* 故障锁存仍由 LQR 保留；调试参数可在故障状态恢复，不改变业务状态。 */
    if (LqrApp_SetTuning(&saved_tuning)) trial_active=false;
}
void DebugApp_Stop(uint32_t now)
{
    TraceApp_Poll(lqr_trace,now); /* 在清除业务故障之前先保存异常现场。 */
    LqrApp_Stop(now);
    TraceApp_Poll(lqr_trace,now);
    restore_tuning();
}
void DebugApp_Poll(uint32_t now)
{
    if (!initialized) return;
    TraceApp_Poll(imu_trace,now);
    TraceApp_Poll(lqr_trace,now);
    if (trial_active && LqrApp_IsActive() && (uint32_t)(now-trial_started_us)>=DEBUG_LQR_TRIAL_DURATION_MS*1000U) {
        LqrApp_Stop(now); TraceApp_Poll(lqr_trace,now);
    }
    restore_tuning();
}
