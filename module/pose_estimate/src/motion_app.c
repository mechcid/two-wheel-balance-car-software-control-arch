#include "motion_app.h"
#include "sensor_app.h"
#include "timebase.h"

static MotionEstimator estimator;
static MotionAppStatus status;
static uint32_t last_poll_us, last_sample_us, last_sequence;
static bool started, have_time;

bool MotionApp_Init(const MotionConfig *config)
{
    if (started || !MotionEstimator_Init(&estimator,config)) return false;
    last_poll_us=Timebase_NowUs();
    started=true;
    return true;
}
uint32_t MotionApp_TimeUntilSampleUs(void)
{
    if (!started) return 0;
    uint32_t elapsed=Timebase_NowUs()-last_poll_us;
    return elapsed<IMU_PERIOD_US ? IMU_PERIOD_US-elapsed : 0;
}
void MotionApp_Poll(void)
{
    if (!started) return;
    uint32_t now=Timebase_NowUs();
    if (have_time && (uint32_t)(now-last_sample_us)>=IMU_TIMEOUT_US) {
        MotionEstimator_Invalidate(&estimator);
        have_time=false;
    }
    uint32_t periods=(uint32_t)(now-last_poll_us)/IMU_PERIOD_US;
    if (!periods) return;
    /* 按原截止时间推进；积压只读取最新值，不补读历史帧。 */
    last_poll_us+=periods*IMU_PERIOD_US;
    status.missed_periods+=periods-1;
    HAL_StatusTypeDef result=SensorApp_Update();
    status.sensor_hz=SensorApp_GetOutputRate();
    if (result==HAL_BUSY) { ++status.no_data; return; }
    if (result!=HAL_OK) {
        ++status.failures;
        MotionEstimator_Invalidate(&estimator);
        have_time=false;
        return;
    }
    const ImuSample *s=SensorApp_GetSample();
    if (!s->valid || (have_time && s->sequence==last_sequence)) return;
    uint32_t elapsed=s->timestamp_us-last_sample_us;
    if (have_time && elapsed>=IMU_TIMEOUT_US) {
        MotionEstimator_Invalidate(&estimator);
        have_time=false;
    }
    float dt=have_time ? (float)elapsed*0.000001f : 0;
    last_sample_us=s->timestamp_us;
    last_sequence=s->sequence;
    have_time=true;
    status.last_dt_s=dt;
    status.timestamp_us=s->timestamp_us;
    if (MotionEstimator_Update(&estimator,s->data.accel,s->data.gyro,dt)) ++status.updates;
    else ++status.failures;
}
const MotionState *MotionApp_GetState(void) { return &estimator.state; }
const MotionAppStatus *MotionApp_GetStatus(void) { return &status; }
void MotionApp_GetEuler(float degrees[3]) { MotionEstimator_GetEuler(&estimator,degrees); }
