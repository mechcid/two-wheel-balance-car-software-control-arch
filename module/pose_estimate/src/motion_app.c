#include "motion_app.h"
#include "sensor_app.h"

static MotionEstimator estimator; /* 四元数和滤波状态仅在模块内使用。 */
static MotionSample sample;
static MotionAppStatus status;
static uint32_t last_sample_us, last_sequence;
static uint32_t last_update_us;
static bool have_update_time;
static bool started, have_time, have_sequence;
static MotionClockFn motion_clock_now_us;

bool MotionApp_Init(const MotionConfig *config, MotionClockFn clock_now_us)
{
    if (started || !clock_now_us || !MotionEstimator_Init(&estimator,config)) return false;
    motion_clock_now_us=clock_now_us;
    status.state=MOTION_WAITING_INPUT;
    started=true;
    return true;
}

static void invalidate_input(MotionRunState reason)
{
    if (have_time) MotionEstimator_Invalidate(&estimator);
    have_time=false;
    sample.valid=false;
    sample.discontinuity=estimator.state.discontinuity;
    status.accel_trusted=false;
    status.state=reason;
}

/* 一份输入只尝试一次；成功后统一发布角度及其元信息。 */
static bool MotionApp_Update(const ImuSample *input)
{
    uint32_t elapsed=input->timestamp_us-last_sample_us;
    if (have_time && elapsed>=IMU_TIMEOUT_US) {
        MotionEstimator_Invalidate(&estimator);
        have_time=false;
    }
    float dt=have_time ? (float)elapsed*0.000001f : 0;
    last_sample_us=input->timestamp_us;
    last_sequence=input->sequence;
    have_sequence=true;
    have_time=true;
    status.last_dt_s=dt;
    bool ok=MotionEstimator_Update(&estimator,input->data.accel,input->data.gyro,dt);
    status.accel_trusted=estimator.state.accel_trusted;
    if (!ok) {
        sample.valid=false;
        sample.discontinuity=estimator.state.discontinuity;
        ++status.fail_update_count;
        status.state=MOTION_ESTIMATION_FAILED;
        return false;
    }
    MotionSample next={
        .timestamp_us=input->timestamp_us,
        .sequence=sample.sequence+1U,
        .valid=true,
        .discontinuity=estimator.state.discontinuity};
    MotionEstimator_GetEuler(&estimator,&next.data);
    sample=next;
    ++status.succ_update_count;
    status.state=MOTION_RUNNING;
    return true;
}

void MotionApp_Poll(void)
{
    if (!started) return;
    uint32_t now=motion_clock_now_us();
    const ImuSample *input=SensorApp_GetSample();
    /* 即使输入持续更新，处理受限时旧输出也不能无限保持有效。 */
    if (sample.valid && (uint32_t)(now-sample.timestamp_us)>=IMU_TIMEOUT_US)
        invalidate_input(MOTION_INPUT_TIMEOUT);
    if (!SensorApp_GetStatus()->ready) {
        invalidate_input(MOTION_INPUT_UNAVAILABLE);
        return;
    }
    if (!input->valid && !have_sequence && input->sequence==0) {
        status.state=MOTION_WAITING_INPUT;
        return;
    }
    if ((uint32_t)(now-input->timestamp_us)>=IMU_TIMEOUT_US) {
        invalidate_input(MOTION_INPUT_TIMEOUT);
        return;
    }
    if (!input->valid) {
        invalidate_input(MOTION_INPUT_UNAVAILABLE);
        return;
    }
    if (have_sequence && input->sequence==last_sequence) return;
    if (have_update_time &&
        (uint32_t)(now-last_update_us)<MOTION_MIN_UPDATE_INTERVAL_US) return;
    last_update_us=now;
    have_update_time=true;
    (void)MotionApp_Update(input);
}

const MotionSample *MotionApp_GetSample(void) { return &sample; }
const MotionAppStatus *MotionApp_GetStatus(void) { return &status; }
