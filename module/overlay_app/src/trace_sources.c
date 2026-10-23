#include "trace_sources.h"
#include "lqr_app.h"
#include "sensor_app.h"
#include <string.h>

/* 格式变化时同步更新 format_id 和导出工具。 */
_Static_assert(sizeof(LqrTracePayload)==72,"LQR format 3 payload size");
_Static_assert(sizeof(ImuTracePayload)==24,"IMU format 1 payload size");

static bool lqr_snapshot(void *context, TraceBusinessStatus *b, void *payload, uint32_t capacity)
{
    (void)context;
    if (capacity<sizeof(LqrTracePayload)) return false;
    const LqrStatus *s=LqrApp_GetStatus();
    *b=(TraceBusinessStatus){.mode=s->mode==LQR_HOLD ? TRACE_BUSINESS_RUNNING :
        s->mode==LQR_FAULT ? TRACE_BUSINESS_FAULT : TRACE_BUSINESS_STOPPED,
        .has_sample=s->updates>0,.valid=s->data_valid,.faults=s->faults,
        .timestamp_us=s->mode==LQR_HOLD ? s->timestamp_us : s->event_timestamp_us,
        .sequence=s->sequence};
    LqrTracePayload p={.input_timestamp_us=s->input_timestamp_us,.input_sequence=s->state_sequence,
        .dt_us=s->dt_us,.age_us=s->data_age_us,.pitch_rate_scale=s->pitch_rate_scale,.yaw_rate_scale=s->yaw_rate_scale};
    memcpy(p.error,s->error,sizeof p.error); memcpy(p.torque_nm,s->requested_nm,sizeof p.torque_nm);
    memcpy(p.duty,s->duty,sizeof p.duty); memcpy(p.relative_rpm,s->relative_rpm,sizeof p.relative_rpm);
    memcpy(payload,&p,sizeof p);
    return true;
}
static bool imu_snapshot(void *context, TraceBusinessStatus *b, void *payload, uint32_t capacity)
{
    (void)context;
    if (capacity<sizeof(ImuTracePayload)) return false;
    const ImuSample *s=SensorApp_GetSample();
    const SensorStatus *health=SensorApp_GetStatus();
    *b=(TraceBusinessStatus){.mode=health->ready ? TRACE_BUSINESS_RUNNING :
        health->failures ? TRACE_BUSINESS_FAULT : TRACE_BUSINESS_STOPPED,
        .has_sample=health->reads>0,.valid=s->valid,.sequence=s->sequence,
        .timestamp_us=s->timestamp_us,.faults=health->ready ? 0 : health->bus_error};
    /* 采样时间原样保留：它是软件读取开始时间，不是芯片内部采样时刻。 */
    ImuTracePayload p; memcpy(p.accel,s->data.accel,sizeof p.accel);
    memcpy(p.gyro,s->data.gyro,sizeof p.gyro); memcpy(payload,&p,sizeof p);
    return true;
}
bool LqrTrace_Init(LqrTrace *t)
{
    if (!t) return false;
    TraceConfig c={.source_id=TRACE_SOURCE_LQR,.format_id=3,
        .payload_size=sizeof(t->scratch),.read_snapshot=lqr_snapshot,
        .storage=t->frames,.storage_bytes=sizeof(t->frames),
        .frame_size=sizeof(t->frames[0]),.payload_offset=offsetof(LqrTraceFrame,data),
        .capacity=LQR_TRACE_CAPACITY,.scratch=&t->scratch,.scratch_bytes=sizeof(t->scratch)};
    return TraceApp_Init(&t->recorder,&c);
}
bool LqrTrace_Read(const LqrTrace *t, uint32_t index, LqrTraceFrame *out)
{
    return t && TraceApp_Read(&t->recorder,index,out,sizeof(*out));
}
bool ImuTrace_Init(ImuTrace *t)
{
    if (!t) return false;
    TraceConfig c={.source_id=TRACE_SOURCE_IMU,.format_id=1,
        .payload_size=sizeof(t->scratch),.read_snapshot=imu_snapshot,
        .storage=t->frames,.storage_bytes=sizeof(t->frames),
        .frame_size=sizeof(t->frames[0]),.payload_offset=offsetof(ImuTraceFrame,data),
        .capacity=IMU_TRACE_CAPACITY,.scratch=&t->scratch,.scratch_bytes=sizeof(t->scratch)};
    return TraceApp_Init(&t->recorder,&c);
}
bool ImuTrace_Read(const ImuTrace *t, uint32_t index, ImuTraceFrame *out)
{
    return t && TraceApp_Read(&t->recorder,index,out,sizeof(*out));
}
