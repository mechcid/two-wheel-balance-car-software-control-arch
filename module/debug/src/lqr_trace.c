#include "trace_adapters.h"
#include "lqr_app.h"
#include <string.h>

void LqrTrace_Prepare(void *context) {
    LqrTrace *t=context;
    const LqrSample *s=LqrApp_GetSample();
    LqrTracePayload p={.sample_timestamp_us=s->timestamp_us,.sample_sequence=s->sequence,
        .sample_valid=s->valid,.input_timestamp_us=s->state_source.timestamp_us,
        .input_sequence=s->state_source.sequence,.dt_us=s->dt_us,.age_us=s->data_age_us,
        .pitch_rate_scale=s->tuning.pitch_rate_scale,.yaw_rate_scale=s->tuning.yaw_rate_scale};
    memcpy(p.error,s->data.error,sizeof p.error);
    memcpy(p.torque_nm,s->data.requested_nm,sizeof p.torque_nm);
    memcpy(p.duty,s->data.duty,sizeof p.duty);
    memcpy(p.relative_rpm,s->relative_rpm,sizeof p.relative_rpm);
    t->source=p;
}
TraceSourceStatus LqrTrace_Check(void *context) {
    (void)context;
    const LqrStatus *s=LqrApp_GetStatus();
    TraceSourceState state=s->mode==LQR_FAULT ? TRACE_SOURCE_FAULT :
        s->mode!=LQR_RUNNING ? TRACE_SOURCE_STOPPED :
        !s->has_output ? TRACE_SOURCE_WAITING :
        !LqrApp_GetSample()->valid ? TRACE_SOURCE_INVALID : TRACE_SOURCE_NORMAL;
    return (TraceSourceStatus){state,s->faults};
}
