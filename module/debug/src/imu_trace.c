#include "trace_adapters.h"
#include "sensor_app.h"
#include <string.h>

void ImuTrace_Prepare(void *context) {
    ImuTrace *t=context;
    const ImuSample *s=SensorApp_GetSample();
    ImuTracePayload p={.sample_timestamp_us=s->timestamp_us,.sample_sequence=s->sequence,.sample_valid=s->valid};
    memcpy(p.accel,s->data.accel,sizeof p.accel);
    memcpy(p.gyro,s->data.gyro,sizeof p.gyro);
    t->source=p;
}
TraceSourceStatus ImuTrace_Check(void *context) {
    (void)context;
    const SensorStatus *s=SensorApp_GetStatus();
    TraceSourceState state=!s->ready ? (s->fail_read_count ? TRACE_SOURCE_FAULT : TRACE_SOURCE_STOPPED) :
        !s->succ_read_count ? TRACE_SOURCE_WAITING :
        !SensorApp_GetSample()->valid ? TRACE_SOURCE_INVALID : TRACE_SOURCE_NORMAL;
    return (TraceSourceStatus){state,s->ready ? 0 : s->bus_error};
}
