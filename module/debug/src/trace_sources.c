#include "trace_adapters.h"

_Static_assert(sizeof(LqrTracePayload)==84,"LQR format 4 payload size");
_Static_assert(sizeof(ImuTracePayload)==36,"IMU format 2 payload size");
typedef struct {
    uint32_t source_id, format_id, frequency_hz;
    TraceCheckSource check;
    void (*prepare)(void *context);
} TraceRegistration;
static const TraceRegistration registrations[]={
    {TRACE_SOURCE_LQR,4,200,LqrTrace_Check,LqrTrace_Prepare},
    {TRACE_SOURCE_IMU,2,200,ImuTrace_Check,ImuTrace_Prepare}
};
static const TraceRegistration *find(uint32_t id) {
    for (size_t i=0;i<sizeof registrations/sizeof registrations[0];++i)
        if (registrations[i].source_id==id) return &registrations[i];
    return NULL;
}
static bool init(Trace *trace,void *context,uint32_t id,void *source,size_t size,
                 void *storage,size_t bytes,size_t frame_size,size_t offset,uint32_t capacity) {
    const TraceRegistration *r=find(id);
    TraceConfig c={.source_id=id,.format_id=r->format_id,.payload_size=size,
        .source=source,.source_bytes=size,.frequency_hz=r->frequency_hz,.check_source=r->check,
        .context=context,.storage=storage,.storage_bytes=bytes,.frame_size=frame_size,
        .payload_offset=offset,.capacity=capacity};
    return TraceApp_Init(trace,&c);
}
bool LqrTrace_Init(LqrTrace *t) {
    return t && init(&t->recorder,t,TRACE_SOURCE_LQR,&t->source,sizeof t->source,
        t->frames,sizeof t->frames,sizeof t->frames[0],offsetof(LqrTraceFrame,data),LQR_TRACE_CAPACITY);
}
bool ImuTrace_Init(ImuTrace *t) {
    return t && init(&t->recorder,t,TRACE_SOURCE_IMU,&t->source,sizeof t->source,
        t->frames,sizeof t->frames,sizeof t->frames[0],offsetof(ImuTraceFrame,data),IMU_TRACE_CAPACITY);
}
static bool prepare(Trace *t) {
    if (!t || !t->initialized || (t->status.mode!=TRACE_ARMED && t->status.mode!=TRACE_RECORDING)) return false;
    const TraceRegistration *r=find(t->config.source_id);
    if (!r) return false;
    r->prepare(t->config.context);
    return true;
}
void TraceSources_Poll(Trace *t,uint32_t now) { if (prepare(t)) TraceApp_Poll(t,now); }
void TraceSources_Freeze(Trace *t,uint32_t now) { if (prepare(t)) TraceApp_Freeze(t,now); }
bool LqrTrace_Read(const LqrTrace *t,uint32_t i,LqrTraceFrame *out) {
    return t && TraceApp_Read(&t->recorder,i,out,sizeof *out);
}
bool ImuTrace_Read(const ImuTrace *t,uint32_t i,ImuTraceFrame *out) {
    return t && TraceApp_Read(&t->recorder,i,out,sizeof *out);
}
