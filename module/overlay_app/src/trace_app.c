#include "trace_app.h"
#include <string.h>

bool TraceApp_Init(Trace *t, const TraceConfig *c){
    if (!t || t->initialized || !c || !c->source_id || !c->format_id ||
        !c->payload_size || !c->read_snapshot || !c->storage || !c->scratch ||
        !c->capacity || c->frame_size<sizeof(TraceHeader) ||
        c->payload_offset<sizeof(TraceHeader) || c->payload_offset>c->frame_size ||
        c->payload_size>c->frame_size-c->payload_offset ||
        c->scratch_bytes<c->payload_size ||
        c->capacity>c->storage_bytes/c->frame_size) return false;
    t->config=*c;
    t->status=(TraceStatus){.mode=TRACE_IDLE,.source_id=c->source_id,
        .format_id=c->format_id,.payload_size=c->payload_size};
    t->initialized=true; return true;
}
bool TraceApp_Arm(Trace *t)
{
    if (!t || !t->initialized || t->status.mode==TRACE_RECORDING || t->status.mode==TRACE_ARMED) return false;
    t->last_business_mode=TRACE_BUSINESS_STOPPED;
    uint32_t session=t->status.session+1U;
    t->status=(TraceStatus){.mode=TRACE_ARMED,.source_id=t->config.source_id,
        .format_id=t->config.format_id,.payload_size=t->config.payload_size,.session=session};
    return true;
}
static void append(Trace *t, const TraceBusinessStatus *b, TraceEvent event, const void *payload){
    TraceHeader f={
        .timestamp_us=b->timestamp_us,
        .sequence=b->sequence,
        .business_mode=b->mode,
        .faults=b->faults,
        .event=event,
        .valid=event==TRACE_SAMPLE};
    unsigned char *slot=(unsigned char *)t->config.storage+
        t->status.write_index*t->config.frame_size;
    memset(slot,0,t->config.frame_size);
    memcpy(slot,&f,sizeof f);
    if (f.valid) memcpy(slot+t->config.payload_offset,payload,t->config.payload_size);
    t->status.write_index=(t->status.write_index+1U)%t->config.capacity;
    if (t->status.count<t->config.capacity) ++t->status.count;
    ++t->status.total_written;
}

static void observe(Trace *t, const TraceBusinessStatus *b, const void *payload, uint32_t now){
    t->last_business_mode=b->mode;
    if (t->status.mode==TRACE_ARMED && !b->faults && b->mode!=TRACE_BUSINESS_FAULT) {
        if (b->mode==TRACE_BUSINESS_STOPPED && !t->status.business_started) return;
        if (b->mode==TRACE_BUSINESS_RUNNING) {
            t->status.business_started=true;
            if (!b->has_sample) return;
        }
    }
    TraceEvent event;
    TraceFreezeReason reason;
    if (b->mode==TRACE_BUSINESS_FAULT || b->faults) {
        event=TRACE_FAULT_EVENT; reason=TRACE_FREEZE_FAULT;
    } else if (b->mode==TRACE_BUSINESS_STOPPED) {
        event=TRACE_STOP_EVENT; reason=TRACE_FREEZE_STOP;
    } else if (b->mode!=TRACE_BUSINESS_RUNNING || !b->has_sample || !b->valid || !payload) {
        event=TRACE_INVALID_EVENT; reason=TRACE_FREEZE_INVALID;
    } else {
        if (t->status.have_sequence && b->sequence==t->status.last_sequence) {
            ++t->status.duplicate_frames; return;
        }
        t->status.mode=TRACE_RECORDING;
        append(t,b,TRACE_SAMPLE,payload);
        t->status.last_sequence=b->sequence; t->status.have_sequence=true;
        return;
    }
    TraceBusinessStatus event_status=*b;
    event_status.timestamp_us=now; /* 轮询发现状态转变的时间，不冒充采样时间。 */
    append(t,&event_status,event,NULL);
    t->status.freeze_reason=reason;
    t->status.mode=TRACE_FROZEN;
}
void TraceApp_Poll(Trace *t, uint32_t now)
{
    if (!t || !t->initialized || (t->status.mode!=TRACE_ARMED && t->status.mode!=TRACE_RECORDING)) return;
    TraceBusinessStatus b={.mode=TRACE_BUSINESS_RUNNING,.timestamp_us=now,.has_sample=true};
    memset(t->config.scratch,0,t->config.payload_size);
    if (!t->config.read_snapshot(t->config.context,&b,t->config.scratch,t->config.payload_size)) {
        b=(TraceBusinessStatus){.mode=t->last_business_mode,.timestamp_us=now,
            .sequence=t->status.last_sequence,.has_sample=true};
        /* 读取失败与业务停止不同，强制按无效事件处理。 */
        append(t,&b,TRACE_INVALID_EVENT,NULL);
        t->status.freeze_reason=TRACE_FREEZE_INVALID; t->status.mode=TRACE_FROZEN;
        return;
    }
    observe(t,&b,t->config.scratch,now);
}
void TraceApp_Freeze(Trace *t, uint32_t now)
{
    if (!t || !t->initialized || (t->status.mode!=TRACE_RECORDING && t->status.mode!=TRACE_ARMED)) return;
    TraceBusinessStatus b={.mode=t->last_business_mode,.timestamp_us=now,
        .sequence=t->status.last_sequence};
    append(t,&b,TRACE_MANUAL_EVENT,NULL);
    t->status.freeze_reason=TRACE_FREEZE_MANUAL; t->status.mode=TRACE_FROZEN;
}
const volatile TraceStatus *TraceApp_GetStatus(const Trace *t) { return t ? &t->status : NULL; }
bool TraceApp_Read(const Trace *t, uint32_t index, void *out, size_t out_size)
{
    if (!t || !t->initialized || !out || out_size<t->config.frame_size ||
        t->status.mode!=TRACE_FROZEN || index>=t->status.count) return false;
    uint32_t start=t->status.count<t->config.capacity ? 0 : t->status.write_index;
    const unsigned char *slot=(const unsigned char *)t->config.storage+
        ((start+index)%t->config.capacity)*t->config.frame_size;
    memcpy(out,slot,t->config.frame_size);
    return true;
}
