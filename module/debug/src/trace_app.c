#include "trace_app.h"
#include <string.h>

bool TraceApp_Init(Trace *t,const TraceConfig *c) {
    if (!t || t->initialized || !c || !c->source_id || !c->format_id ||
        !c->payload_size || !c->source || !c->check_source || !c->storage ||
        !c->frequency_hz || c->frequency_hz>1000000U ||
        !c->capacity || c->frame_size<sizeof(TraceHeader) ||
        c->payload_offset<sizeof(TraceHeader) || c->payload_offset>c->frame_size ||
        c->payload_size>c->frame_size-c->payload_offset || c->source_bytes<c->payload_size ||
        c->capacity>c->storage_bytes/c->frame_size) return false;
    uintptr_t source=(uintptr_t)c->source, storage=(uintptr_t)c->storage;
    size_t bytes=c->capacity*c->frame_size;
    if (source>UINTPTR_MAX-c->payload_size || storage>UINTPTR_MAX-bytes ||
        (source<storage+bytes && storage<source+c->payload_size)) return false;
    t->config=*c;
    t->period_us=(1000000U+c->frequency_hz-1U)/c->frequency_hz;
    t->status=(TraceStatus){.mode=TRACE_IDLE,.source_id=c->source_id,
        .format_id=c->format_id,.payload_size=c->payload_size};
    t->initialized=true;
    return true;
}
bool TraceApp_Arm(Trace *t) {
    if (!t || !t->initialized || t->status.mode==TRACE_RECORDING || t->status.mode==TRACE_ARMED) return false;
    uint32_t session=t->status.session+1;
    t->status=(TraceStatus){.mode=TRACE_ARMED,.source_id=t->config.source_id,
        .format_id=t->config.format_id,.payload_size=t->config.payload_size,.session=session};
    t->last_record_us=0;
    return true;
}
static void append(Trace *t,TraceSourceStatus source,TraceEvent event,uint32_t now) {
    TraceHeader h={.timestamp_us=now,.sequence=t->status.total_written+1,
        .business_mode=source.state,.faults=source.faults,.event=event,.valid=event==TRACE_SAMPLE};
    unsigned char *slot=(unsigned char *)t->config.storage+t->status.write_index*t->config.frame_size;
    memset(slot,0,t->config.frame_size);
    memcpy(slot,&h,sizeof h);
    // 包括终止事件：源数据是否有效由业务载荷自身的标志表达。
    memcpy(slot+t->config.payload_offset,t->config.source,t->config.payload_size);
    t->status.write_index=(t->status.write_index+1)%t->config.capacity;
    if (t->status.count<t->config.capacity) ++t->status.count;
    ++t->status.total_written;
    t->last_record_us=now;
}
static bool active(const Trace *t) {
    return t && t->initialized && (t->status.mode==TRACE_ARMED || t->status.mode==TRACE_RECORDING);
}
void TraceApp_Poll(Trace *t,uint32_t now) {
    if (!active(t)) return;
    TraceSourceStatus source=t->config.check_source(t->config.context);
    TraceEvent event;
    TraceFreezeReason reason;
    if (source.faults || source.state==TRACE_SOURCE_FAULT) {
        event=TRACE_FAULT_EVENT; reason=TRACE_FREEZE_FAULT;
    } else if (source.state==TRACE_SOURCE_STOPPED) {
        if (t->status.mode==TRACE_ARMED && !t->status.business_started) return;
        event=TRACE_STOP_EVENT; reason=TRACE_FREEZE_STOP;
    } else if (source.state==TRACE_SOURCE_WAITING && t->status.mode==TRACE_ARMED) {
        t->status.business_started=true;
        return;
    } else if (source.state==TRACE_SOURCE_NORMAL) {
        if (t->status.mode==TRACE_RECORDING && (uint32_t)(now-t->last_record_us)<t->period_us) return;
        t->status.business_started=true;
        t->status.mode=TRACE_RECORDING;
        append(t,source,TRACE_SAMPLE,now);
        return;
    } else {
        event=TRACE_INVALID_EVENT; reason=TRACE_FREEZE_INVALID;
    }
    append(t,source,event,now);
    t->status.freeze_reason=reason;
    t->status.mode=TRACE_FROZEN;
}
void TraceApp_Freeze(Trace *t,uint32_t now) {
    if (!active(t)) return;
    append(t,t->config.check_source(t->config.context),TRACE_MANUAL_EVENT,now);
    t->status.freeze_reason=TRACE_FREEZE_MANUAL;
    t->status.mode=TRACE_FROZEN;
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
