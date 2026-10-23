#include "trace_app.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#define TEST_CAPACITY 8U
typedef struct { TraceHeader header; uint32_t data; } TestFrame;
static Trace recorder;
static TestFrame frames[TEST_CAPACITY];
static uint32_t scratch;
static TraceBusinessStatus business;
static uint32_t data, calls;
static bool readable=true;
static bool snapshot(void *context, TraceBusinessStatus *b, void *payload, uint32_t cap)
{
    assert(context==&data && cap>=sizeof(data)); ++calls;
    if (!readable) return false;
    *b=business; memcpy(payload,&data,sizeof(data)); return true;
}
int main(void)
{
    TraceConfig config={.source_id=1,.format_id=7,.payload_size=sizeof(data),
        .read_snapshot=snapshot,.context=&data,.storage=frames,.storage_bytes=sizeof frames,
        .frame_size=sizeof(TestFrame),.payload_offset=offsetof(TestFrame,data),
        .capacity=TEST_CAPACITY,.scratch=&scratch,.scratch_bytes=sizeof scratch};
    const volatile TraceStatus *s=TraceApp_GetStatus(&recorder); TestFrame out;
    assert(!TraceApp_Arm(&recorder)); TraceApp_Poll(&recorder,0); assert(!calls);
    assert(!TraceApp_Init(&recorder,NULL)); TraceConfig bad=config; bad.read_snapshot=NULL;
    assert(!TraceApp_Init(&recorder,&bad)); bad=config; bad.payload_size=sizeof(TestFrame)+1;
    assert(!TraceApp_Init(&recorder,&bad)); assert(TraceApp_Init(&recorder,&config)); assert(!TraceApp_Init(&recorder,&config));
    assert(s->mode==TRACE_IDLE); TraceApp_Poll(&recorder,0); assert(!calls);
    assert(TraceApp_Arm(&recorder) && s->mode==TRACE_ARMED); assert(!TraceApp_Arm(&recorder));
    TraceApp_Poll(&recorder,1); assert(!s->count && s->mode==TRACE_ARMED); // stopped before start
    business.mode=TRACE_BUSINESS_RUNNING; TraceApp_Poll(&recorder,2); assert(s->mode==TRACE_ARMED); // no first sample
    business.has_sample=business.valid=true; business.sequence=UINT32_MAX;
    business.timestamp_us=UINT32_MAX-10; data=42; TraceApp_Poll(&recorder,123);
    assert(s->count==1 && s->mode==TRACE_RECORDING && !TraceApp_Read(&recorder,0,&out,sizeof out));
    TraceApp_Poll(&recorder,124); assert(s->count==1 && s->duplicate_frames==1);
    business.sequence=0;business.timestamp_us=5;TraceApp_Poll(&recorder,125);assert(s->count==2);
    for(unsigned i=1;i<=TEST_CAPACITY+10;++i) {
        business.sequence=i; business.timestamp_us=i*5000; data=i; TraceApp_Poll(&recorder,i*5000+200);
    }
    business.mode=TRACE_BUSINESS_STOPPED;TraceApp_Poll(&recorder,999999);
    assert(s->mode==TRACE_FROZEN && s->freeze_reason==TRACE_FREEZE_STOP && s->count==TEST_CAPACITY);
    for(unsigned i=0;i<s->count-1;++i) {
        assert(TraceApp_Read(&recorder,i,&out,sizeof out));uint32_t v=out.data;
        assert(out.header.valid && out.header.event==TRACE_SAMPLE && v==out.header.sequence);
        assert(out.header.timestamp_us==v*5000 && v==i+12); // source timestamp, not poll time
    }
    assert(TraceApp_Read(&recorder,s->count-1,&out,sizeof out) && !out.header.valid && out.header.timestamp_us==999999);
    assert(out.data==0);
    unsigned before=calls; business.mode=TRACE_BUSINESS_RUNNING;TraceApp_Poll(&recorder,0);
    assert(calls==before && !TraceApp_Read(&recorder,s->count,&out,sizeof out));
    for(unsigned kind=0;kind<3;++kind) {
        assert(TraceApp_Arm(&recorder));business.mode=TRACE_BUSINESS_RUNNING;business.faults=0;
        business.has_sample=true;business.valid=true;readable=true;
        if(kind==0) business.valid=false;
        if(kind==1) { business.mode=TRACE_BUSINESS_FAULT;business.faults=4; }
        if(kind==2) readable=false;
        TraceApp_Poll(&recorder,55);
        assert(s->mode==TRACE_FROZEN && s->count==1);
        assert(s->freeze_reason==(kind==1 ? TRACE_FREEZE_FAULT:TRACE_FREEZE_INVALID));
    }
    readable=true;assert(TraceApp_Arm(&recorder));TraceApp_Freeze(&recorder,101);
    assert(s->mode==TRACE_FROZEN && s->freeze_reason==TRACE_FREEZE_MANUAL);
    assert(TraceApp_Read(&recorder,0,&out,sizeof out) && out.header.timestamp_us==101 && !out.header.valid);
    TraceApp_Freeze(&recorder,102);assert(s->count==1);
    assert(TraceApp_Arm(&recorder)); business=(TraceBusinessStatus){.mode=TRACE_BUSINESS_RUNNING};
    TraceApp_Poll(&recorder,200);assert(s->mode==TRACE_ARMED);
    business.mode=TRACE_BUSINESS_STOPPED;TraceApp_Poll(&recorder,201);
    assert(s->mode==TRACE_FROZEN && s->freeze_reason==TRACE_FREEZE_STOP);
    // A second typed instance has independent capacity, storage, sequence and freeze state.
    Trace second={0}; TestFrame second_frames[3]; uint32_t second_scratch;
    TraceConfig second_config=config;
    second_config.storage=second_frames; second_config.storage_bytes=sizeof second_frames;
    second_config.capacity=3; second_config.scratch=&second_scratch;
    assert(TraceApp_Init(&second,&second_config));
    TestFrame saved; assert(TraceApp_Read(&recorder,0,&saved,sizeof saved));
    uint32_t old_session=s->session, old_count=s->count;
    assert(TraceApp_Arm(&second));
    business=(TraceBusinessStatus){.mode=TRACE_BUSINESS_RUNNING,.has_sample=true,.valid=true};
    for (unsigned i=0;i<7;++i) {
        business.sequence=i; data=100+i; TraceApp_Poll(&second,i);
    }
    assert(second.status.count==3 && second.status.total_written==7);
    assert(s->mode==TRACE_FROZEN && s->session==old_session && s->count==old_count);
    assert(TraceApp_Read(&recorder,0,&out,sizeof out) && memcmp(&out,&saved,sizeof out)==0);
    assert(TraceApp_Arm(&recorder)); // Rearming first does not reset second.
    assert(second.status.count==3 && second.status.last_sequence==6);
    TraceApp_Poll(&recorder,10); assert(s->count==1); // Same sequence is new to this instance.
    TraceApp_Freeze(&second,11);
    assert(TraceApp_Read(&second,0,&out,sizeof out) && out.data==105);
    assert(!TraceApp_Read(&second,0,&out,sizeof out-1));
    business.sequence=7; TraceApp_Poll(&recorder,12);
    assert(s->count==2 && s->mode==TRACE_RECORDING && second.status.mode==TRACE_FROZEN);
    puts("PASS: multiple instances have independent storage, wrap, freeze, rearm and sequences");
    puts("PASS: init/arm/wait, polling snapshots, sample timestamps/sequence wrap, overwrite, duplicates, invalid/fault/freeze and no automatic resume");
}
