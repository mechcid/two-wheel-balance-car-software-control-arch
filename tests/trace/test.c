#include "trace_app.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
typedef struct { TraceHeader header; uint32_t data; } Frame;
static TraceSourceStatus business;
static unsigned calls;
static TraceSourceStatus check(void *context) { assert(context==&business); ++calls; return business; }
int main(void) {
    Trace t={0}; Frame frames[4], out; uint32_t source=42;
    TraceConfig c={.source_id=1,.format_id=9,.payload_size=sizeof source,
        .source=&source,.source_bytes=sizeof source,.frequency_hz=200,.check_source=check,.context=&business,
        .storage=frames,.storage_bytes=sizeof frames,.frame_size=sizeof(Frame),
        .payload_offset=offsetof(Frame,data),.capacity=4};
    assert(!TraceApp_Arm(&t)); TraceApp_Poll(&t,0); assert(!calls);
    TraceConfig bad=c; bad.frequency_hz=0; assert(!TraceApp_Init(&t,&bad));
    bad=c; bad.frequency_hz=1000001; assert(!TraceApp_Init(&t,&bad));
    bad=c; bad.source_bytes=0; assert(!TraceApp_Init(&t,&bad));
    bad=c; bad.capacity=5; assert(!TraceApp_Init(&t,&bad));
    bad=c; bad.source=&frames[0].data; assert(!TraceApp_Init(&t,&bad));
    bad=c; bad.payload_offset=0; assert(!TraceApp_Init(&t,&bad));
    bad=c; bad.check_source=NULL; assert(!TraceApp_Init(&t,&bad));
    assert(TraceApp_Init(&t,&c)); assert(!TraceApp_Init(&t,&c));
    assert(TraceApp_Arm(&t)); assert(!TraceApp_Arm(&t));
    business.state=TRACE_SOURCE_STOPPED; TraceApp_Poll(&t,1); assert(t.status.count==0);
    business.state=TRACE_SOURCE_WAITING; TraceApp_Poll(&t,2); assert(t.status.count==0);
    business.state=TRACE_SOURCE_NORMAL;
    uint32_t now=UINT32_MAX-2000;
    TraceApp_Poll(&t,now); assert(t.status.count==1);
    assert(!TraceApp_Read(&t,0,&out,sizeof out));
    TraceApp_Poll(&t,now+4999); assert(t.status.count==1);
    // 相同源数据仍定频记录；时间回绕不影响周期。
    TraceApp_Poll(&t,now+5000); assert(t.status.count==2);
    // 主循环延迟不补写重复历史。
    now+=50000; source=77; TraceApp_Poll(&t,now); assert(t.status.total_written==3);
    unsigned checked=calls;
    business=(TraceSourceStatus){TRACE_SOURCE_FAULT,5}; source=99;
    TraceApp_Poll(&t,now+1); // 未到周期也立即保存现场并冻结。
    assert(calls==checked+1 && t.status.count==4 && t.status.freeze_reason==TRACE_FREEZE_FAULT);
    assert(TraceApp_Read(&t,0,&out,sizeof out) && out.data==42 && out.header.sequence==1);
    assert(out.header.timestamp_us==UINT32_MAX-2000);
    assert(TraceApp_Read(&t,3,&out,sizeof out) && out.data==99 && out.header.faults==5);
    assert(out.header.event==TRACE_FAULT_EVENT && !out.header.valid && out.header.timestamp_us==now+1);
    checked=calls; TraceApp_Poll(&t,now+5000); assert(calls==checked);
    assert(!TraceApp_Read(&t,4,&out,sizeof out)); assert(!TraceApp_Read(&t,0,&out,sizeof out-1));
    // 独立实例与不同频率，覆盖后仍按时间顺序读取。
    Trace second={0}; Frame other[3]; uint32_t other_source=100;
    TraceConfig c2=c; c2.storage=other; c2.storage_bytes=sizeof other; c2.capacity=3;
    c2.source=&other_source; c2.frequency_hz=1000;
    assert(TraceApp_Init(&second,&c2) && TraceApp_Arm(&second));
    business=(TraceSourceStatus){TRACE_SOURCE_NORMAL,0};
    for (unsigned i=0;i<6;++i) { other_source=100+i; TraceApp_Poll(&second,i*1000); }
    assert(second.status.count==3 && second.status.total_written==6);
    other_source=106; TraceApp_Freeze(&second,5001);
    assert(TraceApp_Read(&second,0,&out,sizeof out) && out.data==104);
    assert(TraceApp_Read(&second,2,&out,sizeof out) && out.data==106 && out.header.event==TRACE_MANUAL_EVENT);
    assert(t.status.freeze_reason==TRACE_FREEZE_FAULT && t.status.total_written==4);
    assert(TraceApp_Arm(&t) && t.status.session==2 && !t.status.count);
    business.state=TRACE_SOURCE_WAITING; TraceApp_Poll(&t,0);
    business.state=TRACE_SOURCE_STOPPED; TraceApp_Poll(&t,1);
    assert(t.status.freeze_reason==TRACE_FREEZE_STOP && t.status.count==1);
    assert(TraceApp_Arm(&t)); business.state=TRACE_SOURCE_INVALID; source=123;
    TraceApp_Poll(&t,2); assert(t.status.freeze_reason==TRACE_FREEZE_INVALID);
    assert(TraceApp_Read(&t,0,&out,sizeof out) && out.data==123);
    puts("PASS: fixed-rate copies, clock wrap, skipped periods, terminal snapshots, init validation, independent rings and freeze/rearm");
}
