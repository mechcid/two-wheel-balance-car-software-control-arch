#include "lqr_app.h"
#include "trace_app.h"
#include "debug_app.h"
#include "sensor_app.h"
#include "motor_app.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static LqrTrace lqr_trace;
static ImuTrace imu_trace;
static Trace *trace;
static ImuSample imu;
static SensorStatus imu_health;
const ImuSample *SensorApp_GetSample(void) { return &imu; }
const SensorStatus *SensorApp_GetStatus(void) { return &imu_health; }
static void update(uint32_t now) { LqrApp_Update(now); DebugApp_Poll(now); }
static void poll(uint32_t now) { LqrApp_Poll(now); DebugApp_Poll(now); }
static StatusState state;
static MotorAppStatus motor;
static bool reject_output;
static unsigned writes, stops;
const StatusState *StatusStateApp_GetState(void) { return &state; }
const MotorAppStatus *MotorApp_GetStatus(void) { return &motor; }
void MotorApp_Stop(uint32_t now) { (void)now; ++stops; motor.mode=MOTOR_STOPPED; }
bool MotorApp_ApplyDuty(float l, float r, uint32_t now) {
    (void)now; assert(isfinite(l)&&isfinite(r));
    if (reject_output) return false;
    ++writes; motor.mode=MOTOR_DUTY; return true;
}
static void frame(uint32_t now) {
    state.timestamp_us=state.attitude_timestamp_us=now; ++state.sequence;
    state.motor_sequence=++motor.sequence; state.valid=true;
    motor.valid=motor.left.feedback_valid=motor.right.feedback_valid=true;
}
static void near(float a,float b) { if (fabsf(a-b)>=1e-5f) fprintf(stderr,"near: actual=%g expected=%g\n",a,b); assert(fabsf(a-b)<1e-5f); }
int main(int argc, char **argv) {
    (void)argv;
    assert(LqrTrace_Init(&lqr_trace)); assert(ImuTrace_Init(&imu_trace));
    assert(DebugApp_Init(&lqr_trace,&imu_trace));
    trace=&lqr_trace.recorder;
    if (argc>1) {
        trace=&imu_trace.recorder; assert(TraceApp_Arm(trace));
        TraceApp_Poll(trace,1); assert(TraceApp_GetStatus(trace)->mode==TRACE_ARMED);
        imu_health.ready=true; imu_health.reads=1;
        imu=(ImuSample){.valid=true,.sequence=9,.timestamp_us=100,.data={{1,2,3},{4,5,6}}};
        TraceApp_Poll(trace,500);TraceApp_Poll(trace,600);
        assert(TraceApp_GetStatus(trace)->count==1);
        imu.valid=false;TraceApp_Poll(trace,700);
        ImuTraceFrame f;assert(TraceApp_Read(trace,0,&f,sizeof f) && f.header.timestamp_us==100 && f.header.sequence==9);
        assert(f.data.accel[0]==1 && f.data.gyro[2]==6);
        assert(TraceApp_GetStatus(trace)->freeze_reason==TRACE_FREEZE_INVALID);
        puts("PASS: IMU polling adapter retains sample identity and freezes on invalid data");return 0;
    }

    LqrConfig c=LqrApp_DefaultConfig(); LqrTuning tuning={.pitch_rate_scale=1,.yaw_rate_scale=1}; LqrStatus out={0};
    float hold[3]={0}, rpm[2]={0}; state.valid=true;
    assert(LqrApp_Calculate(&state,hold,rpm,&c,&tuning,&out));
    near(out.requested_nm[0],0); near(out.duty[1],0);
    state.theta_dot_radps=1;
    assert(LqrApp_Calculate(&state,hold,rpm,&c,&tuning,&out)); float baseline=out.requested_nm[0];
    tuning.pitch_rate_scale=.7f; assert(LqrApp_Calculate(&state,hold,rpm,&c,&tuning,&out));
    near(out.requested_nm[0],baseline*.7f); tuning.pitch_rate_scale=1; state.theta_dot_radps=0;
    LqrTuning defaults=LqrApp_DefaultTuning();
    near(defaults.pitch_rate_scale,.5f); near(defaults.yaw_rate_scale,.5f);
    // Only the yaw-rate contribution changes, even with other state terms present.
    state.psi_rad=.02f;state.theta_rad=.01f;state.theta_dot_radps=.2f;state.psi_dot_radps=.8f;
    LqrStatus full, half;
    assert(LqrApp_Calculate(&state,hold,rpm,&c,&tuning,&full));
    tuning.yaw_rate_scale=.5f;
    assert(LqrApp_Calculate(&state,hold,rpm,&c,&tuning,&half));
    for (unsigned wheel=0;wheel<2;++wheel)
        near(half.requested_nm[wheel]-full.requested_nm[wheel],lqr_k[wheel][4]*.8f*.5f);
    assert(memcmp(full.error,half.error,sizeof full.error)==0);
    tuning.yaw_rate_scale=NAN;assert(!LqrApp_Calculate(&state,hold,rpm,&c,&tuning,&out));
    tuning.yaw_rate_scale=.49f;assert(!LqrApp_Calculate(&state,hold,rpm,&c,&tuning,&out));
    tuning.yaw_rate_scale=1.01f;assert(!LqrApp_Calculate(&state,hold,rpm,&c,&tuning,&out));
    tuning.yaw_rate_scale=1;state=(StatusState){.valid=true};
    state.theta_rad=.03f;
    assert(LqrApp_Calculate(&state,hold,rpm,&c,&tuning,&out));
    assert(out.requested_nm[0]>0 && out.requested_nm[1]>0);
    near(out.requested_nm[0],-lqr_k[0][2]*.03f); /* 当前用户选择的 K，不能锁死历史增益。 */
    // Rotation by pi/2: world Y displacement is forward, X is lateral.
    state=(StatusState){.valid=true,.x_m=8,.y_m=3,.psi_rad=1.5707963268f,.v_mps=.2f};
    hold[0]=7; hold[1]=2; hold[2]=1.5707963268f;
    assert(LqrApp_Calculate(&state,hold,rpm,&c,&tuning,&out));
    near(out.error[0],1); near(out.error[3],.2f);
    state.psi_rad=3.13159265f; hold[2]=-3.13159265f;
    assert(LqrApp_Calculate(&state,hold,rpm,&c,&tuning,&out)); near(out.error[1],-.02f);
    state=(StatusState){.valid=true}; hold[0]=hold[1]=hold[2]=0;
    rpm[0]=60; rpm[1]=-60;
    assert(LqrApp_Calculate(&state,hold,rpm,&c,&tuning,&out));
    near(out.duty[0],.01139f*6.283185307f/(.03677f*12));
    near(out.estimated_nm[0],0); near(out.duty[1],-out.duty[0]);
    state.theta_rad=10;
    assert(LqrApp_Calculate(&state,hold,rpm,&c,&tuning,&out));
    assert(out.saturated); near(out.duty[0],.4f);
    state.theta_rad=NAN;
    assert(!LqrApp_Calculate(&state,hold,rpm,&c,&tuning,&out));
    state.theta_rad=0; rpm[0]=INFINITY;
    assert(!LqrApp_Calculate(&state,hold,rpm,&c,&tuning,&out));
    LqrConfig bad=c; bad.actuator.battery_v=0; assert(!LqrApp_Init(&bad,&tuning));
    assert(LqrApp_Init(&c,&tuning)); assert(!LqrApp_Init(&c,&tuning));
    assert(!LqrApp_IsActive() && writes==0);
    uint32_t now=UINT32_MAX-6000U;
    state=(StatusState){0}; frame(now);
    assert(LqrApp_Start(now)); assert(writes==0);
    assert(!LqrApp_Start(now));
    now+=5000; frame(now); state.theta_rad=.01f; update(now);
    assert(writes==1 && LqrApp_IsActive());
    update(now); assert(writes==1); // Never process a duplicate twice.
    now+=5000; frame(now); update(now); assert(writes==2); // Clock wrap.
    state.valid=false; poll(now);
    assert(stops==1 && LqrApp_GetStatus()->mode==LQR_FAULT);
    frame(now); assert(!LqrApp_Start(now)); // Explicit stop required to clear latch.
    DebugApp_Stop(now); frame(now); assert(LqrApp_Start(now));
    now+=8000; frame(now); update(now);
    assert(LqrApp_GetStatus()->faults & LQR_TIMING);
    DebugApp_Stop(now); frame(now); assert(LqrApp_Start(now));
    now+=5000; frame(now); state.theta_rad=.4f; update(now);
    assert(LqrApp_GetStatus()->faults & LQR_RANGE);
    DebugApp_Stop(now); state.theta_rad=0; frame(now); assert(LqrApp_Start(now));
    now+=5000; frame(now); reject_output=true; update(now);
    assert(LqrApp_GetStatus()->faults & LQR_OUTPUT);
    DebugApp_Stop(now); reject_output=false; frame(now); assert(LqrApp_Start(now));
    now+=5000; frame(now); state.attitude_timestamp_us=now-7501; update(now);
    assert(LqrApp_GetStatus()->faults & LQR_BAD_STATE);
    DebugApp_Stop(now); frame(now); state.theta_rad=.11f; assert(!LqrApp_Start(now));
    state.theta_rad=0; frame(now); assert(DebugApp_StartTrial(now,false));
    for (unsigned i=0;i<1399;++i) {
        now+=5000; frame(now); update(now);
        assert(LqrApp_IsActive()); // Includes the former 2-second cutoff.
    }
    poll(now+4999); assert(LqrApp_IsActive()); // 6.999999 seconds.
    now+=5000; frame(now); update(now); // Exactly 7 seconds: stop and freeze.
    assert(!LqrApp_IsActive() && motor.mode==MOTOR_STOPPED);
    const volatile TraceStatus *ts=TraceApp_GetStatus(trace);
    assert(ts->mode==TRACE_FROZEN && ts->freeze_reason==TRACE_FREEZE_STOP);
    assert(ts->count==LQR_TRACE_CAPACITY && !LqrApp_GetStatus()->data_valid);
    LqrTraceFrame f, previous; assert(TraceApp_Read(trace,0,&previous,sizeof previous));
    for (unsigned i=1;i<ts->count-1;++i) {
        assert(TraceApp_Read(trace,i,&f,sizeof f)); assert(f.header.valid && f.header.event==TRACE_SAMPLE);
        assert((uint32_t)(f.header.timestamp_us-previous.header.timestamp_us)==5000); previous=f;
    }
    assert(TraceApp_Read(trace,ts->count-1,&f,sizeof f) && !f.header.valid && f.header.event==TRACE_STOP_EVENT);
    uint32_t session=ts->session, total=ts->total_written;
    frame(now); assert(LqrApp_Start(now)); now+=5000; frame(now); update(now);
    assert(ts->session==session && ts->total_written==total); // No implicit restart.
    DebugApp_Stop(now);
    frame(now); assert(DebugApp_StartTrial(now,false));
    state.valid=false; poll(now);
    assert(ts->mode==TRACE_FROZEN && ts->count==1 && ts->freeze_reason==TRACE_FREEZE_FAULT);
    assert(TraceApp_Read(trace,0,&f,sizeof f) && f.header.faults==LQR_BAD_STATE && !f.header.valid);
    LqrTracePayload zero={0}; assert(memcmp(&f.data,&zero,sizeof zero)==0);
    DebugApp_Stop(now); frame(now); state.theta_dot_radps=.1f;
    assert(DebugApp_StartTrial(now,true)); now+=5000; frame(now); update(now);
    float soft=LqrApp_GetStatus()->requested_nm[0];
    DebugApp_Stop(now); frame(now); assert(DebugApp_StartTrial(now,false));
    now+=5000; frame(now); update(now);
    near(soft,LqrApp_GetStatus()->requested_nm[0]*.7f);
    DebugApp_Stop(now);
    tuning.pitch_rate_scale=.5f; tuning.yaw_rate_scale=.5f; assert(LqrApp_SetTuning(&tuning));
    frame(now); assert(DebugApp_StartTrial(now,true));
    assert(!LqrApp_SetTuning(&tuning)); state.valid=false;poll(now);
    assert(LqrApp_GetStatus()->mode==LQR_FAULT);
    near(LqrApp_GetTuning().pitch_rate_scale,.5f);
    near(LqrApp_GetTuning().yaw_rate_scale,.5f);
    DebugApp_Stop(now); frame(now);
    assert(TraceApp_Arm(trace)); assert(LqrApp_Start(now));
    TraceApp_Poll(trace,now);assert(TraceApp_GetStatus(trace)->mode==TRACE_ARMED);
    now+=5000;frame(now);motor.left.rpm=12;motor.right.rpm=34;
    LqrApp_Update(now);uint32_t output_time=LqrApp_GetStatus()->timestamp_us;
    uint32_t output_seq=LqrApp_GetStatus()->sequence;
    motor.left.rpm=99;motor.right.rpm=99;
    TraceApp_Poll(trace,now+100);DebugApp_Stop(now+200);
    assert(TraceApp_Read(trace,0,&f,sizeof f) && f.header.timestamp_us==output_time && f.header.sequence==output_seq);
    near(f.data.yaw_rate_scale,.5f); near(f.data.pitch_rate_scale,.5f);
    near(f.data.relative_rpm[0],12);near(f.data.relative_rpm[1],34);
    // Simultaneous real adapters: LQR fault freezes only LQR; IMU keeps recording.
    assert(TraceApp_Arm(&imu_trace.recorder));
    imu_health.ready=true; imu_health.reads=1;
    imu=(ImuSample){.valid=true,.sequence=1,.timestamp_us=now,.data={{1,2,3},{4,5,6}}};
    frame(now); assert(DebugApp_StartTrial(now,false));
    now+=5000; frame(now); update(now);
    assert(imu_trace.recorder.status.mode==TRACE_RECORDING && trace->status.mode==TRACE_RECORDING);
    state.valid=false; poll(now);
    uint32_t frozen_count=trace->status.count;
    assert(trace->status.mode==TRACE_FROZEN && trace->status.freeze_reason==TRACE_FREEZE_FAULT);
    ++imu.sequence; imu.timestamp_us=now+100; DebugApp_Poll(now+100);
    assert(imu_trace.recorder.status.count==2 && trace->status.count==frozen_count);
    DebugApp_Stop(now+200);
    assert(imu_trace.recorder.status.mode==TRACE_RECORDING);
    TraceApp_Freeze(&imu_trace.recorder,now+300);
    ImuTraceFrame imuf; assert(ImuTrace_Read(&imu_trace,1,&imuf));
    assert(imuf.header.sequence==2 && imuf.data.gyro[2]==6);
    assert(LqrTrace_Read(&lqr_trace,0,&f));
    puts("PASS: LQR/IMU typed instances record together and freeze independently");
    puts("PASS: LQR projection/wrap/sign/units/saturation, clock wrap, duplicates, explicit start, fault latch, timing/stale/range/output failures");
}
