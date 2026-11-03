#include "motion_app.h"
#include "sensor_app.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static uint32_t now;
static ImuSample input;
static SensorStatus sensor={.ready=true};
static uint32_t clock_us(void) { return now; }
const ImuSample *SensorApp_GetSample(void) { return &input; }
const SensorStatus *SensorApp_GetStatus(void) { return &sensor; }
static void publish(void) {
    input.data.accel[2]=1;
    input.timestamp_us=now; ++input.sequence; input.valid=true;
}
int main(int argc,char **argv) {
    (void)argv;
    MotionConfig c=MotionEstimator_DefaultConfig(); c.accel_sign=1;
    assert(MotionApp_Init(&c,clock_us));
    now=UINT32_MAX-1000;
    uint32_t origin=now;
    publish(); MotionApp_Poll();
    assert(MotionApp_GetSample()->sequence==1);
    assert(MotionApp_GetSample()->imu_source.sequence==1);
    assert(MotionApp_GetSample()->imu_source.timestamp_us==input.timestamp_us);
    if (argc>1) {
        uint32_t last_processed=now, last_sequence=1;
        for (uint32_t elapsed=100;elapsed<=1000000;elapsed+=100) {
            now=origin+elapsed;
            if (elapsed%5000==0) publish();
            MotionApp_Poll();
            const MotionSample *s=MotionApp_GetSample();
            if (s->sequence!=last_sequence) {
                assert((uint32_t)(now-last_processed)>=MOTION_MIN_UPDATE_INTERVAL_US);
                assert(s->timestamp_us==input.timestamp_us && s->valid);
                assert(s->imu_source.sequence==input.sequence);
                assert(s->imu_source.timestamp_us==input.timestamp_us);
                last_processed=now;last_sequence=s->sequence;
            }
        }
        unsigned expected=MOTION_UPDATE_HZ<200 ? MOTION_UPDATE_HZ : 200;
        assert(MotionApp_GetStatus()->succ_update_count==expected+1);
        printf("PASS: cap=%u Hz, input=200 Hz, outputs=%u/sec\n",MOTION_UPDATE_HZ,expected);
        return 0;
    }
    /* 新输入提前到达，不能提前消费；边界处使用输入时间戳计算 dt。 */
    now=origin+MOTION_MIN_UPDATE_INTERVAL_US-1;
    publish(); MotionApp_Poll();
    assert(MotionApp_GetSample()->sequence==1);
    assert(MotionApp_GetSample()->imu_source.sequence==1);
    assert(MotionApp_GetSample()->imu_source.timestamp_us==origin);
    now++;
    MotionApp_Poll();
    assert(MotionApp_GetSample()->sequence==2);
    assert(MotionApp_GetSample()->timestamp_us==input.timestamp_us);
    assert(fabsf(MotionApp_GetStatus()->last_dt_s-
        (MOTION_MIN_UPDATE_INTERVAL_US-1)*0.000001f)<0.000001f);
    MotionApp_Poll(); assert(MotionApp_GetSample()->sequence==2);
    /* 周期到达但没有新输入，不能推进处理时间。 */
    now+=MOTION_MIN_UPDATE_INTERVAL_US;
    MotionApp_Poll(); assert(MotionApp_GetSample()->sequence==2);
    now++;publish();MotionApp_Poll();assert(MotionApp_GetSample()->sequence==3);
    /* 周期尚未到，输入失效也必须立即处理。 */
    SampleSource previous=MotionApp_GetSample()->imu_source;
    input.valid=false;now++;MotionApp_Poll();
    assert(MotionApp_GetSample()->imu_source.sequence==previous.sequence);
    assert(MotionApp_GetSample()->imu_source.timestamp_us==previous.timestamp_us);
    assert(!MotionApp_GetSample()->valid);
    assert(MotionApp_GetStatus()->state==MOTION_INPUT_UNAVAILABLE);
    assert(MotionApp_GetStatus()->fail_update_count==0);
    publish();MotionApp_Poll();assert(MotionApp_GetSample()->sequence==3);
    now+=MOTION_MIN_UPDATE_INTERVAL_US;publish();MotionApp_Poll();
    assert(MotionApp_GetSample()->valid && MotionApp_GetSample()->sequence==4);
    now+=IMU_TIMEOUT_US;MotionApp_Poll();
    assert(!MotionApp_GetSample()->valid);
    assert(MotionApp_GetStatus()->state==MOTION_INPUT_TIMEOUT);
    printf("PASS: cap=%u Hz, gate boundary/wrap/dt/idle/invalid/timeout\n",MOTION_UPDATE_HZ);
}
