#include "timebase.h"
#include "sensor_app.h"
#include "mpu6050.h"
#include "motion_app.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

TestDWT test_dwt;
TestCoreDebug test_debug;
uint32_t test_mask;
static uint64_t total_cycles;
static HAL_StatusTypeDef ready_result=HAL_OK, read_result=HAL_OK;
static unsigned reads;
static unsigned clock_calls;
static unsigned motion_clock_calls;
static int16_t accel_z=16384;
static uint32_t motion_clock(void)
{
    ++motion_clock_calls;
    return Timebase_NowUs();
}
static uint32_t sensor_clock(void)
{
    ++clock_calls;
    return Timebase_NowUs();
}
static void cycles(uint32_t n)
{
    total_cycles+=n;
    test_dwt.CYCCNT+=n;
    assert(Timebase_NowUs()==(uint32_t)(total_cycles/72));
}
static void advance(uint32_t us) { cycles(us*72U); }
HAL_StatusTypeDef MPU6050_Init(MPU6050 *d, I2C_HandleTypeDef *bus)
{
    *d=(MPU6050){.bus=bus,.ready=1};
    return HAL_OK;
}
HAL_StatusTypeDef MPU6050_SetOutputRate(MPU6050 *d, float hz, float *actual)
{
    (void)d;
    assert(hz==200);
    *actual=hz;
    return HAL_OK;
}
HAL_StatusTypeDef MPU6050_DataReady(MPU6050 *d) { (void)d; return ready_result; }
HAL_StatusTypeDef MPU6050_Read(MPU6050 *d, MPU6050_Sample *out)
{
    (void)d;
    ++reads;
    advance(1600); /* 模拟阻塞读取，验证时间戳落在事务开始。 */
    *out=(MPU6050_Sample){.az=accel_z,.gx=131};
    return read_result;
}
static void poll_next_sample(void)
{
    advance(SensorApp_TimeUntilSampleUs());
    SensorApp_Poll();
}
int main(void)
{
    /* 分别绕三个轴旋转 90 度，验证具名欧拉角字段没有交换。 */
    for (unsigned axis=0;axis<3;++axis) {
        MotionEstimator e={0};
        e.state.q[0]=sqrtf(0.5f);
        e.state.q[axis+1]=sqrtf(0.5f);
        EulerAnglesDeg angles;
        MotionEstimator_GetEuler(&e,&angles);
        assert(fabsf(angles.roll-(axis==0 ? 90.0f : 0.0f))<0.05f);
        assert(fabsf(angles.pitch-(axis==1 ? 90.0f : 0.0f))<0.05f);
        assert(fabsf(angles.yaw-(axis==2 ? 90.0f : 0.0f))<0.05f);
    }
    assert(Timebase_Init());
    for (unsigned i=0;i<73;++i) cycles(1);
    cycles(UINT32_MAX-100U);
    cycles(200); /* 原始周期回绕。 */
    for (unsigned i=0;i<80;++i) cycles(4000000000U); /* 微秒回绕。 */
    test_mask=1;
    cycles(71);
    assert(test_mask==1);
    test_mask=0;
    uint32_t now=Timebase_NowUs();
    assert(Timebase_Init() && Timebase_NowUs()==now); /* 重复初始化不重置。 */
    I2C_HandleTypeDef bus={0};
    SensorApp_Poll();
    assert(SensorApp_TimeUntilSampleUs()==0 && reads==0);
    assert(SensorApp_Init(&bus,0,0)==HAL_ERROR);
    assert(SensorApp_Init(&bus,0,sensor_clock)==HAL_OK);
    assert(!SensorApp_GetSample()->valid);
    assert(SensorApp_TimeUntilSampleUs()==IMU_PERIOD_US);
    SensorApp_Poll();
    advance(IMU_PERIOD_US-1);
    SensorApp_Poll();
    assert(reads==0 && SensorApp_TimeUntilSampleUs()==1);
    advance(1);
    unsigned calls_before=clock_calls;
    SensorApp_Poll();
    assert(clock_calls==calls_before+2); /* 周期判断和读取开始分别取时。 */
    ImuSample previous=*SensorApp_GetSample();
    assert(previous.sequence==1 && previous.timestamp_us==now+IMU_PERIOD_US);
    assert(previous.data.accel[2]==1 && previous.data.gyro[0]==1);
    assert(SensorApp_TimeUntilSampleUs()==3400);
    SensorApp_Poll();
    assert(reads==1);
    assert(SensorApp_Init(&bus,0,0)==HAL_ERROR);
    assert(SensorApp_GetSample()->valid && SensorApp_GetSample()->sequence==1);
    ready_result=HAL_BUSY;
    poll_next_sample();
    assert(SensorApp_GetSample()->sequence==1 && SensorApp_GetSample()->valid);
    advance(IMU_TIMEOUT_US);
    SensorApp_Poll();
    assert(!SensorApp_GetSample()->valid);
    assert(SensorApp_GetSample()->timestamp_us==previous.timestamp_us);
    assert(SensorApp_GetStatus()->fail_read_count==0);
    ready_result=HAL_OK;
    read_result=HAL_ERROR;
    poll_next_sample();
    assert(SensorApp_GetSample()->sequence==1 && !SensorApp_GetStatus()->ready);
    assert(SensorApp_GetStatus()->fail_read_count==1);
    assert(SensorApp_Reconnect()==HAL_OK);
    assert(!SensorApp_GetSample()->valid && SensorApp_GetSample()->sequence==1);
    assert(SensorApp_TimeUntilSampleUs()==IMU_PERIOD_US);
    read_result=HAL_OK;
    MotionConfig config=MotionEstimator_DefaultConfig();
    config.accel_sign=1;
    while (Timebase_NowUs()!=UINT32_MAX-3000U) {
        uint32_t remaining=(UINT32_MAX-3000U)-Timebase_NowUs();
        advance(remaining>50000000U ? 50000000U : remaining);
    }
    assert(SensorApp_Reconnect()==HAL_OK); /* 建立跨回绕的采集计划。 */
    assert(MotionApp_GetStatus()->state==MOTION_NOT_INITIALIZED);
    assert(!MotionApp_Init(&config,0));
    MotionApp_Poll();
    assert(motion_clock_calls==0);
    assert(MotionApp_Init(&config,motion_clock));
    assert(MotionApp_GetStatus()->state==MOTION_WAITING_INPUT);
    assert(!MotionApp_Init(&config,motion_clock));
    unsigned before=reads;
    advance(4999);
    SensorApp_Poll();
    MotionApp_Poll();
    assert(reads==before && !MotionApp_GetSample()->valid);
    assert(SensorApp_TimeUntilSampleUs()==1);
    advance(1);
    MotionApp_Poll(); /* 消费者不会触发采集。 */
    assert(reads==before);
    SensorApp_Poll();
    MotionApp_Poll();
    assert(reads==before+1 && MotionApp_GetSample()->valid);
    assert(MotionApp_GetStatus()->state==MOTION_RUNNING);
    assert(MotionApp_GetSample()->sequence==1);
    assert(MotionApp_GetSample()->timestamp_us==SensorApp_GetSample()->timestamp_us);
    assert(fabsf(MotionApp_GetSample()->data.pitch)<0.001f);
    uint32_t updates=MotionApp_GetStatus()->succ_update_count;
    MotionApp_Poll();
    assert(MotionApp_GetStatus()->succ_update_count==updates);
    poll_next_sample();
    MotionApp_Poll();
    assert(fabsf(MotionApp_GetStatus()->last_dt_s-0.005f)<0.000001f);
    /* 错过多个周期只采集一次，并保持原计划的相位。 */
    before=reads;
    advance(15000);
    SensorApp_Poll();
    assert(reads==before+1 && SensorApp_TimeUntilSampleUs()==1800);
    SensorApp_Poll();
    assert(reads==before+1);
    MotionApp_Poll();
    ready_result=HAL_BUSY;
    poll_next_sample();
    MotionApp_Poll();
    assert(MotionApp_GetSample()->valid);
    advance(IMU_TIMEOUT_US);
    MotionApp_Poll();
    assert(!MotionApp_GetSample()->valid && SensorApp_GetSample()->valid);
    assert(MotionApp_GetStatus()->state==MOTION_INPUT_TIMEOUT);
    assert(MotionApp_GetSample()->discontinuity);
    SensorApp_Poll();
    assert(!SensorApp_GetSample()->valid);
    ready_result=HAL_OK;
    poll_next_sample();
    MotionApp_Poll();
    assert(MotionApp_GetSample()->valid && MotionApp_GetStatus()->last_dt_s==0);
    read_result=HAL_ERROR;
    poll_next_sample();
    MotionApp_Poll();
    assert(!MotionApp_GetSample()->valid);
    assert(MotionApp_GetStatus()->state==MOTION_INPUT_UNAVAILABLE);
    assert(SensorApp_Reconnect()==HAL_OK);
    MotionApp_Poll();
    assert(!MotionApp_GetSample()->valid);
    read_result=HAL_OK;
    uint32_t sequence=SensorApp_GetSample()->sequence;
    poll_next_sample();
    poll_next_sample(); /* 不运行姿态模块，仍持续生产样本。 */
    assert(SensorApp_GetSample()->sequence==sequence+2);
    MotionApp_Poll();
    assert(MotionApp_GetSample()->valid && MotionApp_GetStatus()->last_dt_s==0);
    /* 恢复时不可信的加速度使算法失败；不能刷新旧样本的时间/序号。 */
    MotionSample published=*MotionApp_GetSample();
    assert(SensorApp_Reconnect()==HAL_OK);
    MotionApp_Poll();
    accel_z=0;
    poll_next_sample();
    uint32_t failed=MotionApp_GetStatus()->fail_update_count;
    MotionApp_Poll();
    assert(MotionApp_GetStatus()->state==MOTION_ESTIMATION_FAILED);
    assert(MotionApp_GetStatus()->fail_update_count==failed+1);
    assert(!MotionApp_GetSample()->valid);
    assert(MotionApp_GetSample()->timestamp_us==published.timestamp_us);
    assert(MotionApp_GetSample()->sequence==published.sequence);
    assert(MotionApp_GetSample()->data.roll==published.data.roll);
    MotionApp_Poll();
    assert(MotionApp_GetStatus()->fail_update_count==failed+1);
    accel_z=16384;
    poll_next_sample();
    MotionApp_Poll();
    assert(MotionApp_GetSample()->valid && MotionApp_GetSample()->discontinuity);
    assert(MotionApp_GetSample()->sequence==published.sequence+1);
    assert(MotionApp_GetSample()->timestamp_us==SensorApp_GetSample()->timestamp_us);
    assert(MotionApp_GetStatus()->state==MOTION_RUNNING);
    puts("PASS: private update/public poll, deadlines/wrap, independent acquisition, stale data, faults/recovery");
}
