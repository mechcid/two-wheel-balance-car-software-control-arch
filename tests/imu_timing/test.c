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
    *out=(MPU6050_Sample){.az=16384,.gx=131};
    return read_result;
}
int main(void)
{
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
    assert(SensorApp_Init(&bus,0)==HAL_OK);
    assert(!SensorApp_GetSample()->valid);
    assert(SensorApp_Update()==HAL_OK);
    ImuSample previous=*SensorApp_GetSample();
    assert(previous.sequence==1 && previous.timestamp_us==now);
    assert(previous.data.accel[2]==1 && previous.data.gyro[0]==1);
    ready_result=HAL_BUSY;
    assert(SensorApp_Update()==HAL_BUSY);
    assert(SensorApp_GetSample()->sequence==1 && SensorApp_GetSample()->valid);
    advance(IMU_TIMEOUT_US);
    assert(SensorApp_Update()==HAL_BUSY);
    assert(!SensorApp_GetSample()->valid);
    assert(SensorApp_GetSample()->timestamp_us==previous.timestamp_us);
    assert(SensorApp_GetStatus()->failures==0);
    ready_result=HAL_OK;
    read_result=HAL_ERROR;
    assert(SensorApp_Update()==HAL_ERROR);
    assert(SensorApp_GetSample()->sequence==1 && !SensorApp_GetStatus()->ready);
    assert(SensorApp_Reconnect()==HAL_OK);
    assert(!SensorApp_GetSample()->valid && SensorApp_GetSample()->sequence==1);
    read_result=HAL_OK;
    MotionConfig config=MotionEstimator_DefaultConfig();
    config.accel_sign=1;
    /* 让第一次 5 ms 截止时间跨过 uint32_t 微秒回绕。 */
    while (Timebase_NowUs()!=UINT32_MAX-3000U) {
        uint32_t remaining=(UINT32_MAX-3000U)-Timebase_NowUs();
        advance(remaining>50000000U ? 50000000U : remaining);
    }
    assert(MotionApp_Init(&config));
    unsigned before=reads;
    advance(4999);
    MotionApp_Poll();
    assert(reads==before && MotionApp_TimeUntilSampleUs()==1);
    advance(1);
    MotionApp_Poll();
    assert(reads==before+1 && SensorApp_GetSample()->sequence==2);
    assert(MotionApp_GetState()->valid);
    assert(MotionApp_TimeUntilSampleUs()==3400);
    advance(3400);
    MotionApp_Poll();
    assert(fabsf(MotionApp_GetStatus()->last_dt_s-0.005f)<0.000001f);
    advance(15000);
    before=reads;
    MotionApp_Poll();
    assert(reads==before+1 && MotionApp_GetStatus()->missed_periods==2);
    ready_result=HAL_BUSY;
    advance(IMU_TIMEOUT_US);
    MotionApp_Poll();
    assert(!MotionApp_GetState()->valid && !SensorApp_GetSample()->valid);
    ready_result=HAL_OK;
    advance(IMU_PERIOD_US);
    MotionApp_Poll();
    assert(MotionApp_GetState()->valid && MotionApp_GetStatus()->last_dt_s==0);
    puts("PASS: DWT/us wrap, fractional cycles, IRQ mask, sample lifecycle, 200 Hz deadlines, gaps/recovery");
}
