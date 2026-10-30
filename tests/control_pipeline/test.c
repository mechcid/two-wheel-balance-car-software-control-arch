#include "sensor_app.h"
#include "mpu6050.h"
#include "motion_app.h"
#include "status_state_app.h"
#include "motor_app.h"
#include "lqr_app.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

static uint32_t now_us;
uint32_t Timebase_NowUs(void) { return now_us; }
static MotorAppStatus motor;
static MotorAppFeedbackCallback feedback;
static unsigned output_count;
HAL_StatusTypeDef MPU6050_Init(MPU6050 *d,I2C_HandleTypeDef *bus) {
    *d=(MPU6050){.bus=bus,.ready=1}; return HAL_OK;
}
HAL_StatusTypeDef MPU6050_SetOutputRate(MPU6050 *d,float hz,float *actual) {
    (void)d; *actual=hz; return HAL_OK;
}
HAL_StatusTypeDef MPU6050_DataReady(MPU6050 *d) { (void)d; return HAL_OK; }
HAL_StatusTypeDef MPU6050_Read(MPU6050 *d,MPU6050_Sample *s) {
    (void)d; *s=(MPU6050_Sample){.az=16384}; now_us+=1600; return HAL_OK;
}
const MotorAppStatus *MotorApp_GetStatus(void) { return &motor; }
void MotorApp_SetFeedbackCallback(MotorAppFeedbackCallback cb) { feedback=cb; }
void MotorApp_Update(uint32_t now) {
    motor.interval_us=now-motor.timestamp_us; motor.timestamp_us=now;
    ++motor.sequence; motor.valid=true;
    motor.left.feedback_valid=motor.right.feedback_valid=true;
    feedback(&motor);
}
void MotorApp_Stop(uint32_t now) { (void)now; motor.mode=MOTOR_STOPPED; }
bool MotorApp_ApplyDuty(float l,float r,uint32_t now) {
    (void)l; (void)r; (void)now; ++output_count; motor.mode=MOTOR_DUTY; return true;
}
/* 从 main.c 提取真实的采集、姿态、状态与控制执行段，防止测试复制旧调度。 */
#include "pipeline.h"
int main(int argc,char **argv) {
    unsigned delay=argc>1 ? (unsigned)strtoul(argv[1],0,10) : 13000;
    I2C_HandleTypeDef bus={0};
    assert(SensorApp_Init(&bus,0,Timebase_NowUs)==HAL_OK);
    now_us+=delay; /* 模拟传感器之后的 OLED 等初始化，改变采集/控制相位。 */
    MotionConfig motion=MotionEstimator_DefaultConfig(); motion.accel_sign=1;
    assert(MotionApp_Init(&motion,Timebase_NowUs));
    assert(StatusStateApp_Init(.0325f,.0325f));
    LqrConfig config=LqrApp_DefaultConfig(); LqrTuning tuning=LqrApp_DefaultTuning();
    assert(LqrApp_Init(&config,&tuning));
    motor.timestamp_us=now_us;
    pipeline_init();
    bool armed=false;
    for (unsigned i=0;i<1000;++i) {
        pipeline_iteration();
        /* 启动命令由用户在初始化结束后发送，先等待启动阶段的采样节奏稳定。 */
        if (!armed && StatusStateApp_GetState()->valid && StatusStateApp_GetState()->sequence>=3) {
            assert(LqrApp_Start(now_us)); armed=true;
        }
        if (armed && !LqrApp_IsActive()) {
            fprintf(stderr,"FAIL delay=%u now=%u state=%u attitude=%u fault=%u outputs=%u\n",
                delay,now_us,StatusStateApp_GetState()->timestamp_us,
                StatusStateApp_GetState()->attitude_timestamp_us,LqrApp_GetStatus()->faults,output_count);
            return 1;
        }
        now_us+=100;
    }
    assert(armed && output_count>10);
    /* 停止生产后保护必须仍然生效。 */
    now_us+=8000; LqrApp_Poll(now_us);
    assert(!LqrApp_IsActive());
    printf("PASS: pipeline phase=%u, outputs=%u, stale protection retained\n",delay%5000,output_count);
}
