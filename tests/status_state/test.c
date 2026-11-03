#include "status_state_app.h"
#include "odometry_app.h"
#include "motor_app.h"
#include "motion_app.h"
#include "sensor_app.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static MotionSample pose;
static ImuSample imu;
static MotorAppFeedbackCallback callback;
static MotorAppStatus motor;
const MotionSample *MotionApp_GetSample(void) { return &pose; }
const ImuSample *SensorApp_GetSample(void) { return &imu; }
void MotorApp_SetFeedbackCallback(MotorAppFeedbackCallback cb) { callback=cb; }
void MotorApp_Update(uint32_t now) { (void)now; callback(&motor); }
static void near(float a,float b) { assert(fabsf(a-b)<0.0001f); }
static void frame(uint32_t t, uint32_t seq, float left, float right) {
    motor.timestamp_us=t; motor.sequence=seq; motor.interval_us=10000; motor.valid=true;
    motor.left.delta_displacement_m=left; motor.right.delta_displacement_m=right;
    motor.left.velocity_mps=left*100; motor.right.velocity_mps=right*100;
    motor.left.feedback_valid=motor.right.feedback_valid=true;
    pose.timestamp_us=imu.timestamp_us=t-2000;
    ++pose.sequence;
    imu.sequence+=2; /* 模拟 IMU 与姿态输出不同频，不能比较两者输出序号。 */
    pose.imu_source=(SampleSource){imu.timestamp_us,imu.sequence};
    callback(&motor);
}
int main(void) {
    assert(StatusStateApp_GetStatus()->state==STATUS_STATE_NOT_INITIALIZED);
    StatusStateApp_Poll(123);
    assert(StatusStateApp_GetStatus()->state==STATUS_STATE_NOT_INITIALIZED);
    OdometryState o=OdometryApp_Init();
    OdometryApp_Update(&o,0,0,(0)*100,(0)*100,0,UINT32_MAX,UINT32_MAX-10000,10000,true);
    OdometryApp_Update(&o,0.01f,0.03f,(0.01f)*100,(0.03f)*100,0,0,0,10000,true);
    near(o.x_m,0.02f); near(o.v_mps,2); assert(o.continuous);
    OdometryApp_Update(&o,1,1,(1)*100,(1)*100,0,0,0,10000,true); near(o.x_m,0.02f);
    OdometryApp_Update(&o,-0.01f,-0.01f,(-0.01f)*100,(-0.01f)*100,0,1,10000,10000,true); near(o.x_m,0.01f);
    OdometryApp_Update(&o,1,1,(1)*100,(1)*100,0,3,30000,10000,true); assert(!o.continuous); near(o.x_m,0.01f);
    o=OdometryApp_Init();
    OdometryApp_Update(&o,0,0,(0)*100,(0)*100,3.12413936f,1,10000,10000,true);
    OdometryApp_Update(&o,0.01f,0.01f,(0.01f)*100,(0.01f)*100,-3.12413936f,2,20000,10000,true);
    near(o.x_m,-0.01f); near(o.y_m,0);
    OdometryApp_Update(&o,1,1,(1)*100,(1)*100,NAN,3,30000,10000,true); assert(!o.valid && !o.continuous);
    o=OdometryApp_Init();
    OdometryApp_Update(&o,0,0,(0)*100,(0)*100,1.570796327f,1,10000,10000,true);
    OdometryApp_Update(&o,0.02f,0.02f,(0.02f)*100,(0.02f)*100,1.570796327f,2,20000,10000,true);
    near(o.x_m,0); near(o.y_m,0.02f);
    assert(StatusStateApp_Init(0.0325f,0.0325f)); pose.valid=true; imu.valid=true;
    StatusStateApp_Poll(100000);
    assert(StatusStateApp_GetStatus()->state==STATUS_STATE_WAITING_INPUT);
    frame(10000,1,0,0); assert(StatusStateApp_GetState()->valid);
    frame(20000,2,0.01f,0.03f);
    motor.sequence++; motor.left.velocity_mps=3; motor.right.velocity_mps=5;
    motor.left.delta_displacement_m=motor.right.delta_displacement_m=0; callback(&motor);
    float v[7]; assert(StatusStateApp_GetVector(v)); near(v[0],0.02f); near(v[4],4);
    uint32_t seq=StatusStateApp_GetState()->sequence;
    callback(&motor); assert(StatusStateApp_GetState()->sequence==seq);
    StatusState previous=*StatusStateApp_GetState();
    assert(StatusStateApp_GetState()->sequence==3);
    motor.sequence=4; motor.timestamp_us=21000; motor.valid=false;
    MotorApp_Update(21000); StatusStateApp_Poll(21000); assert(!StatusStateApp_GetState()->valid);
    assert(StatusStateApp_GetStatus()->faults&STATUS_STATE_MOTOR_INVALID);
    assert(StatusStateApp_GetState()->sequence==previous.sequence);
    assert(StatusStateApp_GetState()->timestamp_us==previous.timestamp_us);
    previous.valid=false;
    assert(memcmp(StatusStateApp_GetState(),&previous,sizeof previous)==0);
    near(StatusStateApp_GetState()->x_m,previous.x_m);
    near(StatusStateApp_GetState()->v_mps,previous.v_mps);
    assert(StatusStateApp_GetStatus()->state==STATUS_STATE_UNAVAILABLE);
    // 同一失败帧即使内容改变也不能重新消费，必须等待新序号。
    motor.valid=true;
    callback(&motor); StatusStateApp_Poll(21001);
    assert(StatusStateApp_GetStatus()->faults&STATUS_STATE_MOTOR_INVALID);
    assert(memcmp(StatusStateApp_GetState(),&previous,sizeof previous)==0);
    assert(StatusStateApp_GetStatus()->state==STATUS_STATE_UNAVAILABLE);
    assert(StatusStateApp_Init(0.0325f,0.0325f)); frame(20000,2,0,0);
    seq=StatusStateApp_GetState()->sequence;
    pose.data.roll=30; pose.data.pitch=20; pose.data.yaw=179;
    imu.data.gyro[1]=10; imu.data.gyro[2]=20;
    frame(30000,3,0,0); assert(StatusStateApp_GetVector(v));
    near(v[2],20*0.01745329252f);
    near(v[5],(10*cosf(0.523598776f)-20*sinf(0.523598776f))*0.01745329252f);
    near(v[6],(10*sinf(0.523598776f)+20*cosf(0.523598776f))*0.01745329252f/cosf(v[2]));
    pose.data.yaw=-179; frame(40000,4,0,0); near(StatusStateApp_GetState()->psi_rad,181*0.01745329252f);
    StatusStateApp_Poll(60001); assert(!StatusStateApp_GetVector(v));
    assert(StatusStateApp_GetState()->sequence==seq+2);
    motor.sequence=5; motor.timestamp_us=70000; callback(&motor);
    assert(StatusStateApp_GetStatus()->faults&STATUS_STATE_TIME_MISMATCH);
    frame(80000,6,0.01f,0.01f);
    assert(!StatusStateApp_GetState()->valid);
    assert(StatusStateApp_GetStatus()->faults&STATUS_STATE_POSITION_DISCONTINUOUS);
    assert(StatusStateApp_Init(0.0325f,0.0325f)); pose.data.roll=pose.data.pitch=pose.data.yaw=0;
    frame(100000,1,0,0); pose.valid=false; StatusStateApp_Poll(100001);
    pose.valid=true; frame(110000,2,0,0); assert(!StatusStateApp_GetState()->valid);
    motor.sequence++; motor.left.feedback_valid=false; callback(&motor);
    assert(StatusStateApp_GetStatus()->faults&STATUS_STATE_MOTOR_INVALID);
    assert(StatusStateApp_Init(0.0325f,0.0325f)); pose.data.pitch=90; frame(120000,1,0,0);
    assert(StatusStateApp_GetStatus()->faults&STATUS_STATE_ATTITUDE_INVALID);
    // 两轮对地不转：编码器读数为 -radius * theta，补偿后 v/x/y 应为零。
    assert(!StatusStateApp_Init(0,0.0325f));
    assert(!StatusStateApp_Init(NAN,0.0325f));
    assert(StatusStateApp_Init(0.03f,0.04f));
    pose.data.roll=pose.data.pitch=pose.data.yaw=0;
    imu.data.gyro[1]=imu.data.gyro[2]=0;
    frame(130000,1,0,0);
    const float pitch_step=0.01f;
    pose.data.pitch=pitch_step/0.01745329252f;
    imu.data.gyro[1]=1/0.01745329252f;
    frame(140000,2,-0.03f*pitch_step,-0.04f*pitch_step);
    assert(StatusStateApp_GetVector(v)); near(v[0],0); near(v[1],0); near(v[4],0);
    callback(&motor); near(StatusStateApp_GetState()->x_m,0);
    // 相反方向俯仰也抵消，使用不同轮半径以验证逐轮换算。
    pose.data.pitch=0; imu.data.gyro[1]=-1/0.01745329252f;
    frame(150000,3,0.03f*pitch_step,0.04f*pitch_step);
    assert(StatusStateApp_GetVector(v)); near(v[0],0); near(v[4],0);
    // 同时真实前进 1 mm，补偿只消除车身俯仰部分。
    pose.data.pitch=pitch_step/0.01745329252f; imu.data.gyro[1]=1/0.01745329252f;
    frame(160000,4,0.001f-0.03f*pitch_step,0.001f-0.04f*pitch_step);
    assert(StatusStateApp_GetVector(v)); near(v[0],0.001f); near(v[4],0.1f);
    // 不依赖外部轮询也完整消费连续反馈，包括时间/电机序号回绕。
    assert(StatusStateApp_Init(0.03f,0.04f));
    pose.data.pitch=0; imu.data.gyro[1]=0;
    frame(UINT32_MAX-9999,UINT32_MAX,0,0);
    frame(0,0,0.01f,0.01f);
    frame(10000,1,0.02f,0.02f);
    near(StatusStateApp_GetState()->x_m,0.03f);
    assert(StatusStateApp_GetState()->sequence==3);
    StatusStateApp_Poll(30000);
    assert(StatusStateApp_GetState()->valid);
    StatusStateApp_Poll(30001);
    assert(!StatusStateApp_GetState()->valid);
    assert(StatusStateApp_GetStatus()->faults&STATUS_STATE_STALE);
    // 首帧失败不发布；下一个可信输入可建立初始基准。
    assert(StatusStateApp_Init(0.03f,0.04f));
    pose.valid=false; frame(40000,2,0,0);
    assert(StatusStateApp_GetState()->sequence==0);
    assert(StatusStateApp_GetStatus()->state==STATUS_STATE_UNAVAILABLE);
    pose.valid=true; frame(50000,3,0,0);
    assert(StatusStateApp_GetState()->valid);
    assert(StatusStateApp_GetState()->sequence==1);
    assert(StatusStateApp_GetStatus()->faults==0);
    assert(StatusStateApp_GetStatus()->state==STATUS_STATE_RUNNING);
    // 三个来源完整发布；新 IMU 即使时间戳相同，序号不同也不能混用。
    assert(StatusStateApp_GetState()->motion.sequence==pose.sequence);
    assert(StatusStateApp_GetState()->motion.timestamp_us==pose.timestamp_us);
    assert(StatusStateApp_GetState()->imu.sequence==imu.sequence);
    assert(StatusStateApp_GetState()->imu.timestamp_us==imu.timestamp_us);
    assert(StatusStateApp_GetState()->motor.sequence==motor.sequence);
    assert(StatusStateApp_GetState()->motor.timestamp_us==motor.timestamp_us);
    previous=*StatusStateApp_GetState(); previous.valid=false;
    ++imu.sequence; ++motor.sequence; callback(&motor);
    assert(StatusStateApp_GetStatus()->faults&STATUS_STATE_TIME_MISMATCH);
    assert(memcmp(StatusStateApp_GetState(),&previous,sizeof previous)==0);
    // 同序号但来源时间不同也拒绝；失败首帧不产生来源记录。
    assert(StatusStateApp_Init(0.03f,0.04f));
    pose.imu_source.sequence=imu.sequence;
    pose.imu_source.timestamp_us=imu.timestamp_us-1;
    callback(&motor);
    assert(StatusStateApp_GetStatus()->faults&STATUS_STATE_TIME_MISMATCH);
    assert(StatusStateApp_GetState()->sequence==0);
    puts("PASS: publication integrity, failed-frame deduplication, synchronous increments and status transitions");
    puts("PASS: body pitch compensation, both signs, unequal radii, translation plus pitch");
    puts("PASS: signed odometry, wrap/midpoint, duplicate/missed samples, state order/units/rates, stale data and continuity");
}
