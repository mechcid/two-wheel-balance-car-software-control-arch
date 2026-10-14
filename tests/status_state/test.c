#include "status_state_app.h"
#include "odometry_app.h"
#include "motor_app.h"
#include "motion_app.h"
#include "sensor_app.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static MotionState pose;
static MotionAppStatus timing;
static ImuSample imu;
static float angles[3];
static MotorAppFeedbackCallback callback;
static MotorAppStatus motor;
const MotionState *MotionApp_GetState(void) { return &pose; }
const MotionAppStatus *MotionApp_GetStatus(void) { return &timing; }
const ImuSample *SensorApp_GetSample(void) { return &imu; }
void MotionApp_GetEuler(float out[3]) { for(int i=0;i<3;i++) out[i]=angles[i]; }
void MotorApp_SetFeedbackCallback(MotorAppFeedbackCallback cb) { callback=cb; }
void MotorApp_Update(uint32_t now) { (void)now; callback(&motor); }
static void near(float a,float b) { assert(fabsf(a-b)<0.0001f); }
static void frame(uint32_t t, uint32_t seq, float left, float right) {
    motor.timestamp_us=t; motor.sequence=seq; motor.interval_us=10000; motor.valid=true;
    motor.left.delta_displacement_m=left; motor.right.delta_displacement_m=right;
    motor.left.velocity_mps=left*100; motor.right.velocity_mps=right*100;
    motor.left.feedback_valid=motor.right.feedback_valid=true;
    timing.timestamp_us=imu.timestamp_us=t-2000;
    callback(&motor);
}
int main(void) {
    OdometryState o; OdometryApp_Init(&o);
    OdometryApp_Update(&o,0,0,(0)*100,(0)*100,0,UINT32_MAX,UINT32_MAX-10000,10000,true);
    OdometryApp_Update(&o,0.01f,0.03f,(0.01f)*100,(0.03f)*100,0,0,0,10000,true);
    near(o.x_m,0.02f); near(o.v_mps,2); assert(o.continuous);
    OdometryApp_Update(&o,1,1,(1)*100,(1)*100,0,0,0,10000,true); near(o.x_m,0.02f);
    OdometryApp_Update(&o,-0.01f,-0.01f,(-0.01f)*100,(-0.01f)*100,0,1,10000,10000,true); near(o.x_m,0.01f);
    OdometryApp_Update(&o,1,1,(1)*100,(1)*100,0,3,30000,10000,true); assert(!o.continuous); near(o.x_m,0.01f);
    OdometryApp_Init(&o);
    OdometryApp_Update(&o,0,0,(0)*100,(0)*100,3.12413936f,1,10000,10000,true);
    OdometryApp_Update(&o,0.01f,0.01f,(0.01f)*100,(0.01f)*100,-3.12413936f,2,20000,10000,true);
    near(o.x_m,-0.01f); near(o.y_m,0);
    OdometryApp_Update(&o,1,1,(1)*100,(1)*100,NAN,3,30000,10000,true); assert(!o.valid && !o.continuous);
    OdometryApp_Init(&o);
    OdometryApp_Update(&o,0,0,(0)*100,(0)*100,1.570796327f,1,10000,10000,true);
    OdometryApp_Update(&o,0.02f,0.02f,(0.02f)*100,(0.02f)*100,1.570796327f,2,20000,10000,true);
    near(o.x_m,0); near(o.y_m,0.02f);
    StatusStateApp_Init(); pose.valid=true; imu.valid=true;
    frame(10000,1,0,0); assert(StatusStateApp_GetState()->valid);
    frame(20000,2,0.01f,0.03f);
    motor.sequence++; motor.left.velocity_mps=3; motor.right.velocity_mps=5;
    motor.left.delta_displacement_m=motor.right.delta_displacement_m=0; callback(&motor);
    float v[7]; assert(StatusStateApp_GetVector(v)); near(v[0],0.02f); near(v[4],4);
    uint32_t seq=StatusStateApp_GetState()->sequence;
    callback(&motor); assert(StatusStateApp_GetState()->sequence==seq);
    motor.sequence=4; motor.timestamp_us=21000; motor.valid=false;
    StatusStateApp_Update(21000); assert(!StatusStateApp_GetState()->valid);
    assert(StatusStateApp_GetState()->faults&STATUS_STATE_MOTOR_INVALID);
    StatusStateApp_Init(); frame(20000,2,0,0);
    seq=StatusStateApp_GetState()->sequence;
    angles[0]=30; angles[1]=20; angles[2]=179;
    imu.data.gyro[1]=10; imu.data.gyro[2]=20;
    frame(30000,3,0,0); assert(StatusStateApp_GetVector(v));
    near(v[2],20*0.01745329252f);
    near(v[5],(10*cosf(0.523598776f)-20*sinf(0.523598776f))*0.01745329252f);
    near(v[6],(10*sinf(0.523598776f)+20*cosf(0.523598776f))*0.01745329252f/cosf(v[2]));
    angles[2]=-179; frame(40000,4,0,0); near(StatusStateApp_GetState()->psi_rad,181*0.01745329252f);
    StatusStateApp_Poll(60001); assert(!StatusStateApp_GetVector(v));
    assert(StatusStateApp_GetState()->sequence==seq+2);
    motor.sequence=5; motor.timestamp_us=70000; callback(&motor);
    assert(StatusStateApp_GetState()->faults&STATUS_STATE_TIME_MISMATCH);
    frame(80000,6,0.01f,0.01f);
    assert(!StatusStateApp_GetState()->valid);
    assert(StatusStateApp_GetState()->faults&STATUS_STATE_POSITION_DISCONTINUOUS);
    StatusStateApp_Init(); angles[0]=angles[1]=angles[2]=0;
    frame(100000,1,0,0); pose.valid=false; StatusStateApp_Poll(100001);
    pose.valid=true; frame(110000,2,0,0); assert(!StatusStateApp_GetState()->valid);
    motor.sequence++; motor.left.feedback_valid=false; callback(&motor);
    assert(StatusStateApp_GetState()->faults&STATUS_STATE_MOTOR_INVALID);
    StatusStateApp_Init(); angles[1]=90; frame(120000,1,0,0);
    assert(StatusStateApp_GetState()->faults&STATUS_STATE_ATTITUDE_INVALID);
    puts("PASS: signed odometry, wrap/midpoint, duplicate/missed samples, state order/units/rates, stale data and continuity");
}
