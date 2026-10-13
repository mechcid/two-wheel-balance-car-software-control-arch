#include "motor_app.h"
#include "motor.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <initializer_list>

static bool enabled;
static void enable(bool value) { enabled=value; }
static void near(float actual, float expected) { assert(std::fabs(actual-expected)<0.01f); }
static void test_displacement()
{
    GPIO_TypeDef gpio{};
    TIM_TypeDef timer{}, pwm{};
    timer.ARR=65535; pwm.ARR=3599;
    Encoder encoder{}; encoder.timer=&timer;
    MotorConfig c{};
    c.direction1={&gpio,1}; c.direction2={&gpio,2};
    c.pwm={&pwm,3}; c.encoder=&encoder;
    c.counts_per_output_rev=1456; c.radius_m=0.0325f; c.duty_limit=0.4f;
    c.forward_in1=c.reverse_in2=true;
    c.idle_in1=c.idle_in2=true;
    for (float radius : {0.0f, -0.01f, std::numeric_limits<float>::infinity(),
                         std::numeric_limits<float>::quiet_NaN()}) {
        MotorConfig invalid=c; invalid.radius_m=radius;
        Motor motor(invalid); assert(!motor.begin());
    }
    Motor motor(c);
    assert(motor.begin());
    const float circumference=6.28318530718f*0.0325f;
    timer.CNT=1456;
    assert(motor.updateFeedback(0.01f));
    assert(motor.state().total_counts==1456);
    assert(std::fabs(motor.state().displacement_m-circumference)<1e-6f);
    // 同样计数与 dt 无关；前进一圈再退一圈，净行程为零。
    timer.CNT=0;
    assert(motor.updateFeedback(0.02f));
    assert(motor.state().total_counts==0 && motor.state().displacement_m==0);
    // 停机/重建基准时读取尚未采集的计数，并保留累计。
    timer.CNT=100;
    assert(motor.stop() && motor.rebaseFeedback());
    assert(motor.state().total_counts==100 && motor.state().displacement_valid);
    assert(motor.rebaseFeedback() && motor.state().total_counts==100);
    timer.CNT=90;
    assert(motor.updateFeedback(0.01f) && motor.state().total_counts==90);
    // 恰好半量程无法确定方向，保留数值并永久标记本段行程不连续。
    timer.CNT=90+32768;
    assert(!motor.updateFeedback(0.01f));
    assert(!motor.state().displacement_valid && motor.state().total_counts==90);
    assert(motor.rebaseFeedback());
    timer.CNT+=10;
    assert(motor.updateFeedback(0.01f));
    assert(motor.state().total_counts==100 && !motor.state().displacement_valid);
    // 新 begin 建立新原点；反向配置与计数器回绕。
    c.encoder_reversed=true; timer.CNT=5;
    Motor reversed(c); assert(reversed.begin());
    timer.CNT=65526;
    assert(reversed.updateFeedback(0.01f));
    assert(reversed.state().total_counts==15);
    assert(reversed.state().displacement_m>0);
}
int main()
{
    test_displacement();
    GPIO_TypeDef gpio{};
    TIM_TypeDef pwm{}, left{}, right{};
    pwm.ARR=3599;
    left.ARR=right.ARR=65535;
    left.CNT=65530;
    right.CNT=5;
    Encoder left_encoder{}, right_encoder{};
    left_encoder.timer=&left; right_encoder.timer=&right;
    MotorAppConfig config{};
    config.left.direction1={&gpio,1};
    config.left.direction2={&gpio,2};
    config.right.direction1={&gpio,4};
    config.right.direction2={&gpio,8};
    config.left.pwm={&pwm,4};
    config.right.pwm={&pwm,3};
    config.left.encoder=&left_encoder;
    config.right.encoder=&right_encoder;
    config.left.counts_per_output_rev=config.right.counts_per_output_rev=1456;
    config.left.radius_m=config.right.radius_m=0.0325f;
    config.right.encoder_reversed=true;
    config.left.duty_limit=0.4f;
    config.right.duty_limit=0.3f;
    config.left.forward_in1=config.left.reverse_in2=true;
    config.right.forward_in1=config.right.reverse_in2=true;
    config.left.idle_in1=config.left.idle_in2=true;
    config.right.idle_in1=config.right.idle_in2=true;
    config.set_driver_enabled=enable;
    uint32_t now=UINT32_MAX-5000U;
    assert(!MotorApp_Init(nullptr,now));
    assert(!MotorApp_SetDuty(0.1f,0.1f,now));
    MotorAppConfig shared=config;
    shared.right.encoder=shared.left.encoder;
    assert(!MotorApp_Init(&shared,now));
    Encoder alias{}; alias.timer=&left;
    shared.right.encoder=&alias;
    assert(!MotorApp_Init(&shared,now));
    assert(MotorApp_Init(&config,now));
    assert(!enabled && MotorApp_GetStatus()->mode==MOTOR_STOPPED);
    assert(!MotorApp_Init(&config,now));
    // 停止状态仍测速：编码器双向回绕，微秒时间同时回绕。
    left.CNT=9; right.CNT=65526;
    now+=10000;
    MotorApp_Update(now);
    near(MotorApp_GetStatus()->left.rpm,15.0f/1456*6000);
    near(MotorApp_GetStatus()->right.rpm,15.0f/1456*6000);
    assert(!enabled && pwm.CCR3==0 && pwm.CCR4==0);
    assert(MotorApp_GetStatus()->left.total_counts==15);
    assert(MotorApp_GetStatus()->right.total_counts==15);
    assert(MotorApp_GetStatus()->timestamp_us==now);
    assert(!MotorApp_SetDuty(0.41f,0,now));
    assert(!MotorApp_SetDuty(0,0.31f,now));
    assert(!MotorApp_SetDuty(std::numeric_limits<float>::quiet_NaN(),0,now));
    assert(MotorApp_SetDuty(0.2f,-0.3f,now));
    assert(!enabled); // 命令不立即输出。
    now+=12500; left.CNT+=10; right.CNT+=10;
    MotorApp_Update(now);
    assert(enabled && pwm.CCR4==720 && pwm.CCR3==1080);
    assert((gpio.ODR & 15)==9); // 左正向、右反向。
    near(MotorApp_GetStatus()->left.rpm,10.0f/1456*60/0.0125f);
    near(MotorApp_GetStatus()->right.rpm,-10.0f/1456*60/0.0125f);
    assert(MotorApp_SetDuty(-0.1f,0.1f,now));
    now+=10000; MotorApp_Update(now);
    assert((gpio.ODR & 15)==6 && pwm.CCR4==360 && pwm.CCR3==360);
    // 续期允许运行超过一秒，停止续期后必须故障关闭两轮。
    for (int i=0;i<150;++i) {
        now+=10000;
        if (i%20==0) MotorApp_KeepAlive(now);
        MotorApp_Update(now);
        assert(enabled);
    }
    for (int i=0;i<100;++i) { now+=10000; MotorApp_Update(now); }
    assert(!enabled && pwm.CCR3==0 && pwm.CCR4==0);
    assert(MotorApp_GetStatus()->mode==MOTOR_FAULT);
    assert(!MotorApp_SetDuty(0.1f,0.1f,now));
    MotorApp_Stop(now);
    assert(MotorApp_GetStatus()->mode==MOTOR_STOPPED);
    assert(MotorApp_SetDuty(0.1f,0.1f,now));
    now+=50001; MotorApp_Update(now);
    assert(!enabled && MotorApp_GetStatus()->mode==MOTOR_FAULT);
    // 一个编码器失效，两轮一起停机；恢复必须重新建立基准。
    MotorApp_Stop(now);
    assert(MotorApp_SetDuty(0.2f,0.2f,now));
    now+=10000; MotorApp_Update(now);
    assert(enabled);
    right.ARR=0;
    now+=10000; MotorApp_Update(now);
    assert(!enabled && pwm.CCR3==0 && pwm.CCR4==0);
    assert(MotorApp_GetStatus()->mode==MOTOR_FAULT);
    right.ARR=65535;
    MotorApp_Stop(now);
    assert(MotorApp_GetStatus()->mode==MOTOR_STOPPED);
    assert(MotorApp_GetStatus()->left.applied_duty==0 && MotorApp_GetStatus()->right.applied_duty==0);
    now+=10000;
    MotorApp_Update(now);
    assert(!enabled && pwm.CCR3==0 && pwm.CCR4==0);
    const int64_t before_stop=MotorApp_GetStatus()->left.total_counts;
    left.CNT=(left.CNT+7U)&65535U;
    MotorApp_Stop(now);
    assert(MotorApp_GetStatus()->left.total_counts==before_stop+7);
    assert(MotorApp_SetDuty(0.1f,0.1f,now));
    assert(MotorApp_GetStatus()->left.total_counts==before_stop+7);
    MotorApp_Stop(now);
    std::puts("PASS: count-based displacement, radius validation, stop/restart continuity, gap flag;");
    std::puts("PASS: signed duty, RPM, encoder/time wrap, keepalive, gap timeout, dual-wheel fault/recovery");
}
