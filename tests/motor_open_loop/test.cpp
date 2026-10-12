#include "motor_app.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

static bool enabled;
static void enable(bool value) { enabled=value; }
static void near(float actual, float expected) { assert(std::fabs(actual-expected)<0.01f); }
int main()
{
    GPIO_TypeDef gpio{};
    TIM_TypeDef pwm{}, left{}, right{};
    pwm.ARR=3599;
    left.ARR=right.ARR=65535;
    left.CNT=65530;
    right.CNT=5;
    MotorAppConfig config{};
    config.left.direction1={&gpio,1};
    config.left.direction2={&gpio,2};
    config.right.direction1={&gpio,4};
    config.right.direction2={&gpio,8};
    config.left.pwm={&pwm,4};
    config.right.pwm={&pwm,3};
    config.left.encoder_timer=&left;
    config.right.encoder_timer=&right;
    config.left.counts_per_rev=config.right.counts_per_rev=1456;
    config.right.encoder_reversed=true;
    config.duty_limit=0.4f;
    config.forward_in1=config.reverse_in2=true;
    config.idle_in1=config.idle_in2=true;
    config.set_driver_enabled=enable;
    uint32_t now=UINT32_MAX-5000U;
    assert(!MotorApp_Init(nullptr,now));
    assert(!MotorApp_SetDuty(0.1f,0.1f,now));
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
    assert(MotorApp_GetStatus()->timestamp_us==now);
    assert(!MotorApp_SetDuty(0.41f,0,now));
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
    assert(MotorApp_GetStatus()->left.duty==0 && MotorApp_GetStatus()->right.duty==0);
    now+=10000;
    MotorApp_Update(now);
    assert(!enabled && pwm.CCR3==0 && pwm.CCR4==0);
    std::puts("PASS: signed duty, RPM, encoder/time wrap, keepalive, gap timeout, dual-wheel fault/recovery");
}
