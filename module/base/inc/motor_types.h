#ifndef MOTOR_TYPES_H
#define MOTOR_TYPES_H
#include "encoders.h"
#include "pin_type.h"

/* C/C++ 共用数据类型；使用零初始化后显式填写配置。 */
/* 两方向输入加独立 PWM 的驱动板配置。
 * 电平组合必须查实际驱动芯片真值表，不能由此类推断滑行/刹车。
 * 构造只保存配置；begin 在外设配置完成后调用，不启动 PWM/编码器。 */
typedef struct {
    GPIO_pin_t direction1;
    GPIO_pin_t direction2;
    PWM_pin_t pwm;
    /* 编码器由调用方持有，生命周期须覆盖 Motor；只能由对应 Motor 更新。 */
    Encoder *encoder;
    //描述编码器计数与电机输出轴转速的换算关系，单位为每输出轴转一圈的编码器计数。
    float counts_per_output_rev;
    float radius_m; /* 轮半径，米，必须为有限正数。 */
    //定义前进，后退，停止的真值表
    bool forward_in1, forward_in2;
    bool reverse_in1, reverse_in2;
    bool idle_in1, idle_in2;

    //速度方向和编码器方向反转选项
    bool output_reversed;
    bool encoder_reversed;
    float duty_limit; /* 必须显式设置，范围 (0, 1]。 */
} MotorConfig;

//记录电机的状态参数，转速，施加占空比，编码器反馈是否有效
typedef struct {
    float rpm;
    float velocity_mps; /* 本次采样的有符号轮线速度，m/s，与 rpm 共用 feedback_valid。 */
    float delta_displacement_m; /* 最近一次采样区间的有符号轮位移，单位米。 */
    bool delta_valid; /* 单段位移有效性独立于累计行程连续性。 */
    int64_t total_counts; /* 从最近一次成功 begin 起的有符号累计计数。 */
    float signed_travel_m; /* total_counts / 每圈计数 × 2πradius_m；
                            有符号累计轮行程，前后抵消，就是有正负的总录成。 */
    bool travel_valid; /* 累计是否连续；丢失基准后不因恢复测速自动变真。 */
    float applied_duty; // 已写入的逻辑占空比，不代表负载上的实测电压。
    bool feedback_valid;// 编码器反馈是否有效，若无效则 rpm 和 velocity_mps 均不可用。
} MotorState;

#endif
