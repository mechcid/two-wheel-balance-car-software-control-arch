#ifndef MOTOR_HPP
#define MOTOR_HPP
#include "encoders.h"
#include "pin_type.h"
#include <cmath>

/* 两方向输入加独立 PWM 的驱动板配置。
 * 电平组合必须查实际驱动芯片真值表，不能由此类推断滑行/刹车。
 * 构造只保存配置；begin 在外设配置完成后调用，不启动 PWM/编码器。 */
struct MotorConfig {
    GPIO_pin_t direction1;
    GPIO_pin_t direction2;
    PWM_pin_t pwm;
    Encoder *encoder;
    //描述编码器计数与电机输出轴转速的换算关系，单位为每输出轴转一圈的编码器计数。
    float counts_per_output_rev;
    //定义前进，后退，停止的真值表
    bool forward_in1, forward_in2;
    bool reverse_in1, reverse_in2;
    bool idle_in1, idle_in2;

    //速度方向和编码器方向反转选项
    bool output_reversed = false;
    bool encoder_reversed = false;
    float duty_limit = 1.0f;
};
struct MotorState {
    float rpm = 0;
    float applied_duty = 0; // 已写入的逻辑占空比，不代表负载上的实测电压。
    bool feedback_valid = false;
};

class Motor {
public:
    //构造函数，explicit表示禁止隐式类型转换，防止意外的构造函数调用。
    explicit Motor(const MotorConfig &config) : 
    config_(config) {}

    //禁止两种违法的构造函数，防止复制对象后重复操纵同一硬件。暂时看不懂，可以日后再研究。
    Motor(const Motor &) = delete;
    Motor &operator=(const Motor &) = delete; // 防止复制对象后重复操纵同一硬件。

    bool begin(){
        initialized_ = false;
        state_.feedback_valid = false;
        if (!validConfig()) return false;
        if (!stop()) return false;
        state_.feedback_valid = false;
        initialized_ = Encoder_Init(config_.encoder);
        return initialized_;
    }
    const MotorState &state() const { 
        return state_; }

    float dutyLimit() const { 
        return config_.duty_limit; }

    /* dt_s 是距上次成功采样/建立基准的实际秒数，不在内部决定执行周期。
     * 失败保留上次 rpm，但标记不可用；恢复前可调用 rebaseFeedback。
     * 此实例应独占编码器更新权，外部不能再重复调用 Encoder_Update。 */
    bool updateFeedback(float dt_s){
        state_.feedback_valid = false;
        if (!initialized_ || !std::isfinite(dt_s) || dt_s <= 0) return false;
        // 先验证换算因子，避免非法 dt 消耗编码器基准。
        float factor = 60.0f / config_.counts_per_output_rev / dt_s;
        if (!std::isfinite(factor) || !std::isfinite(factor * 32767.0f)) return false;
        if (!Encoder_Update(config_.encoder)) return false;
        int32_t delta = config_.encoder->delta_cnt;
        if (config_.encoder_reversed) delta = -delta;
        state_.rpm = delta * factor;
        state_.feedback_valid = true;
        return true;
    }
    bool rebaseFeedback()
    {
        state_.feedback_valid = false;
        return initialized_ && Encoder_Init(config_.encoder);
    }

    /* 输入范围 -1～1，并受 duty_limit 限制；方向极性只在此统一处理。
     * 先写零比较值再改变方向。若启用了 CCR 预装载，零值未必立即生效；
     * 本草案不保证换向死区，接实物前须按驱动手册补齐换向时序。
     * 不能把 stop 或本函数当作独立于硬件配置的急停措施。 */
    bool setDuty(float duty)
    {
        if (!initialized_) return false;
        if (!std::isfinite(duty)) { stop(); return false; }
        if (duty > config_.duty_limit) duty = config_.duty_limit;
        if (duty < -config_.duty_limit) duty = -config_.duty_limit;
        if (duty == 0) return stop();
        float physical = config_.output_reversed ? -duty : duty;
        // 同方向调速直接更新比较值，避免每次调用都先清零造成波形扰动。
        if (state_.applied_duty == 0 || (duty > 0) != (state_.applied_duty > 0)) {
            if (!PWM_SetDuty(config_.pwm, 0)) return false;
            bool positive = physical > 0;
            GPIO_Write(config_.direction1, positive ? config_.forward_in1 : config_.reverse_in1);
            GPIO_Write(config_.direction2, positive ? config_.forward_in2 : config_.reverse_in2);
        }
        if (!PWM_SetDuty(config_.pwm, std::fabs(duty))) return false;
        state_.applied_duty = duty;
        return true;
    }
    bool stop(){
        if (!GPIO_PinValid(config_.direction1) || 
            !GPIO_PinValid(config_.direction2) ||
            !PWM_SetDuty(config_.pwm, 0)
        ) return false;

        GPIO_Write(config_.direction1, config_.idle_in1);
        GPIO_Write(config_.direction2, config_.idle_in2);
        state_.applied_duty = 0;
        return true;
    }
private:
    bool validConfig() const{
        return GPIO_PinValid(config_.direction1) && GPIO_PinValid(config_.direction2) &&
            !(config_.direction1.port == config_.direction2.port && config_.direction1.pin == config_.direction2.pin) &&
            PWM_PinValid(config_.pwm) && config_.encoder &&
            std::isfinite(config_.counts_per_output_rev) && config_.counts_per_output_rev > 0 &&
            std::isfinite(config_.duty_limit) && config_.duty_limit > 0 && config_.duty_limit <= 1 &&
            (config_.forward_in1 != config_.reverse_in1 || config_.forward_in2 != config_.reverse_in2);
    }
    MotorConfig config_;
    MotorState state_{};
    bool initialized_ = false;
};
#endif
