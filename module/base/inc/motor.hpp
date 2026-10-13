#ifndef MOTOR_HPP
#define MOTOR_HPP
#include "motor_types.h"
#include <cmath>

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
        state_.displacement_valid = false;
        state_.feedback_valid = false;
        if (!validConfig()) return false;
        if (!stop()) return false;
        state_.feedback_valid = false;
        initialized_ = Encoder_Init(config_.encoder);
        if (initialized_) {
            state_.total_counts = 0;
            state_.displacement_m = 0;
            state_.displacement_valid = true;
        }
        return initialized_;
    }
    const MotorState &state() const { 
        return state_; }

    float dutyLimit() const { 
        return config_.duty_limit; }

    /* dt_s 是距上次成功采样/建立基准的实际秒数，不在内部决定执行周期。
     * 失败保留上次 rpm，但标记不可用；恢复前可调用 rebaseFeedback。
     * 此实例应独占编码器更新权，外部不能再重复调用 Encoder_Update。 */
    //更新电机转速的函数
    bool updateFeedback(float dt_s){
        state_.feedback_valid = false;
        if (!initialized_ || !std::isfinite(dt_s) || dt_s <= 0) return false;
        // 先验证换算因子，避免非法 dt 消耗编码器基准。
        float factor = 60.0f / config_.counts_per_output_rev / dt_s;
        if (!std::isfinite(factor) || !std::isfinite(factor * 32767.0f)) return false;
        int32_t delta;
        if (!capture_position(delta)) return false;
        state_.rpm = delta * factor;
        state_.feedback_valid = true;
        return true;
    }
    /* 重建测速基准前补计尚未读取的位移；正常启停不清累计值。
     * 失效基准无法恢复缺失计数，displacement_valid 保持 false。 */
    bool rebaseFeedback(){
        state_.feedback_valid = false;
        if (!initialized_) return false;
        int32_t delta;
        capture_position(delta);
        bool ready = Encoder_Init(config_.encoder);
        if (!ready) state_.displacement_valid = false;
        return ready;
    }

    /* 输入范围 -1～1，并受 duty_limit 限制；方向极性只在此统一处理。
     * 先写零比较值再改变方向。若启用了 CCR 预装载，零值未必立即生效；
     * 本草案不保证换向死区，接实物前须按驱动手册补齐换向时序。
     * 不能把 stop 或本函数当作独立于硬件配置的急停措施。 */
    bool setDuty(float duty){
        //校验是否合法
        if (!initialized_) return false;
        if (!std::isfinite(duty)) { stop(); return false; }

        //限制占空比在 -duty_limit 到 duty_limit 之间，上限来自单轮配置
        if (duty > config_.duty_limit) duty = config_.duty_limit;
        if (duty < -config_.duty_limit) duty = -config_.duty_limit;
        if (duty == 0) return stop();

        //如果反转了朝向，反转一下占空比，否则不变
        float physical = config_.output_reversed ? -duty : duty;
        // 同方向调速直接更新比较值，避免每次调用都先清零造成波形扰动。
        if (state_.applied_duty == 0 || (duty > 0) != (state_.applied_duty > 0)) {
            if (!PWM_SetDuty(config_.pwm, 0)) return false;
            bool positive = physical > 0;
            GPIO_Write(config_.direction1, positive ? config_.forward_in1 : config_.reverse_in1);
            GPIO_Write(config_.direction2, positive ? config_.forward_in2 : config_.reverse_in2);
        }
        //施加占空比并更新记录状态变量中的占空比记录
        if (!PWM_SetDuty(config_.pwm, std::fabs(duty))) return false;
        state_.applied_duty = duty;
        return true;
    }

    //停止函数
    bool stop(){
        //检验是否合法
        if (!GPIO_PinValid(config_.direction1) || 
            !GPIO_PinValid(config_.direction2) ||
            !PWM_SetDuty(config_.pwm, 0)
        ) return false;

        //按照真值表写让电机转速归零的值
        GPIO_Write(config_.direction1, config_.idle_in1);
        GPIO_Write(config_.direction2, config_.idle_in2);
        state_.applied_duty = 0;
        return true;
    }
private:
    /* 位移直接累计整数计数，避免逐帧浮点加法或速度积分误差。 */
    //引用符号类似于传指针，但引用符号更安全，不能为null，且不能改变引用的对象。相当于直接对传入参数进行修改操作
    bool capture_position(int32_t &delta){
        if (!Encoder_Update(config_.encoder)) {
            state_.displacement_valid = false;
            return false;
        }
        delta = config_.encoder->delta_cnt;
        if (config_.encoder_reversed) delta = -delta;
        if ((delta>0 && state_.total_counts>INT64_MAX-delta) ||
            (delta<0 && state_.total_counts<INT64_MIN-delta)) {
            state_.displacement_valid = false;
            return false;
        }
        //累积积累总位移
        int64_t next = state_.total_counts + delta;
        float meters = (float)((double)next / config_.counts_per_output_rev *
                               6.283185307179586 * config_.radius_m);
        if (!std::isfinite(meters)) {
            state_.displacement_valid = false;
            return false;
        }
        state_.total_counts = next;
        state_.displacement_m = meters;
        return true;
    }
    //校验函数，检测传入的配置参数是否合法，不合法则返回false，合法则返回true
    bool validConfig() const{
        return GPIO_PinValid(config_.direction1) && GPIO_PinValid(config_.direction2) &&
            !(config_.direction1.port == config_.direction2.port && config_.direction1.pin == config_.direction2.pin) &&
            PWM_PinValid(config_.pwm) && config_.encoder &&
            std::isfinite(config_.counts_per_output_rev) && config_.counts_per_output_rev > 0 &&
            std::isfinite(config_.radius_m) && config_.radius_m > 0 &&
            std::isfinite(config_.duty_limit) && config_.duty_limit > 0 && config_.duty_limit <= 1 &&
            (config_.forward_in1 != config_.reverse_in1 || config_.forward_in2 != config_.reverse_in2);
    }
    MotorConfig config_;//承接外部传来的参数配置
    MotorState state_{};//内部的参数
    bool initialized_ = false;
};
#endif
