#ifndef MOTOR_SPEED_CONTROLLER_HPP
#define MOTOR_SPEED_CONTROLLER_HPP
#include "motor.hpp"
#include "pid.hpp"

/* 可选的转速控制层。Motor 自身不包含 PID，也不要求使用速度闭环。
 * Motor 必须比本对象活得更久；同一电机不能被多个控制器同时写输出。
 * update 内部已经更新反馈，调用者不要在同一周期再更新一次 Motor。
 * 启停、目标、参数与 update 必须在同一执行上下文中操作。 */
class MotorSpeedController {
public:
    explicit MotorSpeedController(Motor &motor) : motor_(motor) {}
    MotorSpeedController(const MotorSpeedController &) = delete;
    MotorSpeedController &operator=(const MotorSpeedController &) = delete;

    bool configure(const PID::Config &config)
    {
        // 让控制器的输出范围与执行器能力一致，避免未知的下游饱和。
        if (config.output_min < -motor_.dutyLimit() || config.output_max > motor_.dutyLimit())
            return false;
        return pid_.configure(config);
    }
    bool setTargetRPM(float rpm)
    {
        if (!std::isfinite(rpm)) return false;
        target_rpm_ = rpm; // 不在设置目标时暗中滤波。
        return true;
    }
    float targetRPM() const { return target_rpm_; }
    bool enabled() const { return enabled_; }

    bool enable()
    {
        if (enabled_) return true;
        if (!pid_.configured() || !motor_.stop() || !motor_.rebaseFeedback()) return false;
        pid_.reset();
        enabled_ = true;
        return true; // 下一次 update 的 dt 从此处重新计时。
    }
    bool disable()
    {
        enabled_ = false;
        pid_.reset();
        return motor_.stop();
    }
    bool update(float dt_s)
    {
        if (!enabled_) return false;
        float output;
        if (!motor_.updateFeedback(dt_s) ||
            !pid_.update(target_rpm_, motor_.state().rpm, dt_s, output) ||
            !motor_.setDuty(output)) {
            disable(); // 失败后要求调用者检查并显式重新使能，不静默恢复。
            return false;
        }
        return true;
    }
private:
    Motor &motor_;
    PID pid_;
    float target_rpm_ = 0;
    bool enabled_ = false;
};
#endif
