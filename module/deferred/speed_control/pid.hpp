#ifndef PID_HPP
#define PID_HPP
/* 基于 Haoqi Liu 原 PID 库进行的架构整理。
 * 支持位置式和增量式算法，显式传入秒单位 dt；不依赖硬件或电机。
 * 旧接口和旧参数不能直接照搬，旧 ki/kd 已隐含原来的调用周期。 */
#include <cmath>

class PID {
public:
    enum class Mode { Position, Incremental };

    struct Config {
        float kp = 0, ki = 0, kd = 0;
        float output_min = -1, output_max = 1;
        float integral_min = -1, integral_max = 1; // 两种模式都限制积分项的输出贡献。
        Mode mode = Mode::Position; // 默认保持已有位置式行为。
    };

    PID() = default; // 必须先 configure，才能计算。
    explicit PID(const Config &config) { configure(config); }

    bool configure(const Config &config)
    {
        if ((config.mode != Mode::Position && config.mode != Mode::Incremental) ||
            !std::isfinite(config.kp) || !std::isfinite(config.ki) || !std::isfinite(config.kd) ||
            !std::isfinite(config.output_min) || !std::isfinite(config.output_max) ||
            !std::isfinite(config.integral_min) || !std::isfinite(config.integral_max) ||
            config.output_min >= config.output_max || config.integral_min > 0 ||
            config.integral_max < 0 || config.integral_min > config.integral_max)
            return false;
        config_ = config;
        configured_ = true;
        reset(); // 改参后明确清空历史，避免旧积分混入新参数。
        return true;
    }
    const Config &config() const { return config_; }
    bool configured() const { return configured_; }
    void reset()
    {
        integral_ = previous_measurement_ = output_ = 0;
        previous_p_ = previous_d_ = 0;
        has_previous_ = false;
    }
    float output() const { return output_; }

    /* 两种模式都返回最终限幅后的输出，增量式不向外返回需要再次累加的增量。
     * 无效输入返回 false，result 置零，不推进控制器状态。
     * 微分作用于测量值，减少目标突变造成的微分冲击；首帧微分为零。
     * 微分项没有额外滤波，编码器速度环建议先使用 kd=0 的 PI。 */
    bool update(float target, float measurement, float dt_s, float &result)
    {
        result = 0;
        if (!configured_ || !std::isfinite(target) || !std::isfinite(measurement) ||
            !std::isfinite(dt_s) || dt_s <= 0) return false;
        float error = target - measurement;
        float p = config_.kp * error;
        float d = has_previous_ ? -config_.kd * (measurement - previous_measurement_) / dt_s : 0;
        float increment = config_.ki * error * dt_s;
        float sum = integral_ + increment;
        if (!std::isfinite(p) || !std::isfinite(d) || !std::isfinite(sum)) return false;
        float candidate = clamp(sum, config_.integral_min, config_.integral_max);
        // 增量式累计在上次限幅后的输出上；微分增量为相邻两次测量斜率项之差，
        // 每个斜率项使用当次真实 dt，不假定相邻周期完全相等。
        float base = config_.mode == Mode::Position ? p + d :
            output_ + (p - previous_p_) + (d - previous_d_);
        if (!std::isfinite(base)) return false;
        float unlimited = base + (config_.mode == Mode::Position ? candidate : candidate - integral_);
        if (!std::isfinite(unlimited)) return false;
        // 已饱和且积分还向饱和方向推动时，停止本次积分；反向积分仍允许退出饱和。
        if ((unlimited > config_.output_max && increment > 0) ||
            (unlimited < config_.output_min && increment < 0)) candidate = integral_;
        float combined = base + (config_.mode == Mode::Position ? candidate : candidate - integral_);
        if (!std::isfinite(combined)) return false;
        integral_ = candidate;
        output_ = clamp(combined, config_.output_min, config_.output_max);
        previous_measurement_ = measurement;
        previous_p_ = p;
        previous_d_ = d;
        has_previous_ = true;
        result = output_;
        return true;
    }
private:
    static float clamp(float value, float low, float high)
    { return value < low ? low : (value > high ? high : value); }
    Config config_{};
    float integral_ = 0, previous_measurement_ = 0, output_ = 0;
    float previous_p_ = 0, previous_d_ = 0;
    bool configured_ = false, has_previous_ = false;
};
#endif
