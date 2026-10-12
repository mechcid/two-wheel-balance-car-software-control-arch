#include "motor_app.h"
#include "motor.hpp"
#include <new>

namespace {
Encoder left_encoder{};
Encoder right_encoder{};
alignas(Motor) unsigned char left_motor_storage[sizeof(Motor)];
alignas(Motor) unsigned char right_motor_storage[sizeof(Motor)];
Motor *left_motor{};
Motor *right_motor{};
struct MotorCommand {
    float left_duty;
    float right_duty;
};
MotorCommand command{};
MotorAppConfig config{};
MotorAppStatus status{MOTOR_UNCONFIGURED, {}, {}, 0, 0};
uint32_t last_update, last_command;
bool initialized;
constexpr uint32_t command_timeout_us = 1000000;
constexpr uint32_t max_control_gap_us = 50000;

//用于构建两个电机不同的配置
MotorConfig make_motor_config(const MotorWheelConfig &wheel, Encoder &encoder){
    encoder.timer = wheel.encoder_timer;
    MotorConfig c{};
    c.direction1=wheel.direction1; c.direction2=wheel.direction2; c.pwm=wheel.pwm;
    c.encoder=&encoder; c.counts_per_output_rev=wheel.counts_per_rev;
    c.output_reversed=wheel.output_reversed; c.encoder_reversed=wheel.encoder_reversed;
    c.duty_limit=config.duty_limit;
    c.forward_in1=config.forward_in1; c.forward_in2=config.forward_in2;
    c.reverse_in1=config.reverse_in1; c.reverse_in2=config.reverse_in2;
    c.idle_in1=config.idle_in1; c.idle_in2=config.idle_in2;
    return c;
}
bool stop_motor(Motor &motor)
{
    bool stopped = motor.stop();
    bool rebased = motor.rebaseFeedback();
    return stopped && rebased;
}
bool update_motor(Motor &motor, float target_duty, float dt)
{
    if (!motor.updateFeedback(dt)) return false;
    return status.mode!=MOTOR_DUTY || motor.setDuty(target_duty);
}

MotorWheelStatus read_status(const Motor &motor)
{
    const auto &s = motor.state();
    return {s.rpm, s.applied_duty, s.feedback_valid};
}

//用于更新记录两个电机的状态，并且给更新的数据打上时间戳记录，输出到串口
//本身并不参与电机逻辑上的组织
void snapshot(uint32_t now)
{
    status.left = read_status(*left_motor);
    status.right = read_status(*right_motor);
    status.timestamp_us = now;
}
void fault(uint32_t now)
{
    MotorApp_Stop(now);
    status.mode = MOTOR_FAULT;
    status.faults++;
}
bool request_valid(float left, float right, float limit)
{
    return initialized && status.mode != MOTOR_FAULT &&
        std::isfinite(left) && std::isfinite(right) &&
        std::fabs(left) <= limit && std::fabs(right) <= limit;
}
}

bool MotorApp_Init(const MotorAppConfig *settings, uint32_t now)
{
    if (initialized || left_motor) return false; // 仅启动时初始化一次，避免重复构造硬件所有者。
    if (!settings || !std::isfinite(settings->duty_limit) || settings->duty_limit <= 0 ||
        settings->duty_limit > 1)
        return false;
    config = *settings;
    if (!config.set_driver_enabled) return false;
    config.set_driver_enabled(false);
    left_motor = new (left_motor_storage) Motor(
        make_motor_config(config.left, left_encoder));
    if (!left_motor->begin()) {
        left_motor->stop();
        status.mode=MOTOR_FAULT;
        return false;
    }
    right_motor = new (right_motor_storage) Motor(
        make_motor_config(config.right, right_encoder));
    if (!right_motor->begin()) {
        left_motor->stop();
        right_motor->stop();
        status.mode=MOTOR_FAULT;
        return false;
    }
    initialized=true;
    last_update=last_command=now;
    status.mode=MOTOR_STOPPED;
    snapshot(now);
    return true;
}
void MotorApp_Stop(uint32_t now)
{
    if (!initialized) return;
    config.set_driver_enabled(false);
    /* 分别调用，左轮失败也必须执行右轮停机。 */
    bool left_ok = stop_motor(*left_motor);
    bool right_ok = stop_motor(*right_motor);
    command.left_duty=0;
    command.right_duty=0;
    status.mode=(left_ok && right_ok) ? MOTOR_STOPPED : MOTOR_FAULT;
    last_update=now;
    snapshot(now);
}
bool MotorApp_IsActive(void){
    return status.mode==MOTOR_DUTY; }

void MotorApp_KeepAlive(uint32_t now){
    if (MotorApp_IsActive()) last_command=now; }

const MotorAppStatus *MotorApp_GetStatus(void) { return &status; }

//暴露出来用于控制两个电机占空比的函数
bool MotorApp_SetDuty(float left, float right, uint32_t now){
    if (!request_valid(left,right,config.duty_limit)) return false;
    if (status.mode!=MOTOR_DUTY) MotorApp_Stop(now);
    if (status.mode==MOTOR_FAULT) return false;
    command.left_duty=left; command.right_duty=right;
    status.mode=MOTOR_DUTY;
    last_command=now;
    // 输出在下一个调度周期统一写入，命令回调不直接启动电机。
    return true;
}

//更新函数，传入一个时间基准就能自动更新计算出速度
//速度计算是要时刻更新的，但是占空比设置相对于速度计算独立，写个语句就能设置
void MotorApp_Update(uint32_t now){
    if (!initialized) return;
    uint32_t elapsed=now-last_update;
    if (!elapsed) return;
    bool active=MotorApp_IsActive();
    if (active && (elapsed>max_control_gap_us || (uint32_t)(now-last_command)>=command_timeout_us)) {
        fault(now); return;
    }
    last_update=now;
    if (status.mode==MOTOR_FAULT) return;
    float dt=elapsed*0.000001f;
    if (!update_motor(*left_motor, command.left_duty, dt)) {
        fault(now);
        return;
    }
    if (!update_motor(*right_motor, command.right_duty, dt)) {
        fault(now);
        return;
    }
    if (MotorApp_IsActive()) config.set_driver_enabled(true);
    snapshot(now);
}
